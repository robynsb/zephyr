/*
 * Copyright (c) 2023 Stephen Boylan <stephen.boylan@beechwoods.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT raspberrypi_pico_i2s_pio

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <zephyr/logging/log.h>
#include <hardware/clocks.h>

#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_I2S_LOG_LEVEL
LOG_MODULE_REGISTER(i2s_pico_pio);

typedef void (*pio_i2s_irq_config_func_t)(const struct device *dev);

struct pio_i2s_config {
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	const uint32_t data_pin;
	const uint32_t clock_pin_base;
	const uint8_t dma_channel;
    const pio_i2s_irq_config_func_t irq_config;
};

struct pio_i2s_data {
	uint8_t tx_sm;
    uint32_t freq;
    uint8_t dma_channel; // TODO: Look into why there are two dma_channel variables?
};


RPI_PICO_PIO_DEFINE_PROGRAM(i2s_tx, 0, 3,
            //     .wrap_target
    0x7001, //  0: out    pins, 1         side 2
    0x1840, //  1: jmp    x--, 0          side 3
    0x6001, //  2: out    pins, 1         side 0
    0xe82e, //  3: set    x, 14           side 1
    0x6001, //  4: out    pins, 1         side 0
    0x0844, //  5: jmp    x--, 4          side 1
    0x7001, //  6: out    pins, 1         side 2
    0xf82e, //  7: set    x, 14           side 3
            //     .wrap
);

#define audio_i2s_wrap_target 0
#define audio_i2s_wrap 7
#define audio_i2s_offset_entry_point 7u
#define PICO_AUDIO_I2S_DMA_IRQ 0 // We hardcode to use DMA_IRQ_0

static int pio_i2s_tx_init(PIO pio, uint32_t sm, uint32_t data_pin, uint32_t clock_pin_base)
{
	uint32_t offset;
	pio_sm_config sm_config;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(i2s_tx))) {
		return -EBUSY;
	}

	offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(i2s_tx));
	sm_config = pio_get_default_sm_config();
    sm_config_set_wrap(&sm_config, offset + audio_i2s_wrap_target, offset + audio_i2s_wrap);
    sm_config_set_sideset(&sm_config, 2, false, false);
    sm_config_set_out_pins(&sm_config, data_pin, 1);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, true, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, sm, offset, &sm_config);
    uint32_t pin_mask = (1u << data_pin) | (3u << clock_pin_base);
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_set_pins(pio, sm, 0); // clear pins
    pio_sm_exec(pio, sm, pio_encode_jmp(offset + audio_i2s_offset_entry_point));

    // update_pio_frequency
    uint32_t sample_freq = 24000; // TODO: some hardcoded thing idk
                                  // Make it work with config?
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    uint32_t divider = system_clock_frequency * 4 / sample_freq; // avoid arithmetic overflow
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(pio, sm, divider >> 8u, divider & 0xffu);

	return 0;
}

#define SAMPLE_LENGTH 128 // Hardcoded sample length
static int32_t sine_wave_table[SAMPLE_LENGTH] = {0};
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline void audio_start_dma_transfer(const struct device *dev)
{
    const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;


    // if (!) {
    //     // just play some silence
    //     static uint32_t zero;
    //     dma_channel_config c = dma_get_channel_config(shared_state.dma_channel);
    //     channel_config_set_read_increment(&c, false);
    //     dma_channel_set_config(shared_state.dma_channel, &c, false);
    //     dma_channel_transfer_from_buffer_now(shared_state.dma_channel, &zero, SAMPLE_LENGTH);
    //     return;
    // }
    dma_channel_config c = dma_get_channel_config(data->dma_channel);
    channel_config_set_read_increment(&c, true);
    dma_channel_set_config(data->dma_channel, &c, false);
    dma_channel_transfer_from_buffer_now(data->dma_channel, (void *) sine_wave_table, SAMPLE_LENGTH);
}


void audio_i2s_dma_irq_handler(const struct device *dev) {
    const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
    uint dma_channel = data->dma_channel;
    if (dma_irqn_get_channel_status(PICO_AUDIO_I2S_DMA_IRQ, dma_channel)) {
        dma_irqn_acknowledge_channel(PICO_AUDIO_I2S_DMA_IRQ, dma_channel);

        audio_start_dma_transfer(dev);
    }
}

static int pio_i2s_init(const struct device *dev)
{
	const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	size_t tx_sm;
	int retval;
	PIO pio;

	pio = pio_rpi_pico_get_pio(config->piodev);

	retval = pio_rpi_pico_allocate_sm(config->piodev, &tx_sm);
	if (retval < 0) {
		LOG_ERR("pio_rpi_pico_allocate_sm failed with ret = %d", retval);
		return retval;
	}

	data->tx_sm = tx_sm;

	retval = pio_i2s_tx_init(pio, tx_sm, config->data_pin, config->clock_pin_base);
	if (retval < 0) {
		LOG_ERR("pio_i2s_tx_init failed with ret = %d", retval);
		return retval;
	}


	retval = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		LOG_ERR("pinctrl_apply_state failed with ret = %d", retval);
        return retval;
	}

    // TODO: Use zephyr's DMA driver.
    uint8_t dma_channel = config->dma_channel;
    dma_channel_claim(dma_channel);

    data->dma_channel = dma_channel;

    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);

    channel_config_set_dreq(&dma_config,
                            DREQ_PIO1_TX0 + tx_sm // TODO: Hardcoded from device tree choosing PIO1
    );
    // channel_config_set_dreq(&dma_config, DREQ_FORCE);

    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32); // TODO: Hardcoded 16 bit audio, 2*16 for stereo = 4 bytes

    dma_channel_configure(dma_channel,
                          &dma_config,
                          &pio->txf[tx_sm],  // dest
                          NULL, // src
                          0, // count
                          false // trigger
    );

    // irq_add_shared_handler(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, audio_i2s_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    // retval = irq_connect_dynamic(DT_IRQN(DT_NODELABEL(dma)),  // TODO: Do proper interrupt
    //             0, 
    //             audio_i2s_dma_irq_handler, 
    //             NULL, 
    //             0);
    // if (retval < 0) {
	// 	LOG_ERR("irq_connect_dynamic failed with ret = %d", retval);
    //     return retval;
	// }

    dma_irqn_set_channel_enabled(PICO_AUDIO_I2S_DMA_IRQ, dma_channel, 1);

    float volume = 0.15;
    for (int i = 0; i < SAMPLE_LENGTH; i++) {
        int16_t val = 32767 * volume * cosf(i * 2 * (float) (M_PI / SAMPLE_LENGTH));
        // sine_wave_table[i] = 32767 * volume * cosf(i * 2 * (float) (M_PI / SAMPLE_LENGTH));
        sine_wave_table[i]= (val  & 0xFFFF) | (val << 16);
    }

    return !(retval >= 0);
}


void audio_i2s_start(const struct device *dev) {
	const struct pio_i2s_config *config = dev->config;
	const struct pio_i2s_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);

    // irq_set_enabled(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, true);
    // irq_enable(DT_IRQN(DT_NODELABEL(dma)));
    config->irq_config(dev);
    pio_sm_set_enabled(pio, data->tx_sm, true);
    audio_start_dma_transfer(dev);

}

int i2s_rpi_pico_dummy_write(const struct device *dev, void *mem_block, size_t size) {
    audio_i2s_start(dev);
}

static DEVICE_API(i2s, i2s_rpi_pico_driver_api) = {
	.configure = NULL,
	.config_get = NULL,
	.read = NULL,
	.write = i2s_rpi_pico_dummy_write,
	.trigger = NULL,
};

// TODO: magic number 11!
// TODO: hardcoded dma_channel
#define PIO_I2S_INIT(idx)									\
	PINCTRL_DT_INST_DEFINE(idx);								\
    static void pio_i2s_irq_config_##idx(const struct device *dev)				\
	{											\
		IRQ_CONNECT(11,				\
			    0, audio_i2s_dma_irq_handler,					\
			    DEVICE_DT_INST_GET(idx), 0);					\
		irq_enable(11);				\
	}                                                              \
	static const struct pio_i2s_config pio_i2s##idx##_config = {				\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(idx)),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),					\
		.data_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_pins, 0),	\
		.clock_pin_base = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_pins, 1),	\
        .irq_config = pio_i2s_irq_config_##idx,		\
        .dma_channel = 7                    \
	};											\
	static struct pio_i2s_data pio_i2s##idx##_data;					\
												\
	DEVICE_DT_INST_DEFINE(idx, pio_i2s_init, NULL, &pio_i2s##idx##_data,			\
			      &pio_i2s##idx##_config, POST_KERNEL,				\
			      CONFIG_I2S_INIT_PRIORITY,					\
			      &i2s_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_I2S_INIT)

