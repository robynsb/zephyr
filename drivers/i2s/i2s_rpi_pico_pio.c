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
#include <math.h>

#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_I2S_LOG_LEVEL
LOG_MODULE_REGISTER(i2s_pico_pio);

struct queue_item {
	void *mem_block;
	size_t size;
};

struct pio_i2s_config {
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	const uint32_t data_pin;
	const uint32_t clock_pin_base;
	void (*irq_config)(const struct device *dev);
};

struct stream {
	enum i2s_state state;
	struct k_msgq *msgq;
	uint32_t dma_channel;

	struct i2s_config cfg;
	void *mem_block;
};

struct pio_i2s_data {
	uint8_t tx_sm;
    uint32_t freq;
    struct stream tx;
};

static int i2s_rpi_pico_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
    const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
    if (dir != I2S_DIR_TX) {
		LOG_ERR("I2S direction is unsupported."); // TODO:
		return -EINVAL;
    }

    if (i2s_cfg->word_size != 16) {
		LOG_ERR("I2S direction is unsupported.");
		return -EINVAL;
    }

    // TODO: Handle the config better

	struct stream *stream = &data->tx;

	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	stream->state = I2S_STATE_READY;
	return 0;
}

static int i2s_rpi_pico_write(const struct device *dev, void *mem_block, size_t size)
{
    const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	const struct stream *stream = &data->tx;
	enum i2s_state state = stream->state;
	int err = 0;

	if (state != I2S_STATE_RUNNING && state != I2S_STATE_READY) {
		LOG_DBG("Invalid state: %d", (int)state);
		return -EIO;
	}

	if (size > stream->cfg.block_size) {
		LOG_DBG("Max write size is: %u", stream->cfg.block_size);
		return -EIO;
	}

	struct queue_item item = {.mem_block = mem_block, .size = size};

	err = k_msgq_put(stream->msgq, &item,
			 K_MSEC(stream->cfg.timeout));
	if (err < 0) {
		LOG_DBG("TX queue full");
	}

	// return err;
    return 0;
}

static int i2s_rpi_pico_trigger(const struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
    const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	int ret;

    if (dir != I2S_DIR_TX) {
		LOG_ERR("I2S direction is unsupported.");
		return -EINVAL;
    }

	struct stream *stream = &data->tx;

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				    stream->state);
			return -EIO;
		}
        audio_i2s_start(dev);
		stream->state = I2S_STATE_RUNNING;

		break;
	default:
        //TODO: Handle all other trigger commands
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}

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
#define PICO_AUDIO_I2S_DMA_IRQ 0 // TODO: This is hardcoded to use DMA_IRQ_0

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
    uint32_t sample_freq = 24000; // TODO: Frequency is hardcoded 
                                  // Make it work with config
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    uint32_t divider = system_clock_frequency * 4 / sample_freq; // avoid arithmetic overflow
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(pio, sm, divider >> 8u, divider & 0xffu);

	return 0;
}

static inline void audio_start_dma_transfer(const struct device *dev)
{
	// TODO: Currently assumes only one DMA channel/ I2S device exists.
    const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
    struct stream *stream = &data->tx;

	size_t mem_block_size;
	struct queue_item item;
	int ret = k_msgq_get(stream->msgq, &item, SYS_TIMEOUT_MS(0));
    if (ret < 0) {
		return; // TODO: Handle errors
	}

    stream->mem_block = item.mem_block; 
    mem_block_size = item.size;

    // if (!) {
    //     // just play some silence
    //     static uint32_t zero;
    //     dma_channel_config c = dma_get_channel_config(shared_state.dma_channel);
    //     channel_config_set_read_increment(&c, false);
    //     dma_channel_set_config(shared_state.dma_channel, &c, false);
    //     dma_channel_transfer_from_buffer_now(shared_state.dma_channel, &zero, SAMPLE_LENGTH);
    //     return;
    // }
    dma_channel_config c = dma_get_channel_config(data->tx.dma_channel);
    channel_config_set_read_increment(&c, true);
    dma_channel_set_config(data->tx.dma_channel, &c, false);
    dma_channel_transfer_from_buffer_now(data->tx.dma_channel, (void *) stream->mem_block, mem_block_size/4); // Hardcoded 32 bit words
}


void audio_i2s_dma_irq_handler(const struct device *dev) {
	// TODO: This is currently a zephyr interrupt waiting on the physical DMA finished interupt request line.
	// Since it is a zephyr interrupt I specified in my IRQ_CONNECT macro that the i2s device should be passed.
	// When I move my DMA handling over to zephyr DMA module, the IRQ works differently,
	// I will receieve the DMA device instead which in that interupt handler i then need to find access to the
	// i2s device.
    const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
    uint dma_channel = data->tx.dma_channel;
    if (dma_irqn_get_channel_status(PICO_AUDIO_I2S_DMA_IRQ, dma_channel)) {
        dma_irqn_acknowledge_channel(PICO_AUDIO_I2S_DMA_IRQ, dma_channel);

        struct stream *stream = &data->tx;
        k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
        stream->mem_block = NULL;

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
    uint8_t dma_channel = data->tx.dma_channel;
    dma_channel_claim(dma_channel);

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

    dma_irqn_set_channel_enabled(PICO_AUDIO_I2S_DMA_IRQ, dma_channel, 1);

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

static DEVICE_API(i2s, i2s_rpi_pico_driver_api) = {
	.configure = i2s_rpi_pico_configure,
	.config_get = NULL,
	.read = NULL,
	.write = i2s_rpi_pico_write,
	.trigger = i2s_rpi_pico_trigger,
};

// TODO: magic number 11!
// TODO: hardcoded dma_channel
// TODO: hardcoded queue size!
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
	};                                                  \
    K_MSGQ_DEFINE(tx_##idx##_queue, sizeof(struct queue_item),		\
            32, 4);			\
	static struct pio_i2s_data pio_i2s##idx##_data = {                \
        .tx = {                                                        \
            .msgq = &tx_##idx##_queue,                               \
            .state = I2S_STATE_NOT_READY,                                \
			.dma_channel = 7,                    \
        },                                             \
    };					\
	DEVICE_DT_INST_DEFINE(idx, pio_i2s_init, NULL, &pio_i2s##idx##_data,			\
			      &pio_i2s##idx##_config, POST_KERNEL,				\
			      CONFIG_I2S_INIT_PRIORITY,					\
			      &i2s_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_I2S_INIT)

