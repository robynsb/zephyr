/*
 * Copyright (c) 2026 Robin Sachsenweger Ballantyne <makenenjoy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
	TOOO: 
	 - Check that all errors are handled properly
	 - Respect all properties of config
	 - RX stream functionality
	 - Support all trigger commands
	 - Change LOG statements to be more inline with the rest of zephyr
	 - Think about syncronisation primitives
*/


#define DT_DRV_COMPAT raspberrypi_pico_i2s_pio

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <hardware/pio.h>
#include <zephyr/drivers/dma.h>
#include <hardware/dma.h> // TODO: Hopefully remove this include eventually?
#include <zephyr/logging/log.h>
#include <hardware/clocks.h>
#include <math.h>
#if defined(CONFIG_SOC_SERIES_RP2040)
#include <zephyr/dt-bindings/dma/rpi-pico-dma-rp2040.h>
#elif defined(CONFIG_SOC_SERIES_RP2350)
#include <zephyr/dt-bindings/dma/rpi-pico-dma-rp2350.h>
#endif

#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_I2S_LOG_LEVEL
LOG_MODULE_REGISTER(i2s_pico_pio);

static bool queue_is_empty(struct k_msgq *q)
{
	return (k_msgq_num_used_get(q) == 0) ? true : false;
}

struct queue_item {
	void *mem_block;
	size_t size;
};

struct pio_i2s_config {
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	const uint32_t data_pin;
	const uint32_t clock_pin_base;
};

struct stream {
	enum i2s_state state;
	struct k_msgq *msgq;
	uint32_t dma_channel;
	const struct device *dev_dma;
	struct dma_config dma_cfg;
	uint8_t sm;

	struct i2s_config cfg;
	void *mem_block;
};

struct pio_i2s_data {
    struct stream tx;
};

// TODO: Do some experiments to tripple check that this is correct.
void update_pio_frequency(PIO pio, uint32_t sm, uint32_t sample_freq) { 
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    uint32_t divider = system_clock_frequency * 4 / sample_freq; // avoid arithmetic overflow
    assert(divider < 0x1000000); // TODO: These errors should be handled better
    pio_sm_set_clkdiv_int_frac(pio, sm, divider >> 8u, divider & 0xffu);
}

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
		LOG_ERR("I2S word size is unsupported.");
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

	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	update_pio_frequency(pio, data->tx.sm, i2s_cfg->frame_clk_freq);

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
		LOG_ERR("TX queue full");
		return err;
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

	return 0;
}

static int reload_dma(const struct device *dev_dma, uint32_t channel,
		      struct dma_config *dcfg, void *src, void *dst,
		      uint32_t blk_size)
{
	int ret;

	ret = dma_reload(dev_dma, channel, (uint32_t)src, (uint32_t)dst, blk_size);
	if (ret < 0) {
		LOG_ERR("dma_reload failed with ret=%d", ret);
		return ret;
	}

	ret = dma_start(dev_dma, channel);
	if (ret < 0) {
		LOG_ERR("dma_start failed with ret=%d", ret);
		return ret;
	}

	return ret;
}

static int start_dma(const struct device *dev_dma, uint32_t channel,
		     struct dma_config *dcfg, void *src,
		     bool src_addr_increment, void *dst,
		     bool dst_addr_increment,
		     uint32_t blk_size)
{
	struct dma_block_config blk_cfg;
	int ret;

	memset(&blk_cfg, 0, sizeof(blk_cfg));
	blk_cfg.block_size = blk_size;
	blk_cfg.source_address = (uint32_t)src;
	blk_cfg.dest_address = (uint32_t)dst;
	if (src_addr_increment) {
		blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}
	if (dst_addr_increment) {
		blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}
	// blk_cfg.fifo_mode_control = fifo_threshold; // TODO: i guess this does nothing for pico?

	dcfg->head_block = &blk_cfg;

	ret = dma_config(dev_dma, channel, dcfg);
	if (ret < 0) {
		LOG_ERR("dma_config failed with error %d", ret);
		return ret;
	}

	ret = dma_start(dev_dma, channel);
	if (ret < 0) {
		LOG_ERR("dma_start failed with error %d", ret);
		return ret;
	}

	return ret;
}

void audio_i2s_dma_irq_handler(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status) {
	const struct device *dev = (const struct device *)arg;
    const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
    uint dma_channel = data->tx.dma_channel;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	// TODO: Use a spinlock here?

	int retval;

	struct stream *stream = &data->tx;

	if (status < 0) {
		LOG_ERR("Something went wrong with DMA. status=%d", status);
		stream->state = I2S_STATE_ERROR;
		return; // TODO: abort DMA? 
	}

	// TODO: Should we free only if no error or in all cases?
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block); 
	stream->mem_block = NULL;

	if(stream->state == I2S_STATE_STOPPING && queue_is_empty(stream->msgq)) {
		stream->state = I2S_STATE_READY;
		return;
	}

	struct queue_item item;
	size_t mem_block_size;
	int ret = k_msgq_get(stream->msgq, &item, SYS_TIMEOUT_MS(0));
	if (ret < 0) {
		LOG_ERR("Failed to get message from message queue");
		stream->state = I2S_STATE_ERROR;
		return; // TODO: abort DMA?
	}

	stream->mem_block = item.mem_block; 
    mem_block_size = item.size;

	retval = reload_dma(stream->dev_dma, stream->dma_channel,
		&stream->dma_cfg,
		stream->mem_block,
		(void *)&pio->txf[data->tx.sm],
		mem_block_size);

	if (retval < 0) {
		LOG_DBG("Failed to start TX DMA transfer: %d", retval);
		return;
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

	data->tx.sm = tx_sm;
	data->tx.dma_cfg.user_data = (void*) dev;
	data->tx.dma_cfg.dma_slot = RPI_PICO_DMA_DREQ_TO_SLOT(pio_get_dreq(pio, data->tx.sm, true));

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


    uint8_t dma_channel = data->tx.dma_channel;
	retval = dma_config(data->tx.dev_dma, dma_channel, &data->tx.dma_cfg);
	if (retval < 0) {
		LOG_ERR("dma ctrl %p: dma_config failed with %d", data->tx.dev_dma, retval);
		return retval;
	}
	return 0;
}


int audio_i2s_start(const struct device *dev) {
	const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);

    pio_sm_set_enabled(pio, data->tx.sm, true);

    struct stream *stream = &data->tx;

	size_t mem_block_size;
	struct queue_item item;
	int ret = k_msgq_get(stream->msgq, &item, SYS_TIMEOUT_MS(0));
    if (ret < 0) {
		LOG_ERR("Failed to get message from message queue");
		return ret;
	}

	stream->mem_block = item.mem_block; 
    mem_block_size = item.size;

	ret = start_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			stream->mem_block, true, /* TODO: scr addr increment setting? */
			(void *)&pio->txf[data->tx.sm],
			false,
			mem_block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}
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
        ret = audio_i2s_start(dev);
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}
		stream->state = I2S_STATE_RUNNING;
		break;
	case I2S_TRIGGER_DRAIN:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("DRAIN trigger: invalid state %d",
					stream->state);
			return -EIO;
		}
		stream->state = I2S_STATE_STOPPING;
		break;
	default:
        //TODO: Handle all other trigger commands
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}

// TODO: Test this function
static const struct i2s_config *i2s_rpi_pico_config_get(const struct device *dev,
						     enum i2s_dir dir)
{
	struct pio_i2s_data *const dev_data = dev->data;
	struct stream *stream = NULL;

	if (dir == I2S_DIR_RX) {
		stream = NULL;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx;
	}

	if (stream != NULL && stream->state != I2S_STATE_NOT_READY) {
		return &stream->cfg;
	}

	return NULL;
}

static DEVICE_API(i2s, i2s_rpi_pico_driver_api) = {
	.configure = i2s_rpi_pico_configure,
	.config_get = i2s_rpi_pico_config_get,
	.read = NULL,
	.write = i2s_rpi_pico_write,
	.trigger = i2s_rpi_pico_trigger,
};

// TODO: hardcoded queue size!
#define PIO_I2S_INIT(idx)									\
	PINCTRL_DT_INST_DEFINE(idx);								\
	static const struct pio_i2s_config pio_i2s##idx##_config = {				\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(idx)),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),					\
		.data_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_pins, 0),	\
		.clock_pin_base = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_pins, 1),	\
	};                                                  \
    K_MSGQ_DEFINE(tx_##idx##_queue, sizeof(struct queue_item),		\
            32, 4);			\
	static struct pio_i2s_data pio_i2s##idx##_data = {                \
        .tx = {                                                        \
            .msgq = &tx_##idx##_queue,                               \
            .state = I2S_STATE_NOT_READY,                                \
			.dev_dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, tx)),		\
			.dma_channel = DT_INST_DMAS_CELL_BY_NAME(idx, tx, channel),  \
			.dma_cfg = {							\
				.block_count = 1, /* block_count > 1 not supported */	\
				.channel_direction = MEMORY_TO_PERIPHERAL,		\
				.source_data_size = 4,  /* 32bit hard coded */		\
				.dest_data_size = 4,    /* TODO: 32bit hard coded */		\
				/* single transfers (burst length = data size) */	\
				.source_burst_length = 1, /* unused i think */			\
				.dest_burst_length = 1,	/* unused i think */			\
				.channel_priority = 1, /* TODO: hardcoded */		\
				.dma_callback = audio_i2s_dma_irq_handler			\
			},								\
        },                                             \
    };					\
	DEVICE_DT_INST_DEFINE(idx, pio_i2s_init, NULL, &pio_i2s##idx##_data,			\
			      &pio_i2s##idx##_config, POST_KERNEL,				\
			      CONFIG_I2S_INIT_PRIORITY,					\
			      &i2s_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_I2S_INIT)

