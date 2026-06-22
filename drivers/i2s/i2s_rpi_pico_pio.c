/*
 * Copyright (c) 2026 Robin Sachsenweger Ballantyne <makenenjoy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
	TOOO:
	 - Check that all errors are handled properly
		- check that i2s_config->timeout is always respected.
	 - Respect all properties of config
	 - RX stream functionality
	 - Support all trigger commands
	 - Change LOG statements to be more inline with the rest of zephyr
	 - When there are multiple PIO programs written, make static KConfig options for
	      enabling support for various PIO programs. Only the statically enabled ones
	      can be used in the configure function.
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
	const uint32_t clock_pin_base;
};

struct stream {
	enum i2s_state state;
	bool tx_stop_for_drain;
	struct k_msgq *msgq;
	uint32_t dma_channel;
	const struct device *dev_dma;
	struct dma_config dma_cfg;
	uint8_t sm;
	struct k_spinlock lock;

	struct i2s_config cfg;
	void *mem_block;

	const uint32_t data_pin;
};

struct pio_i2s_data {
    struct stream tx;
    struct stream rx;
};

// TODO: Do some experiments to tripple check that this is correct.
/*
 * f_sys = system_clock_frequency
 * f_b = frequency of Bit CLK
 * f_pio = frequency of PIO cycles
 * f_s = sampling frequency
 *
 * f_b = f_s * channel_length * num_channels
 * f_pio = f_sys / divider
 * f_pio = 2 * f_b
 *
 * => 2 * f_b = f_sys / divider
 * => divider = f_sys / (2 * f_b) = f_sys/(2 * f_s * channel_length * num_channels)

 */
void update_pio_frequency(PIO pio, uint32_t sm, uint32_t sample_freq, uint32_t channel_length, uint32_t num_channels) {
    uint64_t system_clock_frequency = clock_get_hz(clk_sys);
    /* 8.8 fixed-point divider: (f_sys << 8) / (2 * f_s * channel_length * num_channels) */
    uint64_t divider = (system_clock_frequency << 8u) /
		       (2u * sample_freq * channel_length * num_channels);
    assert(divider < 0x1000000); // TODO: These errors should be handled better
    pio_sm_set_clkdiv_int_frac(pio, sm, divider >> 8u, divider & 0xffu);
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


/*
 * TODO:
 * Either build 3 different version of this code for the 16bit, 24 bit, 32 bit or
 * find another way to make this program variable like that.
 *
 * Look at this other github project...
 * this guy somehow does it without: malacalypse/rp2040_i2s_example
 */

RPI_PICO_PIO_DEFINE_PROGRAM(i2s_controller_tx, 0, 7,
	        //     .wrap_target
	0xb822, //  0: mov    x, y            side 3
	0x7001, //  1: out    pins, 1         side 2
	0x1841, //  2: jmp    x--, 1          side 3
	0x6001, //  3: out    pins, 1         side 0
	0xa822, //  4: mov    x, y            side 1
	0x6001, //  5: out    pins, 1         side 0
	0x0845, //  6: jmp    x--, 5          side 1
	0x7001, //  7: out    pins, 1         side 2
                //     .wrap
);

#define i2s_controller_tx_offset_entry_point 0u

static int pio_i2s_tx_init(PIO pio, uint32_t sm, uint32_t data_pin, uint32_t clock_pin_base, uint32_t bit_depth)
{
	uint32_t offset;
	pio_sm_config sm_config;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(i2s_controller_tx))) {
		return -EBUSY;
	}

	offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(i2s_controller_tx));
	sm_config = pio_get_default_sm_config();
	sm_config_set_wrap(&sm_config, offset + i2s_controller_tx_wrap_target, offset + i2s_controller_tx_wrap);
	sm_config_set_sideset(&sm_config, 2, false, false);
	sm_config_set_out_pins(&sm_config, data_pin, 1);
	sm_config_set_sideset_pins(&sm_config, clock_pin_base);
	sm_config_set_out_shift(&sm_config, false, true, 32);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
	pio_sm_init(pio, sm, offset, &sm_config);
	uint32_t pin_mask = (1u << data_pin) | (3u << clock_pin_base);
	pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
	pio_sm_set_pins(pio, sm, 0); // clear pins
	pio_sm_exec(pio, sm, pio_encode_set(pio_y, bit_depth-2));
	pio_sm_exec(pio, sm, pio_encode_jmp(offset + i2s_controller_tx_offset_entry_point));


	return 0;
}

static int i2s_rpi_pico_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;


	uint8_t data_format = i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK;

	if (data_format != I2S_FMT_DATA_FORMAT_I2S) {
		LOG_DBG("Unsupported data format: %u", (unsigned int)data_format);
		return -EINVAL;
	}

	/* Number of channels is always 2 for I2S data format */
	const uint32_t num_channels = 2;

	if (dir != I2S_DIR_TX) {
		LOG_ERR("I2S direction is unsupported."); // TODO:
		return -EINVAL;
	}

	if (!(16 <= i2s_cfg->word_size && i2s_cfg->word_size <= 32)) {
		LOG_ERR("I2S word size (%d) is unsupported.", i2s_cfg->word_size);
		return -EINVAL;
	}

	/* For words greater than 16-bit the channel length is considered 32-bit */
	const uint32_t channel_length = i2s_cfg->word_size > 16U ? 32U : 16U;

	struct stream *stream = &dev_data->tx;

	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	bool is_bit_clk_target = i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET;
	bool is_frame_clk_target = i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET;

	if (is_bit_clk_target || is_frame_clk_target) {
		LOG_ERR("I2S target mode unsupported.");
		return -EINVAL;
	}

	if (i2s_cfg->options & I2S_OPT_LOOPBACK) {
		LOG_ERR("I2S loopback mode unsupported.");
		return -EINVAL;
	}

	if (i2s_cfg->options & I2S_OPT_PINGPONG) {
		LOG_ERR("I2S_OPT_PINGPONG unsupported.");
		return -EINVAL;
	}

	if (!(i2s_cfg->options & I2S_OPT_BIT_CLK_GATED)) {
		LOG_ERR("Continous bit clock is unsupported.");
		return -EINVAL;
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));


	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);

	size_t tx_sm;
	int retval;
	retval = pio_rpi_pico_allocate_sm(dev_config->piodev, &tx_sm);
	if (retval < 0) {
		LOG_ERR("pio_rpi_pico_allocate_sm failed with ret = %d", retval);
		return retval;
	}

	dev_data->tx.sm = tx_sm;
	dev_data->tx.dma_cfg.user_data = (void*) dev;
	dev_data->tx.dma_cfg.dma_slot = RPI_PICO_DMA_DREQ_TO_SLOT(pio_get_dreq(pio, dev_data->tx.sm, true));

	// uint32_t data_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_pins, 0),	\
	// clock_pin_base = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_pins, 1),	\
	// DT_RPI_PICO_PIO_PIN_BY_NAME(node, default, 0, tx_gpio, 0)

	// uint32_t data_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(dev, default, 0, tx_pins, 0);

	retval = pio_i2s_tx_init(pio, tx_sm, dev_data->tx.data_pin, dev_config->clock_pin_base, channel_length);
	if (retval < 0) {
		LOG_ERR("pio_i2s_tx_init failed with ret = %d", retval);
		return retval;
	}

	update_pio_frequency(pio, dev_data->tx.sm, i2s_cfg->frame_clk_freq, channel_length, num_channels);

	stream->state = I2S_STATE_READY;
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

void dma_tx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
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

	// I2S_TRIGGER_STOP
	// TODO: combine the two if statements together?
	if(stream->state == I2S_STATE_STOPPING && stream->tx_stop_for_drain) {
		// TODO: dma_stop seems useless?!
		dma_stop(stream->dev_dma, stream->dma_channel);
		stream->state = I2S_STATE_READY;
		return;
	}

	// I2S_TRIGGER_DRAIN
	if(stream->state == I2S_STATE_STOPPING && queue_is_empty(stream->msgq)) {
		stream->state = I2S_STATE_READY;
		return;
	}

	struct queue_item item;
	size_t mem_block_size;
	int ret = k_msgq_get(stream->msgq, &item, SYS_TIMEOUT_MS(0));
	if (ret < 0) {
		LOG_ERR("TX buffer underrun.");
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

void dma_rx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status) {
	// do stuff
	return;
}
static int pio_i2s_init(const struct device *dev)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	int retval;

	retval = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		LOG_ERR("pinctrl_apply_state failed with ret = %d", retval);
        	return retval;
	}

	uint8_t dma_channel = dev_data->tx.dma_channel;
	retval = dma_config(dev_data->tx.dev_dma, dma_channel, &dev_data->tx.dma_cfg);
	if (retval < 0) {
		LOG_ERR("dma ctrl %p: dma_config failed with %d", dev_data->tx.dev_dma, retval);
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
		LOG_ERR("TX buffer is empty.");
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
	k_spinlock_key_t key;

	// TODO: Maybe refactor this to avoid so much code duplication with taking locks
	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				    stream->state);
			return -EIO;
		}
		/* No spin lock because tx callback cannot be running while stream->state == I2S_STATE_READY. */
		stream->tx_stop_for_drain = false;
	        ret = audio_i2s_start(dev);
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}
		stream->state = I2S_STATE_RUNNING;
		break;
	case I2S_TRIGGER_STOP:
		//TODO: what if DMA is not running?
		key = k_spin_lock(&stream->lock);
		if (stream->state != I2S_STATE_RUNNING) {
			k_spin_unlock(&stream->lock, key);
			LOG_ERR("STOP trigger: invalid state %d",
			         stream->state);
			return -EIO;
		}
		stream->state = I2S_STATE_STOPPING;
		stream->tx_stop_for_drain = true;
		k_spin_unlock(&stream->lock, key);
		break;
	case I2S_TRIGGER_DRAIN:
		//TODO: what if queue already empty?
		key = k_spin_lock(&stream->lock);
		if (stream->state != I2S_STATE_RUNNING) {
			k_spin_unlock(&stream->lock, key);
			LOG_ERR("DRAIN trigger: invalid state %d",
			         stream->state);
			return -EIO;
		}
		stream->state = I2S_STATE_STOPPING;
		k_spin_unlock(&stream->lock, key);
		break;
	case I2S_TRIGGER_PREPARE:
		key = k_spin_lock(&stream->lock);
		if (stream->state != I2S_STATE_ERROR) {
			k_spin_unlock(&stream->lock, key);
			LOG_ERR("PREPARE trigger: invalid state %d",
			         stream->state);
			return -EIO;
		}
		stream->state = I2S_STATE_READY;
		k_spin_unlock(&stream->lock, key);
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
/*  TODO:
 *  The PIO program drives the bit clock (BCLK) and word select (WS/LRCLK) from a
 * 2-bit sideset. Sideset pins are a contiguous range starting at a base pin, so
 * the hardware forces WS == BCLK + 1. tx_pins is <DATA BCLK WS>; this asserts the
 * overlay honours the adjacency rather than failing silently at runtime.
 * Add a build assert such as possible this:
 *  BUILD_ASSERT(PIO_I2S_WS_PIN(idx) == PIO_I2S_BCLK_PIN(idx) + 1,                             \
         "I2S word-select pin must be bit-clock pin + 1 "                            \
         "(PIO sideset pins are contiguous); fix tx_pins order in the overlay");\
 */
#define PIO_I2S_INIT(idx)									\
	PINCTRL_DT_INST_DEFINE(idx);								\
	static const struct pio_i2s_config pio_i2s##idx##_config = {				\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(idx)),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),					\
		.clock_pin_base = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, clks, 0)	\
	};                                                  \
	K_MSGQ_DEFINE(tx_##idx##_queue, sizeof(struct queue_item),		\
	        32, 4);			\
	K_MSGQ_DEFINE(rx_##idx##_queue, sizeof(struct queue_item),		\
	        32, 4);			\
	static struct pio_i2s_data pio_i2s##idx##_data = {                \
        .tx = {                                                        \
		.msgq = &tx_##idx##_queue,                               \
		.state = I2S_STATE_NOT_READY,                                \
		.tx_stop_for_drain = false,                                         \
		.dev_dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, tx)),		\
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(idx, tx, channel),  \
		.data_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_data, 0),	\
		.dma_cfg = {							\
			.block_count = 1, /* block_count > 1 not supported */	\
			.channel_direction = MEMORY_TO_PERIPHERAL,		\
			.source_data_size = 4,  /* 32bit hard coded */		\
			.dest_data_size = 4,    /* TODO: 32bit hard coded */		\
			/* single transfers (burst length = data size) */	\
			.source_burst_length = 1, /* unused i think */			\
			.dest_burst_length = 1,	/* unused i think */			\
			.channel_priority = 1, /* TODO: hardcoded */		\
			.dma_callback = dma_tx_callback			\
		},								\
        },                                             \
        .rx = {                                                        \
		.msgq = &rx_##idx##_queue,                               \
		.state = I2S_STATE_NOT_READY,                                \
		.tx_stop_for_drain = false,                                         \
		.dev_dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, rx)),		\
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(idx, rx, channel),  \
		.data_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, rx_data, 0),	\
		.dma_cfg = {							\
			.block_count = 1, /* block_count > 1 not supported */	\
			.channel_direction = PERIPHERAL_TO_MEMORY,		\
			.source_data_size = 4,  /* 32bit hard coded */		\
			.dest_data_size = 4,    /* TODO: 32bit hard coded */		\
			/* single transfers (burst length = data size) */	\
			.source_burst_length = 1, /* unused i think */			\
			.dest_burst_length = 1,	/* unused i think */			\
			.channel_priority = 1, /* TODO: hardcoded */		\
			.dma_callback = dma_rx_callback			\
		},								\
        },                                             \
    };					\
	DEVICE_DT_INST_DEFINE(idx, pio_i2s_init, NULL, &pio_i2s##idx##_data,			\
			      &pio_i2s##idx##_config, POST_KERNEL,				\
			      CONFIG_I2S_INIT_PRIORITY,					\
			      &i2s_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_I2S_INIT)
