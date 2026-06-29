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


#include "zephyr/sys/__assert.h"
#include <stdint.h>
#define DT_DRV_COMPAT raspberrypi_pico_i2s_pio

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <hardware/pio.h>
#include <zephyr/drivers/dma.h>
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
	bool tx_stop_without_draining;
	struct k_msgq *msgq;
	uint32_t dma_channel;
	const struct device *dev_dma;
	struct dma_config dma_cfg;
	struct k_spinlock lock;

	struct i2s_config cfg; // TODO: move config to data struct.
	void *mem_block;

	const uint32_t data_pin;
};

struct pio_i2s_data {
    struct stream tx;
    struct stream rx;
    uint32_t channel_length;
    uint32_t sampling_freq;
    uint8_t sm;
    bool sm_allocated;
    uint32_t offset;
    uint32_t entry_point;
    const pio_program_t *loaded_program;
};


// TODO: Think about integers.
/*
 * f_sys = system_clock_frequency
 * f_b = frequency of Bit CLK
 * f_pio = frequency of PIO cycles
 * f_s = sampling frequency
 * k = 2 for i2s_controller_tx program,
 * k = 4 for i2s_controller program
 *
 * f_b = f_s * channel_length * num_channels
 * f_pio = f_sys / divider
 * f_pio = k * f_b
 *
 * => k * f_b = f_sys / divider
 * => divider = f_sys / (k * f_b) = f_sys/(k * f_s * channel_length * num_channels)
 */

static int i2s_rpi_pico_write(const struct device *dev, void *mem_block, size_t size)
{
	// const struct pio_i2s_config *config = dev->config;
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

static int i2s_rpi_pico_read(const struct device *dev, void **mem_block, size_t *size)
{
	// const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	const struct stream *stream = &dev_data->rx;
	enum i2s_state state = stream->state;

	if (state == I2S_STATE_NOT_READY) {
		LOG_DBG("Invalid state: %d", (int)state);
		return -EIO;
	}

	struct queue_item item;
	int retval = k_msgq_get(stream->msgq, &item, K_MSEC(stream->cfg.timeout));
	if (retval < 0) {
		return -EIO;
	}
	*mem_block = item.mem_block;
	*size = item.size;

	return 0;
}

RPI_PICO_PIO_DEFINE_PROGRAM(i2s_controller, 1, 16,
	0xaa42, //  0: nop                    side 1 [2]
	        //     .wrap_target
	0x6201, //  1: out    pins, 1         side 0 [2]
	0x4801, //  2: in     pins, 1         side 1
	0x0941, //  3: jmp    x--, 1          side 1 [1]
	0x7001, //  4: out    pins, 1         side 2
	0x90c0, //  5: pull   ifempty noblock side 2
	0xb022, //  6: mov    x, y            side 2
	0x5901, //  7: in     pins, 1         side 3 [1]
	0x9840, //  8: push   iffull noblock  side 3
	0x7201, //  9: out    pins, 1         side 2 [2]
	0x5801, // 10: in     pins, 1         side 3
	0x19e9, // 11: jmp    !osre, 9        side 3 [1]
	0x6001, // 12: out    pins, 1         side 0
	0x80c0, // 13: pull   ifempty noblock side 0
	0xa022, // 14: mov    x, y            side 0
	0x4901, // 15: in     pins, 1         side 1 [1]
	0x8840, // 16: push   iffull noblock  side 1
	        //     .wrap
);

static const uint32_t i2s_controller_cycles_factor = 6u;
static const uint32_t i2s_controller_entry_point = 0;

// TODO: check volume with padded stuff
static int pio_i2s_controller_setup(const struct device *dev, enum i2s_dir dir)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	uint32_t sm = dev_data->sm;
	uint32_t channel_length = dev_data->channel_length;
	uint32_t clock_pin_base = dev_config->clock_pin_base;
	pio_sm_config sm_config;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(i2s_controller))) {
		return -EBUSY;
	}

	dev_data->offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(i2s_controller));
	dev_data->entry_point = i2s_controller_entry_point;
	dev_data->loaded_program = RPI_PICO_PIO_GET_PROGRAM(i2s_controller);
	sm_config = pio_get_default_sm_config();
	sm_config_set_wrap(&sm_config, dev_data->offset + i2s_controller_wrap_target, dev_data->offset + i2s_controller_wrap);

	bool setup_tx = dir == I2S_DIR_TX || dir == I2S_DIR_BOTH || dev_data->tx.state != I2S_STATE_NOT_READY;
	bool setup_rx = dir == I2S_DIR_RX || dir == I2S_DIR_BOTH || dev_data->tx.state != I2S_STATE_NOT_READY;
	bool en_loopback = (setup_rx && dev_data->rx.cfg.options & I2S_OPT_LOOPBACK) || (setup_tx && dev_data->tx.cfg.options & I2S_OPT_LOOPBACK);

	uint32_t rx_data_pin = dev_data->rx.data_pin;
	uint32_t tx_data_pin = dev_data->tx.data_pin;

	if(en_loopback) {
		LOG_ERR("en_loopback enabled!");
		tx_data_pin = rx_data_pin;
	}

	if(setup_tx) {
		sm_config_set_out_pins(&sm_config, tx_data_pin, 1);
	}

	if(setup_rx) {
		sm_config_set_in_pins(&sm_config, rx_data_pin);
		sm_config_set_in_pin_count(&sm_config, 1);
	}

	sm_config_set_out_shift(&sm_config, false, false, 32);
	sm_config_set_in_shift(&sm_config, false, false, 32);
	sm_config_set_sideset_pin_base(&sm_config, clock_pin_base);
	sm_config_set_sideset(&sm_config, 2, false, false);
	pio_sm_init(pio, sm, dev_data->offset, &sm_config);

	uint32_t pin_mask, pin_dirs;


	if (setup_tx && setup_rx) {
		pin_mask = (0b1 << tx_data_pin) | (0b1 << rx_data_pin) | (0b11 << clock_pin_base);
		pin_dirs = (0b1 << tx_data_pin) | (0b11 << clock_pin_base);
	} else if(setup_tx) {
		pin_mask = (0b1 << tx_data_pin) | (0b11 << clock_pin_base);
		pin_dirs = (0b1 << tx_data_pin) | (0b11 << clock_pin_base);
	} else {
		pin_mask = (0b1 << rx_data_pin) | (0b11 << clock_pin_base);
		pin_dirs = (0b11 << clock_pin_base);

	}
	pio_sm_set_pindirs_with_mask(pio, sm, pin_dirs, pin_mask);
	pio_sm_set_pins(pio, sm, 0); // clear pins

	uint32_t sample_freq = dev_data->sampling_freq;
	/* Number of channels is always 2 for I2S data format */
	const uint32_t num_channels = 2;
	uint64_t system_clock_frequency = sys_clock_hw_cycles_per_sec();
	/* 8.8 fixed-point divider: (f_sys << 8) / (2 * f_s * channel_length * num_channels) */
	uint64_t divider = (system_clock_frequency << 8u) /
	(i2s_controller_cycles_factor * sample_freq * channel_length * num_channels);
	pio_sm_set_clkdiv_int_frac(pio, sm, divider >> 8u, divider & 0xffu);

	return 0;
}

static void pio_i2s_controller_start(const struct device *dev)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	uint32_t sm = dev_data->sm;
	uint32_t channel_length = dev_data->channel_length;

	pio_sm_set_enabled(pio, sm, false);
	pio_sm_exec(pio, sm, pio_encode_set(pio_x, channel_length-2));
	pio_sm_exec(pio, sm, pio_encode_set(pio_y, channel_length-2));

	pio_sm_exec(pio, sm, pio_encode_jmp(dev_data->offset + dev_data->entry_point));
	pio_sm_set_enabled(pio, sm, true);
}

static void drop_queue(struct stream *stream) {
	struct queue_item item;
	while (k_msgq_get(stream->msgq, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, item.mem_block);
	}
}

// TODO: make stream->state a per-device
static int i2s_rpi_pico_configure_single(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	int retval;

	__ASSERT_NO_MSG(dir == I2S_DIR_RX || dir == I2S_DIR_TX);

	struct stream *stream = dir == I2S_DIR_RX ? &dev_data->rx : &dev_data->tx;
	struct stream *other_stream = dir == I2S_DIR_RX ? &dev_data->tx : &dev_data->rx;

	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		LOG_ERR("stream in invalid state (%d)", stream->state);
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		memset(&stream->cfg, 0, sizeof(struct i2s_config));
		drop_queue(stream);
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}


	if(other_stream->state != I2S_STATE_NOT_READY) {
		if (i2s_cfg->frame_clk_freq != other_stream->cfg.frame_clk_freq) {
			LOG_ERR("simultaneously configured streams have different frame_clk_freq (%d) (%d)", i2s_cfg->frame_clk_freq, other_stream->cfg.frame_clk_freq);
			return -EINVAL;
		}

		if (i2s_cfg->word_size != other_stream->cfg.word_size) {
			LOG_ERR("simultaneously configured streams have different word_size (%d) (%d)", i2s_cfg->word_size, other_stream->cfg.word_size);
			return -EINVAL;
		}
	}


	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);

	size_t sm;
	if(!dev_data->sm_allocated) {
		retval = pio_rpi_pico_allocate_sm(dev_config->piodev, &sm);

		if (retval < 0) {
			LOG_ERR("pio_rpi_pico_allocate_sm failed with ret = %d", retval);
			return retval;
		}
		dev_data->sm = sm;
		dev_data->sm_allocated = true;
	} else {
		sm = dev_data->sm;
	}

	stream->dma_cfg.user_data = (void*) dev;
	stream->dma_cfg.dma_slot = RPI_PICO_DMA_DREQ_TO_SLOT(pio_get_dreq(pio, sm, dir == I2S_DIR_TX));
	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	dev_data->channel_length = i2s_cfg->word_size > 16 ? 32 : 16;
	dev_data->sampling_freq = i2s_cfg->frame_clk_freq;

	if (dev_data->loaded_program != NULL) {
		pio_remove_program(pio, dev_data->loaded_program, dev_data->offset);
	}

	retval = pio_i2s_controller_setup(dev, dir);

	if (retval < 0) {
		return retval;
	}

	stream->state = I2S_STATE_READY;

	return 0;

}

// TODO: Verify that sampling frequency is the same between tx and rx.
// TODO: verify target vs loopback modes.
static int i2s_rpi_pico_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	// const struct pio_i2s_config *dev_config = dev->config;
	// struct pio_i2s_data *dev_data = dev->data;
	int retval;

	uint8_t data_format = i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK;

	if (data_format != I2S_FMT_DATA_FORMAT_I2S) {
		LOG_DBG("Unsupported data format: %u", (unsigned int)data_format);
		return -EINVAL;
	}

	if (!(16 <= i2s_cfg->word_size && i2s_cfg->word_size <= 32)) {
		LOG_ERR("I2S word size (%d) is unsupported.", i2s_cfg->word_size);
		return -EINVAL;
	}

	bool is_bit_clk_target = i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET;
	bool is_frame_clk_target = i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET;

	if (is_bit_clk_target || is_frame_clk_target) {
		LOG_ERR("I2S target mode unsupported.");
		return -EINVAL;
	}

	// if (i2s_cfg->options & I2S_OPT_LOOPBACK) {
	// 	LOG_ERR("I2S loopback mode unsupported.");
	// 	return -EINVAL;
	// }

	if (i2s_cfg->options & I2S_OPT_PINGPONG) {
		LOG_ERR("I2S_OPT_PINGPONG is unsupported.");
		return -EINVAL;
	}

	if (i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) {
		LOG_ERR("Gated bit clock is unsupported.");
		return -EINVAL;
	}

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		retval = i2s_rpi_pico_configure_single(dev, I2S_DIR_RX, i2s_cfg);
		if(retval < 0) {
			return retval;
		}
	}

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		retval = i2s_rpi_pico_configure_single(dev, I2S_DIR_TX, i2s_cfg);
		if(retval < 0) {
			return retval;
		}
	}

	pio_i2s_controller_start(dev);

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
	// uint dma_channel = data->tx.dma_channel;
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
	if(stream->state == I2S_STATE_STOPPING && stream->tx_stop_without_draining) {
		// TODO: dma_stop seems useless?!
		retval = dma_stop(stream->dev_dma, stream->dma_channel);
		if(retval < 0) {
			stream->state = I2S_STATE_ERROR;
			return;
		}
		stream->state = I2S_STATE_READY;
		return;
	}

	// I2S_TRIGGER_DRAIN
	if(stream->state == I2S_STATE_STOPPING && queue_is_empty(stream->msgq)) {
		retval = dma_stop(stream->dev_dma, stream->dma_channel);
		if(retval < 0) {
			stream->state = I2S_STATE_ERROR;
			return;
		}
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
		(void *)&pio->txf[data->sm],
		mem_block_size);

	if (retval < 0) {
		LOG_DBG("Failed to start TX DMA transfer: %d", retval);
		return;
	}
}

void dma_rx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status) {
	const struct device *dev = (const struct device *)arg;
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	// uint dma_channel = dev_data->rx.dma_channel;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	// TODO: Use a spinlock here?

	int retval;

	struct stream *stream = &dev_data->rx;

	if (status < 0) {
		LOG_ERR("Something went wrong with DMA. status=%d", status);
		stream->state = I2S_STATE_ERROR;
		return; // TODO: abort DMA?
	}

	if (stream->state == I2S_STATE_ERROR) {
		return;
	}

	void *mblk_tmp = stream->mem_block;

	/* Prepare to receive the next data block */
	retval = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (retval < 0) {
		stream->state = I2S_STATE_ERROR;
		return;
	}

	// struct queue_item item;
	// size_t mem_block_size;
	// int ret = k_msgq_get(stream->msgq, &item, SYS_TIMEOUT_MS(0));
	struct queue_item item = {.mem_block = mblk_tmp, .size = stream->cfg.block_size};

	retval = k_msgq_put(stream->msgq, &item, K_NO_WAIT);
	if (retval < 0) {
		LOG_ERR("RX overrun");
		stream->state = I2S_STATE_ERROR;
		return;
	}

	if(stream->state == I2S_STATE_STOPPING && dev_data->tx.state != I2S_STATE_STOPPING) {
		retval = dma_stop(stream->dev_dma, stream->dma_channel);
		if(retval < 0) {
			stream->state = I2S_STATE_ERROR;
			return;
		}

		stream->state = I2S_STATE_READY;

		return;
	}

	retval = reload_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)&pio->rxf[dev_data->sm],
			stream->mem_block,
			stream->cfg.block_size);

	if (retval < 0) {
		LOG_ERR("Failed to start RX DMA transfer: %d", retval);
		stream->state = I2S_STATE_ERROR;
		return;
	}

	return;
}

static int pio_i2s_init(const struct device *dev)
{
	const struct pio_i2s_config *dev_config = dev->config;
	// struct pio_i2s_data *dev_data = dev->data;
	int retval;

	retval = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		LOG_ERR("pinctrl_apply_state failed with ret = %d", retval);
        	return retval;
	}
	return 0;
}

int i2s_start_rx_stream_dma(const struct device *dev, struct stream *stream) {
	const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);

	// struct stream *stream = &data->tx;
	int retval;

	retval = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (retval < 0) {
		return retval;
	}


	// size_t mem_block_size;
	// struct queue_item item;
	// int ret = k_msgq_get(stream->msgq, &item, SYS_TIMEOUT_MS(0));

	// if (ret < 0) {
	// 	LOG_ERR("TX buffer is empty.");
	// 	return ret;
	// }

    	// mem_block_size = item.size;
	retval = start_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)&pio->rxf[data->sm],
			false, stream->mem_block,
			true, stream->cfg.block_size
	);

	// ret = start_dma(stream->dev_dma, stream->dma_channel,
	// 		&stream->dma_cfg,
	// 		stream->mem_block, true, /* TODO: scr addr increment setting? */
	// 		(void *)&pio->txf[data->sm],
	// 		false,
	// 		mem_block_size);
	if (retval < 0) {
		LOG_ERR("Failed to start RX DMA transfer: %d", retval);
		return retval;
	}
	return 0;

}

int i2s_start_stream_dma(const struct device *dev, struct stream *stream) {
	const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);

	// struct stream *stream = &data->tx;

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
			(void *)&pio->txf[data->sm],
			false,
			mem_block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}
	return 0;

}

static int i2s_start_stream_tx(const struct device *dev, struct stream *stream) {
	if (stream->state != I2S_STATE_READY) {
		LOG_ERR("START trigger: invalid state %d",
			    stream->state);
		return -EIO;
	}
	stream->tx_stop_without_draining = false;
	int retval = i2s_start_stream_dma(dev, stream);
	if (retval < 0) {
		LOG_ERR("START trigger failed %d", retval);
		return retval;
	}
	// pio_i2s_controller_start(dev);
	stream->state = I2S_STATE_RUNNING;
	return 0;
}

static int i2s_start_stream_rx(const struct device *dev, struct stream *stream) {
	if (stream->state != I2S_STATE_READY) {
		LOG_ERR("START trigger: invalid state %d",
			    stream->state);
		return -EIO;
	}

        int retval = i2s_start_rx_stream_dma(dev, stream);
	if (retval < 0) {
		LOG_ERR("START trigger failed %d", retval);
		return retval;
	}
	// pio_i2s_controller_start(dev);
	stream->state = I2S_STATE_RUNNING;
	return 0;
}

static int i2s_drop_stream(const struct device *dev, struct stream *stream) {
	k_spinlock_key_t key = k_spin_lock(&stream->lock);
	(void) dma_stop(stream->dev_dma, stream->dma_channel);
	drop_queue(stream);
	stream->state = I2S_STATE_READY;
	k_spin_unlock(&stream->lock, key);
	return 0;
}

static int i2s_stop_stream(const struct device *dev, struct stream *stream) {
	k_spinlock_key_t key = k_spin_lock(&stream->lock);
	if (stream->state != I2S_STATE_RUNNING) {
		k_spin_unlock(&stream->lock, key);
		LOG_ERR("STOP trigger: invalid state %d", stream->state);
		return -EIO;
	}
	stream->state = I2S_STATE_STOPPING;
	stream->tx_stop_without_draining = true;
	k_spin_unlock(&stream->lock, key);
	return 0;
}

static int i2s_drain_stream(const struct device *dev, struct stream *stream) {
	k_spinlock_key_t key = k_spin_lock(&stream->lock);
	if (stream->state != I2S_STATE_RUNNING) {
		k_spin_unlock(&stream->lock, key);
		LOG_ERR("DRAIN trigger: invalid state %d",
		         stream->state);
		return -EIO;
	}
	stream->state = I2S_STATE_STOPPING;
	k_spin_unlock(&stream->lock, key);
	return 0;
}

static int i2s_drain_prepare(const struct device *dev, struct stream *stream) {
	k_spinlock_key_t key = k_spin_lock(&stream->lock);
	if (stream->state != I2S_STATE_ERROR) {
		k_spin_unlock(&stream->lock, key);
		LOG_ERR("PREPARE trigger: invalid state %d",
		         stream->state);
		return -EIO;
	}
	stream->state = I2S_STATE_READY;
	k_spin_unlock(&stream->lock, key);
	return 0;
}

static int i2s_rpi_pico_trigger_single(const struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
	// const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	int ret;

	__ASSERT_NO_MSG(dir == I2S_DIR_RX || dir == I2S_DIR_TX);

	struct stream *stream = dir == I2S_DIR_RX ? &dev_data->rx : &dev_data->tx;

	switch (cmd) {
	case I2S_TRIGGER_START:
		if(stream->state != I2S_STATE_READY) {
			LOG_ERR("Stream state must be in ready state to start stream.");
			return -EINVAL; // TODO: correct error code?
		}

		//TODO: inline these functions a bit
		if (dir == I2S_DIR_TX) {
			ret = i2s_start_stream_tx(dev, stream);
		} else {
			ret = i2s_start_stream_rx(dev, stream);
		}
		if (ret < 0) {
			return ret;
		}
		LOG_ERR("i2s_rpi_pico_trigger_single dir=%d started successfully", dir);
		break;
	case I2S_TRIGGER_STOP:
		//TODO: what if DMA is not running?
		i2s_stop_stream(dev, stream);
		break;
	case I2S_TRIGGER_DRAIN:
		//TODO: what if queue already empty?
		i2s_drain_stream(dev, stream);
		break;
	case I2S_TRIGGER_DROP:
		i2s_drop_stream(dev, stream);
		break;
	case I2S_TRIGGER_PREPARE:
		i2s_drain_prepare(dev, stream);
		break;

	default:
        //TODO: Handle all other trigger commands
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}
	return 0;
}

static int i2s_rpi_pico_trigger(const struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
	// const struct pio_i2s_config *dev_config = dev->config;
	// struct pio_i2s_data *dev_data = dev->data;
	int retval;

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		retval = i2s_rpi_pico_trigger_single(dev, I2S_DIR_RX, cmd);
		if(retval < 0) {
			return retval;
		}
	}

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		retval = i2s_rpi_pico_trigger_single(dev, I2S_DIR_TX, cmd);
		if(retval < 0) {
			return retval;
		}
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
		stream = &dev_data->rx;
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
	.read = i2s_rpi_pico_read,
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
		.tx_stop_without_draining = false,                                         \
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
		.tx_stop_without_draining = false,                                         \
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
        .sm_allocated = false,                              \
        .loaded_program = NULL                             \
    };					\
	DEVICE_DT_INST_DEFINE(idx, pio_i2s_init, NULL, &pio_i2s##idx##_data,			\
			      &pio_i2s##idx##_config, POST_KERNEL,				\
			      CONFIG_I2S_INIT_PRIORITY,					\
			      &i2s_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_I2S_INIT)
