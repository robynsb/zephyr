/*
 * Copyright (c) 2026 Robin Sachsenweger Ballantyne <makenenjoy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
	TOOO:
	 - Change LOG statements to be more inline with the rest of zephyr
*/

// TODO: write test for the sampling frequency check works
// TODO: check for code smell involving functions with only one call site.
// TODO: check all the functions are static.

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

#define PIO_I2S_NUM_INST_OK DT_NUM_INST_STATUS_OKAY(raspberrypi_pico_i2s_pio)
#define PIO_I2S_IS_DIR_INST_EN(idx, dir) DT_INST_DMAS_HAS_NAME(idx, dir)
#define PIO_I2S_IS_DIR_EN(dir)                                                                     \
	(LISTIFY(PIO_I2S_NUM_INST_OK, PIO_I2S_IS_DIR_INST_EN, (||), dir))

struct queue_item {
	void *mem_block;
	size_t size;
};

struct pio_i2s_config {
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	const uint32_t clock_pin;
	const uint32_t ws_pin;
	const uint32_t in_base_pin;
};

/* A PIO program loaded into instruction memory. One copy is shared by every
 * state machine running it, so the load is reference counted.
 */
struct pio_prog {
	const pio_program_t *prog; /* NULL = not loaded */
	uint32_t offset;
	uint8_t users;             /* state machines currently running it */
};

struct stream {
	enum i2s_state state;
	bool tx_stop_without_draining;
	struct k_msgq *msgq;
	uint32_t dma_channel;
	const struct device *dev_dma;
	struct dma_config dma_cfg;

	struct i2s_config cfg;
	void *mem_block;

	const uint32_t data_pin;

	size_t sm; /* (size_t)-1 = not claimed */
};

struct pio_i2s_data {
    struct stream tx;
    struct stream rx;
    uint32_t channel_length;
    uint32_t sampling_freq;
    size_t clks_sm; /* (size_t)-1 = not claimed */
    struct pio_prog clks_prog;
    struct pio_prog target_prog;
    struct k_spinlock lock;
};

static bool stream_is_present(const struct stream *stream)
{
	return stream->dev_dma != NULL;
}

static int i2s_rpi_pico_write(const struct device *dev, void *mem_block, size_t size)
{
	// const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	const struct stream *stream = &data->tx;
	enum i2s_state state = stream->state;
	int retval = 0;

	if (!stream_is_present(stream)) {
		LOG_DBG("TX not enabled");
		return -EIO;
	}

	if (state != I2S_STATE_RUNNING && state != I2S_STATE_READY) {
		LOG_DBG("Invalid state: %d", (int)state);
		return -EIO;
	}

	if (size > stream->cfg.block_size) {
		LOG_DBG("Max write size is: %u", stream->cfg.block_size);
		return -EIO;
	}

	struct queue_item item = {.mem_block = mem_block, .size = size};

	retval = k_msgq_put(stream->msgq, &item, SYS_TIMEOUT_MS(stream->cfg.timeout));
	if (retval < 0) {
		LOG_ERR("TX queue full");
		return retval;
	}

    return 0;
}

static int i2s_rpi_pico_read(const struct device *dev, void **mem_block, size_t *size)
{
	// const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	const struct stream *stream = &dev_data->rx;
	enum i2s_state state = stream->state;

	if (!stream_is_present(stream)) {
		LOG_DBG("RX not enabled");
		return -EIO;
	}

	if (state == I2S_STATE_NOT_READY) {
		LOG_DBG("Invalid state: %d", (int)state);
		return -EIO;
	}

	struct queue_item item;
	int retval = k_msgq_get(stream->msgq, &item,
				(state == I2S_STATE_ERROR)
					? K_NO_WAIT
					: SYS_TIMEOUT_MS(stream->cfg.timeout));

	if (retval < 0) {
		if (retval == -ENOMSG) {
			retval = -EIO;
		}
		return retval;
	}
	*mem_block = item.mem_block;
	*size = item.size;

	return 0;
}


/* Clock generator: 1-bit sideset = BCLK; `mov pins, !pins` toggles WS (out/in base).
 * Seeded by the y register (= channel_length - 2). 2 PIO cycles per BCLK period. */
RPI_PICO_PIO_DEFINE_PROGRAM(clks, 0, 3,
		//     .wrap_target
	0xb022, //  0: mov    x, y            side 1
	0xa042, //  1: nop                    side 0
	0x1041, //  2: jmp    x--, 1          side 1
	0xa008, //  3: mov    pins, ~pins     side 0
		//     .wrap
);

static const uint32_t clks_cycles_factor = 2u; /* 2 PIO cycles per BCLK period */
static const uint32_t clks_entry_point = 0;

RPI_PICO_PIO_DEFINE_PROGRAM(target, 4, 12,
	0x20a2, //  0: wait   1 pin, 2
	0x2022, //  1: wait   0 pin, 2
	0x2021, //  2: wait   0 pin, 1
	0x20a1, //  3: wait   1 pin, 1
	        //     .wrap_target
	0x8080, //  4: pull   noblock
	0x2021, //  5: wait   0 pin, 1
	0x6001, //  6: out    pins, 1
	0x20a1, //  7: wait   1 pin, 1
	0x4001, //  8: in     pins, 1
	0x00cd, //  9: jmp    pin, 13
	0x0065, // 10: jmp    !y, 5
	0xa04a, // 11: mov    y, ~y
	0x8000, // 12: push   noblock
	        //     .wrap
	0x006b, // 13: jmp    !y, 11
	0x0005, // 14: jmp    5
);

static int prog_load(const struct device *piodev, struct pio_prog *res, const pio_program_t *prog)
{
	PIO pio = pio_rpi_pico_get_pio(piodev);

	if (res->users > 0) {
		__ASSERT_NO_MSG(res->prog == prog);
		res->users++;
		return 0;
	}

	if (!pio_can_add_program(pio, prog)) {
		LOG_ERR("no PIO instruction memory left for program");
		return -EBUSY;
	}

	res->offset = pio_add_program(pio, prog);
	res->prog = prog;
	res->users = 1;

	return 0;
}

static void prog_unload(const struct device *piodev, struct pio_prog *res)
{
	PIO pio = pio_rpi_pico_get_pio(piodev);

	__ASSERT_NO_MSG(res->users > 0);

	if (--res->users > 0) {
		return;
	}

	pio_remove_program(pio, res->prog, res->offset);
	res->prog = NULL;
}

static int sm_claim(const struct device *piodev, size_t *sm, struct pio_prog *prog_res,
		    const pio_program_t *prog)
{
	PIO pio = pio_rpi_pico_get_pio(piodev);
	int retval;

	__ASSERT_NO_MSG(*sm == (size_t)-1);

	retval = prog_load(piodev, prog_res, prog);
	if (retval < 0) {
		return retval;
	}

	retval = pio_rpi_pico_allocate_sm(piodev, sm);
	if (retval < 0) {
		prog_unload(piodev, prog_res);
		return retval;
	}

	pio_sm_set_enabled(pio, *sm, false);

	return 0;
}

static void sm_release(const struct device *piodev, size_t *sm, struct pio_prog *prog_res,
		       uint32_t out_pins)
{
	PIO pio = pio_rpi_pico_get_pio(piodev);

	if (*sm == (size_t)-1) {
		return;
	}

	pio_sm_set_enabled(pio, *sm, false);

	if (out_pins != 0) {
		pio_sm_set_pindirs_with_mask(pio, *sm, 0, out_pins);
	}

	pio_sm_unclaim(pio, *sm);
	*sm = (size_t)-1;

	prog_unload(piodev, prog_res);
}

static int sm_atomic_set_stream_and_clk(const struct device *dev, enum i2s_dir dir, bool need_clk_sm)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	const struct device *piodev = dev_config->piodev;

	__ASSERT_NO_MSG(dir != I2S_DIR_BOTH);

	int retval;
	bool free_clk_sm_during_error = false;

	if (need_clk_sm && dev_data->clks_sm == (size_t)-1) {
		retval = sm_claim(piodev, &dev_data->clks_sm, &dev_data->clks_prog,
				  RPI_PICO_PIO_GET_PROGRAM(clks));
		if (retval < 0) {
			return -EBUSY;
		}

		free_clk_sm_during_error = true;
	}

	struct stream *stream = dir == I2S_DIR_TX ? &dev_data->tx : &dev_data->rx;

	if (stream->sm == (size_t)-1) {
		retval = sm_claim(piodev, &stream->sm, &dev_data->target_prog,
				  RPI_PICO_PIO_GET_PROGRAM(target));
		if (retval < 0) {
			goto cleanup;
		}
	}

	if (!need_clk_sm && dev_data->clks_sm != (size_t)-1) {
		sm_release(piodev, &dev_data->clks_sm, &dev_data->clks_prog,
			   (1u << dev_config->clock_pin) | (1u << dev_config->ws_pin));
	}

	return 0;

cleanup:
	if(free_clk_sm_during_error) {
		sm_release(piodev, &dev_data->clks_sm, &dev_data->clks_prog, 0);
	}
	return -EBUSY;
}

/*
 * TODO: rewrite this with the used variable names
 * f_sys = system_clock_frequency
 * f_b = frequency of Bit CLK
 * f_pio = instruction frequency of PIO clk state machine
 * f_s = sampling frequency
 * k = number of PIO cycles per bit-clock period in the clks program (k = 2)
 *
 * f_b = f_s * channel_length * num_channels
 * f_pio = f_sys / divider
 * f_pio = k * f_b
 *
 * => k * f_b = f_sys / divider
 * => divider = f_sys / (k * f_b) = f_sys/(k * f_s * channel_length * num_channels)
 */
static uint64_t calculate_divider_shift_8(uint64_t sample_freq, uint64_t channel_length) {
	/* Number of channels is always 2 for I2S data format */
	/* Only I2S supported at this time. */
	const uint64_t num_channels = 2;
	uint64_t system_clock_frequency = sys_clock_hw_cycles_per_sec();
	/* 8.8 fixed-point divider: (f_sys << 8) / (k * f_s * channel_length * num_channels) */
	uint64_t divider = (system_clock_frequency << 8u) /
		(clks_cycles_factor * sample_freq * channel_length * num_channels);
	return divider;
}


static void pio_i2s_setup_clks(const struct device *dev)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	uint32_t bclk_pin = dev_config->clock_pin;
	uint32_t ws_pin = dev_config->ws_pin;
	size_t sm = dev_data->clks_sm;
	uint32_t offset = dev_data->clks_prog.offset;
	pio_sm_config c;
	// int retval;

	c = pio_get_default_sm_config();
	sm_config_set_wrap(&c, offset + clks_wrap_target, offset + clks_wrap);
	sm_config_set_sideset_pin_base(&c, bclk_pin);
	sm_config_set_sideset(&c, 1, false, false);
	sm_config_set_out_pins(&c, ws_pin, 1);
	sm_config_set_in_pins(&c, ws_pin);
	sm_config_set_in_pin_count(&c, 1);
	pio_sm_init(pio, sm, offset, &c);

	/* clks drives BCLK + WS; they are independent pins, so set both bits. */
	uint32_t pin_mask = (1u << bclk_pin) | (1u << ws_pin);
	pio_sm_set_pins_with_mask(pio, sm, 0, pin_mask); /* clear pins */
	pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);

	uint32_t sample_freq = dev_data->sampling_freq;
	uint32_t channel_length = dev_data->channel_length;

	uint64_t divider = calculate_divider_shift_8(sample_freq, channel_length);
	uint64_t div_int = divider >> 8u;
	uint64_t div_frac = divider & 0xffu;

	__ASSERT_NO_MSG(div_int != 0);
	__ASSERT_NO_MSG(div_int <= UINT16_MAX);

	pio_sm_set_clkdiv_int_frac(pio, sm, div_int, div_frac);

	return;
}

static void pio_i2s_setup_stream(const struct device *dev, struct stream *stream,
				enum i2s_dir dir, bool is_controller)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	uint32_t bclk_pin = dev_config->clock_pin;
	uint32_t ws_pin = dev_config->ws_pin;
	size_t sm = stream->sm;
	uint32_t offset = dev_data->target_prog.offset;
	pio_sm_config c;
	// int retval;

	if (dir == I2S_DIR_TX) {
		uint32_t tx_out_pin = stream->data_pin;

		c = pio_get_default_sm_config();
		sm_config_set_wrap(&c, offset + target_wrap_target, offset + target_wrap);
		sm_config_set_out_pins(&c, tx_out_pin, 1);
		sm_config_set_in_pins(&c, dev_config->in_base_pin);
		sm_config_set_in_pin_count(&c, 3);
		sm_config_set_jmp_pin(&c, ws_pin);
		sm_config_set_out_shift(&c, false, false, 1);
		sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
		pio_sm_init(pio, sm, offset, &c);
		pio_sm_set_clkdiv_int_frac(pio, sm, 1, 0);
		pio_sm_set_pins_with_mask(pio, sm, 0, 1u << tx_out_pin);
		pio_sm_set_pindirs_with_mask(pio, sm, 1u << tx_out_pin, 1u << tx_out_pin);
	} else {
		c = pio_get_default_sm_config();
		sm_config_set_wrap(&c, offset + target_wrap_target, offset + target_wrap);
		sm_config_set_in_pins(&c, dev_config->in_base_pin);
		sm_config_set_in_pin_count(&c, 3);
		sm_config_set_jmp_pin(&c, ws_pin);
		sm_config_set_in_shift(&c, false, false, 1);
		sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
		pio_sm_init(pio, sm, offset, &c);
		pio_sm_set_clkdiv_int_frac(pio, sm, 1, 0);

		bool loopback = stream_is_present(&dev_data->tx) &&
				dev_data->tx.data_pin == dev_data->rx.data_pin;

		if (!loopback) {
			pio_sm_set_pindirs_with_mask(pio, sm, 0, 1u << stream->data_pin);
		}
	}

	if (!is_controller) {
		uint32_t clk_pins = (1u << bclk_pin) | (1u << ws_pin);

		pio_sm_set_pindirs_with_mask(pio, sm, 0, clk_pins);
	}

	return;
}

static void pio_i2s_clks_start(const struct device *dev)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	uint32_t channel_length = dev_data->channel_length;
	size_t sm = dev_data->clks_sm;

	pio_sm_set_enabled(pio, sm, false);
	pio_sm_exec(pio, sm, pio_encode_set(pio_y, channel_length - 2));
	pio_sm_exec(pio, sm, pio_encode_jmp(dev_data->clks_prog.offset + clks_entry_point));
	pio_sm_set_enabled(pio, sm, true);
}

static void drop_stream(const struct device *dev, struct stream *stream) {
	const struct pio_i2s_config *dev_config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);

	pio_sm_set_enabled(pio, stream->sm, false);

	struct queue_item item;
	while (k_msgq_get(stream->msgq, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, item.mem_block);
	}
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

}

static int i2s_rpi_pico_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	int retval;

	if (dir != I2S_DIR_RX && dir != I2S_DIR_TX) {
		LOG_ERR("Unsupported I2S direction (%d), configure RX and TX separately", dir);
		return -ENOSYS;
	}

	struct stream *stream = dir == I2S_DIR_RX ? &dev_data->rx : &dev_data->tx;
	struct stream *other_stream = dir == I2S_DIR_RX ? &dev_data->tx : &dev_data->rx;

	if (!stream_is_present(stream)) {
		LOG_DBG("%s not enabled", dir == I2S_DIR_RX ? "RX" : "TX");
		return -EINVAL;
	}

	bool other_is_controller = other_stream->state != I2S_STATE_NOT_READY &&
				!(other_stream->cfg.options &
				  (I2S_OPT_BIT_CLK_TARGET | I2S_OPT_FRAME_CLK_TARGET));


	/* --- config check --- */

	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		LOG_ERR("stream in invalid state (%d)", stream->state);
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		drop_stream(dev, stream);

		sm_release(dev_config->piodev, &stream->sm, &dev_data->target_prog,
			   dir == I2S_DIR_TX ? (1u << stream->data_pin) : 0);
		memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;

		if (!other_is_controller) {
			/* clks drives BCLK + WS (see pio_i2s_setup_clks). */
			sm_release(dev_config->piodev, &dev_data->clks_sm, &dev_data->clks_prog,
				   (1u << dev_config->clock_pin) | (1u << dev_config->ws_pin));
		}
		return 0;
	}

	if (other_stream->state != I2S_STATE_NOT_READY && other_stream->state != I2S_STATE_READY) {
		LOG_ERR("other stream in invalid state (%d)", other_stream->state);
		return -EINVAL;
	}

	if (i2s_cfg->format != I2S_FMT_DATA_FORMAT_I2S) {
		LOG_ERR("Unsupported data format: %u", (unsigned int)i2s_cfg->format);
		return -EINVAL;
	}

	if (i2s_cfg->channels != 2) {
		LOG_ERR("Number of channels not 2 when configured with I2S data format.");
		return -EINVAL;
	}

	if (!(16 <= i2s_cfg->word_size && i2s_cfg->word_size <= 32)) {
		LOG_ERR("I2S word size (%d) is unsupported.", i2s_cfg->word_size);
		return -EINVAL;
	}
	uint32_t channel_length = i2s_cfg->word_size > 16 ? 32 : 16;

	if (i2s_cfg->options & I2S_OPT_LOOPBACK) {
		LOG_ERR("I2S loopback mode unsupported.");
		LOG_DBG("To enable loopback, use the same SD for TX and RX pinctrl");
		return -EINVAL;
	}

	if (i2s_cfg->options & I2S_OPT_PINGPONG) {
		LOG_ERR("I2S_OPT_PINGPONG is unsupported.");
		return -EINVAL;
	}

	bool is_bit_clk_target = i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET;
	bool is_frame_clk_target = i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET;

	if (is_bit_clk_target != is_frame_clk_target) {
		LOG_ERR("I2S bit CLK and frame CLK must be either both target or both controller.");
		return -EINVAL;
	}

	bool i2s_cfg_is_controller = !(i2s_cfg->options &
	        (I2S_OPT_BIT_CLK_TARGET | I2S_OPT_FRAME_CLK_TARGET));

	bool is_controller = i2s_cfg_is_controller || other_is_controller;

	/* check clk configuration */
	if(i2s_cfg_is_controller) {
		if (i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) {
			LOG_ERR("Gated bit clock is unsupported.");
			return -EINVAL;
		}

		if (other_is_controller && i2s_cfg->frame_clk_freq != other_stream->cfg.frame_clk_freq) {
			LOG_ERR("simultaneously configured controller streams have different frame_clk_freq (%d) (%d)", i2s_cfg->frame_clk_freq, other_stream->cfg.frame_clk_freq);
			return -EINVAL;
		}

		uint64_t divider =
			calculate_divider_shift_8(i2s_cfg->frame_clk_freq, channel_length) >> 8u;
		// TODO: I guess that this is actually not strict enough.
		if (divider == 0) {
			LOG_ERR("sampling frequency is too high");
			return -EINVAL;
		}
		if (divider > UINT16_MAX) {
			LOG_ERR("sampling frequency is too low");
			return -EINVAL;
		}
	}

	if (other_stream->state != I2S_STATE_NOT_READY && i2s_cfg->word_size != other_stream->cfg.word_size) {
		LOG_ERR("simultaneously configured streams have different word_size (%d) (%d)", i2s_cfg->word_size, other_stream->cfg.word_size);
		return -EINVAL;
	}

	/* --- configure the stream --- */


	retval = sm_atomic_set_stream_and_clk(dev, dir, is_controller);
	if (retval < 0) {
		return retval;
	}

	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);

	stream->dma_cfg.user_data = (void*) dev;
	stream->dma_cfg.dma_slot = RPI_PICO_DMA_DREQ_TO_SLOT(pio_get_dreq(pio, stream->sm, dir == I2S_DIR_TX));
	stream->dma_cfg.source_data_size = channel_length == 16 ? 2 : 4;
	stream->dma_cfg.dest_data_size = channel_length == 16 ? 2 : 4;

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	dev_data->channel_length = channel_length;


	pio_i2s_setup_stream(dev, stream, dir, is_controller);

	if (i2s_cfg_is_controller) {
		dev_data->sampling_freq = i2s_cfg->frame_clk_freq;
		pio_i2s_setup_clks(dev);
		pio_i2s_clks_start(dev);
	}

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

#if PIO_I2S_IS_DIR_EN(tx)
static void dma_tx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status) {
	const struct device *dev = (const struct device *)arg;
	const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	// uint dma_channel = data->tx.dma_channel;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);

	int retval;

	struct stream *stream = &data->tx;

	void *temp_mem_block = stream->mem_block;
	stream->mem_block = NULL;

	if (status < 0) {
		LOG_ERR("Something went wrong with DMA. status=%d", status);
		stream->state = I2S_STATE_ERROR;
		return;
	}

	// I2S_TRIGGER_STOP
	// I2S_TRIGGER_DRAIN
	if(stream->state == I2S_STATE_STOPPING && (stream->tx_stop_without_draining ||
	   k_msgq_num_used_get(stream->msgq) == 0)) {
		stream->state = I2S_STATE_READY;
		goto free_item;
	}

	struct queue_item item;
	size_t mem_block_size;
	int ret = k_msgq_get(stream->msgq, &item, SYS_TIMEOUT_MS(0));
	if (ret < 0) {
		LOG_ERR("TX buffer underrun.");
		stream->state = I2S_STATE_ERROR;
		goto free_item;
	}

	stream->mem_block = item.mem_block;
	mem_block_size = item.size;


	retval = reload_dma(stream->dev_dma, stream->dma_channel,
		&stream->dma_cfg,
		stream->mem_block,
		(void *)&pio->txf[stream->sm],
		mem_block_size);

	if (retval < 0) {
		LOG_ERR("Failed to start TX DMA transfer: %d", retval);
		stream->state = I2S_STATE_ERROR;
		goto free_item;
	}

free_item:
	k_mem_slab_free(stream->cfg.mem_slab, temp_mem_block);

}
#endif /* PIO_I2S_IS_DIR_EN(tx) */

#if PIO_I2S_IS_DIR_EN(rx)
static void dma_rx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status) {
	const struct device *dev = (const struct device *)arg;
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	// uint dma_channel = dev_data->rx.dma_channel;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);

	int retval;

	struct stream *stream = &dev_data->rx;

	if (status < 0) {
		LOG_ERR("Something went wrong with DMA. status=%d", status);
		stream->state = I2S_STATE_ERROR;
		return;
	}

	if (stream->state == I2S_STATE_ERROR) {
		return;
	}

	struct queue_item item = {.mem_block = stream->mem_block, .size = stream->cfg.block_size};
	stream->mem_block = NULL;

	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		goto put_item;
	}

	retval = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (retval < 0) {
		stream->state = I2S_STATE_ERROR;
		LOG_ERR("RX callback failed to allocate block");
		goto put_item;
	}

	retval = reload_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)&pio->rxf[stream->sm],
			stream->mem_block,
			stream->cfg.block_size);

	if (retval < 0) {
		LOG_ERR("Failed to start RX DMA transfer: %d", retval);
		stream->state = I2S_STATE_ERROR;
		goto put_item;
	}

put_item:
	retval = k_msgq_put(stream->msgq, &item, K_NO_WAIT);

	if (retval < 0) {
		LOG_ERR("RX overrun");
		stream->state = I2S_STATE_ERROR;
		return;
	}
}
#endif /* PIO_I2S_IS_DIR_EN(rx) */

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

static void i2s_reset_stream_sm(const struct device *dev, struct stream *stream)
{
	const struct pio_i2s_config *config = dev->config;
	struct pio_i2s_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);

	pio_sm_set_enabled(pio, stream->sm, false);
	pio_sm_clear_fifos(pio, stream->sm);
	pio_sm_restart(pio, stream->sm);

	pio_sm_exec(pio, stream->sm, pio_encode_set(pio_x, 0));
	pio_sm_exec(pio, stream->sm, pio_encode_set(pio_y, 0));
	pio_sm_exec(pio, stream->sm, pio_encode_jmp(data->target_prog.offset));
}

static int i2s_start_rx_stream_dma(const struct device *dev, struct stream *stream) {
	const struct pio_i2s_config *config = dev->config;
	// struct pio_i2s_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);

	// struct stream *stream = &data->tx;
	int retval;

	retval = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (retval < 0) {
		LOG_ERR("While starting rx stream dma, failed to allocate mem slab");
		return retval;
	}

	i2s_reset_stream_sm(dev, stream);

	retval = start_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)&pio->rxf[stream->sm],
			false, stream->mem_block,
			true, stream->cfg.block_size
	);

	if (retval < 0) {
		LOG_ERR("Failed to start RX DMA transfer: %d", retval);
		return retval;
	}

	pio_sm_set_enabled(pio, stream->sm, true);

	return 0;

}

static int i2s_start_tx_stream_dma(const struct device *dev, struct stream *stream) {
	const struct pio_i2s_config *config = dev->config;
	// struct pio_i2s_data *data = dev->data;
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

	i2s_reset_stream_sm(dev, stream);

	ret = start_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			stream->mem_block, true,
			(void *)&pio->txf[stream->sm],
			false,
			mem_block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}

	pio_sm_set_enabled(pio, stream->sm, true);

	return 0;

}

static int i2s_rpi_pico_trigger(const struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
	// const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	k_spinlock_key_t key;
	int ret = 0;

	if (dir != I2S_DIR_RX && dir != I2S_DIR_TX) {
		LOG_ERR("Unsupported trigger direction %d", dir);
		return -ENOSYS;
	}

	struct stream *stream = dir == I2S_DIR_RX ? &dev_data->rx : &dev_data->tx;

	if (!stream_is_present(stream)) {
		LOG_DBG("%s not enabled", dir == I2S_DIR_RX ? "RX" : "TX");
		return -EINVAL;
	}

	LOG_INF("i2s_rpi_pico_trigger dir=%d cmd=%d", dir, cmd);

	key = k_spin_lock(&dev_data->lock);

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d", stream->state);
			ret = -EIO;
			break;
		}

		if (dir == I2S_DIR_TX) {
			stream->tx_stop_without_draining = false;
			ret = i2s_start_tx_stream_dma(dev, stream);
		} else {
			ret = i2s_start_rx_stream_dma(dev, stream);
		}

		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			break;
		}

		stream->state = I2S_STATE_RUNNING;
		break;

	case I2S_TRIGGER_STOP:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP trigger: invalid state %d", stream->state);
			ret = -EIO;
			break;
		}

		stream->state = I2S_STATE_STOPPING;
		stream->tx_stop_without_draining = true;
		break;

	case I2S_TRIGGER_DRAIN:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("DRAIN trigger: invalid state %d", stream->state);
			ret = -EIO;
			break;
		}

		stream->state = I2S_STATE_STOPPING;
		break;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state %d", stream->state);
			ret = -EIO;
			break;
		}

		dma_stop(stream->dev_dma, stream->dma_channel);

		drop_stream(dev, stream);
		stream->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state %d", stream->state);
			ret = -EIO;
			break;
		}

		dma_stop(stream->dev_dma, stream->dma_channel);

		drop_stream(dev, stream);
		stream->state = I2S_STATE_READY;
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		ret = -EINVAL;
	}

	k_spin_unlock(&dev_data->lock, key);

	return ret;
}

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

#define PIO_I2S_BCLK_PIN(idx)    DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, clks, 0)
#define PIO_I2S_WS_PIN(idx)      DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, ws, 0)
#define PIO_I2S_TX_DATA_PIN(idx) DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_data, 0)
#define PIO_I2S_RX_DATA_PIN(idx) DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, rx_data, 0)

#define PIO_I2S_HAS_GROUP(idx, group)                                                              \
	DT_NODE_EXISTS(DT_CHILD(DT_PINCTRL_BY_NAME(DT_DRV_INST(idx), default, 0), group))

#define PIO_I2S_HAS_TX(idx) PIO_I2S_IS_DIR_INST_EN(idx, tx)
#define PIO_I2S_HAS_RX(idx) PIO_I2S_IS_DIR_INST_EN(idx, rx)

#define PIO_I2S_INIT(idx)                                                                          \
	BUILD_ASSERT(PIO_I2S_HAS_TX(idx) || PIO_I2S_HAS_RX(idx),                                   \
		     "I2S node needs at least one of the \"tx\" / \"rx\" dma-names.");             \
	BUILD_ASSERT(!PIO_I2S_HAS_TX(idx) || PIO_I2S_HAS_GROUP(idx, tx_data),                      \
		     "I2S tx_data pins not defined.");                                             \
	BUILD_ASSERT(!PIO_I2S_HAS_RX(idx) || PIO_I2S_HAS_GROUP(idx, rx_data),                      \
		     "I2S rx_data pins not defined.");                                             \
	BUILD_ASSERT(PIO_I2S_WS_PIN(idx) == PIO_I2S_BCLK_PIN(idx) + 1,                             \
		     "I2S ws pin must be equal to bit-clock pin + 1 "                              \
		     "due to limitations in the PIO program.");                                    \
	IF_ENABLED(PIO_I2S_HAS_RX(idx),                                                            \
		(BUILD_ASSERT(PIO_I2S_RX_DATA_PIN(idx) == PIO_I2S_BCLK_PIN(idx) - 1,               \
			      "I2S rx_data pin must be equal to bit-clock pin - 1 "                \
			      "due to limitations in the PIO program.");))                         \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	static const struct pio_i2s_config pio_i2s##idx##_config = {                               \
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(idx)),                                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		.clock_pin = PIO_I2S_BCLK_PIN(idx),                                                \
		.ws_pin = PIO_I2S_WS_PIN(idx),                                                     \
		.in_base_pin = PIO_I2S_RX_DATA_PIN(idx)                                            \
	};                                                                                         \
	IF_ENABLED(PIO_I2S_HAS_TX(idx),                                                            \
		(K_MSGQ_DEFINE(tx_##idx##_queue, sizeof(struct queue_item),                        \
			       CONFIG_I2S_RPI_PICO_PIO_TX_QUEUE_SIZE, 1);))                        \
	IF_ENABLED(PIO_I2S_HAS_RX(idx),                                                            \
		(K_MSGQ_DEFINE(rx_##idx##_queue, sizeof(struct queue_item),                        \
			       CONFIG_I2S_RPI_PICO_PIO_RX_QUEUE_SIZE, 1);))                        \
	static struct pio_i2s_data pio_i2s##idx##_data = {                                         \
        .tx = {                                                                                    \
		.msgq = COND_CODE_1(PIO_I2S_HAS_TX(idx), (&tx_##idx##_queue), (NULL)),             \
		.state = I2S_STATE_NOT_READY,                                                      \
		.tx_stop_without_draining = false,                                                 \
		.dev_dma = UTIL_AND(DT_INST_DMAS_HAS_NAME(idx, tx),                                \
				    DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, tx))),            \
		.dma_channel = UTIL_AND(DT_INST_DMAS_HAS_NAME(idx, tx),                            \
					DT_INST_DMAS_CELL_BY_NAME(idx, tx, channel)),              \
		.data_pin = COND_CODE_1(PIO_I2S_HAS_TX(idx), (PIO_I2S_TX_DATA_PIN(idx)), (0)),     \
		.dma_cfg = {                                                                       \
			.block_count = 1,                                                          \
			.channel_direction = MEMORY_TO_PERIPHERAL,                                 \
			.source_burst_length = 1,                                                  \
			.dest_burst_length = 1,                                                    \
			.channel_priority = 1,                                                     \
			.dma_callback = COND_CODE_1(PIO_I2S_HAS_TX(idx), (dma_tx_callback), (NULL))\
		},                                                                                 \
		.sm = (size_t)-1,                                                                  \
        },                                                                                         \
        .rx = {                                                                                    \
		.msgq = COND_CODE_1(PIO_I2S_HAS_RX(idx), (&rx_##idx##_queue), (NULL)),             \
		.state = I2S_STATE_NOT_READY,                                                      \
		.tx_stop_without_draining = false,                                                 \
		.dev_dma = UTIL_AND(DT_INST_DMAS_HAS_NAME(idx, rx),                                \
				    DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, rx))),            \
		.dma_channel = UTIL_AND(DT_INST_DMAS_HAS_NAME(idx, rx),                            \
					DT_INST_DMAS_CELL_BY_NAME(idx, rx, channel)),              \
		.data_pin = COND_CODE_1(PIO_I2S_HAS_RX(idx), (PIO_I2S_RX_DATA_PIN(idx)), (0)),     \
		.dma_cfg = {                                                                       \
			.block_count = 1,                                                          \
			.channel_direction = PERIPHERAL_TO_MEMORY,                                 \
			.source_burst_length = 1,                                                  \
			.dest_burst_length = 1,                                                    \
			.channel_priority = 1,                                                     \
			.dma_callback = COND_CODE_1(PIO_I2S_HAS_RX(idx), (dma_rx_callback), (NULL))\
		},                                                                                 \
		.sm = (size_t)-1,                                                                  \
        },                                                                                         \
        .clks_sm = (size_t)-1                                                                      \
    };                                                                                             \
	DEVICE_DT_INST_DEFINE(idx, pio_i2s_init, NULL, &pio_i2s##idx##_data,                       \
			      &pio_i2s##idx##_config, POST_KERNEL,                                 \
			      CONFIG_I2S_INIT_PRIORITY,                                            \
			      &i2s_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_I2S_INIT)
