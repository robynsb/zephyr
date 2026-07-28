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

// TODO: Write raspberry pi specific tests for failing to allocate multiple i2s periperals on the same pio and stuff like that.
//       And check that the state machines are correctly deallocated and stuff like that...
// TODO: write test for the sampling frequency check works
// TODO: check for code smell involving functions with only one call site.
// TODO: claude claims: RX doesn't stop when told, if TX is mid-drain. dma_rx_callback gates its stop path on
//       dev_data->tx.state != I2S_STATE_STOPPING (i2s_rpi_pico_pio.c:805). After TX DRAIN + RX STOP, RX
//       keeps allocating and capturing until the TX drain completes. With a slab sized to the test's
//       exact needs, that's what exhausted it. - WARNINGS?!?!

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
	const uint32_t clock_pin; /* BCLK */
	const uint32_t ws_pin;    /* word select; independent of BCLK */
};

struct pio_sm_res {
	size_t sm;                 /* (size_t)-1 = not claimed */
	const pio_program_t *prog; /* NULL = not loaded */
	uint32_t offset;
};

struct stream {
	enum i2s_state state;
	bool tx_stop_without_draining;
	struct k_msgq *msgq;
	uint32_t dma_channel;
	const struct device *dev_dma;
	struct dma_config dma_cfg;
	struct k_spinlock lock;

	struct i2s_config cfg;
	void *mem_block;

	const uint32_t data_pin;

	struct pio_sm_res res;
};

struct pio_i2s_data {
    struct stream tx;
    struct stream rx;
    uint32_t channel_length;
    uint32_t sampling_freq;
    struct pio_sm_res clks_res;
};



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
	int retval = k_msgq_get(stream->msgq, &item, (state == I2S_STATE_ERROR) ? K_NO_WAIT : K_MSEC(stream->cfg.timeout));

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

/* TX clock-follower: waits on BCLK (in base + 0), `jmp pin` on WS, `out pins, 1` -> tx_data.
 * `pull block` sits at the wrap target (not inside left_loop), so each channel consumes
 * exactly one FIFO word with autopull off. */
RPI_PICO_PIO_DEFINE_PROGRAM(tx_target, 0, 10,
		//     .wrap_target
	0x80a0, //  0: pull   block
	0x2020, //  1: wait   0 pin, 0   (left_loop)
	0x6001, //  2: out    pins, 1
	0x20a0, //  3: wait   1 pin, 0
	0x00c6, //  4: jmp    pin, 6
	0x0001, //  5: jmp    1
	0x80c0, //  6: pull   ifempty noblock
	0x2020, //  7: wait   0 pin, 0
	0x6001, //  8: out    pins, 1
	0x20a0, //  9: wait   1 pin, 0
	0x00c7, // 10: jmp    pin, 7
	        //     .wrap
);

/* RX clock-follower: `in pins, 1` reads data (in base + 0); waits on BCLK (in base + 1);
 * `jmp pin` on WS. Requires rx_data == BCLK - 1 in the overlay. */
RPI_PICO_PIO_DEFINE_PROGRAM(rx_target, 0, 10,
		//     .wrap_target
	0x2021, //  0: wait   0 pin, 1
	0x20a1, //  1: wait   1 pin, 1
	0x4001, //  2: in     pins, 1
	0x00c5, //  3: jmp    pin, 5
	0x0000, //  4: jmp    0
	0x8040, //  5: push   iffull noblock
	0x2021, //  6: wait   0 pin, 1
	0x20a1, //  7: wait   1 pin, 1
	0x4001, //  8: in     pins, 1
	0x00c6, //  9: jmp    pin, 6
	0x8020, // 10: push   block
	        //     .wrap
);

static const uint32_t clks_cycles_factor = 2u; /* k=2: 2 PIO cycles per BCLK period */
static const uint32_t clks_entry_point = 0;

// TODO: delete sm_res_init?
static int sm_res_init(const struct device *piodev, struct pio_sm_res *res,
		       const pio_program_t *prog)
{
	int retval;

	__ASSERT_NO_MSG(res->sm == (size_t)-1);
	__ASSERT_NO_MSG(res->prog == NULL);

	PIO pio = pio_rpi_pico_get_pio(piodev);

	if (!pio_can_add_program(pio, prog)) {
		LOG_ERR("no PIO instruction memory left for program");
		return -EBUSY;
	}

	retval = pio_rpi_pico_allocate_sm(piodev, &res->sm);
	if(retval < 0) {
		return retval;
	}

	pio_sm_set_enabled(pio, res->sm, false);


	res->offset = pio_add_program(pio, prog);
	res->prog = prog;
	return 0;
}

static void sm_res_release(const struct device *piodev, struct pio_sm_res *res,
			   uint32_t out_pins)
{
	PIO pio = pio_rpi_pico_get_pio(piodev);

	if (res->prog != NULL) {
		pio_remove_program(pio, res->prog, res->offset);
		res->prog = NULL;
	}

	if (res->sm != (size_t)-1) {
		pio_sm_set_enabled(pio, res->sm, false);

		if (out_pins != 0) {
			pio_sm_set_pindirs_with_mask(pio, res->sm, 0, out_pins);
		}

		pio_sm_unclaim(pio, res->sm);
		res->sm = (size_t)-1;
	}
}

/*
 * Atomically claim 2/3 state machines.
 * TODO: Think about a different name.
 */

static int sm_res_claim_dir(const struct device *dev, enum i2s_dir dir, bool need_clk_sm)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	const struct device *piodev = dev_config->piodev;
	// PIO pio = pio_rpi_pico_get_pio(piodev);

	__ASSERT_NO_MSG(dir != I2S_DIR_BOTH);

	int retval;
	bool free_clk_sm_during_error = false;

	if (need_clk_sm && dev_data->clks_res.sm == (size_t)-1) {
		retval = sm_res_init(piodev, &dev_data->clks_res, RPI_PICO_PIO_GET_PROGRAM(clks));
		if (retval < 0) {
			return -EBUSY;
		}

		free_clk_sm_during_error = true;
	}

	const pio_program_t *prog;
	struct pio_sm_res *res;
	if (dir == I2S_DIR_TX) {
		res = &dev_data->tx.res;
		prog = RPI_PICO_PIO_GET_PROGRAM(tx_target);
	} else {
		res = &dev_data->rx.res;
		prog = RPI_PICO_PIO_GET_PROGRAM(rx_target);
	}

	if (res->sm == (size_t)-1) {
		retval = sm_res_init(piodev, res, prog);
		if (retval < 0) {
			goto cleanup;
		}
	}

	if (!need_clk_sm && dev_data->clks_res.sm != (size_t)-1) {
		sm_res_release(dev_config->piodev, &dev_data->clks_res,
			       (1u << dev_config->clock_pin) |
			       (1u << dev_config->ws_pin));
	}

	return 0;

cleanup:
	if(free_clk_sm_during_error) {
		sm_res_release(piodev, &dev_data->clks_res, 0);
	}
	return -EBUSY;
}

/*
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


/*
 * Set up the clks SM: load the clock generator program, configure BCLK on the
 * 1-bit sideset and WS on the out/in base, and program the clock divider from
 * the shared sampling parameters. Started separately by pio_i2s_clks_start().
 */
static void pio_i2s_setup_clks(const struct device *dev)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	uint32_t bclk_pin = dev_config->clock_pin;
	uint32_t ws_pin = dev_config->ws_pin;
	struct pio_sm_res *res = &dev_data->clks_res;
	pio_sm_config c;
	// int retval;

	c = pio_get_default_sm_config();
	sm_config_set_wrap(&c, res->offset + clks_wrap_target, res->offset + clks_wrap);
	sm_config_set_sideset_pin_base(&c, bclk_pin);
	sm_config_set_sideset(&c, 1, false, false);
	sm_config_set_out_pins(&c, ws_pin, 1);
	sm_config_set_in_pins(&c, ws_pin);
	sm_config_set_in_pin_count(&c, 1);
	pio_sm_init(pio, res->sm, res->offset, &c);

	/* clks drives BCLK + WS; they are independent pins, so set both bits. */
	uint32_t pin_mask = (1u << bclk_pin) | (1u << ws_pin);
	pio_sm_set_pins_with_mask(pio, res->sm, 0, pin_mask); /* clear pins */
	pio_sm_set_pindirs_with_mask(pio, res->sm, pin_mask, pin_mask);

	uint32_t sample_freq = dev_data->sampling_freq;
	uint32_t channel_length = dev_data->channel_length;

	uint64_t divider = calculate_divider_shift_8(sample_freq, channel_length);
	uint64_t div_int = divider >> 8u;
	uint64_t div_frac = divider & 0xffu;

	__ASSERT_NO_MSG(div_int != 0);
	__ASSERT_NO_MSG(div_int <= UINT16_MAX);

	pio_sm_set_clkdiv_int_frac(pio, res->sm, div_int, div_frac);

	return;
}

static void pio_i2s_setup_stream(const struct device *dev, struct stream *stream,
				enum i2s_dir dir, bool is_controller)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	uint32_t channel_length = dev_data->channel_length;
	uint32_t bclk_pin = dev_config->clock_pin;
	uint32_t ws_pin = dev_config->ws_pin;
	struct pio_sm_res *res = &stream->res;
	pio_sm_config c;
	// int retval;

	if (dir == I2S_DIR_TX) {
		uint32_t tx_out_pin = stream->data_pin;

		c = pio_get_default_sm_config();
		sm_config_set_wrap(&c, res->offset + tx_target_wrap_target,
				   res->offset + tx_target_wrap);
		sm_config_set_out_pins(&c, tx_out_pin, 1);
		sm_config_set_in_pins(&c, bclk_pin);
		sm_config_set_in_pin_count(&c, 1);
		sm_config_set_jmp_pin(&c, ws_pin);
		sm_config_set_out_shift(&c, false, false, channel_length == 16 ? 32 : 1);
		sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
		pio_sm_init(pio, res->sm, res->offset, &c);
		/* Followers run as fast as possible; they gate on the clock pins. */
		pio_sm_set_clkdiv_int_frac(pio, res->sm, 1, 0);
		pio_sm_set_pins_with_mask(pio, res->sm, 0, 1u << tx_out_pin);
		pio_sm_set_pindirs_with_mask(pio, res->sm, 1u << tx_out_pin, 1u << tx_out_pin);
	} else {
		/* in base = rx_data (data at +0, BCLK at +1), jmp pin = WS */
		c = pio_get_default_sm_config();
		sm_config_set_wrap(&c, res->offset + rx_target_wrap_target,
				   res->offset + rx_target_wrap);
		sm_config_set_in_pins(&c, stream->data_pin);
		sm_config_set_in_pin_count(&c, 2); /* data at +0, BCLK at +1 */
		sm_config_set_jmp_pin(&c, ws_pin);
		sm_config_set_in_shift(&c, false, false, channel_length == 16 ? 32 : 1);
		sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
		pio_sm_init(pio, res->sm, res->offset, &c);
		pio_sm_set_clkdiv_int_frac(pio, res->sm, 1, 0);
		if (dev_data->tx.data_pin != dev_data->rx.data_pin) {
			/* rx_data stays an input; in loopback it is the TX out pin
			 * and driven by the TX follower instead. */
			pio_sm_set_pindirs_with_mask(pio, res->sm, 0, 1u << stream->data_pin);
		}
	}

	if (!is_controller) {
		uint32_t clk_pins = (1u << bclk_pin) | (1u << ws_pin);

		pio_sm_set_pindirs_with_mask(pio, res->sm, 0, clk_pins);
	}

	return;
}

static void pio_i2s_clks_start(const struct device *dev)
{
	const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);
	uint32_t channel_length = dev_data->channel_length;
	struct pio_sm_res *res = &dev_data->clks_res;

	/* The clks SM is the I2S controller (BCLK/WS): started here, when it is
	 * (re)configured, and then left free-running — stream START/STOP never
	 * stops or restarts it, and the followers are never touched here. Each
	 * TX/RX follower is instead reset and aligned to this free-running clock
	 * at its own START; starting one stream never disturbs the clks SM or
	 * the other stream's follower. */
	pio_sm_set_enabled(pio, res->sm, false);
	pio_sm_exec(pio, res->sm, pio_encode_set(pio_y, channel_length - 2));
	pio_sm_exec(pio, res->sm, pio_encode_jmp(res->offset + clks_entry_point));
	pio_sm_set_enabled(pio, res->sm, true);
}

static void drop_queue(struct stream *stream) {
	struct queue_item item;
	while (k_msgq_get(stream->msgq, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, item.mem_block);
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

	bool other_is_controller = other_stream->state != I2S_STATE_NOT_READY &&
				!(other_stream->cfg.options &
				  (I2S_OPT_BIT_CLK_TARGET | I2S_OPT_FRAME_CLK_TARGET));


	/* --- config check --- */

	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		LOG_ERR("stream in invalid state (%d)", stream->state);
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		drop_queue(stream);

		sm_res_release(dev_config->piodev, &stream->res,
			       dir == I2S_DIR_TX ? (1u << stream->data_pin) : 0);
		memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;

		if (!other_is_controller) {
			/* clks drives BCLK + WS (see pio_i2s_setup_clks). */
			sm_res_release(dev_config->piodev, &dev_data->clks_res,
				       (1u << dev_config->clock_pin) |
				       (1u << dev_config->ws_pin));
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


	retval = sm_res_claim_dir(dev, dir, is_controller);
	if (retval < 0) {
		return retval;
	}

	PIO pio = pio_rpi_pico_get_pio(dev_config->piodev);

	stream->dma_cfg.user_data = (void*) dev;
	// TODO: think about this dma_slot and the one in the overlay.
	stream->dma_cfg.dma_slot = RPI_PICO_DMA_DREQ_TO_SLOT(pio_get_dreq(pio, stream->res.sm, dir == I2S_DIR_TX));
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
		(void *)&pio->txf[data->tx.res.sm],
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

	struct queue_item item = {.mem_block = stream->mem_block, .size = stream->cfg.block_size};

	retval = k_msgq_put(stream->msgq, &item, K_NO_WAIT);
	if (retval < 0) {
		LOG_ERR("RX overrun");
		stream->state = I2S_STATE_ERROR;
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		return;
	}

	stream->mem_block = NULL;

	if(stream->state == I2S_STATE_STOPPING && dev_data->tx.state != I2S_STATE_STOPPING) {
		retval = dma_stop(stream->dev_dma, stream->dma_channel);
		if(retval < 0) {
			stream->state = I2S_STATE_ERROR;
			return;
		}

		stream->state = I2S_STATE_READY;

		return;
	}

	/* Prepare to receive the next data block */
	retval = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (retval < 0) {
		stream->state = I2S_STATE_ERROR;
		//TODO: think about when does this trigger? Is the queue sized such that in correct operation this never happens
		LOG_ERR("RX callback failed to allocate block");
		return;
	}

	retval = reload_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)&pio->rxf[dev_data->rx.res.sm],
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
		LOG_ERR("While starting rx stream dma, failed to allocate mem slab");
		return retval;
	}

	pio_sm_set_enabled(pio, stream->res.sm, false);
	pio_sm_clear_fifos(pio, stream->res.sm);
	pio_sm_restart(pio, stream->res.sm);
	pio_sm_exec(pio, stream->res.sm, pio_encode_jmp(stream->res.offset));
	pio_sm_set_enabled(pio, stream->res.sm, true);

	uint32_t priming_words = data->channel_length == 32 ? 2 : 1;
	// TODO: think about putting a ISR lock on this startup code.
	for (uint32_t i = 0; i < priming_words; i++) {
		pio_sm_get_blocking(pio, stream->res.sm);
	}

	retval = start_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)&pio->rxf[stream->res.sm],
			false, stream->mem_block,
			true, stream->cfg.block_size
	);

	if (retval < 0) {
		LOG_ERR("Failed to start RX DMA transfer: %d", retval);
		return retval;
	}

	return 0;

}

int i2s_start_tx_stream_dma(const struct device *dev, struct stream *stream) {
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

	pio_sm_set_enabled(pio, stream->res.sm, false);
	pio_sm_restart(pio, stream->res.sm);
	pio_sm_clear_fifos(pio, stream->res.sm);
	pio_sm_exec(pio, stream->res.sm, pio_encode_jmp(stream->res.offset));

	/* Stage one frame of silence ahead of the real data: once enabled the
	 * follower may start mid-frame, and it consumes this silent word while its
	 * `jmp pin` WS checks re-sync it to the next frame boundary. The first real
	 * sample then shifts cleanly on the left channel (and any one-BCLK startup
	 * skew lands on silence, 0 << 1 == 0). A 32-bit channel is one word per slot
	 * (two per frame); a 16-bit channel packs both slots into a single word. */
	uint32_t priming_words = data->channel_length == 32 ? 2 : 1;

	for (uint32_t i = 0; i < priming_words; i++) {
		pio_sm_put_blocking(pio, stream->res.sm, 0);
	}

	ret = start_dma(stream->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			stream->mem_block, true,
			(void *)&pio->txf[stream->res.sm],
			false,
			mem_block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}

	/* Data staged: enable just this follower. Its first pull is the priming
	 * word; the clocks SM and the RX follower are left untouched. */
	pio_sm_set_enabled(pio, stream->res.sm, true);

	return 0;

}

static int i2s_start_stream_tx(const struct device *dev, struct stream *stream) {
	if (stream->state != I2S_STATE_READY) {
		LOG_ERR("START trigger: invalid state %d",
			    stream->state);
		return -EIO;
	}
	stream->tx_stop_without_draining = false;
	int retval = i2s_start_tx_stream_dma(dev, stream);
	if (retval < 0) {
		LOG_ERR("START TX trigger failed %d", retval);
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
	__ASSERT_NO_MSG(stream->mem_block == NULL);

        int retval = i2s_start_rx_stream_dma(dev, stream);
	if (retval < 0) {
		LOG_ERR("START RX trigger failed %d", retval);
		return retval;
	}
	// pio_i2s_controller_start(dev);
	stream->state = I2S_STATE_RUNNING;
	return 0;
}

static int i2s_drop_stream(const struct device *dev, struct stream *stream) {
	k_spinlock_key_t key = k_spin_lock(&stream->lock);
	if (stream->state == I2S_STATE_NOT_READY) {
		k_spin_unlock(&stream->lock, key);
		LOG_ERR("DROP trigger: invalid state %d",
		         stream->state);
		return -EIO;
	}
	(void) dma_stop(stream->dev_dma, stream->dma_channel);
	drop_queue(stream);
	stream->state = I2S_STATE_READY;
	if(stream->mem_block != NULL) { // TODO: Is this free necessary? ESP32 doesn't do it. Are they wrong?
		LOG_INF("freeing inflight thing??");
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
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
	drop_queue(stream);
	stream->state = I2S_STATE_READY;
	k_spin_unlock(&stream->lock, key);
	return 0;
}

static int i2s_rpi_pico_trigger(const struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
	// const struct pio_i2s_config *dev_config = dev->config;
	struct pio_i2s_data *dev_data = dev->data;
	int ret;

	if (dir != I2S_DIR_RX && dir != I2S_DIR_TX) {
		LOG_ERR("Unsupported trigger direction %d", dir);
		return -ENOSYS;
	}

	struct stream *stream = dir == I2S_DIR_RX ? &dev_data->rx : &dev_data->tx;
	LOG_INF("i2s_rpi_pico_trigger dir=%d cmd=%d", dir, cmd);

	switch (cmd) {
	case I2S_TRIGGER_START:
		if(stream->state != I2S_STATE_READY) {
			LOG_ERR("Stream state must be in ready state to start stream.");
			return -EIO;
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
		break;
	case I2S_TRIGGER_STOP:
		//TODO: what if DMA is not running?
		ret = i2s_stop_stream(dev, stream);
		if (ret < 0) {
			return ret;
		}
		break;
	case I2S_TRIGGER_DRAIN:
		//TODO: what if queue already empty?
		ret = i2s_drain_stream(dev, stream);
		if (ret < 0) {
			return ret;
		}
		break;
	case I2S_TRIGGER_DROP:
		ret = i2s_drop_stream(dev, stream);
		if (ret < 0) {
			return ret;
		}
		break;
	case I2S_TRIGGER_PREPARE:
		ret = i2s_drain_prepare(dev, stream);
		if (ret < 0) {
			return ret;
		}
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
 *  BCLK (clks group), WS (ws group) and the data pins are independent in the overlay.
 *  The one hardware constraint that remains is rx_data == BCLK - 1, because rx_target
 *  reads data at in_base+0 and BCLK at in_base+1 off the same input base. This asserts
 *  the overlay honours the adjacency rather than failing silently at runtime.
 *  Add a build assert such as possibly this:
 *  BUILD_ASSERT(PIO_I2S_RX_DATA_PIN(idx) == PIO_I2S_BCLK_PIN(idx) - 1,                  \
         "I2S rx_data pin must be bit-clock pin - 1 "                                \
         "(rx_target reads data at in_base+0 and BCLK at in_base+1); fix the overlay");\
 */
#define PIO_I2S_INIT(idx)									\
	PINCTRL_DT_INST_DEFINE(idx);								\
	static const struct pio_i2s_config pio_i2s##idx##_config = {				\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(idx)),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),					\
		.clock_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, clks, 0),	\
		.ws_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, ws, 0)	\
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
		.res = {.sm = (size_t)-1, .prog = NULL},        \
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
		.res = {.sm = (size_t)-1, .prog = NULL},        \
        },                                             \
        .clks_res = {.sm = (size_t)-1, .prog = NULL}                       \
    };					\
	DEVICE_DT_INST_DEFINE(idx, pio_i2s_init, NULL, &pio_i2s##idx##_data,			\
			      &pio_i2s##idx##_config, POST_KERNEL,				\
			      CONFIG_I2S_INIT_PRIORITY,					\
			      &i2s_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_I2S_INIT)
