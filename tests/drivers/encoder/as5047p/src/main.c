/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as5047p

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <drivers/encoder.h>
#include <drivers/encoder/as5047p.h>

#define REG_NOP      0x0000U
#define REG_ERRFL    0x0001U
#define REG_ZPOSM    0x0016U
#define REG_ZPOSL    0x0017U
#define REG_DIAAGC   0x3FFCU
#define REG_ANGLECOM 0x3FFFU

#define FRAME_RW     BIT(14)
#define FRAME_EF     BIT(14)
#define FRAME_PARITY BIT(15)
#define DATA_MASK    GENMASK(13, 0)
#define ANGLE_MASK   GENMASK(13, 0)

/* Error flag enables in the upper bits of ZPOSL, which a change of the zero
 * position must leave alone. */
#define ZPOSL_KEEP GENMASK(7, 6)

enum inject {
	INJECT_NONE,
	INJECT_SPI,
	INJECT_PARITY,
	INJECT_FLAG,
};

struct as5047p_emul_data {
	/* Angle before the zero position is subtracted. */
	uint16_t raw_angle;
	uint16_t zposm;
	uint16_t zposl;
	uint16_t diaagc;
	uint16_t errfl;
	uint16_t pending_response;
	uint16_t pending_write_addr;
	bool write_pending;
	/* Applied to the next inject_count reads of ANGLECOM. */
	enum inject inject;
	int inject_count;
	bool inject_forever;
};

static bool even_parity(uint16_t frame)
{
	return (__builtin_popcount(frame) & 1) == 0;
}

static uint16_t with_parity(uint16_t frame)
{
	return even_parity(frame) ? frame : (frame | FRAME_PARITY);
}

static uint16_t emul_read_reg(struct as5047p_emul_data *data, uint16_t reg)
{
	uint16_t zpos;
	uint16_t value;

	switch (reg) {
	case REG_ERRFL:
		value = data->errfl;
		data->errfl = 0;
		return value;
	case REG_ZPOSM:
		return data->zposm;
	case REG_ZPOSL:
		return data->zposl;
	case REG_DIAAGC:
		return data->diaagc;
	case REG_ANGLECOM:
		zpos = ((data->zposm & 0xffU) << 6) | (data->zposl & 0x3fU);
		return (data->raw_angle - zpos) & ANGLE_MASK;
	default:
		return 0;
	}
}

static void emul_write_reg(struct as5047p_emul_data *data, uint16_t reg, uint16_t value)
{
	switch (reg) {
	case REG_ZPOSM:
		data->zposm = value & 0xffU;
		break;
	case REG_ZPOSL:
		data->zposl = value & 0xffU;
		break;
	default:
		break;
	}
}

static bool take_injection(struct as5047p_emul_data *data, enum inject *kind)
{
	if (data->inject == INJECT_NONE) {
		return false;
	}

	*kind = data->inject;

	if (!data->inject_forever && --data->inject_count <= 0) {
		data->inject = INJECT_NONE;
	}

	return true;
}

static int as5047p_emul_io(const struct emul *target, const struct spi_config *config,
			   const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct as5047p_emul_data *data = target->data;
	const uint8_t *tx;
	uint8_t *rx;
	uint16_t frame;
	uint16_t payload;
	uint16_t next = 0;
	bool ef = false;
	bool corrupt = false;
	enum inject kind;

	if ((SPI_WORD_SIZE_GET(config->operation) != 8U) ||
	    ((config->operation & SPI_MODE_CPHA) == 0U) ||
	    ((config->operation & SPI_MODE_CPOL) != 0U)) {
		return -EIO;
	}

	if (tx_bufs == NULL || rx_bufs == NULL || tx_bufs->count != 1 || rx_bufs->count != 1 ||
	    tx_bufs->buffers[0].len != 2 || rx_bufs->buffers[0].len != 2) {
		return -EIO;
	}

	tx = tx_bufs->buffers[0].buf;
	rx = rx_bufs->buffers[0].buf;
	frame = ((uint16_t)tx[0] << 8) | tx[1];

	if (!even_parity(frame)) {
		data->write_pending = false;
		data->errfl |= AS5047P_ERRFL_PARERR;
		ef = true;
	} else if (data->write_pending) {
		emul_write_reg(data, data->pending_write_addr, frame & DATA_MASK);
		data->write_pending = false;
		next = emul_read_reg(data, data->pending_write_addr);
	} else if ((frame & FRAME_RW) != 0U) {
		payload = frame & DATA_MASK;

		if (payload == REG_ANGLECOM && take_injection(data, &kind)) {
			if (kind == INJECT_SPI) {
				return -EIO;
			} else if (kind == INJECT_PARITY) {
				corrupt = true;
			} else {
				data->errfl |= AS5047P_ERRFL_FRERR;
				ef = true;
			}
		}

		next = emul_read_reg(data, payload);
	} else {
		data->pending_write_addr = frame & DATA_MASK;
		data->write_pending = true;
	}

	/* The response to this frame was decided by the previous one. */
	rx[0] = data->pending_response >> 8;
	rx[1] = data->pending_response & 0xffU;

	data->pending_response = with_parity((next & DATA_MASK) | (ef ? FRAME_EF : 0U));
	if (corrupt) {
		data->pending_response ^= BIT(0);
	}

	return 0;
}

static int as5047p_emul_init(const struct emul *target, const struct device *parent)
{
	struct as5047p_emul_data *data = target->data;

	ARG_UNUSED(parent);
	memset(data, 0, sizeof(*data));
	data->pending_response = with_parity(0);
	data->zposl = ZPOSL_KEEP;

	return 0;
}

static const struct spi_emul_api as5047p_emul_api = {
	.io = as5047p_emul_io,
};

#define AS5047P_EMUL_DEFINE(inst)                                                                  \
	static struct as5047p_emul_data as5047p_emul_data_##inst;                                  \
	EMUL_DT_INST_DEFINE(inst, as5047p_emul_init, &as5047p_emul_data_##inst, NULL,              \
			    &as5047p_emul_api, NULL);

DT_INST_FOREACH_STATUS_OKAY(AS5047P_EMUL_DEFINE)

static const struct device *const enc0 = DEVICE_DT_GET(DT_NODELABEL(enc0));
static const struct device *const enc1 = DEVICE_DT_GET(DT_NODELABEL(enc1));

static struct as5047p_emul_data *emul0(void)
{
	return EMUL_DT_GET(DT_NODELABEL(enc0))->data;
}

static struct as5047p_emul_data *emul1(void)
{
	return EMUL_DT_GET(DT_NODELABEL(enc1))->data;
}

/* The poll threads only run while the test thread sleeps. Three intervals
 * leave room for one poll to land on either side of the boundary. */
static void let_polls_run(void)
{
	k_sleep(K_MSEC(3));
}

static void inject(struct as5047p_emul_data *data, enum inject kind, int count)
{
	data->inject_count = count;
	data->inject_forever = false;
	data->inject = kind;
}

static int get(const struct device *dev, struct encoder_feedback *fb)
{
	memset(fb, 0, sizeof(*fb));
	return encoder_get_feedback(dev, fb);
}

static uint32_t total_errors(const struct as5047p_stats *stats)
{
	uint32_t sum = 0;

	for (int i = 0; i < AS5047P_ERROR_CAUSE_COUNT; i++) {
		sum += stats->errors[i];
	}

	return sum;
}

ZTEST(encoder_as5047p, test_devices_ready)
{
	zassert_true(device_is_ready(enc0));
	zassert_true(device_is_ready(enc1));
}

ZTEST(encoder_as5047p, test_resolution_is_14_bits)
{
	uint8_t resolution = 0;

	zassert_ok(encoder_get_resolution(enc0, &resolution));
	zassert_equal(resolution, AS5047P_ANGLE_BITS);
}

ZTEST(encoder_as5047p, test_position_starts_at_the_absolute_angle)
{
	struct encoder_feedback fb;

	emul0()->raw_angle = 1234;
	zassert_ok(encoder_reset(enc0));
	let_polls_run();

	zassert_ok(get(enc0, &fb));
	zassert_equal(fb.position, 1234);
	zassert_equal(fb.single_turn, 1234);
	zassert_true(fb.online);
	zassert_false(fb.stale);
	zassert_true((fb.valid_mask & ENCODER_FEEDBACK_SINGLE_TURN) != 0U);
}

ZTEST(encoder_as5047p, test_position_continues_across_the_wrap)
{
	struct encoder_feedback before;
	struct encoder_feedback after;

	emul0()->raw_angle = 16000;
	let_polls_run();
	zassert_ok(get(enc0, &before));

	emul0()->raw_angle = 100;
	let_polls_run();
	zassert_ok(get(enc0, &after));

	zassert_equal(after.position - before.position, 16384 - 16000 + 100);
	zassert_equal(after.position_epoch, before.position_epoch);
	zassert_true((after.valid_mask & ENCODER_FEEDBACK_VELOCITY) != 0U);
}

ZTEST(encoder_as5047p, test_invert_direction_flips_position_but_not_single_turn)
{
	struct encoder_feedback before;
	struct encoder_feedback after;

	emul1()->raw_angle = 1000;
	zassert_ok(encoder_reset(enc1));
	let_polls_run();
	zassert_ok(get(enc1, &before));
	zassert_equal(before.position, -1000);

	emul1()->raw_angle = 1100;
	let_polls_run();
	zassert_ok(get(enc1, &after));

	zassert_equal(after.position - before.position, -100);
	zassert_equal(after.single_turn, 1100);
}

ZTEST(encoder_as5047p, test_set_position_offsets_without_new_epoch)
{
	struct encoder_feedback before;
	struct encoder_feedback after;

	emul0()->raw_angle = 500;
	let_polls_run();
	zassert_ok(get(enc0, &before));

	zassert_ok(encoder_set_position(enc0, 0));
	emul0()->raw_angle = 600;
	let_polls_run();
	zassert_ok(get(enc0, &after));

	zassert_equal(after.position, 100);
	zassert_equal(after.position_epoch, before.position_epoch);
}

ZTEST(encoder_as5047p, test_corrupted_frame_is_stale_and_counted_as_parity)
{
	struct as5047p_stats before;
	struct as5047p_stats after;
	struct encoder_feedback fb;

	let_polls_run();
	zassert_ok(as5047p_get_stats(enc0, &before));

	/* Fail every read until checked, so the stale state is still there. */
	emul0()->inject_forever = true;
	emul0()->inject = INJECT_PARITY;
	k_sleep(K_USEC(1500));

	zassert_equal(get(enc0, &fb), -EAGAIN);
	zassert_true(fb.stale);
	zassert_true(fb.online);

	emul0()->inject = INJECT_NONE;
	zassert_ok(as5047p_get_stats(enc0, &after));
	zassert_true(after.errors[AS5047P_ERROR_PARITY] > before.errors[AS5047P_ERROR_PARITY]);
	zassert_equal(after.errors[AS5047P_ERROR_FLAG], before.errors[AS5047P_ERROR_FLAG]);

	let_polls_run();
	zassert_ok(get(enc0, &fb));
	zassert_false(fb.stale);
}

ZTEST(encoder_as5047p, test_error_flag_is_counted_with_the_latched_cause)
{
	struct as5047p_stats before;
	struct as5047p_stats after;

	let_polls_run();
	zassert_ok(as5047p_get_stats(enc0, &before));

	inject(emul0(), INJECT_FLAG, 1);
	let_polls_run();

	zassert_ok(as5047p_get_stats(enc0, &after));
	zassert_equal(after.errors[AS5047P_ERROR_FLAG], before.errors[AS5047P_ERROR_FLAG] + 1);
	zassert_equal(after.last_errfl, AS5047P_ERRFL_FRERR);
	zassert_equal(emul0()->errfl, 0, "the driver should clear ERRFL by reading it");
}

ZTEST(encoder_as5047p, test_spi_failure_is_counted_and_adds_to_error_count)
{
	struct as5047p_stats before;
	struct as5047p_stats after;
	struct encoder_feedback fb_before;
	struct encoder_feedback fb_after;

	let_polls_run();
	zassert_ok(as5047p_get_stats(enc0, &before));
	zassert_ok(get(enc0, &fb_before));

	inject(emul0(), INJECT_SPI, 1);
	let_polls_run();

	zassert_ok(as5047p_get_stats(enc0, &after));
	zassert_ok(get(enc0, &fb_after));
	zassert_equal(after.errors[AS5047P_ERROR_SPI], before.errors[AS5047P_ERROR_SPI] + 1);
	zassert_equal(fb_after.error_count - fb_before.error_count,
		      total_errors(&after) - total_errors(&before));
}

ZTEST(encoder_as5047p, test_goes_offline_and_rebuilds_with_new_epoch)
{
	struct encoder_feedback before;
	struct encoder_feedback fb;

	let_polls_run();
	zassert_ok(get(enc0, &before));

	emul0()->inject_forever = true;
	emul0()->inject = INJECT_SPI;
	k_sleep(K_MSEC(5));

	zassert_equal(get(enc0, &fb), -EIO);
	zassert_false(fb.online);

	emul0()->inject = INJECT_NONE;
	emul0()->raw_angle = 4321;
	let_polls_run();

	zassert_ok(get(enc0, &fb));
	zassert_true(fb.online);
	zassert_equal(fb.position, 4321);
	zassert_not_equal(fb.position_epoch, before.position_epoch);
}

ZTEST(encoder_as5047p, test_set_zero_makes_the_current_angle_read_zero)
{
	struct encoder_feedback before;
	struct encoder_feedback fb;
	uint16_t zero = 0;

	emul0()->raw_angle = 3000;
	let_polls_run();
	zassert_ok(get(enc0, &before));

	zassert_ok(encoder_set_zero(enc0));
	zassert_ok(as5047p_get_zero_position(enc0, &zero));
	zassert_equal(zero, 3000);
	zassert_equal(emul0()->zposl & ZPOSL_KEEP, ZPOSL_KEEP);

	let_polls_run();
	zassert_ok(get(enc0, &fb));
	zassert_equal(fb.single_turn, 0);
	zassert_equal(fb.position, 0);
	zassert_not_equal(fb.position_epoch, before.position_epoch);
}

ZTEST(encoder_as5047p, test_zero_position_round_trips_and_keeps_zposl_flags)
{
	uint16_t zero = 0;

	zassert_ok(as5047p_set_zero_position(enc0, 0x2aaa));
	zassert_equal(emul0()->zposm, 0xaa);
	zassert_equal(emul0()->zposl & 0x3fU, 0x2a);
	zassert_equal(emul0()->zposl & ZPOSL_KEEP, ZPOSL_KEEP);

	zassert_ok(as5047p_get_zero_position(enc0, &zero));
	zassert_equal(zero, 0x2aaa);

	zassert_equal(as5047p_set_zero_position(enc0, 0x4000), -EINVAL);
}

ZTEST(encoder_as5047p, test_read_diag_decodes_diaagc)
{
	struct as5047p_diag diag;

	emul0()->diaagc = BIT(11) | BIT(8) | 0xc8;
	zassert_ok(as5047p_read_diag(enc0, &diag));

	zassert_equal(diag.agc, 0xc8);
	zassert_true(diag.magl);
	zassert_false(diag.magh);
	zassert_false(diag.cof);
	zassert_true(diag.lf);
}

ZTEST(encoder_as5047p, test_driver_api_rejects_other_devices)
{
	struct as5047p_stats stats;
	const struct device *spi = DEVICE_DT_GET(DT_NODELABEL(spi0));

	zassert_equal(as5047p_get_stats(spi, &stats), -EINVAL);
	zassert_equal(as5047p_get_stats(enc0, NULL), -EINVAL);
}

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);

	emul0()->inject = INJECT_NONE;
	emul0()->inject_forever = false;
	emul1()->inject = INJECT_NONE;
	emul1()->inject_forever = false;
	emul0()->zposm = 0;
	emul0()->zposl = ZPOSL_KEEP;
	emul0()->raw_angle = 0;
	zassert_ok(encoder_reset(enc0));
	let_polls_run();
}

ZTEST_SUITE(encoder_as5047p, NULL, NULL, before_each, NULL, NULL);
