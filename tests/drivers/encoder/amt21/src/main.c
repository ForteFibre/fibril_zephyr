/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The emulated encoder answers from inside the uart_emul transmit callback,
 * which is the earliest moment a response can possibly arrive. A driver that
 * armed its receiver after the transmission completed would lose those bytes,
 * so every test here exercises that property implicitly.
 */

#include <stdarg.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#if defined(CONFIG_ENCODER_AMT21_SHELL)
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#endif

#include <drivers/encoder.h>
#include <drivers/encoder/amt21.h>

#define TEST_UART DEVICE_DT_GET(DT_NODELABEL(test_uart))
#define TEST_BUS DEVICE_DT_GET(DT_NODELABEL(amt21_bus))
#define ENC14 DEVICE_DT_GET(DT_NODELABEL(enc14))
#define ENC12 DEVICE_DT_GET(DT_NODELABEL(enc12))
#define ENC_MT DEVICE_DT_GET(DT_NODELABEL(enc_mt))

/* Second bus, on a board that echoes the transmitted command byte. */
#define TEST_UART_ECHO DEVICE_DT_GET(DT_NODELABEL(test_uart_echo))
#define TEST_BUS_ECHO DEVICE_DT_GET(DT_NODELABEL(amt21_bus_echo))
#define ENC_ECHO DEVICE_DT_GET(DT_NODELABEL(enc_echo))

#define ADDR14 0x54U
#define ADDR12 0x58U
#define ADDR_MT 0x5CU
#define ADDR_ECHO 0x60U

#define CMD_POSITION 0x00U
#define CMD_TURNS 0x01U
#define CMD_EXTENDED 0x02U
#define EXT_SET_ZERO 0x5EU
#define EXT_RESET 0x75U

#define POLL_INTERVAL_US DT_PROP(DT_NODELABEL(amt21_bus), poll_interval_us)
#define INTER_COMMAND_DELAY_US DT_PROP(DT_NODELABEL(amt21_bus), inter_command_delay_us)
#define OFFLINE_THRESHOLD DT_PROP(DT_NODELABEL(amt21_bus), offline_threshold)
#define MAX_RETRIES DT_PROP(DT_NODELABEL(amt21_bus), max_retries)

/** How the emulated encoder answers the next command. */
enum emul_mode {
	/** Well formed response. */
	EMUL_MODE_NORMAL,
	/** No response at all. */
	EMUL_MODE_SILENT,
	/** Only the low byte of the response. */
	EMUL_MODE_TRUNCATED,
	/** Response with the check bits inverted. */
	EMUL_MODE_BAD_CHECKSUM,
	/** Well formed response followed by one extra byte. */
	EMUL_MODE_EXTRA_BYTE,
};

struct emul_encoder {
	/** UART this encoder answers on. */
	const struct device *uart;
	uint8_t addr;
	/** Raw 14-bit position as it appears on the wire. */
	uint16_t position14;
	/** Raw 14-bit turns counter as it appears on the wire. */
	uint16_t turns14;
	enum emul_mode mode;
	/**
	 * Number of responses that honour mode before reverting to a normal one.
	 * Negative means the mode applies indefinitely.
	 */
	int32_t mode_count;
	/** Echo the command byte back, as a permanently enabled receiver would. */
	bool echoes;
	uint32_t position_commands;
	uint32_t turns_commands;
	uint32_t extended_commands;
	uint8_t last_extended;
};

static struct emul_encoder emul[4];

/** Command bytes seen on a bus, with the cycle count at which they arrived. */
struct tx_record {
	const struct device *uart;
	uint8_t bytes[2];
	uint8_t len;
	uint32_t cycle;
};

#define TX_LOG_SIZE 64
static struct tx_record tx_log[TX_LOG_SIZE];
static uint32_t tx_log_count;
static struct k_spinlock log_lock;

/** Extra bytes to inject into the receive path before the next response, along
 *  with the encoder they should precede. Scoping the injection lets a test that
 *  starves one encoder leave the others alone: without the target field, a
 *  stray byte queued for ENC14 could be consumed by whichever encoder is polled
 *  first, which is not always the intended one.
 */
static uint8_t inject_bytes[4];
static uint8_t inject_len;
static const struct device *inject_target_bus;
static uint8_t inject_target_addr;

static struct emul_encoder *emul_lookup(const struct device *uart, uint8_t addr)
{
	for (size_t i = 0; i < ARRAY_SIZE(emul); ++i) {
		if ((emul[i].uart == uart) && (emul[i].addr == addr)) {
			return &emul[i];
		}
	}

	return NULL;
}

/**
 * @brief Build a response frame from a 14-bit value.
 *
 * K1 is the odd parity of the odd numbered data bits and K0 that of the even
 * numbered ones. The datasheet worked example is 0x21AB becoming 0x61AB, which
 * test_frame_matches_datasheet checks.
 */
static uint16_t emul_frame(uint16_t value14)
{
	uint16_t k1 = 0;
	uint16_t k0 = 0;

	value14 &= 0x3FFFU;

	for (int i = 1; i < 14; i += 2) {
		k1 ^= (value14 >> i) & 1U;
	}
	for (int i = 0; i < 14; i += 2) {
		k0 ^= (value14 >> i) & 1U;
	}

	k1 = k1 ? 0U : 1U;
	k0 = k0 ? 0U : 1U;

	return (uint16_t)(value14 | (k0 << 14) | (k1 << 15));
}

static void emul_put_frame(const struct device *uart, uint16_t frame)
{
	uint8_t bytes[2] = {(uint8_t)(frame & 0xFFU), (uint8_t)(frame >> 8)};

	(void)uart_emul_put_rx_data(uart, bytes, sizeof(bytes));
}

static void emul_respond(struct emul_encoder *enc, uint8_t command, uint16_t value14)
{
	enum emul_mode mode = enc->mode;

	if (enc->mode_count == 0) {
		mode = EMUL_MODE_NORMAL;
	} else if (enc->mode_count > 0) {
		enc->mode_count--;
	}

	if (enc->echoes) {
		(void)uart_emul_put_rx_data(enc->uart, &command, 1U);
	}

	if ((inject_len > 0U) &&
	    ((inject_target_bus == NULL) ||
	     ((inject_target_bus == enc->uart) && (inject_target_addr == enc->addr)))) {
		(void)uart_emul_put_rx_data(enc->uart, inject_bytes, inject_len);
		inject_len = 0U;
		inject_target_bus = NULL;
		inject_target_addr = 0U;
	}

	switch (mode) {
	case EMUL_MODE_SILENT:
		break;

	case EMUL_MODE_TRUNCATED: {
		uint16_t frame = emul_frame(value14);
		uint8_t low = (uint8_t)(frame & 0xFFU);

		(void)uart_emul_put_rx_data(enc->uart, &low, 1U);
		break;
	}

	case EMUL_MODE_BAD_CHECKSUM:
		emul_put_frame(enc->uart, emul_frame(value14) ^ 0xC000U);
		break;

	case EMUL_MODE_EXTRA_BYTE: {
		uint8_t stray = 0xA5U;

		emul_put_frame(enc->uart, emul_frame(value14));
		(void)uart_emul_put_rx_data(enc->uart, &stray, 1U);
		break;
	}

	case EMUL_MODE_NORMAL:
	default:
		emul_put_frame(enc->uart, emul_frame(value14));
		break;
	}
}

static void emul_tx_ready(const struct device *dev, size_t size, void *user_data)
{
	uint8_t buf[4];
	size_t len;

	ARG_UNUSED(size);
	ARG_UNUSED(user_data);

	len = uart_emul_get_tx_data(dev, buf, sizeof(buf));
	if (len == 0U) {
		return;
	}

	K_SPINLOCK(&log_lock) {
		if (tx_log_count < TX_LOG_SIZE) {
			tx_log[tx_log_count].uart = dev;
			tx_log[tx_log_count].len = (uint8_t)MIN(len, sizeof(tx_log[0].bytes));
			memcpy(tx_log[tx_log_count].bytes, buf, tx_log[tx_log_count].len);
			tx_log[tx_log_count].cycle = k_cycle_get_32();
			tx_log_count++;
		}
	}

	if ((len == 2U) && ((buf[0] & 0x3U) == CMD_EXTENDED)) {
		/* Extended command: node address plus the command itself. Guarding
		 * on the command bits keeps two back-to-back single-byte commands
		 * that happen to be drained together from being misread as one
		 * extended command.
		 */
		struct emul_encoder *enc = emul_lookup(dev, (uint8_t)(buf[0] & ~0x3U));

		if (enc != NULL) {
			enc->extended_commands++;
			enc->last_extended = buf[1];
		}
		return;
	}

	uint8_t addr = (uint8_t)(buf[0] & ~0x3U);
	uint8_t command = (uint8_t)(buf[0] & 0x3U);
	struct emul_encoder *enc = emul_lookup(dev, addr);

	if (enc == NULL) {
		return;
	}

	if (command == CMD_POSITION) {
		enc->position_commands++;
		emul_respond(enc, buf[0], enc->position14);
	} else if (command == CMD_TURNS) {
		enc->turns_commands++;
		emul_respond(enc, buf[0], enc->turns14);
	}
}

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

/** @brief One scan of every encoder, generously rounded up. */
#define SCAN_MS 20

static void wait_scans(unsigned int scans)
{
	k_sleep(K_MSEC(SCAN_MS * scans));
}

static void reset_emul(void)
{
	K_SPINLOCK(&log_lock) {
		tx_log_count = 0U;
	}

	inject_len = 0U;
	inject_target_bus = NULL;
	inject_target_addr = 0U;

	for (size_t i = 0; i < ARRAY_SIZE(emul); ++i) {
		emul[i].mode = EMUL_MODE_NORMAL;
		emul[i].mode_count = -1;
		emul[i].position_commands = 0U;
		emul[i].turns_commands = 0U;
		emul[i].extended_commands = 0U;
		emul[i].last_extended = 0U;
	}
}

/**
 * @brief Wait for a successful reading taken after this call started.
 *
 * Waiting merely for a successful reading is not enough: the driver caches the
 * last good snapshot, so a stale value from before the test changed the
 * emulated encoder would satisfy that.
 */
static int wait_fresh(const struct device *dev, struct encoder_feedback *fb)
{
	int64_t started = k_uptime_get();

	/* Make sure the snapshot timestamp can be strictly greater than started. */
	k_sleep(K_MSEC(2));

	for (int i = 0; i < 60; ++i) {
		int ret = encoder_get_feedback(dev, fb);

		if ((ret == 0) && (fb->timestamp_ms > started)) {
			return 0;
		}
		k_sleep(K_MSEC(SCAN_MS));
	}

	return -ETIMEDOUT;
}

static uint32_t tx_count(void)
{
	uint32_t count;

	K_SPINLOCK(&log_lock) {
		count = tx_log_count;
	}

	return count;
}

/* -------------------------------------------------------------------------- */
/* Protocol decoding                                                          */
/* -------------------------------------------------------------------------- */

ZTEST(encoder_amt21, test_frame_matches_datasheet)
{
	/* The datasheet worked example: a 14-bit position of 0x21AB is sent as
	 * 0x61AB. This pins the check bit convention used by the emulator, and
	 * therefore by every other test.
	 */
	zassert_equal(emul_frame(0x21ABU), 0x61ABU);
}

ZTEST(encoder_amt21, test_position_14bit)
{
	struct encoder_feedback fb;

	emul[0].position14 = 0x21ABU;

	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_equal(fb.single_turn, 8619U, "single turn %u", fb.single_turn);
	zassert_true((fb.valid_mask & ENCODER_FEEDBACK_POSITION) != 0);
	zassert_true((fb.valid_mask & ENCODER_FEEDBACK_SINGLE_TURN) != 0);
	zassert_true((fb.valid_mask & ENCODER_FEEDBACK_TURNS) == 0);
	zassert_true(fb.online);
	zassert_false(fb.stale);

	/* 8619 of 16384 counts is 189.4 degrees. The class API reports raw counts,
	 * so a consumer converts using the resolution.
	 */
	uint8_t resolution;

	zassert_ok(encoder_get_resolution(ENC14, &resolution));
	zassert_within((int32_t)(((uint64_t)fb.single_turn * 360000U) >> resolution), 189404, 100,
		       "angle from %u counts", fb.single_turn);
}

ZTEST(encoder_amt21, test_position_12bit)
{
	struct encoder_feedback fb;
	uint8_t resolution;

	/* A 12-bit device reports its position in the upper 12 bits. */
	emul[1].position14 = 0x21ABU;

	zassert_ok(wait_fresh(ENC12, &fb));
	zassert_equal(fb.single_turn, 8619U >> 2, "position %u", fb.single_turn);

	zassert_ok(encoder_get_resolution(ENC12, &resolution));
	zassert_equal(resolution, 12);
}

ZTEST(encoder_amt21, test_multiturn_positive_and_negative)
{
	struct encoder_feedback fb;

	emul[2].position14 = 1234U;
	emul[2].turns14 = 1U;

	zassert_ok(wait_fresh(ENC_MT, &fb));
	zassert_true((fb.valid_mask & ENCODER_FEEDBACK_TURNS) != 0);
	zassert_equal(fb.turns, 1, "turns %d", fb.turns);
	zassert_equal(fb.single_turn, 1234U);

	/* Minus three as a 14-bit two's complement number. */
	emul[2].turns14 = (uint16_t)((-3) & 0x3FFF);
	wait_scans(3);

	zassert_ok(encoder_get_feedback(ENC_MT, &fb));
	zassert_equal(fb.turns, -3, "turns %d", fb.turns);
}

ZTEST(encoder_amt21, test_resolution_reported)
{
	uint8_t resolution = 0U;

	zassert_ok(encoder_get_resolution(ENC14, &resolution));
	zassert_equal(resolution, 14);
	zassert_equal(encoder_get_resolution(ENC14, NULL), -EINVAL);
}

/* -------------------------------------------------------------------------- */
/* Accumulated position, velocity and offset                                  */
/* -------------------------------------------------------------------------- */

/*
 * The driver polls continuously from boot, so the accumulator holds a running
 * total whose absolute value depends on everything that came before. These
 * tests therefore take a baseline reading and assert how the position moves,
 * rather than what it happens to be.
 */

/**
 * @brief Move the emulated encoder by @p step and capture the resulting velocity.
 *
 * Velocity covers one sampling interval, so the delta appears in a single
 * snapshot and the next one reads zero again. The snapshot wanted is therefore
 * the first one whose position has moved, which means polling faster than the
 * encoder is polled rather than sleeping between checks.
 *
 * @return true when that snapshot was caught, with its velocity in @p velocity.
 */
static bool move_and_capture_velocity(int32_t step, int32_t *velocity)
{
	struct encoder_feedback before;
	struct encoder_feedback fb;

	if (encoder_get_feedback(ENC14, &before) != 0) {
		return false;
	}

	emul[0].position14 = (uint16_t)((before.single_turn + (uint32_t)step) & 0x3FFFU);

	/* Sleeping rather than busy waiting, because the poll thread needs the CPU
	 * to take the reading being waited for.
	 */
	for (int i = 0; i < 10000; ++i) {
		if ((encoder_get_feedback(ENC14, &fb) == 0) && (fb.position != before.position)) {
			*velocity = fb.velocity;
			return (fb.valid_mask & ENCODER_FEEDBACK_VELOCITY) != 0U;
		}
		k_sleep(K_USEC(100));
	}

	return false;
}

ZTEST(encoder_amt21, test_position_accumulates_single_turn_deltas)
{
	struct encoder_feedback before;
	struct encoder_feedback after;

	emul[0].position14 = 5000U;
	zassert_ok(wait_fresh(ENC14, &before));

	emul[0].position14 = 5300U;
	zassert_ok(wait_fresh(ENC14, &after));

	zassert_equal(after.position - before.position, 300,
		      "position moved by %lld", after.position - before.position);
	zassert_equal(after.position_epoch, before.position_epoch,
		      "the accumulator was rebuilt during normal operation");
}

ZTEST(encoder_amt21, test_position_unwraps_forwards_and_backwards)
{
	struct encoder_feedback before;
	struct encoder_feedback after;

	/* Just below a full revolution, so the next reading crosses zero. Taking
	 * the shorter way round has to give +8 rather than the -16376 a plain
	 * subtraction would produce.
	 */
	emul[0].position14 = 16380U;
	zassert_ok(wait_fresh(ENC14, &before));

	emul[0].position14 = 4U;
	zassert_ok(wait_fresh(ENC14, &after));
	zassert_equal(after.position - before.position, 8,
		      "forward wrap moved by %lld", after.position - before.position);

	before = after;

	emul[0].position14 = 16380U;
	zassert_ok(wait_fresh(ENC14, &after));
	zassert_equal(after.position - before.position, -8,
		      "backward wrap moved by %lld", after.position - before.position);
}

ZTEST(encoder_amt21, test_velocity_follows_the_direction_of_travel)
{
	struct encoder_feedback fb;

	emul[0].position14 = 8000U;
	zassert_ok(wait_fresh(ENC14, &fb));

	/* A standing encoder reads zero rather than leaving the previous value in
	 * place, which is what a stalled motor has to look like.
	 */
	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_true((fb.valid_mask & ENCODER_FEEDBACK_VELOCITY) != 0);
	zassert_equal(fb.velocity, 0, "velocity %d while standing still", fb.velocity);
	zassert_true(fb.sample_interval_us > 0U, "no sampling interval reported");

	/* Velocity is a single-interval estimate, so a one-off jump shows up in
	 * exactly one snapshot and the next one reads zero again. Keeping the
	 * encoder turning is both the realistic case and the observable one.
	 */
	int32_t velocity = 0;

	zassert_true(move_and_capture_velocity(200, &velocity), "missed the forward transition");
	zassert_true(velocity > 0, "velocity %d moving forwards", velocity);

	zassert_true(move_and_capture_velocity(-200, &velocity), "missed the backward transition");
	zassert_true(velocity < 0, "velocity %d moving backwards", velocity);
}

ZTEST(encoder_amt21, test_set_position_shifts_only_the_accumulator)
{
	struct encoder_feedback fb;

	emul[0].position14 = 6000U;
	zassert_ok(wait_fresh(ENC14, &fb));

	zassert_ok(encoder_set_position(ENC14, 1000));
	zassert_ok(encoder_get_feedback(ENC14, &fb));
	zassert_equal(fb.position, 1000, "position %lld after being set", fb.position);
	zassert_equal(fb.single_turn, 6000U, "set_position moved the device reading");

	/* The offset is a shift, not a fixed value, so later motion still shows up. */
	emul[0].position14 = 6250U;
	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_equal(fb.position, 1250, "position %lld after moving", fb.position);
}

ZTEST(encoder_amt21, test_set_position_does_not_disturb_velocity)
{
	struct encoder_feedback fb;
	int32_t velocity = 0;

	/* The offset shifts the reported position, so a velocity worked out by
	 * subtracting two reported positions would show a spike unless the offset
	 * is present in both of them.
	 */
	emul[0].position14 = 4000U;
	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_ok(encoder_set_position(ENC14, -1000000));

	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_equal(fb.velocity, 0, "velocity %d after the offset was applied", fb.velocity);

	zassert_true(move_and_capture_velocity(150, &velocity), "missed the transition");
	zassert_true(velocity > 0, "velocity %d moving forwards after an offset", velocity);
}

ZTEST(encoder_amt21, test_set_position_rejects_a_position_with_no_headroom)
{
	struct encoder_feedback fb;

	emul[0].position14 = 4500U;
	zassert_ok(wait_fresh(ENC14, &fb));

	int64_t before = fb.position;

	/* Accepting these would make the offset itself, or a later sum of the raw
	 * count and the offset, overflow int64_t.
	 */
	zassert_equal(encoder_set_position(ENC14, INT64_MIN), -EINVAL);
	zassert_equal(encoder_set_position(ENC14, INT64_MAX), -EINVAL);

	zassert_ok(encoder_get_feedback(ENC14, &fb));
	zassert_equal(fb.position, before, "a rejected request moved the position");
}

ZTEST(encoder_amt21, test_multiturn_rebuild_folds_in_a_negative_turns_counter)
{
	struct encoder_feedback fb;
	int ret = 0;

	/* Only a rebuild reads the turns counter into the accumulator, so the
	 * encoder is taken offline to force one.
	 */
	emul[2].mode = EMUL_MODE_SILENT;

	for (int i = 0; i < 200; ++i) {
		ret = encoder_get_feedback(ENC_MT, &fb);
		if (ret == -EIO) {
			break;
		}
		k_sleep(K_MSEC(2));
	}

	zassert_equal(ret, -EIO, "expected -EIO once offline, got %d", ret);

	/* Minus one turn, which the rebuild has to scale without relying on the
	 * shift of a negative value that C leaves undefined.
	 */
	emul[2].mode = EMUL_MODE_NORMAL;
	emul[2].position14 = 500U;
	emul[2].turns14 = (uint16_t)((-1) & 0x3FFF);

	zassert_ok(wait_fresh(ENC_MT, &fb), "did not come back online");
	zassert_equal(fb.turns, -1, "turns %d", fb.turns);
	zassert_equal(fb.single_turn, 500U, "single turn %u", fb.single_turn);
	zassert_equal(fb.position, -(1LL << 14) + 500, "position %lld", fb.position);
}

ZTEST(encoder_amt21, test_going_offline_rebuilds_the_accumulator)
{
	struct encoder_feedback before;
	struct encoder_feedback fb;
	int ret = 0;

	emul[0].position14 = 900U;
	zassert_ok(wait_fresh(ENC14, &before));
	zassert_ok(encoder_set_position(ENC14, 500));

	/* Silence the encoder for long enough to be declared offline. While
	 * offline the encoder may turn past the half revolution the unwrap can
	 * resolve, so continuing the old total would be a fabrication.
	 */
	emul[0].mode = EMUL_MODE_SILENT;

	for (int i = 0; i < 200; ++i) {
		ret = encoder_get_feedback(ENC14, &fb);
		if (ret == -EIO) {
			break;
		}
		k_sleep(K_MSEC(2));
	}

	zassert_equal(ret, -EIO, "expected -EIO once offline, got %d", ret);

	emul[0].mode = EMUL_MODE_NORMAL;
	emul[0].position14 = 7777U;

	zassert_ok(wait_fresh(ENC14, &fb), "did not come back online");
	zassert_true(fb.position_epoch != before.position_epoch,
		     "recovery did not report a new epoch");

	/* Rebuilt from the absolute reading, and the offset set beforehand is gone
	 * rather than being applied to a total it no longer relates to.
	 */
	zassert_equal(fb.position, 7777, "position %lld after recovery", fb.position);
}

ZTEST(encoder_amt21, test_rejected_frames_do_not_rebuild_the_accumulator)
{
	struct encoder_feedback before;
	struct encoder_feedback fb;

	emul[0].position14 = 3200U;
	zassert_ok(wait_fresh(ENC14, &before));

	/* Fewer bad frames than offline-threshold. The reading goes stale, but the
	 * encoder stays online and the accumulator has to keep running so that a
	 * single dropped response does not reset a control loop.
	 */
	emul[0].mode = EMUL_MODE_BAD_CHECKSUM;
	emul[0].mode_count = OFFLINE_THRESHOLD - 1;
	wait_scans(2);

	emul[0].mode = EMUL_MODE_NORMAL;
	emul[0].position14 = 3450U;

	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_equal(fb.position_epoch, before.position_epoch,
		      "a rejected frame rebuilt the accumulator");
	zassert_equal(fb.position - before.position, 250,
		      "position moved by %lld across a rejected frame",
		      fb.position - before.position);
}

/* -------------------------------------------------------------------------- */
/* Dropped and corrupted responses                                            */
/* -------------------------------------------------------------------------- */

ZTEST(encoder_amt21, test_checksum_error_keeps_last_reading)
{
	struct encoder_feedback good;
	struct encoder_feedback fb;

	emul[0].position14 = 4096U;
	zassert_ok(wait_fresh(ENC14, &good));
	zassert_equal(good.single_turn, 4096U);

	/* Corrupt every response, and change the position so that accepting a bad
	 * frame would be visible.
	 */
	emul[0].position14 = 200U;
	emul[0].mode = EMUL_MODE_BAD_CHECKSUM;
	wait_scans(3);

	/* A rejected frame leaves the reading stale, so a non-zero return here is
	 * expected. What must not happen is the bad value being adopted.
	 */
	int ret = encoder_get_feedback(ENC14, &fb);

	zassert_true((ret == -EAGAIN) || (ret == -EIO), "unexpected %d", ret);
	zassert_equal(fb.single_turn, 4096U, "a rejected frame changed the position");
	zassert_true(fb.error_count > good.error_count, "error count did not move");

	emul[0].mode = EMUL_MODE_NORMAL;
	wait_scans(3);
	zassert_ok(encoder_get_feedback(ENC14, &fb));
	zassert_equal(fb.single_turn, 200U, "did not recover after the corruption stopped");
}

ZTEST(encoder_amt21, test_stale_precedes_offline)
{
	struct encoder_feedback fb;
	bool saw_stale_online = false;
	bool saw_offline = false;

	emul[0].position14 = 777U;
	zassert_ok(wait_fresh(ENC14, &fb));

	/* Silence the encoder for good and watch the transition. A dropped response
	 * must first surface as a stale reading that keeps the last good value, and
	 * only after offline-threshold consecutive failures may the encoder be
	 * declared offline. Asserting the order rather than sampling one instant
	 * avoids depending on how wide the stale window happens to be.
	 */
	emul[0].mode = EMUL_MODE_SILENT;

	for (int i = 0; i < 4000; ++i) {
		int ret = encoder_get_feedback(ENC14, &fb);

		if (ret == -EAGAIN) {
			zassert_true(fb.online, "reported stale and offline at once");
			zassert_equal(fb.single_turn, 777U,
				      "the last good reading was discarded while stale");
			saw_stale_online = true;
		} else if (ret == -EIO) {
			zassert_false(fb.online);
			saw_offline = true;
			break;
		}

		k_sleep(K_USEC(200));
	}

	zassert_true(saw_stale_online,
		     "went straight to offline without a stale reading in between");
	zassert_true(saw_offline, "never went offline despite permanent silence");
}

ZTEST(encoder_amt21, test_silence_takes_encoder_offline_then_recovers)
{
	struct encoder_feedback fb;

	emul[0].position14 = 321U;
	zassert_ok(wait_fresh(ENC14, &fb));

	emul[0].mode = EMUL_MODE_SILENT;

	int ret = 0;

	/* Bound the wait rather than assuming a scan rate: only the transition has
	 * to happen, and how many scans that takes is a configuration detail.
	 */
	for (int i = 0; i < 200; ++i) {
		ret = encoder_get_feedback(ENC14, &fb);
		if (ret == -EIO) {
			break;
		}
		k_sleep(K_MSEC(2));
	}

	zassert_equal(ret, -EIO, "expected -EIO once offline, got %d", ret);
	zassert_false(fb.online);
	zassert_true(fb.stale);

	emul[0].mode = EMUL_MODE_NORMAL;
	emul[0].position14 = 4321U;

	zassert_ok(wait_fresh(ENC14, &fb), "did not come back online");
	zassert_equal(fb.single_turn, 4321U);
	zassert_true(fb.online);
	zassert_false(fb.stale);
}

ZTEST(encoder_amt21, test_truncated_response_recovers)
{
	struct encoder_feedback fb;

	emul[0].position14 = 555U;
	zassert_ok(wait_fresh(ENC14, &fb));

	/* A response that stops after one byte leaves a byte in flight. If the
	 * transport did not resynchronise, the following responses would decode
	 * against the wrong byte boundary and never recover.
	 */
	emul[0].mode = EMUL_MODE_TRUNCATED;
	wait_scans(2);
	emul[0].mode = EMUL_MODE_NORMAL;
	emul[0].position14 = 666U;

	zassert_ok(wait_fresh(ENC14, &fb), "did not resynchronise");
	zassert_equal(fb.single_turn, 666U, "position %u after resynchronising", fb.single_turn);
}

ZTEST(encoder_amt21, test_extra_byte_recovers)
{
	struct encoder_feedback fb;

	emul[0].position14 = 111U;
	zassert_ok(wait_fresh(ENC14, &fb));

	emul[0].mode = EMUL_MODE_EXTRA_BYTE;
	wait_scans(2);
	emul[0].mode = EMUL_MODE_NORMAL;
	emul[0].position14 = 222U;

	zassert_ok(wait_fresh(ENC14, &fb), "did not recover from a desync");
	zassert_equal(fb.single_turn, 222U);
}

ZTEST(encoder_amt21, test_stray_byte_before_response_recovers)
{
	struct encoder_feedback fb;

	emul[0].position14 = 1000U;
	zassert_ok(wait_fresh(ENC14, &fb));

	/* A late byte from a previous exchange arriving just before the response.
	 * Scoped to ENC14 so it does not accidentally hit the other encoders on
	 * the same bus when the poll thread services them first.
	 */
	inject_bytes[0] = 0xFFU;
	inject_len = 1U;
	inject_target_bus = TEST_UART;
	inject_target_addr = ADDR14;
	wait_scans(2);

	emul[0].position14 = 2000U;
	zassert_ok(wait_fresh(ENC14, &fb), "did not recover from a stray byte");
	zassert_equal(fb.single_turn, 2000U);
}

ZTEST(encoder_amt21, test_echoed_command_is_discarded)
{
	struct encoder_feedback fb;

	/* Boards that tie the transceiver receiver enable low see their own command
	 * byte come back on the receive line. That bus declares tx-echo, and the
	 * transport has to discard the echo instead of decoding it as data.
	 */
	emul[3].position14 = 3000U;

	zassert_ok(wait_fresh(ENC_ECHO, &fb), "did not cope with an echoed command byte");
	zassert_equal(fb.single_turn, 3000U, "position %u with echo enabled", fb.single_turn);
	zassert_true(fb.online);

	emul[3].position14 = 4000U;
	zassert_ok(wait_fresh(ENC_ECHO, &fb));
	zassert_equal(fb.single_turn, 4000U);
}

ZTEST(encoder_amt21, test_one_bad_encoder_does_not_starve_the_others)
{
	struct encoder_feedback fb;

	emul[0].position14 = 1U;
	emul[1].position14 = 2U;
	emul[2].position14 = 3U;
	emul[2].turns14 = 0U;

	zassert_ok(wait_fresh(ENC12, &fb));

	/* The first encoder stops answering entirely. The others must keep being
	 * polled rather than being held up behind the retries.
	 */
	emul[0].mode = EMUL_MODE_SILENT;
	emul[1].position_commands = 0U;
	emul[2].position_commands = 0U;
	wait_scans(4);

	zassert_true(emul[1].position_commands > 0U, "12-bit encoder stopped being polled");
	zassert_true(emul[2].position_commands > 0U, "multi-turn encoder stopped being polled");

	zassert_ok(encoder_get_feedback(ENC12, &fb));
	zassert_equal(fb.single_turn, 2U >> 2);
}

/* -------------------------------------------------------------------------- */
/* Timing                                                                     */
/* -------------------------------------------------------------------------- */

ZTEST(encoder_amt21, test_inter_command_gap_is_observed)
{
	struct encoder_feedback fb;
	uint32_t count;
	uint32_t prev = 0;
	bool first = true;
	unsigned int checked = 0;

	zassert_ok(wait_fresh(ENC14, &fb));

	K_SPINLOCK(&log_lock) {
		tx_log_count = 0U;
	}
	wait_scans(3);

	count = tx_count();
	zassert_true(count >= 4U, "only %u commands were sent", count);

	for (uint32_t i = 0; i < count; ++i) {
		/* Each bus honours its own gap, so records from the second bus must
		 * not be mixed into the comparison.
		 */
		if (tx_log[i].uart != TEST_UART) {
			continue;
		}

		if (!first) {
			uint32_t delta_us =
				(uint32_t)k_cyc_to_us_floor64(tx_log[i].cycle - prev);

			/* Only the lower bound is meaningful: the gap between two scans
			 * is governed by the poll interval, which is longer. Allow one
			 * timer tick (10 us at CONFIG_SYS_CLOCK_TICKS_PER_SEC=100000) of
			 * rounding, otherwise a gap that lands exactly on the boundary
			 * flakes when k_cyc_to_us_floor64 rounds it down.
			 */
			zassert_true(delta_us + 10U >= INTER_COMMAND_DELAY_US,
				     "commands %u and %u were only %u us apart, expected at "
				     "least %u us",
				     i - 1, i, delta_us, (unsigned int)INTER_COMMAND_DELAY_US);
			checked++;
		}

		prev = tx_log[i].cycle;
		first = false;
	}

	zassert_true(checked >= 3U, "only %u gaps were checked", checked);
}

/* -------------------------------------------------------------------------- */
/* Extended commands                                                          */
/* -------------------------------------------------------------------------- */

ZTEST(encoder_amt21, test_set_zero_sends_extended_command)
{
	struct encoder_feedback fb;

	zassert_ok(wait_fresh(ENC14, &fb));
	emul[0].extended_commands = 0U;

	zassert_ok(encoder_set_zero(ENC14));

	zassert_equal(emul[0].extended_commands, 1U, "the extended command was not sent");
	zassert_equal(emul[0].last_extended, EXT_SET_ZERO);

	/* The encoder resets itself and answers nothing until it has restarted, so
	 * the driver must report it as unavailable rather than keep polling it.
	 */
	int ret = encoder_get_feedback(ENC14, &fb);

	zassert_true((ret == -ENODATA) || (ret == -EIO), "unexpected %d during blackout", ret);

	emul[0].position_commands = 0U;
	wait_scans(3);
	zassert_equal(emul[0].position_commands, 0U, "polled during the reset blackout");
}

ZTEST(encoder_amt21, test_reset_sends_extended_command)
{
	struct encoder_feedback fb;

	zassert_ok(wait_fresh(ENC14, &fb));
	emul[0].extended_commands = 0U;

	zassert_ok(encoder_reset(ENC14));
	zassert_equal(emul[0].extended_commands, 1U);
	zassert_equal(emul[0].last_extended, EXT_RESET);
}

ZTEST(encoder_amt21, test_set_zero_rejected_on_multiturn)
{
	/* Only single-turn devices can store a zero point. */
	zassert_equal(encoder_set_zero(ENC_MT), -ENOTSUP);
}

/* -------------------------------------------------------------------------- */
/* Statistics                                                                 */
/* -------------------------------------------------------------------------- */

ZTEST(encoder_amt21, test_stats_rejects_foreign_device)
{
	struct amt21_stats stats;
	struct amt21_bus_stats bus_stats;

	/* The UART is a real device but not an AMT21 encoder. */
	zassert_equal(amt21_get_stats(TEST_UART, &stats), -EINVAL);
	zassert_equal(amt21_clear_stats(TEST_UART), -EINVAL);
	zassert_equal(amt21_bus_get_stats(ENC14, &bus_stats), -EINVAL);
	zassert_equal(amt21_get_stats(NULL, &stats), -EINVAL);
	zassert_equal(amt21_get_stats(ENC14, NULL), -EINVAL);
}

#if defined(CONFIG_ENCODER_AMT21_STATS)

ZTEST(encoder_amt21, test_stats_count_causes_separately)
{
	struct amt21_stats before;
	struct amt21_stats after;
	struct encoder_feedback fb;

	zassert_ok(wait_fresh(ENC14, &fb));

	zassert_ok(amt21_clear_stats(ENC14));
	wait_scans(3);
	zassert_ok(amt21_get_stats(ENC14, &before));
	zassert_true(before.successes > 0U, "no successful transactions recorded");
	zassert_true(before.transactions >= before.successes);

	emul[0].mode = EMUL_MODE_BAD_CHECKSUM;
	wait_scans(3);
	emul[0].mode = EMUL_MODE_NORMAL;

	zassert_ok(amt21_get_stats(ENC14, &after));
	zassert_true(after.errors[AMT21_ERROR_CHECKSUM] > 0U, "checksum errors not counted");
	zassert_equal(after.errors[AMT21_ERROR_TIMEOUT], before.errors[AMT21_ERROR_TIMEOUT],
		      "a checksum failure was counted as a timeout");
	zassert_true(after.retries > 0U, "retries not counted");
	zassert_true(after.max_consecutive_errors > 0U);
	zassert_equal(after.last_error, AMT21_ERROR_CHECKSUM);
}

ZTEST(encoder_amt21, test_stats_count_timeouts_separately)
{
	struct amt21_stats stats;
	struct encoder_feedback fb;

	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_ok(amt21_clear_stats(ENC14));

	emul[0].mode = EMUL_MODE_SILENT;
	wait_scans(3);
	emul[0].mode = EMUL_MODE_NORMAL;

	zassert_ok(amt21_get_stats(ENC14, &stats));
	zassert_true(stats.errors[AMT21_ERROR_TIMEOUT] > 0U, "timeouts not counted");
	zassert_equal(stats.errors[AMT21_ERROR_CHECKSUM], 0U,
		      "a timeout was counted as a checksum failure");
}

ZTEST(encoder_amt21, test_clear_stats_resets_counters)
{
	struct amt21_stats stats;
	struct encoder_feedback fb;

	zassert_ok(wait_fresh(ENC14, &fb));
	wait_scans(2);

	zassert_ok(amt21_clear_stats(ENC14));
	zassert_ok(amt21_get_stats(ENC14, &stats));

	zassert_equal(stats.transactions, 0U);
	zassert_equal(stats.successes, 0U);
	zassert_equal(stats.max_consecutive_errors, 0U);
	zassert_equal(stats.errors[AMT21_ERROR_CHECKSUM], 0U);
}

ZTEST(encoder_amt21, test_bus_stats_are_bus_scoped)
{
	struct amt21_bus_stats before;
	struct amt21_bus_stats after;
	struct encoder_feedback fb;

	zassert_ok(wait_fresh(ENC14, &fb));

	zassert_ok(amt21_bus_clear_stats(TEST_BUS));
	wait_scans(3);
	zassert_ok(amt21_bus_get_stats(TEST_BUS, &before));
	zassert_true(before.scans > 0U, "scans not counted");

	/* A truncated response forces a resynchronisation, which belongs to the bus
	 * rather than to any one encoder.
	 */
	emul[0].mode = EMUL_MODE_TRUNCATED;
	wait_scans(3);
	emul[0].mode = EMUL_MODE_NORMAL;

	zassert_ok(amt21_bus_get_stats(TEST_BUS, &after));
	zassert_true(after.rx_restarts > before.rx_restarts, "receive restarts not counted");
	zassert_true(after.scans > before.scans);
}

ZTEST(encoder_amt21, test_bus_stats_count_stray_bytes)
{
	struct amt21_bus_stats before;
	struct amt21_bus_stats after;
	struct encoder_feedback fb;

	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_ok(amt21_bus_clear_stats(TEST_BUS));
	zassert_ok(amt21_bus_get_stats(TEST_BUS, &before));

	emul[0].mode = EMUL_MODE_EXTRA_BYTE;
	wait_scans(3);
	emul[0].mode = EMUL_MODE_NORMAL;

	zassert_ok(amt21_bus_get_stats(TEST_BUS, &after));
	zassert_true((after.stray_bytes > before.stray_bytes) ||
			     (after.rx_restarts > before.rx_restarts),
		     "an extra byte was neither counted nor resynchronised");
}

#if CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE > 0

ZTEST(encoder_amt21, test_error_log_keeps_raw_bytes)
{
	struct amt21_error_record records[CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE];
	struct encoder_feedback fb;
	int n;

	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_ok(amt21_clear_stats(ENC14));

	emul[0].position14 = 0x21ABU;
	emul[0].mode = EMUL_MODE_BAD_CHECKSUM;
	wait_scans(2);
	emul[0].mode = EMUL_MODE_NORMAL;

	n = amt21_get_error_log(ENC14, records, ARRAY_SIZE(records));
	zassert_true(n > 0, "nothing was recorded, got %d", n);

	zassert_equal(records[0].cause, AMT21_ERROR_CHECKSUM);
	zassert_equal(records[0].node_addr, ADDR14);
	zassert_equal(records[0].command, ADDR14 | CMD_POSITION);
	zassert_equal(records[0].rx_len, 2, "rx_len %u", records[0].rx_len);

	/* The raw bytes are what make the failure diagnosable: this is the correct
	 * frame with its check bits flipped.
	 */
	uint16_t expected = emul_frame(0x21ABU) ^ 0xC000U;

	zassert_equal(records[0].rx_bytes[0], (uint8_t)(expected & 0xFFU));
	zassert_equal(records[0].rx_bytes[1], (uint8_t)(expected >> 8));
}

ZTEST(encoder_amt21, test_error_log_overwrites_oldest)
{
	struct amt21_error_record records[CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE + 4];
	struct encoder_feedback fb;
	int n;

	zassert_ok(wait_fresh(ENC14, &fb));
	zassert_ok(amt21_clear_stats(ENC14));

	emul[0].mode = EMUL_MODE_SILENT;
	wait_scans(CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE + 4);
	emul[0].mode = EMUL_MODE_NORMAL;

	n = amt21_get_error_log(ENC14, records, ARRAY_SIZE(records));
	zassert_equal(n, CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE,
		      "log did not saturate at its size, got %d", n);

	for (int i = 0; i < n; ++i) {
		zassert_equal(records[i].cause, AMT21_ERROR_TIMEOUT);
	}
}

#endif /* CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE > 0 */

#else /* !CONFIG_ENCODER_AMT21_STATS */

ZTEST(encoder_amt21, test_stats_disabled_returns_enotsup)
{
	struct amt21_stats stats;
	struct amt21_bus_stats bus_stats;
	struct amt21_error_record record;
	struct encoder_feedback fb;

	zassert_equal(amt21_get_stats(ENC14, &stats), -ENOTSUP);
	zassert_equal(amt21_clear_stats(ENC14), -ENOTSUP);
	zassert_equal(amt21_get_error_log(ENC14, &record, 1U), -ENOTSUP);
	zassert_equal(amt21_bus_get_stats(TEST_BUS, &bus_stats), -ENOTSUP);
	zassert_equal(amt21_bus_clear_stats(TEST_BUS), -ENOTSUP);

	/* The aggregate count in the class API stays available regardless. */
	zassert_ok(wait_fresh(ENC14, &fb));

	uint32_t before = fb.error_count;

	emul[0].mode = EMUL_MODE_BAD_CHECKSUM;
	wait_scans(3);
	emul[0].mode = EMUL_MODE_NORMAL;

	(void)encoder_get_feedback(ENC14, &fb);
	zassert_true(fb.error_count > before, "the aggregate error count did not move");
}

#endif /* CONFIG_ENCODER_AMT21_STATS */

/* -------------------------------------------------------------------------- */
/* Shell                                                                      */
/* -------------------------------------------------------------------------- */

#if defined(CONFIG_ENCODER_AMT21_SHELL)

static void run_shell_cmd(const char *fmt, ...)
{
	char cmd[64];
	va_list args;

	va_start(args, fmt);
	(void)vsnprintk(cmd, sizeof(cmd), fmt, args);
	va_end(args);

	shell_backend_dummy_clear_output(shell_backend_dummy_get_ptr());
	zassert_ok(shell_execute_cmd(NULL, cmd), "\"%s\" failed", cmd);

	/* A command that succeeds but prints nothing is not doing its job. */
	size_t out_len = 0;

	(void)shell_backend_dummy_get_output(shell_backend_dummy_get_ptr(), &out_len);
	zassert_true(out_len > 0U, "\"%s\" printed nothing", cmd);
}

ZTEST(encoder_amt21, test_shell_commands)
{
	struct encoder_feedback fb;

	zassert_ok(wait_fresh(ENC14, &fb));

	/* Provoke a few recorded failures so that errlog has something to format. */
	emul[0].mode = EMUL_MODE_BAD_CHECKSUM;
	wait_scans(2);
	emul[0].mode = EMUL_MODE_NORMAL;

	run_shell_cmd("amt21 list");
	run_shell_cmd("amt21 read %s", ENC14->name);
	run_shell_cmd("amt21 read %s", ENC_MT->name);
	run_shell_cmd("amt21 stats %s", ENC14->name);
	run_shell_cmd("amt21 errlog %s", ENC14->name);
	run_shell_cmd("amt21 errlog %s", ENC12->name);
	run_shell_cmd("amt21 bus-stats %s", TEST_BUS->name);
	run_shell_cmd("amt21 stats-clear %s", ENC14->name);
	run_shell_cmd("amt21 bus-stats-clear %s", TEST_BUS->name);

	/* A name that is not an AMT21 encoder must be rejected, not dereferenced. */
	zassert_not_equal(shell_execute_cmd(NULL, "amt21 read no_such_device"), 0);
	zassert_not_equal(shell_execute_cmd(NULL, "amt21 bus-stats no_such_bus"), 0);
}

ZTEST(encoder_amt21, test_shell_extended_commands)
{
	struct encoder_feedback fb;
	char cmd[64];

	zassert_ok(wait_fresh(ENC14, &fb));
	emul[0].extended_commands = 0U;

	(void)snprintk(cmd, sizeof(cmd), "amt21 zero %s", ENC14->name);
	zassert_ok(shell_execute_cmd(NULL, cmd));
	zassert_equal(emul[0].extended_commands, 1U);

	/* A multi-turn device cannot store a zero point. The command reports that
	 * rather than failing, so it must not have reached the bus.
	 */
	emul[2].extended_commands = 0U;
	(void)snprintk(cmd, sizeof(cmd), "amt21 zero %s", ENC_MT->name);
	zassert_ok(shell_execute_cmd(NULL, cmd));
	zassert_equal(emul[2].extended_commands, 0U);

	(void)snprintk(cmd, sizeof(cmd), "amt21 reset %s", ENC_MT->name);
	zassert_ok(shell_execute_cmd(NULL, cmd));
	zassert_equal(emul[2].extended_commands, 1U);
	zassert_equal(emul[2].last_extended, EXT_RESET);
}

#endif /* CONFIG_ENCODER_AMT21_SHELL */

/* -------------------------------------------------------------------------- */
/* Suite                                                                      */
/* -------------------------------------------------------------------------- */

static void *suite_setup(void)
{
	emul[0].uart = TEST_UART;
	emul[0].addr = ADDR14;
	emul[1].uart = TEST_UART;
	emul[1].addr = ADDR12;
	emul[2].uart = TEST_UART;
	emul[2].addr = ADDR_MT;
	emul[3].uart = TEST_UART_ECHO;
	emul[3].addr = ADDR_ECHO;
	emul[3].echoes = true;

	reset_emul();

	uart_emul_callback_tx_data_ready_set(TEST_UART, emul_tx_ready, NULL);
	uart_emul_callback_tx_data_ready_set(TEST_UART_ECHO, emul_tx_ready, NULL);

	zassert_true(device_is_ready(TEST_BUS));
	zassert_true(device_is_ready(ENC14));
	zassert_true(device_is_ready(ENC12));
	zassert_true(device_is_ready(ENC_MT));
	zassert_true(device_is_ready(TEST_BUS_ECHO));
	zassert_true(device_is_ready(ENC_ECHO));

	return NULL;
}

static void test_before(void *fixture)
{
	ARG_UNUSED(fixture);

	reset_emul();

	for (size_t i = 0; i < ARRAY_SIZE(emul); ++i) {
		emul[i].position14 = 0U;
		emul[i].turns14 = 0U;
	}

	/* Let any blackout or error state from the previous test drain away. */
	k_sleep(K_MSEC(300));

#if defined(CONFIG_ENCODER_AMT21_STATS)
	(void)amt21_clear_stats(ENC14);
	(void)amt21_clear_stats(ENC12);
	(void)amt21_clear_stats(ENC_MT);
	(void)amt21_bus_clear_stats(TEST_BUS);
#endif
}

ZTEST_SUITE(encoder_amt21, NULL, suite_setup, test_before, NULL, NULL);
