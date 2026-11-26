/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The emulated UART delivers what a test puts into it from its own work
 * queue, which is where the driver's interrupt callback runs on native_sim.
 * Every send therefore sleeps briefly before the test reads the result.
 */

#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <drivers/imu.h>

#define TEST_UART DEVICE_DT_GET(DT_NODELABEL(test_uart))
#define GYRO DEVICE_DT_GET(DT_NODELABEL(gyro))

#define STALE_TIMEOUT_MS DT_PROP(DT_NODELABEL(gyro), stale_timeout_ms)
#define OFFLINE_TIMEOUT_MS DT_PROP(DT_NODELABEL(gyro), offline_timeout_ms)

#define PACKET_LEN 26
#define DEG (3.14159265358979f / 180.0f)
#define G 9.80665f
#define TOL 1e-4f

static const char reset_command[] = "$MIB,RESET*87";

struct sample {
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int16_t gyro[3];
	int16_t accel[3];
};

static uint8_t sequence;

/* What the driver did before any test ran. */
static uint8_t boot_tx[64];
static uint32_t boot_tx_len;
static int boot_feedback_ret;

static void make_packet(const struct sample *s, uint8_t *out)
{
	uint8_t sum = 0;

	memset(out, 0, PACKET_LEN);
	out[0] = 0xA6;
	out[1] = 0xA6;
	out[2] = sequence++;
	sys_put_le16((uint16_t)s->roll, &out[3]);
	sys_put_le16((uint16_t)s->pitch, &out[5]);
	sys_put_le16((uint16_t)s->yaw, &out[7]);
	for (int i = 0; i < 3; i++) {
		sys_put_le16((uint16_t)s->gyro[i], &out[9 + 2 * i]);
		sys_put_le16((uint16_t)s->accel[i], &out[15 + 2 * i]);
	}

	for (int i = 2; i < 25; i++) {
		sum += out[i];
	}
	out[25] = sum;
}

static void put(const uint8_t *bytes, size_t len)
{
	zassert_equal(uart_emul_put_rx_data(TEST_UART, bytes, len), len);
	k_sleep(K_MSEC(2));
}

static void send_yaw(int16_t yaw)
{
	const struct sample s = {.yaw = yaw};
	uint8_t packet[PACKET_LEN];

	make_packet(&s, packet);
	put(packet, sizeof(packet));
}

static struct imu_feedback read_ok(void)
{
	struct imu_feedback fb;
	const int ret = imu_get_feedback(GYRO, &fb);

	zassert_ok(ret, "imu_get_feedback() returned %d", ret);

	return fb;
}

static void *r6093u_setup(void)
{
	struct imu_feedback fb;

	boot_tx_len = uart_emul_get_tx_data(TEST_UART, boot_tx, sizeof(boot_tx));
	boot_feedback_ret = imu_get_feedback(GYRO, &fb);

	return NULL;
}

static void r6093u_before(void *fixture)
{
	ARG_UNUSED(fixture);

	uart_irq_rx_enable(TEST_UART);
	zassert_ok(imu_reset(GYRO));
	uart_emul_flush_tx_data(TEST_UART);
	uart_emul_flush_rx_data(TEST_UART);
}

ZTEST(r6093u, test_reset_command_with_its_nul_is_sent_at_boot)
{
	zassert_equal(boot_tx_len, sizeof(reset_command));
	zassert_mem_equal(boot_tx, reset_command, sizeof(reset_command));
}

ZTEST(r6093u, test_no_reading_before_the_first_packet)
{
	zassert_equal(boot_feedback_ret, -ENODATA);
}

ZTEST(r6093u, test_packet_decodes_to_si_units)
{
	const struct sample s = {
		.roll = 1000,
		.pitch = -500,
		.yaw = 9000,
		.gyro = {100, -200, 300},
		.accel = {1000, 0, -1000},
	};
	uint8_t packet[PACKET_LEN];
	struct imu_feedback fb;

	make_packet(&s, packet);
	put(packet, sizeof(packet));
	fb = read_ok();

	zassert_equal(fb.valid_mask, IMU_FEEDBACK_ACCEL | IMU_FEEDBACK_GYRO | IMU_FEEDBACK_ATTITUDE);
	zassert_within(fb.roll, 10.0f * DEG, TOL);
	zassert_within(fb.pitch, -5.0f * DEG, TOL);
	zassert_within(fb.yaw, 90.0f * DEG, TOL);
	zassert_within(fb.gyro[0], 1.0f * DEG, TOL);
	zassert_within(fb.gyro[1], -2.0f * DEG, TOL);
	zassert_within(fb.gyro[2], 3.0f * DEG, TOL);
	zassert_within(fb.accel[0], G, TOL);
	zassert_within(fb.accel[1], 0.0f, TOL);
	zassert_within(fb.accel[2], -G, TOL);
	zassert_true(fb.online);
	zassert_false(fb.stale);
}

ZTEST(r6093u, test_yaw_keeps_counting_past_the_wrap_in_both_directions)
{
	send_yaw(17000);
	send_yaw(-17000);
	zassert_within(read_ok().yaw, 190.0f * DEG, TOL);

	send_yaw(17000);
	send_yaw(0);
	send_yaw(-17000);
	zassert_within(read_ok().yaw, -170.0f * DEG, TOL);

	/* Steps of 90 and 170 deg backwards, two of them across the wrap. */
	send_yaw(10000);
	send_yaw(-7000);
	send_yaw(12000);
	send_yaw(-5000);
	zassert_within(read_ok().yaw, -770.0f * DEG, TOL);
}

ZTEST(r6093u, test_reset_restarts_the_yaw_and_advances_the_epoch)
{
	uint8_t tx[sizeof(reset_command)];
	uint32_t epoch;

	send_yaw(17000);
	send_yaw(-17000);
	epoch = read_ok().yaw_epoch;

	zassert_ok(imu_reset(GYRO));
	zassert_equal(uart_emul_get_tx_data(TEST_UART, tx, sizeof(tx)), sizeof(reset_command));
	zassert_mem_equal(tx, reset_command, sizeof(reset_command));

	send_yaw(500);
	zassert_within(read_ok().yaw, 5.0f * DEG, TOL);
	zassert_equal(read_ok().yaw_epoch, epoch + 1);
}

ZTEST(r6093u, test_bad_checksum_is_counted_and_the_packet_dropped)
{
	const struct sample s = {.yaw = 4500};
	uint8_t packet[PACKET_LEN];
	uint32_t errors;

	send_yaw(1000);
	errors = read_ok().error_count;

	make_packet(&s, packet);
	packet[25] ^= 0xFF;
	put(packet, sizeof(packet));

	zassert_within(read_ok().yaw, 10.0f * DEG, TOL);
	zassert_equal(read_ok().error_count, errors + 1);

	send_yaw(4500);
	zassert_within(read_ok().yaw, 45.0f * DEG, TOL);
}

ZTEST(r6093u, test_packet_after_garbage_and_split_across_reads_is_decoded)
{
	static const uint8_t garbage[] = {0x00, 0xA6, 0x12, 0xA6, 0xA6, 0x34};
	const struct sample s = {.yaw = -3000};
	uint8_t packet[PACKET_LEN];

	put(garbage, sizeof(garbage));

	make_packet(&s, packet);
	put(packet, 10);
	put(packet + 10, sizeof(packet) - 10);

	zassert_within(read_ok().yaw, -30.0f * DEG, TOL);
}

ZTEST(r6093u, test_packet_right_after_a_corrupted_one_is_decoded)
{
	const struct sample bad = {.yaw = 1000};
	const struct sample good = {.yaw = 2000};
	uint8_t stream[2 * PACKET_LEN];

	make_packet(&bad, stream);
	stream[25] ^= 0xFF;
	make_packet(&good, stream + PACKET_LEN);
	put(stream, sizeof(stream));

	zassert_within(read_ok().yaw, 20.0f * DEG, TOL);
}

ZTEST(r6093u, test_uart_error_is_counted_and_the_damaged_packet_discarded)
{
	const struct sample s = {.yaw = 6000};
	uint8_t packet[PACKET_LEN];
	uint32_t errors;

	send_yaw(1000);
	errors = read_ok().error_count;

	make_packet(&s, packet);
	uart_emul_set_errors(TEST_UART, UART_ERROR_OVERRUN);
	put(packet, 13);
	put(packet + 13, sizeof(packet) - 13);

	zassert_within(read_ok().yaw, 10.0f * DEG, TOL);
	zassert_equal(read_ok().error_count, errors + 1);

	send_yaw(6000);
	zassert_within(read_ok().yaw, 60.0f * DEG, TOL);
}

ZTEST(r6093u, test_reading_turns_stale_then_offline_and_keeps_the_last_values)
{
	struct imu_feedback fb;

	send_yaw(1500);

	k_sleep(K_MSEC(STALE_TIMEOUT_MS + 10));
	zassert_equal(imu_get_feedback(GYRO, &fb), -EAGAIN);
	zassert_true(fb.stale);
	zassert_true(fb.online);

	k_sleep(K_MSEC(OFFLINE_TIMEOUT_MS));
	zassert_equal(imu_get_feedback(GYRO, &fb), -EIO);
	zassert_false(fb.online);
	zassert_within(fb.yaw, 15.0f * DEG, TOL);
}

ZTEST(r6093u, test_watchdog_rearms_a_receiver_that_stopped)
{
	const struct sample s = {.yaw = 7000};
	uint8_t packet[PACKET_LEN];
	struct imu_feedback fb;

	send_yaw(1000);

	/* Stands in for a receiver wedged by an error nobody cleared. */
	uart_irq_rx_disable(TEST_UART);
	make_packet(&s, packet);
	put(packet, sizeof(packet));
	(void)imu_get_feedback(GYRO, &fb);
	zassert_within(fb.yaw, 10.0f * DEG, TOL);

	/* Silent for longer than offline-timeout-ms, then up to one more watchdog
	 * period before it notices. */
	k_sleep(K_MSEC(3 * OFFLINE_TIMEOUT_MS));
	(void)imu_get_feedback(GYRO, &fb);
	zassert_within(fb.yaw, 70.0f * DEG, TOL);
}

ZTEST_SUITE(r6093u, NULL, r6093u_setup, r6093u_before, NULL, NULL);
