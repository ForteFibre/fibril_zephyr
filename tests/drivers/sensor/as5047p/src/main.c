/*
 * Copyright (c) 2026 Forte Fibre
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as5047p

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <drivers/sensor/as5047p.h>

#define AS5047P_NODE DT_NODELABEL(as5047p)

#define AS5047P_REG_NOP       0x0000U
#define AS5047P_REG_ZPOSM     0x0016U
#define AS5047P_REG_ZPOSL     0x0017U
#define AS5047P_REG_SETTINGS1 0x0018U
#define AS5047P_REG_SETTINGS2 0x0019U
#define AS5047P_REG_ANGLECOM  0x3FFFU

#define AS5047P_FRAME_RW     BIT(14)
#define AS5047P_FRAME_EF     BIT(14)
#define AS5047P_FRAME_PARITY BIT(15)
#define AS5047P_DATA_MASK    GENMASK(13, 0)
#define AS5047P_ZPOSL_KEEP   GENMASK(13, 6)

struct as5047p_emul_data {
	uint16_t anglecom;
	uint16_t zposm;
	uint16_t zposl;
	uint16_t settings1;
	uint16_t settings2;
	uint16_t pending_response;
	uint16_t pending_write_addr;
	bool write_pending;
	bool inject_ef;
	bool inject_bad_parity;
};

static bool frame_even_parity(uint16_t frame)
{
	return (__builtin_popcount(frame) & 1) == 0;
}

static uint16_t frame_add_parity(uint16_t frame)
{
	if (!frame_even_parity(frame)) {
		frame |= AS5047P_FRAME_PARITY;
	}

	return frame;
}

static uint16_t response_frame(uint16_t value, bool ef)
{
	uint16_t frame = value & AS5047P_DATA_MASK;

	if (ef) {
		frame |= AS5047P_FRAME_EF;
	}

	return frame_add_parity(frame);
}

static uint16_t emul_read_reg(struct as5047p_emul_data *data, uint16_t reg)
{
	switch (reg) {
	case AS5047P_REG_ZPOSM:
		return data->zposm;
	case AS5047P_REG_ZPOSL:
		return data->zposl;
	case AS5047P_REG_SETTINGS1:
		return data->settings1;
	case AS5047P_REG_SETTINGS2:
		return data->settings2;
	case AS5047P_REG_ANGLECOM:
		return data->anglecom;
	default:
		return 0;
	}
}

static void emul_write_reg(struct as5047p_emul_data *data, uint16_t reg, uint16_t value)
{
	value &= AS5047P_DATA_MASK;

	switch (reg) {
	case AS5047P_REG_ZPOSM:
		data->zposm = value;
		break;
	case AS5047P_REG_ZPOSL:
		data->zposl = value;
		break;
	case AS5047P_REG_SETTINGS1:
		data->settings1 = value;
		break;
	case AS5047P_REG_SETTINGS2:
		data->settings2 = value;
		break;
	case AS5047P_REG_ANGLECOM:
		data->anglecom = value;
		break;
	default:
		break;
	}
}

static void emul_set_next_response(struct as5047p_emul_data *data, uint16_t value, bool ef)
{
	data->pending_response = response_frame(value, ef || data->inject_ef);

	if (data->inject_bad_parity) {
		data->pending_response ^= BIT(0);
	}

	data->inject_ef = false;
	data->inject_bad_parity = false;
}

static uint16_t first_tx_frame(const struct spi_buf_set *tx_bufs)
{
	const uint8_t *buf = tx_bufs->buffers[0].buf;

	return ((uint16_t)buf[0] << 8) | buf[1];
}

static void put_first_rx_frame(const struct spi_buf_set *rx_bufs, uint16_t frame)
{
	uint8_t *buf = rx_bufs->buffers[0].buf;

	buf[0] = frame >> 8;
	buf[1] = frame & 0xff;
}

static int as5047p_emul_io(const struct emul *target, const struct spi_config *config,
			   const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct as5047p_emul_data *data = target->data;
	uint16_t tx_frame;
	uint16_t payload;
	bool is_read;

	if ((SPI_WORD_SIZE_GET(config->operation) != 8U) ||
	    ((config->operation & SPI_MODE_CPHA) == 0U)) {
		return -EIO;
	}

	if (tx_bufs == NULL || rx_bufs == NULL || tx_bufs->count < 1 || rx_bufs->count < 1 ||
	    tx_bufs->buffers[0].len != 2 || rx_bufs->buffers[0].len != 2) {
		return -EIO;
	}

	tx_frame = first_tx_frame(tx_bufs);
	put_first_rx_frame(rx_bufs, data->pending_response);

	if (!frame_even_parity(tx_frame)) {
		data->write_pending = false;
		emul_set_next_response(data, 0, true);
		return 0;
	}

	payload = tx_frame & AS5047P_DATA_MASK;
	is_read = (tx_frame & AS5047P_FRAME_RW) != 0U;

	if (data->write_pending) {
		emul_write_reg(data, data->pending_write_addr, payload);
		data->write_pending = false;
		emul_set_next_response(data, 0, false);
	} else if (is_read) {
		emul_set_next_response(data, emul_read_reg(data, payload), false);
	} else if (payload == AS5047P_REG_NOP) {
		emul_set_next_response(data, 0, false);
	} else {
		data->pending_write_addr = payload;
		data->write_pending = true;
		emul_set_next_response(data, 0, false);
	}

	return 0;
}

static int as5047p_emul_init(const struct emul *target, const struct device *parent)
{
	struct as5047p_emul_data *data = target->data;

	ARG_UNUSED(parent);
	memset(data, 0, sizeof(*data));
	data->pending_response = response_frame(0, false);

	return 0;
}

static struct spi_emul_api as5047p_emul_api = {
	.io = as5047p_emul_io,
};

#define AS5047P_EMUL_DEFINE(inst)                                                                  \
	static struct as5047p_emul_data as5047p_emul_data_##inst;                                  \
	EMUL_DT_INST_DEFINE(inst, as5047p_emul_init, &as5047p_emul_data_##inst, NULL,              \
			    &as5047p_emul_api, NULL);

DT_INST_FOREACH_STATUS_OKAY(AS5047P_EMUL_DEFINE)

static const struct device *const dev = DEVICE_DT_GET(AS5047P_NODE);
static const struct emul *const emul = EMUL_DT_GET(AS5047P_NODE);

static struct as5047p_emul_data *emul_data(void)
{
	return emul->data;
}

static void reset_emul(void)
{
	struct as5047p_emul_data *data = emul_data();

	memset(data, 0, sizeof(*data));
	data->pending_response = response_frame(0, false);
	data->zposl = AS5047P_ZPOSL_KEEP;
}

static void set_angle(uint16_t raw_angle)
{
	emul_data()->anglecom = raw_angle & AS5047P_DATA_MASK;
}

static void fetch_angle_and_expect(uint16_t raw_angle, int32_t degrees, int32_t micro)
{
	struct sensor_value val;

	set_angle(raw_angle);
	zassert_ok(sensor_sample_fetch(dev));
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &val));
	zassert_equal(val.val1, degrees);
	zassert_equal(val.val2, micro);
	zassert_ok(sensor_channel_get(dev, (enum sensor_channel)AS5047P_CHAN_RAW_ANGLE, &val));
	zassert_equal(val.val1, raw_angle);
	zassert_equal(val.val2, 0);
}

ZTEST(as5047p, test_device_ready)
{
	zassert_true(device_is_ready(dev));
}

ZTEST(as5047p, test_angle_conversion_and_raw_channel)
{
	fetch_angle_and_expect(0, 0, 0);
	fetch_angle_and_expect(8192, 180, 0);
	fetch_angle_and_expect(16383, 359, 978027);
}

ZTEST(as5047p, test_zero_position_attr)
{
	struct sensor_value val = { .val1 = 0x2aaa };

	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ROTATION, AS5047P_ATTR_ZERO_POSITION, &val));
	zassert_equal(emul_data()->zposm, 0xaa);
	zassert_equal(emul_data()->zposl & GENMASK(5, 0), 0x2a);
	zassert_equal(emul_data()->zposl & AS5047P_ZPOSL_KEEP, AS5047P_ZPOSL_KEEP);

	val.val1 = 0;
	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ROTATION, AS5047P_ATTR_ZERO_POSITION, &val));
	zassert_equal(val.val1, 0x2aaa);
	zassert_equal(val.val2, 0);
}

ZTEST(as5047p, test_settings_attrs)
{
	struct sensor_value val = { .val1 = 0x0123 };

	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ROTATION, AS5047P_ATTR_SETTINGS1, &val));
	val.val1 = 0x2345;
	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ROTATION, AS5047P_ATTR_SETTINGS2, &val));

	zassert_equal(emul_data()->settings1, 0x0123);
	zassert_equal(emul_data()->settings2, 0x2345);

	val.val1 = 0;
	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ROTATION, AS5047P_ATTR_SETTINGS1, &val));
	zassert_equal(val.val1, 0x0123);
	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ROTATION, AS5047P_ATTR_SETTINGS2, &val));
	zassert_equal(val.val1, 0x2345);
}

ZTEST(as5047p, test_spi_error_flags)
{
	emul_data()->inject_ef = true;
	zassert_equal(sensor_sample_fetch(dev), -EIO);

	emul_data()->inject_bad_parity = true;
	zassert_equal(sensor_sample_fetch(dev), -EIO);
}

ZTEST(as5047p, test_unsupported_channels_and_attrs)
{
	struct sensor_value val = { 0 };

	zassert_equal(sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP), -ENOTSUP);
	zassert_equal(sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &val), -ENOTSUP);
	zassert_equal(sensor_attr_get(dev, SENSOR_CHAN_AMBIENT_TEMP, AS5047P_ATTR_SETTINGS1, &val),
		      -ENOTSUP);
	zassert_equal(sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, AS5047P_ATTR_SETTINGS1, &val),
		      -ENOTSUP);

	zassert_equal(sensor_attr_get(dev, SENSOR_CHAN_ROTATION, SENSOR_ATTR_SAMPLING_FREQUENCY,
				      &val),
		      -ENOTSUP);
	zassert_equal(sensor_attr_set(dev, SENSOR_CHAN_ROTATION, SENSOR_ATTR_SAMPLING_FREQUENCY,
				      &val),
		      -ENOTSUP);

	val.val1 = 0x4000;
	zassert_equal(sensor_attr_set(dev, SENSOR_CHAN_ROTATION, AS5047P_ATTR_SETTINGS1, &val),
		      -EINVAL);
}

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);
	reset_emul();
}

ZTEST_SUITE(as5047p, NULL, NULL, before_each, NULL, NULL);
