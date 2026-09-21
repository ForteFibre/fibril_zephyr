/*
 * Copyright (c) 2026 Forte Fibre
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as5047p

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/sensor/as5047p.h>

LOG_MODULE_REGISTER(as5047p, CONFIG_SENSOR_LOG_LEVEL);

#define AS5047P_REG_NOP       0x0000U
#define AS5047P_REG_ZPOSM     0x0016U
#define AS5047P_REG_ZPOSL     0x0017U
#define AS5047P_REG_SETTINGS1 0x0018U
#define AS5047P_REG_SETTINGS2 0x0019U
#define AS5047P_REG_ANGLECOM  0x3FFFU

#define AS5047P_FRAME_PARITY BIT(15)
#define AS5047P_FRAME_RW     BIT(14)
#define AS5047P_FRAME_EF     BIT(14)
#define AS5047P_DATA_MASK    GENMASK(13, 0)
#define AS5047P_ZPOSM_MASK   GENMASK(7, 0)
#define AS5047P_ZPOSL_MASK   GENMASK(5, 0)
#define AS5047P_COUNTS_PER_REV  16384U
#define AS5047P_DEGREES_PER_REV 360U
#define AS5047P_MICRO_UNITS     1000000U

struct as5047p_config {
	struct spi_dt_spec spi;
};

struct as5047p_data {
	uint16_t raw_angle;
};

static bool as5047p_even_parity(uint16_t frame)
{
	return (__builtin_popcount(frame) & 1) == 0;
}

static uint16_t as5047p_add_parity(uint16_t frame)
{
	if (!as5047p_even_parity(frame)) {
		frame |= AS5047P_FRAME_PARITY;
	}

	return frame;
}

static uint16_t as5047p_read_cmd(uint16_t reg)
{
	return as5047p_add_parity(AS5047P_FRAME_RW | (reg & AS5047P_DATA_MASK));
}

static uint16_t as5047p_write_cmd(uint16_t reg)
{
	return as5047p_add_parity(reg & AS5047P_DATA_MASK);
}

static uint16_t as5047p_data_frame(uint16_t value)
{
	return as5047p_add_parity(value & AS5047P_DATA_MASK);
}

static int as5047p_check_response(uint16_t frame)
{
	if (!as5047p_even_parity(frame)) {
		return -EIO;
	}

	if ((frame & AS5047P_FRAME_EF) != 0U) {
		return -EIO;
	}

	return 0;
}

static int as5047p_transfer(const struct device *dev, uint16_t tx_frame, uint16_t *rx_frame)
{
	const struct as5047p_config *config = dev->config;
	uint8_t tx_buf[2] = { tx_frame >> 8, tx_frame & 0xff };
	uint8_t rx_buf[2] = { 0 };
	const struct spi_buf tx = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx,
		.count = 1,
	};
	struct spi_buf rx = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx,
		.count = 1,
	};
	int ret;

	ret = spi_transceive_dt(&config->spi, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	*rx_frame = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

	return 0;
}

static int as5047p_read_reg(const struct device *dev, uint16_t reg, uint16_t *value)
{
	uint16_t response;
	int ret;

	ret = as5047p_transfer(dev, as5047p_read_cmd(reg), &response);
	if (ret < 0) {
		return ret;
	}

	ret = as5047p_transfer(dev, as5047p_data_frame(AS5047P_REG_NOP), &response);
	if (ret < 0) {
		return ret;
	}

	ret = as5047p_check_response(response);
	if (ret < 0) {
		return ret;
	}

	*value = response & AS5047P_DATA_MASK;

	return 0;
}

static int as5047p_write_reg(const struct device *dev, uint16_t reg, uint16_t value)
{
	uint16_t response;
	int ret;

	ret = as5047p_transfer(dev, as5047p_write_cmd(reg), &response);
	if (ret < 0) {
		return ret;
	}

	ret = as5047p_transfer(dev, as5047p_data_frame(value), &response);
	if (ret < 0) {
		return ret;
	}

	ret = as5047p_check_response(response);
	if (ret < 0) {
		return ret;
	}

	ret = as5047p_transfer(dev, as5047p_data_frame(AS5047P_REG_NOP), &response);
	if (ret < 0) {
		return ret;
	}

	return as5047p_check_response(response);
}

static int as5047p_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as5047p_data *data = dev->data;
	uint16_t raw_angle;
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ROTATION &&
	    chan != (enum sensor_channel)AS5047P_CHAN_RAW_ANGLE) {
		return -ENOTSUP;
	}

	ret = as5047p_read_reg(dev, AS5047P_REG_ANGLECOM, &raw_angle);
	if (ret < 0) {
		LOG_ERR("Failed to read ANGLECOM (%d)", ret);
		return ret;
	}

	data->raw_angle = raw_angle & AS5047P_DATA_MASK;

	return 0;
}

static int as5047p_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct as5047p_data *data = dev->data;
	uint64_t micro_degrees;

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		micro_degrees = (uint64_t)data->raw_angle * AS5047P_DEGREES_PER_REV *
				AS5047P_MICRO_UNITS / AS5047P_COUNTS_PER_REV;
		val->val1 = micro_degrees / AS5047P_MICRO_UNITS;
		val->val2 = micro_degrees % AS5047P_MICRO_UNITS;
		return 0;
	case AS5047P_CHAN_RAW_ANGLE:
		val->val1 = data->raw_angle;
		val->val2 = 0;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int as5047p_attr_get(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, struct sensor_value *val)
{
	uint16_t zposm;
	uint16_t zposl;
	uint16_t reg_value;
	int ret;

	if (chan != SENSOR_CHAN_ROTATION && chan != (enum sensor_channel)AS5047P_CHAN_RAW_ANGLE) {
		return -ENOTSUP;
	}

	switch ((enum as5047p_sensor_attribute)attr) {
	case AS5047P_ATTR_ZERO_POSITION:
		ret = as5047p_read_reg(dev, AS5047P_REG_ZPOSM, &zposm);
		if (ret < 0) {
			return ret;
		}
		ret = as5047p_read_reg(dev, AS5047P_REG_ZPOSL, &zposl);
		if (ret < 0) {
			return ret;
		}
		val->val1 = ((zposm & AS5047P_ZPOSM_MASK) << 6) | (zposl & AS5047P_ZPOSL_MASK);
		val->val2 = 0;
		return 0;
	case AS5047P_ATTR_SETTINGS1:
		ret = as5047p_read_reg(dev, AS5047P_REG_SETTINGS1, &reg_value);
		break;
	case AS5047P_ATTR_SETTINGS2:
		ret = as5047p_read_reg(dev, AS5047P_REG_SETTINGS2, &reg_value);
		break;
	default:
		return -ENOTSUP;
	}

	if (ret < 0) {
		return ret;
	}

	val->val1 = reg_value & AS5047P_DATA_MASK;
	val->val2 = 0;

	return 0;
}

static int as5047p_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	uint16_t reg_value;
	uint16_t zposl;
	int ret;

	if (chan != SENSOR_CHAN_ROTATION && chan != (enum sensor_channel)AS5047P_CHAN_RAW_ANGLE) {
		return -ENOTSUP;
	}

	if (val->val1 < 0 || val->val1 > AS5047P_DATA_MASK || val->val2 != 0) {
		return -EINVAL;
	}

	reg_value = val->val1;

	switch ((enum as5047p_sensor_attribute)attr) {
	case AS5047P_ATTR_ZERO_POSITION:
		ret = as5047p_read_reg(dev, AS5047P_REG_ZPOSL, &zposl);
		if (ret < 0) {
			return ret;
		}
		ret = as5047p_write_reg(dev, AS5047P_REG_ZPOSM,
					(reg_value >> 6) & AS5047P_ZPOSM_MASK);
		if (ret < 0) {
			return ret;
		}
		return as5047p_write_reg(dev, AS5047P_REG_ZPOSL,
					 (zposl & ~AS5047P_ZPOSL_MASK) |
						 (reg_value & AS5047P_ZPOSL_MASK));
	case AS5047P_ATTR_SETTINGS1:
		return as5047p_write_reg(dev, AS5047P_REG_SETTINGS1, reg_value);
	case AS5047P_ATTR_SETTINGS2:
		return as5047p_write_reg(dev, AS5047P_REG_SETTINGS2, reg_value);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, as5047p_api) = {
	.sample_fetch = as5047p_sample_fetch,
	.channel_get = as5047p_channel_get,
	.attr_get = as5047p_attr_get,
	.attr_set = as5047p_attr_set,
};

static int as5047p_init(const struct device *dev)
{
	const struct as5047p_config *config = dev->config;
	uint16_t response;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	return as5047p_transfer(dev, as5047p_data_frame(AS5047P_REG_NOP), &response);
}

#define AS5047P_INIT(inst)                                                                         \
	static struct as5047p_data as5047p_data_##inst;                                           \
	static const struct as5047p_config as5047p_config_##inst = {                              \
		.spi = SPI_DT_SPEC_INST_GET(inst,                                                 \
					    SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8U)),  \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, as5047p_init, NULL, &as5047p_data_##inst,              \
				     &as5047p_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &as5047p_api);

DT_INST_FOREACH_STATUS_OKAY(AS5047P_INIT)
