/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "ltc2992.h"

#define DT_DRV_COMPAT adi_ltc2992

LOG_MODULE_REGISTER(LTC2992, CONFIG_SENSOR_LOG_LEVEL);

static const struct ltc2992_chan_regmap chan_regmap[6] = {
	{
		.data = LTC2992_REG_SENSE(0),
		.max = LTC2992_REG_SENSE_MAX(0),
		.min = LTC2992_REG_SENSE_MIN(0),
		.max_thresh = LTC2992_REG_SENSE_MAX_THRESH(0),
		.min_thresh = LTC2992_REG_SENSE_MIN_THRESH(0),
		.fault = LTC2992_REG_FAULT1,
		.fault_mask = LTC2992_POWER_FAULT_MSK | LTC2992_DSENSE_FAULT_MSK |
			      LTC2992_SENSE_FAULT_MSK,
		.alert = LTC2992_REG_ALERT1,
		.alert_mask = LTC2992_POWER_ALERT_MSK | LTC2992_DSENSE_ALERT_MSK |
			      LTC2992_SENSE_ALERT_MSK,
	},
	{
		.data = LTC2992_REG_SENSE(1),
		.max = LTC2992_REG_SENSE_MAX(1),
		.min = LTC2992_REG_SENSE_MIN(1),
		.max_thresh = LTC2992_REG_SENSE_MAX_THRESH(1),
		.min_thresh = LTC2992_REG_SENSE_MIN_THRESH(1),
		.fault = LTC2992_REG_FAULT2,
		.fault_mask = LTC2992_POWER_FAULT_MSK | LTC2992_DSENSE_FAULT_MSK |
			      LTC2992_SENSE_FAULT_MSK,
		.alert = LTC2992_REG_ALERT2,
		.alert_mask = LTC2992_POWER_ALERT_MSK | LTC2992_DSENSE_ALERT_MSK |
			      LTC2992_SENSE_ALERT_MSK,
	},
	{
		.data = LTC2992_REG_G1,
		.max = LTC2992_REG_G1_MAX,
		.min = LTC2992_REG_G1_MIN,
		.max_thresh = LTC2992_REG_G1_MAX_THRESH,
		.min_thresh = LTC2992_REG_G1_MIN_THRESH,
		.fault = LTC2992_REG_FAULT1,
		.fault_mask = GENMASK(1, 0),
		.alert = LTC2992_REG_ALERT1,
		.alert_mask = GENMASK(1, 0),
	},
	{
		.data = LTC2992_REG_G2,
		.max = LTC2992_REG_G2_MAX,
		.min = LTC2992_REG_G2_MIN,
		.max_thresh = LTC2992_REG_G2_MAX_THRESH,
		.min_thresh = LTC2992_REG_G2_MIN_THRESH,
		.fault = LTC2992_REG_FAULT2,
		.fault_mask = GENMASK(1, 0),
		.alert = LTC2992_REG_ALERT2,
		.alert_mask = GENMASK(1, 0),
	},
	{
		.data = LTC2992_REG_G3,
		.max = LTC2992_REG_G3_MAX,
		.min = LTC2992_REG_G3_MIN,
		.max_thresh = LTC2992_REG_G3_MAX_THRESH,
		.min_thresh = LTC2992_REG_G3_MIN_THRESH,
		.fault = LTC2992_REG_FAULT3,
		.fault_mask = GENMASK(7, 6),
		.alert = LTC2992_REG_ALERT3,
		.alert_mask = GENMASK(7, 6),
	},
	{
		.data = LTC2992_REG_G4,
		.max = LTC2992_REG_G4_MAX,
		.min = LTC2992_REG_G4_MIN,
		.max_thresh = LTC2992_REG_G4_MAX_THRESH,
		.min_thresh = LTC2992_REG_G4_MIN_THRESH,
		.fault = LTC2992_REG_FAULT3,
		.fault_mask = GENMASK(5, 4),
		.alert = LTC2992_REG_ALERT3,
		.alert_mask = GENMASK(5, 4),
	},
};

static int ltc2992_read_reg(const struct device *dev, uint8_t reg_addr,
			    uint32_t *data, uint8_t num_bytes)
{
	const struct ltc2992_config *cfg = dev->config;

	int ret;
	uint8_t rx_data[3];

	ret = i2c_burst_read_dt(&cfg->bus, reg_addr, rx_data, num_bytes);
	if (ret) {
		return ret;
	}

	switch (num_bytes) {
	case 1:
		*data = rx_data[0];
		break;
	case 2:
		*data = sys_get_be16(rx_data);
		break;
	case 3:
		*data = sys_get_be24(rx_data);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ltc2992_write_reg(const struct device *dev, uint8_t reg_addr,
			     uint32_t data, uint8_t num_bytes)
{
	const struct ltc2992_config *cfg = dev->config;
	uint8_t tx_data[3];

	switch (num_bytes) {
	case 1:
		tx_data[0] = data & 0xff;
		break;
	case 2:
		sys_put_be16(data, tx_data);
		break;
	case 3:
		sys_put_be24(data, tx_data);
		break;
	default:
		return -EINVAL;
	}

	return i2c_burst_write_dt(&cfg->bus, reg_addr, tx_data, num_bytes);
}

static int ltc2992_set_thresh(const struct device *dev,
			      enum sensor_channel chan,
			      enum sensor_attribute attr,
			      const struct sensor_value *val)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;
	enum ltc2992_sense_gpio sense = data->sense_chan;

	uint64_t value;
	uint64_t lsb;
	uint64_t rshunt = 1;
	uint32_t tx_data;
	uint32_t mask;
	uint8_t num_bytes = 2;
	uint8_t addr;

	if (chan != SENSOR_CHAN_VOLTAGE && sense >= LTC2992_GPIO1) {
		LOG_ERR("GPIO %d has no channel %d.\n", sense, chan);
		return -EINVAL;
	}

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		tx_data = sensor_value_to_micro(val);

		if (sense >= LTC2992_GPIO1) {
			lsb = (uint64_t)gpio_uv_lsb[cfg->resolution];
		} else {
			lsb = (uint64_t)sense_uv_lsb[cfg->resolution];
		}

		if (cfg->resolution == LTC2992_ADC_12_BIT) {
			mask = LTC2992_12B_SENSE_VAL_MSK;
		} else {
			mask = LTC2992_8B_SENSE_VAL_MSK;
		}

		if (attr == SENSOR_ATTR_UPPER_THRESH) {
			addr = chan_regmap[sense].max_thresh;
		} else {
			addr = chan_regmap[sense].min_thresh;
		}

		break;
	case SENSOR_CHAN_VSHUNT:
		if (sense >= LTC2992_GPIO1) {
			LOG_ERR("Sense %d does not have channel VSHUNT.", sense);
			return -EINVAL;
		}

		tx_data = sensor_value_to_micro(val);
		lsb = (uint64_t)dsense_nv_lsb[cfg->resolution];

		if (cfg->resolution == LTC2992_ADC_12_BIT) {
			mask = LTC2992_12B_SENSE_VAL_MSK;
		} else {
			mask = LTC2992_8B_SENSE_VAL_MSK;
		}

		if (attr == SENSOR_ATTR_UPPER_THRESH) {
			addr = LTC2992_REG_DSENSE_MAX_THRESH(sense);
		} else {
			addr = LTC2992_REG_DSENSE_MIN_THRESH(sense);
		}

		break;
	case SENSOR_CHAN_CURRENT:
		if (sense >= LTC2992_GPIO1) {
			LOG_ERR("Sense %d does not have channel CURRENT.", sense);
			return -EINVAL;
		}

		if (cfg->shunt_resistor[sense] <= 0) {
			LOG_ERR("Sense resistor must be > 0.");
			return -EINVAL;
		}

		tx_data = sensor_value_to_micro(val);
		lsb = (uint64_t)dsense_nv_lsb[cfg->resolution];
		rshunt = (uint64_t)cfg->shunt_resistor[sense];

		if (cfg->resolution == LTC2992_ADC_12_BIT) {
			mask = LTC2992_12B_SENSE_VAL_MSK;
		} else {
			mask = LTC2992_8B_SENSE_VAL_MSK;
		}

		if (attr == SENSOR_ATTR_UPPER_THRESH) {
			addr = LTC2992_REG_DSENSE_MAX_THRESH(sense);
		} else {
			addr = LTC2992_REG_DSENSE_MIN_THRESH(sense);
		}

		break;
	case SENSOR_CHAN_POWER:
		if (sense >= LTC2992_GPIO1) {
			LOG_ERR("Sense %d does not have channel POWER.", sense);
			return -EINVAL;
		}

		if (cfg->shunt_resistor[sense] <= 0) {
			LOG_ERR("Sense resistor must be > 0.");
			return -EINVAL;
		}

		tx_data = sensor_value_to_micro(val);
		lsb = (uint64_t)dsense_nv_lsb[cfg->resolution] *
		      (uint64_t)sense_uv_lsb[cfg->resolution] / LTC2992_MILLISCALE;
		rshunt = (uint64_t)cfg->shunt_resistor[sense] * LTC2992_MILLISCALE;

		if (cfg->resolution == LTC2992_ADC_12_BIT) {
			mask = LTC2992_12B_POWER_VAL_MSK;
		} else {
			mask = LTC2992_8B_POWER_VAL_MSK;
		}

		if (attr == SENSOR_ATTR_UPPER_THRESH) {
			addr = LTC2992_REG_POWER_MAX_THRESH(sense);
		} else {
			addr = LTC2992_REG_POWER_MIN_THRESH(sense);
		}

		num_bytes++;

		break;
	default:
		LOG_ERR("Channel %d not a SENSE/GPIO chan.", chan);
		return -ENOTSUP;
	}

	value = (uint64_t)tx_data * rshunt / lsb;

	tx_data = FIELD_PREP(mask, (uint32_t)value);

	return ltc2992_write_reg(dev, addr, tx_data, num_bytes);
}

static int ltc2992_set_alert(const struct device *dev,
			     enum sensor_channel chan,
			     const struct sensor_value *val)
{
	struct ltc2992_data *data = dev->data;
	enum ltc2992_sense_gpio sense = data->sense_chan;

	int ret;
	uint32_t rx_data;
	uint32_t tx_data = CLAMP((uint8_t)val->val1, 0, 3);
	uint8_t mask;

	ret = ltc2992_read_reg(dev, chan_regmap[sense].alert, &rx_data, 1);
	if (ret) {
		return ret;
	}

	if (sense <= LTC2992_SENSE2) {
		switch (chan) {
		case SENSOR_CHAN_VOLTAGE:
			mask = LTC2992_SENSE_ALERT_MSK;
			break;
		case SENSOR_CHAN_VSHUNT:
		case SENSOR_CHAN_CURRENT:
			mask = LTC2992_DSENSE_ALERT_MSK;
			break;
		case SENSOR_CHAN_POWER:
			mask = LTC2992_POWER_ALERT_MSK;
			break;
		default:
			return -ENOTSUP;
		}
	} else {
		mask = chan_regmap[sense].alert_mask;
	}

	rx_data &= ~mask;
	tx_data = FIELD_PREP(mask, tx_data) | rx_data;

	return ltc2992_write_reg(dev, chan_regmap[sense].alert, tx_data, 1);
}

static int ltc2992_get_fault(const struct device *dev,
			     enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct ltc2992_data *data = dev->data;
	enum ltc2992_sense_gpio sense = data->sense_chan;

	int ret;
	uint32_t rx_data;
	uint8_t mask;

	ret = ltc2992_read_reg(dev, chan_regmap[sense].fault, &rx_data, 1);
	if (ret) {
		return ret;
	}

	if (sense <= LTC2992_SENSE2) {
		switch (chan) {
		case SENSOR_CHAN_VOLTAGE:
			mask = LTC2992_SENSE_FAULT_MSK;
			break;
		case SENSOR_CHAN_VSHUNT:
		case SENSOR_CHAN_CURRENT:
			mask = LTC2992_DSENSE_FAULT_MSK;
			break;
		case SENSOR_CHAN_POWER:
			mask = LTC2992_POWER_FAULT_MSK;
			break;
		default:
			return -ENOTSUP;
		}
	} else {
		mask = chan_regmap[sense].fault_mask;
	}

	val->val1 = FIELD_GET(mask, rx_data);

	return 0;
}

static int ltc2992_get_minmax_data(const struct device *dev,
				   enum sensor_channel chan,
				   enum sensor_attribute attr,
				   struct sensor_value *val)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;
	enum ltc2992_sense_gpio sense = data->sense_chan;

	int ret;
	uint64_t value;
	uint64_t lsb;
	uint64_t scale = 1;
	uint32_t rx_data;
	uint8_t shift;
	uint8_t addr;
	size_t num_bytes = 2;

	if (chan == SENSOR_CHAN_POWER || chan == SENSOR_CHAN_ALL) {
		num_bytes++;
	}

	if (sense <= LTC2992_SENSE2) {
		switch (chan) {
		case SENSOR_CHAN_VOLTAGE:
			shift = LTC2992_12B_ADC_SSHIFT * (cfg->resolution + 1);
			lsb = (uint64_t)sense_uv_lsb[cfg->resolution];

			if (attr == SENSOR_ATTR_LTC2992_MAX_DATA) {
				addr = chan_regmap[sense].max;
			} else {
				addr = chan_regmap[sense].min;
			}
			break;
		case SENSOR_CHAN_VSHUNT:
			shift = LTC2992_12B_ADC_SSHIFT * (cfg->resolution + 1);
			lsb = (uint64_t)dsense_nv_lsb[cfg->resolution];

			if (attr == SENSOR_ATTR_LTC2992_MAX_DATA) {
				addr = LTC2992_REG_DSENSE_MAX(sense);
			} else {
				addr = LTC2992_REG_DSENSE_MIN(sense);
			}
			break;
		case SENSOR_CHAN_CURRENT:
			if (cfg->shunt_resistor[sense] <= 0) {
				LOG_ERR("Sense resistor must be > 0.");
				return -EINVAL;
			}

			shift = LTC2992_12B_ADC_SSHIFT * (cfg->resolution + 1);
			lsb = (uint64_t)dsense_nv_lsb[cfg->resolution];
			scale = (uint64_t)cfg->shunt_resistor[sense];

			if (attr == SENSOR_ATTR_LTC2992_MAX_DATA) {
				addr = LTC2992_REG_DSENSE_MAX(sense);
			} else {
				addr = LTC2992_REG_DSENSE_MIN(sense);
			}
			break;
		case SENSOR_CHAN_POWER:
			if (cfg->shunt_resistor[sense] <= 0) {
				LOG_ERR("Sense resistor must be > 0.");
				return -EINVAL;
			}

			shift = LTC2992_8B_ADC_PSHIFT * (cfg->resolution);
			lsb = (uint64_t)sense_uv_lsb[cfg->resolution] *
			      (uint64_t)dsense_nv_lsb[cfg->resolution] / LTC2992_MILLISCALE;
			scale = (uint64_t)cfg->shunt_resistor[sense] * LTC2992_MILLISCALE;

			if (attr == SENSOR_ATTR_LTC2992_MAX_DATA) {
				addr = LTC2992_REG_POWER_MAX(sense);
			} else {
				addr = LTC2992_REG_POWER_MIN(sense);
			}
			break;
		default:
			return -ENOTSUP;
		}
	} else {
		shift = LTC2992_12B_ADC_SSHIFT * (cfg->resolution + 1);
		lsb = (uint64_t)gpio_uv_lsb[cfg->resolution];

		if (attr == SENSOR_ATTR_LTC2992_MAX_DATA) {
			addr = chan_regmap[sense].max;
		} else {
			addr = chan_regmap[sense].min;
		}
	}

	ret = ltc2992_read_reg(dev, addr, &rx_data, num_bytes);
	if (ret) {
		return ret;
	}

	value = ((uint64_t)(rx_data >> shift) * lsb) / scale;

	val->val1 = (int32_t)(value / LTC2992_MICROSCALE);
	val->val2 = (int32_t)(value % LTC2992_MICROSCALE);

	return 0;
}

static int ltc2992_fetch_voltage_data(const struct device *dev)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;

	int ret;
	enum ltc2992_sense_gpio i;

	for (i = LTC2992_SENSE1; i <= LTC2992_GPIO4; i++) {
		if (i >= LTC2992_GPIO1 && !cfg->enable_gpios) {
			break;
		}

		ret = ltc2992_read_reg(dev, chan_regmap[i].data, &data->voltage[i], 2);
		if (ret)
			return ret;

		data->voltage[i] >>= (LTC2992_12B_ADC_SSHIFT * (cfg->resolution + 1));
	}

	return 0;
}

static int ltc2992_fetch_shunt_data(const struct device *dev)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;

	int ret;
	enum ltc2992_sense_gpio i;

	for (i = LTC2992_SENSE1; i <= LTC2992_SENSE2; i++) {
		ret = ltc2992_read_reg(dev, LTC2992_REG_DSENSE(i), &data->dvoltage[i], 2);
		if (ret) {
			return ret;
		}

		data->dvoltage[i] >>= (LTC2992_12B_ADC_SSHIFT * (cfg->resolution + 1));
	}

	return 0;
}

static int ltc2992_fetch_power_data(const struct device *dev)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;

	int ret;
	enum ltc2992_sense_gpio i;

	for (i = LTC2992_SENSE1; i <= LTC2992_SENSE2; i++) {
		ret = ltc2992_read_reg(dev, LTC2992_REG_POWER(i), &data->power[i], 3);
		if (ret) {
			return ret;
		}

		data->power[i] >>= (LTC2992_8B_ADC_PSHIFT * cfg->resolution);
	}

	return 0;
}


static int ltc2992_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	int ret;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		return ltc2992_fetch_voltage_data(dev);
	case SENSOR_CHAN_VSHUNT:
	case SENSOR_CHAN_CURRENT:
		return ltc2992_fetch_shunt_data(dev);
	case SENSOR_CHAN_POWER:
		return ltc2992_fetch_power_data(dev);
	case SENSOR_CHAN_ALL:
		ret = ltc2992_fetch_voltage_data(dev);
		if (ret) {
			return ret;
		}

		ret = ltc2992_fetch_shunt_data(dev);
		if (ret) {
			return ret;
		}

		ret = ltc2992_fetch_power_data(dev);
		if (ret) {
			return ret;
		}

		break;
	default:
		LOG_ERR("Channel %d does not exist", chan);
		return -ENOTSUP;
	}

	return 0;
}

static int ltc2992_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;
	enum ltc2992_sense_gpio sense = data->sense_chan;

	uint64_t value;

	if (sense >= LTC2992_GPIO1 && !cfg->enable_gpios) {
		LOG_ERR("GPIOS not enabled in device tree.");
		return -EINVAL;
	}

	switch (sense) {
	case LTC2992_SENSE1:
	case LTC2992_SENSE2:
		switch (chan) {
		case SENSOR_CHAN_VOLTAGE:
			value = (uint64_t)data->voltage[sense] *
				(uint64_t)sense_uv_lsb[cfg->resolution];
			break;
		case SENSOR_CHAN_VSHUNT:
			value = (uint64_t)data->dvoltage[sense] *
				(uint64_t)dsense_nv_lsb[cfg->resolution];
			break;
		case SENSOR_CHAN_CURRENT:
			if (cfg->shunt_resistor[sense] <= 0) {
				LOG_ERR("Sense resistor must be > 0.");
				return -EINVAL;
			}

			value = (uint64_t)data->dvoltage[sense] *
				(uint64_t)dsense_nv_lsb[cfg->resolution] /
				(uint64_t)cfg->shunt_resistor[sense];
			break;
		case SENSOR_CHAN_POWER:
		case SENSOR_CHAN_ALL:
			if (cfg->shunt_resistor[sense] <= 0) {
				LOG_ERR("Sense resistor must be > 0.");
				return -EINVAL;
			}

			value = (uint64_t)data->power[sense];
			value *= (uint64_t)sense_uv_lsb[cfg->resolution] *
				 (uint64_t)dsense_nv_lsb[cfg->resolution];
			value /= (uint64_t)cfg->shunt_resistor[sense] * LTC2992_MICROSCALE;

			break;
		default:
			LOG_ERR("Channel %d does not exist", chan);
			return -ENOTSUP;
		}
		break;
	case LTC2992_GPIO1:
	case LTC2992_GPIO2:
	case LTC2992_GPIO3:
	case LTC2992_GPIO4:
		if (chan == SENSOR_CHAN_VOLTAGE || chan == SENSOR_CHAN_ALL) {
			value = (uint64_t)data->voltage[sense] *
				(uint64_t)gpio_uv_lsb[cfg->resolution];
		} else {
			LOG_ERR("Channel %d does not exist", chan);
			return -ENOTSUP;
		}
		break;
	default:
		LOG_ERR("Sense channel attribute %d not supported.", chan);
		return -ENOTSUP;
	}

	val->val1 = (int32_t)(value / LTC2992_MICROSCALE);
	val->val2 = (int32_t)(value % LTC2992_MICROSCALE);

	return 0;
}

static int ltc2992_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;
	enum ltc2992_sense_gpio sense = data->sense_chan;

	if (sense >= LTC2992_GPIO1 && !cfg->enable_gpios) {
		LOG_ERR("GPIOS not enabled in device tree.");
		return -EINVAL;
	}

	switch ((int)attr) {
	case SENSOR_ATTR_UPPER_THRESH:
	case SENSOR_ATTR_LOWER_THRESH:
		return ltc2992_set_thresh(dev, chan, attr, val);
	case SENSOR_ATTR_ALERT:
		return ltc2992_set_alert(dev, chan, val);
	case SENSOR_ATTR_LTC2992_SENSE_CHAN:
		if (val->val1 < 1 || val->val1 > 6) {
			return -EINVAL;
		}
		data->sense_chan = val->val1 - 1;
		return 0;
	default:
		LOG_ERR("Attribute %d cannot be set.", attr);
		return -ENOTSUP;
	}
}

static int ltc2992_attr_get(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    struct sensor_value *val)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;
	enum ltc2992_sense_gpio sense = data->sense_chan;

	if (sense >= LTC2992_GPIO1 && !cfg->enable_gpios) {
		LOG_ERR("GPIOS not enabled in device tree.");
		return -EINVAL;
	}

	switch ((int)attr) {
	case SENSOR_ATTR_ALERT:
		return ltc2992_get_fault(dev, chan, val);
	case SENSOR_ATTR_LTC2992_MAX_DATA:
	case SENSOR_ATTR_LTC2992_MIN_DATA:
		return ltc2992_get_minmax_data(dev, chan, attr, val);
	default:
		LOG_ERR("Attribute %d cannot be obtained.", attr);
		return -ENOTSUP;
	}
}

static int ltc2992_init(const struct device *dev)
{
	const struct ltc2992_config *cfg = dev->config;
	struct ltc2992_data *data = dev->data;

	int ret;
	uint32_t id = 0;
	uint8_t tx_data = 0;

	__ASSERT_NO_MSG(cfg->shunt_resistor[0] > 0);
	__ASSERT_NO_MSG(cfg->shunt_resistor[1] > 0);

	/* check i2c bus */
	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("Device not ready.");
		return -ENODEV;
	}

	/* check id */
	ret = ltc2992_read_reg(dev, LTC2992_REG_MFR_SPECIAL_ID, &id, 2);
	if (ret) {
		LOG_ERR("I2C communication error.");
		return ret;
	}

	if (id != LTC2992_MFR_ID_VALUE) {
		LOG_ERR("Wrong mfr id.");
		return ret;
	}

	/* reset all regs including minmax regs */
	ret = ltc2992_write_reg(dev, LTC2992_REG_CTRLB, LTC2992_PEAK_RST_BIT | LTC2992_RESET_BIT,
				1);
	if (ret) {
		LOG_ERR("Reset failed.");
		return ret;
	}

	/* alerta scan mode settings */
	tx_data = FIELD_PREP(LTC2992_OFFSET_CAL_MSK, cfg->offset_calibration) |
		  FIELD_PREP(LTC2992_MEAS_MODE_MSK, cfg->measure_mode) |
		  FIELD_PREP(LTC2992_VSEL_CON_MSK, cfg->vsel_continuous) |
		  FIELD_PREP(LTC2992_VSEL_SNAP_MSK, cfg->vsel_snapshot);

	ret = ltc2992_write_reg(dev, LTC2992_REG_CTRLA, tx_data, 1);
	if (ret) {
		LOG_ERR("Scan mode setting failed.");
		return ret;
	}

	/* alertb settings */
	tx_data = FIELD_PREP(LTC2992_ALERT_CLR_BIT, cfg->alert_clear) |
		  FIELD_PREP(LTC2992_READ_CTRL_BIT, cfg->fault_clear_on_read) |
		  FIELD_PREP(LTC2992_STUCK_BUS_TIMEOUT_BIT, cfg->stuck_bus_timer_wakeup);

	ret = ltc2992_write_reg(dev, LTC2992_REG_CTRLB, tx_data, 1);
	if (ret) {
		LOG_ERR("CTRL B setting failed.");
		return ret;
	}

	/* set ADC resolution */
	ret = ltc2992_write_reg(dev, LTC2992_REG_NADC,
				cfg->resolution << LTC2992_RESOLUTION_POS, 1);
	if (ret) {
		LOG_ERR("ADC resolution setting failed.");
		return ret;
	}

	data->sense_chan = cfg->selected_channel - 1;

	return 0;
}

static const struct sensor_driver_api ltc2992_driver_api = {
	.sample_fetch = ltc2992_sample_fetch,
	.channel_get = ltc2992_channel_get,
	.attr_set = ltc2992_attr_set,
	.attr_get = ltc2992_attr_get,
};

#define LTC2992_DEFINE(inst)									\
	static struct ltc2992_data ltc2992_data_##inst;						\
	static const struct ltc2992_config ltc2992_config_##inst = {				\
		.bus = I2C_DT_SPEC_INST_GET(inst),						\
		.selected_channel = DT_INST_PROP(inst, selected_channel),			\
		.shunt_resistor = DT_INST_PROP(inst, shunt_resistor),				\
		.offset_calibration = DT_INST_PROP(inst, offset_calibration),			\
		.measure_mode = DT_INST_PROP(inst, measure_mode),				\
		.vsel_continuous = DT_INST_PROP(inst, vsel_continuous),				\
		.vsel_snapshot = DT_INST_PROP(inst, vsel_snapshot),				\
		.enable_gpios = DT_INST_PROP(inst, enable_gpios),				\
		.alert_clear = DT_INST_PROP(inst, alert_clear),					\
		.fault_clear_on_read = DT_INST_PROP(inst, fault_clear_on_read),			\
		.stuck_bus_timer_wakeup = DT_INST_PROP(inst, stuck_bus_timer_wakeup),		\
		.resolution = DT_INST_PROP(inst, resolution),					\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ltc2992_init, NULL, &ltc2992_data_##inst,		\
				     &ltc2992_config_##inst, POST_KERNEL,			\
				     CONFIG_SENSOR_INIT_PRIORITY, &ltc2992_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LTC2992_DEFINE)
