/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "ade9430.h"

#define DT_DRV_COMPAT adi_ade9430

LOG_MODULE_REGISTER(ADE9430, CONFIG_SENSOR_LOG_LEVEL);

static int ade9430_read(const struct device *dev, uint16_t reg_addr, uint32_t *reg_data)
{
	const struct ade9430_config *cfg = dev->config;
	struct spi_buf_set tx;
	struct spi_buf_set rx;
	int ret;
	uint8_t rw_reg, addr_reg;
	uint8_t data[4] = {0};

	addr_reg = reg_addr >> 4;
	rw_reg = ADE9430_SPI_READ | reg_addr << 4;

	struct spi_buf buf[3] = {
		{
			.buf = &addr_reg,
			.len = 1,
		},
		{
			.buf = &rw_reg,
			.len = 1,
		},
		{
			.buf = data,
			.len = 4,	
		}
	};

	tx.buffers = buf;
	tx.count = 2;
	rx.buffers = buf;
	rx.count = 3;

	if (reg_addr >= ADE9430_REG_RUN && reg_addr <= ADE9430_REG_VERSION) {
		buf[2].len = 2;
		ret = spi_transceive_dt(&cfg->bus, &tx, &rx);
		if (ret < 0) {
			return ret;
		}
		*reg_data = sys_get_be16(data);
	} else {
		ret = spi_transceive_dt(&cfg->bus, &tx, &rx);
		if (ret < 0) {
			return ret;
		}
		*reg_data = sys_get_be32(data);
	}

	return 0;
}

static int ade9430_write(const struct device *dev, uint16_t reg_addr,
			 uint32_t reg_data)
{
	const struct ade9430_config *cfg = dev->config;
	struct spi_buf_set tx;
	uint8_t rw_reg, addr_reg;
	uint8_t data[4] = {0};

	addr_reg = reg_addr >> 4;
	rw_reg = reg_addr << 4;

	struct spi_buf buf[3] = {
		{
			.buf = &addr_reg,
			.len = 1,
		},
		{
			.buf = &rw_reg,
			.len = 1,
		},
		{
			.buf = data,
			.len = 4,	
		}
	};

	tx.buffers = buf;
	tx.count = 3;

	if (reg_addr >= ADE9430_REG_RUN && reg_addr <= ADE9430_REG_VERSION) {
		buf[2].len = 2;
		sys_put_be16(reg_data, data);
		return spi_write_dt(&cfg->bus, &tx);
	}

	sys_put_be32(reg_data, data);
	return spi_write_dt(&cfg->bus, &tx);
}

static int ade9430_update_bits(const struct device *dev, uint16_t reg_addr,
			       uint32_t mask, uint32_t reg_data)
{
	int ret;
	uint32_t data;

	ret = ade9430_read(dev, reg_addr, &data);
	if (ret < 0) {
		return ret;
	}

	data &= ~mask;
	data |= reg_data & mask;

	return ade9430_write(dev, reg_addr, data);
}

int ade9430_read_temp(const struct device *dev)
{
	struct ade9430_data *data = dev->data;
	int ret;
	uint32_t temp_raw, temp, gain, offset;

	ret = ade9430_update_bits(dev, ADE9430_REG_TEMP_CFG, ADE9430_TEMP_START,
				  FIELD_PREP(ADE9430_TEMP_START, 1));
	if (ret < 0) {
		return ret;
	}

	/* New temperature reading available after 1.25ms */
	k_msleep(2);

	ret = ade9430_read(dev, ADE9430_REG_TEMP_RSLT, &temp_raw);
	if (ret < 0) {
		return ret;
	}

	temp_raw = FIELD_GET(ADE9430_TEMP_RESULT, temp_raw);

	ret = ade9430_read(dev, ADE9430_REG_TEMP_TRIM, &temp);
	if (ret < 0) {
		return ret;
	}

	gain = FIELD_GET(ADE9430_TEMP_GAIN, temp);
	offset = FIELD_GET(ADE9430_TEMP_OFFSET, temp);

	data->temp_deg = temp_raw * ((-1000) * (int64_t)gain / 65536) /
			1000 + (offset / 32);

	return 0;
}

static int ade9430_read_data_ph(const struct device *dev,
				enum sensor_channel chan,
				enum ade9430_phase phase)
{
	struct ade9430_data *data = dev->data;
	int ret;
	uint32_t temp;
	uint16_t irms_reg, vrms_reg, watt_reg;

	switch (phase) {
	case ADE9430_PHASE_A:
		irms_reg = ADE9430_REG_AIRMS;
		vrms_reg = ADE9430_REG_AVRMS;
		watt_reg = ADE9430_REG_AWATT;
		break;
	case ADE9430_PHASE_B:
		irms_reg = ADE9430_REG_BIRMS;
		vrms_reg = ADE9430_REG_BVRMS;
		watt_reg = ADE9430_REG_BWATT;
		break;
	case ADE9430_PHASE_C:
		irms_reg = ADE9430_REG_CIRMS;
		vrms_reg = ADE9430_REG_CVRMS;
		watt_reg = ADE9430_REG_CWATT;
		break;
	default:
		return -EINVAL;
	}

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_CURRENT) {
		ret = ade9430_read(dev, irms_reg, &temp);
		if (ret < 0) {
			return ret;
		}

		data->irms_val[phase] = temp * ADE9430_I_RES_NA / ADE9430_MILLI_SCALE;
	}

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE) {
		ret = ade9430_read(dev, vrms_reg, &temp);
		if (ret < 0) {
			return ret;
		}

		data->vrms_val[phase] = temp * ADE9430_V_RES_NV / ADE9430_MILLI_SCALE;
	}

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POWER) {
		ret = ade9430_read(dev, watt_reg, &temp);
		if (ret < 0) {
			return ret;
		} 

		data->watt_val[phase] = temp * ADE9430_W_RES_UW;
	}

	return 0;
}

static int ade9430_read_enabled_ph(const struct device *dev, enum sensor_channel chan)
{
	const struct ade9430_config *cfg = dev->config;
	enum ade9430_phase phase;
	int ret;

	for (phase = ADE9430_PHASE_A; phase <= ADE9430_PHASE_C; phase++) {
		if (cfg->enable_phase[phase]) {
			ret = ade9430_read_data_ph(dev, chan, phase);
			if (ret < 0) {
				return ret;
			}
		}
	}

	return 0;
}

static int ade9430_set_egy_model(const struct device *dev, enum ade9430_egy_model model,
				 uint16_t value)
{
	int ret;
	uint16_t egy_ld_accum, egy_tmr_mode, rd_rst_en, egy_pwr_en;
	uint16_t reg_val;

	switch (model) {
	case ADE9430_EGY_WITH_RESET:
		/* For reading energy with reset the value should be 1 */
		if (value != 1) {
			reg_val = 1;
		}
		egy_ld_accum = 0;
		egy_tmr_mode = 0;
		rd_rst_en = 1;
		egy_pwr_en = 1;
		break;
	case ADE9430_EGY_HALF_LINE_CYCLES:
		reg_val = value;
		egy_ld_accum = 1;
		egy_tmr_mode = 1;
		rd_rst_en = 0;
		egy_pwr_en = 1;
		break;
	case ADE9430_EGY_NR_SAMPLES:
		reg_val = value;
		egy_ld_accum = 1;
		egy_tmr_mode = 0;
		rd_rst_en = 0;
		egy_pwr_en = 1;
		break;
	default:
		return -EINVAL;
	}

	ret = ade9430_write(dev, ADE9430_REG_RUN, 0);
	if (ret < 0) {
		return ret;
	}

	ret = ade9430_write(dev, ADE9430_REG_EP_CFG,
			    FIELD_PREP(ADE9430_EGY_LD_ACCUM, egy_ld_accum) |
			    FIELD_PREP(ADE9430_EGY_TMR_MODE, egy_tmr_mode) |
			    FIELD_PREP(ADE9430_RD_RST_EN, rd_rst_en) |
			    FIELD_PREP(ADE9430_EGY_PWR_EN, egy_pwr_en));
	if (ret < 0) {
		return ret;
	}
		
	ret = ade9430_write(dev, ADE9430_REG_EGY_TIME, reg_val);
	if (ret < 0) {
		return ret;
	}

	return ade9430_write(dev, ADE9430_REG_RUN, 1);
}

static int ade9430_sample_fetch(const struct device *dev, 
				enum sensor_channel chan)
{
	const struct ade9430_config *cfg = dev->config;
	int ret;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
	case SENSOR_CHAN_CURRENT:
	case SENSOR_CHAN_POWER:
		return ade9430_read_enabled_ph(dev, chan);
	case SENSOR_CHAN_DIE_TEMP:
		if (cfg->enable_temp) {
			return ade9430_read_temp(dev);
		} else {
			return 0;
		}
	case SENSOR_CHAN_ALL:
		ret = ade9430_read_enabled_ph(dev, chan);
		if (ret < 0) {
			return ret;
		}
		return ade9430_read_temp(dev);
	default:
		LOG_ERR("Channel %d does not exist", chan);
		return -ENOTSUP;
	}
}

static int ade9430_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct ade9430_config *cfg = dev->config;
	struct ade9430_data *data = dev->data;
	enum ade9430_phase phase = data->selected_phase;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		val->val1 = data->vrms_val[phase] / ADE9430_MICRO_SCALE;
		val->val2 = data->vrms_val[phase] % ADE9430_MICRO_SCALE;
		break;
	case SENSOR_CHAN_CURRENT:
		val->val1 = data->irms_val[phase] / ADE9430_MICRO_SCALE;
		val->val2 = data->irms_val[phase] % ADE9430_MICRO_SCALE;
		break;
	case SENSOR_CHAN_DIE_TEMP:
		if (!cfg->enable_temp) {
			LOG_DBG("Temp sensor not enabled.");
			return -EINVAL;
		}
		val->val1 = data->temp_deg;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_POWER:
	case SENSOR_CHAN_ALL:
		val->val1 = data->watt_val[phase] / ADE9430_MICRO_SCALE;
		val->val2 = data->watt_val[phase] % ADE9430_MICRO_SCALE;
		break;
	default:
		LOG_ERR("Channel %d does not exist", chan);
		return -ENOTSUP;
	}

	return 0;
}

static int ade9430_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	struct ade9430_data *data = dev->data;

	switch ((int)attr) {
	case SENSOR_ATTR_ADE9430_PHASE:
		if (val->val1 < ADE9430_PHASE_A || val->val1 > ADE9430_PHASE_C) {
			LOG_ERR("Phase setting %d is invalid.", attr);
			return -EINVAL;
		}
		data->selected_phase = (enum ade9430_phase)val->val1;
		return 0;
	default:
		LOG_ERR("Attribute %d cannot be set.", attr);
		return -ENOTSUP;
	}
}

static int ade9430_init(const struct device *dev)
{
	const struct ade9430_config *cfg = dev->config;
	struct ade9430_data *data = dev->data;
	uint32_t chip_id;
	int ret;

	__ASSERT_NO_MSG(cfg->selected_phase >= 0 && cfg->selected_phase <= 2);

	/* Check spi bus */
	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("Device not ready.");
		return -ENODEV;
	}

	/* Toggle SW reset */
	ret = ade9430_update_bits(dev, ADE9430_REG_CONFIG1, ADE9430_SWRST,
				  FIELD_PREP(ADE9430_SWRST, 1));
	if (ret < 0) {
		return ret;
	}

	k_msleep(100);

	ret = ade9430_update_bits(dev, ADE9430_REG_CONFIG1, ADE9430_SWRST,
				  FIELD_PREP(ADE9430_SWRST, 0));
	if (ret < 0) {
		return ret;
	}

	/* Validate chip ID */
	ret = ade9430_read(dev, ADE9430_REG_CONFIG5, &chip_id);
	if (ret < 0) {
		return ret;
	}

	if (chip_id != ADE9430_CHIP_ID) {
		return -EIO;
	}

	/* Enable temp sensor */
	ret = ade9430_update_bits(dev, ADE9430_REG_TEMP_CFG, ADE9430_TEMP_EN,
				  FIELD_PREP(ADE9430_TEMP_EN, cfg->enable_temp));
	if (ret < 0) {
		return ret;
	}

	/* Set PHASE for channel_get */
	data->selected_phase = CLAMP(cfg->selected_phase, ADE9430_PHASE_A, ADE9430_PHASE_C);

	/* Set energy model */
	return ade9430_set_egy_model(dev, cfg->egy_model, cfg->egy_model_value);
}

static const struct sensor_driver_api ade9430_driver_api = {
	.sample_fetch = ade9430_sample_fetch,
	.channel_get = ade9430_channel_get,
	.attr_set = ade9430_attr_set,
};

#define ADE9430_DEFINE(inst)									\
	static struct ade9430_data ade9430_data_##inst;						\
	static const struct ade9430_config ade9430_config_##inst = {				\
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),	\
		.egy_model = DT_INST_PROP(inst, egy_model),					\
		.selected_phase = DT_INST_PROP(inst, selected_phase),				\
		.egy_model_value = DT_INST_PROP(inst, egy_model_value),				\
		.enable_phase = DT_INST_PROP(inst, enable_phase),				\
		.enable_temp = DT_INST_PROP(inst, enable_temp),					\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ade9430_init, NULL, &ade9430_data_##inst,		\
				     &ade9430_config_##inst, POST_KERNEL,			\
				     CONFIG_SENSOR_INIT_PRIORITY, &ade9430_driver_api);		\

DT_INST_FOREACH_STATUS_OKAY(ADE9430_DEFINE)
