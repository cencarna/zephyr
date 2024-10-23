/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#define LTC2992_REG_CTRLA			0x00
#define LTC2992_REG_CTRLB			0x01
#define LTC2992_REG_ALERT1			0x02
#define LTC2992_REG_FAULT1			0x03
#define LTC2992_REG_NADC			0x04
#define LTC2992_REG_POWER1			0x05
#define LTC2992_REG_POWER1_MAX			0x08
#define LTC2992_REG_POWER1_MIN			0x0B
#define LTC2992_REG_POWER1_MAX_THRESH		0x0E
#define LTC2992_REG_POWER1_MIN_THRESH		0x11
#define LTC2992_REG_DSENSE1			0x14
#define LTC2992_REG_DSENSE1_MAX			0x16
#define LTC2992_REG_DSENSE1_MIN			0x18
#define LTC2992_REG_DSENSE1_MAX_THRESH		0x1A
#define LTC2992_REG_DSENSE1_MIN_THRESH		0x1C
#define LTC2992_REG_SENSE1			0x1E
#define LTC2992_REG_SENSE1_MAX			0x20
#define LTC2992_REG_SENSE1_MIN			0x22
#define LTC2992_REG_SENSE1_MAX_THRESH		0x24
#define LTC2992_REG_SENSE1_MIN_THRESH		0x26
#define LTC2992_REG_G1				0x28
#define LTC2992_REG_G1_MAX			0x2A
#define LTC2992_REG_G1_MIN			0x2C
#define LTC2992_REG_G1_MAX_THRESH		0x2E
#define LTC2992_REG_G1_MIN_THRESH		0x30
#define LTC2992_REG_ALERT2			0x34
#define LTC2992_REG_FAULT2			0x35
#define LTC2992_REG_G2				0x5A
#define LTC2992_REG_G2_MAX			0x5C
#define LTC2992_REG_G2_MIN			0x5E
#define LTC2992_REG_G2_MAX_THRESH		0x60
#define LTC2992_REG_G2_MIN_THRESH		0x62
#define LTC2992_REG_G3				0x64
#define LTC2992_REG_G3_MAX			0x66
#define LTC2992_REG_G3_MIN			0x68
#define LTC2992_REG_G3_MAX_THRESH		0x6A
#define LTC2992_REG_G3_MIN_THRESH		0x6C
#define LTC2992_REG_G4				0x6E
#define LTC2992_REG_G4_MAX			0x70
#define LTC2992_REG_G4_MIN			0x72
#define LTC2992_REG_G4_MAX_THRESH		0x74
#define LTC2992_REG_G4_MIN_THRESH		0x76
#define LTC2992_REG_ALERT3			0x91
#define LTC2992_REG_FAULT3			0x92
#define LTC2992_REG_MFR_SPECIAL_ID		0xE7

#define LTC2992_REG_POWER(x)			(LTC2992_REG_POWER1 + ((x) * 0x32))
#define LTC2992_REG_POWER_MAX(x)		(LTC2992_REG_POWER1_MAX + ((x) * 0x32))
#define LTC2992_REG_POWER_MIN(x)		(LTC2992_REG_POWER1_MIN + ((x) * 0x32))
#define LTC2992_REG_POWER_MAX_THRESH(x)		(LTC2992_REG_POWER1_MAX_THRESH + ((x) * 0x32))
#define LTC2992_REG_POWER_MIN_THRESH(x)		(LTC2992_REG_POWER1_MIN_THRESH + ((x) * 0x32))
#define LTC2992_REG_DSENSE(x)			(LTC2992_REG_DSENSE1 + ((x) * 0x32))
#define LTC2992_REG_DSENSE_MAX(x)		(LTC2992_REG_DSENSE1_MAX + ((x) * 0x32))
#define LTC2992_REG_DSENSE_MIN(x)		(LTC2992_REG_DSENSE1_MIN + ((x) * 0x32))
#define LTC2992_REG_DSENSE_MAX_THRESH(x)	(LTC2992_REG_DSENSE1_MAX_THRESH + ((x) * 0x32))
#define LTC2992_REG_DSENSE_MIN_THRESH(x)	(LTC2992_REG_DSENSE1_MIN_THRESH + ((x) * 0x32))
#define LTC2992_REG_SENSE(x)			(LTC2992_REG_SENSE1 + ((x) * 0x32))
#define LTC2992_REG_SENSE_MAX(x)		(LTC2992_REG_SENSE1_MAX + ((x) * 0x32))
#define LTC2992_REG_SENSE_MIN(x)		(LTC2992_REG_SENSE1_MIN + ((x) * 0x32))
#define LTC2992_REG_SENSE_MAX_THRESH(x)		(LTC2992_REG_SENSE1_MAX_THRESH + ((x) * 0x32))
#define LTC2992_REG_SENSE_MIN_THRESH(x)		(LTC2992_REG_SENSE1_MIN_THRESH + ((x) * 0x32))
#define LTC2992_REG_POWER_FAULT(x)		(LTC2992_REG_FAULT1 + ((x) * 0x32))
#define LTC2992_REG_SENSE_FAULT(x)		(LTC2992_REG_FAULT1 + ((x) * 0x32))
#define LTC2992_REG_DSENSE_FAULT(x)		(LTC2992_REG_FAULT1 + ((x) * 0x32))

#define LTC2992_OFFSET_CAL_MSK			BIT(7)
#define LTC2992_MEAS_MODE_MSK			GENMASK(6,5)
#define LTC2992_VSEL_CON_MSK			GENMASK(4,3)
#define LTC2992_VSEL_SNAP_MSK			GENMASK(2,0)
#define LTC2992_POWER_ALERT_MSK			GENMASK(7,6)
#define LTC2992_DSENSE_ALERT_MSK		GENMASK(5,4)
#define LTC2992_SENSE_ALERT_MSK			GENMASK(3,2)
#define LTC2992_POWER_FAULT_MSK			GENMASK(7,6)
#define LTC2992_DSENSE_FAULT_MSK		GENMASK(5,4)
#define LTC2992_SENSE_FAULT_MSK			GENMASK(3,2)
#define LTC2992_12B_SENSE_VAL_MSK		GENMASK(15,4)
#define LTC2992_12B_POWER_VAL_MSK		GENMASK(23,0)
#define LTC2992_8B_SENSE_VAL_MSK		GENMASK(15,8)
#define LTC2992_8B_POWER_VAL_MSK		GENMASK(23,8)

#define LTC2992_ALERT_CLR_BIT			BIT(7)
#define LTC2992_READ_CTRL_BIT			BIT(5)
#define LTC2992_STUCK_BUS_TIMEOUT_BIT		BIT(4)
#define LTC2992_PEAK_RST_BIT			BIT(3)
#define LTC2992_RESET_BIT			BIT(0)

#define LTC2992_MFR_ID_VALUE			0x62
#define LTC2992_12B_ADC_SSHIFT			4
#define LTC2992_8B_ADC_SSHIFT			8
#define LTC2992_8B_ADC_PSHIFT			8
#define LTC2992_RESOLUTION_POS			7
#define LTC2992_MILLISCALE			1000
#define LTC2992_MICROSCALE			1000000

#define SENSOR_ATTR_LTC2992_SENSE_CHAN		(SENSOR_ATTR_PRIV_START + 1)
#define SENSOR_ATTR_LTC2992_MAX_DATA		(SENSOR_ATTR_PRIV_START + 2)
#define SENSOR_ATTR_LTC2992_MIN_DATA		(SENSOR_ATTR_PRIV_START + 3)

enum ltc2992_resolution {
	LTC2992_ADC_12_BIT = 0,
	LTC2992_ADC_8_BIT,
};

enum ltc2992_sense_gpio {
	LTC2992_SENSE1 = 0,
	LTC2992_SENSE2,
	LTC2992_GPIO1,
	LTC2992_GPIO2,
	LTC2992_GPIO3,
	LTC2992_GPIO4,
};

static const uint32_t sense_mv_lsb[2] = {25, 400};
static const uint32_t gpio_uv_lsb[2] = {500, 8000};
static const uint32_t dsense_nv_lsb[2] = {12500, 200000};

struct ltc2992_chan_regmap {
	uint8_t data;
	uint8_t max;
	uint8_t min;
	uint8_t max_thresh;
	uint8_t min_thresh;
	uint8_t fault;
	uint8_t fault_mask;
	uint8_t alert;
	uint8_t alert_mask;
};

struct ltc2992_data {
	enum ltc2992_sense_gpio sense_chan;
	uint32_t voltage[6];
	uint32_t dvoltage[2];
	uint32_t power[2];
};

struct ltc2992_config {
	struct i2c_dt_spec bus;
	uint8_t selected_channel;
	uint32_t shunt_resistor[2];
	uint8_t offset_calibration;
	uint8_t measure_mode;
	uint8_t vsel_continuous;
	uint8_t vsel_snapshot;
	bool enable_gpios;
	bool alert_clear;
	bool fault_clear_on_read;
	bool stuck_bus_timer_wakeup;
	enum ltc2992_resolution resolution;
};
