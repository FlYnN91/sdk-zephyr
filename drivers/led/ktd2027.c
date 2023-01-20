/*
 * Copyright (c) 2023 Dainius Narsutis <dainius@dainiusnarsutis.co.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kt_ktd2027

/**
 * @file
 * @brief KTD2027 LED driver
 *
 * The KTD2027 is a 4-channel output current sink device,
 * offering constant current regulation with high efficiency and ultralow
 * internal voltage drop.
 *
 * Each current sink can be configured independently to one of the
 * 192-step current levels or turned off.
 *
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ktd2027);

#include "led_context.h"

/* Registers */
#define KTD2027_REG_ENABLE_RST		0x00	/* Reset Value 0x00 */
#define KTD2027_REG_FLASH_PERIOD	0x01	/* Reset Value 0x00 */
#define KTD2027_REG_FLASH_ON_T1		0x02	/* Reset Value 0x01 */
#define KTD2027_REG_FLASH_ON_T2		0x03	/* Reset Value 0x01 */
#define KTD2027_REG_CH_CONTROL		0x04	/* Reset Value 0x00 */	
#define KTD2027_REG_RAMP_RATE		0x05	/* Reset Value 0x00 */
#define KTD2027_REG_L1_IOUT			0x06	/* Reset Value 0x4F */
#define KTD2027_REG_L2_IOUT			0x07	/* Reset Value 0x4F */
#define KTD2027_REG_L3_IOUT			0x08	/* Reset Value 0x4F */
#define KTD2027_REG_L4_IOUT			0x09	/* Reset Value 0x4F */

#define KTD2027_CH_CONTROL_MASK		0x03

#define KTD2027_CHANNEL_MAX			0x04

/*
 * The maximum flash period is 16.38s
 */
#define KTD2027_MAX_FLASH_PERIOD  16380

/*
 * The minimum flash period is 0.128ms
 */
#define KTD2027_MIN_FLASH_PERIOD 128

/* Brightness limits in percent */
#define KTD2027_MIN_BRIGHTNESS 0
#define KTD2027_MAX_BRIGHTNESS 100

/* LED Current setting increments by 0.125mA */
#define KTD2027_IOUT_STEP_UA	125

/*
 * Available channels. There are four LED channels usable with the KTD2027. While
 * they can be mapped to LEDs of any color, the driver's typical application is
 * with a red, a green, a blue and a white LED. Since the data sheet's
 * nomenclature uses RGBW, we keep it that way.
 */
enum ktd2027_led_channels {
	KTD2027_CHANNEL_B,
	KTD2027_CHANNEL_G,
	KTD2027_CHANNEL_R,
	KTD2027_CHANNEL_W,

	KTD2027_CHANNEL_COUNT,
};

enum ktd2027_function {
	KTD2027_FUNC_TSLOT1,
	KTD2027_FUNC_TSLOT2,
	KTD2027_FUNC_TSLOT3,
	KTD2027_FUNC_TSLOT4,
	KTD2027_FUNC_DO_NOTHING,
	KTD2027_FUNC_RESET_REGS,
	KTD2027_FUNC_RESET_DIGITAL,
	KTD2027_FUNC_RESET_FULL
};

enum ktd2027_turn_on_mode {
	KTD2027_ON_MODE_I2C_DEFAULT,	/* Either SCL or SDA goes low */
	KTD2027_ON_MODE_I2C_TOGGLE,		/* Either SCL goes low or SDA stops toggling */
	KTD2027_ON_MODE_SCL_LOW,		/* SCL goes low */
	KTD2027_ON_MODE_ALWAYS_ON		/* Device always ON */
};

enum ktd2027_ramp_scale{
	KTD2027_RAMP_SCALE_NORMAL,
	KTD2027_RAMP_SCALE_SLOWER_2,
	KTD2027_RAMP_SCALE_SLOWER_4,
	KTD2027_RAMP_SCALE_FASTER_8
};

enum ktd2027_ramp_mode{
	KTD2027_RAMP_LOG_S,				/* Logarithmic-like S ramp up and down curve */
	KTD2027_RAMP_LINEAR			
};

enum ktd2027_led_mode{
	KTD2027_LED_MODE_OFF,
	KTD2027_LED_MODE_ON,
	KTD2027_LED_PWM1,
	KTD2027_LED_PWM2
};


struct ktd2027_config {
	struct i2c_dt_spec bus;
};

struct ktd2027_data {
	struct led_data dev_data;
};

static inline int ktd2027_led_on(const struct device *dev, uint32_t led)
{
	const struct ktd2027_config *config = dev->config;
	
	if (led > KTD2027_CHANNEL_MAX) {
		return -EINVAL;
	}

	uint8_t reg = 0;
	if (i2c_reg_read_byte_dt(&config->bus, KTD2027_REG_CH_CONTROL,&reg))
	{
		LOG_ERR("Failed to read Channel Control from DTK2027");
		return -EIO;
	}
	
	reg |= KTD2027_CH_CONTROL_MASK << led;
	
	if (i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_CH_CONTROL,reg)){
		LOG_ERR("Failed to update Channel Control to DTK2027");
		return -EIO;
	}

	return 0;
}

static inline int ktd2027_led_off(const struct device *dev, uint32_t led)
{
	const struct ktd2027_config *config = dev->config;
	
	if (led > KTD2027_CHANNEL_MAX) {
		return -EINVAL;
	}

	uint8_t reg = 0;
	if (i2c_reg_read_byte_dt(&config->bus, KTD2027_REG_CH_CONTROL,&reg))
	{
		LOG_ERR("Failed to read Channel Control from DTK2027");
		return -EIO;
	}
	
	reg &= ~(KTD2027_CH_CONTROL_MASK << led);
	
	if (i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_CH_CONTROL,reg)){
		LOG_ERR("Failed to update Channel Control to DTK2027");
		return -EIO;
	}

	return 0;
}

static int ktd2027_led_init(const struct device *dev)
{
	const struct ktd2027_config *config = dev->config;
	struct ktd2027_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	/* Hardware specific limits */
	dev_data->min_period = KTD2027_MIN_FLASH_PERIOD;
	dev_data->max_period = KTD2027_MAX_FLASH_PERIOD;
	dev_data->min_brightness = KTD2027_MIN_BRIGHTNESS;
	dev_data->max_brightness = KTD2027_MAX_BRIGHTNESS;

	/* Default Mode: Device always on, Function do nothing */
	if (i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_ENABLE_RST,
		((KTD2027_ON_MODE_ALWAYS_ON << 3)| KTD2027_FUNC_DO_NOTHING))) {
		LOG_ERR("Enabling KTD2027 LED chip failed.");
		return -EIO;
	}

	return 0;
}

static const struct led_driver_api ktd2027_led_api = {
	.on = ktd2027_led_on,
	.off = ktd2027_led_off,
};

#define KTD2027_DEFINE(id)						\
	static const struct ktd2027_config ktd2027_config_##id = {	\
		.bus = I2C_DT_SPEC_INST_GET(id),			\
	};								\
									\
	struct ktd2027_data ktd2027_data_##id;				\
	DEVICE_DT_INST_DEFINE(id, &ktd2027_led_init, NULL,		\
			&ktd2027_data_##id,				\
			&ktd2027_config_##id, POST_KERNEL,		\
			CONFIG_LED_INIT_PRIORITY,			\
			&ktd2027_led_api);				\

DT_INST_FOREACH_STATUS_OKAY(KTD2027_DEFINE)
