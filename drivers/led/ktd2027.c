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
#include <zephyr/drivers/led/ktd2027.h>
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

#define KTD2027_MAX_IOUT_uA			2400U

/* LED Current setting increments by 0.125mA */
#define KTD2027_IOUT_STEP_uA	125

/*
 * The maximum flash period is 16.38s
 */
#define KTD2027_MAX_FLASH_PERIOD  16380

/*
 * The minimum flash period is 0.128ms
 */
#define KTD2027_MIN_FLASH_PERIOD 128

/* Default blink period 1s */
#define KT2027_DEFAULT_FLASH_PERIOD	2000U

/* Brightness limits in percent */
#define KTD2027_MIN_BRIGHTNESS 0
#define KTD2027_MAX_BRIGHTNESS 100



struct ktd2027_config {
	struct i2c_dt_spec bus;
};

struct ktd2027_data {
	struct led_data dev_data;
};

static uint8_t ktd2027_regs[KTD2027_REG_L4_IOUT+1U];

static inline int ktd2027_led_on(const struct device *dev, uint32_t led)
{
	if (led > KTD2027_CHANNEL_MAX) {
		return -EINVAL;
	}
	
	return ktd2027_set_led_mode(dev,(enum ktd2027_led_channels)led, KTD2027_LED_MODE_ON);
}

static inline int ktd2027_led_off(const struct device *dev, uint32_t led)
{
	if (led > KTD2027_CHANNEL_MAX) {
		return -EINVAL;
	}
	
	return ktd2027_set_led_mode(dev,(enum ktd2027_led_channels)led, KTD2027_LED_MODE_OFF);
}

static inline int ktd2027_set_led_pwm_mode(const struct device *dev, uint32_t led, enum ktd2027_led_mode mode)
{
	if (led > KTD2027_CHANNEL_MAX) {
		return -EINVAL;
	}
	
	if((KTD2027_LED_PWM1 != mode) && (KTD2027_LED_PWM2 != mode))
	{
		return -EINVAL;
	}
	
	return ktd2027_set_led_mode(dev,(enum ktd2027_led_channels)led, mode);
}

int ktd2027_set_reset(const struct device* dev, enum ktd2027_function reset)
{
	const struct ktd2027_config *config = dev->config;
	
	if(reset < KTD2027_FUNC_RESET_REGS)
	{
		return -EINVAL;
	}
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_ENABLE_RST, (uint8_t)reset);
	if (err) {
		LOG_ERR("Reseting device failed. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_scaling(const struct device* dev, enum ktd2027_ramp_scale scale)
{
	const struct ktd2027_config *config = dev->config;
	// Reset scaling
	ktd2027_regs[KTD2027_REG_ENABLE_RST] &= ~(KTD2027_RAMP_SCALE_NORMAL << 5U);
	// Set new scaling
	ktd2027_regs[KTD2027_REG_ENABLE_RST] |= (scale << 5U);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_ENABLE_RST,
		ktd2027_regs[KTD2027_REG_ENABLE_RST]);
	if (err) {
		LOG_ERR("Setting scaling failed. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_enable_mode(const struct device* dev, enum ktd2027_function mode)
{
	const struct ktd2027_config *config = dev->config;
	// Reset scaling
	ktd2027_regs[KTD2027_REG_ENABLE_RST] &= ~(KTD2027_ON_MODE_I2C_DEFAULT << 3U);
	// Set new scaling
	ktd2027_regs[KTD2027_REG_ENABLE_RST] |= (mode < 3U);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_ENABLE_RST,
		ktd2027_regs[KTD2027_REG_ENABLE_RST]);
	if (err) {
		LOG_ERR("Setting scaling failed.Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_flash_period(const struct device* dev, uint16_t period_ms)
{
	const struct ktd2027_config *config = dev->config;
	
	uint8_t bin_period = ((period_ms - KTD2027_MIN_FLASH_PERIOD) / KTD2027_MIN_FLASH_PERIOD);
	
	ktd2027_regs[KTD2027_REG_FLASH_PERIOD] = bin_period & 0x7FU;
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_FLASH_PERIOD,
		ktd2027_regs[KTD2027_REG_FLASH_PERIOD]);
	if (err) {
		LOG_ERR("Setting flash period failed.Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_ramp_mode(const struct device* dev, enum ktd2027_ramp_mode r_mode)
{
	const struct ktd2027_config *config = dev->config;
	
	if(KTD2027_RAMP_LOG_S == r_mode)
	{
		ktd2027_regs[KTD2027_REG_CH_CONTROL] &= ~(ktd2027_regs[KTD2027_REG_CH_CONTROL] & r_mode << 7U);
	}
	else
	{
		ktd2027_regs[KTD2027_REG_CH_CONTROL] |= ktd2027_regs[KTD2027_REG_CH_CONTROL] & (r_mode << 7U);
	}
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_CH_CONTROL,
		ktd2027_regs[KTD2027_REG_CH_CONTROL]);
	if (err) {
		LOG_ERR("Setting ramp mode failed. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_period_on_duty(const struct device* dev, uint32_t on_time)
{	
	const struct ktd2027_config *config = dev->config;
	ktd2027_regs[KTD2027_REG_FLASH_ON_T1] = ((ktd2027_regs[KTD2027_REG_FLASH_PERIOD] * 100U)/on_time);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_FLASH_ON_T1,
		ktd2027_regs[KTD2027_REG_FLASH_ON_T1]);
	if (err) {
		LOG_ERR("Setting flash duty failed. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_led_mode(const struct device* dev, enum ktd2027_led_channels led, enum ktd2027_led_mode on_mode)
{
	const struct ktd2027_config *config = dev->config;
	
	if (led >= KTD2027_CHANNEL_MAX) {
		return -EINVAL;
	}
	
	// LED OFF
	ktd2027_regs[KTD2027_REG_CH_CONTROL] &= ~(KTD2027_LED_MODE_OFF << (led * 2U));
	// LED PWM
	ktd2027_regs[KTD2027_REG_CH_CONTROL] |= on_mode << (led * 2U);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_CH_CONTROL,ktd2027_regs[KTD2027_REG_CH_CONTROL]);
	if (err){
		LOG_ERR("Failed to update Channel mode to DTK2027, Err %d",err);
		return -EIO;
	}

	return 0;
}

int ktd2027_set_ramp_rate(const struct device* dev, uint16_t rise_period, uint16_t fall_period)
{
	const struct ktd2027_config *config = dev->config;
	
	uint8_t r_scale = ((((ktd2027_regs[KTD2027_REG_ENABLE_RST]>>5U) & 0x3U) + 1U) * 2U);
	
	uint8_t trise = 0;
	uint8_t tfall = 0;
	if(r_scale == 8U)
	{
		
	}
	else
	{
		trise = ((rise_period/KTD2027_MIN_FLASH_PERIOD)/r_scale) & 0xFU;
		tfall = ((fall_period/KTD2027_MIN_FLASH_PERIOD)/r_scale) & 0xFU;
	}
	
	ktd2027_regs[KTD2027_REG_RAMP_RATE] = (tfall << 4U) | trise;
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_RAMP_RATE,ktd2027_regs[KTD2027_REG_RAMP_RATE]);
	if (err){
		LOG_ERR("Failed to update ramp rates DTK2027. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_max_current(const struct device* dev, enum ktd2027_led_channels led, uint8_t current_ma)
{
	const struct ktd2027_config *config = dev->config;
	
	if(current_ma > KTD2027_MAX_IOUT_uA)
	{
		return -EINVAL;
	}
	
	uint8_t reg_idx = KTD2027_REG_L1_IOUT;
	switch(led)
	{
		case KTD2027_CHANNEL_B:
		{
			reg_idx = KTD2027_REG_L3_IOUT;
			break;
		}
		case KTD2027_CHANNEL_G:
		{
			reg_idx = KTD2027_REG_L2_IOUT;
			break;
		}
		case KTD2027_CHANNEL_W:
		{
			reg_idx = KTD2027_REG_L4_IOUT;
			break;
		}
		default:
		{
			return -EINVAL;
		}
	}
	
	ktd2027_regs[reg_idx] = ((current_ma * 1000U) / KTD2027_IOUT_STEP_uA);;
	int err = i2c_reg_write_byte_dt(&config->bus, reg_idx,ktd2027_regs[reg_idx]);
	if (err){
		LOG_ERR("Failed to update Iout register DTK2027. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

static int ktd2027_blink(const struct device* dev, uint32_t led,
				uint32_t delay_on, uint32_t delay_off)
{
	if (led > KTD2027_CHANNEL_MAX) {
		return -EINVAL;
	}
	
	ktd2027_set_led_pwm_mode(dev,led,KTD2027_LED_PWM1);
	ktd2027_set_flash_period(dev,KT2027_DEFAULT_FLASH_PERIOD);
	ktd2027_set_period_on_duty(dev,delay_off);
	
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
	ktd2027_regs[KTD2027_REG_ENABLE_RST] = ((KTD2027_ON_MODE_ALWAYS_ON << 3)| KTD2027_FUNC_DO_NOTHING);

	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_ENABLE_RST,
		ktd2027_regs[KTD2027_REG_ENABLE_RST]);
	if (err) {
		LOG_ERR("Enabling KTD2027 LED chip failed. Err %d",err);
		return -EIO;
	}
	
	/* All LEDs of at power up */
	ktd2027_led_off(dev,0U);
	ktd2027_led_off(dev,1U);
	ktd2027_led_off(dev,2U);
	ktd2027_led_off(dev,3U);

	return 0;
}

static const struct led_driver_api ktd2027_led_api = {
	.on = ktd2027_led_on,
	.off = ktd2027_led_off,
	.blink = ktd2027_blink,
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
