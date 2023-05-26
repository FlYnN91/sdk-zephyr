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
#define KTD2027_REG_COUNT			(KTD2027_REG_L4_IOUT+1U)

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
	DEVICE_MMIO_ROM;
	struct i2c_dt_spec bus;
};

struct ktd2027_data {
	struct led_data dev_data;
	uint8_t regs[KTD2027_REG_COUNT];
};

static inline int ktd2027_led_on(const struct device *dev, uint32_t led)
{
	if (led > KTD2027_CHANNEL_MAX) {
		LOG_ERR("Invalid LED Channel %d",led);
		return -EINVAL;
	}
	
	return ktd2027_set_led_mode(dev,(enum ktd2027_led_channels)led, KTD2027_LED_MODE_ON);
}

static inline int ktd2027_led_off(const struct device *dev, uint32_t led)
{
	if (led > KTD2027_CHANNEL_MAX) {
		LOG_ERR("Invalid LED Channel %d",led);
		return -EINVAL;
	}
	
	LOG_DBG("LED %d Off",led);
	
	return ktd2027_set_led_mode(dev,(enum ktd2027_led_channels)led, KTD2027_LED_MODE_OFF);
}

static inline int ktd2027_set_led_pwm_mode(const struct device *dev, uint32_t led, enum ktd2027_led_mode mode)
{
	if (led > KTD2027_CHANNEL_MAX) {
		LOG_ERR("Invalid LED Channel %d",led);
		return -EINVAL;
	}
	
	if((KTD2027_LED_PWM1 != mode) && (KTD2027_LED_PWM2 != mode))
	{
		LOG_ERR("Invalid LED PWM Mode %d",mode);
		return -EINVAL;
	}
	
	LOG_DBG("Changing LED %d Mode",led);
	
	return ktd2027_set_led_mode(dev,(enum ktd2027_led_channels)led, mode);
}

int ktd2027_set_reset(const struct device* dev, enum ktd2027_function reset)
{
	const struct ktd2027_config *config = dev->config;
	
	if(reset < KTD2027_FUNC_RESET_REGS)
	{
		LOG_ERR("Invalid Reset Function %d",reset);
		return -EINVAL;
	}
	LOG_DBG("Resetting Device Mode %d",reset);
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
	struct ktd2027_data *data = dev->data;
	
	// Reset scaling
	data->regs[KTD2027_REG_ENABLE_RST] &= ~(3U << 5U);
	// Set new scaling
	data->regs[KTD2027_REG_ENABLE_RST] |= (scale << 5U);
	
	LOG_DBG("Setting Scaling %d [0x%08X]",scale,data->regs[KTD2027_REG_ENABLE_RST]);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_ENABLE_RST,
		data->regs[KTD2027_REG_ENABLE_RST]);
	if (err) {
		LOG_ERR("Setting scaling failed. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_enable_mode(const struct device* dev, enum ktd2027_function mode)
{
	const struct ktd2027_config *config = dev->config;
	struct ktd2027_data *data = dev->data;
	
	// Reset scaling
	data->regs[KTD2027_REG_ENABLE_RST] &= ~(3U << 3U);
	// Set new scaling
	data->regs[KTD2027_REG_ENABLE_RST] |= (mode < 3U);
	
	LOG_DBG("Setting Enable Mode %d [0x%08X]",mode,data->regs[KTD2027_REG_ENABLE_RST]);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_ENABLE_RST,
		data->regs[KTD2027_REG_ENABLE_RST]);
	if (err) {
		LOG_ERR("Setting scaling failed.Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_flash_period(const struct device* dev, uint16_t period_ms)
{
	const struct ktd2027_config *config = dev->config;
	struct ktd2027_data *data = dev->data;
	
	uint8_t bin_period = ((period_ms - KTD2027_MIN_FLASH_PERIOD) / KTD2027_MIN_FLASH_PERIOD);
	
	data->regs[KTD2027_REG_FLASH_PERIOD] = bin_period & 0x7FU;
	
	LOG_DBG("Setting Flash Period %d [0x%08X]",period_ms,data->regs[KTD2027_REG_FLASH_PERIOD]);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_FLASH_PERIOD,
		data->regs[KTD2027_REG_FLASH_PERIOD]);
	if (err) {
		LOG_ERR("Setting flash period failed.Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_ramp_mode(const struct device* dev, enum ktd2027_ramp_mode r_mode)
{
	const struct ktd2027_config *config = dev->config;
	struct ktd2027_data *data = dev->data;
	
	if(KTD2027_RAMP_LOG_S == r_mode)
	{
		data->regs[KTD2027_REG_FLASH_PERIOD] &= ~(1U << 7U);
	}
	else
	{
		data->regs[KTD2027_REG_FLASH_PERIOD] |= (r_mode << 7U);
	}
	LOG_DBG("Setting Ramp Mode %d [0x%08X]",r_mode,data->regs[KTD2027_REG_FLASH_PERIOD]);
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_FLASH_PERIOD,
		data->regs[KTD2027_REG_FLASH_PERIOD]);
	if (err) {
		LOG_ERR("Setting ramp mode failed. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_period_on_duty(const struct device* dev, uint32_t on_time)
{	
	const struct ktd2027_config *config = dev->config;
	struct ktd2027_data *data = dev->data;
	
	data->regs[KTD2027_REG_FLASH_ON_T1] = (255U / ( 100U / on_time));
	
	LOG_DBG("Setting T1 Period Duty %d [0x%08X]",on_time,data->regs[KTD2027_REG_FLASH_ON_T1]);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_FLASH_ON_T1,
		data->regs[KTD2027_REG_FLASH_ON_T1]);
	if (err) {
		LOG_ERR("Setting flash duty failed. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_led_mode(const struct device* dev, enum ktd2027_led_channels led, enum ktd2027_led_mode on_mode)
{
	const struct ktd2027_config *config = dev->config;
	struct ktd2027_data *data = dev->data;
	
	if (led >= KTD2027_CHANNEL_MAX) {
		return -EINVAL;
	}
	
	// LED OFF
	data->regs[KTD2027_REG_CH_CONTROL] &= ~(3U << (led * 2U));
	// LED PWM
	data->regs[KTD2027_REG_CH_CONTROL] |= on_mode << (led * 2U);
	
	LOG_DBG("Setting LED %d Mode %d [0x%08X]",led,on_mode,data->regs[KTD2027_REG_CH_CONTROL]);
	
	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_CH_CONTROL,data->regs[KTD2027_REG_CH_CONTROL]);
	if (err){
		LOG_ERR("Failed to update Channel mode to DTK2027, Err %d",err);
		return -EIO;
	}

	return 0;
}

int ktd2027_set_ramp_rate(const struct device* dev, uint16_t rise_period, uint16_t fall_period)
{
	const struct ktd2027_config *config = dev->config;
	struct ktd2027_data *data = dev->data;
	
	uint8_t r_scale = ((data->regs[KTD2027_REG_ENABLE_RST]>>5U) & 0x3U);
	uint16_t r_min, r_max = 0U;
	uint16_t trise = 0;
	uint16_t tfall = 0;
	uint16_t power = 1U;
	switch(r_scale)
	{
		default:
		case KTD2027_RAMP_SCALE_NORMAL:
		{
			power = 1U;
			r_min = KTD2027_MIN_FLASH_PERIOD;
			break;
		}
		case KTD2027_RAMP_SCALE_SLOWER_2:
		{
			power = 2U;
			r_min = KTD2027_MIN_FLASH_PERIOD * 2U;
			break;
		}
		case KTD2027_RAMP_SCALE_SLOWER_4:
		{
			power = 4U;
			r_min = KTD2027_MIN_FLASH_PERIOD * 3U;
			break;
		}
		case KTD2027_RAMP_SCALE_FASTER_8:
		{
			power = 8U;
			r_min = KTD2027_MIN_FLASH_PERIOD / 8U;
			break;
		}
	}
	
	r_max = r_min * 15U;

	if(r_scale != KTD2027_RAMP_SCALE_FASTER_8)
	{
		trise = (rise_period + KTD2027_MIN_FLASH_PERIOD - 1U)/ KTD2027_MIN_FLASH_PERIOD / power;
		tfall = (fall_period + KTD2027_MIN_FLASH_PERIOD - 1U)/ KTD2027_MIN_FLASH_PERIOD / power;
	}
	else
	{
		trise = (rise_period /r_min);
		tfall = (fall_period /r_min);
	}
	
	data->regs[KTD2027_REG_RAMP_RATE] = ((tfall & 0xFFU) << 4U) | (trise& 0xFFU);
	
	LOG_DBG("Setting Rise (%d) %d, Fall (%d) %d, [0x%08X]",trise,rise_period,tfall,fall_period,data->regs[KTD2027_REG_RAMP_RATE]);

	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_RAMP_RATE,data->regs[KTD2027_REG_RAMP_RATE]);
	if (err){
		LOG_ERR("Failed to update ramp rates DTK2027. Err %d",err);
		return -EIO;
	}
	
	return 0;
}

int ktd2027_set_max_current(const struct device* dev, enum ktd2027_led_channels led, uint8_t current_ma)
{
	const struct ktd2027_config *config = dev->config;
	struct ktd2027_data *data = dev->data;
	
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
	
	data->regs[reg_idx] = ((current_ma * 1000U) / KTD2027_IOUT_STEP_uA);
	
	LOG_DBG("Setting LED %d mA %d, [0x%08X]",led,current_ma,data->regs[reg_idx]);
	
	int err = i2c_reg_write_byte_dt(&config->bus, reg_idx,data->regs[reg_idx]);
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
	
	LOG_DBG("LED %d, Blink %d -> %d",led, delay_on, delay_off);
	
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
	
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	/* Hardware specific limits */
	dev_data->min_period = KTD2027_MIN_FLASH_PERIOD;
	dev_data->max_period = KTD2027_MAX_FLASH_PERIOD;
	dev_data->min_brightness = KTD2027_MIN_BRIGHTNESS;
	dev_data->max_brightness = KTD2027_MAX_BRIGHTNESS;
	
	/* Default Mode: Device always on, Function full reset */
	data->regs[KTD2027_REG_ENABLE_RST] = (uint8_t)((KTD2027_ON_MODE_ALWAYS_ON << 3U)| KTD2027_FUNC_DO_NOTHING);

	LOG_DBG("Resetting KTD2027 [0x%01X]",data->regs[KTD2027_REG_ENABLE_RST]);

	int err = i2c_reg_write_byte_dt(&config->bus, KTD2027_REG_ENABLE_RST, data->regs[KTD2027_REG_ENABLE_RST]);
	if (err) {
		LOG_ERR("Enabling KTD2027 LED chip failed. Err %d",err);
		return -EIO;
	}
	
	k_sleep(K_MSEC(10));
	
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

#define KTD2027(idx) DT_NODELABEL(ktd2027##idx)

#define KTD2027_DEFINE(id)						\
	static const struct ktd2027_config ktd2027_config_##id = {	\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(id)),\
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
