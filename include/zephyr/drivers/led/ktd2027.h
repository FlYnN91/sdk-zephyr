/*
 * Copyright (c) 2023 Dainius Narsutis <dainius@dainiusnarsutis.co.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_LED_KTD2027_H_
#define ZEPHYR_DRIVERS_LED_KTD2027_H_

/*
 * Available channels. There are four LED channels usable with the KTD2027. While
 * they can be mapped to LEDs of any color, the driver's typical application is
 * with a red, a green, a blue and a white LED. Since the data sheet's
 * nomenclature uses RGBW, we keep it that way.
 */
enum ktd2027_led_channels {
	KTD2027_CHANNEL_R,
	KTD2027_CHANNEL_G,
	KTD2027_CHANNEL_B,
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

int ktd2027_set_reset(const struct device* dev, enum ktd2027_function reset);
int ktd2027_set_scaling(const struct device* dev, enum ktd2027_ramp_scale scale);
int ktd2027_set_enable_mode(const struct device* dev, enum ktd2027_function mode);
int ktd2027_set_flash_period(const struct device* dev, uint16_t period_ms);
int ktd2027_set_ramp_mode(const struct device* dev, enum ktd2027_ramp_mode r_mode);
int ktd2027_set_period_on_duty(const struct device* dev, uint32_t on_time);
int ktd2027_set_led_mode(const struct device* dev, enum ktd2027_led_channels led, enum ktd2027_led_mode on_mode);
int ktd2027_set_ramp_rate(const struct device* dev, uint16_t rise_period, uint16_t fall_period);
int ktd2027_set_max_current(const struct device* dev, enum ktd2027_led_channels led, uint8_t current_ma);



#endif /* ZEPHYR_DRIVERS_LED_KTD2027_H_ */