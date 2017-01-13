/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <status_led.h>
#include "fpga.h"

static unsigned int saved_state[4] = {STATUS_LED_OFF,
									  STATUS_LED_OFF,
									  STATUS_LED_OFF,
									  STATUS_LED_OFF};

void red_led_on(void)
{
	fpga_gpio_output(EN_RED_LEDN, 0);
	saved_state[STATUS_LED_RED] = STATUS_LED_ON;
}

void red_led_off(void)
{
	fpga_gpio_output(EN_RED_LEDN, 1);
	saved_state[STATUS_LED_RED] = STATUS_LED_OFF;
}

void blue_led_on(void)
{
	fpga_gpio_output(EN_BLUE_LED, 1);
	saved_state[STATUS_LED_BLUE] = STATUS_LED_ON;
}

void blue_led_off(void)
{
	fpga_gpio_output(EN_BLUE_LED, 0);
	saved_state[STATUS_LED_BLUE] = STATUS_LED_OFF;
}

void yellow_led_on(void)
{
	fpga_gpio_output(EN_YEL_LEDN, 0);
	saved_state[STATUS_LED_YELLOW] = STATUS_LED_ON;
}

void yellow_led_off(void)
{
	fpga_gpio_output(EN_YEL_LEDN, 1);
	saved_state[STATUS_LED_YELLOW] = STATUS_LED_OFF;
}

void green_led_on(void)
{
	fpga_gpio_output(EN_GREEN_LEDN, 0);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_ON;
}

void green_led_off(void)
{
	fpga_gpio_output(EN_GREEN_LEDN, 1);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_OFF;
}

void __led_init(led_id_t mask, int state)
{
	__led_set(mask, state);
}

void __led_toggle(led_id_t mask)
{
	if (STATUS_LED_RED == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_RED])
			red_led_off();
		else
			red_led_on();

	} else if (STATUS_LED_GREEN == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_GREEN])
			green_led_off();
		else
			green_led_on();

	} else if (STATUS_LED_BLUE == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_BLUE])
			blue_led_off();
		else
			blue_led_on();

	} else if (STATUS_LED_YELLOW == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_YELLOW])
			yellow_led_off();
		else
			yellow_led_on();

	}
}

void __led_set(led_id_t mask, int state)
{
	if (STATUS_LED_RED == mask) {
		if (STATUS_LED_ON == state)
			red_led_on();
		else
			red_led_off();

	} else if (STATUS_LED_GREEN == mask) {
		if (STATUS_LED_ON == state)
			green_led_on();
		else
			green_led_off();
	} else if (STATUS_LED_BLUE == mask) {
		if (STATUS_LED_ON == state)
			blue_led_on();
		else
			blue_led_off();
	} else if (STATUS_LED_YELLOW == mask) {
		if (STATUS_LED_ON == state)
			yellow_led_on();
		else
			yellow_led_off();
	}
}
