/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/arch/mx6-pins.h>
#include <status_led.h>
#include <asm/imx-common/iomux-v3.h>




static iomux_v3_cfg_t const led_pads[] = {

   MX6_PAD_CSI_DATA02__GPIO4_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),
   MX6_PAD_CSI_DATA03__GPIO4_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),
   MX6_PAD_CSI_DATA04__GPIO4_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),
   MX6_PAD_CSI_DATA05__GPIO4_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL),   
};

#define EN_YEL_LEDN		IMX_GPIO_NR(4, 23)
#define EN_RED_LEDN		IMX_GPIO_NR(4, 24)
#define EN_GREEN_LEDN	IMX_GPIO_NR(4, 25)
#define EN_BLUE_LED		IMX_GPIO_NR(4, 26)


static unsigned int saved_state[4] = {
   STATUS_LED_OFF,
	STATUS_LED_OFF,
	STATUS_LED_OFF,
	STATUS_LED_OFF
};
									  
void red_led_on(void)
{
   gpio_set_value(EN_RED_LEDN, 0);	
	saved_state[STATUS_LED_RED] = STATUS_LED_ON;
}

void red_led_off(void)
{
	gpio_set_value(EN_RED_LEDN, 1);
	saved_state[STATUS_LED_RED] = STATUS_LED_OFF;
}

void blue_led_on(void)
{
	gpio_set_value(EN_BLUE_LED, 1);
	saved_state[STATUS_LED_BLUE] = STATUS_LED_ON;
}

void blue_led_off(void)
{
	gpio_set_value(EN_BLUE_LED, 0);
	saved_state[STATUS_LED_BLUE] = STATUS_LED_OFF;
}

void yellow_led_on(void)
{
	gpio_set_value(EN_YEL_LEDN, 0);
	saved_state[STATUS_LED_YELLOW] = STATUS_LED_ON;
}

void yellow_led_off(void)
{
	gpio_set_value(EN_YEL_LEDN, 1);
	saved_state[STATUS_LED_YELLOW] = STATUS_LED_OFF;
}

void green_led_on(void)
{
	gpio_set_value(EN_GREEN_LEDN, 0);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_ON;
}

void green_led_off(void)
{
	gpio_set_value(EN_GREEN_LEDN, 1);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_OFF;
}

void __led_init(led_id_t mask, int state)
{
   imx_iomux_v3_setup_multiple_pads(led_pads,
					 ARRAY_SIZE(led_pads));

   gpio_direction_output(EN_YEL_LEDN, 1);
   gpio_direction_output(EN_GREEN_LEDN, 1);
   gpio_direction_output(EN_RED_LEDN, 1);
   gpio_direction_output(EN_BLUE_LED, 0);

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