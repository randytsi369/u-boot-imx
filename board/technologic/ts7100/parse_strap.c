/*
 * Copyright (C) 2019 Technologic Systems
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>

uint8_t parse_strap(const char *env_name)
{
	static uint8_t opts;
	static uint8_t read;

	if(!read) {
		gpio_direction_input(IMX_GPIO_NR(4, 13));
		gpio_direction_input(IMX_GPIO_NR(1, 20));
		gpio_direction_input(IMX_GPIO_NR(1, 30));
		gpio_direction_input(IMX_GPIO_NR(1, 28));
		gpio_direction_input(IMX_GPIO_NR(1, 24));
		gpio_direction_input(IMX_GPIO_NR(4, 14));
		gpio_direction_input(IMX_GPIO_NR(3, 13));
		gpio_direction_input(IMX_GPIO_NR(1, 26));

		mdelay(1);

		opts |= (gpio_get_value(IMX_GPIO_NR(4, 13)) << 0);
		opts |= (gpio_get_value(IMX_GPIO_NR(1, 20)) << 1);
		opts |= (gpio_get_value(IMX_GPIO_NR(1, 30)) << 2);
		opts |= (gpio_get_value(IMX_GPIO_NR(1, 28)) << 3);
		opts |= (gpio_get_value(IMX_GPIO_NR(1, 24)) << 4);
		opts |= (gpio_get_value(IMX_GPIO_NR(4, 14)) << 5);
		opts |= (gpio_get_value(IMX_GPIO_NR(3, 13)) << 6);
		opts |= (gpio_get_value(IMX_GPIO_NR(1, 26)) << 7);

		read = 1;
	}
	if (env_name) setenv_hex(env_name, opts);
	return opts;
}
