/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>

void fpga_gpio_output(int io, int value)
{
	uint8_t val;

	// Crossbar
	val = 0x7c;

	// OE
	val |= 0x1;
	// DATA
	if(value) val |= 0x2;
	i2c_set_bus_num(0);
	i2c_write(0x28, io, 2, &val, 1);
}

int fpga_gpio_input(int io)
{
	uint8_t val = 0x7c;
	// Set input with 0
	i2c_set_bus_num(0);
	i2c_write(0x28, io, 2, &val, 1);
	i2c_read(0x28, io, 2, &val, 1);
	return (val & 0x2) ? 1 : 0;
}
