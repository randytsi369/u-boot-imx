#include <common.h>
#include <i2c.h>

int fpga_get_rev(void)
{
	uint8_t val = 0;
	i2c_read(0x28, 306, 2, &val, 1);

	return val;
}

void fpga_gpio_output(int io, int value)
{
	uint8_t val;

	// OE
	val = 0x1;
	// DATA
	if(value) val |= 0x2;

	i2c_write(0x28, io, 2, &val, 1);
}

int fpga_gpio_input(int io)
{
	uint8_t val = 0;
	// Set input with 0
	i2c_write(0x28, io, 2, &val, 1);
	i2c_read(0x28, io, 2, &val, 1);
	return (val & 0x4) ? 1 : 0;
}
