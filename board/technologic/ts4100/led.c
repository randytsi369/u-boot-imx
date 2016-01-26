#include <common.h>
#include <i2c.h>
#include <status_led.h>

static unsigned int saved_state[3] = {STATUS_LED_OFF,
	STATUS_LED_OFF, STATUS_LED_OFF};

void red_led_on(void)
{
	uint8_t val;
	i2c_read(0x28, 59, 2, &val, 1);
	val &= ~(1 << 2);
	i2c_write(0x28, 59, 2, &val, 1);

	saved_state[STATUS_LED_RED] = STATUS_LED_ON;
}

void red_led_off(void)
{
	uint8_t val;
	i2c_read(0x28, 59, 2, &val, 1);
	val |= (1 << 2);
	i2c_write(0x28, 59, 2, &val, 1);

	saved_state[STATUS_LED_RED] = STATUS_LED_OFF;
}

void green_led_on(void)
{
	uint8_t val;
	i2c_read(0x28, 59, 2, &val, 1);
	val &= ~(1 << 1);
	
	i2c_write(0x28, 59, 2, &val, 1);

	saved_state[STATUS_LED_GREEN] = STATUS_LED_ON;
}

void green_led_off(void)
{
	uint8_t val;
	i2c_read(0x28, 59, 2, &val, 1);
	val |= (1 << 1);
	i2c_write(0x28, 59, 2, &val, 1);

	saved_state[STATUS_LED_GREEN] = STATUS_LED_OFF;
}

void __led_init(led_id_t mask, int state)
{
	/* LEDs are all off of the FPGA I2C Regs */
	i2c_set_bus_num(2);

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

	}
}
