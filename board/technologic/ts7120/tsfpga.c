#include <common.h>

#include <asm/gpio.h>
#include <command.h>
#include <status_led.h>
#include <asm/io.h>

#include "tsfpga.h"

#define FPGA_RESET_GPIO		IMX_GPIO_NR(4, 17)

/* The bootup FPGA has basic code needed for startup, but when poked
 * at FPGA_RELOAD it will reload itself from 0xf0000 on the ASMI accessible
 * SPI flash.  This will have the application load of the FPGA intended for Linux
 */
static int do_tsfpga_reload(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	writew((1<<15), FPGA_RELOAD);
	/* With the current revision we can only delay long enough until the FPGA
	 * has likely been reloaded.  On new revs we will poll for completion and
	 * a delayed FPGA_DONE will cause FPGA_RESET#. */
	mdelay(500);
	gpio_direction_output(FPGA_RESET_GPIO, 0);

	return 0;
}

U_BOOT_CMD(tsfpga, 2, 1, do_tsfpga_reload,
	"Boot from factory FPGA to application load",
	"Returns 0 on success"
);
