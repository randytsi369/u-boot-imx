#ifndef _TSFPGA_H_
#define _TSFPGA_H_

#include <common.h>

#define FPGA_POWER_FAIL 14
#define FPGA_DIO_0 19
#define FPGA_DIO_1 20
#define FPGA_DIO_2 21
#define FPGA_DIO_3 22
#define FPGA_DIO_4 23
#define FPGA_DIO_5 24
#define FPGA_DIO_6 25
#define FPGA_REV_OPS 51
#define FPGA_OPS_P13 (1 << 3)
#define FPGA_OPS_L14 (1 << 2)
#define FPGA_OPS_G12 (1 << 1)
#define FPGA_OPS_H12 (1 << 0)
#define FPGA_OPS2 57
#define FPGA_OPS_OKAYA (1 << 3)
#define FPGA_OPS_LXD (1 << 4)
#define FPGA_ENS 59
#define FPGA_ENS_TOUCHRST (1 << 5)

void fpga_gpio_output(int io, int value);
int fpga_gpio_input(int io);

#endif
