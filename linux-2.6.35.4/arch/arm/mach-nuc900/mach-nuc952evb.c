/*
 * linux/arch/arm/mach-w90x900/mach-nuc952evb.c
 *
 * Based on mach-s3c2410/mach-smdk2410.c by Jonas Dietsche
 *
 * Copyright (C) 2008 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation;version 2 of the License.
 *   history:
 *     Wang Qiang (rurality.linux@gmail.com) add LCD support
 *
 */

#include <linux/platform_device.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <mach/map.h>

#include <mach/irqs.h>
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <mach/regs-aic.h>
#include <linux/i2c.h>

#include "nuc952.h"

/* I2C clients */
static struct i2c_board_info __initdata nuc900_i2c_clients[] = {
        {
                I2C_BOARD_INFO("nau8822", 0x1a),
        },
        {
		I2C_BOARD_INFO("rtc-pcf8563", 0x51),
		.type = "pcf8563",
        },
};

static void __init nuc952evb_map_io(void)
{
	nuc952_map_io();
	nuc952_init_clocks();
}

static void __init nuc952evb_init(void)
{
	nuc952_board_init();
        i2c_register_board_info(0, nuc900_i2c_clients, sizeof(nuc900_i2c_clients)/sizeof(struct i2c_board_info));
}

MACHINE_START(W90X900, "W90P952EVB")
	/* Maintainer: Wan ZongShun */
	.phys_io	= W90X900_PA_UART,
	.io_pg_offst	= (((u32)W90X900_VA_UART) >> 18) & 0xfffc,
	.boot_params	= 0x100,
	.map_io		= nuc952evb_map_io,
	.init_irq	= nuc900_init_irq,
	.init_machine	= nuc952evb_init,
	.timer		= &nuc900_timer,
MACHINE_END
