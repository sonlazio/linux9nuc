/*
 * linux/arch/arm/mach-w90x900/nuc946.c
 *
 * Based on linux/arch/arm/plat-s3c24xx/s3c244x.c by Ben Dooks
 *
 * Copyright (c) 2008 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * NUC946 cpu support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>

#include "cpu.h"

/* define specific CPU platform device */

static struct platform_device *nuc946_dev[] __initdata = {
	&nuc900_device_fmi,
};

/* define specific CPU platform io map */

static struct map_desc nuc946evb_iodesc[] __initdata = {
        IODESC_ENT(I2C),
        IODESC_ENT(EMC),        
};

/*Init NUC946 evb io*/

void __init nuc946_map_io(void)
{
	nuc900_map_io(nuc946evb_iodesc, ARRAY_SIZE(nuc946evb_iodesc));
}

/*Init NUC946 clock*/

void __init nuc946_init_clocks(void)
{
	nuc900_init_clocks();
}

/*Init NUC946 board info*/

void __init nuc946_board_init(void)
{
	nuc900_board_init(nuc946_dev, ARRAY_SIZE(nuc946_dev));
}
