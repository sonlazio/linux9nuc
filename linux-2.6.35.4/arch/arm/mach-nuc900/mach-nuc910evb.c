/*
 * linux/arch/arm/mach-w90x900/mach-nuc910evb.c
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
#include <mach/fb.h>

#include <mach/irqs.h>
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <mach/regs-aic.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2007.h>
#include "nuc910.h"
/////////////////////////////////////////////////////////////
//// tsc2007 touch driver support
/////////////////////////////////////////////////////////////
static int nuc900_get_pendown_state(void)
{
        int val;

        //get GPIO-H0 status
        val = readl(REG_GPIOH_DATAIN) & 0x1;

        val = val ? 0 : 1;
        return val;
}

extern void nuc900_enable_group_irq(int src);
static int nuc900_init_ts(void)
{
        //set GPIO-H0 to input mode
        writel(readl(REG_GPIOH_DIR) &  ~0x1, REG_GPIOH_DIR);
        //PENIRQ(nIRQ0) is low sensitive
        writel(readl(REG_AIC_IRQSC) & ~0x3, REG_AIC_IRQSC);
	nuc900_enable_group_irq(IRQ_GROUP0_IRQ0);

        return 0;
}

struct tsc2007_platform_data nuc900_tsc2007_data = {
        .model = 2007,
        .x_plate_ohms = 180,
        .get_pendown_state = nuc900_get_pendown_state,
        .init_platform_hw = nuc900_init_ts,
};

/* I2C clients */
static struct i2c_board_info __initdata nuc900_i2c_clients[] = {
        {
                I2C_BOARD_INFO("nau8822", 0x1a),
        },
#ifdef CONFIG_EEPROM_AT24
        {
                I2C_BOARD_INFO("at24", 0x50),
                .type="24c16",
        },        
#endif
        {
                I2C_BOARD_INFO("tsc2007", 0x48),
                .type = "tsc2007",
                .platform_data = &nuc900_tsc2007_data,
                .irq = IRQ_GROUP0,
        },
#ifdef CONFIG_RTC_DRV_PCF8563        
        {
		I2C_BOARD_INFO("rtc-pcf8563", 0x51),
		.type = "pcf8563",
        },
#endif        
#ifdef CONFIG_RTC_DRV_HT1382
        {
                I2C_BOARD_INFO("rtc-ht1382", 0x68),
                .type = "ht1382",
        },        
#endif
};
static void __init nuc910evb_map_io(void)
{
	nuc910_map_io();
	nuc910_init_clocks();
}

static void __init nuc910evb_init(void)
{
	nuc910_board_init();
        i2c_register_board_info(0, nuc900_i2c_clients, sizeof(nuc900_i2c_clients)/sizeof(struct i2c_board_info));
}

MACHINE_START(W90X900, "W90P910EVB")
	/* Maintainer: Wan ZongShun */
	.phys_io	= W90X900_PA_UART,
	.io_pg_offst	= (((u32)W90X900_VA_UART) >> 18) & 0xfffc,
	.boot_params	= 0x100,
	.map_io		= nuc910evb_map_io,
	.init_irq	= nuc900_init_irq,
	.init_machine	= nuc910evb_init,
	.timer		= &nuc900_timer,
MACHINE_END
