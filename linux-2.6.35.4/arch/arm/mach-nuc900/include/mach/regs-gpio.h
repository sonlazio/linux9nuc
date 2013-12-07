/* linux/include/asm-arm/arch-nuc900/nuc900_reg.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2006/08/26     vincen.zswan add this file for nuvoton nuc900 MCU ip REG.
 */

#ifndef __ASM_ARCH_REGS_GPIO_H
#define __ASM_ARCH_REGS_GPIO_H

#include <mach/map.h>
#include <mach/regs-gcr.h>
#define GPIO_BA   W90X900_VA_GPIO /* GPIO Control */

/* GPIO Control Registers */
#define REG_GPIOC_DIR		(GPIO_BA+0x04)  /* GPIO portC direction control register */
#define REG_GPIOC_DATAOUT	(GPIO_BA+0x08)  /* GPIO portC data output register */
#define REG_GPIOC_DATAIN	(GPIO_BA+0x0C)  /* GPIO portC data input register */
#define REG_GPIOD_DIR		(GPIO_BA+0x14)  /* GPIO portD direction control register */
#define REG_GPIOD_DATAOUT	(GPIO_BA+0x18)  /* GPIO portD data output register */
#define REG_GPIOD_DATAIN	(GPIO_BA+0x1C)  /* GPIO portD data input register */
#define REG_GPIOE_DIR		(GPIO_BA+0x24)  /* GPIO portE direction control register */
#define REG_GPIOE_DATAOUT	(GPIO_BA+0x28)  /* GPIO portE data output register */
#define REG_GPIOE_DATAIN	(GPIO_BA+0x2C)  /* GPIO portE data input register */
#define REG_GPIOF_DIR		(GPIO_BA+0x34)  /* GPIO portF direction control register */
#define REG_GPIOF_DATAOUT	(GPIO_BA+0x38)  /* GPIO portF data output register */
#define REG_GPIOF_DATAIN	(GPIO_BA+0x3C)  /* GPIO portF data input register */
#define REG_GPIOG_DIR		(GPIO_BA+0x44)  /* GPIO portG direction control register */
#define REG_GPIOG_DATAOUT	(GPIO_BA+0x48)  /* GPIO portG data output register */
#define REG_GPIOG_DATAIN	(GPIO_BA+0x4C)  /* GPIO portG data input register */
#define REG_GPIOH_DBNCE		(GPIO_BA+0x50)  /* GPIO portH input debounce control reg. */
#define REG_GPIOH_DIR		(GPIO_BA+0x54)  /* GPIO portH direction control register */
#define REG_GPIOH_DATAOUT	(GPIO_BA+0x58)  /* GPIO portH data output register */
#define REG_GPIOH_DATAIN	(GPIO_BA+0x5C)  /* GPIO portH data input register */
#define REG_GPIOI_DIR		(GPIO_BA+0x64)  /* GPIO portI direction control register */
#define REG_GPIOI_DATAOUT	(GPIO_BA+0x68)  /* GPIO portI data output register */
#define REG_GPIOI_DATAIN	(GPIO_BA+0x6C)  /* GPIO portI data input register */

#endif
