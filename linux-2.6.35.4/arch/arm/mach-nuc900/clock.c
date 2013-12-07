/*
 * linux/arch/arm/mach-w90x900/clock.c
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/regs-clock.h>
#include "clock.h"

#define SUBCLK 0x24

static DEFINE_SPINLOCK(clocks_lock);

int clk_enable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->enabled++ == 0)
		(clk->enable)(clk, 1);
	spin_unlock_irqrestore(&clocks_lock, flags);

	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	WARN_ON(clk->enabled == 0);

	spin_lock_irqsave(&clocks_lock, flags);
	if (--clk->enabled == 0)
		(clk->enable)(clk, 0);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

#define EXT_CLK 15000000
unsigned long clk_get_rate(struct clk *clk)
{
	return EXT_CLK;
}
EXPORT_SYMBOL(clk_get_rate);

unsigned long get_cpu_clk(void)
{
	unsigned long clk, reg;
	
	reg = __raw_readl(REG_PLLCON0);  // won't be REG_PLLCON1 based on Nuvoton's loaders
	if(reg & 0x10000)
		clk = EXT_CLK;
        else if (reg == PLL_200MHZ)
                clk = 200 * 1000;
        else if (reg == PLL_166MHZ)
                clk = 166 * 1000;
        else if (reg == PLL_120MHZ)
                clk = 120 * 1000;
        else if (reg == PLL_100MHZ)
                clk = 100 * 1000;
        else
                clk = 66 * 1000;
	return clk;
}
EXPORT_SYMBOL(get_cpu_clk);

unsigned long get_ahb_clk(void)
{
	unsigned long clk = get_cpu_clk();
	unsigned long reg = __raw_readl(REG_CLKDIV) & 0x3000000;
	
	if(reg == 0x3000000)
		clk >>= 3;
	else if(reg == 0x2000000)
		clk >>= 2;
	else if(reg == 0x1000000)
		clk >>= 1;

	return clk;
}
EXPORT_SYMBOL(get_ahb_clk);

unsigned long get_apb_clk(void)
{
	unsigned long clk = get_ahb_clk();
	unsigned long reg = __raw_readl(REG_CLKDIV) & 0xC000000;

	if(reg == 0xC000000)
		clk >>= 3;
	else if(reg == 0x8000000)
		clk >>= 2;
	else
		clk >>= 1;

	return clk;
}
EXPORT_SYMBOL(get_apb_clk);

void nuc900_clk_enable(struct clk *clk, int enable)
{
	unsigned int clocks = clk->cken;
	unsigned long clken;

	clken = __raw_readl(W90X900_VA_CLKPWR);

	if (enable)
		clken |= clocks;
	else
		clken &= ~clocks;

	__raw_writel(clken, W90X900_VA_CLKPWR);
}

void nuc900_subclk_enable(struct clk *clk, int enable)
{
	unsigned int clocks = clk->cken;
	unsigned long clken;

	clken = __raw_readl(W90X900_VA_CLKPWR + SUBCLK);

	if (enable)
		clken |= clocks;
	else
		clken &= ~clocks;

	__raw_writel(clken, W90X900_VA_CLKPWR + SUBCLK);
}
