/*
 * linux/arch/arm/mach-w90x900/irq.c
 *
 * based on linux/arch/arm/plat-s3c24xx/irq.c by Ben Dooks
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/regs-irq.h>

struct group_irq {
	unsigned long		gpen;
	unsigned int		enabled;
	void			(*enable)(struct group_irq *, int enable);
};

static DEFINE_SPINLOCK(groupirq_lock);

#define DEFINE_GROUP(_name, _ctrlbit, _num)				\
struct group_irq group_##_name = {					\
		.enable		= nuc900_group_enable,			\
		.gpen		= ((1 << _num) - 1) << _ctrlbit,	\
	}


void nuc900_enable_group_irq(int src)
{
	unsigned long regval;
	unsigned long flags;

	spin_lock_irqsave(&groupirq_lock, flags);
	regval = __raw_readl(REG_AIC_GEN);

	regval |= src;

	__raw_writel(regval, REG_AIC_GEN);
	spin_unlock_irqrestore(&groupirq_lock, flags);
}
EXPORT_SYMBOL(nuc900_enable_group_irq);

void nuc900_disable_group_irq(int src)
{
	unsigned long regval;
	unsigned long flags;

	spin_lock_irqsave(&groupirq_lock, flags);
	regval = __raw_readl(REG_AIC_GEN);

	regval &= ~src;

	__raw_writel(regval, REG_AIC_GEN);
	spin_unlock_irqrestore(&groupirq_lock, flags);
}
EXPORT_SYMBOL(nuc900_disable_group_irq);

static void nuc900_irq_mask(unsigned int irq)
{
	__raw_writel(1 << irq, REG_AIC_MDCR);
}

/*
 * By the w90p910 spec,any irq,only write 1
 * to REG_AIC_EOSCR for ACK
 */

static void nuc900_irq_ack(unsigned int irq)
{
	__raw_writel(0x01, REG_AIC_EOSCR);
}

static void nuc900_irq_unmask(unsigned int irq)
{

	__raw_writel(1 << irq, REG_AIC_MECR);
}

/*
 * nuc900 startup function
 */
static unsigned int nuc900_irq_startup(unsigned int irq)
{

	nuc900_irq_unmask(irq);
	return 0;
}

/*
 * nuc900 shutdown function
 */
static void nuc900_irq_shutdown(unsigned int irq)
{
	nuc900_irq_mask(irq);
}

static struct irq_chip nuc900_irq_chip = {
	.ack		= nuc900_irq_ack,
	.mask		= nuc900_irq_mask,
	.unmask		= nuc900_irq_unmask,
	.startup	= nuc900_irq_startup,
	.shutdown	= nuc900_irq_shutdown,
};

void __init nuc900_init_irq(void)
{
	int irqno;

	__raw_writel(0xFFFFFFFE, REG_AIC_MDCR);

	for (irqno = IRQ_WDT; irqno <= IRQ_ADC; irqno++) {
		set_irq_chip(irqno, &nuc900_irq_chip);
		set_irq_handler(irqno, handle_level_irq);
		set_irq_flags(irqno, IRQF_VALID);
	}
}
