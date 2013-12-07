/* linux/arch/arm/mach-w90x900/dmac.c
 *
 * Copyright (c) 2010 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/blkdev.h>
#include <linux/string.h>

#include <scsi/scsi.h>
#include <scsi/scsi_device.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/cacheflush.h>

#include <mach/map.h>
#include <mach/regs-fmi.h>

#ifndef CONFIG_CPU_NUC960

DECLARE_WAIT_QUEUE_HEAD(dmac_wq);
DECLARE_MUTEX(dmac_sem);


EXPORT_SYMBOL(dmac_sem);

#if 0
static irqreturn_t dmac_interrupt( int irq, void *dev_id)
{
        if (__raw_readl(REG_DMACISR) & 0x01) {
                __raw_writel(__raw_readl(REG_DMACCSR)|0x03,REG_DMACCSR);
                __raw_writel(0x01,REG_DMACISR); //clear
        }
        return IRQ_HANDLED;

}
#endif

static int __init dmac_init(void)
{
#if 0
        if (request_irq(IRQ_DMAC, dmac_interrupt, SA_INTERRUPT, "NUC900_DMAC", 0)) {
                printk("DMAC : Request IRQ error.\n");
        }
#endif
        return 0;

}


static void __exit dmac_exit(void)
{

        free_irq(IRQ_DMAC, NULL);

}

module_init(dmac_init);
module_exit(dmac_exit);

#endif //#ifndef CONFIG_CPU_NUC960
