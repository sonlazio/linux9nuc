/*
 * linux/arch/arm/mach-nuc900/cpu.c
 *
 * Copyright (c) 2010 Nuvoton corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
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
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#ifndef CONFIG_CPU_NUC960
// semaphore for preventing concurrent FMI devices activity
DECLARE_MUTEX(fmi_sem);
EXPORT_SYMBOL(fmi_sem);
#endif //#ifndef CONFIG_CPU_NUC960
