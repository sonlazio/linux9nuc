/* linux/include/asm/arch-nuc900/regs-iic.h
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
 *   2006/08/26     vincen.zswan add this file for nuvoton nuc900 evb.
 */



#ifndef ___ASM_ARCH_REGS_PCI_H
#define ___ASM_ARCH_REGS_PCI_H "$Id: lcd.h,v 1.3 2003/06/26 13:25:06 ben Exp $"

#define PCI_BA    W90X900_VA_PCI /* PCI Control */
/* PCI Control Registers */
#define REG_PCICTR			(PCI_BA + 0x000)    /* PCI Control Register */
#define REG_PCISTR			(PCI_BA + 0x004)    /* PCI Status Register */
#define REG_PCILATIMER		(PCI_BA + 0x008)    /* PCI Latency Timer Register */
#define REG_PCIINTEN		(PCI_BA + 0x010)    /* PCI Interrupt Enable Register */
#define REG_PCIINT			(PCI_BA + 0x014)    /* PCI Interrupt Flag Register */
#define REG_CFGADDR			(PCI_BA + 0x020)    /* Configuration Address Register */
#define REG_CFGDATA			(PCI_BA + 0x024)    /* Configuration Data Register */
#define REG_PCIARB			(PCI_BA + 0x04C)    /* PCI Arbitration Register */
#define REG_PCIBIST			(PCI_BA + 0x050)    /* PCI FIFO BIST Register */

#if 1  // 2011.10.14
#define NUC900_PCI_IO_BASE		0xE0000000
#define NUC900_PCI_IO_END		0xE000FFFF
#define NUC900_PCI_IO_SIZE		0x10000		

#define NUC900_PCI_MEM_BASE		0xC0000000
#define NUC900_PCI_MEM_END		0xDFFFEFFFF
#define NUC900_PCI_MEM_SIZE		0x20000000		

#else
#define NUC900_PCI_MEM_BASE		0xE0000000
#define NUC900_PCI_IO_BASE		NUC900_PCI_MEM_BASE + 0x80000	

#define NUC900_PCI_MEM_SIZE		0x40000	
#define NUC900_PCI_IO_SIZE		NUC900_PCI_MEM_SIZE	
#endif


#endif /* ___ASM_ARCH_REGS_PCI_H */



