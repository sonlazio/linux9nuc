/* linux/include/asm-arm/arch-nuc900/nuc900_sdio.h
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


#ifndef _NUC900_SDIO_H_
#define _NUC900_SDIO_H_


/* Flash buffer 0 registers */
#define FB0_BASE_ADDR		(0x000 + DMAC_BA)
#define FB0_SIZE		0x200

/* Flash buffer 1 registers */
#define FB1_BASE_ADDR		(0x200 + DMAC_BA)
#define FB1_SIZE		0x200

#define DMA_BLOCK_SIZE		0x200
#define SD_BLOCK_SIZE		0x200

/* FMI Global Control and Status Register(FMICSR) */
#define FMICSR_SW_RST		(1)
#define FMICSR_SD_EN		(1<<1)
#define FMICSR_MS_EN		(1<<2)
#define FMICSR_SM_EN		(1<<3)
#define FMICSR_CF_EN		(1<<4)

/* FMI Global Interrupt Control Register(FMIIER) */
#define FMIIER_DTA_IE		(1)

/* FMI Global Interrupt Status Register (FMIISR) */
#define FMIISR_DTA_IF		(1)

/* SD Control and Status Register (SDCSR) */
#define SDCSR_CO_EN		(1)
#define SDCSR_RI_EN		(1<<1)
#define SDCSR_DI_EN		(1<<2)
#define SDCSR_DO_EN		(1<<3)
#define SDCSR_R2_EN		(1<<4)
#define SDCSR_CLK74_OE		(1<<5)
#define SDCSR_CLK8_OE		(1<<6)
#define SDCSR_CLK_KEEP0		(1<<7)
#define SDCSR_SW_RST		(1<<14)
#define SDCSR_DBW		(1<<15)
#define SDCSR_CLK_KEEP1		(1<<31)

/* SD Interrupt Control Register (SDIER) */
#define SDIER_BLKD_IE		(1)
#define SDIER_CRC_IE		(1<<1)
#define SDIER_CD0_IE		(1<<8)
#define SDIER_CD1_IE		(1<<9)
#define SDIER_SDIO0_IE		(1<<10)
#define SDIER_SDIO1_IE		(1<<11)
#define SDIER_RITO_IE		(1<<12)
#define SDIER_DITO_IE		(1<<13)
#define SDIER_WKUP_EN		(1<<14)
#define SDIER_CD0SRC		(1<<30)
#define SDIER_CD1SRC		(1<<31)

/* SD Interrupt Status Register (SDISR) */
#define SDISR_BLKD_IF		(1)
#define SDISR_CRC_IF		(1<<1)
#define SDISR_CRC_7		(1<<2)
#define SDISR_CRC_16		(1<<3)
#define SDISR_SDDAT0		(1<<7)
#define SDISR_CD0_IF		(1<<8)
#define SDISR_CD1_IF		(1<<9)
#define SDISR_SDIO0_IF		(1<<10)
#define SDISR_SDIO1_IF		(1<<11)
#define SDISR_RITO_IF		(1<<12)
#define SDISR_DITO_IF		(1<<13)
#define SDISR_CDPS0		(1<<16)
#define SDISR_CDPS1		(1<<17)
#define SDISR_SD0DAT1		(1<<18)
#define SDISR_SD1DAT1		(1<<19)

/* DMAC Control and Status Register (DMACCSR) */
#define DMACCSR_DMACEN		(1)
#define DMACCSR_SW_RST		(1<<1)
#define DMACCSR_SG_EN1		(1<<2)
#define DMACCSR_SG_EN2		(1<<3)
#define DMACCSR_ATA_BUSY	(1<<8)
#define DMACCSR_FMI_BUSY	(1<<9)

/* DMAC Interrupt Enable Register (DMACIER) */
#define DMACIER_TABORT_IE	(1)
#define DMACIER_WEOT_IE		(1<<1)

/* DMAC Interrupt Status Register (DMACISR) */
#define DMACISR_TABORT_IF	(1)
#define DMACISR_WEOT_IF		(1<<1)

/* DMAC BIST Control and Status Register (DMACBIST) */
#define DMACBIST_BIST_EN	(1)
#define DMACBIST_FINISH		(1<<1)
#define DMACBIST_FAILED		(1<<2)

#endif


