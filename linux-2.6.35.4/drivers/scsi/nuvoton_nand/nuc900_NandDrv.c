/* linux/driver/scsi/nuvoton_gnand/NandDrv.c
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
 *   2008/08/19     jcao add this file for nuvoton all nand driver.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/blkdev.h>
#include <linux/string.h>
#include <linux/slab.h>

#include <mach/map.h>
#include <mach/regs-fmi.h>
#include <mach/gnand/GNAND.h>
#include <mach/gnand/GNAND_global.h>
#include <mach/nuc900_nand.h>

extern bool volatile _fmi_bIsSMDataReady;

extern u32 _fmi_ucNANDBuffer;
extern u32 _fmi_pNANDBuffer;
extern u8 *_fmi_gptr1;


#if 0
#define DEBUG
#define SD_DEBUG
#define DUMP
#define SD_DEBUG_ENABLE_ENTER_LEAVE
#define SD_DEBUG_ENABLE_MSG
#define SD_DEBUG_ENABLE_MSG2
#define SD_DEBUG_PRINT_LINE
#endif

#ifdef SD_DEBUG
#define PDEBUG(fmt, arg...)		printk(fmt, ##arg)
#else
#define PDEBUG(fmt, arg...)
#endif

#ifdef SD_DEBUG_PRINT_LINE
#define PRN_LINE()				PDEBUG("[%-20s] : %d\n", __FUNCTION__, __LINE__)
#else
#define PRN_LINE()
#endif

#ifdef SD_DEBUG_ENABLE_ENTER_LEAVE
#define ENTER()					PDEBUG("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()					PDEBUG("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

#ifdef SD_DEBUG_ENABLE_MSG
#define MSG(msg)				PDEBUG("[%-20s] : %s\n", __FUNCTION__, msg)
#else
#define MSG(msg)
#endif

#ifdef SD_DEBUG_ENABLE_MSG2
#define MSG2(fmt, arg...)			PDEBUG("[%-20s] : "fmt, __FUNCTION__, ##arg)
#define PRNBUF(buf, count)		{int i;MSG2("CID Data: ");for(i=0;i<count;i++)\
									PDEBUG("%02x ", buf[i]);PDEBUG("\n");}
#else
#define MSG2(fmt, arg...)
#define PRNBUF(buf, count)
#endif

extern struct semaphore fmi_sem;
extern struct semaphore dmac_sem;
#if 0

NDRV_T _nandDiskDriver0 = {
        nandInit0,
        nandpread0,
        nandpwrite0,
        nand_is_page_dirty0,
        nand_is_valid_block0,
        nand_ioctl_0,
        nand_block_erase0,
        nand_chip_erase0,
        0
};
#endif
/* functions */
int fmiSMCheckRB(void)
{
        while (1) {
                if (nuc900_nand_read(REG_SMISR) & 0x400) {
                        nuc900_nand_write(REG_SMISR, 0x400);
                        return 1;
                }
        }
        return 0;
}

// SM functions
int fmiSM_Reset(void)
{

        u32 volatile i;

        nuc900_nand_write(REG_SMCMD, 0xff);
        for (i=100; i>0; i--);

        if (!fmiSMCheckRB())
                return FMI_SM_RB_ERR;
        return 0;
}


void fmiSM_Initial(FMI_SM_INFO_T *pSM)
{
        if (pSM->bIs2KPageSize)
                nuc900_nand_write(REG_SMCSR, (nuc900_nand_read(REG_SMCSR)&0xfffffff0)|0x00000008);	// psize:2048; wp# set 1
        else
                nuc900_nand_write(REG_SMCSR, (nuc900_nand_read(REG_SMCSR)&0xfffffff0)|0x00000000);	// psize:512; wp# set 1

        nuc900_nand_write(REG_SMIER, 0x01);
}


bool gbIsSupport2PlaneNand = 0;
int fmiSM_ReadID(FMI_SM_INFO_T *pSM, NDISK_T *NDISK_info)
{

        u32 tempID[5];
        fmiSM_Reset();
        nuc900_nand_write(REG_SMCMD, 0x90);		// read ID command
        nuc900_nand_write(REG_SMADDR, 0x80000000);	// address 0x00

        tempID[0] = nuc900_nand_read(REG_SMDATA);
        tempID[1] = nuc900_nand_read(REG_SMDATA);
        tempID[2] = nuc900_nand_read(REG_SMDATA);
        tempID[3] = nuc900_nand_read(REG_SMDATA);
        tempID[4] = nuc900_nand_read(REG_SMDATA);

        NDISK_info->vendor_ID = tempID[0];
        NDISK_info->device_ID = tempID[1];

        switch (tempID[1]) {
                /* page size 512B */
        case 0x79:	// 128M
                pSM->uSectorPerFlash = 255744;
                pSM->uBlockPerFlash = 8191;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = 1;
                pSM->bIs2KPageSize = 0;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
		NDISK_info->nZone = 1;				/* number of zones */
		NDISK_info->nBlockPerZone = 8192;	/* blocks per zone */
		NDISK_info->nPagePerBlock = 32;		/* pages per block */
		NDISK_info->nLBPerZone = 8000;		/* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x76:	// 64M
                pSM->uSectorPerFlash = 127872;
                pSM->uBlockPerFlash = 4095;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = 1;
                pSM->bIs2KPageSize = 0;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
		NDISK_info->nZone = 1;				/* number of zones */
		NDISK_info->nBlockPerZone = 4096;	/* blocks per zone */
		NDISK_info->nPagePerBlock = 32;		/* pages per block */
		NDISK_info->nLBPerZone = 4000;		/* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x75:	// 32M
                pSM->uSectorPerFlash = 63936;
                pSM->uBlockPerFlash = 2047;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = 0;
                pSM->bIs2KPageSize = 0;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
		NDISK_info->nZone = 1;				/* number of zones */
		NDISK_info->nBlockPerZone = 2048;	/* blocks per zone */
		NDISK_info->nPagePerBlock = 32;		/* pages per block */
		NDISK_info->nLBPerZone = 2000;		/* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x73:	// 16M
                pSM->uSectorPerFlash = 31968;	// max. sector no. = 999 * 32
                pSM->uBlockPerFlash = 1023;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = 0;
                pSM->bIs2KPageSize = 0;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->nZone = 1;				/* number of zones */
                NDISK_info->nBlockPerZone = 1024;	/* blocks per zone */
                NDISK_info->nPagePerBlock = 32;		/* pages per block */
                NDISK_info->nLBPerZone = 1000;		/* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

                /* page size 2KB */
        case 0xf1:	// 128M
                pSM->uBlockPerFlash = 1023;
                pSM->uPagePerBlock = 64;
                pSM->uSectorPerBlock = 256;
                pSM->uSectorPerFlash = 255744;
                pSM->bIsMulticycle = 0;
                pSM->bIs2KPageSize = 1;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->nZone = 1;				/* number of zones */
                NDISK_info->nBlockPerZone = 1024;	/* blocks per zone */
                NDISK_info->nPagePerBlock = 64;		/* pages per block */
                NDISK_info->nLBPerZone = 1000;		/* logical blocks per zone */
                NDISK_info->nPageSize = 2048;
                break;

        case 0xda:	// 256M
                if ((tempID[3] & 0x33) == 0x11) {
                        pSM->uBlockPerFlash = 2047;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
			NDISK_info->nZone = 1;				/* number of zones */
			NDISK_info->nPagePerBlock = 64;		/* pages per block */
			NDISK_info->nBlockPerZone = 2048;	/* blocks per zone */
			NDISK_info->nLBPerZone = 2000;		/* logical blocks per zone */
                } else if ((tempID[3] & 0x33) == 0x21) {
                        pSM->uBlockPerFlash = 1023;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->bIsMLCNand = 1;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
			NDISK_info->nZone = 1;				/* number of zones */
			NDISK_info->nPagePerBlock = 128;	/* pages per block */
			NDISK_info->nBlockPerZone = 1024;	/* blocks per zone */
			NDISK_info->nLBPerZone = 1000;		/* logical blocks per zone */
                }
                pSM->uSectorPerFlash = 511488;
                pSM->bIsMulticycle = 1;
                pSM->bIs2KPageSize = 1;

                NDISK_info->nPageSize = 2048;
                break;

        case 0xdc:	// 512M
                if ((tempID[3] & 0x33) == 0x11) {
                        pSM->uBlockPerFlash = 4095;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
			NDISK_info->nZone = 1;				/* number of zones */
			NDISK_info->nPagePerBlock = 64;		/* pages per block */
			NDISK_info->nBlockPerZone = 4096;	/* blocks per zone */
			NDISK_info->nLBPerZone = 4000;		/* logical blocks per zone */
                } else if ((tempID[3] & 0x33) == 0x21) {
                        pSM->uBlockPerFlash = 2047;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->bIsMLCNand = 1;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
			NDISK_info->nZone = 1;				/* number of zones */
			NDISK_info->nPagePerBlock = 128;	/* pages per block */
			NDISK_info->nBlockPerZone = 2048;	/* blocks per zone */
			NDISK_info->nLBPerZone = 2000;		/* logical blocks per zone */
                }
                pSM->uSectorPerFlash = 1022976;
                pSM->bIsMulticycle = 1;
                pSM->bIs2KPageSize = 1;

                NDISK_info->nPageSize = 2048;
                break;

        case 0xd3:	// 1024M
                if ((tempID[3] & 0x33) == 0x11) {
                        pSM->uBlockPerFlash = 8191;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
			NDISK_info->nZone = 1;				/* number of zones */
			NDISK_info->nPagePerBlock = 64;		/* pages per block */
			NDISK_info->nBlockPerZone = 8192;	/* blocks per zone */
			NDISK_info->nLBPerZone = 8000;		/* logical blocks per zone */
                } else if ((tempID[3] & 0x33) == 0x21) {
                        pSM->uBlockPerFlash = 4095;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->bIsMLCNand = 1;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
			NDISK_info->nZone = 1;				/* number of zones */
			NDISK_info->nPagePerBlock = 128;	/* pages per block */
			NDISK_info->nBlockPerZone = 4096;	/* blocks per zone */
			NDISK_info->nLBPerZone = 4000;		/* logical blocks per zone */
                }
                pSM->uSectorPerFlash = 2045952;
                pSM->bIsMulticycle = 1;
                pSM->bIs2KPageSize = 1;

                NDISK_info->nPageSize = 2048;
                break;

        case 0xd5:	// 2048M
                if ((tempID[3] & 0x33) == 0x11) {
                        pSM->uBlockPerFlash = 16383;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
			NDISK_info->nZone = 1;				/* number of zones */
			NDISK_info->nPagePerBlock = 64;		/* pages per block */
			NDISK_info->nBlockPerZone = 16384;	/* blocks per zone */
			NDISK_info->nLBPerZone = 16000;		/* logical blocks per zone */
                } else if ((tempID[3] & 0x33) == 0x21) {
                        pSM->uBlockPerFlash = 8191;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->bIsMLCNand = 1;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
			NDISK_info->nZone = 1;				/* number of zones */
			NDISK_info->nPagePerBlock = 128;	/* pages per block */
			NDISK_info->nBlockPerZone = 8192;	/* blocks per zone */
			NDISK_info->nLBPerZone = 8000;		/* logical blocks per zone */
                }
                pSM->uSectorPerFlash = 4091904;
                pSM->bIsMulticycle = 1;
                pSM->bIs2KPageSize = 1;

                NDISK_info->nPageSize = 2048;

#if 0
                if ((tempID[0] == 0xec) /*|| (tempID[0] == 0x98)*/) {
                        //printk("2 plane nand found\n");
                        gbIsSupport2PlaneNand = 1;
                        NDISK_info->nZone = 4;				/* number of zones */
                        NDISK_info->nPageSize = 4096;
                }
#endif
                break;

        default:
#ifdef DEBUG
                printk("SM ID not support!![%x][%x]\n", tempID[0], tempID[1]);
#endif
                return FMI_SM_ID_ERR;
        }

#ifdef DEBUG
        printk("SM ID [%x][%x][%x][%x]\n", tempID[0], tempID[1], tempID[2], tempID[3]);
#endif
        return 0;
}


int fmiSM2BufferM(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        nuc900_nand_write(REG_SMCMD, 0x00);		// read command
        nuc900_nand_write(REG_SMADDR, ucColAddr);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, uSector & 0xff);	// PA0 - PA7
        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((uSector >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (uSector >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((uSector >> 16) & 0xff)|0x80000000);		// PA16 - PA17
        }

        if (!fmiSMCheckRB()) {
                //printk("failed 2\n");
                return FMI_SM_RB_ERR;
        }
        return 0;
}


int fmiSM2BufferM_RA(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        nuc900_nand_write(REG_SMCMD, 0x50);		// read command
        nuc900_nand_write(REG_SMADDR, ucColAddr);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, uSector & 0xff);	// PA0 - PA7
        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((uSector >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (uSector >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((uSector >> 16) & 0xff)|0x80000000);		// PA16 - PA17
        }

        if (!fmiSMCheckRB())
                return FMI_SM_RB_ERR;

        return 0;
}


int fmiSMCorrectData_512(u32 uDAddr)
{

        u32 volatile reg, byte;
        unsigned int volatile data, errCount;

        reg = nuc900_nand_read(REG_ECC4ST);

        if (reg & 0x02) {
//#ifdef DEBUG
#if 0
                printk("uncorrectable!!\n");
#endif
                return FMI_SM_ECC_ERROR;
        } else if (reg & 0x01) {
                errCount = (nuc900_nand_read(REG_ECC4ST) >> 2) & 0x7;
                switch (errCount) {
                case 1:
                        byte = nuc900_nand_read(REG_ECC4F1A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F1D) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        break;

                case 2:
                        byte = nuc900_nand_read(REG_ECC4F1A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F1D) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        byte = (nuc900_nand_read(REG_ECC4F1A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        break;

                case 3:
                        byte = nuc900_nand_read(REG_ECC4F1A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F1D) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        byte = (nuc900_nand_read(REG_ECC4F1A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        byte = nuc900_nand_read(REG_ECC4F1A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        break;

                case 4:
                        byte = nuc900_nand_read(REG_ECC4F1A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F1D) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        byte = (nuc900_nand_read(REG_ECC4F1A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        byte = nuc900_nand_read(REG_ECC4F1A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        byte = (nuc900_nand_read(REG_ECC4F1A2) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 24) & 0xff;
                        __raw_writeb(__raw_readb(uDAddr+byte)^(data & 0xff), uDAddr+byte);
#ifdef DEBUG
                        printk("correctable 4 byte!! byte[%d], data[%d]\n", byte, data);
#endif
                        break;
                }
        }

        return 0;
}


int fmiSM_Read_512(FMI_SM_INFO_T *pSM, u32 uSector, u32 uDAddr)
{
//	nuc900_nand_write(REG_DMACSAR2, uDAddr);

        if (down_interruptible(&dmac_sem)) {
                //printk("io err\n");
                return(GNERR_IO_ERR);
        }
        while (nuc900_nand_read(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe

        nuc900_nand_write(REG_DMACSAR2, _fmi_ucNANDBuffer);

        fmiSM2BufferM(pSM, uSector, 0);

        _fmi_bIsSMDataReady = 0;
        nuc900_nand_write(REG_SMCSR, nuc900_nand_read(REG_SMCSR) | 0x02);
        while (!_fmi_bIsSMDataReady);
        up(&dmac_sem);
        // check ECC
        if (nuc900_nand_read(REG_SMISR) & 0x02) {

                //printk("ECC error Sector[%x]\n", uSector);

                nuc900_nand_write(REG_SMISR, 0x02);
                memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512);
                return fmiSMCorrectData_512(uDAddr);
        }

        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512);
        return 0;
}


void fmiBuffer2SMM(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        // set the spare area configuration
        /* write byte 514, 515 as used page */
        nuc900_nand_write(REG_SMRA_0, 0x0000FFFF);

        // send command
        nuc900_nand_write(REG_SMCMD, 0x80);		// serial data input command
        nuc900_nand_write(REG_SMADDR, ucColAddr);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, uSector & 0xff);	// PA0 - PA7
        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((uSector >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (uSector >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((uSector >> 16) & 0xff)|0x80000000);		// PA16 - PA17
        }
}


int fmiSM_Write_512(FMI_SM_INFO_T *pSM, u32 uSector, u32 uSAddr)
{
//	printk("fmiSM_Write_512!!\n");
        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 512);

        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
        while (nuc900_nand_read(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe

        nuc900_nand_write(REG_DMACSAR2, _fmi_ucNANDBuffer);
        fmiBuffer2SMM(pSM, uSector, 0);

        _fmi_bIsSMDataReady = 0;
        nuc900_nand_write(REG_SMCSR, nuc900_nand_read(REG_SMCSR) | 0x04);
        while (!_fmi_bIsSMDataReady);

        nuc900_nand_write(REG_SMCMD, 0x10);		// auto program command

        if (!fmiSMCheckRB())
                return FMI_SM_RB_ERR;

        nuc900_nand_write(REG_SMCMD, 0x70);		// status read command
        if (nuc900_nand_read(REG_SMDATA) & 0x01) {	// 1:fail; 0:pass
#ifdef DEBUG
                printk("fmiSM_Write_512: data error!!\n");
#endif
                return FMI_SM_STATE_ERROR;
        }

        up(&dmac_sem);
        return 0;
}


int fmiSMCorrectData_2K(u32 uDAddr)
{

        u32 volatile byte, correct_addr;
        unsigned int volatile reg, data, errCount;

        reg = nuc900_nand_read(REG_ECC4ST);

        if ((reg & 0x02) || (reg & 0x200) || (reg & 0x20000) || (reg & 0x2000000)) {
#if 0
                printk("uncorrectable!!\n");
#endif
                return FMI_SM_ECC_ERROR;
        } else if (reg & 0x01) {
                errCount = (nuc900_nand_read(REG_ECC4ST) >> 2) & 0x7;
                correct_addr = uDAddr;
                switch (errCount) {
                case 1:
                        byte = nuc900_nand_read(REG_ECC4F1A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F1D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 2:
                        byte = nuc900_nand_read(REG_ECC4F1A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F1D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F1A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 3:
                        byte = nuc900_nand_read(REG_ECC4F1A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F1D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F1A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = nuc900_nand_read(REG_ECC4F1A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 4:
                        byte = nuc900_nand_read(REG_ECC4F1A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F1D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F1A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = nuc900_nand_read(REG_ECC4F1A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F1A2) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F1D) >> 24) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 4 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;
                }
        }
        if (reg & 0x100) {
                errCount = (nuc900_nand_read(REG_ECC4ST) >> 10) & 0x7;
                correct_addr = uDAddr + 0x200;
                switch (errCount) {
                case 1:
                        byte = nuc900_nand_read(REG_ECC4F2A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F2D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 2:
                        byte = nuc900_nand_read(REG_ECC4F2A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F2D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F2A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F2D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 3:
                        byte = nuc900_nand_read(REG_ECC4F2A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F2D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F2A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F2D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = nuc900_nand_read(REG_ECC4F2A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F2D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 4:
                        byte = nuc900_nand_read(REG_ECC4F2A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F2D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F2A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F2D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = nuc900_nand_read(REG_ECC4F2A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F2D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F2A2) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F2D) >> 24) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 4 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;
                }
        }
        if (reg & 0x10000) {
                errCount = (nuc900_nand_read(REG_ECC4ST) >> 18) & 0x7;
                correct_addr = uDAddr + 0x400;
                switch (errCount) {
                case 1:
                        byte = nuc900_nand_read(REG_ECC4F3A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F3D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 2:
                        byte = nuc900_nand_read(REG_ECC4F3A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F3D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F3A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F3D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 3:
                        byte = nuc900_nand_read(REG_ECC4F3A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F3D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F3A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F3D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = nuc900_nand_read(REG_ECC4F3A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F3D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 4:
                        byte = nuc900_nand_read(REG_ECC4F3A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F3D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F3A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F3D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = nuc900_nand_read(REG_ECC4F3A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F3D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F3A2) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F3D) >> 24) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 4 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;
                }
        }
        if (reg & 0x1000000) {
                errCount = (nuc900_nand_read(REG_ECC4ST) >> 26) & 0x7;
                correct_addr = uDAddr + 0x600;
                switch (errCount) {
                case 1:
                        byte = nuc900_nand_read(REG_ECC4F4A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F4D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 2:

                        byte = nuc900_nand_read(REG_ECC4F4A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F4D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F4A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F4D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 3:

                        byte = nuc900_nand_read(REG_ECC4F4A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F4D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F4A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F4D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = nuc900_nand_read(REG_ECC4F4A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F4D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;

                case 4:

                        byte = nuc900_nand_read(REG_ECC4F4A1) & 0x1ff;
                        data = nuc900_nand_read(REG_ECC4F4D) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 1 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F4A1) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F4D) >> 8) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 2 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = nuc900_nand_read(REG_ECC4F4A2) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F4D) >> 16) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 3 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        byte = (nuc900_nand_read(REG_ECC4F4A2) >> 16) & 0x1ff;
                        data = (nuc900_nand_read(REG_ECC4F4D) >> 24) & 0xff;
                        __raw_writeb(__raw_readb(correct_addr+byte)^(data & 0xff), correct_addr+byte);
#ifdef DEBUG
                        printk("correctable 4 byte!! correct_addr[%x], byte[%x], data[%x]\n", correct_addr, byte, data);
                        printk("correct data [%x]\n", __raw_readb(correct_addr+byte));
#endif
                        break;
                }
        }
        return 0;
}


int fmiSM_Read_2K(FMI_SM_INFO_T *pSM, u32 uPage, u32 uDAddr)
{
//	printk("fmiSM_Read_2K uPage=%d   uDAddr=%x\n",uPage,uDAddr);

        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
//printk("nand dma in - r\n");

        while (nuc900_nand_read(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe

//	nuc900_nand_write(REG_DMACSAR2, uDAddr);	// set DMA transfer starting address
        nuc900_nand_write(REG_DMACSAR2, _fmi_ucNANDBuffer);

        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        nuc900_nand_write(REG_SMCMD, 0x00);		// read command
        nuc900_nand_write(REG_SMADDR, 0);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, 0);	// CA8 - CA11
        nuc900_nand_write(REG_SMADDR, uPage & 0xff);	// PA0 - PA7
        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((uPage >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (uPage >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((uPage >> 16) & 0xff)|0x80000000);		// PA16 - PA17
        }
        nuc900_nand_write(REG_SMCMD, 0x30);		// read command

        if (!fmiSMCheckRB()) {

                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }
        _fmi_bIsSMDataReady = 0;

        nuc900_nand_write(REG_SMCSR, nuc900_nand_read(REG_SMCSR) | 0x02);

        while (!_fmi_bIsSMDataReady);

//printk("nand dma out - r\n");
        up(&dmac_sem);

        // check ECC
        if (nuc900_nand_read(REG_SMISR) & 0x02) {
#ifdef DEBUG
                printk("ECC error! Page[%d]\n", uPage);
#endif
                nuc900_nand_write(REG_SMISR, 0x02);

                memcpy((char *)uDAddr,(char *) _fmi_pNANDBuffer, 512*4); //add by JCAO¡¡£²£°£°£¸£®£µ£®£²£³
                return fmiSMCorrectData_2K(uDAddr);
        }
        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512*4);
        return 0;
}


int fmiSM_Read_2K_RA(FMI_SM_INFO_T *pSM, u32 uPage, u32 ucColAddr)
{
        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        nuc900_nand_write(REG_SMCMD, 0x00);		// read command
        nuc900_nand_write(REG_SMADDR, ucColAddr);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, (ucColAddr >> 8) & 0x0f);	// CA8 - CA11
        nuc900_nand_write(REG_SMADDR, uPage & 0xff);	// PA0 - PA7
        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((uPage >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (uPage >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((uPage >> 16) & 0xff)|0x80000000);		// PA16 - PA17
        }
        nuc900_nand_write(REG_SMCMD, 0x30);		// read command

        if (!fmiSMCheckRB())
                return FMI_SM_RB_ERR;

        return 0;
}



int fmiSM_Read_2Plane(FMI_SM_INFO_T *pSM, u32 PBA, u32 uPage, u32 uDAddr)
{

        int status;
        u32 pageNo;

        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
        while (nuc900_nand_read(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe

        //nuc900_nand_write(REG_DMACSAR2, uDAddr);	// set DMA transfer starting address
        nuc900_nand_write(REG_DMACSAR2, _fmi_ucNANDBuffer);

        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        pageNo = (PBA << 1) * pSM->uPagePerBlock + uPage;
        nuc900_nand_write(REG_SMCMD, 0x00);		// read command
        nuc900_nand_write(REG_SMADDR, 0);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, 0);	// CA8 - CA11
        nuc900_nand_write(REG_SMADDR, pageNo & 0xff);	// PA0 - PA7

        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((pageNo >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (pageNo >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((pageNo >> 16) & 0x0f)|0x80000000);		// PA16 - PA17
        }
        nuc900_nand_write(REG_SMCMD, 0x30);		// read command

        if (!fmiSMCheckRB()) {
#if  0
                printk("2. fmiSM_Write_2Plane: R/B- timeout!!\n");
#endif
                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }

        _fmi_bIsSMDataReady = 0;

        nuc900_nand_write(REG_SMCSR, nuc900_nand_read(REG_SMCSR) | 0x02);

        while (!_fmi_bIsSMDataReady);

        // check ECC
        memcpy((char *)uDAddr,(char *) _fmi_pNANDBuffer, 512*4);
        if (nuc900_nand_read(REG_SMISR) & 0x02) {
#if  0
                printk("ECC error! Page[%d]\n", uPage);
#endif
                nuc900_nand_write(REG_SMISR, 0x02);
                status = fmiSMCorrectData_2K(uDAddr);
                if (status < 0) {
                        up(&dmac_sem);
                        return status;
                }
        }

        nuc900_nand_write(REG_DMACSAR2, _fmi_ucNANDBuffer);	// set DMA transfer starting address

        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        pageNo = ((PBA << 1) + 1) * pSM->uPagePerBlock + uPage;
        nuc900_nand_write(REG_SMCMD, 0x00);		// read command
        nuc900_nand_write(REG_SMADDR, 0);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, 0);	// CA8 - CA11
        nuc900_nand_write(REG_SMADDR, (pageNo & 0xff));	// PA0 - PA7

        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((pageNo >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (pageNo >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((pageNo >> 16) & 0x0f)|0x80000000);		// PA16 - PA17
        }
        nuc900_nand_write(REG_SMCMD, 0x30);		// read command

        if (!fmiSMCheckRB()) {
                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }
        _fmi_bIsSMDataReady = 0;

        nuc900_nand_write(REG_SMCSR, nuc900_nand_read(REG_SMCSR) | 0x02);

        while (!_fmi_bIsSMDataReady);
        up(&dmac_sem);
        // check ECC
        memcpy((char *)(uDAddr + 2048),(char *) _fmi_pNANDBuffer, 512*4);
        if (nuc900_nand_read(REG_SMISR) & 0x02) {
                nuc900_nand_write(REG_SMISR, 0x02);
                return fmiSMCorrectData_2K(uDAddr+2048);
        }
        return 0;
}


int fmiSM_Read_2Plane_RA(FMI_SM_INFO_T *pSM, u32 uPage, u32 ucColAddr)
{
        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        nuc900_nand_write(REG_SMCMD, 0x00);		// read command
        nuc900_nand_write(REG_SMADDR, ucColAddr);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, (ucColAddr >> 8) & 0x0f);	// CA8 - CA11
        nuc900_nand_write(REG_SMADDR, uPage & 0xff);	// PA0 - PA7
        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((uPage >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (uPage >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((uPage >> 16) & 0x0f)|0x80000000);		// PA16 - PA17
        }
        nuc900_nand_write(REG_SMCMD, 0x30);		// read command

        if (!fmiSMCheckRB())
                return FMI_SM_RB_ERR;

        return 0;
}


int fmiSM_Write_2K(FMI_SM_INFO_T *pSM, u32 uSector, u32 ucColAddr, u32 uSAddr)
{
        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
//printk("nand dma in - w\n");
        while (nuc900_nand_read(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 512*4);

//	nuc900_nand_write(REG_DMACSAR2, uSAddr);	// set DMA transfer starting address
        nuc900_nand_write(REG_DMACSAR2, _fmi_ucNANDBuffer);

        // set the spare area configuration
        /* write byte 2050, 2051 as used page */
        nuc900_nand_write(REG_SMRA_0, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_1, 0xFFFFFFFF);
        nuc900_nand_write(REG_SMRA_4, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_5, 0xFFFFFFFF);
        nuc900_nand_write(REG_SMRA_8, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_9, 0xFFFFFFFF);
        nuc900_nand_write(REG_SMRA_12, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_13, 0xFFFFFFFF);

        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        // send command
        nuc900_nand_write(REG_SMCMD, 0x80);		// serial data input command
        nuc900_nand_write(REG_SMADDR, ucColAddr);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, (ucColAddr >> 8) & 0x0f);	// CA8 - CA11
        nuc900_nand_write(REG_SMADDR, uSector & 0xff);	// PA0 - PA7
        if (!pSM->bIsMulticycle)
                nuc900_nand_write(REG_SMADDR, ((uSector >> 8) & 0xff)|0x80000000);		// PA8 - PA15
        else {
                nuc900_nand_write(REG_SMADDR, (uSector >> 8) & 0xff);		// PA8 - PA15
                nuc900_nand_write(REG_SMADDR, ((uSector >> 16) & 0xff)|0x80000000);		// PA16 - PA17
        }

        _fmi_bIsSMDataReady = 0;

        nuc900_nand_write(REG_SMCSR, nuc900_nand_read(REG_SMCSR) | 0x04);

        while (!_fmi_bIsSMDataReady);

        nuc900_nand_write(REG_SMCMD, 0x10);		// auto program command

        if (!fmiSMCheckRB())
                return FMI_SM_RB_ERR;

        nuc900_nand_write(REG_SMCMD, 0x70);		// status read command
        if (nuc900_nand_read(REG_SMDATA) & 0x01) {	// 1:fail; 0:pass
#ifdef DEBUG
                printk("fmiSM_Write_2K: data error!!\n");
#endif
                return FMI_SM_STATE_ERROR;
        }

//printk("nand dma out - w\n");
        up(&dmac_sem);
        return 0;
}


int fmiSM_Write_2Plane(FMI_SM_INFO_T *pSM, u32 uSector, u32 ucColAddr, u32 uSAddr)
{
        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
        while (nuc900_nand_read(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 512*4);
        /* first page */
        nuc900_nand_write(REG_DMACSAR2, _fmi_ucNANDBuffer);	// set DMA transfer starting address

        /* write byte 2050, 2051 as used page */
        nuc900_nand_write(REG_SMRA_0, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_4, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_8, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_12, 0x0000FFFF);

        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        // send command
        nuc900_nand_write(REG_SMCMD, 0x80);		// serial data input command
        nuc900_nand_write(REG_SMADDR, 0);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, 0);	// CA8 - CA11
        nuc900_nand_write(REG_SMADDR, 0);	// PA0 - PA7
        nuc900_nand_write(REG_SMADDR, 0);		// PA8 - PA15
        nuc900_nand_write(REG_SMADDR, ((uSector >> 16) & 0x08)|0x80000000);		// PA16 - PA17

        _fmi_bIsSMDataReady = 0;

        nuc900_nand_write(REG_SMCSR, nuc900_nand_read(REG_SMCSR) | 0x04);

        while (!_fmi_bIsSMDataReady);

        nuc900_nand_write(REG_SMCMD, 0x11);		// auto program command
        if (!fmiSMCheckRB())  {
                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }


        /* second page */
        memcpy((char *)_fmi_pNANDBuffer, (char *)(uSAddr + 2048), 512*4);
        nuc900_nand_write(REG_DMACSAR2, _fmi_ucNANDBuffer);	// set DMA transfer starting address

        /* write byte 2050, 2051 as used page */
        nuc900_nand_write(REG_SMRA_0, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_4, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_8, 0x0000FFFF);
        nuc900_nand_write(REG_SMRA_12, 0x0000FFFF);

        // send command
        nuc900_nand_write(REG_SMCMD, 0x81);		// serial data input command
        nuc900_nand_write(REG_SMADDR, 0);	// CA0 - CA7
        nuc900_nand_write(REG_SMADDR, 0);	// CA8 - CA11
        nuc900_nand_write(REG_SMADDR, (uSector & 0xff)|0x80);	// PA0 - PA7
        nuc900_nand_write(REG_SMADDR, (uSector >> 8) & 0xff);		// PA8 - PA15
        nuc900_nand_write(REG_SMADDR, ((uSector >> 16) & 0x0f)|0x80000000);		// PA16 - PA17

        _fmi_bIsSMDataReady = 0;

        nuc900_nand_write(REG_SMCSR, nuc900_nand_read(REG_SMCSR) | 0x04);

        while (!_fmi_bIsSMDataReady);

        nuc900_nand_write(REG_SMCMD, 0x10);		// auto program command

        if (!fmiSMCheckRB())
                return FMI_SM_RB_ERR;

        nuc900_nand_write(REG_SMCMD, 0x70);		// status read command
        if (nuc900_nand_read(REG_SMDATA) & 0x01) {	// 1:fail; 0:pass
#if 0
                printk("fmiSM_Write_2Plane: data error!!\n");
#endif
                return FMI_SM_STATE_ERROR;
        }
        up(&dmac_sem);

        return 0;
}


int fmiSMCheckBootHeader(FMI_SM_INFO_T *pSM)
{
        int volatile status, imageCount, i, infoPage, block;
        unsigned int *pImageList = (unsigned int *)_fmi_gptr1;
        int  fmiNandSysArea = 0;

        memset(_fmi_gptr1, 0xff, 4096);
        infoPage = pSM->uPagePerBlock-1;

        /* read physical block 0 - image information */
        if (pSM->bIs2KPageSize == 0) {
                status = fmiSM_Read_512(pSM, infoPage, (u32)_fmi_gptr1);
                if (status < 0)
                        return status;
        } else {
                status = fmiSM_Read_2K(pSM, infoPage, (u32)_fmi_gptr1);
                if (status < 0)
                        return status;
        }

        if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963)) {
                fmiNandSysArea = *(pImageList+1);
        }

        if (fmiNandSysArea != 0xFFFFFFFF && fmiNandSysArea != 0) {
                pSM->uLibStartBlock = (fmiNandSysArea / pSM->uSectorPerBlock) + 1;
        } else {
                infoPage = pSM->uPagePerBlock-2;

                /* read physical block 0 - image information */
                if (pSM->bIs2KPageSize == 0) {	// 512
                        status = fmiSM_Read_512(pSM, infoPage, (u32)_fmi_gptr1);
                        if (status < 0)
                                return status;
                } else {
                        status = fmiSM_Read_2K(pSM, infoPage, (u32)_fmi_gptr1);
                        if (status < 0)
                                return status;
                }

                if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963)) {
                        imageCount = *(pImageList+1);

                        /* pointer to image information */
                        pImageList = pImageList+4;

                        for (i=0; i<imageCount; i++) {
                                block = (*(pImageList + 1) & 0xFFFF0000) >> 16;
                                if (block > pSM->uLibStartBlock)
                                        pSM->uLibStartBlock = block;

                                /* pointer to next image */
                                pImageList = pImageList+8;
                        }
                        pSM->uLibStartBlock++;
                }
        }

        return 0;
}
int fmiCheckInvalidBlock(FMI_SM_INFO_T *pSM, u32 BlockNo)
{

        int volatile status=0;
        unsigned int volatile sector;

        //printk("fmiCheckInvalidBlock  pSM0->uLibStartBlock=%d\n",BlockNo);
        /* MLC check the 2048 byte of last page per block */
        if (pSM->bIsMLCNand == 1) {
                if (gbIsSupport2PlaneNand) {
                        //sector = (BlockNo+2) * pSM->uPagePerBlock - 1;
                        sector = ((BlockNo << 1) + 2) * pSM->uPagePerBlock - 1;
                        /* Read 2048 byte */
                        status = fmiSM_Read_2K_RA(pSM, sector, 2048);
                        if (status < 0)
                                return status;
                        if ((nuc900_nand_read(REG_SMDATA) & 0xff) != 0xFF)
                                return 1;	// invalid block
                        sector = ((BlockNo << 1) + 1) * pSM->uPagePerBlock - 1;
                        /* Read 2048 byte */
                        status = fmiSM_Read_2K_RA(pSM, sector, 2048);
                        if (status < 0)
                                return status;
                        if ((nuc900_nand_read(REG_SMDATA) & 0xff) != 0xFF)
                                return 1;	// invalid block
                } else {

                        sector = (BlockNo+1) * pSM->uPagePerBlock - 1;
                        /* Read 2048 byte */
                        status = fmiSM_Read_2K_RA(pSM, sector, 2048);
                        if (status < 0)
                                return status;
                        if ((nuc900_nand_read(REG_SMDATA) & 0xff) != 0xFF)
                                return 1;	// invalid block
                }
        }
        /* SLC check the 2048 byte of 1st or 2nd page per block */
        else {	// SLC
                sector = BlockNo * pSM->uPagePerBlock;
                if (pSM->bIs2KPageSize == 1) {
                        status = fmiSM_Read_2K_RA(pSM, sector, 2048);
                        if (status < 0)
                                return status;
                        if ((nuc900_nand_read(REG_SMDATA) & 0xff) != 0xFF) {
                                status = fmiSM_Read_2K_RA(pSM, sector+1, 2048);
                                if (status < 0)
                                        return status;
                                if ((nuc900_nand_read(REG_SMDATA) & 0xff) != 0xFF) {
                                        return 1;	// invalid block
                                }
                        }
                } else {	/* page size 512B */
                        status = fmiSM2BufferM_RA(pSM, sector, 0);
                        if (status < 0)
                                return status;
                        if ((nuc900_nand_read(REG_SMDATA) & 0xff) != 0xFF) {
                                fmiSM_Reset();
                                status = fmiSM2BufferM_RA(pSM, sector+1, 0);
                                if (status < 0)
                                        return status;
                                if ((nuc900_nand_read(REG_SMDATA) & 0xff) != 0xFF) {
                                        fmiSM_Reset();
                                        return 1;	// invalid block
                                }
                        }
                        fmiSM_Reset();
                }
        }
        return 0;
}


/* function pointer */
FMI_SM_INFO_T *pSM0, *pSM1;
static int sicSMInit(int NandPort, NDISK_T *NDISK_info)
{

        int status=0;
	int count=0;

        if (down_interruptible(&fmi_sem))
                return GNERR_IO_ERR;
        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (NandPort == 0) {
                /* init SM interface */
                nuc900_nand_write(REG_SMCSR, (nuc900_nand_read(REG_SMCSR)&0xf8ffffc0)|0x01000020);	// enable ecc4

                pSM0 = kmalloc(sizeof(FMI_SM_INFO_T),GFP_KERNEL);
                memset((char *)pSM0, 0, sizeof(FMI_SM_INFO_T));

                if ((status = fmiSM_ReadID(pSM0, NDISK_info)) < 0) {
                        up(&fmi_sem);
                        return status;
                }
                fmiSM_Initial(pSM0);

                // check NAND boot header
                fmiSMCheckBootHeader(pSM0);

                while (1) {
                        if (fmiCheckInvalidBlock(pSM0, pSM0->uLibStartBlock) != 1)	// valid block
                                break;
                        else
                                pSM0->uLibStartBlock++;
                }
                printk("fmiCheckInvalidBlock  pSM0->uLibStartBlock=%d\n",pSM0->uLibStartBlock);
                NDISK_info->nStartBlock = pSM0->uLibStartBlock;		/* available start block */
                pSM0->uBlockPerFlash -= pSM0->uLibStartBlock;

        } else {
                /* init SM interface */
                nuc900_nand_write(REG_SMCSR, (nuc900_nand_read(REG_SMCSR)&0xf8ffffc0)|0x03000020);	// enable ecc4

                pSM1 = kmalloc(sizeof(FMI_SM_INFO_T),GFP_KERNEL);
                memset((char *)pSM1, 0, sizeof(FMI_SM_INFO_T));

                if ((status = fmiSM_ReadID(pSM1, NDISK_info)) < 0) {
                        up(&fmi_sem);
                        return status;
                }
                fmiSM_Initial(pSM1);

                // check NAND boot header
                fmiSMCheckBootHeader(pSM1);
                while (1) {
                        if (fmiCheckInvalidBlock(pSM1, pSM1->uLibStartBlock) != 1)	// valid block
                                break;
                        else
                                pSM1->uLibStartBlock++;
                }
                NDISK_info->nStartBlock = pSM1->uLibStartBlock;		/* available start block */
                pSM1->uBlockPerFlash -= pSM1->uLibStartBlock;

        }
        up(&fmi_sem);

	count = NDISK_info->nBlockPerZone * 2 / 100 + 10;
	NDISK_info->nBlockPerZone = (NDISK_info->nBlockPerZone * NDISK_info->nZone - NDISK_info->nStartBlock) / NDISK_info->nZone;
	NDISK_info->nLBPerZone = NDISK_info->nBlockPerZone - count;


        return 0;
}

static int sicSMpread(int NandPort, int PBA, int page, u8 *buff)
{

        //printk("sicSMpread  PBA=%d  page=%d buff=%x\n",PBA,page,buff);
        FMI_SM_INFO_T *pSM;
        int pageNo;
        int status;

        if (NandPort == 0)
                pSM = pSM0;
        else
                pSM = pSM1;

        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (gbIsSupport2PlaneNand) {

                PBA += (pSM->uLibStartBlock/2);
                if ((pSM->uLibStartBlock%2) != 0)
                        PBA++;

                //pageNo = (PBA << 1) * pSM->uPagePerBlock + page;
                status = fmiSM_Read_2Plane(pSM, PBA, page, (u32)buff);

                return (status);
        } else {
                PBA += pSM->uLibStartBlock;
                pageNo = PBA * pSM->uPagePerBlock + page;
                if (pSM->bIs2KPageSize) {	/* 2KB */
                        status = fmiSM_Read_2K(pSM, pageNo, (u32)buff);

                        return (status);
                } else {	/* 512B */
                        status = fmiSM_Read_512(pSM, pageNo, (u32)buff);

                        return (status);
                }
        }
}

static int sicSMpwrite(int NandPort, int PBA, int page, u8 *buff)
{

        FMI_SM_INFO_T *pSM;
        int pageNo;
        int status;

        if (NandPort == 0)
                pSM = pSM0;
        else
                pSM = pSM1;


        nuc900_nand_write(REG_FMICSR, 0x08);

        if (gbIsSupport2PlaneNand) {
                PBA += (pSM->uLibStartBlock/2);
                if ((pSM->uLibStartBlock%2) != 0)
                        PBA++;

                pageNo = (PBA << 1) * pSM->uPagePerBlock + page;
                status = fmiSM_Write_2Plane(pSM, pageNo, 0, (u32)buff);

                return (status);
        } else {
                PBA += pSM->uLibStartBlock;
                pageNo = PBA * pSM->uPagePerBlock + page;
                if (pSM->bIs2KPageSize) {	/* 2KB */
                        status = fmiSM_Write_2K(pSM, pageNo, 0, (u32)buff);

                        return (status);
                } else {	/* 512B */
                        status = fmiSM_Write_512(pSM, pageNo, (u32)buff);
                        return (status);
                }
        }
}

static int sicSM_is_page_dirty(int NandPort, int PBA, int page)
{

        FMI_SM_INFO_T *pSM;
        int pageNo;
        u8 data0, data1;

        if (NandPort == 0)
                pSM = pSM0;
        else
                pSM = pSM1;

        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (gbIsSupport2PlaneNand) {
                PBA += (pSM->uLibStartBlock/2);
                if ((pSM->uLibStartBlock%2) != 0)
                        PBA++;

                pageNo = (PBA << 1) * pSM->uPagePerBlock + page;
                fmiSM_Read_2K_RA(pSM, pageNo, 2050);
        } else {
                PBA += pSM->uLibStartBlock;
                pageNo = PBA * pSM->uPagePerBlock + page;
                if (pSM->bIs2KPageSize)	/* 2KB */
                        fmiSM_Read_2K_RA(pSM, pageNo, 2050);
                else	/* 512B */
                        fmiSM2BufferM_RA(pSM, pageNo, 2);
        }
        data0 = nuc900_nand_read(REG_SMDATA);
        data1 = nuc900_nand_read(REG_SMDATA);

        if (!pSM->bIs2KPageSize)	/* 512B */
                fmiSM_Reset();

        if ((data0 == 0) && (data1 == 0x00))
                return 1;	// used page

        return 0;	// un-used page
}


static int sicSM_is_valid_block(int NandPort, int PBA)
{

        FMI_SM_INFO_T *pSM;

        if (NandPort == 0)
                pSM = pSM0;
        else
                pSM = pSM1;

        if (gbIsSupport2PlaneNand) {
                PBA += (pSM->uLibStartBlock/2);
                if ((pSM->uLibStartBlock%2) != 0)
                        PBA++;
        } else
                PBA += pSM->uLibStartBlock;

        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (fmiCheckInvalidBlock(pSM, PBA) == 1)	// invalid block
                return 0;
        else
                return 1;	// valid block
}


static int sicSMblock_erase(int NandPort, int PBA)
{

        FMI_SM_INFO_T *pSM;
        u32 page_no;

        if (NandPort == 0)
                pSM = pSM0;
        else
                pSM = pSM1;
        PBA += pSM->uLibStartBlock;
        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (fmiCheckInvalidBlock(pSM, PBA) != 1) {
                page_no = PBA * pSM->uPagePerBlock;		// get page address

                //fmiSM_Reset();

                /* clear R/B flag */
                while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
                nuc900_nand_write(REG_SMISR, 0x400);

                nuc900_nand_write(REG_SMCMD, 0x60);		// erase setup command
                nuc900_nand_write(REG_SMADDR, (page_no & 0xff));		// PA0 - PA7
                if (!pSM->bIsMulticycle)
                        nuc900_nand_write(REG_SMADDR, ((page_no >> 8) & 0xff)|0x80000000);		// PA8 - PA15
                else {
                        nuc900_nand_write(REG_SMADDR, ((page_no >> 8) & 0xff));		// PA8 - PA15
                        nuc900_nand_write(REG_SMADDR, ((page_no >> 16) & 0xff)|0x80000000);		// PA16 - PA17
                }

                nuc900_nand_write(REG_SMCMD, 0xd0);		// erase command

                if (!fmiSMCheckRB())
                        return FMI_SM_RB_ERR;

                nuc900_nand_write(REG_SMCMD, 0x70);		// status read command
                if (nuc900_nand_read(REG_SMDATA) & 0x01) {	// 1:fail; 0:pass
#ifdef DEBUG
                        printk("sicSMblock_erase error!!\n");
#endif
                        return FMI_SM_STATUS_ERR;
                }
        } else
                return FMI_SM_INVALID_BLOCK;

        return 0;
}

static int sicSMblock_erase_2Plane(int NandPort, int PBA)
{

        FMI_SM_INFO_T *pSM;
        u32 page_no;

        if (NandPort == 0)
                pSM = pSM0;
        else
                pSM = pSM1;

        PBA += (pSM->uLibStartBlock / 2);
        if ((pSM->uLibStartBlock % 2) != 0)
                PBA++;

        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (fmiCheckInvalidBlock(pSM, PBA) == 1)
                return FMI_SM_INVALID_BLOCK;

        page_no = (PBA << 1) * pSM->uPagePerBlock;		// get page address

        //fmiSM_Reset();

        /* clear R/B flag */
        while (!(nuc900_nand_read(REG_SMISR) & 0x40000));
        nuc900_nand_write(REG_SMISR, 0x400);

        /* first block */
        nuc900_nand_write(REG_SMCMD, 0x60);		// erase setup command

        nuc900_nand_write(REG_SMADDR, 0);		// PA0 - PA7
        nuc900_nand_write(REG_SMADDR, 0);		// PA8 - PA15
        nuc900_nand_write(REG_SMADDR, ((page_no  >> 16) & 0x0f)|0x80000000);		// PA16 - PA17

        nuc900_nand_write(REG_SMCMD, 0x60);		// erase setup command

        nuc900_nand_write(REG_SMADDR, 0x80);		// PA0 - PA7
        nuc900_nand_write(REG_SMADDR, ((page_no >> 8) & 0xff));		// PA8 - PA15
        nuc900_nand_write(REG_SMADDR, ((page_no >> 16) & 0x0f)|0x80000000);		// PA16 - PA17

        nuc900_nand_write(REG_SMCMD, 0xd0);		// erase command

        if (!fmiSMCheckRB())
                return FMI_SM_RB_ERR;

        nuc900_nand_write(REG_SMCMD, 0x70);		// status read command
        if (nuc900_nand_read(REG_SMDATA) & 0x01) {	// 1:fail; 0:pass
#ifdef DEBUG
                printk("sicSMblock_erase_2Plane error!!\n");
#endif
                return FMI_SM_STATUS_ERR;
        }
        return 0;
}


static int sicSMchip_erase(int NandPort)
{

        int i, status=0;
        FMI_SM_INFO_T *pSM;

        if (NandPort == 0)
                pSM = pSM0;
        else
                pSM = pSM1;

        // enable SM
        nuc900_nand_write(REG_FMICSR, 0x08);

        if (gbIsSupport2PlaneNand) {
                for (i=0; i<=(pSM->uBlockPerFlash/2); i++) {
                        status = sicSMblock_erase_2Plane(NandPort, i);
#ifdef DEBUG
                        if (status < 0)
                                printf("SM block erase fail <%d>!!\n", i);
#endif
                }
        } else {
                // erase all chip
                for (i=0; i<=pSM->uBlockPerFlash; i++) {
                        status = sicSMblock_erase(NandPort, i);

                        if (status < 0)
                                printk("SM block erase fail <%d>!!\n", i);

                }
        }
        return status;
}

/* driver function */
int nandInit0(NDISK_T *NDISK_info)
{
        return (sicSMInit(0, NDISK_info));
}

int nandpread0(int PBA, int page, u8 *buff)
{
        return (sicSMpread(0, PBA, page, buff));
}

int nandpwrite0(int PBA, int page, u8 *buff)
{
        return (sicSMpwrite(0, PBA, page, buff));
}

int nand_is_page_dirty0(int PBA, int page)
{
        return (sicSM_is_page_dirty(0, PBA, page));
}

int nand_is_valid_block0(int PBA)
{
        return (sicSM_is_valid_block(0, PBA));
}

int nand_block_erase0(int PBA)
{
        if (gbIsSupport2PlaneNand)
                return (sicSMblock_erase_2Plane(0, PBA));
        else
                return (sicSMblock_erase(0, PBA));
}

int nand_chip_erase0(void)
{
        return (sicSMchip_erase(0));
}

int nand_ioctl_0(int param1, int param2, int param3, int param4)
{
        return 0;
}

int nandInit1(NDISK_T *NDISK_info)
{
        return (sicSMInit(1, NDISK_info));
}

int nandpread1(int PBA, int page, u8 *buff)
{
        return (sicSMpread(1, PBA, page, buff));
}

int nandpwrite1(int PBA, int page, u8 *buff)
{
        return (sicSMpwrite(1, PBA, page, buff));
}

int nand_is_page_dirty1(int PBA, int page)
{
        return (sicSM_is_page_dirty(1, PBA, page));
}

int nand_is_valid_block1(int PBA)
{
        return (sicSM_is_valid_block(1, PBA));
}

int nand_block_erase1(int PBA)
{
        if (gbIsSupport2PlaneNand)
                return (sicSMblock_erase_2Plane(1, PBA));
        else
                return (sicSMblock_erase(1, PBA));
}

int nand_chip_erase1(void)
{
        return (sicSMchip_erase(1));
}





