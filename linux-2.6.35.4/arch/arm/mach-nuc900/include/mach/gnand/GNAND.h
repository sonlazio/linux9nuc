/* linux/include/asm/arch/gnand/GNAND.h
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


#ifndef _GNAND_H_
#define _GNAND_H_


#define GNAND_OK					      0
#define GNERR_READ_L2P_MISS			1 	/* read block not found in L2P        */
#define GNAND_ERR					      0xFFFFC000

/* GENERAL ERRORs */
#define GNERR_GENERAL               (GNAND_ERR+1)		/* general error                      */
#define GNERR_MEMORY_OUT      		  (GNAND_ERR+0x5)		/* memory not enough                  */
#define GNERR_GNAND_FORMAT    		  (GNAND_ERR+0x10) 	/* not GNAND format                   */
#define GNERR_FAT_FORMAT            (GNAND_ERR+0x15) 	/* NAND disk was not formatted as FAT */
#define GNERR_BLOCK_OUT				      (GNAND_ERR+0x20) 	/* there's no available free blocks   */
#define GNERR_P2LN_SYNC				      (GNAND_ERR+0x25)
#define GNERR_READONLY_NAND			(GNAND_ERR+0x26)	/* XtraROM */

/* for NAND driver return value */
#define GNERR_IO_ERR				        (GNAND_ERR+0x30) 	/* NAND read/write/erase access failed*/
#define GNERR_NAND_NOT_FOUND		    (GNAND_ERR+0x40) 	/* NAND driver cannot find NAND disk  */
#define GNERR_UNKNOW_ID				      (GNAND_ERR+0x42) 	/* Not supported NAND ID              */


/*-------------------------------------------------------------------*/
/*  NAND driver function set                                         */
/*-------------------------------------------------------------------*/
#define NDISK_T		struct ndisk_t
#define NDRV_T		struct ndrv_t



typedef struct p2lm_t {
        u16  lba;			/* logical block address                 */
        u16  age;			/* times this block has been used        */
}  P2LM_T;


typedef struct l2pm_t {
        u16	pba;           	/* physical block address                */
        u16  reserved;      	/* reserved for future used              */
}  L2PM_T;


#define NAND_TYPE_SLC		0x01
#define NAND_TYPE_MLC		0x00


/*-------------------------------------------------------------------*/
/*  NAND disk infotmation. This information was provided             */
/*  by NAND driver.                                                  */
/*-------------------------------------------------------------------*/
struct ndisk_t {
        int  	vendor_ID;
        int  	device_ID;
        int  	NAND_type;
        int  	nZone;            	/* number of zones                       */
        int  	nBlockPerZone;     	/* blocks per zone                       */
        int  	nPagePerBlock;     	/* pages per block                       */
        int  	nLBPerZone;        	/* logical blocks per zone               */
        int  	nPageSize;
        int  	nStartBlock;        /* available start block                 */
        NDRV_T	*driver;			/* NAND driver to work on this NAND disk */
        int  	nNandNo;
        void  	*pDisk;
        int  	reserved[60];
        int	need2L2PN;
        int  	p2ln_block1;

        /* for GNAND internal used */
        P2LM_T	*p2lm;
        L2PM_T	*l2pm;
        u8	*dp_tbl;		/* dirty page bit map */
        u16  db_idx[16];		/* data block search index, up to 8 zone */
        u16	p2ln_block;		/* P2LN block No. */
        u16	op_block;		/* OP block No. */
        int  	op_offset;		/* operation index */
        u8   last_op[32];	/* the last op code in op table */

        int		err_sts;
        struct ndisk_t  *next;
};


struct ndrv_t {
        int  (*init)(NDISK_T *NDInfo);
        int  (*pread)(int nPBlockAddr, int nPageNo, u8 *buff);
        int  (*pwrite)(int nPBlockAddr, int nPageNo, u8 *buff);
        int  (*is_page_dirty)(int nPBlockAddr, int nPageNo);
        int  (*is_valid_block)(int nPBlockAddr);
        int  (*ioctl)(int param1, int param2, int param3, int param4);
        int  (*block_erase)(int nPBlockAddr);
        int  (*chip_erase)(void);
        void *next;
} ;


/*-------------------------------------------------------------------*/
/* Export functions                                                  */
/*-------------------------------------------------------------------*/
int  GNAND_InitNAND(NDRV_T *ndriver, NDISK_T *ptNDisk, s8 bEraseIfNotGnandFormat);
int  GNAND_MountNandDisk(NDISK_T *ptNDisk);
int  GNAND_read(NDISK_T *ptNDisk, u32 nSectorNo, int nSectorCnt, u8 *buff);
int  GNAND_write(NDISK_T *ptNDisk, u32 nSectorNo, int nSectorCnt, u8 *buff);
int  GNAND_block_erase(NDISK_T *ptNDisk, int pba);
int  GNAND_chip_erase(NDISK_T *ptNDisk);
void GNAND_CloseNAND(NDISK_T *ptNDisk);

#endif 	/* _GNAND_H_ */

