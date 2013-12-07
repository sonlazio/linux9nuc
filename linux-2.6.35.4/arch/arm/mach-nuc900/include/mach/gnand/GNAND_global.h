/* linux/include/asm/arch/gnand/GNAND_global.h
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

#ifndef _GNAND_GLOBAL_H_
#define _GNAND_GLOBAL_H_

//#define DBG_MSG		printk
//#define DBG_MSG		sysprintf
#define DBG_MSG(...)

#define FREE_BLOCK			0xFFFF
#define BAD_BLOCK			0xFFF0
#define OP_BLOCK			0xFFAA
#define L2PN_BLOCK			0xFF55

/* global string define */
#define P2LN_INFO_MAGIC		"GNAND"
#define P2LN_INFO_VERSION	"V1.01"
#define P2LN_INFO_DATE		"20110420_1"


#define P2LN_INFO_T		struct p2ln_info_t

struct p2ln_info_t {
        char    magic[8];		/* "GNAND"                               */
        char    version[8];		/* "V1.00"                               */
        char    date_code[16];	/* for example: "20080220_1"             */
        u16	op_block;    	/* OP block address                      */
        u16  old_op_block;	/* Old OP block address                  */
        u16  old_p2ln;		/* old P2LN block address                */
        u16  old_p2ln1;		/* old P2LN1 block address               */
        u32  block;			/* block per zone                        */
};


/*-------------------------------------------------------------------*/
/* Operation history                                                 */
/*-------------------------------------------------------------------*/
#define OP_LINK				"LINK"
#define OP_RELINK			"RELINK"
#define OP_P2LN				"P2LN"

#define GNOP_LINK_T		struct gnop_link_t
#define GNOP_RELINK_T	struct gnop_relink_t
#define GNOP_P2LN_T		struct gnop_p2ln_t

struct gnop_link_t {
        char  	op[8];			/* string "LINK"  */
        u16	lba;
        u16	pba;
        char    reserved[20];
} ;


struct gnop_relink_t {
        char  	op[8];  		/* string "RELINK" */
        u16	lba;
        u16	old_pba;
        u16  new_pba;
        u16	start_page;
        u16	page_cnt;
        u16	check_mark;
        char    reserved[12];
} ;


struct gnop_p2ln_t {
        char	op[8];			/* string "P2LN" */
        u16	old_p2ln;
        u16	old_op;
        u16	new_p2ln;
        u16	new_op;
        u16	old_p2ln1;
        u16	new_p2ln1;
        char	reserved[12];
} ;


/*-------------------------------------------*/
/* global variable extern					 */
/*-------------------------------------------*/
extern u8 *_gnand_pDMABuffer;
extern u8 *_gnand_pUBBuffer;



/*-------------------------------------------------------------------*/
/* Export functions (GNAND library internal)                         */
/*-------------------------------------------------------------------*/
int  GNAND_OP_LinkNew(NDISK_T *ptNDisk, u16 LBlockAddr, u16 *PBlockAddr);
int  GNAND_OP_ReLink(NDISK_T *ptNDisk, u16 LBlockAddr, u16 *PBlockAddr, int nStartPage, int nPageCnt, u8 bIsBegin);
int  GNAND_OP_ReCover(NDISK_T *ptNDisk, u16 LBlockAddr, int nStartPage, int nPageCnt);
int  GNAND_OP_P2LN(NDISK_T *ptNDisk);
int  GNAND_UpdateP2LN(NDISK_T *ptNDisk);
int  GNAND_IsValidP2LN(NDISK_T *ptNDisk, u16 pba, P2LN_INFO_T *p2ln_info);

int  GNAND_ParseNandROM(NDISK_T *ptNDisk);
int  GNAND_ParseNandDisk(NDISK_T *ptNDisk, s8 bEraseIfNotGnandFormat);
int  GNAND_get_new_data_block(NDISK_T *ptNDisk, u16 LBlockAddr, u16 *PBlockAddr);
int  GNAND_check_empty(NDISK_T *ptNDisk, int PBA);

void  GNAND_DirtyPageSet(NDISK_T *ptNDisk, int pba, int page);
void  GNAND_DirtyPageClear(NDISK_T *ptNDisk, int pba, int page);
void  GNAND_DirtyPageClearBlock(NDISK_T *ptNDisk, int pba);

void fsDumpBufferHex(u8 *pucBuff, int nSize);

#endif 	/* _GNAND_GLOBAL_H_ */
