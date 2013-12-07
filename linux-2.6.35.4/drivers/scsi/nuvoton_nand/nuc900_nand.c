/* linux/driver/scsi/nuvoton_gnand/nuc900_nand.c
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
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi.h>
#include <scsi/scsi_device.h>

#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-fmi.h>
#include <mach/gnand/GNAND.h>
#include <mach/nuc900_nand.h>

#define REG_MFSEL	(W90X900_VA_GCR + 0xC)


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

extern int  GNAND_read(NDISK_T *ptNDisk, u32 nSectorNo, int nSectorCnt, u8 *buff)  __attribute__ ((weak));
extern int  GNAND_InitNAND(NDRV_T *ndriver, NDISK_T *ptNDisk, s8 bEraseIfNotGnandFormat)  __attribute__ ((weak));
extern int  GNAND_write(NDISK_T *ptNDisk, u32 nSectorNo, int nSectorCnt, u8 *buff)  __attribute__ ((weak));
extern u8 *_gnand_pDMABuffer __attribute__ ((weak));

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

NDISK_T *ptNDisk;



struct nand_hostdata nand_host, nand_host1;
NAND_FLAG_SETTING nand_flagSetting;
unsigned int nand_registered = 0;
unsigned int nand_registered1 = 0;
volatile int nand_has_command = 0,nand_has_command1 = 0, nand_event = 0, nand_event1 = 0;
static DECLARE_WAIT_QUEUE_HEAD(sd_wq);
static DECLARE_WAIT_QUEUE_HEAD(scsi_wq);
static DECLARE_WAIT_QUEUE_HEAD(scsi_wq1);
static DECLARE_WAIT_QUEUE_HEAD(nand_event_wq);
static DECLARE_WAIT_QUEUE_HEAD(nand_event_wq1);
static DECLARE_MUTEX(sem);
static DECLARE_MUTEX(sem_r);
static DECLARE_MUTEX(sem_w);

int nand_curMap[2];
int nand_curMap2[6];
volatile int nand_i_o = 0;
volatile int nand1_i_o = 0;
volatile bool  nand_fmi_bIsSDDataReady = 0;

u8 _fmi_pNANDBuffer1[1024*512] __attribute__((aligned (32)));
u8 *_fmi_gptr1;
u32 _fmi_ucNANDBuffer;
u32 _fmi_pNANDBuffer;


bool volatile  _fmi_bIsSMDataReady=0;
extern FMI_SM_INFO_T *pSM0,*pSM1;

static void nand_done(struct scsi_cmnd  *cmd )
{

        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
        wake_up_interruptible(&sd_wq);

}

static irqreturn_t fmi_interrupt(int irq, void *devid)
{
        unsigned int volatile isr;
        // SM interrupt status
        isr = nuc900_nand_read(REG_SMISR);

        //DMA read/write transfer is done
        if (isr & 0x01) {
                _fmi_bIsSMDataReady = 1;
                nuc900_nand_write(REG_SMISR, 0x01);
                return IRQ_HANDLED;
        } else {
                return IRQ_NONE;
        }

}

static u16 inline get_be16(u8 *buf)
{
        return ((u16) buf[0] << 8) | ((u16) buf[1]);
}

static u32 inline get_be32(u8 *buf)
{
        return ((u32) buf[0] << 24) | ((u32) buf[1] << 16) |
               ((u32) buf[2] << 8) | ((u32) buf[3]);
}

static void inline put_be16(u16 val, u8 *buf)
{
        buf[0] = val >> 8;
        buf[1] = val;
}

static void inline put_be32(u32 val, u8 *buf)
{
        buf[0] = val >> 24;
        buf[1] = val >> 16;
        buf[2] = val >> 8;
        buf[3] = val;
}

#ifdef CONFIG_NUCXXX_SD_AUTOMOUNT
//for auto mount & umont use
static int nuvoton_hotplug(char *cmd, char *parameter1, char *parameter2, char *parameter3)
{
        char *argv[5],*env[4];
        argv[0] = cmd;
        argv[1] = parameter1;
        argv[2] = parameter2;
        argv[3] = parameter3;
        argv[4] = NULL;

        env[0] = "HOME=/" ;
        env[1] = "TERM=linux" ;
        env[2] = "PATH=/bin:/sbin" ;
        env[3] = NULL ;

        int ret;
        ret = call_usermodehelper(argv[0], argv, env, 1);
        if (ret==0)
                MSG2("Run nuvoTon_hotplug successful!\n");
        else
                printk("novoton_hotplug faild with %d\n",ret);

        return 0;
}
#endif

static void nand_make_sense_buffer(struct nand_hostdata *dev,int key, int asc, int ascq)
{

        unsigned char *sense_buffer;

        sense_buffer = dev->cmd->sense_buffer;
        memset(sense_buffer, 0, 18);

        sense_buffer[0] = 0x70;		// error code
        sense_buffer[2] = key;
        sense_buffer[7] = 0x0A;		// sdditional sense length
        sense_buffer[12] = asc;
        sense_buffer[13] = ascq;

}

extern FMI_SM_INFO_T *pSM0;

static int nand_nand_card_reset(void)
{
        int retval;
	int nSectorPerPage;

        mdelay(100);

        ptNDisk = (NDISK_T *)kmalloc(sizeof(NDISK_T),GFP_KERNEL);
        if (ptNDisk == NULL)
                printk("malloc error!!\n");

        retval=GNAND_InitNAND(&_nandDiskDriver0, ptNDisk, 1);
        if (retval) {
                printk("GNAND init failed !!!!!!!! \n");
                return -1;
        }
  //nand_host.nTotalSectors = pSM0->uSectorPerFlash - pSM0->uLibStartBlock;
  nSectorPerPage = ptNDisk->nPageSize / 512;
  nand_host.nTotalSectors = ptNDisk->nZone * (ptNDisk->nLBPerZone-1) * ptNDisk->nPagePerBlock * nSectorPerPage;

        nand_host.nSectorSize = 512;
        nand_host.nCapacityInByte = nand_host.nTotalSectors * nand_host.nSectorSize;

        return SD_SUCCESS;
}




static void nand_host_reset(u32 uChipSel)
{
        struct nand_hostdata *dev=&nand_host;

        // enable SM
        //nuc900_nand_write(REG_FMICSR, 0x08);  // ya..

//	 nuc900_nand_write(REG_SMTCR, 0x10305);     //  AHB:cpu=1:1


        /* init SM interface */
        nuc900_nand_write(REG_SMCSR, (nuc900_nand_read(REG_SMCSR)&0xf8ffffc0)|0x01000020);	// enable ecc4

        if (down_interruptible(&dmac_sem))
                return;
        while (nuc900_nand_read(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe


        // enable all
        nuc900_nand_write(REG_DMACCSR, nuc900_nand_read(REG_DMACCSR) | DMACCSR_DMACEN); //enable DMAC for FMI

        /* enable all interrupt */
        nuc900_nand_write(REG_DMACIER, DMACIER_TABORT_IE); //Enable target abort interrupt generation during DMA transfer
        nuc900_nand_write(REG_FMIIER, FMIIER_DTA_IE); //Enable DMAC READ/WRITE target abort interrupt generation


        nand_host.nSectorSize = 512;

        dev->state = 0;
        dev->sense = 0;

        nand_flagSetting.bCrcCheck = 1;
        nand_flagSetting.needReset = 1;

        nand_host.myID = 0;

        up(&dmac_sem);
}




static int nand_card_init(void)
{
        int retval = SD_FAILED;
        struct nand_hostdata *dev=NULL;
        if (nand_flagSetting.curCard == 0)
                dev = &nand_host;
        if (nand_flagSetting.curCard == 1)
                dev = &nand_host1;

        nand_host_reset(0);

        if (nand_flagSetting.curCard == 0) {
                if ( nand_flagSetting.bCardExist == 0)
                        return SD_REMOVED;
        }

        retval = nand_nand_card_reset();

        if (retval)
                return retval;

        MSG2("Init SD Card Result = %d\n", retval);

        return retval;
}



static void nand_check_valid_medium(struct nand_hostdata *dev)
{
        if (dev->myID == 0) { //card 0
                if (nand_flagSetting.bCardExist == 0) {
                        MSG( "card not exist\n" );
                        return;
                }

                if (nand_flagSetting.bMediaChanged != 0 ) {
                        nand_flagSetting.bMediaChanged = 0;
                        if (!nand_card_init())
                                nand_flagSetting.bInitSuccess = 1;
                        else
                                nand_flagSetting.bInitSuccess = 0;
                }
        }

        if (dev->myID == 1) { //card 1
                if (nand_flagSetting.bCardExist1 == 0) {
                        MSG( "card 1 not exist\n" );
                        return;
                }

                if (nand_flagSetting.bMediaChanged1 != 0 ) {
                        nand_flagSetting.bMediaChanged1 = 0;
                        if (!nand_card_init())
                                nand_flagSetting.bInitSuccess1 = 1;
                        else
                                nand_flagSetting.bInitSuccess1 = 0;
                }
        }
}

static unsigned char *nand_get_buffer(struct scsi_cmnd *cmd, int * length)
{
        struct scatterlist *p;
        unsigned char *buf;

        p = (struct scatterlist *)cmd->sdb.table.sgl;
//        buf =(unsigned char *)kmap_atomic(sg_page(p), KM_USER0) + p->offset;
        buf =(unsigned char *)page_address(sg_page(p)) + p->offset;
        *length = p->length;

        return buf;
}

static int nand_test_unit_ready(struct nand_hostdata *dev)
{

        struct scsi_cmnd  *cmd;
        int retval = 0;
        cmd = dev->cmd;

        nand_check_valid_medium(dev);

        if (dev->myID == 0) {
                if (nand_flagSetting.bCardExist != 0) {
                        if (nand_flagSetting.bMediaChanged == 0) {
                                if (nand_flagSetting.bInitSuccess != 0) {

                                        // SenseKey: SCSI_SENSE_NO_SENSE
                                        // AdditionalSenseCode: SCSI_ADSENSE_NO_SENSE
                                        nand_make_sense_buffer(dev,0x00, 0x00, 0x00 );

                                        retval = 0;
                                } else {
                                        // SenseKey: SCSI_SENSE_MEDIUM_ERROR
                                        // AdditionalSenseCode: SCSI_ADSENSE_INVALID_MEDIA
                                        nand_make_sense_buffer(dev,0x03, 0x30, 0x00 );
                                        retval = 1;
                                }
                        } else {
                                nand_flagSetting.bMediaChanged = 0;
                                // SenseKey: SCSI_SENSE_UNIT_ATTENTION
                                // AdditionalSenseCode: SCSI_ADSENSE_MEDIUM_CHANGED
                                nand_make_sense_buffer(dev,0x06, 0x28, 0x00 );
                                cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, SAM_STAT_CONDITION_MET);
                                retval = 1;
                        }
                } else {
                        // SenseKey: SCSI_SENSE_NOT_READY
                        // AdditionalSenseCode: SCSI_ADSENSE_NO_MEDIA_IN_DEVICE
                        nand_make_sense_buffer(dev,0x02, 0x3a, 0x00 );

                        // NOT_READY
                        //nand_make_sense_buffer(dev,0x02, 0x00, 0x00 );
                        //cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, CHECK_CONDITION);
                        retval = 1;
                }
        }

        if (dev->myID == 1) {
                if (nand_flagSetting.bCardExist1 != 0) {
                        if (nand_flagSetting.bMediaChanged1 == 0) {
                                if (nand_flagSetting.bInitSuccess1 != 0) {

                                        // SenseKey: SCSI_SENSE_NO_SENSE
                                        // AdditionalSenseCode: SCSI_ADSENSE_NO_SENSE
                                        nand_make_sense_buffer(dev,0x00, 0x00, 0x00 );

                                        retval = 0;
                                } else {
                                        // SenseKey: SCSI_SENSE_MEDIUM_ERROR
                                        // AdditionalSenseCode: SCSI_ADSENSE_INVALID_MEDIA
                                        nand_make_sense_buffer(dev,0x03, 0x30, 0x00 );
                                        retval = 1;
                                }
                        } else {
                                nand_flagSetting.bMediaChanged1 = 0;
                                // SenseKey: SCSI_SENSE_UNIT_ATTENTION
                                // AdditionalSenseCode: SCSI_ADSENSE_MEDIUM_CHANGED
                                nand_make_sense_buffer(dev,0x06, 0x28, 0x00 );
                                cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, SAM_STAT_CONDITION_MET);
                                retval = 1;
                        }
                } else {
                        // SenseKey: SCSI_SENSE_NOT_READY
                        // AdditionalSenseCode: SCSI_ADSENSE_NO_MEDIA_IN_DEVICE
                        nand_make_sense_buffer(dev,0x02, 0x3a, 0x00 );

                        // NOT_READY
                        //nand_make_sense_buffer(dev,0x02, 0x00, 0x00 );
                        //cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, CHECK_CONDITION);
                        retval = 1;
                }
        }

        return retval;

}


static int nand_scsi_read(struct scsi_cmnd *cmd , struct nand_hostdata *dev)
{
        unsigned int count, lba;
        volatile int DMAoffset = 0;
        int curDMAAddr;
        struct scatterlist *curList;

        if ( nand_test_unit_ready(dev) )
                goto quit;

        if ( cmd->cmnd[0] == READ_6) {
                lba = ((cmd->cmnd[1] & 0x1f) << 16) + get_be16(&cmd->cmnd[2]);
                count = (cmd->cmnd[4] & 0xff);
        } else {
                lba = get_be32(&cmd->cmnd[2]);
                count = get_be16(&cmd->cmnd[7]);
        }
        //printk("--Read from = %d(lba),  count = %d   \n", lba, count);
        if ( lba > dev->nTotalSectors || (lba + count) > dev->nTotalSectors) {
                //printk("--Read from = %d(lba),  count = %d   \n", lba, count);
                dev->sense = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
                goto quit_with_make_sense;
        }

        dev->firstList = (struct scatterlist *)cmd->sdb.table.sgl;

        dev->curList = 0;
        dev->curOffset = 0;
        dev->sense = SS_NO_SENSE;


        wait_event_interruptible(sd_wq, dev->state == SD_STATE_NOP);

        if (GNAND_read(ptNDisk,lba,count,_fmi_pNANDBuffer1))
                goto quit_with_make_sense;

        /* connect with SCSI */

        curList = &nand_host.firstList[nand_host.curList];
        MSG2("sd_host.curOffset  %d\n",nand_host.curOffset);
        MSG2("curList->length  %d\n",(curList->length)/512);
        MSG2("count  %d\n",count);
        curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host.curOffset;

        while ((curList->length)<(count*512)) {
                memcpy((char *)curDMAAddr,(char *) _fmi_pNANDBuffer1+DMAoffset, curList->length);

                count = count-(curList->length)/512;
                DMAoffset+=(curList->length);
                nand_host.curOffset = 0;
                nand_host.curList ++;
                curList = &nand_host.firstList[nand_host.curList];

                curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host.curOffset;

                MSG2("curList->length  %d\n",(curList->length)/512);
                MSG2("count  %d\n",count);
        }


        if ((curList->length)==(count*512)) {
                MSG2("=\n");
                MSG2("curList->length  %d\n",curList->length/512);

                memcpy((char *)curDMAAddr, (char *)_fmi_pNANDBuffer1+DMAoffset, curList->length);
                DMAoffset=0;
        }
        //=====================
        nand_done(cmd);
        dev->state = SD_STATE_NOP;
        wait_event_interruptible(sd_wq, dev->state == SD_STATE_NOP);

        return 0;

quit_with_make_sense:

        nand_make_sense_buffer(dev,dev->sense >> 16,
                               dev->sense >> 8,
                               dev->sense);
quit:
        return -1;
}




static int nand_scsi_write(struct scsi_cmnd  *cmd, struct nand_hostdata *dev)
{
        unsigned int count, lba,write_count;
        volatile int DMAoffset = 0;
        int curDMAAddr;
        struct scatterlist *curList;

        if ( nand_test_unit_ready(dev)) {
                goto quit;
        }
        if ((dev->myID == 0) &&(nand_flagSetting.bWriteProtect == 1)) {
                dev->sense = SS_WRITE_PROTECTED;
                goto quit_with_make_sense;
        }
        if ((dev->myID == 1) &&(nand_flagSetting.bWriteProtect1 == 1)) {
                dev->sense = SS_WRITE_PROTECTED;
                goto quit_with_make_sense;
        }


        if ( cmd->cmnd[0] == WRITE_6) {
                lba = ((cmd->cmnd[1] & 0x1f) << 16) + get_be16(&cmd->cmnd[2]);
                count = (cmd->cmnd[4] & 0xff);
        } else {
                lba = get_be32(&cmd->cmnd[2]);
                count = get_be16(&cmd->cmnd[7]);
        }

        if ( lba > dev->nTotalSectors || (lba + count) > dev->nTotalSectors) {
                //printk("--write to = %d(lba),  length = %d  \n ", lba, count);
                dev->sense = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
                goto quit_with_make_sense;
        }

        dev->firstList = (struct scatterlist *)cmd->sdb.table.sgl;
        dev->curOffset = 0;
        dev->curList = 0;
        dev->sense = SS_NO_SENSE;
        write_count=count;
        wait_event_interruptible(sd_wq, dev->state == SD_STATE_NOP);
        //printk("nand_scsi_write--write to = %d(lba),  count = %d  \n ", lba, count);

        /* connect with SCSI */
        curList = &nand_host.firstList[nand_host.curList];
        MSG2("sd_host.curOffset  %d\n",nand_host.curOffset);
        MSG2("curList->length  %d\n",(curList->length)/512);
        MSG2("count  %d\n",count);
        curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host.curOffset;

        while ((curList->length)<(count*512)) {
                memcpy((char *)_fmi_pNANDBuffer1+DMAoffset, (char *)curDMAAddr, curList->length);
                MSG2("_fmi_pSMBuffer+DMAoffset  %x\n",_fmi_pNANDBuffer1+DMAoffset);
                count = count-(curList->length)/512;
                DMAoffset+=(curList->length);
                nand_host.curOffset = 0;
                nand_host.curList ++;
                curList = &nand_host.firstList[nand_host.curList];

                curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host.curOffset;

                MSG2("curList->length  %d\n",(curList->length)/512);
                MSG2("count  %d\n",count);
        }


        if ((curList->length)==(count*512)) {
                MSG2("=\n");
                MSG2("curList->length  %d\n",curList->length/512);

                memcpy((char *)_fmi_pNANDBuffer1+DMAoffset, (char *)curDMAAddr, curList->length);
                DMAoffset=0;
        }

        if (GNAND_write(ptNDisk,lba,write_count,_fmi_pNANDBuffer1))
                goto quit_with_make_sense;

        //=============
        nand_done(cmd );
        dev->state = SD_STATE_NOP;
        wait_event_interruptible(sd_wq, dev->state == SD_STATE_NOP);

        return 0;

quit_with_make_sense:
        nand_make_sense_buffer(dev,dev->sense >> 16,
                               dev->sense >> 8,
                               dev->sense);
quit:
        return -1;
}



static void nand_scsi_start_stop(struct nand_hostdata *dev)
{
        struct scsi_cmnd  *cmd;
        cmd = dev->cmd;

        if (cmd->cmnd[4] & 0x01) {		/* start */
                if (! nand_test_unit_ready(dev)) {
                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                }
        } else
                cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
}


static void nand_scsi_request_sense(struct scsi_cmnd  *cmd)
{

        int len;
        unsigned char *buffer = nand_get_buffer(cmd, &len);

        memcpy(buffer, cmd->sense_buffer, 18);

        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
}


static void nand_scsi_media_removal(struct nand_hostdata *dev)
{

        struct scsi_cmnd  *cmd = dev->cmd;
        //prevent removal cmnd is illegal since SD card can be removable
        if ( ( cmd->cmnd[4] & 0x01 ) )  {
                // SenseKey: SCSI_SENSE_ILLEGAL_REQUEST
                // AdditionalSenseCode: SCSI_ADSENSE_ILLEGAL_COMMAND
                nand_make_sense_buffer(dev,0x05, 0x20, 0x00 );
        } else {
                cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
        }
}

static void nand_scsi_test_unit_ready(struct nand_hostdata *dev)
{

        struct scsi_cmnd  *cmd = dev->cmd;
        if (!nand_test_unit_ready(dev) )
                cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
}

static void nand_scsi_inquiry(struct nand_hostdata *dev)
{
        int len;
        struct scsi_cmnd  *cmd = dev->cmd;
        unsigned char *buf = nand_get_buffer(cmd, &len);

        static char vendor_id[] = "NUVOTON";
        static char product_id[] = "GNAND DRIVER";
        static char release_id[]="2.00";

        if (dev->myID == 0) {
                if (nand_registered == 0) {
                        nand_registered = 1;
                        // stuff necessary inquiry data

                        memset(buf, 0, 36);
                        buf[1] = 0x80;	/* removable */
                        buf[2] = 0;		// ANSI SCSI level 2
                        buf[3] = 2;		// SCSI-2 INQUIRY data format //2
                        buf[4] = 0x1f;		// Additional length
                        // No special options

                        sprintf(buf + 8, "%-8s%-16s%-4s", vendor_id, product_id,
                                release_id);

                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                } else
                        cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, 0x00, 0x00);
        }

        if (dev->myID == 1) {
                if (nand_registered1 == 0) {

                        nand_registered1 = 1;
                        // stuff necessary inquiry data

                        memset(buf, 0, 36);
                        buf[1] = 0x80;	/* removable */
                        buf[2] = 0;		// ANSI SCSI level 2
                        buf[3] = 2;		// SCSI-2 INQUIRY data format //2
                        buf[4] = 0x1f;		// Additional length
                        // No special options

                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                } else
                        cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, 0x00, 0x00);
        }
}

static void nand_scsi_mode_sense(struct nand_hostdata *dev)
{
        int bProtectFlag, len;
        struct scsi_cmnd  *cmd = dev->cmd;
        unsigned char *buf = nand_get_buffer(cmd, &len);

        if ( nand_test_unit_ready(dev) )
                return;

        if (dev->myID == 0) {
                if ( nand_flagSetting.bCardExist ) {

                        memset(buf, 0, 8);

                        bProtectFlag = 0;
                        if (nand_flagSetting.bWriteProtect)
                                bProtectFlag = 0x80;

                        if ( cmd->cmnd[0] == MODE_SENSE ) {
                                buf[0] = 0x03;
                                buf[2] = bProtectFlag;
                        } else {
                                buf[1] = 0x06;
                                buf[3] = bProtectFlag;

                        }

                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                } else { // card is not in
                        nand_make_sense_buffer(dev, 0x02, 0x3a, 0x00 );
                        MSG( "SD 0 card may not be inserted!\n" );
                }
        }

        if (dev->myID == 1) {
                if ( nand_flagSetting.bCardExist1 ) {

                        memset(buf, 0, 8);

                        bProtectFlag = 0;
                        if (nand_flagSetting.bWriteProtect1)
                                bProtectFlag = 0x80;

                        if ( cmd->cmnd[0] == MODE_SENSE ) {
                                buf[0] = 0x03;
                                buf[2] = bProtectFlag;
                        } else {
                                buf[1] = 0x06;
                                buf[3] = bProtectFlag;

                        }

                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                } else { // card is not in
                        nand_make_sense_buffer(dev,0x02, 0x3a, 0x00 );
                        MSG( "SD 1 card may not be inserted!\n" );
                }
        }
}

static void nand_scsi_read_capacity(struct nand_hostdata *dev)
{
        struct scsi_cmnd  *cmd = dev->cmd;

        int len;
        unsigned char *buf = nand_get_buffer(cmd, &len);

        if (nand_test_unit_ready(dev)) {
                MSG( "SCSI_READ_CAPACITY - The unit not ready\r\n" );
                return;
        }

        memset(buf, 0, 8);

        put_be32(dev->nTotalSectors - 1, &buf[0]);	// Max logical block
        put_be32(512, &buf[4]);				// Block length

        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );

        return;
}

static void nand_scsi_process_cmd(struct nand_hostdata *dev)
{
        struct scsi_cmnd  * cmd;
        down_interruptible(&sem_r);

        cmd = dev->cmd;

        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_CHECK_CONDITION );

        nand_check_valid_medium(dev);

        switch (cmd->cmnd[0]) {

        case START_STOP:
                PDEBUG("SC_START_STOP_UNIT\n");
                nand_scsi_start_stop(dev);
                break;

        case REQUEST_SENSE:

                PDEBUG("SC_PREVENT_ALLOW_MEDIUM_REMOVAL\n");
                nand_scsi_request_sense(cmd);
                break;

        case ALLOW_MEDIUM_REMOVAL:
                PDEBUG("SC_PREVENT_ALLOW_MEDIUM_REMOVAL\n" );
                nand_scsi_media_removal(dev);
                break;

        case TEST_UNIT_READY:
                PDEBUG( "SC_TEST_UNIT_READY\n" );
                nand_scsi_test_unit_ready(dev );
                break;

        case INQUIRY:
                PDEBUG( "SC_INQUIRY\n" );
                nand_scsi_inquiry( dev );
                break;

        case READ_6:
                PDEBUG( "R" );
                if (down_interruptible(&fmi_sem))
                        break;
                nand_scsi_read(cmd,dev);
                up(&fmi_sem);
                break;

        case READ_10:
                PDEBUG( "R" );
                if (down_interruptible(&fmi_sem))
                        break;
//printk("nand fmi in - r\n");
                nand_scsi_read(cmd,dev);
//printk("nand fmi out - r\n");
                up(&fmi_sem);
                break;

        case WRITE_6:
                PDEBUG( "W" );
                if (down_interruptible(&fmi_sem))
                        break;
                nand_scsi_write(cmd,dev);
                up(&fmi_sem);
                break;

        case WRITE_10:
                PDEBUG( "W" );
                if (down_interruptible(&fmi_sem))
                        break;
//printk("nand fmi in - w\n");
                nand_scsi_write(cmd,dev);
//printk("nand fmi out - w\n");
                up(&fmi_sem);
                break;

        case MODE_SENSE:
                PDEBUG( "SC_MODE_SENSE_6\n" );
                nand_scsi_mode_sense( dev );
                break;

        case MODE_SENSE_10:
                PDEBUG( "SC_MODE_SENSE_6\n" );
                nand_scsi_mode_sense( dev );
                break;


        case READ_CAPACITY:
                PDEBUG( "SC_READ_CAPACITY\n" );
                nand_scsi_read_capacity( dev );
                break;

        default:
                PDEBUG("UNKNOWN command : %02x\n", cmd->cmnd[0] );
                nand_make_sense_buffer(dev, ILLEGAL_REQUEST, 0x20, 0x00 );
                cmd->result = SD_CMD_RESULT( DID_OK, 0, 2);
                break;

        }

        MSG2("Result : %08x\n", cmd->result);
        MSG2("Sense : [%02x%02x%02x]\n", cmd->sense_buffer[2],
             cmd->sense_buffer[12], cmd->sense_buffer[13]);

        if (cmd->result == 0) {
                MSG("Command Finished OK.\n");

                memset(cmd->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
        }
        cmd->scsi_done(cmd);
        up(&sem_r);
}

static int nand_queue_cmd( struct scsi_cmnd  *cmd, void (* done )( struct scsi_cmnd  * ) )
{
        int i;
        struct request *rq;
        struct gendisk *disk;
        char * curDiskName;
        char * diskLisk[] ={"sda","sdb","sdc","sdd","sde","sdf"};

        if (done == NULL)
                return 0;

        if (cmd->device->lun > 0) {
                cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, 0, 0);
                done(cmd);
                return 0;
        }

        rq = cmd->request;
        disk = rq->rq_disk;
        curDiskName = disk->disk_name;



        PDEBUG("**curDiskName**:%s\n",curDiskName);


        if ((nand_flagSetting.update == 1)&&((int)curDiskName!= 0xc)) { //update mapping
                MSG2("******setup map*********\n");
                nand_flagSetting.update = 0; //only when there is a new car insert or remove,it needs update.

                for (i=0; i<6; i++) {
                        if (!strcmp(curDiskName,diskLisk[i])) {
                                nand_curMap[0]=i; //sd port 0 <==> sdx
                                nand_curMap2[i]=0;

                        }
                }
        }

        down_interruptible(&sem);
        nand_host.cmd = cmd;
        cmd->scsi_done = done;
        cmd->result = 0;

        nand_has_command = 1;
        wake_up_interruptible(&scsi_wq);
        up(&sem);

        return 0;
}


static int nand_kernel_thread(void *param)
{
        daemonize("nandthread");

        for (;;) {

                wait_event_interruptible(scsi_wq, (nand_has_command != 0)&&(nand1_i_o ==0)); //no car inserting
                if (nand_has_command == SD_EVENT_QUIT)
                        break;
                nand_has_command = 0;
                nand_scsi_process_cmd(&nand_host);

        }

        MSG("BUG : Nand Kernel Thread Quit\n");

        return 0;
}



static const char* nand_info( struct Scsi_Host * psh)
{
        return "Nuvoton NUC900 GNAND DRIVER!";
}


static int nand_abort(struct scsi_cmnd * cmd)
{
        struct nand_hostdata *dev = &nand_host;
        //printk("nand_abort!!!!!!!!\n");

        if (nand_flagSetting.bCardExist && dev->state != SD_STATE_NOP) {
                dev->sense = SS_COMMUNICATION_FAILURE;
                cmd->scsi_done(cmd);
                wake_up_interruptible(&sd_wq);

                return 0;
        }

        return 1;
}





static int nand_reset( struct scsi_cmnd *cmd)
{
        nand_flagSetting.curCard = 0;
        nand_host_reset(0);

        return 0;
}

static int nand_bios_param(struct scsi_device *sdev,
                           struct block_device *bdev, sector_t capacity, int *info)
{
        info[0] = 2;		// heads

        info[1] = 61;		// sectors
        //info[1] = 63;

        info[2] = capacity >> 7;

        return 0;
}



static int nand_ioctl(struct scsi_device *scsi_dev, int cmd, void  *arg)
{
        return 0;
}



static struct scsi_host_template driver_template = {
        .name 					= "NAND0",
        .info					= nand_info,
        .queuecommand			= nand_queue_cmd,
        .eh_abort_handler 		= nand_abort,
        .eh_host_reset_handler		= nand_reset,
        .bios_param				= nand_bios_param,
        .ioctl					= nand_ioctl,
        .can_queue	     			= 1,
        .this_id        				= -1,
        .sg_tablesize   			= 128,
        .cmd_per_lun    			= 1,
        .unchecked_isa_dma		= 0,
        .use_clustering  			= ENABLE_CLUSTERING,
        .module					= THIS_MODULE,
};

//static struct scsi_host_template driver_template = WB_NAND_DRIVER;
//static struct scsi_host_template driver_template1 = WB_NAND_DRIVER1;

MODULE_LICENSE( "GPL" );

static int sd_add = 0;
static int sd_add1 = 0;

static void nand_device_release(struct device * dev)
{
}

static struct device nand_device = {
        .init_name = "nand_bus",
        .release = nand_device_release,
};

static int nand_bus_match(struct device *dev, struct device_driver *dev_driver)
{
        return 1;
}

static struct bus_type nand_bus = {
        .name = "nand_bus",
        .match = nand_bus_match,
};

static int nand_driver_probe(struct device *dev)
{
        struct Scsi_Host *shp;

        if (nand_flagSetting.curCard == 0) { //card 0
                shp = scsi_host_alloc(&driver_template, 0);
                if ( shp == NULL) {
                        printk(KERN_ERR "%s: scsi_register failed\n", __FUNCTION__);
                        return -ENODEV;
                }

                if ( scsi_add_host(shp, &nand_host.dev) ) {
                        printk(KERN_ERR "%s: scsi_add_host 0 failed\n", __FUNCTION__);
                        scsi_host_put(shp);
                        return -ENODEV;
                }

                scsi_scan_host(shp);

                nand_host.shost = shp;
        }

        return 0;
}

static int nand_driver_remove(struct device *dev)
{
        if (nand_flagSetting.sdRemove == 0) {
                scsi_remove_host(nand_host.shost);
                scsi_host_put(nand_host.shost);

        }

        if (nand_flagSetting.sdRemove == 1) {
                scsi_remove_host(nand_host1.shost);
                scsi_host_put(nand_host1.shost);

        }

        return 0;
}

static struct device_driver nand_driver = {
        .name 		= "nand_scsi",
        .bus		= &nand_bus,
        .probe          = nand_driver_probe,
        .remove         = nand_driver_remove,
};

static void nand_release_host(struct device *dev)
{
}

static int sd_add_card(void)
{

        int err;
 
        if ((sd_add1 == 0 )&&(nand_flagSetting.curCard == 1)) { //add card 1
                char name[] = "card1";
                sd_add1 = 1;
                memset(&nand_host1.dev, 0, sizeof(struct device ));
                nand_host1.DMAvaddr = (int )dma_alloc_writecombine(NULL, (256 * DMA_BLOCK_SIZE), &nand_host1.DMApaddr, GFP_KERNEL);

                nand_host1.dev.bus = &nand_bus;
                nand_host1.dev.parent = &nand_device;
                nand_host1.dev.release = nand_release_host;
                nand_host1.dev.init_name = name;

                err = device_register(&nand_host1.dev);
                if (err)
                        return err;



                return 0;
        }

        if ((sd_add == 0 )&&(nand_flagSetting.curCard == 0)) { //add card 0
                char name[] = "card0";
                sd_add = 1;
                memset(&nand_host.dev, 0, sizeof(struct device ));
                nand_host.DMAvaddr = (int )dma_alloc_writecombine(NULL, (256 * DMA_BLOCK_SIZE), &nand_host.DMApaddr, GFP_KERNEL);

                nand_host.dev.bus = &nand_bus;
                nand_host.dev.parent = &nand_device;
                nand_host.dev.release = nand_release_host;
                nand_host.dev.init_name = name;

                err = device_register(&nand_host.dev);
                if (err)
                        return err;

                return 0;
        }

        if (nand_flagSetting.update == 1)
                nand_flagSetting.update = 0;
        return 0;

}


static int nand_event_thread(void *unused)
{
        int event;

        daemonize("nandeventd");

        for (;;) {

                wait_event_interruptible(nand_event_wq, nand_event != SD_EVENT_NONE);
                event = nand_event;
                nand_event = 0;

                down_interruptible(&sem_w);

                switch (event) {
                case SD_EVENT_ADD:
                        nand_i_o = 1;
                        while (nand_host1.state != SD_STATE_NOP); //wait last cmd done
                        nand_flagSetting.bCardExist = 1;
                        nand_flagSetting.bMediaChanged = 1;
                        nand_flagSetting.bWriteProtect = 0;
                        nand_flagSetting.update = 1;
                        nand_flagSetting.curCard = 0;

                        sd_add_card();
                        nand_i_o = 0;
                        wake_up_interruptible(&scsi_wq1);
                        break;



                case SD_EVENT_QUIT:
                        goto quit;

                default:
                        MSG("NO THIS EVENT !!\n");
                        break;
                }

                up(&sem_w);
        }

quit:
        MSG("Quit Event Thread\n");

        return 0;

}

extern u8 *_gnand_pDMABuffer;
extern u8 *_gnand_gDMABuffer;

static int __init nand_init(void)
{

        struct nand_hostdata *dev;
        //printk("NAND: nand_init!!!!!\n");
        // enable FMI and DMAC CLOCK
        nuc900_nand_write(REG_CLKEN, nuc900_nand_read(REG_CLKEN) | 0x30);

        nuc900_nand_write(REG_MFSEL, (nuc900_nand_read(REG_MFSEL) | 0x4)&0xfffffff7);


        device_register(&nand_device);
        bus_register(&nand_bus);
        driver_register(&nand_driver);

        driver_template.proc_name = "nand_scsi_0";
        dev=&nand_host;

        if (request_irq(20, fmi_interrupt, IRQF_SHARED, "900_NAND", dev)) {
                printk("NAND: Request IRQ error\n");
                return -1;
        }

        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
        while (nuc900_nand_read(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe
        // DMAC Initial
        nuc900_nand_write(REG_DMACCSR, 0x00000003);
        //Enable DMAC
        nuc900_nand_write(REG_DMACCSR, 0x00000001);
        // Enable target abort interrupt generation during DMA transfer.
        nuc900_nand_write(REG_DMACIER, 0x00000001);

        up(&dmac_sem);

        kernel_thread(nand_event_thread, NULL, 0);

        kernel_thread(nand_kernel_thread, NULL, 0);

        _fmi_pNANDBuffer = (int )dma_alloc_writecombine(NULL, 512*4, &_fmi_ucNANDBuffer, GFP_KERNEL);
        _fmi_gptr1 = _fmi_pNANDBuffer1;
#if 1
        if (_gnand_pDMABuffer == 0)
                _gnand_pDMABuffer = (u8 *)kmalloc(512*4,GFP_KERNEL);
#endif
        nand_host.state = 0;
        nand_host.sense = 0;

        nand_host_reset(0);

        nand_event = SD_EVENT_ADD;
        wake_up_interruptible(&nand_event_wq);

        //printk("nand_init exit 0x%x\n", (int)dev);

        return 0;
}

static void __exit nand_exit(void)
{
        dma_free_writecombine(NULL, (512*4), &_fmi_pNANDBuffer, _fmi_ucNANDBuffer);
//  dma_free_writecombine(NULL, (32 * DMA_BLOCK_SIZE), _fmi_pNANDBuffer1, _fmi_ucNANDBuffer1);
//  dma_free_writecombine(NULL, (256 * DMA_BLOCK_SIZE), &nand_host.DMAvaddr, nand_host.DMApaddr);

        free_irq(IRQ_FMI, NULL);
        //free_irq(IRQ_DMAC, NULL);

        driver_unregister(&nand_driver);
        bus_unregister(&nand_bus);
        device_unregister(&nand_device);
}

module_init(nand_init);
module_exit(nand_exit);
MODULE_VERSION( "V1.00" );
