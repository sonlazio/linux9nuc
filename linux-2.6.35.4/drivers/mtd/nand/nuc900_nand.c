/*
 * Copyright © 2009 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/blkdev.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>


#include <mach/map.h>
#include <mach/regs-fmi.h>
#define REG_MFSEL	(W90X900_VA_GCR + 0xC)
/*
#define REG_FMICSR   	0x00
#define REG_SMCSR    	0xa0
#define REG_SMISR    	0xac
#define REG_SMCMD    	0xb0
#define REG_SMADDR   	0xb4
#define REG_SMDATA   	0xb8
*/
#define RESET_FMI	0x01
#define NAND_EN		0x08
#define READYBUSY	(0x01 << 18)

#define SWRST		0x01
#define PSIZE		(0x01 << 3)
#define DMARWEN		(0x03 << 1)
#define BUSWID		(0x01 << 4)
#define ECC4EN		(0x01 << 5)
#define WP		(0x01 << 24)
#define NANDCS		(0x03 << 25)
#define ENDADDR		(0x01 << 31)

#define NANDCS1		(0x01 << 25)
#define ECCBYTE512 6
#define ECCBYTE2K  12

/* DMAC Control and Status Register (DMACCSR) */
#define DMACCSR_DMACEN		(1)
#define DMACCSR_SW_RST		(1<<1)
#define DMACCSR_SG_EN1		(1<<2)
#define DMACCSR_SG_EN2		(1<<3)
#define DMACCSR_ATA_BUSY	(1<<8)
#define DMACCSR_FMI_BUSY	(1<<9)

/******************************************/
#define FMI_ERR_ID	0xFFFF0100

#define FMI_TIMEOUT				(FMI_ERR_ID|0x01)
/* NAND error */
#define FMI_SM_INIT_ERROR		(FMI_ERR_ID|0x20)
#define FMI_SM_RB_ERR			(FMI_ERR_ID|0x21)
#define FMI_SM_STATE_ERROR		(FMI_ERR_ID|0x22)
#define FMI_SM_ECC_ERROR		(FMI_ERR_ID|0x23)
#define FMI_SM_STATUS_ERR		(FMI_ERR_ID|0x24)
#define FMI_SM_ID_ERR			(FMI_ERR_ID|0x25)
#define FMI_SM_INVALID_BLOCK	(FMI_ERR_ID|0x26)



//#define nuc900_nand_debug(fmt,args...) printk(fmt,##args)
#define nuc900_nand_debug(fmt,args...) 


#define read_data_reg(dev)		\
	__raw_readl(REG_SMDATA)

#define write_data_reg(dev, val)	\
	__raw_writel((val), REG_SMDATA)

#define write_cmd_reg(dev, val)		\
	__raw_writel((val), REG_SMCMD)

#define write_addr_reg(dev, val)	\
	__raw_writel((val), REG_SMADDR)
#define nuc900_nand_read(reg)		__raw_readl(reg)

struct nuc900_nand {
	struct mtd_info mtd;
	struct nand_chip chip;
	void __iomem *reg;
	struct clk *clk;
	spinlock_t lock;
};
extern struct semaphore  fmi_sem;



#define NUM_PARTITIONS 4

/*请保证分区的总和数和nandflash的实际大小一致，目前开发板上的nandflash是128M*/
#define UBOOT_SIZE 	SZ_1M*1
#define KERNEL_SIZE 	SZ_1M*5
#define ROOT_SIZE		SZ_1M*44
#define USER_SIZE 	SZ_1M*78

/*vitual addr and phy addr for dma */
static unsigned char * nand_vaddr = NULL;
static unsigned char * nand_phyaddr = NULL;

static const struct mtd_partition partitions[] = {
	{ .name = "U-boot",
	  .offset = 0,
	  .size = UBOOT_SIZE
	 },
	{ .name = "linux 2.6.35 kernel",
	  .offset = UBOOT_SIZE,
	  .size = KERNEL_SIZE
	},
	{ .name = "root",
 	  .offset = UBOOT_SIZE+KERNEL_SIZE,
	  .size = ROOT_SIZE
	},
	{ .name = "user",
	  .offset = UBOOT_SIZE+KERNEL_SIZE+ROOT_SIZE,
	  .size = USER_SIZE
	}
};
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
static unsigned char nuc900_nand_read_byte(struct mtd_info *mtd)
{
	unsigned char ret;
	struct nuc900_nand *nand;

	nand = container_of(mtd, struct nuc900_nand, mtd);

	ret = (unsigned char)read_data_reg(nand);

	return ret;
}

static void nuc900_nand_read_buf(struct mtd_info *mtd,
				 unsigned char *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;
	//nuc900_nand_debug("nuc900_nand_read_buf in \n");
	
	if(len==mtd->oobsize){// read oob data
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize, -1);
	  if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	  {
		  printk("nuc900 mtd nand driver read buf sem error\n");
		  return;
	  }
	  for (i = 0; i < len; i++)
		   buf[i] = __raw_readl(REG_SMDATA)& 0xff;
		   
#ifdef DEBUG_NAND
	  nuc900_nand_debug("oob read\n");
	  for (i = 0; i < len; i++)
		   nuc900_nand_debug(" 0x%02x |",buf[i]);
	  nuc900_nand_debug("\n");	
#endif	  
	  
		goto readout1;
	} 
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
  {
		  printk("nuc900 mtd nand driver read buf sem error\n");
		  return;
	}
	
	//normal page read use dma
	while (__raw_readl(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe
	__raw_writel((unsigned long)nand_phyaddr,REG_DMACSAR2);
	__raw_writel(__raw_readl(REG_SMCSR) | 0x02,REG_SMCSR); //enable DMA read
	while(!(__raw_readl(REG_SMISR)&0x01));//wait for dma finished
	__raw_writel(__raw_readl(REG_SMISR)|0x01,REG_SMISR);  //clear DMA finished flag
	
	memcpy(buf,nand_vaddr,len);
	
readout1:
	up(&fmi_sem);	
}

static void nuc900_nand_write_buf(struct mtd_info *mtd,
				  const unsigned char *buf, int len)
{
	int i;
  struct nand_chip *chip = mtd->priv;
	int length = mtd->oobsize;
	int dmanum = 0;
  //nuc900_nand_debug("nuc900_nand_write_buf in \n");
	//nuc900_nand_debug("nuc900nand write buf:len=%d\n",len);

	if(len==length){//  write for oob 
		chip->cmdfunc(mtd, NAND_CMD_RNDIN, mtd->writesize, -1);
		if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	  {
		   printk("nuc900 mtd nand driver write buf sem error\n");
		   return;
	  }
		#ifdef DEBUG_NAND
		nuc900_nand_debug("oobdata:len=%d\n",len);
	  for (i = 0; i < len; i++)
		   nuc900_nand_debug(" 0x%02x |",buf[i]);
	  nuc900_nand_debug("\n");	
	  #endif
	  
		i=0;
		while(i<len){
			__raw_writel(buf[i],REG_SMDATA);
			i=i+1;
		}
		
    goto write_out;
	}
	
	  if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	  {
		   printk("nuc900 mtd nand driver write buf sem error\n");
		   return;
	  }
	  //normal page write use dma
		while(dmanum < len)//give the first 512 to the dma space
		{
			nand_vaddr[dmanum] = buf[dmanum];
			dmanum++;
 		}	
 			
 		while(dmanum < 2112)
 		{
 			 nand_vaddr[dmanum] = 0xff;
 			 dmanum++;
 		}
 		#if 0
 		nuc900_nand_debug("\n");
 	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_0));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_1));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_2));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_3));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_4));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_5));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_6));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_7));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_8));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_9));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_10));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_11));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_12));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_13));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_14));
	  nuc900_nand_debug(" 0x%02x |",__raw_readl(REG_SMRA_15));
 		nuc900_nand_debug("\n");
 		#endif 
		//normal page write use dma
	  while (__raw_readl(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe
	  __raw_writel((unsigned long)nand_phyaddr,REG_DMACSAR2);
		__raw_writel(__raw_readl(REG_SMCSR) | 0x04,REG_SMCSR); //enable DMA write
	  while(!(__raw_readl(REG_SMISR)&0x01));//wait for dma finished
	  __raw_writel(__raw_readl(REG_SMISR)|0x01,REG_SMISR);  //clear DMA finished flag
	 // __raw_writel(0x11223344,REG_SMRA_15);
write_out:
	 up(&fmi_sem);
	 return;	
}

/* select chip */
static void nuc900_nand_select_chip(struct mtd_info *mtd, int chipnr)
{
  struct nuc900_nand *nand;
	nand = container_of(mtd, struct nuc900_nand, mtd);
		
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("nuc900 mtd nand driver select_chip sem error\n");
		return ;
	}

	switch (chipnr) {
	case -1://no chip selected 
		__raw_writel(__raw_readl(REG_SMCSR) | NANDCS,
			       REG_SMCSR);
		break;
	case 0://select nand chip 0
		 __raw_writel(__raw_readl(REG_SMCSR) & ~NANDCS,
			       REG_SMCSR);
		break;
	case 1://select nand chip 1
		 __raw_writel((__raw_readl(REG_SMCSR) & (~NANDCS))| NANDCS1,
			       REG_SMCSR);
		break;
	default:
		BUG();
	}
  
  up(&fmi_sem);

}
static int nuc900_check_rb(struct nuc900_nand *nand)
{
	unsigned int val;
	spin_lock(&nand->lock);
	val = __raw_readl(REG_SMISR );
	val &= READYBUSY;
	spin_unlock(&nand->lock);

	return val;
}

static int nuc900_nand_devready(struct mtd_info *mtd)
{
	struct nuc900_nand *nand;
	int ready;

	nand = container_of(mtd, struct nuc900_nand, mtd);

	ready = (nuc900_check_rb(nand)) ? 1 : 0;
	return ready;
}

static void nuc900_nand_command_lp(struct mtd_info *mtd, unsigned int command,
				   int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	struct nuc900_nand *nand;

	nand = container_of(mtd, struct nuc900_nand, mtd);

	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	write_cmd_reg(nand, command & 0xff);

	if (column != -1 || page_addr != -1) {

		if (column != -1) {
			if (chip->options & NAND_BUSWIDTH_16)
				column >>= 1;
			write_addr_reg(nand, column);
			write_addr_reg(nand, column >> 8 | ENDADDR);
		}
		if (page_addr != -1) {
			write_addr_reg(nand, page_addr);

			if (chip->chipsize > (128 << 20)) {
				write_addr_reg(nand, page_addr >> 8);
				write_addr_reg(nand, page_addr >> 16 | ENDADDR);
			} else {
				write_addr_reg(nand, page_addr >> 8 | ENDADDR);
			}
		}
	}

	switch (command) {
	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		return;

	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(chip->chip_delay);
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);

		write_cmd_reg(nand, NAND_CMD_STATUS);
		write_cmd_reg(nand, command);

		while (!nuc900_check_rb(nand))
			;

		return;

	case NAND_CMD_RNDOUT:
		write_cmd_reg(nand, NAND_CMD_RNDOUTSTART);
		return;

	case NAND_CMD_READ0:

		write_cmd_reg(nand, NAND_CMD_READSTART);
	default:

		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	while (!chip->dev_ready(mtd))
		;
}

/*
 * Enable HW ECC : unused on most chips
 */
void nuc900_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	/*
	struct nand_chip *chip = mtd->priv;
	debug("nuc900_nand_enable_hwecc(%p, %d)\n", mtd, mode);
	
	*/
}
/*
 * Calculate HW ECC
 *
 * function called after a write
 *
 * mtd:        MTD block structure
 * dat:        raw data (unused)
 * ecc_code:   buffer for ECC
 */

static int nuc900_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				      u_char *ecc_code)
{
	struct nand_chip *chip = mtd->priv;
	unsigned int ecc_value;
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("nuc900 mtd nand driver nand_enable sem error\n");
		return 0;
	}
	//nuc900_nand_debug("nuc900_nand_calculate_ecc\n");
	/* get the first 3 ECC bytes */
	ecc_value = __raw_readl(REG_SMECC0);
	ecc_code[0] = ecc_value & 0xFF;
	ecc_code[1] = (ecc_value >> 8) & 0xFF;
	ecc_code[2] = (ecc_value >> 16) & 0xFF;
	
	/* get the second 3 ECC bytes */
	ecc_value = __raw_readl(REG_SMECC1);

	ecc_code[3] = ecc_value & 0xFF;
	ecc_code[4] = (ecc_value >> 8) & 0xFF;
	ecc_code[5] = (ecc_value >> 16) & 0xFF;
	if (chip->ecc.bytes==12)
	{
		/* get the third 3 ECC bytes */
	ecc_value = __raw_readl(REG_SMECC2);
	ecc_code[6] = ecc_value & 0xFF;
	ecc_code[7] = (ecc_value >> 8) & 0xFF;
	ecc_code[8] = (ecc_value >> 16) & 0xFF;
	
	/* get the fourth 3 ECC bytes */
	ecc_value = __raw_readl(REG_SMECC3);

	ecc_code[9] = ecc_value & 0xFF;
	ecc_code[10] = (ecc_value >> 8) & 0xFF;
	ecc_code[11] = (ecc_value >> 16) & 0xFF;	
	}
	up(&fmi_sem);
	#if 0
	nuc900_nand_debug("nuc900_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[0], ecc_code[1], ecc_code[2]);
	nuc900_nand_debug("nuc900_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[3], ecc_code[4], ecc_code[5]);
	nuc900_nand_debug("nuc900_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[6], ecc_code[7], ecc_code[8]);
	nuc900_nand_debug("nuc900_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[9], ecc_code[10], ecc_code[11]);
	#endif     	       	       
	return 0;
}

/*
 * HW ECC Correction
 *
 * function called after a read
 *
 * mtd:        MTD block structure
 * dat:        raw data read from the chip
 * read_ecc:   ECC from the chip (unused)
 * isnull:     unused
 *
 * Detect and correct a 1 bit error for a page
 */
static int nuc900_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	//struct nand_chip *nand_chip = mtd->priv;
	unsigned int ecc_status;
	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("nuc900 mtd nand driver nand_enable sem error\n");
		return 0;
	}
	
	#ifdef DEBUG_NAND
	//printf("nuc900_nand_correct_data\n");
	#endif
	/* get the status from the Status Register */
	ecc_status = __raw_readl(REG_SMISR);

	/* if there's no error */
	if (likely(!(ecc_status & 0x02))){
		up(&fmi_sem);	
		return 0;
	}
		
  __raw_writel(0x02,REG_SMISR);
  
  if (fmiSMCorrectData_2K((unsigned long)dat)==0){
  		printk(KERN_WARNING "nuvoton_nand : error corrected\n");
  		up(&fmi_sem);	
	    return 1;
   }
  /* 
	* We can't correct so many errors */
	printk(KERN_WARNING "nuvoton_nand : multiple errors detected."
				" Unable to correct.\n");
	up(&fmi_sem);			
	return -EIO;	
}



/* functions */
int fmiSMCheckRB(void)
{
        while (1) {
                if (__raw_readl(REG_SMISR) & 0x400) {
                        __raw_writel(0x400,REG_SMISR);
                        return 1;
                }
        }
        return 0;
}
// SM functions
int fmiSM_Reset(void)
{

        u32 volatile i;

        __raw_writel(0xff,REG_SMCMD);
        for (i=100; i>0; i--);

        if (!fmiSMCheckRB())
                return -1;
        return 0;
}

static void nuc900_nand_enable(struct nuc900_nand *nand)
{
		
	unsigned int val;

	if(down_interruptible(&fmi_sem)) //jhe+ 2010.12.21
	{
		printk("nuc900 mtd nand driver nand_enable sem error\n");
		return;
	}
	spin_lock(&nand->lock);
  __raw_writel(((__raw_readl(REG_MFSEL) & 0xFFFFFFF3) | 0x00000004),REG_MFSEL); /* select NAND function pins */
	// DMAC Initial
	__raw_writel( 0x00000003, REG_DMACCSR );
	__raw_writel( 0x00000001, REG_DMACCSR );
	__raw_writel( 0x00000001, REG_DMACIER );
	
	__raw_writel( 0x01, REG_FMICSR );		// reset FMI engine
	// enable SM
	val = __raw_readl(REG_FMICSR);
	if (!(val & NAND_EN))
		__raw_writel(val | NAND_EN,  REG_FMICSR);
		
	/* init SM interface */
	__raw_writel( 0x3050b, REG_SMTCR );//set timer control
	__raw_writel( (__raw_readl(REG_SMCSR)&0xf8ffffc0)|0x01000028 , REG_SMCSR);//wp=1, ecc4_en=1,psize=1/2048+64
	
	val = __raw_readl(REG_SMCSR);
	//nuc900_nand_debug("REG_SMCSR01:0x%02x\n",val);
	val &= ~( SWRST | DMARWEN | PSIZE | BUSWID | ECC4EN | NANDCS );
	val |= WP | ECC4EN ;
	val |= PSIZE ;
	
	//nuc900_nand_debug("REG_SMCSR02:0x%02x\n",val);
	__raw_writel(val, REG_SMCSR);
	fmiSM_Reset();
	
	spin_unlock(&nand->lock);
	up(&fmi_sem);
}
static int __devinit nuc900_nand_probe(struct platform_device *pdev)
{
	struct nuc900_nand *nuc900_nand;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	int retval;
	struct resource *res;
  nuc900_nand_debug("nuc900_nand_probe in\n");
  
  nand_vaddr = (unsigned char *) dma_alloc_coherent(NULL,2112, (dma_addr_t *) &nand_phyaddr, GFP_KERNEL);
  if(nand_vaddr == NULL){
  	printk(KERN_ERR "NUC900_nand: failed to allocate ram for nand data.\n");
		return -ENOMEM;
  } 
  
	retval = 0;
  /* Allocate memory for the device structure (and zero it) */
	nuc900_nand = kzalloc(sizeof(struct nuc900_nand), GFP_KERNEL);
	if (!nuc900_nand){
		printk(KERN_ERR "NUC900_nand: failed to allocate device structure.\n");
		return -ENOMEM;
	}
	mtd=&nuc900_nand->mtd;
	chip = &(nuc900_nand->chip);
	
	chip->priv = nuc900_nand;		/* link the private data structures */
	mtd->priv	= chip;
	mtd->owner	= THIS_MODULE;
	spin_lock_init(&nuc900_nand->lock);
	nuc900_nand->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(nuc900_nand->clk)) {
		retval = -ENOENT;
		goto fail1;
	}
	clk_enable(nuc900_nand->clk);//enable fmi clk
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_ERR "nuc900_nand: can't get I/O resource mem\n");
		retval = -ENXIO;
		goto fail1;
	}
	  //nuc900_nand_debug("res->start:0x%02x\n",res->start);
  /*
	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		retval = -EBUSY;
		goto fail1;
	}
	*/
	
	/*
	nuc900_nand->reg = ioremap(res->start, resource_size(res));
	nuc900_nand_debug("res->start:0x%02x\n",nuc900_nand->reg);
	if (!nuc900_nand->reg) {
		retval = -ENOMEM;
		goto fail2;
	}
	*/
	chip->cmdfunc		= nuc900_nand_command_lp;
	chip->dev_ready		= nuc900_nand_devready;
	chip->read_byte		= nuc900_nand_read_byte;
	chip->write_buf		= nuc900_nand_write_buf;
	chip->read_buf		= nuc900_nand_read_buf;
	//chip->verify_buf	= nuc900_verify_buf;
	chip->select_chip  = nuc900_nand_select_chip;
	chip->chip_delay	= 50;
	chip->options		= 0;
	//chip->ecc.mode		= NAND_ECC_SOFT;
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.calculate = nuc900_nand_calculate_ecc;
	chip->ecc.correct = nuc900_nand_correct_data;
	chip->ecc.hwctl = nuc900_nand_enable_hwecc;
	
  platform_set_drvdata(pdev, nuc900_nand);
  
	nuc900_nand_enable(nuc900_nand);
	

	//chip->ecc.read_page = nuc900_nand_read_page;
	//chip->ecc.read_page = nuc900_nand_read_page_hwecc;
	//chip->ecc.read_page_raw = nuc900_nand_read_page_raw;
	//chip->ecc.write_page = nuc900_nand_write_page_hwecc;
	//chip->ecc.write_page_raw = nuc900_nand_write_page_raw;
	
	/* first scan to find the device and get the page size */
	if (nand_scan_ident(mtd, 1, NULL)) {
		retval = -ENXIO;
		goto err_scan_ident;
	}
	
	
    //chip->ecc.bytes = CONFIG_SYS_NAND_ECCBYTES;
		/* ECC is calculated for the whole page (1 step) */
	  chip->ecc.size = mtd->writesize;
			/* set ECC page size */
		switch (mtd->writesize) {
		case 512:
			__raw_writel( __raw_readl(REG_SMCSR) & ( ~PSIZE ), REG_SMCSR );	// psize:512; wp# set 1
			chip->ecc.bytes =ECCBYTE512;
			break;
		case 2048:
			__raw_writel( __raw_readl(REG_SMCSR)|PSIZE , REG_SMCSR );	// psize:2048; wp# set 1
			chip->ecc.bytes =ECCBYTE2K;
			break;
		default:
			/* page size not handled by HW ECC */
			/* switching back to soft ECC */
			chip->ecc.mode = NAND_ECC_SOFT;
			chip->ecc.calculate = NULL;
			chip->ecc.correct = NULL;
			chip->ecc.hwctl = NULL;
			chip->ecc.read_page = NULL;
			chip->ecc.postpad = 0;
			chip->ecc.prepad = 0;
			chip->ecc.bytes = 0;
			break;
		}
	
	/* second phase scan */
	if (nand_scan_tail(mtd)) {
		
		dma_free_coherent(NULL, 2112, nand_vaddr, (dma_addr_t )nand_phyaddr);
		retval = -ENXIO;
		goto fail3;
	}

	add_mtd_partitions(mtd, partitions,
						ARRAY_SIZE(partitions));
	//platform_set_drvdata(pdev, nuc900_nand);
  //nuc900_nand_debug("nuc900_nand_probe 12\n");
	return retval;

fail3:	
	nuc900_nand_debug("nuc900_nand_probe fail3\n");
	//iounmap(nuc900_nand->reg);
err_scan_ident:	
	platform_set_drvdata(pdev, NULL);
fail2:
	//nuc900_nand_debug("nuc900_nand_probe fail2\n");	
	//release_mem_region(res->start, resource_size(res));
fail1:
	nuc900_nand_debug("nuc900_nand_probe fail1\n");
	kfree(nuc900_nand);
	return retval;
}

static int __devexit nuc900_nand_remove(struct platform_device *pdev)
{
	struct nuc900_nand *nuc900_nand = platform_get_drvdata(pdev);
	//struct resource *res;
	struct mtd_info *mtd = &nuc900_nand->mtd;

	nand_release(mtd);
	//iounmap(nuc900_nand->reg);
	//res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//release_mem_region(res->start, resource_size(res));

	clk_disable(nuc900_nand->clk);
	clk_put(nuc900_nand->clk);

	kfree(nuc900_nand);
	/* Free dma space */
	dma_free_coherent(NULL, 2112, nand_vaddr, (dma_addr_t )nand_phyaddr);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver nuc900_nand_driver = {
	.probe		= nuc900_nand_probe,
	.remove		= __devexit_p(nuc900_nand_remove),
	.driver		= {
		.name	= "nuc900-fmi",
		.owner	= THIS_MODULE,
	},
};

static int __init nuc900_nand_init(void)
{
	return platform_driver_register(&nuc900_nand_driver);
}

static void __exit nuc900_nand_exit(void)
{
	platform_driver_unregister(&nuc900_nand_driver);
}

module_init(nuc900_nand_init);
module_exit(nuc900_nand_exit);

MODULE_AUTHOR("Wan ZongShun <mcuos.com@gmail.com>");
MODULE_DESCRIPTION("w90p910/NUC9xx nand driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc900-fmi");
