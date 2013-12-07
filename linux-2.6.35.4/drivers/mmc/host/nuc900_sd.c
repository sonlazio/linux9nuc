/*
 *  linux/drivers/mmc/host/nuc900_sd.c - Nuvoton NUC900 SD Driver
 *
 *  Copyright (C) 2005 Cougar Creek Computing Devices Ltd, All Rights Reserved
 *
 *  Copyright (C) 2006 Malcolm Noyes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/atmel_pdc.h>
#include <linux/gfp.h>

#include <linux/mmc/host.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-fmi.h>
#include <mach/regs-gpio.h>
#include <mach/nuc900_sd.h>

#if defined (CONFIG_NUC900_SD_SD0) &&  defined (CONFIG_NUC900_SD_SD1)
#error CANNOT ENABLE SD PORT 0 AND PORT 1 AT THE SAME TIME!!
#endif

#define nuc900_sd_debug	printk

#define DRIVER_NAME "nuc900-fmi"

#define REG_MFSEL	(W90X900_VA_GCR + 0xC)

#define FL_SENT_COMMAND	(1 << 0)
#define FL_SENT_STOP	(1 << 1)

#define nuc900_sd_read(reg)		__raw_readl(reg)
#define nuc900_sd_write(reg, val)	__raw_writel((val), (reg))

#define MCI_BLKSIZE 		512
#define MCI_MAXBLKSIZE 		4095
#define MCI_BLKATONCE 		256
#define MCI_BUFSIZE 		(MCI_BLKSIZE * MCI_BLKATONCE)

/* Driver thread command */
#define SD_EVENT_NONE		0x00000000
#define SD_EVENT_CMD_OUT	0x00000001
#define SD_EVENT_RSP_IN		0x00000010
#define SD_EVENT_RSP2_IN	0x00000100
#define SD_EVENT_CLK_KEEP0	0x00001000
#define SD_EVENT_CLK_KEEP1	0x00010000

static volatile int sd_event=0, sd_state=0, sd_state_xfer=0, sd_ri_timeout=0, sd_send_cmd=0;
static DECLARE_WAIT_QUEUE_HEAD(sd_event_wq);
static DECLARE_WAIT_QUEUE_HEAD(sd_wq);
static DECLARE_WAIT_QUEUE_HEAD(sd_wq_xfer);

extern struct semaphore fmi_sem;
extern struct semaphore dmac_sem;

/*
 * Low level type for this driver
 */
struct nuc900_sd_host {
        struct mmc_host *mmc;
        struct mmc_command *cmd;
        struct mmc_request *request;

        void __iomem *sd_base;
        int irq;

        int present;

        struct clk *fmi_clk, *sd_clk, *dmac_clk;

        /*
         * Flag indicating when the command has been sent. This is used to
         * work out whether or not to send the stop
         */
        unsigned int flags;
        /* flag for current port */
        u32 bus_mode;
        u32 port;

        /* DMA buffer used for transmitting */
        unsigned int* buffer;
        dma_addr_t physical_address;
        unsigned int total_length;

        /* Latest in the scatterlist that has been enabled for transfer, but not freed */
        int in_use_index;

        /* Latest in the scatterlist that has been enabled for transfer */
        int transfer_index;

        /* Timer for timeouts */
        struct timer_list timer;
};

struct nuc900_sd_host *sd_host;

/*
 * Reset the controller and restore most of the state
 */
static void nuc900_sd_reset_host(struct nuc900_sd_host *host)
{
        unsigned long flags;

        local_irq_save(flags);

        nuc900_sd_write(REG_DMACCSR, DMACCSR_DMACEN | DMACCSR_SW_RST); //enable DMAC for FMI
        nuc900_sd_write(REG_FMICSR, FMICSR_SW_RST);	/* Enable SD functionality of FMI */
        nuc900_sd_write(REG_FMICSR, FMICSR_SD_EN);
        local_irq_restore(flags);
}

static void nuc900_sd_timeout_timer(unsigned long data)
{
        struct nuc900_sd_host *host;

        host = (struct nuc900_sd_host *)data;

        if (host->request) {
                dev_err(host->mmc->parent, "Timeout waiting end of packet\n");

                if (host->cmd && host->cmd->data) {
                        host->cmd->data->error = -ETIMEDOUT;
                } else {
                        if (host->cmd)
                                host->cmd->error = -ETIMEDOUT;
                        else
                                host->request->cmd->error = -ETIMEDOUT;
                }

                nuc900_sd_reset_host(host);
                mmc_request_done(host->mmc, host->request);
        }
}

/*
 * Copy from sg to a dma block - used for transfers
 */
static inline void nuc900_sd_sg_to_dma(struct nuc900_sd_host *host, struct mmc_data *data)
{
        unsigned int len, i, size;
        unsigned *dmabuf = host->buffer;

        size = data->blksz * data->blocks;
        len = data->sg_len;

        /*
         * Just loop through all entries. Size might not
         * be the entire list though so make sure that
         * we do not transfer too much.
         */
        for (i = 0; i < len; i++) {
                struct scatterlist *sg;
                int amount;
                unsigned int *sgbuffer;

                sg = &data->sg[i];

                sgbuffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
                amount = min(size, sg->length);
                size -= amount;

                {
                        char *tmpv = (char *)dmabuf;
                        memcpy(tmpv, sgbuffer, amount);
                        tmpv += amount;
                        dmabuf = (unsigned *)tmpv;
                }

                kunmap_atomic(sgbuffer, KM_BIO_SRC_IRQ);
                data->bytes_xfered += amount;

                if (size == 0)
                        break;
        }

        /*
         * Check that we didn't get a request to transfer
         * more data than can fit into the SG list.
         */
        BUG_ON(size != 0);
}

/*
 * Handle after a dma read
 */
static void nuc900_sd_post_dma_read(struct nuc900_sd_host *host)
{
        struct mmc_command *cmd;
        struct mmc_data *data;
        unsigned int len, i, size;
        unsigned *dmabuf = host->buffer;


        cmd = host->cmd;
        if (!cmd) {
                nuc900_sd_debug("no command\n");
                return;
        }

        data = cmd->data;
        if (!data) {
                nuc900_sd_debug("no data\n");
                return;
        }

        size = data->blksz * data->blocks;
        len = data->sg_len;

        for (i = 0; i < len; i++) {
                struct scatterlist *sg;
                int amount;
                unsigned int *sgbuffer;

                sg = &data->sg[i];

                sgbuffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
                amount = min(size, sg->length);
                size -= amount;

                {
                        char *tmpv = (char *)dmabuf;
                        memcpy(sgbuffer, tmpv, amount);
                        tmpv += amount;
                        dmabuf = (unsigned *)tmpv;
                }

                flush_kernel_dcache_page(sg_page(sg));
                kunmap_atomic(sgbuffer, KM_BIO_SRC_IRQ);
                data->bytes_xfered += amount;
                if (size == 0)
                        break;
        }
}

/*
 * Handle transmitted data
 */
static void nuc900_sd_handle_transmitted(struct nuc900_sd_host *host)
{
        //nuc900_sd_debug("Handling the transmit\n");

        if (nuc900_sd_read(REG_SDISR) & SDISR_CRC_IF)
                nuc900_sd_write(REG_SDISR, SDISR_CRC_IF);

        /* check read/busy */
        if (host->port == 0)
		nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) | SDCSR_CLK_KEEP0);
        else
                nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) |SDCSR_CLK_KEEP1);
}


/*
 * Update bytes tranfered count during a write operation
 */
static void nuc900_sd_update_bytes_xfered(struct nuc900_sd_host *host)
{
        struct mmc_data *data;

        /* always deal with the effective request (and not the current cmd) */

        if (host->request->cmd && host->request->cmd->error != 0)
                return;

        if (host->request->data) {
                data = host->request->data;
                if (data->flags & MMC_DATA_WRITE) {
                        /* card is in IDLE mode now */
                        data->bytes_xfered = data->blksz * data->blocks;
                        //nuc900_sd_debug("-> bytes_xfered %d, total_length = %d\n",
                        //	data->bytes_xfered, host->total_length);
                }
        }
}


/*
 * Enable the controller
 */
static void nuc900_sd_enable(struct nuc900_sd_host *host)
{
#ifdef CONFIG_NUC900_SD_SD0
        //SD 0 GPIO select
        nuc900_sd_write(REG_MFSEL, (nuc900_sd_read(REG_MFSEL) & 0xFFFFFF0F)|0xa0); //GPSELD[5:4]=10 ; [7:6]=10
        nuc900_sd_write(REG_GPIOD_DIR, 0x100);	// set gpiod-8 to output
        nuc900_sd_write(REG_GPIOD_DATAOUT, nuc900_sd_read(REG_GPIOD_DATAOUT) | 0x100);	// set gpiod-8 output high
        udelay(1000);
#endif
#ifdef CONFIG_NUC900_SD_SD1
        //SD 1 GPIO select
        nuc900_sd_write(REG_MFSEL, (nuc900_sd_read(REG_MFSEL)&0xffe3ffff)|0x80001); //GPSELD[19:18]=10 ; [20]=0 ;[0]=1
        nuc900_sd_write(REG_GPIOG_DIR, 0x20);	// set gpiog-5 to output
        nuc900_sd_write(REG_GPIOG_DATAOUT, nuc900_sd_read(REG_GPIOG_DATAOUT) | 0x20);	// set gpiog-5 output high
        udelay(1000);
#endif
        nuc900_sd_write(REG_DMACCSR, DMACCSR_DMACEN);	// enable DMAC for FMI
        nuc900_sd_write(REG_FMICSR, FMICSR_SD_EN);	/* Enable SD functionality of FMI */
        if (host->port == 0) {
                nuc900_sd_write(REG_SDIER, nuc900_sd_read(REG_SDIER) | SDIER_CD0SRC);	// select GPIO detect
                nuc900_sd_write(REG_SDCSR, (nuc900_sd_read(REG_SDCSR) & 0x9fffffff)); //SD Port 0 is selected
        } else {
                nuc900_sd_write(REG_SDIER, nuc900_sd_read(REG_SDIER) | SDIER_CD1SRC);	// select GPIO detect
                nuc900_sd_write(REG_SDCSR, (nuc900_sd_read(REG_SDCSR) & 0x9fffffff)|0x40000000); //SD Port 1 is selected
        }
}

/*
 * Disable the controller
 */
static void nuc900_sd_disable(struct nuc900_sd_host *host)
{
        nuc900_sd_write(REG_DMACCSR, DMACCSR_DMACEN | DMACCSR_SW_RST); //enable DMAC for FMI
        nuc900_sd_write(REG_FMICSR, FMICSR_SW_RST);	/* Enable SD functionality of FMI */
}

/*
 * Send a command
 */
static void nuc900_sd_send_command(struct nuc900_sd_host *host, struct mmc_command *cmd)
{
        unsigned int csr;
        unsigned int block_length;
        struct mmc_data *data = cmd->data;

        unsigned int blocks;

        host->cmd = cmd;
        sd_host = host;
        sd_state = 0;
        sd_state_xfer = 0;

        if (!(host->flags & FL_SENT_STOP)) {
                if (down_interruptible(&fmi_sem))
                        return;
        }

        if(nuc900_sd_read(REG_FMICSR) != FMICSR_SD_EN)
	        nuc900_sd_write(REG_FMICSR, FMICSR_SD_EN);

        if (host->port == 0)
                csr = ((nuc900_sd_read(REG_SDCSR) | 0x01010000) & 0xff00c080);
        else
                csr = ((nuc900_sd_read(REG_SDCSR) | 0x41010000) & 0xff00c080);

        csr = csr | (cmd->opcode << 8) | SDCSR_CO_EN;	// set command code and enable command out
        sd_event |= SD_EVENT_CMD_OUT;

        if (host->bus_mode == MMC_BUS_WIDTH_4)
                csr |= SDCSR_DBW;

        if (mmc_resp_type(cmd) != MMC_RSP_NONE) {
                /* if a response is expected then allow maximum response latancy */

                /* set 136 bit response for R2, 48 bit response otherwise */
                if (mmc_resp_type(cmd) == MMC_RSP_R2) {
                        csr |= SDCSR_R2_EN;
                        sd_event |= SD_EVENT_RSP2_IN;
                } else {
                        csr |= SDCSR_RI_EN;
                        sd_event |= SD_EVENT_RSP_IN;
                }
                nuc900_sd_write(REG_SDISR, SDISR_RITO_IF);
                sd_ri_timeout = 0;
                nuc900_sd_write(REG_SDTMOUT, 0xffff);
        }

        if (data) {
                nuc900_sd_write(REG_SDIER, nuc900_sd_read(REG_SDIER) | SDIER_BLKD_IE);	//Enable SD interrupt & select GPIO detect
                block_length = data->blksz;
                blocks = data->blocks;

                nuc900_sd_write(REG_SDBLEN, block_length-1);
                if (block_length == 0x200) {
                        if (blocks < 256)
                                csr = csr | (blocks << 16);
                        else
                                csr = csr & 0xff00ffff;
                }
        } else {
                block_length = 0;
                blocks = 0;
        }

        /*
         * Set the arguments and send the command
         */
        //nuc900_sd_debug("Sending command %d as 0x%0X, arg = 0x%08X, blocks = %d, length = %d\n",
        //	cmd->opcode, csr, cmd->arg, blocks, block_length);

        if (data) {

                data->bytes_xfered = 0;
                host->transfer_index = 0;
                host->in_use_index = 0;
                if (data->flags & MMC_DATA_READ) {
                        /*
                         * Handle a read
                         */
                        host->total_length = 0;
                        nuc900_sd_write(REG_DMACSAR2, host->physical_address);

                } else if (data->flags & MMC_DATA_WRITE) {
                        /*
                         * Handle a write
                         */
                        if (down_interruptible(&dmac_sem))
                                return;
                        host->total_length = block_length * blocks;
                        nuc900_sd_sg_to_dma(host, data);
                        //nuc900_sd_debug("Transmitting %d bytes\n", host->total_length);
                        nuc900_sd_write(REG_DMACSAR2, host->physical_address);
                        csr = csr | SDCSR_DO_EN;
                }
        }

        /*
         * Send the command and then enable the PDC - not the other way round as
         * the data sheet says
         */

        nuc900_sd_write(REG_SDARG, cmd->arg);
        nuc900_sd_write(REG_SDCSR, csr);
        sd_send_cmd = 1;
        wake_up_interruptible(&sd_event_wq);

        if (!(host->flags & FL_SENT_STOP)) {
                wait_event_interruptible(sd_wq, (sd_state != 0));
                up(&fmi_sem);
        }

        if (data) {
                if (data->flags & MMC_DATA_WRITE) {
                        while (1) {
                                if (nuc900_sd_read(REG_SDISR) & SDISR_SDDAT0) {
                                        if (host->port == 0)
                                                nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) & ~SDCSR_CLK_KEEP0);
                                        else
                                                nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) & ~SDCSR_CLK_KEEP1);

                                        up(&dmac_sem);
                                        nuc900_sd_update_bytes_xfered(host);
                                        break;
                                }
                        }
                }
        }

        mmc_request_done(host->mmc, host->request);
}

/*
 * Process the request
 */
static void nuc900_sd_send_request(struct nuc900_sd_host *host)
{

        if (!(host->flags & FL_SENT_COMMAND)) {
                host->flags |= FL_SENT_COMMAND;
                nuc900_sd_send_command(host, host->request->cmd);
        } else if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
                host->flags |= FL_SENT_STOP;
                nuc900_sd_send_command(host, host->request->stop);
        } else {
                sd_state = 1;
                wake_up_interruptible(&sd_wq);
                del_timer(&host->timer);
        }
}

/*
 * Handle a command that has been completed
 */
static void nuc900_sd_completed_command(struct nuc900_sd_host *host, unsigned int status)
{
        struct mmc_command *cmd = host->cmd;
        struct mmc_data *data = cmd->data;
        unsigned int i, j, tmp[5], err;
        unsigned char *ptr;

        err = nuc900_sd_read(REG_SDISR);

        if ((err & SDISR_RITO_IF) || (cmd->error)) {
                nuc900_sd_write(REG_SDTMOUT, 0x0);
                nuc900_sd_write(REG_SDISR, SDISR_RITO_IF);
                cmd->error = -ETIMEDOUT;
                cmd->resp[0] = cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
        } else {
                if (status & SD_EVENT_RSP_IN) {
                        // if not R2
                        cmd->resp[0] = (nuc900_sd_read(REG_SDRSP0) << 8)|(nuc900_sd_read(REG_SDRSP1) & 0xff);
                        cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
                } else if (status & SD_EVENT_RSP2_IN) {
                        // if R2
                        ptr = (unsigned char *)FB0_BASE_ADDR;
                        for (i=0, j=0; j<5; i+=4, j++)
                                tmp[j] = (*(ptr+i)<<24)|(*(ptr+i+1)<<16)|(*(ptr+i+2)<<8)|(*(ptr+i+3));
                        for (i=0; i<4; i++)
                                cmd->resp[i] = ((tmp[i] & 0x00ffffff)<<8)|((tmp[i+1] & 0xff000000)>>24);
                }
        }
        //nuc900_sd_debug("Event = 0x%0X [0x%08X 0x%08X] <0x%x>\n", status, cmd->resp[0], cmd->resp[1], err);

        if (!cmd->error) {
                if ((err & SDISR_CRC_7) == 0) {
                        if (!(mmc_resp_type(cmd) & MMC_RSP_CRC)) {
                                cmd->error = 0;
                                nuc900_sd_write(REG_SDISR, SDISR_CRC_IF);
                        } else {
                                cmd->error = -EIO;
                                nuc900_sd_debug("Error detected and set to %d/%d (cmd = %d, retries = %d)\n",
                                                cmd->error, data ? data->error : 0,
                                                cmd->opcode, cmd->retries);
                        }
                } else
                        cmd->error = 0;

                if (data) {
                        data->bytes_xfered = 0;
                        host->transfer_index = 0;
                        host->in_use_index = 0;
                        if (data->flags & MMC_DATA_READ) {
                                if (down_interruptible(&dmac_sem))
                                        return;
                                nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) | SDCSR_DI_EN);
                        }
                        wait_event_interruptible(sd_wq_xfer, (sd_state_xfer != 0));
                        up(&dmac_sem);
                }
        }
        nuc900_sd_send_request(host);
}

/*
 * Handle an MMC request
 */
static int nuc900_sd_card_detect(struct mmc_host *mmc)
{
        struct nuc900_sd_host *host = mmc_priv(mmc);
        int ret;

        if (down_interruptible(&fmi_sem))
                return -1;
                
        if(nuc900_sd_read(REG_FMICSR) != FMICSR_SD_EN)        
           nuc900_sd_write(REG_FMICSR, FMICSR_SD_EN);        
        if (host->port == 0)
                // SD port 0
                host->present = nuc900_sd_read(REG_SDISR) & SDISR_CDPS0;
        else
                // SD port 1
                host->present = nuc900_sd_read(REG_SDISR) & SDISR_CDPS1;

        ret = host->present ? 0 : 1;
        up(&fmi_sem);	
        return ret;
}

static void nuc900_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
        struct nuc900_sd_host *host = mmc_priv(mmc);
        host->request = mrq;
        host->flags = 0;

        /* more than 1s timeout needed with slow SD cards */
        //mod_timer(&host->timer, jiffies +  msecs_to_jiffies(2000));

        if (nuc900_sd_card_detect(mmc) == 0) {
                nuc900_sd_debug("no medium present\n");
                host->request->cmd->error = -ENOMEDIUM;
                mmc_request_done(host->mmc, host->request);
        } else
                nuc900_sd_send_request(host);
}

/*
 * Set the IOS
 */
extern unsigned long get_cpu_clk(void);
static void nuc900_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
        int clkdiv;
        struct nuc900_sd_host *host = mmc_priv(mmc);
        unsigned long nuc900_sd_master_clock = clk_get_rate(host->fmi_clk);

        host->bus_mode = ios->bus_width;
        /* for test */
        nuc900_sd_master_clock = get_cpu_clk() * 1000;

        /* maybe switch power to the card */
        switch (ios->power_mode) {
        case MMC_POWER_OFF:
                if (host->port == 0)
                        // SD port 0
                        nuc900_sd_write(REG_GPIOD_DATAOUT, nuc900_sd_read(REG_GPIOD_DATAOUT) | 0x100);	// set gpiod-8 output high
                else
                        // SD port 1
                        nuc900_sd_write(REG_GPIOG_DATAOUT, nuc900_sd_read(REG_GPIOG_DATAOUT) | 0x20);	// set gpiog-5 output high
                break;
        case MMC_POWER_UP:
                if (host->port == 0)
                        // SD port 0
                        nuc900_sd_write(REG_GPIOD_DATAOUT, nuc900_sd_read(REG_GPIOD_DATAOUT) & 0xfffffeff);	// set gpiod-8 output low
                else
                        // SD port 1
                        nuc900_sd_write(REG_GPIOG_DATAOUT, nuc900_sd_read(REG_GPIOG_DATAOUT) & 0xffffffdf);	// set gpiog-5 output low
                nuc900_sd_debug("SD power ON\n");
                nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) | SDCSR_CLK74_OE);
                while (nuc900_sd_read(REG_SDCSR) & SDCSR_CLK74_OE);
                //nuc900_sd_debug("power on: sdcsr 0x%x\n", nuc900_sd_read(REG_SDCSR));
                break;
        case MMC_POWER_ON: // enable 74 clocks
                break;
        default:
                WARN_ON(1);
        }

        if (ios->clock == 0) {
                /* Disable the FMI controller */
                nuc900_sd_write(REG_FMICSR, 0);
                clkdiv = 0;
        } else {
                /* Enable the FMI controller */
                nuc900_sd_write(REG_FMICSR, 0x2);

                if ((nuc900_sd_master_clock % ios->clock) == 0)
                        clkdiv = (nuc900_sd_master_clock / ios->clock) - 1;
                else
                        clkdiv = nuc900_sd_master_clock / ios->clock;

                //nuc900_sd_debug("sd_master = %d, clock = %d, clkdiv = %d\n", (int)nuc900_sd_master_clock, (int)ios->clock, (int)clkdiv);
        }
        if (ios->bus_width == MMC_BUS_WIDTH_4) {
                nuc900_sd_debug("MMC: Setting controller bus width to 4\n");
                nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) | SDCSR_DBW);
        } else {
                nuc900_sd_debug("MMC: Setting controller bus width to 1\n");
                nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) & ~SDCSR_DBW);
        }

        /* Set the clock divider */
        nuc900_sd_write(REG_CLKDIV1, (nuc900_sd_read(REG_CLKDIV1) & 0x00ff) | (clkdiv << 8));
}


/*
 * Handle CO, RI, and R2 event
 */
static int sd_event_thread(void *unused)
{
        int event = 0;
        int completed = 0;

        daemonize("sdioeventd");

        for (;;) {
                wait_event_interruptible(sd_event_wq, (sd_event != SD_EVENT_NONE) && (sd_send_cmd));

                completed = 0;
                event = sd_event;
                sd_event = SD_EVENT_NONE;
                sd_send_cmd = 0;
                if (event & SD_EVENT_CMD_OUT) {
                        while (1) {
                                if (!(nuc900_sd_read(REG_SDCSR) & SDCSR_CO_EN)) {
                                        completed = 1;
                                        break;
                                }
                        }
                }

                if (event & SD_EVENT_RSP_IN) {
                        while (1) {
                                if (!(nuc900_sd_read(REG_SDCSR) & SDCSR_RI_EN)) {
                                        completed = 1;
                                        break;
                                }

                                if (nuc900_sd_read(REG_SDISR) & SDISR_RITO_IF) {
                                        nuc900_sd_write(REG_SDTMOUT, 0x0);
                                        nuc900_sd_write(REG_SDISR, SDISR_RITO_IF);

                                        completed = 1;
                                        sd_host->cmd->error = -ETIMEDOUT;
                                        break;
                                }
                        }
                }

                if (event & SD_EVENT_RSP2_IN) {
                        while (1) {
                                if (!(nuc900_sd_read(REG_SDCSR) & SDCSR_R2_EN)) {
                                        completed = 1;
                                        break;
                                }
                        }
                }

                if (completed) {
                        //nuc900_sd_debug("Completed command\n");
                        nuc900_sd_completed_command(sd_host, event);
                }
        }
        nuc900_sd_debug("event quit\n");
        return 0;
}

/*
 * Handle an interrupt
 */
static irqreturn_t nuc900_sd_irq(int irq, void *devid)
{
        struct nuc900_sd_host *host = devid;
        unsigned int int_status, present;

        int_status = nuc900_sd_read(REG_SDISR);

        //nuc900_sd_debug("FMI irq: status = %08X\n", int_status);

        if (int_status & SDISR_BLKD_IF) {
                //nuc900_sd_debug("Block transfer has ended\n");
                if (host->cmd->data->flags & MMC_DATA_WRITE) {
                        nuc900_sd_handle_transmitted(host);
                } else if (host->cmd->data->flags & MMC_DATA_READ) {
                        nuc900_sd_post_dma_read(host);
                }
                nuc900_sd_write(REG_SDISR, SDISR_BLKD_IF);
                sd_state_xfer = 1;
                wake_up_interruptible(&sd_wq_xfer);
        }

        /*
         * we expect this irq on both insert and remove,
         * and use a short delay to debounce.
         */

#ifdef CONFIG_NUC900_SD_SD0
        /* SD card port 0 detect */
        if (int_status & SDISR_CD0_IF) {
                present = int_status & SDISR_CDPS0;
                host->present = present;
                nuc900_sd_debug("%s: card %s\n", mmc_hostname(host->mmc),
                                present ? "remove" : "insert");
                if (!present) {
                        nuc900_sd_debug("****** Resetting SD-card bus width ******\n");
                        nuc900_sd_write(REG_GPIOD_DATAOUT, nuc900_sd_read(REG_GPIOD_DATAOUT) & 0xfffffeff);	// set gpiod-8 output low
                        nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) & ~SDCSR_DBW);
                }
                /* 0.5s needed because of early card detect switch firing */
                mmc_detect_change(host->mmc, msecs_to_jiffies(500));
                nuc900_sd_write(REG_SDISR, SDISR_CD0_IF);
        }
#endif
#ifdef CONFIG_NUC900_SD_SD1
        /* SD card port 1 detect */
        if (int_status & SDISR_CD1_IF) {
                present = int_status & SDISR_CDPS1;
                host->present = present;
                nuc900_sd_debug("%s: card %s\n", mmc_hostname(host->mmc),
                                present ? "remove" : "insert");
                if (!present) {
                        nuc900_sd_debug("****** Resetting SD-card bus width ******\n");
                        nuc900_sd_write(REG_GPIOG_DATAOUT, nuc900_sd_read(REG_GPIOG_DATAOUT) & 0xffffffdf);	// set gpiog-5 output low
                        nuc900_sd_write(REG_SDCSR, nuc900_sd_read(REG_SDCSR) & ~SDCSR_DBW);
                }
                /* 0.5s needed because of early card detect switch firing */
                mmc_detect_change(host->mmc, msecs_to_jiffies(500));
                nuc900_sd_write(REG_SDISR, SDISR_CD1_IF);
        }
#endif

        return IRQ_HANDLED;
}

static int nuc900_sd_get_ro(struct mmc_host *mmc)
{
//chp	struct nuc900_sd_host *host = mmc_priv(mmc);

        /* TODO: check write protect pin */
        /* if write protect, it should return >0 value */

        /* no write protect */
        return 0;

        /*
         * Board doesn't support read only detection; let the mmc core
         * decide what to do.
         */
        //return -ENOSYS;
}

static void nuc900_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
//chp	struct nuc900_sd_host *host = mmc_priv(mmc);

        nuc900_sd_write(REG_DMACIER, DMACIER_TABORT_IE);	//Enable target abort interrupt generation during DMA transfer
        nuc900_sd_write(REG_FMIIER, FMIIER_DTA_IE);	//Enable DMAC READ/WRITE target abort interrupt generation
}

static const struct mmc_host_ops nuc900_sd_ops = {
        .request	= nuc900_sd_request,
        .set_ios	= nuc900_sd_set_ios,
        .get_ro		= nuc900_sd_get_ro,
        .get_cd		= nuc900_sd_card_detect,
        .enable_sdio_irq = nuc900_sd_enable_sdio_irq,
};

/*
 * Probe for the device
 */
static int __init nuc900_sd_probe(struct platform_device *pdev)
{
        struct mmc_host *mmc;
        struct nuc900_sd_host *host;
        struct resource *res;
        int ret;

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res)
                return -ENXIO;

        if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
                return -EBUSY;

        mmc = mmc_alloc_host(sizeof(struct nuc900_sd_host), &pdev->dev);
        if (!mmc) {
                ret = -ENOMEM;
                dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
                goto fail6;
        }

        mmc->ops = &nuc900_sd_ops;
        mmc->f_min = 400000;
        mmc->f_max = 50000000;
        mmc->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
        mmc->caps = 0;

        mmc->max_blk_size  = MCI_MAXBLKSIZE;
        mmc->max_blk_count = MCI_BLKATONCE;
        mmc->max_req_size  = MCI_BUFSIZE;
        mmc->max_phys_segs = MCI_BLKATONCE;
        mmc->max_hw_segs   = MCI_BLKATONCE;
        mmc->max_seg_size  = MCI_BUFSIZE;

        host = mmc_priv(mmc);
        sd_host = host;
        host->mmc = mmc;
        host->bus_mode = 0;
#ifdef CONFIG_NUC900_SD_SD0
        host->port = 0;
#endif
#ifdef CONFIG_NUC900_SD_SD1
        host->port = 1;
#endif
        mmc->caps |= MMC_CAP_4_BIT_DATA;

        host->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE,
                                          &host->physical_address, GFP_KERNEL);
        if (!host->buffer) {
                ret = -ENOMEM;
                dev_err(&pdev->dev, "Can't allocate transmit buffer\n");
                goto fail5;
        }

        /*
         * Get Clock
         */
        host->dmac_clk = clk_get(&pdev->dev, NULL);
        if (IS_ERR(host->dmac_clk)) {
                ret = -ENODEV;
                dev_dbg(&pdev->dev, "no dmac_clk?\n");
                goto fail2;
        }

        host->fmi_clk = clk_get(&pdev->dev, NULL);
        if (IS_ERR(host->fmi_clk)) {
                ret = -ENODEV;
                dev_dbg(&pdev->dev, "no fmi_clk?\n");
                goto fail2;
        }

        host->sd_clk = clk_get(&pdev->dev, "SD");
        if (IS_ERR(host->sd_clk)) {
                ret = -ENODEV;
                dev_dbg(&pdev->dev, "no sd_clk?\n");
                goto fail2;
        }

        /*
         * Reset hardware
         */
        clk_enable(host->dmac_clk);		/* Enable the peripheral clock */
        clk_enable(host->fmi_clk);		/* Enable the peripheral clock */
        clk_enable(host->sd_clk);		/* Enable the peripheral clock */
        nuc900_sd_disable(host);
        nuc900_sd_enable(host);

        /*
         * Allocate the MCI interrupt
         */
        host->irq = platform_get_irq(pdev, 0);
        ret = request_irq(host->irq, nuc900_sd_irq, IRQF_SHARED,
                          mmc_hostname(mmc), host);
        if (ret) {
                dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
                goto fail0;
        }

        /* add a thread to check CO, RI, and R2 */
        kernel_thread(sd_event_thread, NULL, 0);

        setup_timer(&host->timer, nuc900_sd_timeout_timer, (unsigned long)host);

        platform_set_drvdata(pdev, mmc);

        /*
         * Add host to MMC layer
         */
        if (host->port == 0) {
                // SD port 0
                host->present = nuc900_sd_read(REG_SDISR) & SDISR_CDPS0;
                nuc900_sd_write(REG_SDIER, SDIER_CD0_IE | SDIER_CD0SRC);	//Enable SD interrupt & select GPIO detect
        } else {
                // SD port 1
                host->present = nuc900_sd_read(REG_SDISR) & SDISR_CDPS1;
                nuc900_sd_write(REG_SDIER, SDIER_CD1_IE | SDIER_CD1SRC);	//Enable SD interrupt & select GPIO detect
        }

        mmc_add_host(mmc);

        nuc900_sd_debug("Added NUC900 SD driver\n");

        return 0;

fail0:
        clk_disable(host->dmac_clk);
        clk_put(host->dmac_clk);
        clk_disable(host->fmi_clk);
        clk_put(host->fmi_clk);
        clk_disable(host->sd_clk);
        clk_put(host->sd_clk);
fail2:
        if (host->buffer)
                dma_free_coherent(&pdev->dev, MCI_BUFSIZE,
                                  host->buffer, host->physical_address);
fail5:
        mmc_free_host(mmc);
fail6:
        release_mem_region(res->start, res->end - res->start + 1);
        dev_err(&pdev->dev, "probe failed, err %d\n", ret);
        return ret;
}

/*
 * Remove a device
 */
static int __exit nuc900_sd_remove(struct platform_device *pdev)
{
        struct mmc_host *mmc = platform_get_drvdata(pdev);
        struct nuc900_sd_host *host;

        if (!mmc)
                return -1;

        host = mmc_priv(mmc);

        if (host->buffer)
                dma_free_coherent(&pdev->dev, MCI_BUFSIZE,
                                  host->buffer, host->physical_address);

        nuc900_sd_disable(host);
        del_timer_sync(&host->timer);
        mmc_remove_host(mmc);
        free_irq(host->irq, host);

        clk_disable(host->fmi_clk);			/* Disable the peripheral clock */
        clk_put(host->fmi_clk);
        clk_disable(host->sd_clk);			/* Disable the peripheral clock */
        clk_put(host->sd_clk);

        mmc_free_host(mmc);
        platform_set_drvdata(pdev, NULL);
        nuc900_sd_debug("NUC900 SD Removed\n");

        return 0;
}

#ifdef CONFIG_PM
static int nuc900_sd_suspend(struct platform_device *pdev, pm_message_t state)
{
        struct mmc_host *mmc = platform_get_drvdata(pdev);
        struct nuc900_sd_host *host = mmc_priv(mmc);
        int ret = 0;

        if (mmc)
                ret = mmc_suspend_host(mmc);

        return ret;
}

static int nuc900_sd_resume(struct platform_device *pdev)
{
        struct mmc_host *mmc = platform_get_drvdata(pdev);
        struct nuc900_sd_host *host = mmc_priv(mmc);
        int ret = 0;

        if (mmc)
                ret = mmc_resume_host(mmc);

        return ret;
}
#else
#define nuc900_sd_suspend	NULL
#define nuc900_sd_resume	NULL
#endif

static struct platform_driver nuc900_sd_driver = {
        .remove		= __exit_p(nuc900_sd_remove),
        .suspend	= nuc900_sd_suspend,
        .resume		= nuc900_sd_resume,
        .driver		= {
                .name	= DRIVER_NAME,
                .owner	= THIS_MODULE,
        },
};

static int __init nuc900_sd_init(void)
{
        return platform_driver_probe(&nuc900_sd_driver, nuc900_sd_probe);
}

static void __exit nuc900_sd_exit(void)
{
        platform_driver_unregister(&nuc900_sd_driver);
}

module_init(nuc900_sd_init);
module_exit(nuc900_sd_exit);

MODULE_DESCRIPTION("NUC900 SD Card Interface driver");
MODULE_AUTHOR("HPChen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc900_sd");
