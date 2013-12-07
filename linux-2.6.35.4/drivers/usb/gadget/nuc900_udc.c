/*
 * linux/drivers/usb/gadget/nuc900_udc.c
 *
 * Nuvoton NUC900 MCU on-chip full speed USB device controllers
 *
 * Copyright (C) 2010 Nuvoton Technology Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/system.h>

#include <mach/nuc900_usbd.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include "nuc900_udc.h"

#define DRIVER_DESC     "NUVOTON USB Device Controller Gadget"
#define DRIVER_VERSION  "16 July 2009"
#define DRIVER_AUTHOR	"shirley <clyu2@nuvoton.com>"

static const char       gadget_name [] = "nuc900-udc";
static const char	driver_desc [] = DRIVER_DESC;
static const char ep0name [] = "ep0";

static const char *const ep_name[] = {
        ep0name,                                /* everyone has ep0 */
        "ep1", "ep2", "ep3", "ep4", "ep5", "ep6"
};



#define EP0_FIFO_SIZE           64
#define EP_FIFO_SIZE            512

static struct nuc900_udc controller;

static void udc_isr_rst(struct nuc900_udc *dev);
static void udc_isr_dma(struct nuc900_udc *dev);
static void udc_isr_ctrl_pkt(struct nuc900_udc *dev);
static void udc_isr_update_dev(struct nuc900_udc *dev);
static u32 udc_transfer(struct nuc900_ep *ep, u8* buf, size_t size, u32 mode);


static void nuke (struct nuc900_udc *udc, struct nuc900_ep *ep)
{

        while (!list_empty (&ep->queue)) {
                struct nuc900_request  *req;
                req = list_entry (ep->queue.next, struct nuc900_request, queue);
                list_del_init (&req->queue);
                req->req.status = -ESHUTDOWN;
                spin_unlock (&udc->lock);
                req->req.complete (&ep->ep, &req->req);
                spin_lock (&udc->lock);
        }

}


static void done(struct nuc900_ep *ep, struct nuc900_request *req, int status)
{
        struct nuc900_udc *udc = &controller;

        list_del_init(&req->queue); //del req->queue from ep->queue

        if (list_empty(&ep->queue)) {
                if (ep->index)
                        __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
        } else {
                __raw_writel(ep->irq_enb, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
        }

        if (likely (req->req.status == -EINPROGRESS))
                req->req.status = status;
        else
                status = req->req.status;

        if (req->dma_mapped) {
                dma_unmap_single(&udc->pdev->dev, req->req.dma, req->req.length, ep->EP_Dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
                req->req.dma = DMA_ADDR_INVALID;
                req->dma_mapped = 0;
        }

        req->req.complete(&ep->ep, &req->req);
}


static void start_write(struct nuc900_ep *ep, u8* buf,u32 length)
{
        struct nuc900_udc *dev = ep->dev;
        u32 volatile reg;

        if (dev->usb_dma_trigger) {
                printk("*** dma trigger ***\n");
                return;
        }

        dev->usb_dma_trigger = 1;
        dev->usb_dma_cnt = length;
        dev->usb_dma_owner = ep->index;

        __raw_writel((USB_DMA_REQ | USB_RST_STS | USB_SUS_REQ), controller.reg + REG_USBD_IRQ_ENB);

        __raw_writel((u32)buf, controller.reg + REG_USBD_AHB_DMA_ADDR);//Tell DMA the memory physcal address
        __raw_writel(length, controller.reg + REG_USBD_DMA_CNT);

        reg = __raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS);
        if ((reg & 0x40) != 0x40)
                __raw_writel((reg | 0x00000020), controller.reg + REG_USBD_DMA_CTRL_STS);

//printk("start_write: togle %d\n", __raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1)));
        return ;
}

static void start_read(struct nuc900_ep *ep, u8* buf, u32 length)
{
        struct nuc900_udc	*dev = ep->dev;

        if (dev->usb_dma_trigger) {
                printk("*** dma trigger ***\n");
                return;
        }

        __raw_writel((USB_DMA_REQ | USB_RST_STS | USB_SUS_REQ), controller.reg + REG_USBD_IRQ_ENB);
        __raw_writel((u32)buf, controller.reg + REG_USBD_AHB_DMA_ADDR);//Tell DMA the memory address
        __raw_writel(length, controller.reg + REG_USBD_DMA_CNT);

        dev->usb_dma_trigger = 1;
        dev->usb_dma_cnt = length;
        dev->usb_dma_loop = (length+31)/32;
        dev->usb_dma_owner = ep->index;

        __raw_writel(__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)|0x00000020, controller.reg + REG_USBD_DMA_CTRL_STS);

        return ;
}

static inline void clear_ep_state (struct nuc900_udc *dev)
{
        unsigned i;

        /* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
        * fifos, and pending transactions mustn't be continued in any case.
        */
        for (i = 0; i < NUC900_ENDPOINTS; i++)
                nuke(dev, &dev->ep[i]);
}

/*
 * 	write_packet
 */
static inline int
write_packet(struct nuc900_ep *ep, struct nuc900_request *req)
{
        struct nuc900_udc *udc = &controller;
        unsigned	len, tmp;
        u8		*buf;
        u16 data, i;
        u32 max;

        buf = req->req.buf + req->req.actual;

        if (ep->EP_Num == 0) { //ctrl pipe don't use DMA
                max = ep->ep.maxpacket;
                len = min(req->req.length - req->req.actual, max);

                if (len == 0) {
                        if (req->req.zero&&!req->req.length) {
                                __raw_writel(CEP_ZEROLEN, controller.reg + REG_USBD_CEP_CTRL_STAT);
                        }
                } else {
                        tmp = len / 2;

                        for (i=0; i<tmp; i++) {
                                data = *buf++;
                                data |= *buf<<8;
                                buf++;
                                __raw_writel(data, controller.reg + REG_USBD_CEP_DATA_BUF);
                        }

                        if (len % 2) {
                                __raw_writeb( *buf & 0xff, controller.reg + REG_USBD_CEP_DATA_BUF);
                                tmp++;
                        }

                        __raw_writel(len, controller.reg + REG_USBD_IN_TRNSFR_CNT);
                }

                req->req.actual += len;

        } else {
                len = req->req.length - req->req.actual;

                if (req->req.dma == DMA_ADDR_INVALID) {
                        req->req.dma = dma_map_single(&udc->pdev->dev, req->req.buf, req->req.length, ep->EP_Dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
                        req->dma_mapped = 1;
                } else {
                        dma_sync_single_for_device(&udc->pdev->dev, req->req.dma, req->req.length, ep->EP_Dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
                        req->dma_mapped = 0;
                }
                buf = (u8*)(req->req.dma + req->req.actual);

                if (len == 0) {
                        printk("write_packet send zero packet\n");
                        __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|EP_ZERO_IN,
                                     controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));
                } else {
                        len = udc_transfer(ep, buf, len, DMA_WRITE);
                }

                req->req.actual += len;
        }

        return len;
}

/*
 * 	write_fifo
 */
// return:  0 = still running, 1 = completed, negative = errno
static int write_fifo(struct nuc900_ep *ep, struct nuc900_request *req)
{
        u32 len;

        len = write_packet(ep, req);

        /* last packet is often short (sometimes a zlp) */

        if (req->req.length == req->req.actual/* && !req->req.zero*/) {
                done(ep, req, 0);
                return 1;
        } else
                return 0;

}

static inline int read_packet(struct nuc900_ep *ep,u8 *buf,
                              struct nuc900_request *req, u16 cnt)
{
        struct nuc900_udc *udc = &controller;
        unsigned	len, tmp, fifo_count;
        u16 data, i;


        if (ep->EP_Num == 0) { //ctrl pipe don't use DMA
                fifo_count = __raw_readl(controller.reg + REG_USBD_CEP_CNT);
                len = min(req->req.length - req->req.actual, fifo_count);

                tmp = len/2;

                for (i=0; i<tmp; i++) {
                        data = __raw_readl(controller.reg + REG_USBD_CEP_DATA_BUF);
                        *buf++ = data&0xFF;
                        *buf++ = (data&0xFF00)>>8;
                }

                if ((len % 2) != 0) {
                        data = __raw_readb(controller.reg + REG_USBD_CEP_DATA_BUF);
                        *buf++ = data&0xFF;
                }
                req->req.actual += len;

        } else {
                if (req->req.dma == DMA_ADDR_INVALID) {
                        req->req.dma = dma_map_single(&udc->pdev->dev, req->req.buf, req->req.length, ep->EP_Dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
                        req->dma_mapped = 1;
                } else {
                        dma_sync_single_for_device(&udc->pdev->dev, req->req.dma, req->req.length, ep->EP_Dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
                        req->dma_mapped = 0;
                }
                buf = (u8*)req->req.dma;
                len = req->req.length - req->req.actual;

                if (cnt && cnt < ep->ep.maxpacket)
                        len = udc_transfer(ep, buf, cnt, DMA_READ);
                else if (len) {
                        len = udc_transfer(ep, buf, len, DMA_READ);
                }
                req->req.actual += len;
        }

        return len;
}

// return:  0 = still running, 1 = queue empty, negative = errno
static int read_fifo(struct nuc900_ep *ep, struct nuc900_request *req, u16 cnt)
{
        u8		*buf;
        unsigned	bufferspace;
        int is_last=1;
        int fifo_count = 0;


        buf = req->req.buf + req->req.actual;
        bufferspace = req->req.length - req->req.actual;
        if (!bufferspace) {
                printk("read_fifo: Buffer full !!\n");
                return -1;
        }

        fifo_count=read_packet(ep, buf, req, cnt);

        if (req->req.length == req->req.actual)
                done(ep, req, 0);
        else if (fifo_count && fifo_count < ep->ep.maxpacket) {
                done(ep, req, 0);
                /* overflowed this request?  flush extra data */
                if (req->req.length != req->req.actual) {
                        printk("%s(): EOVERFLOW set\n", __FUNCTION__);
                        if (req->req.short_not_ok)
                                req->req.status = -EOVERFLOW;	//device read less then host write
                }
        } else
                is_last = 0;


        return is_last;
}

static void Get_SetupPacket(struct usb_ctrlrequest *pcrq, u32 temp)
{
        pcrq->bRequestType = (u8)temp & 0xff;
        pcrq->bRequest = (u8)(temp >> 8) & 0xff;
        pcrq->wValue = (u16)__raw_readl(controller.reg + REG_USBD_SETUP3_2);
        pcrq->wIndex = (u16)__raw_readl(controller.reg + REG_USBD_SETUP5_4);
        pcrq->wLength = (u16)__raw_readl(controller.reg + REG_USBD_SETUP7_6);

        printk("setup:%x,%x,%x,%x\n", temp, pcrq->wValue, pcrq->wIndex, pcrq->wLength);
}

void paser_irq_stat(int irq, struct nuc900_udc *dev)
{

        __raw_writel(irq, controller.reg + REG_USBD_IRQ_STAT);//clear irq bit

        switch (irq) {
        case USB_SOF:

                break;

        case USB_RST_STS://reset
                udc_isr_rst(dev);

                break;

        case USB_RESUME:

                __raw_writel((USB_RST_STS|USB_SUS_REQ), controller.reg + REG_USBD_IRQ_ENB);

                break;

        case USB_SUS_REQ:
                if (dev == NULL) {
                        break;
                }

                __raw_writel((USB_RST_STS | USB_RESUME), controller.reg + REG_USBD_IRQ_ENB);

                break;

        case USB_HS_SETTLE:
                dev->usb_devstate = USB_FULLSPEED;		//default state
                dev->usb_address = 0;		//zero
                __raw_writel(0x002, controller.reg + REG_USBD_CEP_IRQ_ENB);
                break;

        case USB_DMA_REQ:
                udc_isr_dma(dev);
                break;

        case USABLE_CLK:

                break;
        default:

                break;

        }

        return ;

}


void paser_irq_cep(int irq, struct nuc900_udc *dev, u32 IrqSt)
{
        struct nuc900_ep	*ep = &dev->ep[0];
        struct nuc900_request	*req;
        int		is_last=1;

        if (list_empty(&ep->queue)) {
                req = 0;
        } else {
                req = list_entry(ep->queue.next, struct nuc900_request, queue);
        }

        switch (irq) {
        case CEP_SUPPKT://receive setup packet
                dev->ep0state=EP0_IDLE;
                dev->setup_ret = 0;

                udc_isr_ctrl_pkt(dev);
                break;

        case CEP_DATA_RXD:


                if (dev->ep0state == EP0_OUT_DATA_PHASE) {
                        if (req)
                                is_last = read_fifo(ep,req, 0);

                        __raw_writel(0x400, controller.reg + REG_USBD_CEP_IRQ_STAT);

                        if (!is_last)
                                __raw_writel(0x440, controller.reg + REG_USBD_CEP_IRQ_ENB);//enable out token and status complete int
                        else { //transfer finished
                                __raw_writel(0x04C, controller.reg + REG_USBD_CEP_IRQ_STAT);
                                __raw_writel(CEP_NAK_CLEAR, controller.reg + REG_USBD_CEP_CTRL_STAT);	// clear nak so that sts stage is complete
                                __raw_writel(0x400, controller.reg + REG_USBD_CEP_IRQ_ENB);		// suppkt int//enb sts completion int
                                dev->ep0state = EP0_END_XFER;
                        }
                }

                return;

        case CEP_IN_TOK:

                if ((IrqSt & CEP_STS_END))
                        dev->ep0state=EP0_IDLE;

                if (dev->setup_ret < 0) { // == -EOPNOTSUPP)
                        printk("CEP send zero pkt\n");
                        __raw_writel(CEP_ZEROLEN, controller.reg + REG_USBD_CEP_CTRL_STAT);
                        __raw_writel(0x400, controller.reg + REG_USBD_CEP_IRQ_ENB);		//enb sts completion int
                }

                else if (dev->ep0state == EP0_IN_DATA_PHASE) {


                        if (req) {
                                is_last = write_fifo(ep,req);
                        }


                        if (!is_last)
                                __raw_writel(0x408, controller.reg + REG_USBD_CEP_IRQ_ENB);
                        else {
                                if (dev->setup_ret >= 0)
                                        __raw_writel(CEP_NAK_CLEAR, controller.reg + REG_USBD_CEP_CTRL_STAT);	// clear nak so that sts stage is complete
                                __raw_writel(0x402, controller.reg + REG_USBD_CEP_IRQ_ENB);		// suppkt int//enb sts completion int

                                if (dev->setup_ret < 0)//== -EOPNOTSUPP)
                                        dev->ep0state=EP0_IDLE;
                                else if (dev->ep0state != EP0_IDLE)
                                        dev->ep0state=EP0_END_XFER;
                        }
                }

                return;

        case CEP_PING_TOK:

                __raw_writel(0x402, controller.reg + REG_USBD_CEP_IRQ_ENB);		// suppkt int//enb sts completion int
                return;

        case CEP_DATA_TXD:
                return;

        case CEP_STS_END:

                __raw_writel(0x4A, controller.reg + REG_USBD_CEP_IRQ_ENB);
                udc_isr_update_dev(dev);
                dev->ep0state=EP0_IDLE;
                dev->setup_ret = 0;

                break;

        default:
                break;

        }

        return ;

}


void paser_irq_nep(int irq, struct nuc900_ep *ep, u32 IrqSt)
{
        struct nuc900_udc *dev = ep->dev;
        struct nuc900_request	*req;
        int i;
        u16 data, fifo_count, tmp, loop;
        u8 *buf;
        u32 datacnt_reg;

        if (list_empty(&ep->queue)) {
                printk("nep->queue is empty\n");
                req = 0;
        } else {
                __raw_writel(__raw_readl(controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1)),
                             controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));
                req = list_entry(ep->queue.next, struct nuc900_request, queue);
        }
        //printk("paser_irq_nep:0x%x\n", (int)req->req.dma);
        switch (irq) {
        case EP_IN_TOK:
                __raw_writel(irq, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));

                if (ep->EP_Type == EP_TYPE_BLK) {
                        if (__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0x40) { //send last packet
                                printk("send last packet\n");
                                break;
                        }
                }
                if (req == NULL) {
                        __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
                        break;
                }

                while (__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)&0x20);//wait DMA complete
                if (dev->usb_dma_trigger) {
                        printk("IN dma triggered\n");
                        while ((__raw_readl(controller.reg + REG_USBD_IRQ_STAT) & 0x20) == 0);
                        __raw_writel(0x20, controller.reg + REG_USBD_IRQ_STAT);
                        udc_isr_dma(dev);
                }

                write_fifo(ep,req);
                break;

        case EP_BO_SHORT_PKT:
                if (req) {
                        if (dev->usb_dma_trigger) {
                                loop = __raw_readl(controller.reg + REG_USBD_EPA_DATA_CNT + 0x28*(ep->index-1))>>16;
                                printk("loop=%d, %d\n", loop, dev->usb_dma_loop);
                                loop = dev->usb_dma_loop - loop;

                                if (loop)
                                        req->req.actual += loop*32;//each loop 32 bytes
                                //printk("reset dma\n");
                                dev->usb_dma_trigger = 0;
                                //reset DMA
                                __raw_writel(0x80, controller.reg + REG_USBD_DMA_CTRL_STS);
                                __raw_writel(0x00, controller.reg + REG_USBD_DMA_CTRL_STS);

                                //printk("after DMA reset DATA_CNT=%x, %x\n", __raw_readl(controller.reg + REG_USBD_EPA_DATA_CNT + 0x28*(ep->index-1)), dev->irq_enbl);

                                __raw_writel(dev->irq_enbl, controller.reg + REG_USBD_IRQ_ENB_L);
                        }

                        fifo_count = __raw_readl(controller.reg + REG_USBD_EPA_DATA_CNT + 0x28*(ep->index-1));

                        buf = req->req.buf + req->req.actual;
                        tmp = fifo_count / 2;

                        for (i=0; i<tmp; i++) {
                                data = __raw_readl(controller.reg + REG_USBD_EPA_DATA_BUF + 0x28*(ep->index-1));
                                //printk("%x ", data);
                                *buf++ = data&0xFF;
                                *buf++ = (data&0xFF00)>>8;
                        }

                        if ((fifo_count % 2) != 0) {
                                data = __raw_readb(controller.reg + REG_USBD_EPA_DATA_BUF + 0x28*(ep->index-1));
                                //printk("%x\n", data);
                                *buf++ = data&0xFF;
                        }

                        if (ep->buffer_disabled) {
                                __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1)))&0x77,
                                             controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1));//enable buffer
                                __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|0x80,
                                             controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));//disable buffer when short packet
                        }

                        req->req.actual += fifo_count;

                        done(ep, req, 0);
                } else {
                        __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
                }

                break;

        case EP_DATA_RXD:

                if (req == NULL) {
                        __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
                        break;
                }
                datacnt_reg = (u32)(REG_USBD_EPA_DATA_CNT + 0x28*(ep->index-1));
                if (__raw_readl(controller.reg + datacnt_reg) == 0)
                        break;

                while (__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)&0x20);//wait DMA complete
                fifo_count = __raw_readl(controller.reg + datacnt_reg);


                if (dev->usb_dma_trigger) {
                        printk("RxED dma triggered\n");
                        while ((__raw_readl(controller.reg + REG_USBD_IRQ_STAT) & 0x20) == 0);
                        __raw_writel(0x02, controller.reg + REG_USBD_IRQ_STAT);
                        udc_isr_dma(dev);
                }

                read_fifo(ep,req, __raw_readl(controller.reg + datacnt_reg));

                break;
        default:
                printk("irq: %d not handled !\n",irq);
                __raw_writel(irq, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));
                break;

        }

        return ;
}

void paser_irq_nepint(int irq, struct nuc900_ep *ep, u32 IrqSt)
{
        struct nuc900_udc *dev = ep->dev;
        struct nuc900_request	*req;

        __raw_writel(irq, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));

        if (list_empty(&ep->queue)) {
                printk("nepirq->queue is empty\n");
                req = 0;
                return;
        } else {
                req = list_entry(ep->queue.next, struct nuc900_request, queue);
        }

        switch (irq) {
        case EP_IN_TOK:

                while (__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)&0x20);//wait DMA complete
                if (dev->usb_dma_trigger) {
                        printk("int IN dma triggered\n");
                        while ((__raw_readl(controller.reg + REG_USBD_IRQ_STAT) & 0x20) == 0);
                        __raw_writel(0x20, controller.reg + REG_USBD_IRQ_STAT);
                        udc_isr_dma(dev);
                }
                write_fifo(ep,req);

                break;
        default:
                printk("irq: %d not handled !\n",irq);
                __raw_writel(irq, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));
                break;

        }

        return ;
}


/*
 *      nuc900_udc_irq - interrupt handler
 */
static irqreturn_t nuc900_udc_irq(int irq, void *_dev)
{
        struct nuc900_udc *dev;
        struct nuc900_ep *ep;
        u32 volatile IrqStL, IrqEnL;
        u32 volatile  IrqSt, IrqEn;
        int i=0, j;

        dev=(struct nuc900_udc *)(_dev);

        IrqStL = __raw_readl(controller.reg + REG_USBD_IRQ_STAT_L);	/* 0x000 register get interrupt status */
        IrqEnL = __raw_readl(controller.reg + REG_USBD_IRQ_ENB_L);

        IrqStL = IrqStL & IrqEnL ;
        if (!IrqStL) {
                printk("Not our interrupt !\n");
                return IRQ_HANDLED;
        }

        if (IrqStL & IRQ_USB_STAT) {
                IrqSt = __raw_readl(controller.reg + REG_USBD_IRQ_STAT);
                IrqEn = __raw_readl(controller.reg + REG_USBD_IRQ_ENB);
                __raw_writel(IrqSt, controller.reg + REG_USBD_IRQ_STAT);

                IrqSt = IrqSt & IrqEn ;

                if (IrqSt && dev->driver) {
                        for (i=0; i<6; i++) {
                                if (IrqSt&(1<<i)) {
                                        paser_irq_stat(1<<i,dev);
                                        break;
                                }
                        }
                }

        }//end IRQ_USB_STAT


        if (IrqStL & IRQ_CEP) {
                IrqSt = __raw_readl(controller.reg + REG_USBD_CEP_IRQ_STAT);
                IrqEn = __raw_readl(controller.reg + REG_USBD_CEP_IRQ_ENB);
                //printk("cep:%x, %x\n", IrqSt, IrqEn);
                IrqSt = IrqSt & IrqEn ;
                __raw_writel(IrqSt, controller.reg + REG_USBD_CEP_IRQ_STAT);

                if (IrqSt && dev->driver) {
                        //for(i=12;i>=0;i--)
                        if (IrqSt&CEP_STS_END) { //deal with STS END
                                if (dev->ep0state == EP0_OUT_DATA_PHASE)
                                        IrqSt &= 0x1BF7;
                                paser_irq_cep(CEP_STS_END,dev,IrqSt);
                        }
                        for (i=0; i<13; i++) {
                                if (i == 10)
                                        continue;
                                if (IrqSt&(1<<i)) {
                                        paser_irq_cep(1<<i,dev,IrqSt);
                                        //break;
                                }
                        }
                }
        }

        if (IrqStL & IRQ_NCEP) {
                IrqStL >>= 2;

                for (j = 0; j < 6; j++) { //6 endpoints
                        if (IrqStL & (1 << j)) {
                                //in-token and out token interrupt can deal with one only
                                IrqSt = __raw_readl(controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28 * j);
                                IrqEn = __raw_readl(controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28 * j);

                                IrqSt = IrqSt & IrqEn ;
                                if (IrqSt && dev->driver) {
                                        ep = &dev->ep[j+1];

                                        for (i=12; i>=0; i--) {
                                                if (IrqSt&(1<<i)) {
                                                        if ((1<<i) == EP_BO_SHORT_PKT)
                                                                IrqSt &= 0x1FCF;//clear out token/RxED intr
                                                        if ((ep->EP_Type == EP_TYPE_BLK) || (ep->EP_Type == EP_TYPE_ISO))
                                                                paser_irq_nep(1<<i, ep, IrqSt);
                                                        else if (ep->EP_Type == EP_TYPE_INT)
                                                                paser_irq_nepint(1<<i, ep, IrqSt);
                                                        break;
                                                }
                                        }
                                }
                        }
                }
        }//if end

        return IRQ_HANDLED;


}


static s32 sram_data[7][2] = {{0,0x40}};

//0-3F for Ctrl pipe
s32 get_sram_base(struct nuc900_udc	*dev, u32 max)
{
        int i, cnt = 1, j;
        s32 start, end;

        for (i = 1; i < NUC900_ENDPOINTS; i++) {
                struct nuc900_ep *ep = &dev->ep[i];

                start = __raw_readl(controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
                end = __raw_readl(controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));
                if (end - start > 0) {
                        sram_data[cnt][0] = start;
                        sram_data[cnt][1] = end + 1;
                        cnt++;
                }
        }

        if (cnt == 1)
                return 0x40;

        //sorting from small to big
        j= 1;
        while ((j<cnt)) {
                for (i=0; i<cnt -j; i++) {
                        if (sram_data[i][0]>sram_data[i+1][0]) {
                                start = sram_data[i][0];
                                end = sram_data[i][1];
                                sram_data[i][0] = sram_data[i+1][0];
                                sram_data[i][1] = sram_data[i+1][1];
                                sram_data[i+1][0] = start;
                                sram_data[i+1][1] = end;
                        }
                }
                j++;
        }

        for (i = 0; i< cnt-1; i++) {
                if (sram_data[i+1][0] - sram_data[i][1] >= max)
                        return sram_data[i][1];
        }

        if (0x800 - sram_data[cnt-1][1] >= max)
                return sram_data[cnt-1][1];


        return -ENOBUFS;
}

/*
 * 	nuc900_ep_enable
 */
static int nuc900_ep_enable (struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
        struct nuc900_udc	*dev;
        struct nuc900_ep	*ep;
        u32			max, tmp;
        unsigned long		flags;
        u32			int_en_reg;
        s32 sram_addr;

        ep = container_of (_ep, struct nuc900_ep, ep);
        if (!_ep || !desc || ep->desc || _ep->name == ep0name
                        || desc->bDescriptorType != USB_DT_ENDPOINT)
                return -EINVAL;
        dev = ep->dev;

        if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
                return -ESHUTDOWN;

        max = le16_to_cpu (desc->wMaxPacketSize) & 0x1fff;

        spin_lock_irqsave (&dev->lock, flags);
        _ep->maxpacket = max & 0x7ff;

        ep->desc = desc;
        ep->bEndpointAddress = desc->bEndpointAddress;

        /* set max packet */
        if (ep->index != 0) {
                __raw_writel(max, controller.reg + REG_USBD_EPA_MPS + 0x28*(ep->index-1));
                ep->ep.maxpacket = max;

                sram_addr = get_sram_base(dev, max);

                if (sram_addr < 0)
                        return sram_addr;

                __raw_writel(sram_addr, controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
                sram_addr = sram_addr + max;
                __raw_writel(sram_addr-1, controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));
        }

        /* set type, direction, address; reset fifo counters */
        if (ep->index != 0) {
                ep->EP_Num = desc->bEndpointAddress & ~USB_DIR_IN;
                ep->EP_Dir = desc->bEndpointAddress &0x80 ? 1 : 0;
                ep->EP_Type = ep->desc->bmAttributes&USB_ENDPOINT_XFERTYPE_MASK;
                if (ep->EP_Type == USB_ENDPOINT_XFER_ISOC) {
                        ep->EP_Type = EP_TYPE_ISO;
                        ep->EP_Mode = EP_MODE_FLY;
                } else if (ep->EP_Type == USB_ENDPOINT_XFER_BULK) {
                        ep->EP_Type = EP_TYPE_BLK;
                        ep->EP_Mode = EP_MODE_AUTO;
                }
                if (ep->EP_Type == USB_ENDPOINT_XFER_INT) {
                        ep->EP_Type = EP_TYPE_INT;
                        ep->EP_Mode = EP_MODE_MAN;
                }
                __raw_writel(0x9, controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));//DATA0 and flush SRAM

                __raw_writel(ep->EP_Num<<4|ep->EP_Dir<<3|ep->EP_Type<<1|1,
                             controller.reg + REG_USBD_EPA_CFG+0x28*(ep->index-1));
                __raw_writel(ep->EP_Mode, controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));


                /* enable irqs */
                int_en_reg = __raw_readl(controller.reg + REG_USBD_IRQ_ENB_L);
                __raw_writel(int_en_reg | (1<<(ep->index+1)),
                             controller.reg + REG_USBD_IRQ_ENB_L);
                dev->irq_enbl = __raw_readl(controller.reg + REG_USBD_IRQ_ENB_L);

                if (ep->EP_Type == EP_TYPE_BLK) {
                        if (ep->EP_Dir)//IN
                                ep->irq_enb = 0x40;
                        else {
                                ep->irq_enb = 0x10;//0x1020;
                                __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|0x80,
                                             controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1));//disable buffer when short packet
                                ep->buffer_disabled = 1;
                        }
                } else if (ep->EP_Type == EP_TYPE_INT)
                        ep->irq_enb = 0x40;
                else if (ep->EP_Type == EP_TYPE_ISO) {
                        if (ep->EP_Dir)//IN
                                ep->irq_enb = 0x40;
                        else
                                ep->irq_enb = 0x20;
                }
        }


        /* print some debug message */
        tmp = desc->bEndpointAddress;
        printk ("enable %s(%d) ep%x%s-blk max %02x\n",
                _ep->name,ep->EP_Num, tmp, desc->bEndpointAddress & USB_DIR_IN ? "in" : "out", max);

        spin_unlock_irqrestore (&dev->lock, flags);

        return 0;
}

/*
 * nuc900_ep_disable
 */
static int nuc900_ep_disable (struct usb_ep *_ep)
{
        struct nuc900_ep *ep = container_of(_ep, struct nuc900_ep, ep);
        unsigned long	flags;

        if (!_ep || !ep->desc) {
                return -EINVAL;
        }

        spin_lock_irqsave(&ep->dev->lock, flags);
        ep->desc = 0;

        __raw_writel(0, controller.reg + REG_USBD_EPA_CFG+0x28*(ep->index-1));
        __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));

        nuke (ep->dev, ep);

        __raw_writel(0, controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
        __raw_writel(0, controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));

        spin_unlock_irqrestore(&ep->dev->lock, flags);

        printk("%s disabled\n", _ep->name);

        return 0;
}

/*
 * nuc900_alloc_request
 */
static struct usb_request *
nuc900_alloc_request (struct usb_ep *_ep, gfp_t mem_flags) {
        struct nuc900_ep	*ep;
        struct nuc900_request	*req;

        //printk("nuc900_alloc_request(ep=%p,flags=%d) ", _ep, mem_flags);

        ep = container_of (_ep, struct nuc900_ep, ep);
        if (!_ep)
                return 0;

        req = kmalloc (sizeof *req, mem_flags);
        if (!req)
                return 0;
        memset (req, 0, sizeof *req);
        INIT_LIST_HEAD (&req->queue);
        req->req.dma = DMA_ADDR_INVALID;

        //printk("req=0x%x, buf0x%x\n", (int)req, (int)req->req.buf);
        return &req->req;
}

/*
 * nuc900_free_request
 */
static void
nuc900_free_request (struct usb_ep *_ep, struct usb_request *_req)
{
        struct nuc900_ep	*ep;
        struct nuc900_request	*req;

        //printk("nuc900_free_request(ep=%p,req=%p)\n", _ep, _req);

        ep = container_of (_ep, struct nuc900_ep, ep);
        if (!ep || !_req || (!ep->desc && _ep->name != ep0name))
                return;

        req = container_of (_req, struct nuc900_request, req);

        list_del_init(&req->queue);

        WARN_ON (!list_empty (&req->queue));
        kfree (req);
}


/*
 * 	nuc900_queue
 */
static int
nuc900_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
        struct nuc900_request	*req;
        struct nuc900_ep	*ep;
        struct nuc900_udc	*dev;
        unsigned long flags;

        local_irq_save(flags);

        req = container_of(_req, struct nuc900_request, req);

        if (unlikely (!_req || !_req->complete || !_req->buf
                        || !list_empty(&req->queue))) {
                if (!_req) {
                        printk("nuc900_queue: 1 X X X\n");
                } else {
                        printk("nuc900_queue: 0 %01d %01d %01d\n",!_req->complete,!_req->buf, !list_empty(&req->queue));
                }
                local_irq_restore(flags);
                return -EINVAL;
        }

        ep = container_of(_ep, struct nuc900_ep, ep);
        if (unlikely (!_ep || (!ep->desc && ep->ep.name != ep0name))) {
                printk("nuc900_queue: inval 2\n");
                local_irq_restore(flags);
                return -EINVAL;
        }

        dev = ep->dev;
        if (unlikely (!dev->driver
                        || dev->gadget.speed == USB_SPEED_UNKNOWN)) {
                local_irq_restore(flags);
                printk("nuc900_queue: speed =%d\n",dev->gadget.speed);
                return -ESHUTDOWN;
        }

        /* iso is always one packet per request, that's the only way
         * we can report per-packet status.  that also helps with dma.
         */
        if (ep->desc) { //clyu
                if (unlikely (ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC
                                && req->req.length > le16_to_cpu
                                (ep->desc->wMaxPacketSize))) {
                        local_irq_restore(flags);
                        return -EMSGSIZE;
                }
        }

        _req->status = -EINPROGRESS;
        _req->actual = 0;

        /* pio or dma irq handler advances the queue. */
        if (likely (req != 0)) {
                list_add_tail(&req->queue, &ep->queue);
        }

        if (ep->index==0) { //delayed status
                if (dev->setup_ret > 1000||
                                ((req->req.length==0)&&(dev->ep0state == EP0_OUT_DATA_PHASE))) {
                        printk("delayed status done\n");
                        __raw_writel(CEP_NAK_CLEAR, controller.reg + REG_USBD_CEP_CTRL_STAT);	// clear nak so that sts stage is complete
                        __raw_writel(0x402, controller.reg + REG_USBD_CEP_IRQ_ENB);		// suppkt int//enb sts completion int
                        done(ep, req, 0);
                }
        } else if (ep->index > 0) {
                if (ep->EP_Dir) { //IN
                        if (!dev->usb_dma_trigger || (ep->index!=dev->usb_dma_owner)) {
                                __raw_writel(ep->irq_enb, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
                        }
                } else { //OUT
                        if (!dev->usb_dma_trigger || (ep->index!=dev->usb_dma_owner))
                                __raw_writel(ep->irq_enb, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
                }
        }

        local_irq_restore(flags);

        return 0;
}

/*
 * 	nuc900_dequeue
 */
static int nuc900_dequeue (struct usb_ep *_ep, struct usb_request *_req)
{
        struct nuc900_ep	*ep;
        struct nuc900_udc	*udc = &controller;
        int			retval = -EINVAL;
        unsigned long		flags;
        struct nuc900_request	*req;

        printk("nuc900_dequeue(ep=%p,req=%p)\n", _ep, _req);

        if (!udc->driver)
                return -ESHUTDOWN;

        if (!_ep || !_req)
                return retval;
        ep = container_of (_ep, struct nuc900_ep, ep);
        udc = container_of (ep->gadget, struct nuc900_udc, gadget);

        spin_lock_irqsave (&udc->lock, flags);
        list_for_each_entry(req, &ep->queue, queue) {
                if (&req->req == _req) {
                        list_del_init (&req->queue);
                        _req->status = -ECONNRESET;
                        retval = 0;
                        break;
                }
        }
        spin_unlock_irqrestore (&udc->lock, flags);
        printk("dequeue: %d, req %p\n", retval,  &req->req);
        if (retval == 0) {
                printk( "dequeued req %p from %s, len %d buf %p\n",
                        req, _ep->name, _req->length, _req->buf);

                _req->complete (_ep, _req);
                done(ep, req, -ECONNRESET);
        }

        return retval;
}


/*
 * nuc900_set_halt
 */
static int nuc900_set_halt (struct usb_ep *_ep, int value)
{
        printk("set halt\n");
        return 0;
}


static const struct usb_ep_ops nuc900_ep_ops = {
        .enable         = nuc900_ep_enable,
        .disable        = nuc900_ep_disable,

        .alloc_request  = nuc900_alloc_request,
        .free_request   = nuc900_free_request,

        .queue          = nuc900_queue,
        .dequeue        = nuc900_dequeue,

        .set_halt       = nuc900_set_halt,
};

/*
 * 	nuc900_g_get_frame
 */
static int nuc900_g_get_frame (struct usb_gadget *_gadget)
{
        int tmp;

        tmp = __raw_readl(controller.reg + REG_USBD_FRAME_CNT);

        return tmp & 0xffff;
}

/*
 * 	nuc900_wakeup
 */
static int nuc900_wakeup (struct usb_gadget *_gadget)
{
        return 0;
}

/*
 * 	nuc900_set_selfpowered
 */
static int nuc900_set_selfpowered (struct usb_gadget *_gadget, int value)
{

        return 0;
}



static const struct usb_gadget_ops nuc900_ops = {
        .get_frame          = nuc900_g_get_frame,
        .wakeup             = nuc900_wakeup,
        .set_selfpowered    = nuc900_set_selfpowered,
};


/*
 * 	nop_release
 */
static void nop_release (struct device *dev)
{
        return;
}
/*
 *	usb_gadget_register_driver
 */
int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
        struct nuc900_udc *udc = &controller;
        int retval;


        printk("usb_gadget_register_driver() '%s'\n", driver->driver.name);

        if (!udc)
                return -ENODEV;

        if (udc->driver)
                return -EBUSY;
        if (!driver->bind || !driver->unbind || !driver->setup
                        || driver->speed == USB_SPEED_UNKNOWN)
                return -EINVAL;
        printk("driver->speed=%d\n", driver->speed);
        udc->gadget.name = gadget_name;
        udc->gadget.ops = &nuc900_ops;
        udc->gadget.is_dualspeed = 1;
        udc->gadget.speed = USB_SPEED_HIGH;//USB_SPEED_FULL;
        udc->ep0state = EP0_IDLE;

        udc->gadget.dev.release = nop_release;

#if 0
        //printk("aaa %x\n", (int)controller.reg);
        INIT_LIST_HEAD (&udc->gadget.ep_list);
        for (i = 0; i < NUC900_ENDPOINTS; i++) {
                struct nuc900_ep *ep = &udc->ep[i];

                //printk("ep %d\n", i);
                if (!ep_name[i])
                        break;
                ep->index = i;
                ep->ep.name = ep_name[i];
                ep->ep.ops = &nuc900_ep_ops;
                list_add_tail (&ep->ep.ep_list, &udc->gadget.ep_list);

                /* maxpacket differs between ep0 and others ep */
                if (!i) {
                        ep->EP_Num = 0;
                        ep->ep.maxpacket = EP0_FIFO_SIZE;
                        __raw_writel(0x00000000, controller.reg + REG_USBD_CEP_START_ADDR);
                        __raw_writel(0x0000003f, controller.reg + REG_USBD_CEP_END_ADDR);
                } else {
                        ep->ep.maxpacket = EP_FIFO_SIZE;
                        //sram_addr = udc->ep[0].ep.maxpacket + EP_FIFO_SIZE * (i - 1);
                        //printk("sram_addr=%x\n", sram_addr);
                        __raw_writel(0, controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
                        __raw_writel(0, controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));
                }
                ep->gadget = &udc->gadget;
                ep->dev = udc;
                ep->desc = 0;
                INIT_LIST_HEAD (&ep->queue);
        }

        udc->gadget.ep0 = &udc->ep[0].ep;
        list_del_init (&udc->ep[0].ep.ep_list);
#endif

        udc->driver = driver;

        udc->gadget.dev.driver = &driver->driver;

        printk( "binding gadget driver '%s'\n", driver->driver.name);
        if ((retval = driver->bind (&udc->gadget)) != 0) {
                printk("bind fail\n");
                udc->driver = 0;
                udc->gadget.dev.driver = 0;
                return retval;
        }
        printk( "after driver bind:%p\n" , driver->bind);

#if 0
        driver->driver.bus = udc->gadget.dev.parent->bus;
        driver_register (&driver->driver);
        device_bind_driver (&udc->gadget.dev);
#endif

        mdelay(300);
        __raw_writel(__raw_readl(REG_PWRON) | 0x400, REG_PWRON);//power on usb D+ high

        return 0;
}


/*
 * 	usb_gadget_unregister_driver
 */
int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
        struct nuc900_udc *udc = &controller;


        if (!udc)
                return -ENODEV;
        if (!driver || driver != udc->driver)
                return -EINVAL;

        printk("usb_gadget_unregister_driver() '%s'\n",
               driver->driver.name);

        printk("call driver->unbind\n");
        driver->unbind (&udc->gadget);
        udc->driver = 0;

        printk("device_release_driver\n");
#if 0
        device_release_driver (&udc->gadget.dev);
        driver_unregister (&driver->driver);
#endif
        __raw_writel(__raw_readl(REG_PWRON) & ~0x400, REG_PWRON);//power off usb D+ high


        return 0;
}


static void udc_isr_rst(struct nuc900_udc	*dev)
{
        int i;

        clear_ep_state(dev);

        dev->usb_devstate=0;
        dev->usb_address = 0;


        dev->usb_less_mps=0;

        //reset DMA
        __raw_writel(0x80, controller.reg + REG_USBD_DMA_CTRL_STS);
        __raw_writel(0x00, controller.reg + REG_USBD_DMA_CTRL_STS);

        dev->usb_devstate = 1;		//default state

        printk("speed:%x\n", __raw_readl(controller.reg + REG_USBD_OPER));

        __raw_writel(__raw_readl(controller.reg + REG_USBD_CEP_CTRL_STAT)|CEP_FLUSH,
                     controller.reg + REG_USBD_CEP_CTRL_STAT);// flush fifo
        for (i = 1; i < NUC900_ENDPOINTS; i++) {
                __raw_writel(0x09, controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(i-1)); // flush fifo
        }

        __raw_writel(0, controller.reg + REG_USBD_ADDR);


        __raw_writel(0x002, controller.reg + REG_USBD_CEP_IRQ_ENB);


}

static void udc_isr_dma(struct nuc900_udc *dev)
{
        struct nuc900_request	*req;
        struct nuc900_ep	*ep;
        u32 datacnt_reg;

        if (!dev->usb_dma_trigger) {
                printk("DMA not trigger, intr?\n");
                return;
        }

        ep = &dev->ep[dev->usb_dma_owner];

        datacnt_reg = (u32)(REG_USBD_EPA_DATA_CNT+0x28*(ep->index-1));

        if (dev->usb_dma_dir == Ep_In) {
                __raw_writel(0x40, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));
        }

        dev->usb_dma_trigger = 0;


        if (list_empty(&ep->queue)) {
                printk("DMA ep->queue is empty\n");
                req = 0;
                __raw_writel(dev->irq_enbl, controller.reg + REG_USBD_IRQ_ENB_L);
                return;
        } else {
                req = list_entry(ep->queue.next, struct nuc900_request, queue);
                //printk("req = %x\n", req);
        }

        if (req) {
                if (ep->EP_Type == EP_TYPE_BLK) {
                        if (dev->usb_less_mps == 1) {
                                __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|0x40,
                                             controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1)); // packet end
                                dev->usb_less_mps = 0;
                        }
                } else if (ep->EP_Type == EP_TYPE_INT) {
                        __raw_writel(dev->usb_dma_cnt, controller.reg + REG_USBD_EPA_TRF_CNT+0x28*(ep->index-1));
                }
                req->req.actual += dev->usb_dma_cnt;
                if ((req->req.length == req->req.actual) || dev->usb_dma_cnt < ep->ep.maxpacket) {
                        __raw_writel(dev->irq_enbl, controller.reg + REG_USBD_IRQ_ENB_L);
                        if ((ep->EP_Type == EP_TYPE_BLK) &&
                                        (ep->EP_Dir == 0) && //OUT
                                        dev->usb_dma_cnt < ep->ep.maxpacket) {
                                if (ep->buffer_disabled) {
                                        __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1)))&0x77,
                                                     controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1));//enable buffer
                                        __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|0x80,
                                                     controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));//disable buffer when short packet
                                }
                        }

                        done(ep, req, 0);

                        return;
                }
        }

        if (dev->usb_dma_dir == Ep_Out) {
                if (dev->usb_dma_trigger_next) {
                        dev->usb_dma_trigger_next = 0;
                        printk("dma out\n");
                        read_fifo(ep, req, 0);
                }
        }

        else if (dev->usb_dma_dir == Ep_In) {
                if (dev->usb_less_mps == 1) {
                        dev->usb_less_mps = 0;

                }

                if (dev->usb_dma_trigger_next) {
                        dev->usb_dma_trigger_next = 0;
                        printk("dma in\n");
                        write_fifo(ep, req);
                }
        }
}


static void udc_isr_ctrl_pkt(struct nuc900_udc *dev)
{
        u32	temp;
        u32	ReqErr=0;
        struct nuc900_ep *ep = &dev->ep[0];
        struct usb_ctrlrequest	crq;
        struct nuc900_request	*req;
        int ret;

        if (list_empty(&ep->queue)) {
                //printk("ctrl ep->queue is empty\n");
                req = 0;
        } else {
                req = list_entry(ep->queue.next, struct nuc900_request, queue);
                //printk("req = %x\n", req);
        }

        temp = __raw_readl(controller.reg + REG_USBD_SETUP1_0);

        Get_SetupPacket(&crq,temp);

        dev->crq = crq;

        switch (dev->ep0state) {
        case EP0_IDLE:
                switch (crq.bRequest) {

                case USBR_SET_ADDRESS:
                        ReqErr = ((crq.bRequestType == 0) && ((crq.wValue & 0xff00) == 0)
                                  && (crq.wIndex == 0) && (crq.wLength == 0)) ? 0 : 1;

                        if ((crq.wValue & 0xffff) > 0x7f) {	//within 7f
                                ReqErr=1;	//Devaddr > 127
                        }

                        if (dev->usb_devstate == 3) {
                                ReqErr=1;	//Dev is configured
                        }

                        if (ReqErr==1) {
                                break;		//break this switch loop
                        }

                        if (dev->usb_devstate == 2) {
                                if (crq.wValue == 0)
                                        dev->usb_devstate = 1;		//enter default state
                                dev->usb_address = crq.wValue;	//if wval !=0,use new address
                        }

                        if (dev->usb_devstate == 1) {
                                if (crq.wValue != 0) {
                                        dev->usb_address = crq.wValue;
                                        dev->usb_devstate = 2;
                                }
                        }

                        break;

                case USBR_SET_CONFIGURATION:
                        ReqErr = ((crq.bRequestType == 0) && ((crq.wValue & 0xff00) == 0) &&
                                  ((crq.wValue & 0x80) == 0) && (crq.wIndex == 0) &&
                                  (crq.wLength == 0)) ? 0 : 1;

                        if (dev->usb_devstate == 1) {
                                ReqErr=1;
                        }

                        if (ReqErr==1) {
                                break;	//break this switch loop
                        }

                        if (crq.wValue == 0)
                                dev->usb_devstate = 2;
                        else
                                dev->usb_devstate = 3;
                        break;

                case USBR_SET_INTERFACE:
                        ReqErr = ((crq.bRequestType == 0x1) && ((crq.wValue & 0xff80) == 0)
                                  && ((crq.wIndex & 0xfff0) == 0) && (crq.wLength == 0)) ? 0 : 1;

                        if (!((dev->usb_devstate == 0x3) && (crq.wIndex == 0x0) && (crq.wValue == 0x0)))
                                ReqErr=1;

                        if (ReqErr == 1) {
                                break;	//break this switch loop
                        }

                        break;

                default:
                        break;
                }//switch end

                if (crq.bRequestType & USB_DIR_IN) {
                        dev->ep0state = EP0_IN_DATA_PHASE;
                        __raw_writel(0x08, controller.reg + REG_USBD_CEP_IRQ_ENB);
                } else {
                        dev->ep0state = EP0_OUT_DATA_PHASE;
                        __raw_writel(0x40, controller.reg + REG_USBD_CEP_IRQ_ENB);
                }

                ret = dev->driver->setup(&dev->gadget, &crq);
                dev->setup_ret = ret;
                if (ret < 0) {

                        __raw_writel(0x400, controller.reg + REG_USBD_CEP_IRQ_STAT);
                        __raw_writel(0x448, controller.reg + REG_USBD_CEP_IRQ_ENB);		// enable in/RxED/status complete interrupt
                        __raw_writel(CEP_NAK_CLEAR, controller.reg + REG_USBD_CEP_CTRL_STAT);	//clear nak so that sts stage is complete


                        if (ret == -EOPNOTSUPP)
                                printk("Operation %x not supported\n", crq.bRequest);
                        else {
                                printk("dev->driver->setup failed. (%d)\n",ret);
                        }
                } else if (ret > 1000) { //DELAYED_STATUS
                        printk("DELAYED_STATUS:%p\n", req);
                        dev->ep0state = EP0_END_XFER;
                        __raw_writel(0, controller.reg + REG_USBD_CEP_IRQ_ENB);
                }

                break;

        case EP0_STALL:
                break;
        default:
                break;
        }

        if (ReqErr == 1) {
                __raw_writel(CEP_SEND_STALL, controller.reg + REG_USBD_CEP_CTRL_STAT);
                dev->ep0state = EP0_STALL;
        }

}

void udc_isr_update_dev(struct nuc900_udc *dev)
{
        struct usb_ctrlrequest	*pcrq = &dev->crq;

        //update this device for set requests
        // TODO: write me!!
        switch (pcrq->bRequest) {
        case USBR_SET_ADDRESS:
                __raw_writel(dev->usb_address, controller.reg + REG_USBD_ADDR);
                break;

        case USBR_SET_CONFIGURATION:
                break;

        case USBR_SET_INTERFACE:
                break;

        case USBR_SET_FEATURE:
#if 0
                for (i = 1; i < NUC900_ENDPOINTS; i++) {
                        if (dev->ep[i].EP_Num == dev->usb_haltep) {
                                index = i;
                                break;
                        }
                }
                if (dev->usb_haltep == 0)
                        USB_WRITE(REG_USBD_CEP_CTRL_STAT, CEP_SEND_STALL);
                else if (index)
                        USB_WRITE(REG_USBD_EPA_RSP_SC + 0x28*(dev->ep[index].index-1), EP_HALT);
                else if (dev->usb_enableremotewakeup == 1) {
                        dev->usb_enableremotewakeup = 0;
                        dev->usb_remotewakeup = 1;
                }
#endif
                break;

        case USBR_CLEAR_FEATURE:
#if 0
                if (dev->usb_unhaltep == 1 && dev->usb_haltep == 1) {
                        USB_WRITE(REG_USBD_EPA_RSP_SC, 0x0);
                        USB_WRITE(REG_USBD_EPA_RSP_SC, EP_TOGGLE);
                        dev->usb_haltep = 4; // just for changing the haltep value
                }
                if (dev->usb_unhaltep == 2 && dev->usb_haltep == 2) {
                        USB_WRITE(REG_USBD_EPB_RSP_SC, 0x0);
                        USB_WRITE(REG_USBD_EPB_RSP_SC, EP_TOGGLE);
                        dev->usb_haltep = 4; // just for changing the haltep value
                } else if (dev->usb_disableremotewakeup == 1) {
                        dev->usb_disableremotewakeup=0;
                        dev->usb_remotewakeup=0;
                }
#endif
                break;

        default:
                break;
        }//switch end

        return;
}


static void USB_Init(struct nuc900_udc *dev)
{
        int	j;

        dev->usb_devstate=0;
        dev->usb_address = 0;

        /*
         * configure USB controller
         */
        __raw_writel(0x03, controller.reg + REG_USBD_IRQ_ENB_L);	/* enable usb, cep interrupt */
        __raw_writel((USB_RESUME | USB_RST_STS), controller.reg + REG_USBD_IRQ_ENB);

        __raw_writel(USB_HS, controller.reg + REG_USBD_OPER);//USB 2.0

        __raw_writel(0, controller.reg + REG_USBD_ADDR);
        __raw_writel((CEP_SUPPKT | CEP_STS_END), controller.reg + REG_USBD_CEP_IRQ_ENB);

        for (j = 0; j < NUC900_ENDPOINTS; j++) {
                dev->ep[j].EP_Num = 0xff;
                dev->ep[j].EP_Dir = 0xff;
                dev->ep[j].EP_Type = 0xff;

        }

}

static u32 udc_transfer(struct nuc900_ep *ep, u8* buf, size_t size, u32 mode)
{
        struct nuc900_udc	*dev = ep->dev;
        unsigned int volatile count=0;
        int volatile loop,len=0;

        loop = size / USBD_DMA_LEN;

        if (mode == DMA_WRITE) {
                while (!(__raw_readl(controller.reg + REG_USBD_EPA_IRQ_STAT + (0x28* (ep->index-1))) & 0x02));
                {
                        dev->usb_dma_dir = Ep_In;
                        dev->usb_less_mps = 0;
                        __raw_writel(0x03, controller.reg + REG_USBD_IRQ_ENB_L);


                        __raw_writel((__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)&0xe0) | 0x10 | ep->EP_Num,
                                     controller.reg + REG_USBD_DMA_CTRL_STS);// bulk in, write

                        __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + (0x28* (ep->index-1)));

                        if (loop > 0) {
                                loop--;
                                if (loop > 0)
                                        dev->usb_dma_trigger_next = 1;
                                start_write(ep, buf, USBD_DMA_LEN);
                                //len = USBD_DMA_LEN;
                        } else {
                                if (size >= ep->ep.maxpacket) {
                                        count = size/ep->ep.maxpacket;
                                        count *= ep->ep.maxpacket;

                                        if (count < size)
                                                dev->usb_dma_trigger_next = 1;
                                        start_write(ep, buf, count);
                                        //len = count;
                                } else {
                                        if (ep->EP_Type == EP_TYPE_BLK)
                                                dev->usb_less_mps = 1;
                                        start_write(ep, buf, size);
                                        //len = size;
                                }
                        }
                }
        } else if (mode == DMA_READ) {
                dev->usb_dma_dir = Ep_Out;
                dev->usb_less_mps = 0;
                //dev->irq_enbl = __raw_readl(controller.reg + REG_USBD_IRQ_ENB_L);
                __raw_writel(0x03, controller.reg + REG_USBD_IRQ_ENB_L);

                __raw_writel((__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS) & 0xe0)|ep->EP_Num,
                             controller.reg + REG_USBD_DMA_CTRL_STS);	//read
                __raw_writel(0x1000, controller.reg + REG_USBD_EPA_IRQ_ENB + (0x28* (ep->index-1)));
                __raw_writel(__raw_readl(controller.reg + REG_USBD_IRQ_ENB_L)|(ep->index<<2),
                             controller.reg + REG_USBD_IRQ_ENB_L);

                if (loop > 0) {
                        loop--;
                        if (loop > 0)
                                dev->usb_dma_trigger_next = 1;
                        start_read(ep, buf, USBD_DMA_LEN);
                        //len = USBD_DMA_LEN;
                } else {
                        if (size >= ep->ep.maxpacket) {
                                count = size/ep->ep.maxpacket;
                                count *= ep->ep.maxpacket;
                                if (count < size)
                                        dev->usb_dma_trigger_next = 1;
                                start_read(ep, buf, count);
                                //len = count;
                        } else {
                                //using short packet intr to deal with
                                start_read(ep, buf, size);
                                //len = size;
                        }
                }
        }

        return len;

}



/*
 *	probe - binds to the platform device
 */
static int /*__init*/ nuc900_udc_probe(struct platform_device *pdev)
{
        struct nuc900_udc *udc = &controller;
        struct device *dev = &pdev->dev;
        int error, i;

        dev_dbg(dev, "%s()\n", __func__);

	if(__raw_readl(REG_PWRON) & (1 << 9)) {	// check USBHD bit
		error = -ENODEV;
		dev_err(dev, "***************************************************\n");
		dev_err(dev, "*   Power on setting configured USB PHY0 as host  *\n");
		dev_err(dev, "*   Please disable gadget driver or change power  *\n");
		dev_err(dev, "*   on setting of MA13                            *\n");
		dev_err(dev, "***************************************************\n");
		goto fail1;
	} 
	
        udc->pdev = pdev;
        udc->gadget.dev.parent = &pdev->dev;
        udc->gadget.dev.dma_mask = pdev->dev.dma_mask;

        udc->clk = clk_get(&pdev->dev, NULL);
        if (IS_ERR(udc->clk)) {
                dev_err(dev, "failed to get udc clock\n");
                error = PTR_ERR(udc->clk);
                goto fail1;
        }
        clk_enable(udc->clk);

        udc->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (udc->res == NULL) {
                dev_err(dev, "failed to get I/O memory\n");
                error = -ENXIO;
                goto fail1;
        }

        if (!request_mem_region(udc->res->start,
                                resource_size(udc->res), pdev->name)) {
                dev_err(dev, "failed to request I/O memory\n");
                error = -EBUSY;
                goto fail1;
        }

        udc->reg = ioremap(udc->res->start, resource_size(udc->res));
        if (udc->reg == NULL) {
                dev_err(dev, "failed to remap I/O memory\n");
                error = -ENXIO;
                goto fail1;
        }

        device_initialize(&udc->gadget.dev);
        dev_set_name(&udc->gadget.dev, "gadget");
        udc->gadget.dev.parent = dev;
#if 0
        error = device_register (&udc->gadget.dev);
        if (error < 0) {
                dev_err(dev, "device_register() fail\n");
                goto fail2;
        }
#endif

        platform_set_drvdata (pdev, udc);

        spin_lock_init (&udc->lock);

        __raw_writel(__raw_readl(controller.reg + REG_USBD_PHY_CTL) | 0x200,
                     controller.reg + REG_USBD_PHY_CTL);
        // FIXME: is it possible to loop forever?
        while (1) {
                __raw_writel(0x20, controller.reg + REG_USBD_EPA_MPS);
                if (__raw_readl(controller.reg + REG_USBD_EPA_MPS) == 0x20)
                        break;
        }

        USB_Init(udc);

        /* setup endpoint information */
        INIT_LIST_HEAD (&udc->gadget.ep_list);
        for (i = 0; i < NUC900_ENDPOINTS; i++) {
                struct nuc900_ep *ep = &udc->ep[i];

                printk("ep %d\n", i);
                if (!ep_name[i])
                        break;
                ep->index = i;
                ep->ep.name = ep_name[i];
                ep->ep.ops = &nuc900_ep_ops;
                list_add_tail (&ep->ep.ep_list, &udc->gadget.ep_list);

                /* maxpacket differs between ep0 and others ep */
                if (!i) {
                        ep->EP_Num = 0;
                        ep->ep.maxpacket = EP0_FIFO_SIZE;
                        __raw_writel(0x00000000, controller.reg + REG_USBD_CEP_START_ADDR);
                        __raw_writel(0x0000003f, controller.reg + REG_USBD_CEP_END_ADDR);
                } else {
                        ep->ep.maxpacket = EP_FIFO_SIZE;
                        //sram_addr = udc->ep[0].ep.maxpacket + EP_FIFO_SIZE * (i - 1);
                        //printk("sram_addr=%x\n", sram_addr);
                        __raw_writel(0, controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
                        __raw_writel(0, controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));
                }
                ep->gadget = &udc->gadget;
                ep->dev = udc;
                ep->desc = 0;
                INIT_LIST_HEAD (&ep->queue);
        }

        udc->gadget.ep0 = &udc->ep[0].ep;
        list_del_init (&udc->ep[0].ep.ep_list);

        udc->irq = platform_get_irq(pdev, 0);
        if (udc->irq < 0) {
                dev_err(dev, "Failed to get irq\n");
                error = -ENXIO;
                goto fail2;
        }
        error = request_irq(udc->irq, nuc900_udc_irq,
                            IRQF_DISABLED, gadget_name, udc);
        if (error != 0) {
                dev_err(dev, "request_irq() failed\n");
                goto fail2;
        }


        error = device_add(&udc->gadget.dev);
        if (error != 0) {
                dev_err(dev, "device_add() failed\n");
                goto fail3;
        }

        return 0;
fail3:
        free_irq(udc->irq, udc);
fail2:
        iounmap(udc->reg);
fail1:
        return error;
}

/*
 * 	nuc900_udc_remove
 */
static int __exit nuc900_udc_remove(struct platform_device *pdev)
{
        struct nuc900_udc *udc = platform_get_drvdata (pdev);

        dev_dbg(&pdev->dev, "%s()\n", __func__);

        free_irq(udc->irq, udc);
        iounmap(udc->reg);

        platform_set_drvdata (pdev, NULL);
        device_unregister (&udc->gadget.dev);

        __raw_writel(__raw_readl(controller.reg + REG_USBD_PHY_CTL) & ~0x200,
                     controller.reg + REG_USBD_PHY_CTL);    // phy suspend
        clk_disable(udc->clk);

        return 0;
}

#ifdef CONFIG_PM
static int nuc900_udc_suspend (struct platform_device *pdev, pm_message_t state)
{
        // TODO:
        return 0;
}

static int nuc900_udc_resume (struct platform_device *pdev)
{
        // TODO:
        return 0;
}
#else
#define nuc900_udc_suspend     NULL
#define nuc900_udc_resume      NULL
#endif

static struct platform_driver udc_driver = {
        .probe		= nuc900_udc_probe,
        .remove		= __exit_p(nuc900_udc_remove),
        .suspend	= nuc900_udc_suspend,
        .resume		= nuc900_udc_resume,
        .driver		= {
                .owner	= THIS_MODULE,
                .name	= (char *) "nuc900-usbgadget",
        },
};


//insmod g_mass_storage.ko file=/dev/mmcblk0p1 stall=0 removable=1
static int __init udc_init(void)
{
        return platform_driver_register(&udc_driver);
}

static void __exit udc_exit(void)
{
        platform_driver_unregister (&udc_driver);
}

EXPORT_SYMBOL (usb_gadget_unregister_driver);
EXPORT_SYMBOL (usb_gadget_register_driver);

module_init(udc_init);
module_exit(udc_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
