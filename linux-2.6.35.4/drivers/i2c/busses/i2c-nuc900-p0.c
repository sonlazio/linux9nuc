/* linux/drivers/i2c/busses/i2c-nuc900-p0.c
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
 *   2008/09/09     Vincen.zswan add this file for nuvoton i2c.
 *   2008/09/10     CCwang change this file for delete compile warning.
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

//#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-iic.h>
#include <mach/regs-aic.h>
#include <mach/iic.h>

/* i2c controller state */
extern unsigned long get_apb_clk(void);

enum nuc900_i2c_port0_state {
        STATE_IDLE,
        STATE_START,
        STATE_READ,
        STATE_WRITE,
        STATE_STOP
};

struct nuc900_i2c_port0 {
        spinlock_t		lock;
        wait_queue_head_t	wait;

        struct i2c_msg		*msg;
        unsigned int		msg_num;
        unsigned int		msg_idx;
        unsigned int		msg_ptr;

        enum nuc900_i2c_port0_state	state;

        void __iomem		*regs;
        struct clk			*clk;
        struct device		*dev;
        struct resource		*irq;
        struct resource		*ioarea;
        struct i2c_adapter	adap;
};

/* default platform data to use if not supplied in the platform_device
*/

static struct nuc900_platform_i2c nuc900_i2c_default_platform = {
        .flags		= 0,
        .slave_addr	= 0x10,
        .bus_freq	= 100,
        .max_freq	= 400,
        .channel	= 0,
        .bus_num	=0,
};

#define i2c0_outl(dword, addr)		writel(dword, I2C0_BA + addr)
#define i2c0_inl(addr)				readl(I2C0_BA + addr)



/* nuc900_i2c_get_platformdata
 *
 * get the platform data associated with the given device, or return
 * the default if there is none
*/

static inline struct nuc900_platform_i2c *nuc900_i2c_get_platformdata(struct device *dev) {
        if (dev->platform_data != NULL)
                return (struct nuc900_platform_i2c *)dev->platform_data;

        return &nuc900_i2c_default_platform;
}

/* nuc900_i2c_master_complete
 *
 * complete the message and wake up the caller, using the given return code,
 * or zero to mean ok.
*/

static inline void nuc900_i2c_master_complete(struct nuc900_i2c_port0 *i2c, int ret)
{
        dev_dbg(i2c->dev, "master_complete %d\n", ret);

        i2c->msg_ptr = 0;
        i2c->msg = NULL;
        i2c->msg_idx ++;
        i2c->msg_num = 0;
        if (ret)
                i2c->msg_idx = ret;

        wake_up(&i2c->wait);
}

/* irq enable/disable functions */

static inline void nuc900_i2c_disable_irq(struct nuc900_i2c_port0 *i2c)
{
        i2c0_outl(0x00, NUC900_I2CCSR);
}

static inline void nuc900_i2c_enable_irq(struct nuc900_i2c_port0 *i2c)
{
        i2c0_outl(0x03, NUC900_I2CCSR);
}


/* nuc900_i2c_message_start
 *
 * put the start of a message onto the bus
*/

static void nuc900_i2c_message_start(struct nuc900_i2c_port0 *i2c,
                                     struct i2c_msg *msg)
{
        unsigned int addr = (msg->addr & 0x7f) << 1;	//slave addr

        if (msg->flags & I2C_M_RD)
                addr |= 1;

        i2c0_outl(addr & 0xff, NUC900_I2CTXR);	/* send first byte */
        i2c0_outl(I2C_CMD_START | I2C_CMD_WRITE, NUC900_I2CCMDR);
}

static inline void nuc900_i2c_stop(struct nuc900_i2c_port0 *i2c, int ret)
{
        /* stop the transfer */
        i2c->state = STATE_STOP;

        nuc900_i2c_master_complete(i2c, ret);
        nuc900_i2c_disable_irq(i2c);
}

/* i2s_s3c_irq_nextbyte
 *
 * process an interrupt and work out what to do
 */

static int i2c_irq_nextbyte(struct nuc900_i2c_port0 *i2c, unsigned long iiccsr)
{
        unsigned char byte;
        int ret = 0;

        if (iiccsr & 0x100)	//transfer in pregress ?
                goto out;

        switch (i2c->state) {

        case STATE_START:
                if (i2c->msg->flags & I2C_M_RD)
                        i2c->state = STATE_READ;
                else
                        i2c->state = STATE_WRITE;

                if (i2c->state == STATE_READ)
                        goto prepare_read;


        case STATE_WRITE:
retry_write:
                if (i2c->msg_ptr < i2c->msg->len) {
                        byte = i2c->msg->buf[i2c->msg_ptr++];
                        i2c0_outl(byte, NUC900_I2CTXR);
                        if (i2c->msg_ptr == i2c->msg->len &&  i2c->msg_idx == i2c->msg_num - 1)		//end data ?
                                i2c0_outl( I2C_CMD_WRITE | I2C_CMD_STOP, NUC900_I2CCMDR);
                        else
                                i2c0_outl( I2C_CMD_WRITE, NUC900_I2CCMDR);
                } else if (i2c->msg_idx < i2c->msg_num - 1) {	//restart a new transmittion
                        i2c->msg_ptr = 0;
                        i2c->msg_idx ++;
                        i2c->msg++;

                        /* check to see if we need to do another message */
                        if (i2c->msg->flags & I2C_M_NOSTART) {

                                if (i2c->msg->flags & I2C_M_RD) {
                                        /* cannot do this, the controller
                                         * forces us to send a new START
                                         * when we change direction */

                                        nuc900_i2c_stop(i2c, -EINVAL);
                                }

                                goto retry_write;
                        } else {
                                /* send the new start */
                                nuc900_i2c_message_start(i2c, i2c->msg);
                                i2c->state = STATE_START;
                        }

                } else if (i2c->msg->len == 0) {
                        i2c0_outl( I2C_CMD_STOP, NUC900_I2CCMDR);
                        nuc900_i2c_stop(i2c, 0);
                } else
                        nuc900_i2c_stop(i2c, 0);

                break;

        case STATE_READ:
                byte = i2c0_inl(NUC900_I2CRXR) & 0xff;		//skip first read
                i2c->msg->buf[i2c->msg_ptr-1] = byte;

prepare_read:
                i2c->msg_ptr ++;
                if (i2c->msg_ptr < i2c->msg->len + 1) {
                        if (i2c->msg_ptr == i2c->msg->len)	//reach the end data
                                i2c0_outl( I2C_CMD_READ | I2C_CMD_STOP | I2C_CMD_NACK, NUC900_I2CCMDR);
                        else
                                i2c0_outl( I2C_CMD_READ, NUC900_I2CCMDR);
                } else
                        nuc900_i2c_stop(i2c, 0);

                break;

        case STATE_IDLE:
        case STATE_STOP:
                break;

        }

out:
        return ret;
}

/* nuc900_i2c_irq
 *
 * top level IRQ servicing routine
*/

static irqreturn_t nuc900_i2c_irq(int irqno, void *dev_id)
{
        struct nuc900_i2c_port0 *i2c = dev_id;
        unsigned long status;

        if (!(readl(REG_AIC_GASR) & 0x04000000))        //check irq for i2c port0
                goto out;
                
        status = i2c0_inl(NUC900_I2CCSR);

        /* mark interrupt flag */
        i2c0_outl(  status | 0x04, NUC900_I2CCSR);

        if (status & 0x200) { // deal with arbitration loss
                //dev_err(i2c->dev, "deal with arbitration loss\n");
                goto out;
        }

        if ((status & 0x800) && (i2c->state == STATE_WRITE || i2c->state == STATE_START)) { // deal with NACK
                //dev_err(i2c->dev, "deal with nack\n");
                i2c0_outl( I2C_CMD_STOP, NUC900_I2CCMDR);
                i2c->state = STATE_STOP;
                nuc900_i2c_disable_irq(i2c);
                goto out;
        }

        if (i2c->state == STATE_IDLE) {
                //dev_dbg(i2c->dev, "IRQ: error i2c->state == IDLE\n");
                goto out;
        }

        /* pretty much this leaves us with the fact that we've
         * transmitted or received whatever byte we last sent */

        i2c_irq_nextbyte(i2c, status);

out:
        return IRQ_HANDLED;
}


/* nuc900_i2c_set_master
 *
 * get the i2c bus for a master transaction
*/

static int nuc900_i2c_set_master(struct nuc900_i2c_port0 *i2c)
{
        int timeout = 400;

        while (timeout-- > 0) {
                if (	(i2c0_inl(NUC900_I2CSWR) & 0x18) == 0x18	&& 	//SDR and SCR keep high
                                (i2c0_inl(NUC900_I2CCSR) & 0x0400) == 0	) {  	//I2C_BUSY is false
                        return 0;
                }

                msleep(1);
        }

        dev_dbg(i2c->dev, "timeout\n");

        return -ETIMEDOUT;
}

/* nuc900_i2c_doxfer
 *
 * this starts an i2c transfer
*/

static int nuc900_i2c_doxfer(struct nuc900_i2c_port0 *i2c, struct i2c_msg *msgs, int num)
{
        unsigned long timeout;
        int ret;

        ret = nuc900_i2c_set_master(i2c);	//check bus
        if (ret != 0) {
                dev_err(i2c->dev, "cannot get bus (error %d)\n", ret);
                ret = -EAGAIN;
                goto out;
        }

        spin_lock_irq(&i2c->lock);

        i2c->msg     = msgs;
        i2c->msg_num = num;
        i2c->msg_ptr = 0;
        i2c->msg_idx = 0;
        i2c->state   = STATE_START;

        nuc900_i2c_enable_irq(i2c);
        nuc900_i2c_message_start(i2c, msgs);
        spin_unlock_irq(&i2c->lock);

        timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 2);

        ret = i2c->msg_idx;

        /* having these next two as dev_err() makes life very
         * noisy when doing an i2cdetect */

        if (timeout == 0) {
                dev_dbg(i2c->dev, "timeout\n");
                ret = -EIO;
        } else if (ret != num) {
                dev_dbg(i2c->dev, "incomplete xfer (%d)\n", ret);
                ret = -EIO;
        }

out:
        return ret;
}

/* nuc900_i2c_xfer
 *
 * first port of call from the i2c bus code when an message needs
 * transferring across the i2c bus.
*/

static int nuc900_i2c_xfer(struct i2c_adapter *adap,
                           struct i2c_msg *msgs, int num)
{
        struct nuc900_i2c_port0 *i2c = (struct nuc900_i2c_port0 *)adap->algo_data;
        int retry;
        int ret;

        for (retry = 0; retry < adap->retries; retry++) {

                ret = nuc900_i2c_doxfer(i2c, msgs, num);

                if (ret != -EAGAIN)
                        return ret;

                dev_dbg(i2c->dev, "Retrying transmission (%d)\n", retry);

        }

        return -EREMOTEIO;
}

/* declare our i2c functionality */
static u32 nuc900_i2c_func(struct i2c_adapter *adap)
{
        return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

/* i2c bus registration info */

static struct i2c_algorithm nuc900_i2c_algorithm = {
        .master_xfer		= nuc900_i2c_xfer,
        .functionality		= nuc900_i2c_func,
};

static struct nuc900_i2c_port0 nuc900_i2c = {
        .lock	= SPIN_LOCK_UNLOCKED,
        .wait	= __WAIT_QUEUE_HEAD_INITIALIZER(nuc900_i2c.wait),
        .adap	= {
                .name			= "nuc900-i2c-p0",
                .owner			= THIS_MODULE,
                .algo			= &nuc900_i2c_algorithm,
                .retries		= 2,
                .class			= I2C_CLASS_HWMON,
        },
};

/* nuc900_i2c_init
 *
 * initialise the controller, set the IO lines and frequency
*/

static int nuc900_i2c_init(struct nuc900_i2c_port0 *i2c)
{
        struct nuc900_platform_i2c *pdata;
        unsigned int reg;

        /* get the plafrom data */

        pdata = nuc900_i2c_get_platformdata(i2c->adap.dev.parent);

        /* set the i2c speed */
        reg = get_apb_clk()/(pdata->bus_freq * 5) - 1;
        i2c0_outl( reg & 0xffff, NUC900_I2CDIVIDER);
        dev_info(i2c->dev, "bus frequency set to %ld KHz\n", pdata->bus_freq);

        return 0;
}

static void nuc900_i2c_free(struct nuc900_i2c_port0 *i2c)
{
        free_irq(IRQ_I2CGROUP, i2c);
}

/* nuc900_i2c_probe
 *
 * called by the bus driver when a suitable device is found
*/

static struct clk *clk;
static int nuc900_i2c_probe(struct platform_device *pdev)
{
        struct nuc900_i2c_port0 *i2c = &nuc900_i2c;
        int ret;


        /* setup info block for the i2c core */
        i2c->dev = &pdev->dev;
        i2c->adap.algo_data = i2c;
        i2c->adap.dev.parent = &pdev->dev;
        i2c->adap.retries = 2;
        i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;

        /* enable i2c clock */
        clk = clk_get(&pdev->dev, NULL);
        clk_enable(clk);
        
	/* setup I2C share pin for I2C */
	mfp_set_groupg(&pdev->dev);

        /* initialise the i2c controller */
        ret = nuc900_i2c_init(i2c);
        if (ret != 0)
                goto out;

        nuc900_enable_group_irq(IRQ_I2C_LINE0);
        ret = request_irq(IRQ_I2CGROUP, nuc900_i2c_irq, IRQF_DISABLED | IRQF_SHARED,  pdev->name, i2c);
        if (ret != 0) {
                dev_err(&pdev->dev, "cannot claim IRQ\n");
                goto out;
        }

        i2c->adap.nr = 0;

        ret = i2c_add_numbered_adapter(&i2c->adap);
        if (ret < 0) {
                dev_err(&pdev->dev, "failed to add bus to i2c core\n");
                goto out;
        }

        platform_set_drvdata(pdev, i2c);
        dev_info(&pdev->dev, "Add nuc900 I2C port0 adapter\n");

out:
        if (ret < 0)
                nuc900_i2c_free(i2c);

        return ret;
}

/* nuc900_i2c_remove
 *
 * called when device is removed from the bus
*/

static int nuc900_i2c_remove(struct platform_device *pdev)
{
        struct nuc900_i2c_port0 *i2c = platform_get_drvdata(pdev);

        if (i2c != NULL) {
                nuc900_i2c_free(i2c);
                platform_set_drvdata(pdev, NULL);
        }
        clk_disable(clk);
	clk_put(clk);
        return 0;
}

#ifdef CONFIG_PM
static int nuc900_i2c_resume(struct platform_device *dev)
{
        struct nuc900_i2c_port0 *i2c = platform_get_drvdata(dev);

        if (i2c != NULL)
                nuc900_i2c_init(i2c);

        return 0;
}

#else
#define nuc900_i2c_resume NULL
#endif

/* device driver for platform bus bits */

static struct platform_driver nuc900_i2c_driver = {
        .probe		= nuc900_i2c_probe,
        .remove		= nuc900_i2c_remove,
        .resume		= nuc900_i2c_resume,
        .driver		= {
                .owner	= THIS_MODULE,
                .name	= "nuc900-i2c-p0",
        },
};

static int __init i2c_adap_nuc900_init(void)
{
        int ret;

        ret = platform_driver_register(&nuc900_i2c_driver);

        return ret;
}

static void __exit i2c_adap_nuc900_exit(void)
{
        platform_driver_unregister(&nuc900_i2c_driver);
}

module_init(i2c_adap_nuc900_init);
module_exit(i2c_adap_nuc900_exit);

MODULE_DESCRIPTION("NUC900 I2C Port-0 Bus driver");
MODULE_LICENSE("GPL");
