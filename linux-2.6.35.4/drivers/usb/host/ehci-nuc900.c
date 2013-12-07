/*
 * linux/driver/usb/host/ehci-nuc900.c
 *
 * Copyright (c) 2010 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */


#include <linux/platform_device.h>
#include <linux/signal.h>

#include <linux/clk.h>
#include <mach/irqs.h>
#include <mach/regs-aic.h>
#include <mach/regs-timer.h>

extern void nuc900_enable_group_irq(int src);

#ifdef CONFIG_USB_VIDEO_CLASS


static irqreturn_t ehci_iso (int irq, void *__hcd)
{
        struct usb_hcd *hcd = __hcd;
        unsigned long flags;

        local_irq_save(flags);
       // printk(":");
        ehci_irq(hcd);

        __raw_writel(0x04, REG_TISR); /* clear TIF2 */

        local_irq_restore(flags);
        return IRQ_HANDLED;
}
#endif

static int usb_nuc900_probe(const struct hc_driver *driver,
                            struct platform_device *pdev)
{
        struct usb_hcd *hcd;
        struct ehci_hcd *ehci;
        u32  physical_map_ehci, physical_map_ohci;
        int retval;

        if (IS_ERR(clk_get(&pdev->dev, NULL))) {
                printk("clk_get error!!\n");
                return -1;
        }

        /* enable USB Host clock */
        clk_enable(clk_get(&pdev->dev, NULL));

        /* enable group IRQ */
        nuc900_enable_group_irq(IRQ_USB_EHCI);

        if (pdev->resource[1].flags != IORESOURCE_IRQ) {
                pr_debug("resource[1] is not IORESOURCE_IRQ");
                retval = -ENOMEM;
        }

        hcd = usb_create_hcd(driver, &pdev->dev, "nuc900-ehci");
        if (!hcd) {
                retval = -ENOMEM;
                goto err1;
        }

        hcd->rsrc_start = pdev->resource[0].start;
        hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

        if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
                pr_debug("ehci probe request_mem_region failed");
                retval = -EBUSY;
                goto err2;
        }

        hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
        if (hcd->regs == NULL) {
                pr_debug("ehci error mapping memory\n");
                retval = -EFAULT;
                goto err3;
        }

        ehci = hcd_to_ehci(hcd);
        ehci->caps = hcd->regs;
        ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));

        /* enable PHY 0/1 */
        physical_map_ehci = (u32)ehci->caps;
        __raw_writel(0x160, physical_map_ehci+0xC4);
        __raw_writel(0x520, physical_map_ehci+0xC8);

	/* for EV board */
	if (request_mem_region(0xB0007000, 0x400, hcd_name))
	{
		physical_map_ohci = ioremap(0xB0007000, 0x400);
		if (physical_map_ohci != NULL)
		{
			__raw_writel(0x08, physical_map_ohci+0x204);

#if 1 
			// USB host port reset failed with some USB 2.0 device => 
 	  		// After turning the port power on, these devices will drive a 
 	  		// short K state to bus. This makes the host recognizing it to 
 	  		// be a low-speed device. It will cause the port reset failed. 
 	  		// It's necessary to turn on port power in OHCI (USB 1.1 host) 
 	  		// mode then change the port owner to EHCI (USB 2.0 host) 
 	  		// controller to avoid this issue.
 			__raw_writel(0x10000, physical_map_ohci+0x50);		// HC_RH_STATUS, set OHCI global power 
 			msleep(100);
 			// enable EHCI and port power
 			__raw_writel(0x01, physical_map_ehci+0x60);		// UCFGR
 			__raw_writel(0x1000, physical_map_ehci+0x64);	// UPSCR0
 			__raw_writel(0x1000, physical_map_ehci+0x68);	// UPSCR1
#endif
			iounmap(physical_map_ohci);
		}
		release_mem_region(0xB0007000, 0x400);
	}
	else
	{
		printk("usb_nuc900_probe - Failed to set board!\n"); 
	}


        /* cache this readonly data; minimize chip reads */
        ehci->hcs_params = readl(&ehci->caps->hcs_params);
        ehci->sbrn = 0x20;

        retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);

        if (retval != 0)
                goto err4;

        /* enable EHCI */
        //ehci->regs->configured_flag = 1;
        __raw_writel(1, &ehci->regs->configured_flag);

#ifdef CONFIG_USB_VIDEO_CLASS
#define EVENTS_PER_SEC  100
#define PRESCALE        0x63 // will be 99 + 1 for timer
#define PERIOD          (0x01 << 27)
#define COUNTEN         (0x01 << 30)
#define INTEN           (0x01 << 29)
        {
                struct clk *clk = clk_get(NULL, "timer2");

                clk_enable(clk);

                __raw_writel(clk_get_rate(NULL) / ((PRESCALE + 1) * EVENTS_PER_SEC), REG_TICR2);
                __raw_writel(PERIOD | COUNTEN | INTEN | PRESCALE, REG_TCSR2);
                nuc900_enable_group_irq(IRQ_TIMER2);
                request_irq(IRQ_T_INT_GROUP, ehci_iso, IRQF_DISABLED, "ehci_iso", hcd);

        }
#endif

        return retval;

err4:
        iounmap(hcd->regs);
err3:
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err2:
        usb_put_hcd(hcd);
err1:

        return retval;
}

void usb_nuc900_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
        usb_remove_hcd(hcd);
        iounmap(hcd->regs);
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
        usb_put_hcd(hcd);
}


static const struct hc_driver ehci_nuc900_hc_driver = {
        .description = hcd_name,
        .product_desc = "Nuvoton nuc900 EHCI Host Controller",
        .hcd_priv_size = sizeof(struct ehci_hcd),

        /*
         * generic hardware linkage
         */
        .irq = ehci_irq,
        .flags = HCD_USB2|HCD_MEMORY,

        /*
         * basic lifecycle operations
         */
        .reset = ehci_init,
        .start = ehci_run,

        .stop = ehci_stop,

        /*
         * managing i/o requests and associated device resources
         */
        .urb_enqueue = ehci_urb_enqueue,
        .urb_dequeue = ehci_urb_dequeue,
        .endpoint_disable = ehci_endpoint_disable,

        /*
         * scheduling support
         */
        .get_frame_number = ehci_get_frame,

        /*
         * root hub support
         */
        .hub_status_data = ehci_hub_status_data,
        .hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
        .bus_suspend = ehci_bus_suspend,
        .bus_resume = ehci_bus_resume,
#endif
};

static int ehci_nuc900_probe(struct platform_device *pdev)
{
        //printk("ehci_nuc900_probe()\n");
        if (usb_disabled())
                return -ENODEV;

        return usb_nuc900_probe(&ehci_nuc900_hc_driver, pdev);
}

static int ehci_nuc900_remove(struct platform_device *pdev)
{
        struct usb_hcd *hcd = platform_get_drvdata(pdev);

        usb_nuc900_remove(hcd, pdev);

        return 0;
}

static struct platform_driver ehci_hcd_nuc900_driver = {

        .probe = ehci_nuc900_probe,
        .remove = ehci_nuc900_remove,
        .driver = {
                .name = "nuc900-ehci",
                .owner= THIS_MODULE,
        },
};

static int __init ehci_nuc900_init(void)
{

	return platform_driver_register(&ehci_hcd_nuc900_driver);
}

static void __exit ehci_nuc900_cleanup(void)
{
	platform_driver_unregister(&ehci_hcd_nuc900_driver);

}

//module_init(ehci_nuc900_init);
//module_exit(ehci_nuc900_cleanup);
