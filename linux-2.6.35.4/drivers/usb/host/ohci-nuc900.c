/*
 * linux/driver/usb/host/ohci-nuc900.c
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

extern void nuc900_enable_group_irq(int src);

/**
 * usb_hcd_ppc_soc_probe - initialize On-Chip HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
static int usb_hcd_nuc900_probe(const struct hc_driver *driver,
                                struct platform_device *pdev)
{
        int retval;
        struct usb_hcd *hcd;
        struct ohci_hcd *ohci ;
        u32  physical_map_ohci;

        if (IS_ERR(clk_get(&pdev->dev, NULL))) {
                printk("clk_get error!!\n");
                return -1;
        }

        /* enable USB Host clock */
        clk_enable(clk_get(&pdev->dev, NULL));

        /* enable group IRQ */
        nuc900_enable_group_irq(IRQ_USB_OHCI);

        hcd = usb_create_hcd(driver, &pdev->dev, "nuc900-ohci");
        if (!hcd)
                return -ENOMEM;

        hcd->rsrc_start = pdev->resource[0].start;
        hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

        if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
                pr_debug("ohci probe request_mem_region failed");
                retval = -EBUSY;
                goto err1;
        }

        hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
        if (!hcd->regs) {
                pr_debug("ohci error mapping memory\n");
                retval = -ENOMEM;
                goto err2;
        }

        ohci = hcd_to_ohci(hcd);
        ohci_hcd_init(ohci);

        physical_map_ohci = (u32)hcd->regs;
        __raw_writel(0x08, physical_map_ohci+0x204);
        __raw_writel(0x10000, physical_map_ohci+0x50);		// HC_RH_STATUS, set OHCI global power

        retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);

        if (retval == 0)
                return retval;

        pr_debug("Removing nuc900 OHCI USB Controller\n");

        iounmap(hcd->regs);
err2:
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:

        usb_put_hcd(hcd);
        return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_ppc_soc_remove - shutdown processing for On-Chip HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_ppc_soc_probe().
 * It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_nuc900_remove(struct usb_hcd *hcd,
                                  struct platform_device *dev)
{
        usb_remove_hcd(hcd);

        //pr_debug("stopping PPC-SOC USB Controller\n");

        iounmap(hcd->regs);
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
        usb_put_hcd(hcd);
}


static int ohci_nuc900_start (struct usb_hcd *hcd)
{
        struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
        int ret;

        if ((ret = ohci_init(ohci)) < 0)
                return ret;

        if ((ret = ohci_run (ohci)) < 0) {
                err ("can't start %s", hcd->self.bus_name);
                ohci_stop (hcd);
                return ret;
        }

        return 0;
}


static const struct hc_driver ohci_nuc900_hc_driver = {
        .description =		hcd_name,
        .product_desc = 	"Nuvoton nuc900 ohci Host Controller",
        .hcd_priv_size =	sizeof(struct ohci_hcd),

        /*
         * generic hardware linkage
         */
        .irq =			ohci_irq,
        .flags =		HCD_USB11 | HCD_MEMORY,

        /*
         * basic lifecycle operations
         */
        .start =        ohci_nuc900_start,
        .stop =			ohci_stop,

        /*
         * managing i/o requests and associated device resources
         */
        .urb_enqueue =		ohci_urb_enqueue,
        .urb_dequeue =		ohci_urb_dequeue,
        .endpoint_disable =	ohci_endpoint_disable,

        /*
         * scheduling support
         */
        .get_frame_number =	ohci_get_frame,

        /*
         * root hub support
         */
        .hub_status_data =	ohci_hub_status_data,
        .hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
        .bus_suspend =		ohci_bus_suspend,
        .bus_resume =		ohci_bus_resume,
#endif
        .start_port_reset =	ohci_start_port_reset,
};


static int ohci_hcd_nuc900_drv_probe(struct platform_device *pdev)
{
        int ret;

        if (usb_disabled())
                return -ENODEV;

        ret = usb_hcd_nuc900_probe(&ohci_nuc900_hc_driver, pdev);
        return ret;
}

static int ohci_hcd_nuc900_drv_remove(struct platform_device *pdev)
{
        struct usb_hcd *hcd = platform_get_drvdata(pdev);

        usb_hcd_nuc900_remove(hcd, pdev);
        return 0;
}

static struct platform_driver ohci_hcd_nuc900_driver = {
        .probe		= ohci_hcd_nuc900_drv_probe,
        .remove		= ohci_hcd_nuc900_drv_remove,
#ifdef	CONFIG_PM

#endif
        .driver		= {
                .name	= "nuc900-ohci",
                .owner	= THIS_MODULE,
        },
};

static int __init ohci_hcd_nuc900_init(void)
{

	return platform_driver_register(&ohci_hcd_nuc900_driver);
}

static void __exit ohci_hcd_nuc900_cleanup(void)
{
	platform_driver_unregister(&ohci_hcd_nuc900_driver);
}
module_init(ohci_hcd_nuc900_init);
module_exit(ohci_hcd_nuc900_cleanup);
