/*
 *  linux/arch/arm/mach-nuc900/pci.c
 *
 * (C) Copyright Koninklijke Philips Electronics NV 2004. All rights reserved.
 * You can redistribute and/or modify this software under the terms of version 2
 * of the GNU General Public License as published by the Free Software Foundation.
 * THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY; WITHOUT EVEN THE IMPLIED
 * WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * Koninklijke Philips Electronics nor its subsidiaries is obligated to provide any support for this software.
 *
 * ARM NUC920/NUC960 PCI driver.
 *
 * 14/04/2005 Initial version, colin.king@philips.com
 * 10/01/2011 Initial version for nuc900 pci, Wan ZongShun <mcuos.com@gmail.com>
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>


#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>

#include <mach/regs-pci.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <mach/regs-aic.h>

static int nuc900_read_config(struct pci_bus *bus, unsigned int devfn, 
							int where,  int size, u32 *val)
{
	u32 v;

	__raw_writel(devfn * 0x100 + (where&0xfffffffc), REG_CFGADDR);
	v = __raw_readl(REG_CFGDATA);
	switch (size) 
	{
		case 1:
			*val = (v >> ((where % 4)*8))& 0xff;
			break;

		case 2:
			*val = (v >> ((where % 4)*8))& 0xffff;
			break;

		default:
			*val = v;
			break;
	}

	//printk("nuc900_read_config: devfn:%d, where:0x%x, size:%d, val:%x (%x)\n", devfn, where, size, *val, v);

	return PCIBIOS_SUCCESSFUL;
}


static int nuc900_write_config(struct pci_bus *bus, unsigned int devfn, 
							int where, int size, u32 val)
{
	u32  v;

	//printk("nuc900_write_config: devfn:%d, where:0x%x, size:%d, val:%x\n", devfn, where, size, val);

	__raw_writel(devfn*0x100+where, REG_CFGADDR);
	
	v = __raw_readl(REG_CFGDATA);
	
	__raw_writel(devfn*0x100+where, REG_CFGADDR);
	
	switch (size) 
	{
		case 1:
			v &= ~(0xff << (where % 4) * 8);
			v |= (val << (where % 4) * 8);
			__raw_writel(val, REG_CFGDATA);
			break;

		case 2:
			v &= ~(0xffff << (where % 4) * 8);
			v |= (val << (where % 4) * 8);
			__raw_writel(val, REG_CFGDATA);
			break;

		case 4:
			__raw_writel(val, REG_CFGDATA);
			break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops pci_nuc900_ops = 
{
	.read	= nuc900_read_config,
	.write	= nuc900_write_config,
};

static irqreturn_t  pci_irq(int irq, void *dev_id)
{
	//printk("PCI irq!\n");
	return IRQ_HANDLED;
}


static struct resource pci_io = {
	.name	= "NUC900 PCI IO",
	.start	= NUC900_PCI_IO_BASE,
	.end	= NUC900_PCI_IO_BASE + NUC900_PCI_IO_SIZE - 1,
	.flags	= IORESOURCE_IO,
};


static struct resource pci_mem = {
	.name	= "nuc900 PCI Memory",
	.start	= NUC900_PCI_MEM_BASE,
	.end	= NUC900_PCI_MEM_BASE + NUC900_PCI_MEM_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};

static int __init pci_nuc900_setup_resources(struct resource **resource) 
{
	int ret = 0;

	ret = request_resource(&iomem_resource, &pci_io);
	if (ret) {
		printk(KERN_ERR "PCI: unable to allocate I/O "
		       "memory region (%d)\n", ret);
		goto out;
	}
	ret = request_resource(&iomem_resource, &pci_mem);
	if (ret) {
		printk(KERN_ERR "PCI: unable to allocate non-prefetchable "
		       "memory region (%d)\n", ret);
		goto release_io_mem;
	}

	/*
	 * bus->resource[0] is the IO resource for this bus
	 * bus->resource[1] is the mem resource for this bus
	 * bus->resource[2] is the prefetch mem resource for this bus
	 */
	resource[0] = &pci_io;
	resource[1] = &pci_mem;
	resource[2] = NULL;

	goto out;

 release_io_mem:
	release_resource(&pci_io);
 out:
	return ret;
}
int __init pci_nuc900_setup(int nr, struct pci_sys_data *sys)
{
	int ret = 0;

	if (nr == 0) {
		sys->mem_offset = 0;
		sys->io_offset = 0;
		ret = pci_nuc900_setup_resources(sys->resource);
		if (ret) {
			printk("pci_versatile_setup: resources... oops?\n");
			goto out;
		}
	} else {
		printk("pci_versatile_setup: resources... nr == 0??\n");
		goto out;
	}
	ret = 1;
out:
	return ret;
}

struct pci_bus *pci_nuc900_scan_bus(int nr, struct pci_sys_data *sys)
{
	return pci_scan_bus(sys->busnr, &pci_nuc900_ops, sys);
}

void __init pci_nuc900_preinit(void)
{
#if 1
	/* set multi-function pin */
	__raw_writel( ((((((__raw_readl(REG_MFSEL) & (~(3<<2))) & (~(0xf<<4))) & (~(3<<11))) | (1<<2)) | (5<<4)) | (3<<11)), REG_MFSEL);

#if 0
	/* set PLL1 as 66 MHz */
	__raw_writel(0x2B63, REG_PLLCON1);

	/* CLK33 select input from PLL1 */
	__raw_writel( ((__raw_readl(REG_CLKSEL) & (~(0x3 << 2))) | (1 << 2)) , REG_CLKSEL);

	/* set CLK33 as PLL1/2 => 33 MHz */
	__raw_writel( ((__raw_readl(REG_CLKDIV) & ~(0xf << 4)) | (1 << 4)) , REG_CLKDIV);
#else
    __raw_writel(__raw_readl(REG_CLKSEL) & ~(0x3<<2), REG_CLKSEL);			//CK33 from PLL0
    __raw_writel(((__raw_readl(REG_CLKDIV) &~(0xf<<4)) | (5<<4)), REG_CLKDIV);	//PCI CLOCK = 200/6 = 33Mhz
#endif


	/* enable PCI clock */
	__raw_writel(__raw_readl(REG_CLKEN) | 0x4, REG_CLKEN);
	
	__raw_writel(0x20C0, REG_PCICTR);
	msleep(100);
	__raw_writel(0x20CF, REG_PCICTR);
	
	// 2^25 PCI clocks
	msleep(200); 
#endif
}


void __init pci_nuc900_postinit(void)
{
	int  ret;
#if 1
	ret = request_irq(IRQ_PCI, pci_irq, 0, "bus timeout", NULL);
	if (ret)
		printk("pci_nuc900_postinit: unable to grab PCI interrupt: %d\n", ret);
#endif
}

/* 
 * A small note about bridges and interrupts.  The DECchip 21050 (and
 * later) adheres to the PCI-PCI bridge specification.  This says that
 * the interrupts on the other side of a bridge are swizzled in the
 * following manner:
 *
 * Dev    Interrupt   Interrupt 
 *        Pin on      Pin on 
 *        Device      Connector
 *
 *   4    A           A
 *        B           B
 *        C           C
 *        D           D
 * 
 *   5    A           B
 *        B           C
 *        C           D
 *        D           A
 *
 *   6    A           C
 *        B           D
 *        C           A
 *        D           B
 *
 *   7    A           D
 *        B           A
 *        C           B
 *        D           C
 *
 * Where A = pin 1, B = pin 2 and so on and pin=0 = default = A.
 * Thus, each swizzle is ((pin-1) + (device#-4)) % 4
 *
 * The following code swizzles for exactly one bridge.  
 */
static inline int bridge_swizzle(int pin, unsigned int slot) 
{
	return (pin + slot) & 3;
}

/*
 * This routine handles multiple bridges.
 */
static u8 __init nuc900_swizzle(struct pci_dev *dev, u8 *pinp)
{
	int pin = *pinp;

	if (pin == 0)
		pin = 1;

	pin -= 1;
	while (dev->bus->self) {
		pin = bridge_swizzle(pin, PCI_SLOT(dev->devfn));
		/*
		 * move up the chain of bridges, swizzling as we go.
		 */
		dev = dev->bus->self;
	}
	*pinp = pin + 1;

	return PCI_SLOT(dev->devfn);
}


/*
 * map the specified device/slot/pin to an IRQ.  This works out such
 * that slot 9 pin 1 is INT0, pin 2 is INT1, and slot 10 pin 1 is INT1.
 */
static int __init nuc900_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
#if 1
	int  int_num = 0;
	
	if ((slot == 11) && (pin == 1))
		int_num = 0;
	if ((slot == 9) && (pin == 1))
		int_num = 1;

	__raw_writel((__raw_readl(REG_GPIOH_DIR) & (~(1 << int_num))), REG_GPIOH_DIR);
	__raw_writel((__raw_readl(REG_AIC_IRQSC) & (~(0x3<<(int_num << 1)))), REG_AIC_IRQSC);  //GPIO0 low-level sensitive
	__raw_writel((__raw_readl(REG_AIC_GEN) | (1 << int_num)), REG_AIC_GEN);  //group enable GPIO pin number
	__raw_writel(0x41, REG_AIC_SCR2);
	return 2;
#endif
//return 2;
}

static struct hw_pci nuc900_pci __initdata = {
	.swizzle		= nuc900_swizzle,
	.map_irq		= nuc900_map_irq,
	.setup			= pci_nuc900_setup,
	.nr_controllers		= 1,
	.scan			= pci_nuc900_scan_bus,
	.preinit		= pci_nuc900_preinit,
	.postinit		= pci_nuc900_postinit,
};

int __init nuc900_pci_init(void)
{
	pci_common_init(&nuc900_pci);
	return 0;
}

subsys_initcall(nuc900_pci_init);
