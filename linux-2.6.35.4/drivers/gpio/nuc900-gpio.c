/*
 *  linux/drivers/drivers/gpio/nuc900-gpio.c - Nuvoton NUC900 GPIO Drive
 *
 *  Copyright (c) 2012 Nuvoton Technology Corp.
 *  Author: shanchun 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

//#define GPIO_BASE 0xB800300
#define GPIO_BASE W90X900_VA_GPIO

#define GPIOC_DIR 	GPIO_BASE+0x04
#define GPIOC_DATAOUT 	GPIO_BASE+0x08
#define GPIOC_DATAIN 	GPIO_BASE+0x0c

#define GPIOD_DIR 	GPIO_BASE+0x14
#define GPIOD_DATAOUT 	GPIO_BASE+0x18
#define GPIOD_DATAIN 	GPIO_BASE+0x1C

#define GPIOE_DIR 	GPIO_BASE+0x24
#define GPIOE_DATAOUT 	GPIO_BASE+0x28
#define GPIOE_DATAIN 	GPIO_BASE+0x2C

#define GPIOF_DIR 	GPIO_BASE+0x34
#define GPIOF_DATAOUT 	GPIO_BASE+0x38
#define GPIOF_DATAIN 	GPIO_BASE+0x3C

#define GPIOG_DIR 	GPIO_BASE+0x44
#define GPIOG_DATAOUT 	GPIO_BASE+0x48
#define GPIOG_DATAIN 	GPIO_BASE+0x4C

#define GPIOH_DBNCE 	GPIO_BASE+0x50
#define GPIOH_DIR	GPIO_BASE+0x54
#define GPIOH_DATAOUT 	GPIO_BASE+0x58
#define GPIOH_DATAIN 	GPIO_BASE+0x5C

#define GPIOI_DIR 	GPIO_BASE+0x64
#define GPIOI_DATAOUT 	GPIO_BASE+0x68
#define GPIOI_DATAIN 	GPIO_BASE+0x6c

#define GPIO_OFFSET 0x20
//#define nuc900_gpio_debug printk
#define nuc900_gpio_debug(fmt,args...)
#define	DRIVER_NAME "nuc900-gpio"

#define NUMGPIO 0x20 * 7	//(PortC~PortI)

static DEFINE_SPINLOCK(gpio_lock);

static unsigned short gpio_ba;

struct gpio_port {
	unsigned long dir;
	unsigned long out;
	unsigned long in;
};

static const struct gpio_port port_class[] = {
	{(unsigned long)GPIOC_DIR, (unsigned long)GPIOC_DATAOUT,
	 (unsigned long)GPIOC_DATAIN},
	{(unsigned long)GPIOD_DIR, (unsigned long)GPIOD_DATAOUT,
	 (unsigned long)GPIOD_DATAIN},
	{(unsigned long)GPIOE_DIR, (unsigned long)GPIOE_DATAOUT,
	 (unsigned long)GPIOE_DATAIN},
	{(unsigned long)GPIOF_DIR, (unsigned long)GPIOF_DATAOUT,
	 (unsigned long)GPIOF_DATAIN},
	{(unsigned long)GPIOG_DIR, (unsigned long)GPIOG_DATAOUT,
	 (unsigned long)GPIOG_DATAIN},
	{(unsigned long)GPIOH_DIR, (unsigned long)GPIOH_DATAOUT,
	 (unsigned long)GPIOH_DATAIN},
	{(unsigned long)GPIOI_DIR, (unsigned long)GPIOI_DATAOUT,
	 (unsigned long)GPIOI_DATAIN},
	{},
};

static const struct gpio_port *nuc900_gpio_cla_port(unsigned gpio_num,
						    unsigned *num)
{
	int group;
	group = gpio_num / GPIO_OFFSET;
	*num = gpio_num % GPIO_OFFSET;
	return &port_class[group];
}

static int nuc900_gpio_core_direction_in(struct gpio_chip *gc,
					 unsigned gpio_num)
{
	int port_num;
	unsigned long value;
	const struct gpio_port *port =
	    nuc900_gpio_cla_port(gpio_num, &port_num);
	spin_lock(&gpio_lock);
	value = __raw_readl(port->dir);
	value &= ~(1 << port_num);
	__raw_writel(value, port->dir);
	spin_unlock(&gpio_lock);
	return 0;
}

static int nuc900_gpio_core_get(struct gpio_chip *gc, unsigned gpio_num)
{
	int port_num, value;
	const struct gpio_port *port;
	port = nuc900_gpio_cla_port(gpio_num, &port_num);
	value = 0;

	if ((__raw_readl(port->dir) & (1 << port_num))) {	//GPIO OUT
		value = (__raw_readl(port->out) >> port_num) & 0x1;
		nuc900_gpio_debug("out value=0x%08x\n", value);

	} else {		//GPIO IN
		value = (__raw_readl(port->in) >> port_num) & 0x1;
		__raw_writel(value, port->in);
		nuc900_gpio_debug("in value=0x%08x\n", value);
	}
	return value;
}

static void nuc900_gpio_core_set(struct gpio_chip *gc, unsigned gpio_num,
				 int val)
{
	int port_num, value;
	const struct gpio_port *port =
	    nuc900_gpio_cla_port(gpio_num, &port_num);
	spin_lock(&gpio_lock);

	if ((__raw_readl(port->dir) & (1 << port_num))) {	//GPIO OUT    
		value = __raw_readl(port->out);
		if (val)
			value |= (1 << port_num);
		else
			value &= ~(1 << port_num);
		nuc900_gpio_debug("out value=0x%08x\n", value);
		__raw_writel(value, port->out);

	} else {		//GPIO IN
		value = __raw_readl(port->in);
		if (val)
			value |= (1 << port_num);
		else
			value &= ~(1 << port_num);
		nuc900_gpio_debug("in value=0x%08x\n", value);
		__raw_writel(value, port->in);;
	}

	spin_unlock(&gpio_lock);
}

static int nuc900_gpio_core_direction_out(struct gpio_chip *gc,
					  unsigned gpio_num, int val)
{
	int port_num;
	unsigned long value;
	const struct gpio_port *port =
	    nuc900_gpio_cla_port(gpio_num, &port_num);

	spin_lock(&gpio_lock);
	value = __raw_readl(port->dir);
	value |= (1 << port_num);
	__raw_writel(value, port->dir);
	nuc900_gpio_core_set(gc, gpio_num, val);
	spin_unlock(&gpio_lock);

	return 0;
}

static int nuc900_gpio_core_to_request(struct gpio_chip *chip, unsigned offset)
{
	nuc900_gpio_debug("%s\n", __func__);
	return 0;
}

static void nuc900_gpio_core_to_free(struct gpio_chip *chip, unsigned offset)
{
	nuc900_gpio_debug("%s\n", __func__);
}

static struct gpio_chip nuc900_gpio_port = {
	.label = "nuc900_gpio_port",
	.owner = THIS_MODULE,
	.direction_input = nuc900_gpio_core_direction_in,
	.get = nuc900_gpio_core_get,
	.direction_output = nuc900_gpio_core_direction_out,
	.set = nuc900_gpio_core_set,
	.request = nuc900_gpio_core_to_request,
	.free = nuc900_gpio_core_to_free,
	.base = 0,
	.ngpio = NUMGPIO,
};

static int __devinit nuc900_gpio_probe(struct platform_device *pdev)
{
	int err;
	nuc900_gpio_port.dev = &pdev->dev;
	err = gpiochip_add(&nuc900_gpio_port);
	if (err < 0) {
		goto err_nuc900_gpio_port;
	}
	return 0;

 err_nuc900_gpio_port:
	gpio_ba = 0;
	return err;
}

static int __devexit nuc900_gpio_remove(struct platform_device *pdev)
{
	struct resource *res;
	if (gpio_ba) {
		int err;

		err = gpiochip_remove(&nuc900_gpio_port);
		if (err)
			dev_err(&pdev->dev, "%s failed, %d\n",
				"gpiochip_remove()", err);
		res = platform_get_resource(pdev, IORESOURCE_IO, 0);
		release_region(res->start, resource_size(res));
		gpio_ba = 0;
		return err;
	}

	return 0;
}

static struct platform_driver nuc900_gpio_driver = {
	.probe = nuc900_gpio_probe,
	.remove = __devexit_p(nuc900_gpio_remove),
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

static struct resource nuc900_gpio_resource[] = {
	[0] = {
	       .start = 0xB8003000,
	       .end = 0xB8003FFF,
	       .flags = IORESOURCE_MEM,
	       },
};

struct platform_device nuc900_gpio_device = {
	.name = DRIVER_NAME,
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc900_gpio_resource),
	.resource = nuc900_gpio_resource,
};

static int __init nuc900_gpio_init(void)
{
	int ret;
	ret = platform_driver_register(&nuc900_gpio_driver);

	if (!ret)
		ret = platform_device_register(&nuc900_gpio_device);
	return ret;
}

static void __exit nuc900_gpio_exit(void)
{
	platform_driver_unregister(&nuc900_gpio_driver);
}

module_init(nuc900_gpio_init);
module_exit(nuc900_gpio_exit);

MODULE_AUTHOR("shan chun <SCChung@nuvoton.com>");
MODULE_DESCRIPTION("GPIO interface for Nuvoton NUC900 GPIO Drive");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc900_gpio");
