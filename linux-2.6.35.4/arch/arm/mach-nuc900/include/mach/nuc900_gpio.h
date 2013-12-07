/* linux/include/linux/nuc900_gpio.h
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
 *   2008/11/10     First version
 */
#include <mach/regs-gpio.h>

/* GPIO group definition */
#define GPIO_GROUP_C 0
#define GPIO_GROUP_D 1
#define GPIO_GROUP_E 2
#define GPIO_GROUP_F 3
#define GPIO_GROUP_G 4
#define GPIO_GROUP_H 5
#define GPIO_GROUP_I 6

/* GPIO register offset definition */
static int __iomem * gpio_reg_dir[7] = { REG_GPIOC_DIR, REG_GPIOD_DIR, REG_GPIOE_DIR, REG_GPIOF_DIR, REG_GPIOG_DIR, REG_GPIOH_DIR, REG_GPIOI_DIR};
static int __iomem * gpio_reg_out[7] = { REG_GPIOC_DATAOUT, REG_GPIOD_DATAOUT, REG_GPIOE_DATAOUT, REG_GPIOF_DATAOUT, REG_GPIOG_DATAOUT, REG_GPIOH_DATAOUT, REG_GPIOI_DATAOUT};
static int __iomem * gpio_reg_in[7] = { REG_GPIOC_DATAIN, REG_GPIOD_DATAIN, REG_GPIOE_DATAIN, REG_GPIOF_DATAIN, REG_GPIOG_DATAIN, REG_GPIOH_DATAIN, REG_GPIOI_DATAIN};

/* returns the value of the GPIO pin */
static inline int nuc900_gpio_get(int group, int num)
{
        return readl(gpio_reg_in[group]) & (1 << num) ? 1:0;
}

/* set direction of pin to input mode */
static inline void nuc900_gpio_set_input(int group, int num)
{
        writel (readl(gpio_reg_dir[group]) & ~(1 << num), gpio_reg_dir[group]);
}

/* set direction of pin to output mode */
static inline void nuc900_gpio_set_output(int group, int num)
{
        writel (readl(gpio_reg_dir[group]) | (1 << num), gpio_reg_dir[group]);
}

/* drive the GPIO signal to state */
static inline void nuc900_gpio_set(int group, int num, int state)
{
        if (state)
                writel (readl(gpio_reg_out[group]) | (1 << num), gpio_reg_out[group]); 		//set high
        else
                writel (readl(gpio_reg_out[group]) & ~(1 << num), gpio_reg_out[group]); 	//set low
}

/* set share pin and direction of gpios */
static inline int nuc900_gpio_configure(int group, int num)
{
        switch (group) {
        case GPIO_GROUP_C:
                writel (readl(GCR_BA + 0xC) & 0xFFFFFFF3, GCR_BA + 0xC); 	//config share pin
                writel (readl(GPIO_BA + 0x4) | (1 << num), GPIO_BA + 0x4); 	//config direction of gpio
                break;

        case GPIO_GROUP_D:
                writel (readl(GCR_BA + 0xC) & 0xFFFFFF0F, GCR_BA + 0xC); 	//config share pin
                writel (readl(GPIO_BA + 0x14) | (1 << num), GPIO_BA + 0x14); //config direction of gpio
                break;

        case GPIO_GROUP_E:
#ifdef CONFIG_CPU_NUC945
                writel (readl(GCR_BA + 0xC) & 0xFFFFFEFF, GCR_BA + 0xC); 	//config share pin
#else
                writel (readl(GCR_BA + 0xC) & 0xFFFFC0FF, GCR_BA + 0xC); 	//config share pin
#endif

                writel (readl(GPIO_BA + 0x24) | (1 << num), GPIO_BA + 0x24); //config direction of gpio
                break;

        case GPIO_GROUP_F:
                writel (readl(GCR_BA + 0xC) & 0xFFFFFFFD, GCR_BA + 0xC); 	//config share pin
                writel (readl(GPIO_BA + 0x34) | (1 << num), GPIO_BA + 0x34); //config direction of gpio
                break;

        case GPIO_GROUP_G:
#if defined(CONFIG_CPU_W90P910) || defined(CONFIG_CPU_NUC960) || defined(CONFIG_CPU_NUC950)
                writel (readl(GCR_BA + 0xC) & 0xFF003FFF, GCR_BA + 0xC); 	//config share pin
                writel (readl(GPIO_BA + 0x44) | (1 << num), GPIO_BA + 0x44); //config direction of gpio
#else
                printk("Error, Not support GPIO Group G!!\n");
                return 0;
#endif
                break;


        case GPIO_GROUP_H:
#if defined(CONFIG_CPU_W90P910) || defined(CONFIG_CPU_NUC960)
                writel (readl(GCR_BA + 0xC) & 0xFCFFFFFF, GCR_BA + 0xC); 	//config share pin
#elif defined(CONFIG_CPU_NUC950)
                writel (readl(GCR_BA + 0xC) & 0xFEFFFFFF, GCR_BA + 0xC); 	//config share pin
#else
                writel (readl(GCR_BA + 0xC) & 0xFEFFFFFF, GCR_BA + 0xC); 	//config share pin
#endif

                writel (readl(GPIO_BA + 0x54) | (1 << num), GPIO_BA + 0x54); //config direction of gpio
                break;

        case GPIO_GROUP_I:
#if defined(CONFIG_CPU_W90P910) || defined(CONFIG_CPU_NUC960)
                writel (readl(GCR_BA + 0xC) & 0xF3FFFFFF, GCR_BA + 0xC); 	//config share pin
                writel (readl(GPIO_BA + 0x64) | (1 << num), GPIO_BA + 0x64); //config direction of gpio
#else
                printk("Error, Not support GPIO Group I!!\n");
                return 0;
#endif
                break;


        default:
                break;
        }

        return 1;
}
