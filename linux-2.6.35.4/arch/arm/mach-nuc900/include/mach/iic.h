/*linux/include/asm-arm/arch-nuc900/iic.h
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
 *   2006/08/26     vincen.zswan add this file for nuvoton nuc900 .
 */
 
 
#ifndef __ASM_ARCH_IIC_H
#define __ASM_ARCH_IIC_H __FILE__
 
 
 struct nuc900_platform_i2c 
 {
         unsigned int    flags;
         unsigned int    slave_addr;     // slave address for controller 
         unsigned long   bus_freq;       // standard bus frequency 
         unsigned long   max_freq;       // max frequency for the bus 
         unsigned long   min_freq;       // min frequency for the bus 
         unsigned int	 channel;		 // i2c channel number
         unsigned int	 bus_num;		 // i2c bus number
 };

/* bit map in CMDR */
#define I2C_CMD_START			0x10
#define I2C_CMD_STOP			0x08
#define I2C_CMD_READ			0x04
#define I2C_CMD_WRITE			0x02
#define I2C_CMD_NACK			0x01

/* for transfer use */
#define I2C_WRITE				0x00
#define I2C_READ				0x01

#define I2C_STATE_NOP			0x00
#define I2C_STATE_READ			0x01
#define I2C_STATE_WRITE			0x02
#define I2C_STATE_PROBE			0x03

extern void nuc900_enable_group_irq(int src);
extern void mfp_set_groupg(struct device *dev);
 
#endif // __ASM_ARCH_IIC_H 
