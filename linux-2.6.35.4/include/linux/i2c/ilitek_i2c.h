#ifndef __LINUX_I2C_ILITEK_H
#define __LINUX_I2C_ILITEK_H

/* linux/i2c/ilitek_i2c.h */






struct ilitek_i2c_platform_data {
	u16	x_res;				/* 2007. */
	u16	y_res;

	int	(*get_pendown_state)(void);
	void	(*clear_penirq)(void);		/* If needed, clear 2nd level
						   interrupt source */
	int	(*init_platform_hw)(void);
	void	(*exit_platform_hw)(void);
};

#endif
