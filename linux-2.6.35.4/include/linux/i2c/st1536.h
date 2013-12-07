#ifndef __LINUX_I2C_ST1536_H
#define __LINUX_I2C_ST1536_H

/* linux/i2c/st1536.h */

typedef struct{
	u8 y_h:3,
	   reserved:1,
	   x_h:3,
	   valid:1;
	u8 x_l;
	u8 y_l;
	u8 z;
}xyz_data_t;

typedef struct{
	u8 fingers:4,
	   reserved:4;
	u8 keys;
	xyz_data_t xy_data[10];
}stx_report_data_t;


struct st1536_platform_data {
	u16	x_res;				/* 2007. */
	u16	y_res;

	int	(*get_pendown_state)(void);
	void	(*clear_penirq)(void);		/* If needed, clear 2nd level
						   interrupt source */
	int	(*init_platform_hw)(void);
	void	(*exit_platform_hw)(void);
};

#endif
