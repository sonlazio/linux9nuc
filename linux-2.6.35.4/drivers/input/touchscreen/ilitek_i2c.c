/*
 * drivers/input/touchscreen/st1536.c
 *
 * Copyright (c) 2012 Nuvoton Co., Ltd.
 *	shirley <clyu2@nuvoton.com>
 *
 * Using code from:
 *  - tsc2007.c
 *	Copyright (c) 2008 Kwangwoo Lee
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/ilitek_i2c.h>

#define DRIVER_VERSION "aimv11"

#define TS_POLL_DELAY			20 /* ms delay between samples */
#define TS_POLL_PERIOD			20 /* ms delay between samples */

#define FW_REG				0x00
#define STATUS_REG			0x01
#define CTRL_REG			0x02
#define TIMEOUT_REG			0x03
#define RESOLUTION_REG		0x04
#define SENSE_COUNTER_REG	0x05
#define FW_REVISION_REG		0x0C
#define XY_COORDINATE_REG	0x10
#define MAX_CONTACT_REG		0x3F
#define PAGE_REG			0xFF

#define	MAX_11BIT			((1 << 11) - 1)

#define X_RES_H_SHFT 4
#define X_RES_H_BMSK 0xf
#define Y_RES_H_SHFT 0
#define Y_RES_H_BMSK 0xf
#define FINGERS_SHFT 0
#define FINGERS_BMSK 0xf
#define X_COORD_VALID_SHFT 7
#define X_COORD_VALID_BMSK 0x1
#define X_COORD_H_SHFT 4
#define X_COORD_H_BMSK 0x7
#define Y_COORD_H_SHFT 0
#define Y_COORD_H_BMSK 0x7

// definitions
#define ILITEK_I2C_RETRY_COUNT			3
#define ILITEK_I2C_DRIVER_NAME			"ilitek_i2c"
#define ILITEK_FILE_DRIVER_NAME			"ilitek_file"
#define ILITEK_DEBUG_LEVEL			KERN_INFO
#define ILITEK_ERROR_LEVEL			KERN_ALERT

// i2c command for ilitek touch screen
#define ILITEK_TP_CMD_READ_DATA			0x10
#define ILITEK_TP_CMD_READ_SUB_DATA		0x11
#define ILITEK_TP_CMD_GET_RESOLUTION		0x20
#define ILITEK_TP_CMD_GET_FIRMWARE_VERSION	0x40
#define ILITEK_TP_CMD_GET_PROTOCOL_VERSION	0x42
#define	ILITEK_TP_CMD_CALIBRATION			0xCC
#define	ILITEK_TP_CMD_CALIBRATION_STATUS	0xCD
#define ILITEK_TP_CMD_ERASE_BACKGROUND		0xCE

// define the application command
#define ILITEK_IOCTL_BASE                       100
#define ILITEK_IOCTL_I2C_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 0, unsigned char*)
#define ILITEK_IOCTL_I2C_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 1, int)
#define ILITEK_IOCTL_I2C_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 2, unsigned char*)
#define ILITEK_IOCTL_I2C_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 3, int)
#define ILITEK_IOCTL_USB_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 4, unsigned char*)
#define ILITEK_IOCTL_USB_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 5, int)
#define ILITEK_IOCTL_USB_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 6, unsigned char*)
#define ILITEK_IOCTL_USB_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 7, int)
#define ILITEK_IOCTL_I2C_UPDATE_RESOLUTION      _IOWR(ILITEK_IOCTL_BASE, 8, int)
#define ILITEK_IOCTL_USB_UPDATE_RESOLUTION      _IOWR(ILITEK_IOCTL_BASE, 9, int)
#define ILITEK_IOCTL_I2C_INT_FLAG	            _IOWR(ILITEK_IOCTL_BASE, 10, int)
#define ILITEK_IOCTL_I2C_UPDATE                 _IOWR(ILITEK_IOCTL_BASE, 11, int)
#define ILITEK_IOCTL_STOP_READ_DATA             _IOWR(ILITEK_IOCTL_BASE, 12, int)
#define ILITEK_IOCTL_START_READ_DATA            _IOWR(ILITEK_IOCTL_BASE, 13, int)
#define ILITEK_IOCTL_GET_INTERFANCE				_IOWR(ILITEK_IOCTL_BASE, 14, int)//default setting is i2c interface
#define ILITEK_IOCTL_I2C_SWITCH_IRQ				_IOWR(ILITEK_IOCTL_BASE, 15, int)
#define ILITEK_IOCTL_UPDATE_FLAG				_IOWR(ILITEK_IOCTL_BASE, 16, int)
#define ILITEK_IOCTL_I2C_UPDATE_FW				_IOWR(ILITEK_IOCTL_BASE, 18, int)




struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
};
typedef struct{
	u8 x_l;
	u8 x_h;
	u8 y_l;
	u8 y_h;
}xyz_data_t;



typedef struct{
	u8 key1_status:1,key2_status:1,
	   reserved:6;
	xyz_data_t xy_data[2];
}ilitek_report_data_t;

struct ilitek_i2c {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;

	struct i2c_client	*client;
	// maximum x
    int max_x;
    // maximum y
    int max_y;
	// maximum touch point
	int max_tp;
	// maximum key button
	int max_btn;
    // the total number of x channel
    int x_ch;
    // the total number of y channel
    int y_ch;
	// protocol version
	int protocol_ver;

	u16			x_res;
	u16			y_res;

	bool			pendown;
	int			irq;

	int			(*get_pendown_state)(void);
	void			(*clear_penirq)(void);
};

u16 old_x = 0;
u16 old_y = 0;

/*
description
	read data from i2c device with delay between cmd & return data
parameter
	client
		i2c client data
	addr
		i2c address
	data
		data for transmission
	length
		data length
return
	status
*/
static int ilitek_i2c_read_info(struct ilitek_i2c *tsc,uint8_t cmd, uint8_t *data, int length)
{
	int ret;
  ret = i2c_master_send(tsc->client, &cmd, 1);
	if(ret < 0){
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret);
	}
	
	msleep(10);
	ret = i2c_master_recv(tsc->client, data, length);
	if (ret <= 0) {
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret);	
		return ret;
	}
	return ret;
}


static int ilitek_i2c_read_tp_info(struct ilitek_i2c *tsc)
{

	int res_len;
	u8 buf[32]={0};         
	// read driver version
	printk(ILITEK_ERROR_LEVEL "%s, Driver Vesrion: %s\n", __func__, DRIVER_VERSION);
	// read firmware version
  if(ilitek_i2c_read_info(tsc, ILITEK_TP_CMD_GET_FIRMWARE_VERSION, buf, 4) < 0){
		return -1;
	}
	printk(ILITEK_DEBUG_LEVEL "%s, firmware version %d.%d.%d.%d\n", __func__, buf[0], buf[1], buf[2], buf[3]);

	// read protocol version
  res_len = 6;
  if(ilitek_i2c_read_info(tsc, ILITEK_TP_CMD_GET_PROTOCOL_VERSION, buf, 2) < 0){
		return -1;
	}	
	tsc->protocol_ver = (((int)buf[0]) << 8) + buf[1];
  printk(ILITEK_DEBUG_LEVEL "%s, protocol version: %d.%d\n", __func__, buf[0], buf[1]);
  if(tsc->protocol_ver >= 0x200){
     res_len = 8;
  }
  
  // read touch resolution
	tsc->max_tp = 2;
  if(ilitek_i2c_read_info(tsc, ILITEK_TP_CMD_GET_RESOLUTION, buf, res_len) < 0){
		return -1;
	}
  if(tsc->protocol_ver >= 0x200){
    // maximum touch point
    tsc->max_tp = buf[6];
    // maximum button number
    tsc->max_btn = buf[7];
  }
	// calculate the resolution for x and y direction
  tsc->max_x = buf[0];
  tsc->max_x+= ((int)buf[1]) * 256;
  tsc->max_y = buf[2];
  tsc->max_y+= ((int)buf[3]) * 256;
  tsc->x_ch = buf[4];
  tsc->y_ch = buf[5];
  printk(ILITEK_DEBUG_LEVEL "%s, max_x: %d, max_y: %d, ch_x: %d, ch_y: %d\n", 
		__func__, tsc->max_x, tsc->max_y, tsc->x_ch, tsc->y_ch);
  printk(ILITEK_DEBUG_LEVEL "%s, max_tp: %d, max_btn: %d\n", __func__, tsc->max_tp, tsc->max_btn);
	//printk("%s,get ok \n",__func__);
	
#if 0        
	buffer[0] = ILITEK_TP_CMD_GET_FIRMWARE_VERSION;
	ret = i2c_master_send(tsc->client, buffer, 1);
	if (ret < 0){
		printk("send resolution command error (%d)\n", ret);
		return ret;
	}
	ret = i2c_master_recv(tsc->client, buffer, 3);
	if (ret < 0){
		printk("read resolution error (%d)\n", ret);
		return ret;
	}else{
		res_x = ((buffer[0] & (X_RES_H_BMSK << X_RES_H_SHFT)) << 4) | buffer[1];
		res_y = ((buffer[0] & Y_RES_H_BMSK) << 8) | buffer[2];
		printk("resolution = %d x %d\n", res_x, res_y);
	}
#endif 
	
	return 0;
}

static void ilitek_i2c_send_up_event(struct ilitek_i2c *tsc)
{
	struct input_dev *input = tsc->input;

	dev_dbg(&tsc->client->dev, "UP\n");

	
	input_report_abs(input, ABS_PRESSURE, 0);
	//input_report_key(input, BTN_TOUCH, 0);
	input_sync(input);
}

static void ilitek_i2c_work(struct work_struct *work)
{
	struct ilitek_i2c *ts =
	container_of(to_delayed_work(work), struct ilitek_i2c, work);
	ilitek_report_data_t *pdata;
	u8  buf[9];
	int ret = 0;
	u32 tx,ty;
	
  struct input_dev *input = ts->input;
	struct ts_event tc;
	int rt = 0 ;
	//int i = 0;
	
  ret = ilitek_i2c_read_info(ts, ILITEK_TP_CMD_READ_DATA, buf, 9);
  if (ret <0){
  	  printk("pen error \n");
		  goto out;	
  }
  pdata=(ilitek_report_data_t * )buf;	
  
  if(pdata->key1_status){
  	  tx = (u32)(pdata->xy_data[0].x_h<<8) | pdata->xy_data[0].x_l;
			ty = (u32)(pdata->xy_data[0].y_h<<8) | pdata->xy_data[0].y_l;
  		tc.x=(u16)((tx*ts->x_res)/ts->max_x);
  		tc.y=(u16)((ty*ts->y_res)/ts->max_y);
  	if ((!ts->pendown)||(tc.x!=old_x )||(tc.y!=old_y)){
  		dev_dbg(&ts->client->dev, "DOWN\n");
  		ts->pendown = true;
  		input_report_abs(input, ABS_X, tc.x);
  		input_report_abs(input, ABS_Y, tc.y);
  		input_report_abs(input, ABS_PRESSURE, 1);
  		input_report_key(input, BTN_TOUCH, 1);
			input_sync(input);
  		//printk("point(%4d,%4d)\n",tc.x, tc.y);
  		
  		dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
			 tc.x, tc.y, rt); 
			old_x = tc.x;
    	old_y = tc.y;
			goto out;
  	}
  	
  	dev_dbg(&ts->client->dev, "pen is still down\n");
  		
	}else if(ts->pendown){
		  ilitek_i2c_send_up_event(ts);
		  ts->pendown = false;
		  
	}
 out:
	if (ts->pendown){
     dev_dbg(&ts->client->dev,"work\n");
		 schedule_delayed_work(&ts->work,
		 			msecs_to_jiffies(TS_POLL_PERIOD));
  }
	else{
		enable_irq(ts->irq);
    dev_dbg(&ts->client->dev,"stenable\n");
  }
}

static irqreturn_t ilitek_i2c_irq(int irq, void *handle)
{
	struct ilitek_i2c *ts = handle;
//printk("st1536_irq\n");
	if (!ts->get_pendown_state || likely(ts->get_pendown_state())) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}

	if (ts->clear_penirq)
		ts->clear_penirq();

	return IRQ_HANDLED;
}

static void ilitek_i2c_free_irq(struct ilitek_i2c *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int __devinit ilitek_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct ilitek_i2c *ts;
	struct ilitek_i2c_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;
	
	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;
  old_x = 0;
  old_y = 0;
	ts = kzalloc(sizeof(struct ilitek_i2c), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}
	
	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	INIT_DELAYED_WORK(&ts->work, ilitek_i2c_work);

	ts->x_res  = pdata->x_res;
	ts->y_res	 = pdata->y_res;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;
	ts->pendown = false;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));
		 
	i2c_set_clientdata(client, ts); 
	err = ilitek_i2c_read_tp_info(ts);
	if((err < 0))
		goto err_free_mem;
		 

	input_dev->name = "ilitek_i2c Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, 0, MAX_11BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_11BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();
  //printk("ilitek_i2c_probe03:%d\n",ts->irq);
	err = request_irq(ts->irq, ilitek_i2c_irq, 0,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}
	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;
  dev_dbg(&ts->client->dev,"ilitek_i2c_probe is ok\n");
  
	return 0;
err_free_irq:
	ilitek_i2c_free_irq(ts);
	if(pdata->exit_platform_hw)
		pdata->exit_platform_hw();
err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit ilitek_i2c_remove(struct i2c_client *client)
{
	struct ilitek_i2c	*ts = i2c_get_clientdata(client);
	struct ilitek_i2c_platform_data *pdata = client->dev.platform_data;

	ilitek_i2c_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id ilitek_i2c_idtable[] = {
	{ "ilitek_i2c", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ilitek_i2c_idtable);

static struct i2c_driver ilitek_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ilitek_i2c"
	},
	.id_table	= ilitek_i2c_idtable,
	.probe		= ilitek_i2c_probe,
	.remove		= __devexit_p(ilitek_i2c_remove),
};

static int __init ilitek_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ilitek_i2c_driver);
	return ret;
}

static void __exit ilitek_exit(void)
{
	//printk("%s,enter\n",__func__);
		i2c_del_driver(&ilitek_i2c_driver);
}

/* set init and exit function for this module */
module_init(ilitek_init);
module_exit(ilitek_exit);

MODULE_AUTHOR("tanshi li <cjli1@nuvoton.com>");
MODULE_DESCRIPTION("ilitek TouchScreen Driver");
MODULE_LICENSE("GPL");
