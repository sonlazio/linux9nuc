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
#include <linux/i2c/st1536.h>

#define TS_POLL_DELAY			0 /* ms delay between samples */
#define TS_POLL_PERIOD			14 /* ms delay between samples */

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


struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
};

struct st1536 {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;

	struct i2c_client	*client;

	u16			x_res;
	u16			y_res;

	bool			pendown;
	int			irq;

	int			(*get_pendown_state)(void);
	void			(*clear_penirq)(void);
};

u16 old_x = 0;
u16 old_y = 0;

static inline int st1536_get_fw_version(struct st1536 *tsc, u32 *ver)
{
	u8  buf[1];
	int ret;

	buf[0] = FW_REG;
	
	ret = i2c_master_send(tsc->client, buf, sizeof(buf));
	if (ret <= 0) {
		dev_err(&tsc->client->dev, "i2c send error: %d\n", buf[0]);
		return ret;
	}
	ret = i2c_master_recv(tsc->client, buf, sizeof(buf));
	if (ret <= 0) {
		dev_err(&tsc->client->dev, "i2c rcv error: %d\n", ret);
		return ret;
	}
	*ver = (u32)buf[0];

        printk("1563_ver(%d)",*ver);
	
	buf[0] = XY_COORDINATE_REG;
	
	ret = i2c_master_send(tsc->client, buf, sizeof(buf));
	if (ret <= 0) {
		dev_err(&tsc->client->dev, "i2c send error: %d\n", buf[0]);
		return ret;
	}
	
	return ret;
}

static int sitronix_ts_get_device_status(struct st1536 *tsc, u8 *dev_status)
{
	int ret = 0;
	u8 buf[1];
         
  buf[0] = STATUS_REG; 

//	DbgMsg("%s\n", __FUNCTION__);

        ret = i2c_master_send(tsc->client, buf, sizeof(buf));
	if (ret <= 0) {
		dev_err(&tsc->client->dev, "i2c send error: %d\n", buf[0]);
		return ret;
	}
	ret = i2c_master_recv(tsc->client, buf, sizeof(buf));
	if (ret <= 0) {
		dev_err(&tsc->client->dev, "i2c rcv error: %d\n", ret);
		return ret;
	}
        printk("status(%d)",buf[0]);

	*dev_status = buf[0] & 0xf;

	return 0;
}
static int sitronix_get_resolution(struct st1536 *tsc)
{
	int ret = 0;
	u8 buffer[3];
        int res_x;
        int res_y;
	buffer[0] = RESOLUTION_REG;
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
	return 0;
}





static inline int st1536_set_power_down(struct st1536 *tsc)
{
	u8  buf[2];
	int ret;

	buf[0] = CTRL_REG;
	buf[1] = 0xA;	//keep Gesture bit and set PD bit to enter Power Down
	
	ret = i2c_master_send(tsc->client, buf, sizeof(buf));
	if (ret <= 0) {
		dev_err(&tsc->client->dev, "i2c send error: %d\n", buf[0]);
		return ret;
	}

	return ret;
}

static inline int st1536_set_resolution(struct st1536 *tsc, u16 x_res, u16 y_res)
{
	u8  buf[4];
	int ret;

	buf[0] = RESOLUTION_REG;
	buf[1] = ((x_res & 0xF00)>>4)|((y_res&0xF00)>>4); //set x_res_h and y_res_h
	buf[2] = x_res & 0xFF;//set x_res_l
	buf[3] = y_res & 0xFF;//set y_res_l
	
	ret = i2c_master_send(tsc->client, buf, sizeof(buf));
	if(ret <= 0) {
		dev_err(&tsc->client->dev, "i2c send error: %d\n", buf[0]);
		return ret;
	}

	buf[0] = XY_COORDINATE_REG;
	ret = i2c_master_send(tsc->client, buf, 1);
	if (ret <= 0) {
		dev_err(&tsc->client->dev, "i2c send error: %d\n", buf[0]);
		return ret;
	}

	return ret;
}


static int st1536_get_coordinates(struct st1536 *tsc, struct ts_event *tc)
{
	u8  buf[42];
	int ret = 0;
	stx_report_data_t *pdata;
	
  buf[0] = 0x10;
  ret = i2c_master_send(tsc->client, buf,1);
	if (ret <= 0) {
		
		dev_err(&tsc->client->dev, "i2c send error: %d,errornum=%d\n", buf[0],ret);
		return ret;
	}

	ret = i2c_master_recv(tsc->client, buf, 22);
	if (ret <= 0) {
		dev_err(&tsc->client->dev, "i2c rcv error: %d\n", ret);
		return ret;
	}
	pdata=(stx_report_data_t*)buf;
	
  //printk("finger(%d)\n",pdata->fingers);
  if(pdata->fingers){
  	//ret=pdata->xy_data[0].valid;
  	//printk("valid=%d\n",pdata->xy_data[0].valid);
    if(pdata->xy_data[0].valid){
			tc->x = (u16)(pdata->xy_data[0].x_h<<8) | pdata->xy_data[0].x_l;
			tc->y = (u16)(pdata->xy_data[0].y_h<<8) | pdata->xy_data[0].y_l;
      // printk("ts(%d)(%d)",tc->x,tc->y); 
      ret=1;          
		}
		else
			ret=0;
    //printk("out");
	}
  else{
    ret = 0;
  }
  
	return ret;
}

static void st1536_send_up_event(struct st1536 *tsc)
{
	struct input_dev *input = tsc->input;

	dev_dbg(&tsc->client->dev, "UP\n");

	
	input_report_abs(input, ABS_PRESSURE, 0);
	input_report_key(input, BTN_TOUCH, 0);
	input_sync(input);
}

static void st1536_work(struct work_struct *work)
{
	struct st1536 *ts =
	container_of(to_delayed_work(work), struct st1536, work);
  struct input_dev *input = ts->input;
	struct ts_event tc;
	int rt = 0 ;
	int i = 0;
  //u8 flash
  tc.x=801;
  tc.y=481;
  rt = st1536_get_coordinates(ts, &tc); 
  if (rt<0){
  		ts->pendown = false;
  		//printk("pen error \n");
		  goto out;
  }
  if(rt==0){//the pen up state
  	if(ts->pendown){
  	  st1536_send_up_event(ts);
		  ts->pendown = false;
		  //printk("pen up\n");
		} 
		goto out;
  }
  dev_dbg(&ts->client->dev, "pen is still down\n");


  
  if (!ts->pendown) {
			dev_dbg(&ts->client->dev, "DOWN\n");		
			ts->pendown = true;
	}
 	input_report_abs(input, ABS_X, tc.x);
  input_report_abs(input, ABS_Y, tc.y);
  input_report_abs(input, ABS_PRESSURE, 1);
  input_report_key(input, BTN_TOUCH, 1);
	input_sync(input);
  //printk("point(%4d,%4d), pressure (%4u)\n",tc.x, tc.y, rt);
  dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
			 tc.x, tc.y, rt); 
#if 0
  if(((old_x == 0)&&(old_y == 0))||((old_x != tc.x)&&(old_y != tc.y))){
            if((( tc.x >= 0) && (tc.x <= 800 ))&&((tc.y >=0)&&(tc.y <= 480))){
                  old_x = tc.x;
                  old_y = tc.y;
		              struct input_dev *input = ts->input;
		              if (!ts->pendown) {
			                dev_dbg(&ts->client->dev, "DOWN\n");
			                input_report_key(input, BTN_TOUCH, 1);
			                ts->pendown = true;
		              }
		              input_report_abs(input, ABS_X, tc.x);
		              input_report_abs(input, ABS_Y, tc.y);
		              input_report_abs(input, ABS_PRESSURE, 1);
		              input_sync(input);
                  //printk("point(%4d,%4d), pressure (%4u)\n",tc.x, tc.y, rt);
		              dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
			                  tc.x, tc.y, rt);
            }
            // mdelay(100);
            //i = ts->get_pendown_stat();
        }
	   } else if(ts->pendown){// (!ts->get_pendown_state) && ts->pendown) {
		/*
		 * We don't have callback to check pendown state, so we
		 * have to assume that since pressure reported is 0 the
		 * pen was lifted up.
		 */
		     st1536_send_up_event(ts);
		     ts->pendown = false;
         //printk("pendown\n");
	   }
#endif 
 out:
	if (ts->pendown){
     // printk("work\n");
		 schedule_delayed_work(&ts->work,
		 			msecs_to_jiffies(TS_POLL_PERIOD));
  }
	else{
		enable_irq(ts->irq);
    // printk("stenable\n");
  }
}

static irqreturn_t st1536_irq(int irq, void *handle)
{
	struct st1536 *ts = handle;
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

static void st1536_free_irq(struct st1536 *ts)
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

static int __devinit st1536_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct st1536 *ts;
	struct st1536_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;
	uint8_t dev_status = 0;
	
  //    u16  tempx;
  //    u16  tempy;
 // struct ts_event tc;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;
  old_x = 0;
  old_y = 0;
	ts = kzalloc(sizeof(struct st1536), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}
	
	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	INIT_DELAYED_WORK(&ts->work, st1536_work);

	ts->x_res  = pdata->x_res;
	ts->y_res	 = pdata->y_res;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));
		 
	i2c_set_clientdata(client, ts); 
	err = sitronix_ts_get_device_status(ts, &dev_status);
	if((err < 0) || (dev_status == 0x6))
		goto err_free_mem;
		 

	input_dev->name = "ST1536 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
  //printk("st1536_probe02\n");
	input_set_abs_params(input_dev, ABS_X, 0, MAX_11BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_11BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();
  //printk("st1536_probe03:%d\n",ts->irq);
	err = request_irq(ts->irq, st1536_irq, 0,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}
  //st1536_get_fw_version(ts,&tempver);
  //sitronix_ts_get_device_status(ts,tempsts);
       
  //printk("st1536_probe04\n");
	/* Prepare for touch readings - power down ADC and enable PENIRQ */
	//err = st1536_set_resolution(ts, ts->x_res, ts->y_res);
	//if (err < 0)
	//	goto err_free_irq;
  //printk("st1536_probe05\n");

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

  sitronix_get_resolution(ts);
  
	return 0;
err_free_irq:
	st1536_free_irq(ts);
	if(pdata->exit_platform_hw)
		pdata->exit_platform_hw();
err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit st1536_remove(struct i2c_client *client)
{
	struct st1536	*ts = i2c_get_clientdata(client);
	struct st1536_platform_data *pdata = client->dev.platform_data;

	st1536_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id st1536_idtable[] = {
	{ "st1536", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, st1536_idtable);

static struct i2c_driver st1536_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "st1536"
	},
	.id_table	= st1536_idtable,
	.probe		= st1536_probe,
	.remove		= __devexit_p(st1536_remove),
};

static int __init st1536_init(void)
{
	return i2c_add_driver(&st1536_driver);
}

static void __exit st1536_exit(void)
{
	i2c_del_driver(&st1536_driver);
}

module_init(st1536_init);
module_exit(st1536_exit);

MODULE_AUTHOR("shirley <clyu2@nuvoton.com>");
MODULE_DESCRIPTION("ST1536 TouchScreen Driver");
MODULE_LICENSE("GPL");
