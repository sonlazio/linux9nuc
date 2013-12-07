/*
 * An I2C driver for the Holtek HT1382  RTC
 * Copyright 2012 Nuvoton Technology Corp.
 *
 * Author: Yi-An Chen <yachen@nuvoton.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#define DRV_VERSION "0.1"

#define HT1382_REG_SECONDS   0x00
#define HT1382_REG_MINUTES   0x01
#define HT1382_REG_HOURS     0x02
#define HT1382_REG_DATE      0x03
#define HT1382_REG_MONTH     0x04
#define HT1382_REG_DAY       0x05
#define HT1382_REG_YEAR      0x06
#define HT1382_REG_ST0       0x07
#define HT1382_REG_ST1       0x08
#define HT1382_REG_INT       0x09
#define HT1382_REG_SECONDS_A 0x0A
#define HT1382_REG_MINUTES_A 0x0B
#define HT1382_REG_HOURS_A   0x0C
#define HT1382_REG_DATE_A    0x0D
#define HT1382_REG_MONTH_A   0x0E
#define HT1382_REG_DAY_A     0x0F
#define HT1382_REG_DT        0x10
#define HT1382_REG_USR0      0x11
#define HT1382_REG_USR1      0x12
#define HT1382_REG_USR2      0x13
#define HT1382_REG_USR3      0x14


static struct i2c_driver ht1382_driver;

struct ht1382 {
	struct rtc_device *rtc;
        int     irq_num;
};
static int ht1382_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
        struct i2c_client *client = to_i2c_client(dev);        
        unsigned char buf[7] = { HT1382_REG_SECONDS };

        struct i2c_msg msgs[] = {
                { client->addr, 0, 1, buf },    /* setup read ptr */
                { client->addr, I2C_M_RD, 7, buf },     /* read status + date */
        };

        /* read registers */
        if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
                dev_err(&client->dev, "%s: read error\n", __func__);
                return -EIO;
        }

        dev_dbg(&client->dev,
                "%s: raw data is sec=%02x, min=%02x, hr=%02x, "
                "date=%02x, mon=%02x, day=%02x, year=%02x\n",
                __func__,
                buf[0], buf[1], buf[2], buf[3],
                buf[4], buf[5], buf[6]);


        tm->tm_sec = bcd2bin(buf[HT1382_REG_SECONDS] & 0x7F);
        tm->tm_min = bcd2bin(buf[HT1382_REG_MINUTES] & 0x7F);
        tm->tm_hour = bcd2bin(buf[HT1382_REG_HOURS] & 0x3F); /* rtc hr 0-23 */
        tm->tm_mday = bcd2bin(buf[HT1382_REG_DATE] & 0x3F);
        tm->tm_mon = bcd2bin(buf[HT1382_REG_MONTH] & 0x1F) - 1; /* rtc mn 1-12 */
        tm->tm_wday = bcd2bin(buf[HT1382_REG_DAY] & 0x7);       
        tm->tm_year = bcd2bin(buf[HT1382_REG_YEAR]) + 100;

        dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
                "mday=%d, mon=%d, year=%d, wday=%d\n",
                __func__,
                tm->tm_sec, tm->tm_min, tm->tm_hour,
                tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

        /* the clock can give out invalid datetime, but we cannot return
         * -EINVAL otherwise hwclock will refuse to set the time on bootup.
         */
        if (rtc_valid_tm(tm) < 0)
                dev_err(&client->dev, "retrieved date/time is not valid.\n");

        return 0;        
}

static int ht1382_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
        int i, err;
        unsigned char buf[7];
        unsigned char wp[2] = {HT1382_REG_ST0, 0};

        dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
                "mday=%d, mon=%d, year=%d, wday=%d\n",
                __func__,
                tm->tm_sec, tm->tm_min, tm->tm_hour,
                tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

        /* hours, minutes and seconds */
        buf[HT1382_REG_SECONDS] = bin2bcd(tm->tm_sec);
        buf[HT1382_REG_MINUTES] = bin2bcd(tm->tm_min);
        buf[HT1382_REG_HOURS] = bin2bcd(tm->tm_hour) | 0x80; // 24hr

        buf[HT1382_REG_DATE] = bin2bcd(tm->tm_mday);

        /* month, 1 - 12 */
        buf[HT1382_REG_MONTH] = bin2bcd(tm->tm_mon + 1);

        /* year and century */
        buf[HT1382_REG_YEAR] = bin2bcd(tm->tm_year % 100);

        buf[HT1382_REG_DAY] = tm->tm_wday & 0x07;
        
        /* write protect off */
        err = i2c_master_send(client, wp, sizeof(wp));
        if (err != sizeof(wp)) {
                dev_err(&client->dev,
                        "%s: err=%d addr=%02x, data=%02x\n",
                        __func__, err, wp[0], wp[1]);
                return -EIO;
        }        
        
        /* write register's data */
        for (i = 0; i < 7; i++) {
                unsigned char data[2] = { HT1382_REG_SECONDS + i,
                                                buf[i] };

                err = i2c_master_send(client, data, sizeof(data));
                if (err != sizeof(data)) {
                        dev_err(&client->dev,
                                "%s: err=%d addr=%02x, data=%02x\n",
                                __func__, err, data[0], data[1]);
                        return -EIO;
                }
        };
        
        /* write protect on */
        wp[1] = 0x80;
        err = i2c_master_send(client, wp, sizeof(wp));
        if (err != sizeof(wp)) {
                dev_err(&client->dev,
                        "%s: err=%d addr=%02x, data=%02x\n",
                        __func__, err, wp[0], wp[1]);
                return -EIO;
        }           

        return 0;        
}

#if 0

static int ht1382_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned char buf[6] = { HT1382_REG_SECONDS_A };
        unsigned char buf1[1] = { HT1382_REG_YEAR };        
        
        struct i2c_msg msgs[] = {
                { client->addr, 0, 1, buf },    /* setup read ptr */
                { client->addr, I2C_M_RD, 6, buf },     /* read status + date */
        };

        struct i2c_msg msgs1[] = {
                { client->addr, 0, 1, buf },    /* setup read ptr */
                { client->addr, I2C_M_RD, 1, buf },     /* read status + date */
        };
        /* read registers */
        if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
                dev_err(&client->dev, "%s: read error\n", __func__);
                return -EIO;
        }

        if ((i2c_transfer(client->adapter, msgs1, 2)) != 2) {
                dev_err(&client->dev, "%s: read error\n", __func__);
                return -EIO;
        }
        
        dev_dbg(&client->dev,
                "%s: raw data is sec=%02x, min=%02x, hr=%02x, "
                "date=%02x, mon=%02x, day=%02x, year=%02x\n",
                __func__,
                buf[0], buf[1], buf[2], buf[3],
                buf[4], buf[5], buf1[0]);


        alrm->time.tm_sec = bcd2bin(buf[0] & 0x7F);
        alrm->time.tm_min = bcd2bin(buf[1] & 0x7F);
        alrm->time.tm_hour = bcd2bin(buf[2] & 0x3F); /* rtc hr 0-23 */
        alrm->time.tm_mday = bcd2bin(buf[3] & 0x3F);
        alrm->time.tm_mon = bcd2bin(buf[4] & 0x1F) - 1; /* rtc mn 1-12 */
        alrm->time.tm_wday = bcd2bin(buf[5] & 0x7);       
        alrm->time.tm_year = bcd2bin(buf1[0]) + 100;
        
        dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
                "mday=%d, mon=%d, wday=%d year=%d\n",
                __func__,
                alrm->time.tm_sec, alrm->time.tm_min, alrm->time.tm_hour,
                alrm->time.tm_mday, alrm->time.tm_mon, alrm->time.tm_wday, alrm->time.tm_year);

        if (rtc_valid_tm(&alrm->time) < 0)
                dev_err(&client->dev, "retrieved date/time is not valid.\n");

        return 0;
}

static int ht1382_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{        
        struct i2c_client *client = to_i2c_client(dev);
        int i, err;
        unsigned char buf[6];
        unsigned char wp[2] = {HT1382_REG_ST0, 0};

        dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
                "mday=%d, mon=%d, wday=%d\n",
                __func__,
                alrm->time.tm_sec, alrm->time.tm_min, alrm->time.tm_hour,
                alrm->time.tm_mday, alrm->time.tm_mon, alrm->time.tm_wday); 

        buf[0] = bin2bcd(alrm->time.tm_sec) | 0x80;
        buf[1] = bin2bcd(alrm->time.tm_min) | 0x80;
        buf[2] = bin2bcd(alrm->time.tm_hour) | 0x80;
        buf[3] = bin2bcd(alrm->time.tm_mday) | 0x80;
        buf[4] = bin2bcd(alrm->time.tm_mon + 1) | 0x80;
        //buf[HT1382_REG_DAY] = alrm->time->tm_wday & 0x07;       
        
        /* write protect off */
        err = i2c_master_send(client, wp, sizeof(wp));
        if (err != sizeof(wp)) {
                dev_err(&client->dev,
                        "%s: err=%d addr=%02x, data=%02x\n",
                        __func__, err, wp[0], wp[1]);
                return -EIO;
        }        
        
        /* write register's data */
        for (i = 0; i < 7; i++) {
                unsigned char data[2] = { HT1382_REG_SECONDS_A + i,
                                                buf[i] };

                err = i2c_master_send(client, data, sizeof(data));
                if (err != sizeof(data)) {
                        dev_err(&client->dev,
                                "%s: err=%d addr=%02x, data=%02x\n",
                                __func__, err, data[0], data[1]);
                        return -EIO;
                }
        };
        
        /* write protect on */
        wp[1] = 0x80;
        err = i2c_master_send(client, wp, sizeof(wp));
        if (err != sizeof(wp)) {
                dev_err(&client->dev,
                        "%s: err=%d addr=%02x, data=%02x\n",
                        __func__, err, wp[0], wp[1]);
                return -EIO;
        }          
        
        return 0;
}

static int ht1382_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
        struct i2c_client *client = to_i2c_client(dev);
        int err;
        unsigned char buf[2] = {HT1382_REG_ST1, 0};       
        unsigned char buf1[2] = {HT1382_REG_ST0, 0};  
        
        struct i2c_msg msgs[] = {
                { client->addr, 0, 1, buf },    /* setup read ptr */
                { client->addr, I2C_M_RD, 6, buf },     /* read status + date */
        };       
        
        if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
                dev_err(&client->dev, "%s: read error\n", __func__);
                return -EIO;
        } 
        
        /* write protect off */
        err = i2c_master_send(client, buf1, sizeof(buf1));
        
        if (err != sizeof(buf1)) {
                dev_err(&client->dev,
                        "%s: err=%d addr=%02x, data=%02x\n",
                        __func__, err, buf1[0], buf1[1]);
                return -EIO;
        }         
        
        
        if(enabled) {
                // Set ARE & AE bit
                buf[0] |= 0x80;
                buf[1] |= 0x40;
                
        } else {
                // Clr ARE & AE bit
                buf[0] &= ~0x80;
                buf[1] &= ~0x40;                                
        }
        
        buf1[0] = HT1382_REG_ST1;
        buf1[1] = buf[0];
        err = i2c_master_send(client, buf1, sizeof(buf1));
        if (err != sizeof(buf1)) {
                dev_err(&client->dev,
                        "%s: err=%d addr=%02x, data=%02x\n",
                        __func__, err, buf1[0], buf1[1]);
                return -EIO;
        }    
        
        buf1[0] = HT1382_REG_INT;
        buf1[1] = buf[1];
        err = i2c_master_send(client, buf1, sizeof(buf1));
        if (err != sizeof(buf1)) {
                dev_err(&client->dev,
                        "%s: err=%d addr=%02x, data=%02x\n",
                        __func__, err, buf1[0], buf1[1]);
                return -EIO;
        }    
        
        /* write protect on */
        buf1[0] = HT1382_REG_ST0;
        buf1[1] = 0x80;
        err = i2c_master_send(client, buf1, sizeof(buf1));
        if (err != sizeof(buf1)) {
                dev_err(&client->dev,
                        "%s: err=%d addr=%02x, data=%02x\n",
                        __func__, err, buf1[0], buf1[1]);
                return -EIO;
        }          
        
        return 0;
        
}        
#endif

static const struct rtc_class_ops ht1382_rtc_ops = {
	.read_time	= ht1382_rtc_read_time,
	.set_time	= ht1382_rtc_set_time,
#if 0        
        .read_alarm     = ht1382_rtc_read_alarm,
        .set_alarm      = ht1382_rtc_set_alarm,
        .alarm_irq_enable = ht1382_rtc_alarm_irq_enable,
#endif        
};

static int ht1382_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct ht1382 *ht1382;
        int err = 0;        
        unsigned char buf[2] = {HT1382_REG_HOURS};
        struct i2c_msg msgs[] = {
                { client->addr, 0, 1, buf },    /* setup read ptr */
                { client->addr, I2C_M_RD, 1, buf },     /* read status + date */
        };        


	dev_dbg(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ht1382 = kzalloc(sizeof(struct ht1382), GFP_KERNEL);
	if (!ht1382)
		return -ENOMEM;

	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	i2c_set_clientdata(client, ht1382);

	ht1382->rtc = rtc_device_register(ht1382_driver.driver.name,
				&client->dev, &ht1382_rtc_ops, THIS_MODULE);

	if (IS_ERR(ht1382->rtc)) {
		err = PTR_ERR(ht1382->rtc);
		goto exit_kfree;
	}

        /* configure to 24hr mode if current is working in 12hr mode */
        
        if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
                dev_err(&client->dev, "%s: read error\n", __func__);
                err = -EIO;
                goto exit_kfree;                
        }
        
        if((buf[0] & 0x80) == 0) {
        
                /* write protect off */
                buf[0] = HT1382_REG_ST0;
                buf[1] = 0;
                err = i2c_master_send(client, buf, sizeof(buf));
                if (err != sizeof(buf)) {
                        dev_err(&client->dev,
                                "%s: err=%d addr=%02x, data=%02x\n",
                                __func__, err, buf[0], buf[1]);
                        err = -EIO;
                        goto exit_kfree;   
                }  
                
                buf[0] = HT1382_REG_HOURS;
                // Don't care about the correct time. If it's in 12hr mode, it hasn't been configured yet        
                buf[1] = 0x80;  
                err = i2c_master_send(client, buf, sizeof(buf));
                if (err != sizeof(buf)) {
                        dev_err(&client->dev,
                                "%s: err=%d addr=%02x, data=%02x\n",
                                __func__, err, buf[0], buf[1]);
                        err = -EIO;
                        goto exit_kfree; 
                } 
        
                /* write protect on */
                buf[0] = HT1382_REG_ST0;
                buf[1] = 0x80;
                err = i2c_master_send(client, buf, sizeof(buf));
                if (err != sizeof(buf)) {
                        dev_err(&client->dev,
                                "%s: err=%d addr=%02x, data=%02x\n",
                                __func__, err, buf[0], buf[1]);
                        err = -EIO;
                        goto exit_kfree; 
                }  
        }
	return 0;

exit_kfree:
	kfree(ht1382);

	return err;
}

static int ht1382_remove(struct i2c_client *client)
{
	struct ht1382 *ht1382 = i2c_get_clientdata(client);

	if (ht1382->rtc)
		rtc_device_unregister(ht1382->rtc);

	kfree(ht1382);

	return 0;
}

static const struct i2c_device_id ht1382_id[] = {
	{ "ht1382", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ht1382_id);

static struct i2c_driver ht1382_driver = {
	.driver		= {
		.name	= "rtc-ht1382",
	},
	.probe		= ht1382_probe,
	.remove		= ht1382_remove,
	.id_table	= ht1382_id,
};

static int __init ht1382_init(void)
{
	return i2c_add_driver(&ht1382_driver);
}

static void __exit ht1382_exit(void)
{
	i2c_del_driver(&ht1382_driver);
}

MODULE_AUTHOR("Yi-An Chen <yachen@nuvoton.com>");
MODULE_DESCRIPTION("HT1382 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(ht1382_init);
module_exit(ht1382_exit);
