/* 
/* drivers/hwmon/mt6516/amit/cm36283.c - CM36283 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "cm36283.h"
#ifdef MT6575
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif

/******************************************************************************
 * configuration
*******************************************************************************/
#define I2C_DRIVERID_CM36283 36283
/*----------------------------------------------------------------------------*/

#define CM36283_DEV_NAME     "CM36283"


/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_FUN1(f)               printk(KERN_INFO APS_TAG"%s -%d\n", __FUNCTION__,f)
#define APS_FUN2(a,b)               printk(KERN_INFO APS_TAG"%s -%d  para=%d\n", __FUNCTION__,a,b)
#define APS_ERR(fmt, args...)    printk(KERN_ERR APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO fmt, ##args)                 


#define ALSPS_DBG(trc_mask,fmt,args...)     \
    {   \
        if(atomic_read(&obj->trace) & (trc_mask))   \
        {   \
            printk(KERN_INFO APS_TAG fmt, ##args);  \
        }   \
    }   \

/*----------------------------------------------------------------------------*/
 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
	int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={{0,0,0},};

static struct i2c_client *cm36283_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id cm36283_i2c_id[] = {{CM36283_DEV_NAME,0},{}};

static struct i2c_board_info __initdata i2c_cm36283={ I2C_BOARD_INFO("CM36283", (0x60))};

/*the adapter id & i2c address will be available in customization*/
//static unsigned short cm36283_force[] = {0x00, 0x00, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const cm36283_forces[] = { cm36283_force, NULL };
//static struct i2c_client_address_data cm36283_addr_data = { .forces = cm36283_forces,};
/*----------------------------------------------------------------------------*/
static int cm36283_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int cm36283_i2c_remove(struct i2c_client *client);
static int cm36283_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int cm36283_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int cm36283_i2c_resume(struct i2c_client *client);

static struct cm36283_priv *g_cm36283_ptr = NULL;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_TRC_ALS_DATA= 0x0001,
    CMC_TRC_PS_DATA = 0x0002,
    CMC_TRC_EINT    = 0x0004,
    CMC_TRC_IOCTL   = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS = 0x0020,
    CMC_TRC_CVT_PS  = 0x0040,
    CMC_TRC_DBG_PS  = 0x0080,
    CMC_TRC_DBG_ALS = 0x0100,
    CMC_TRC_DEBUG   = 0x8000,
} CMC_TRC;
/*----------------------------------------------------------------------------*/
/*mark, to check*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
/*mark, to check*/
struct cm36283_i2c_addr {    /*define a series of i2c slave address*/
    u8  rar;        /*Alert Response Address*/
    u8  init;       /*device initialization */
    u8  als_cmd;    /*ALS command*/
    u8  als_dat1;   /*ALS MSB*/
    u8  als_dat0;   /*ALS LSB*/
    u8  ps_cmd;     /*PS command*/
    u8  ps_dat;     /*PS data*/
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct cm36283_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;

    /*i2c address group*/
    struct cm36283_i2c_addr  addr;
    
    /*misc*/
    atomic_t    trace;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u8          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
#if 0
/*mark, to check*/
    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
#else
    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val_2;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val_3;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_canc_val;     /*the cmd value can't be read, stored in ram*/
#endif	
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver cm36283_i2c_driver = {	
	.probe      = cm36283_i2c_probe,
	.remove     = cm36283_i2c_remove,
	//.detect     = cm36283_i2c_detect,//modified
	.suspend    = cm36283_i2c_suspend,
	.resume     = cm36283_i2c_resume,
	.id_table   = cm36283_i2c_id,
	//.address_data = &cm36283_addr_data,
	.driver = {
		//.owner          = THIS_MODULE,
		.name           = CM36283_DEV_NAME,
	},
};

static int ps_raw_data=0 ;
static int als_raw_data=0;
static int val_temp=0;
static struct cm36283_priv *cm36283_obj = NULL;
static struct platform_driver cm36283_alsps_driver;
static int cm36283_get_ps_value(struct cm36283_priv *obj, u16 ps);
static int cm36283_get_als_value(struct cm36283_priv *obj, u16 als);
static int cm36283_read_als(struct i2c_client *client, u16 *data);
static int cm36283_read_ps(struct i2c_client *client, u8 *data);
/*----------------------------------------------------------------------------*/
int cm36283_get_addr(struct alsps_hw *hw, struct cm36283_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	
	addr->rar       = hw->i2c_addr[0];
	addr->init      = hw->i2c_addr[0];
	addr->als_cmd   = hw->i2c_addr[0];
	addr->als_dat1  = hw->i2c_addr[0];
	addr->als_dat0  = hw->i2c_addr[0];
	addr->ps_cmd    = hw->i2c_addr[0];
	addr->ps_thd    = hw->i2c_addr[0];
	addr->ps_dat    = hw->i2c_addr[0];

	return 0;
}


/*----------------------------------------------------------------------------*/
/*
int cm36283_config_timing(int sample_div, int step_div)
{
	u32 base = I2C2_BASE; 
	unsigned long tmp;

	tmp  = __raw_readw(mt6516_I2C_TIMING) & ~((0x7 << 8) | (0x1f << 0));
	tmp  = (sample_div & 0x7) << 8 | (step_div & 0x1f) << 0 | tmp;

	return (__raw_readw(mt6516_I2C_HS) << 16) | (tmp);
}
*/
/*----------------------------------------------------------------------------*/
int cm36283_master_recv(struct i2c_client *client, u16 addr, char *buf ,int count)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);        
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret = 0, retry = 0;
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);
	
	//addr = (addr >> 1);//modified
	//APS_LOG("address : 0x%02x\n", addr); //modified

//	msg.addr = addr | I2C_A_FILTER_MSG | I2C_A_CHANGE_TIMING;
	msg.addr = addr | I2C_A_FILTER_MSG;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;
//	msg.timing = cm36283_config_timing(0, 41);
	msg.timing = 200;

	while(retry++ < max_try)
	{
		ret = i2c_transfer(adap, &msg, 1);
		if(ret == 1)
		{
			break;
		}

		udelay(100);
	}

	if(unlikely(trc))
	{
		if(trc & CMC_TRC_I2C)
		{
			APS_LOG("(recv) %x %d %d %p [%02X]\n", msg.addr, msg.flags, msg.len, msg.buf, msg.buf[0]);    
		}

		if((retry != 1) && (trc & CMC_TRC_DEBUG))
		{
			APS_LOG("(recv) %d/%d\n", retry-1, max_try); 

		}
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 1) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int cm36283_master_send(struct i2c_client *client, u16 addr, char *buf ,int count)
{
	int ret = 0, retry = 0;
	struct cm36283_priv *obj = i2c_get_clientdata(client);        
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);

#if defined(TPD_PS_SUPPORT)
	return count;
#endif

	//addr = (addr >> 1);//modified
	//APS_LOG("address : 0x%02x\n", addr);//modified

//	msg.addr = addr | I2C_A_FILTER_MSG | I2C_A_CHANGE_TIMING;
	msg.addr = addr | I2C_A_FILTER_MSG;
	msg.flags = client->flags & I2C_M_TEN;	
	msg.len = count;
	msg.buf = (char *)buf;
//	msg.timing = cm36283_config_timing(0, 41);
	msg.timing = 200;
	
	while(retry++ < max_try)
	{
		ret = i2c_transfer(adap, &msg, 1);
		if (ret == 1)
		break;
		udelay(100);
	}

	if(unlikely(trc))
	{
		if(trc & CMC_TRC_I2C)
		{
			APS_LOG("(send) %x %d %d %p [%02X]\n", msg.addr, msg.flags, msg.len, msg.buf, msg.buf[0]);    
		}

		if((retry != 1) && (trc & CMC_TRC_DEBUG))
		{
			APS_LOG("(send) %d/%d\n", retry-1, max_try);
		}
	}
	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

int cm36283_read(struct i2c_client *client,u16 addr, u8 reg, char *buf ,u16 count)
{
	int i;
	int ret;
	struct cm36283_priv *obj = i2c_get_clientdata(client);        
	int max_try = atomic_read(&obj->i2c_retry);
	unsigned short tmp;
#if defined(TPD_PS_SUPPORT)
	return count;
#endif
	buf[0] = (char)reg;

	tmp = client->addr;
 	client->timing=200;
    client->addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG |I2C_RS_FLAG;
	

	for (i = 0; i < max_try; i++) {
		if (i2c_master_send(client, (const char*)buf, count<<8 | 0x01)) {
			break;
		}
		udelay(100);
	}

	if(i>=max_try){
		ret = -EIO;
	}
	else{
		ret = count;
	}
	ALSPS_DBG(CMC_TRC_I2C,"%s -1,i2c send ret=%d,max_try=%d",__FUNCTION__,ret,max_try);
	client->addr = tmp;//i2c_client->addr& I2C_MASK_FLAG;
	return ret;
}

/*----------------------------------------------------------------------------*/
int cm36283_read_als(struct i2c_client *client, u16 *data)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2];

	if(2 != (ret = cm36283_read(client, obj->addr.als_dat1,0x09,(char*)&buf[0], 2)))
	{
		APS_ERR("reads als data,ret = %d\n", ret);
		return -EFAULT;
	}
	
	*data = (buf[1] << 8) | (buf[0]);
	if(atomic_read(&obj->trace) & CMC_TRC_ALS_DATA)
	{
		APS_DBG("ALS: 0x%04X\n", (u32)(*data));
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int cm36283_write_als(struct i2c_client *client, u8 cmd)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);
	u8 buf[3];
	int ret = 0;

	buf[0] = 0x00;/*reg*/
	buf[1] = cmd;
	buf[2] = 0;/*reserve*/
	if(sizeof(buf) != (ret = cm36283_master_send(client, obj->addr.als_cmd, (char*)&buf, sizeof(buf))))
	{
		APS_ERR("write als = %d\n", ret);  // -121
		return -EFAULT;
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int cm36283_read_ps(struct i2c_client *client, u8 *data)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2];

	if(sizeof(buf) != (ret = cm36283_read(client, obj->addr.ps_dat,0x08,(char*)buf, sizeof(buf))))
	{
		APS_ERR("reads ps data failed ,ret = %d\n", ret);
		return -EFAULT;
	} 

	*data = buf[0];
	if(atomic_read(&obj->trace) & CMC_TRC_PS_DATA)
	{
		APS_DBG("PS:  0x%04X\n", (u32)(*data));
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
int cm36283_write_ps(struct i2c_client *client, u8 cmd)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);        
	u8 buf[3];
	int ret = 0;

	buf[0] = 0x03;/*reg*/
	buf[1] = cmd;
	buf[2] = 0x03;/*reserve*/
	if(sizeof(buf) != (ret = cm36283_master_send(client, obj->addr.ps_cmd, (char*)&buf, sizeof(buf))))
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	} 

	buf[0] = 0x04;/*reg*/
	buf[1] = 0x00;
	buf[2] = 0x00;/*reserve*/
	if(sizeof(buf) != (ret = cm36283_master_send(client, obj->addr.ps_cmd, (char*)&buf, sizeof(buf))))
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
int cm36283_write_ps_thd(struct i2c_client *client, u8 thd_low, u8 thd_high)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);        
	u8 buf[3];
	int ret = 0;
	
	 if (thd_high<3)
	{
		APS_ERR("%s -1, invalid thd = %d\n", __FUNCTION__,thd_high);
		return -1;
	}

	buf[0] = 0x06;/*reg*/
	if(ps_cali.valid==1)
    {
	     buf[1]=ps_cali.far_away;
         buf[2]=ps_cali.close;
    }
    else
	{
	
         buf[1] = thd_low;
	     buf[2] = thd_high;
	}

	if(sizeof(buf) != (ret = cm36283_master_send(client, obj->addr.ps_thd, (char*)&buf, sizeof(buf))))
	{
		APS_ERR("write thd = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int cm36283_init_device(struct i2c_client *client)
{
#if 1//def DEBUG_CM36283
	return 0;
#else
	struct cm36283_priv *obj = i2c_get_clientdata(client);        
	u8 buf[] = {0x10};
	int ret = 0;
	
	if(sizeof(buf) != (ret = cm36283_master_send(client, obj->addr.init, (char*)&buf, sizeof(buf))))
	{
		APS_ERR("init = %d\n", ret);
		return -EFAULT;
	} 
	return 0;
#endif
}
/*----------------------------------------------------------------------------*/
int cm36283_read_rar(struct i2c_client *client, u8 *data)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;
	u8 buf[2];

	if(sizeof(buf) != (ret = cm36283_read(client, obj->addr.rar,0x0b,(char*)buf, sizeof(buf))))
	{
		APS_ERR("rar = %d\n", ret);
		return -EFAULT;
	}
	*data = buf[1];	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void cm36283_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "CM36283")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "CM36283")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}


#if !defined(SIMCOM_DA18)
	if(power_on == on)
	{
		APS_LOG("ignore power control: %d\n", on);
	}
	else if(on)
	{
	#if defined(SIMCOM_I715M)
		hwPowerOn(MT65XX_POWER_LDO_VGP,VOL_2800,"TP"); 
			msleep(50);
	#endif
            mt_set_gpio_mode(GPIO_ALS_LDOEN_PIN, GPIO_ALS_LDOEN_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_ALS_LDOEN_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_ALS_LDOEN_PIN, GPIO_OUT_ONE);
			msleep(80);
	}
	else
	{
            mt_set_gpio_mode(GPIO_ALS_LDOEN_PIN, GPIO_ALS_LDOEN_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_ALS_LDOEN_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_ALS_LDOEN_PIN, GPIO_OUT_ZERO);
	}
#endif

	power_on = on;
}
/*----------------------------------------------------------------------------*/
static int cm36283_enable_als(struct i2c_client *client, int enable)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->als_cmd_val);
	int trc = atomic_read(&obj->trace);

	if(enable)
	{
		cur = old & (~SD_ALS);   
	}
	else
	{
		cur = old | (SD_ALS); 
	}
	
	if(trc & CMC_TRC_DEBUG)
	{
		APS_LOG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}
	
	if(0 == (cur ^ old))
	{
		return 0;
	}
	
	if(0 == (err = cm36283_write_als(client, cur))) 
	{
		atomic_set(&obj->als_cmd_val, cur);
	}
	
	if(enable)
	{
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));

		obj->hw = get_cust_alsps_hw();
		if (0 == obj->hw->polling_mode_als)
		{
			set_bit(CMC_BIT_ALS,  &obj->pending_intr);
			schedule_delayed_work(&obj->eint_work,260); //after enable the value is not accurate
		}
	}

	if(trc & CMC_TRC_DEBUG)
	{
		APS_LOG("enable als (%d)\n", enable);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int cm36283_enable_ps(struct i2c_client *client, int enable)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->ps_cmd_val);
	int trc = atomic_read(&obj->trace);

	if(enable)
	{
		cur = old & (~SD_PS);   
	}
	else
	{
		cur = old | (SD_PS);
	}
	
	if(trc & CMC_TRC_DEBUG)
	{
		APS_LOG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}
	
	if(0 == (cur ^ old))
	{
		return 0;
	}
	
	if(0 == (err = cm36283_write_ps(client, cur))) 
	{
		atomic_set(&obj->ps_cmd_val, cur);
	}
	
	if(enable)
	{
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		
		obj->hw = get_cust_alsps_hw();
		if (0 == obj->hw->polling_mode_ps)
		{
			set_bit(CMC_BIT_PS,  &obj->pending_intr);
			schedule_delayed_work(&obj->eint_work,120);
		}
	}

	if(trc & CMC_TRC_DEBUG)
	{
		APS_LOG("enable ps  (%d)\n", enable);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int cm36283_check_and_clear_intr(struct i2c_client *client) 
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);    

#if 1//def DEBUG_CM36283
	ALSPS_DBG(CMC_TRC_DBG_PS,"%s -1,nothing to do for cm36283",__FUNCTION__);
	return 0;
#else
	struct cm36283_priv *obj = i2c_get_clientdata(client);
	int err;
	u8 addr;
	
	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
		return 0;

	if((err = cm36283_read_rar(client, &addr)))
	{
		APS_ERR("WARNING: read rar: %d\n", err);
		return 0;
	}
    
	if(addr == 0x90)//need repaired?
	{
		set_bit(CMC_BIT_ALS, &obj->pending_intr);
	}
	else
	{
	   	clear_bit(CMC_BIT_ALS, &obj->pending_intr);
	}
	
	if(addr == 0xf0)//need repaired?
	{
		set_bit(CMC_BIT_PS,  &obj->pending_intr);
	}
	else
	{
	    clear_bit(CMC_BIT_PS, &obj->pending_intr);
	}
	
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG)
	{
		APS_LOG("check intr: 0x%02X => 0x%08lX\n", addr, obj->pending_intr);
	}

	return 0;
#endif
}
int cm36283_read_eint_state(struct i2c_client *client, u8 *data)
{
//longxuewei
	struct cm36283_priv *obj = i2c_get_clientdata(client);        
	int res = 0;
	u8 buf[2];


	if(sizeof(buf) != (res = cm36283_read(client, obj->addr.rar,0x0b,(char*)buf, sizeof(buf))))
	{
		APS_ERR("rar = %d\n", res);
		return -EFAULT;
	}

/*
	buf[0]=0x0B ;
	res = i2c_master_send(client, buf, 0x1);
	if(res <= 0)
	{
		return -EFAULT;
	}
	if(sizeof(buf) !=(res = i2c_master_recv(client, (char*)buf, sizeof(buf))));
	{
		APS_ERR("reads eint_state failed ,ret = %d\n", res);
		return -EFAULT;
	}
	*/
	//*data = buf[0];	
        *data = buf[1];	
	return 0;
}
/*----------------------------------------------------------------------------*/
void cm36283_eint_func(void)
{
	struct cm36283_priv *obj = g_cm36283_ptr;
	APS_LOG(" interrupt fuc\n");
	if(!obj)
	{
		return;
	}
	
	//schedule_work(&obj->eint_work);
	schedule_delayed_work(&obj->eint_work,0);
	if(atomic_read(&obj->trace) & CMC_TRC_EINT)
	{
		APS_LOG("eint: als/ps intrs\n");
	}
}
/*----------------------------------------------------------------------------*/
static void cm36283_eint_work(struct work_struct *work)
{
	struct cm36283_priv *obj = g_cm36283_ptr;
	int err;
        u8 data=0;
	hwm_sensor_data sensor_data;
	
	memset(&sensor_data, 0, sizeof(sensor_data));

	APS_LOG(" eint work\n");

        cm36283_read_eint_state(obj->client,&data); //read to clear int

	//APS_LOG("cm36283_read_eint_state:data %x \n", data);
	if((err = cm36283_check_and_clear_intr(obj->client)))
	{
		APS_ERR("check intrs: %d\n", err);
	}

    APS_LOG(" &obj->pending_intr =%lx\n",obj->pending_intr);
	
	if((1<<CMC_BIT_ALS) & obj->pending_intr)
	{
	  //get raw data
	  APS_LOG(" als change\n");
	  if((err = cm36283_read_als(obj->client, &obj->als)))
	  {
		 APS_ERR("cm36283 read als data: %d\n", err);;
	  }
	  //map and store data to hwm_sensor_data
	 
 	  sensor_data.values[0] = cm36283_get_als_value(obj, obj->als);
	  sensor_data.value_divide = 1;
	  sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	  APS_LOG("als raw %x -> value %d \n", obj->als,sensor_data.values[0]);
	  //let up layer to know
	  if((err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)))
	  {
		APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	  }
	  
	}
	if((1<<CMC_BIT_PS) &  obj->pending_intr)
	{
	  //get raw data
	  APS_LOG(" ps change\n");
	  if((err = cm36283_read_ps(obj->client, &obj->ps)))
	  {
		 APS_ERR("cm36283 read ps data: %d\n", err);
	  }
	  //map and store data to hwm_sensor_data
	  sensor_data.values[0] = cm36283_get_ps_value(obj, obj->ps);
	  sensor_data.value_divide = 1;
	  sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	  //let up layer to know
	  if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
	  {
		APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	  }
	}
	
	#ifdef MT6516
	MT6516_EINTIRQUnmask(CUST_EINT_ALS_NUM);      
	#endif     

	#ifdef MT6575
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);      
	#endif
}
/*----------------------------------------------------------------------------*/
int cm36283_setup_eint(struct i2c_client *client)
{
	//struct cm36283_priv *obj = i2c_get_clientdata(client);        

	//g_cm36283_ptr = obj;
	/*configure to GPIO function, external interrupt*/
	
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#ifdef MT6516

	MT6516_EINT_Set_Sensitivity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	MT6516_EINT_Set_Polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	MT6516_EINT_Set_HW_Debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	MT6516_EINT_Registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, cm36283_eint_func, 0);
	MT6516_EINTIRQUnmask(CUST_EINT_ALS_NUM);  
#endif
    //
#ifdef MT6573
	
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, cm36283_eint_func, 0);
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
#endif  

#ifdef MT6575
#ifndef	TPD_PS_SUPPORT//LK@ use polling mode for tp ps function	
		mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
		mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
		mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
		mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, cm36283_eint_func, 0);
		mt65xx_eint_unmask(CUST_EINT_ALS_NUM);	
#endif
#endif 

    return 0;
	
}
/*----------------------------------------------------------------------------*/
static int cm36283_init_client(struct i2c_client *client)
{
	struct cm36283_priv *obj = i2c_get_clientdata(client);
	int err;
	u8 addr;

	g_cm36283_ptr = obj;
#if defined(TPD_PS_SUPPORT)
	return 0;
#endif
	if((err = cm36283_setup_eint(client)))
	{
		APS_ERR("setup eint: %d\n", err);
		return err;
	}
	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 0) 
	{
		if((err = cm36283_read_rar(client, &addr)))
		{
			APS_ERR("WARNING: read rar: %d\n", err);
			//return 0;
		}
	}
	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 0) 
	{
		if((err = cm36283_read_rar(client, &addr)))
		{
			APS_ERR("WARNING: read rar: %d\n", err);
			//return 0;
		}
	}

	if (0)//((err = cm36283_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", err);
		//    return err;
	}

	if((err = cm36283_init_device(client)))
	{
		
		APS_ERR("init dev: %d\n", err);
		return err;
	}
	if((err = cm36283_write_als(client, atomic_read(&obj->als_cmd_val))))
	{
		APS_ERR("write als: %d\n", err);  // -14
		return err;
	}
	
	if((err = cm36283_write_ps(client, atomic_read(&obj->ps_cmd_val))))
	{
		APS_ERR("write ps: %d\n", err);
		return err;        
	}
	
	if((err = cm36283_write_ps_thd(client, atomic_read(&obj->ps_thd_val_low),atomic_read(&obj->ps_thd_val_high))))
	{
		APS_ERR("write thd: %d\n", err);
		return err;        
	}
	return 0;
}
/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t cm36283_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&cm36283_obj->i2c_retry), atomic_read(&cm36283_obj->als_debounce), 
		atomic_read(&cm36283_obj->ps_mask), atomic_read(&cm36283_obj->ps_thd_val), atomic_read(&cm36283_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&cm36283_obj->i2c_retry, retry);
		atomic_set(&cm36283_obj->als_debounce, als_deb);
		atomic_set(&cm36283_obj->ps_mask, mask);
		atomic_set(&cm36283_obj->ps_thd_val, thres);        
		atomic_set(&cm36283_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&cm36283_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&cm36283_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	if((res = cm36283_read_als(cm36283_obj->client, &cm36283_obj->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", cm36283_obj->als);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	
	if((res = cm36283_read_ps(cm36283_obj->client, &cm36283_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", cm36283_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_reg(struct device_driver *ddri, char *buf)
{
	const int max_reg=13;
	u16 reg_dat[max_reg];
	int i;
	int len=0;
	int ret;

	APS_FUN1(1);

	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}

	for(i=0;i<max_reg;i++)
	{
		ret = cm36283_read(cm36283_obj->client, cm36283_obj->addr.ps_dat,i,(char*)(reg_dat+i), 2);
		if(ret<0)
		{
			APS_ERR("read reg failed ,ret = %d,i=%d\n", ret,i);
			return -EFAULT;
		} 
	}

	for(i=0;i<max_reg;i++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "cm36283 reg %d: 0x%04X\n",i, reg_dat[i]);
		APS_LOG("cm36283 reg %d: 0x%04X\n",i, reg_dat[i]);
	}

	return len;
}

static ssize_t cm36283_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	int reg;
	u16 data;
	int ret;

	APS_FUN2(1,count);
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return count;
	}
	
	ret = sscanf(buf, "%d", &reg);
	APS_LOG("input buf = '%s', reg=%d\n",buf,reg);
	if(ret<1)
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return count;
	}


	ret = cm36283_read(cm36283_obj->client, cm36283_obj->addr.ps_dat,reg,(char*)(&data), 2);
	
	APS_LOG("cm36283 reg %d: 0x%04X,  ret=%d\n",reg, data,ret);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	APS_LOG("send(%02X, %02X) = %d\n", addr, cmd, 
	cm36283_master_send(cm36283_obj->client, (u16)addr, (char*)&dat, sizeof(dat)));
	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	u8 dat;
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	APS_LOG("recv(%02X) = %d, 0x%02X\n", addr, 
	cm36283_master_recv(cm36283_obj->client, (u16)addr, (char*)&dat, sizeof(dat)), dat);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	
	if(cm36283_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d) (%02X %02X) (%02X %02X %02X) (%02X %02X %02X)\n", 
			cm36283_obj->hw->i2c_num, cm36283_obj->hw->power_id, cm36283_obj->hw->power_vol, cm36283_obj->addr.init, 
			cm36283_obj->addr.rar,cm36283_obj->addr.als_cmd, cm36283_obj->addr.als_dat0, cm36283_obj->addr.als_dat1,
			cm36283_obj->addr.ps_cmd, cm36283_obj->addr.ps_dat, cm36283_obj->addr.ps_thd);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02X %02lX %02lX\n", 
				atomic_read(&cm36283_obj->als_cmd_val), atomic_read(&cm36283_obj->ps_cmd_val), 
				atomic_read(&cm36283_obj->ps_thd_val),cm36283_obj->enable, cm36283_obj->pending_intr);
	#ifdef MT6516
	len += snprintf(buf+len, PAGE_SIZE-len, "EINT: %d (%d %d %d %d)\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN),
				CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_DEBOUNCE_CN);

	len += snprintf(buf+len, PAGE_SIZE-len, "GPIO: %d (%d %d %d %d)\n",	GPIO_ALS_EINT_PIN, 
				mt_get_gpio_dir(GPIO_ALS_EINT_PIN), mt_get_gpio_mode(GPIO_ALS_EINT_PIN), 
				mt_get_gpio_pull_enable(GPIO_ALS_EINT_PIN), mt_get_gpio_pull_select(GPIO_ALS_EINT_PIN));
	#endif

	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&cm36283_obj->als_suspend), atomic_read(&cm36283_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct cm36283_priv *obj, const char* buf, size_t count,
                             u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < cm36283_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", cm36283_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
//	struct cm36283_priv *obj;
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(cm36283_obj->als_level, cm36283_obj->hw->als_level, sizeof(cm36283_obj->als_level));
	}
	else if(cm36283_obj->als_level_num != read_int_from_buf(cm36283_obj, buf, count, 
			cm36283_obj->hw->als_level, cm36283_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < cm36283_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", cm36283_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36283_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!cm36283_obj)
	{
		APS_ERR("cm36283_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(cm36283_obj->als_value, cm36283_obj->hw->als_value, sizeof(cm36283_obj->als_value));
	}
	else if(cm36283_obj->als_value_num != read_int_from_buf(cm36283_obj, buf, count, 
			cm36283_obj->hw->als_value, cm36283_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, cm36283_show_als,   NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, cm36283_show_ps,    NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, cm36283_show_config,cm36283_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, cm36283_show_alslv, cm36283_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, cm36283_show_alsval,cm36283_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, cm36283_show_trace, cm36283_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, cm36283_show_status,  NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, cm36283_show_send,  cm36283_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, cm36283_show_recv,  cm36283_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, cm36283_show_reg,   cm36283_store_reg);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *cm36283_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int cm36283_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(cm36283_attr_list)/sizeof(cm36283_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, cm36283_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", cm36283_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int cm36283_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(cm36283_attr_list)/sizeof(cm36283_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, cm36283_attr_list[idx]);
	}
	
	return err;
}
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int cm36283_get_als_value(struct cm36283_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	als=als*1000/C_CUST_ALS_FACTOR;
	if(als>=als_raw_data)
	{
		for(idx = 0; idx < obj->als_level_num; idx++)
		{
			if(als < obj->hw->als_level[idx])
			{
				break;
			}
		}
	}
	else
	{
		for(idx = 0; idx < obj->als_level_num; idx++)
		{
			if(als < obj->hw->als_level[idx]-20)
			{
				break;
			}
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	als_raw_data=als;	
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		}
#if defined(ACER_Z1)
		return als;
#else
		return obj->hw->als_value[idx];
#endif		
	}
	else
	{
		if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		}
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
static int cm36283_get_ps_value(struct cm36283_priv *obj, u16 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
        static int ps_val_pre=1;
	ps_raw_data=ps;
	if(ps_cali.valid==1)
	{
		if(ps >= ps_cali.close)
		{
			val = 0;  /*close*/
		}
		else if(ps <= ps_cali.far_away)
		{
			val = 1;  /*far away*/
		}
		else
		{
			val=ps_val_pre;
		}
	}
	else
	{
	 if(ps >= atomic_read(&obj->ps_thd_val_high))
	{
		val = 0;  /*close*/
	}
	else if(ps <= atomic_read(&obj->ps_thd_val_low))
	{
		val = 1;  /*far away*/
	}
	else
	{
		        val=ps_val_pre;
	       }
        }
	ps_val_pre=val;
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		if(0 == test_bit(CMC_BIT_PS,  &obj->enable))
		{
		  //if ps is disable do not report value
		  APS_DBG("PS: not enable and do not report this value\n");
		  return -1;
		}
		else
		{
		   return val;
		}
		
	}	
	else
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int cm36283_open(struct inode *inode, struct file *file)
{
	file->private_data = cm36283_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int cm36283_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static cm36283_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{

	 //  APS_LOG("tmd2771_WriteCalibration  1 %d," ,data_cali->close);
	//	   APS_LOG("tmd2771_WriteCalibration  2 %d," ,data_cali->far_away);
	//	   APS_LOG("tmd2771_WriteCalibration  3 %d,", data_cali->valid);
		   
	  if(data_cali->valid == 1)
	  {  
		ps_cali.close = data_cali->close;
		ps_cali.far_away= data_cali->far_away;
		ps_cali.valid = 1;
	  }	  
}
/************************longxuewei add 2012-02-28 start*************************************/
#define ALSPS				0X84
#define ALSPS_GET_ALS_RAW_DATA2  	_IOR(ALSPS, 0x09, int)
#define ALSPS_GET_PS_THD              _IOR(ALSPS, 0x0a, int)
#define ALSPS_SET_PS_CALI  		_IOR(ALSPS, 0x0b, int)
/************************longxuewei add 2012-02-28 end*************************************/

//static int cm36283_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
       //unsigned long arg)
static long cm36283_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct cm36283_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
        struct PS_CALI_DATA_STRUCT ps_cali_temp;

printk("cm36283_unlocked_ioctl()\n");

	APS_FUN2(1,cmd);
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = cm36283_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = cm36283_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			
			printk("IOctl()----ALSPS_GET_PS_DATA\n");
			
			if((err = cm36283_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = cm36283_get_ps_value(obj, obj->ps);
			printk("tengdeqiang %s,%d,%d",__func__,obj->ps,dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    

			printk("IOctl()----ALSPS_GET_PS_RAW_DATA\n");
			
			if((err = cm36283_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			printk("tengdeqiang 2222%s,%d",__func__,obj->ps);
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;            

		case ALSPS_SET_ALS_MODE:

			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = cm36283_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = cm36283_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = cm36283_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = cm36283_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:  

			printk("IOctl()----ALSPS_GET_ALS_RAW_DATA\n");
			
			if((err = cm36283_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
/************************longxuewei add 2012-02-28 start*************************************/
		case ALSPS_GET_ALS_RAW_DATA2:     

			printk("IOctl()----ALSPS_GET_ALS_RAW_DATA2\n");
			
			if((err = cm36283_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}
			//dat = obj->als;
			dat = obj->als*1000/C_CUST_ALS_FACTOR;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		case ALSPS_GET_PS_THD:    //ps_threshold
	                if(ps_cali.valid==1)
			{
				ps_cali_temp.close=ps_cali.close;
				ps_cali_temp.far_away=ps_cali.far_away;
				ps_cali_temp.valid=ps_cali.valid;
			}
			else
			{
				ps_cali_temp.close=atomic_read(&obj->ps_thd_val_high);
				ps_cali_temp.far_away=atomic_read(&obj->ps_thd_val_low);
				ps_cali_temp.valid=0;
			}			
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}       
			break;
              	case ALSPS_SET_PS_CALI:
			dat = (void __user*)arg;
			if(dat == NULL)
			{
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&ps_cali_temp,dat, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;	  
			}
			cm36283_WriteCalibration(&ps_cali_temp);
printk("[LONG][ALS/PS] ALSPS_SET_PS_CALI %d,%d,%d\n\r",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
			cm36283_write_ps_thd(obj->client, atomic_read(&obj->ps_thd_val_low),atomic_read(&obj->ps_thd_val_high));
			break;
/************************longxuewei add 2012-02-28 end*************************************/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations cm36283_fops = {
	.owner = THIS_MODULE,
	.open = cm36283_open,
	.release = cm36283_release,
        .unlocked_ioctl = cm36283_unlocked_ioctl,
	//.ioctl = cm36283_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice cm36283_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &cm36283_fops,
};
/*----------------------------------------------------------------------------*/
static int cm36283_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	//struct cm36283_priv *obj = i2c_get_clientdata(client);    
	//int err;
	APS_FUN();  
	/*
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if((err = cm36283_enable_als(client, 0)))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}
		APS_LOG("cm36283_i2c_suspend do not disable ps: %d\n");


		atomic_set(&obj->ps_suspend, 1);
		if((err = cm36283_enable_ps(client, 0)))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		cm36283_power(obj->hw, 0);
	}
	*/
	return 0;
}
/*----------------------------------------------------------------------------*/
static int cm36283_i2c_resume(struct i2c_client *client)
{
	//struct cm36283_priv *obj = i2c_get_clientdata(client);        
	//int err;
	APS_FUN();
	/*

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	cm36283_power(obj->hw, 1);
	if((err = cm36283_init_client(client)))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = cm36283_enable_als(client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	APS_LOG("cm36283_i2c_resume do not enable ps: %d\n");
	
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if((err = cm36283_enable_ps(client, 1)))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
*/
	return 0;
}
/*----------------------------------------------------------------------------*/
static void cm36283_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct cm36283_priv *obj = container_of(h, struct cm36283_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1);    
	if((err = cm36283_enable_als(obj->client, 0)))
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
}
/*----------------------------------------------------------------------------*/
static void cm36283_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct cm36283_priv *obj = container_of(h, struct cm36283_priv, early_drv);         
	int err;
	hwm_sensor_data sensor_data;
	
	memset(&sensor_data, 0, sizeof(sensor_data));
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = cm36283_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
}

#if defined(TPD_PS_SUPPORT)
extern void msg2133_ps_enable(int enable);
extern unsigned char msg2133_get_ps_value(void);

void msg2133_ps_set_hwm_data(int ps_data)//for intterupt mode, not confirm yet
{
	int err;
	hwm_sensor_data sensor_data;
	int res = 0;

      printk("msg2133_ps_set_hwm_data");
    
	sensor_data.values[0] = ps_data;
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			
	//let up layer to know
	if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
	{
	  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	}
}

EXPORT_SYMBOL (msg2133_ps_set_hwm_data);

#endif

int cm36283_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct cm36283_priv *obj = (struct cm36283_priv *)self;
	
	//APS_FUN(f);
	APS_FUN2(1,command);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{

                #if defined(TPD_PS_SUPPORT)
					msg2133_ps_enable(1);
		  #else
					if((err = cm36283_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
		#endif
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
				#if defined(TPD_PS_SUPPORT)
					msg2133_ps_enable(0);
				#else
					if((err = cm36283_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
			       #endif
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
		#if defined(TPD_PS_SUPPORT)//polling mode
                		sensor_data->values[0] = msg2133_get_ps_value();

				//printk("%s %d,sensor_data->values[0]=%d,",__func__,obj->ps,sensor_data->values[0]);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		#else
				if((err = cm36283_read_ps(obj->client, &obj->ps)))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = cm36283_get_ps_value(obj, obj->ps);
					sensor_data->values[1] = ps_raw_data;

					printk("11111tengdeqiang %s %d,%d,",__func__,obj->ps,sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}		
        #endif
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int cm36283_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct cm36283_priv *obj = (struct cm36283_priv *)self;
	
	//APS_FUN(f);
	APS_FUN2(1,command);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if((err = cm36283_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = cm36283_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
								
				if((err = cm36283_read_als(obj->client, &obj->als)))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = cm36283_get_als_value(obj, obj->als);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}				
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/

static int cm36283_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, CM36283_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int cm36283_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cm36283_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	cm36283_obj = obj;

	obj->hw = get_cust_alsps_hw();
	cm36283_get_addr(obj->hw, &obj->addr);

	INIT_DELAYED_WORK(&obj->eint_work, cm36283_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 2000);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 150);//1000
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);

#if 0
	atomic_set(&obj->als_cmd_val, 0xc3);//DF 800ms D3 100ms  /*time*/

	
	if(1 == obj->hw->polling_mode_ps && 1 == obj->hw->polling_mode_als)
	{
	  atomic_set(&obj->ps_cmd_val,  0x51);// disable interrupt  //0xC1:253ms ;0x51:67ms
	  APS_LOG("disable ps and als interrupt\n");
	}
	if(0 == obj->hw->polling_mode_ps && 0 == obj->hw->polling_mode_als)
	{
	  atomic_set(&obj->ps_cmd_val,  0xCD);//enable interrupt
	  APS_LOG("enable ps and als interrupt\n");
	}
	if(0 == obj->hw->polling_mode_ps && 1 == obj->hw->polling_mode_als)
	{
	  atomic_set(&obj->ps_cmd_val,  0x03C3);//enable ps interrupt
	  APS_LOG("only enable ps interrupt\n");
	}
	if(1 == obj->hw->polling_mode_ps && 0 == obj->hw->polling_mode_als)
	{
	  atomic_set(&obj->ps_cmd_val,  0xC9);//enable ps interrupt
	  APS_LOG("only enable als interrupt\n");
	}
	
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
#else
	if(1 == obj->hw->polling_mode_als)
	{
#if defined(ACER_Z1)
		atomic_set(&obj->als_cmd_val, 0x04);//640ms0xc5
#else
		atomic_set(&obj->als_cmd_val, 0x05);//bit7:6--ALS_IT:(0:0)--80MS;(0:1)--160MS;(1:0)--320MS;(1:1)--640MS
#endif
	}
	if(0 == obj->hw->polling_mode_als)
	{
		atomic_set(&obj->als_cmd_val, 0x0B);//bit1--ALS_INT_EN:0--disable;1--enable
	}

/*	bit0=PS_SD = [1] , PS Power off
	bit3:2=PS_PERS = [1,1] , PS interrupt persistence = 4
	bit5:4=PS_IT = [0,0] , PS integration time = 1T,T=0.32ms
	bit7:6=PS_DUTY = [1,1] , PS duty = 1/320   */

	if(1 == obj->hw->polling_mode_ps)
	{
	  atomic_set(&obj->ps_cmd_val,  0x00CF);// disable interrupt  //0xC1:253ms ;0x51:67ms
	}
	if(0 == obj->hw->polling_mode_ps )
	{
	  atomic_set(&obj->ps_cmd_val,  0x03CF);//enable interrupt
	}
	
	atomic_set(&obj->ps_cmd_val_3, 0x00); //bit4--PS_AMART_PERS:0--smart disable;1--smart_enable
        atomic_set(&obj->ps_canc_val, 0x00);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	atomic_set(&obj->ps_thd_val_low,  CM36283_ps_threshold_low);
	atomic_set(&obj->ps_thd_val_high,  CM36283_ps_threshold_high);
//cm36283 setting end
#endif
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	if(!(atomic_read(&obj->als_cmd_val) & SD_ALS))
	{
		set_bit(CMC_BIT_ALS, &obj->enable);
	}
	
	if(!(atomic_read(&obj->ps_cmd_val) & SD_PS))
	{
		set_bit(CMC_BIT_PS, &obj->enable);
	}
	
	cm36283_i2c_client = client;

	
	if((err = cm36283_init_client(client)))
	{
		goto exit_init_failed;
	}
	
	if((err = misc_register(&cm36283_device)))
	{
		APS_ERR("cm36283_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = cm36283_create_attr(&cm36283_alsps_driver.driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	obj_ps.self = cm36283_obj;
	if(1 == obj->hw->polling_mode_ps)
	{
	  obj_ps.polling = 1;
	}
	else
	{
	  obj_ps.polling = 0;//interrupt mode
	}
	obj_ps.sensor_operate = cm36283_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = cm36283_obj;
	if(1 == obj->hw->polling_mode_als)
	{
	  obj_als.polling = 1;
	}
	else
	{
	  obj_als.polling = 0;//interrupt mode
	}
	obj_als.sensor_operate = cm36283_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = cm36283_early_suspend,
	obj->early_drv.resume   = cm36283_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	printk("tengdeqiang %s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&cm36283_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
//	exit_kfree:
	kfree(obj);
	exit:
	cm36283_i2c_client = NULL;           
	#ifdef MT6516        
	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	#endif
	printk("tengdeqiang %s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int cm36283_i2c_remove(struct i2c_client *client)
{
	int err;	
	
	if((err = cm36283_delete_attr(&cm36283_i2c_driver.driver)))
	{
		APS_ERR("cm36283_delete_attr fail: %d\n", err);
	} 

	if((err = misc_deregister(&cm36283_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	cm36283_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int cm36283_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	struct cm36283_i2c_addr addr;

	printk("cm36283_probe()\n");
	
	cm36283_power(hw, 1);    
	cm36283_get_addr(hw, &addr);
	//cm36283_force[0] = hw->i2c_num;
	//cm36283_force[1] = addr.init;
	if(i2c_add_driver(&cm36283_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	//APS_LOG("cm36283_force: %d,0x%02x\n",cm36283_force[0],cm36283_force[1]);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int cm36283_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	cm36283_power(hw, 0);    
	i2c_del_driver(&cm36283_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver cm36283_alsps_driver = {
	.probe      = cm36283_probe,
	.remove     = cm36283_remove,    
	.driver     = {
		.name  = "als_ps",
		//.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init cm36283_init(void)
{
	APS_FUN();

	printk("cm36283_init()\n");
	
	i2c_register_board_info(0, &i2c_cm36283, 1);
	if(platform_driver_register(&cm36283_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit cm36283_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&cm36283_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(cm36283_init);
module_exit(cm36283_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("MingHsien Hsieh");
MODULE_DESCRIPTION("CM36283 ALS/PS driver");
MODULE_LICENSE("GPL");
