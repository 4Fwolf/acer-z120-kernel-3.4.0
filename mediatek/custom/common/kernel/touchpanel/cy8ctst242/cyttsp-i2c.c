/* Source for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * drivers/input/touchscreen/cyttsp-i2c.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */


#include "cyttsp.h"

#include "tpd.h"
#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif

#if defined(__TPD_I2C_CHECK_SW_WORKAROUD__)
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#endif


#if defined(__TPD_FW_UPDATE_THREAD__)
#include <linux/kthread.h>
#define MODTIMER
#endif

extern struct tpd_device *tpd;
struct cyttsp * this_ts;
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
//#define CYTTSP_DEBUG
#ifdef CYTTSP_DEBUG
#define CYTTSP_PRINT(x...)		printk("CYTTSP:"x)
#else
#define CYTTSP_PRINT(x...)
#endif

uint32_t cyttsp_tsdebug1;
module_param_named(tsdebug1, cyttsp_tsdebug1, uint, 0664);

/* CY TTSP I2C Driver private data */
struct cyttsp {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct timer_list timer;
	struct mutex mutex;
	spinlock_t lock;
	char phys[32];
	struct cyttsp_platform_data *platform_data;
	u8 num_prev_st_touch;
	u16 active_track[CYTTSP_NUM_TRACK_ID];
	u16 prev_st_touch[CYTTSP_NUM_ST_TOUCH_ID];
	u16 prev_mt_touch[CYTTSP_NUM_MT_TOUCH_ID];
	u16 prev_mt_pos[CYTTSP_NUM_MT_TOUCH_ID][2];
	atomic_t irq_enabled;
	struct early_suspend early_suspend;
};

static struct cyttsp_platform_data cyttsp_data = {
	.maxx = 320,
	.maxy = 480,
	.flags = 0,
//	.gen = CYTTSP_SPI10,	/* either */
//	.gen = CYTTSP_GEN2,	/* or */
	.gen = CYTTSP_GEN2,	/* or */
	.use_st = 0,
	.use_mt = CYTTSP_USE_MT,
	.use_trk_id = 1,//LK@0->1 for Android 4.1
	.use_sleep = 1,
	.use_gestures = 0,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CYTTSP_GEST_GRP1 | CYTTSP_GEST_GRP2 | 
				CYTTSP_GEST_GRP3 | CYTTSP_GEST_GRP4 | 
				CYTTSP_ACT_DIST,
	/* change act_intrvl to customize the Active power state 
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CYTTSP_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CYTTSP_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state 
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CYTTSP_LP_INTRVL_DFLT,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_early_suspend(struct early_suspend *handler);
static void cyttsp_late_resume(struct early_suspend *handler);
#endif /* CONFIG_HAS_EARLYSUSPEND */


unsigned int cyttsp_app_ver;//LK@add

#if defined(ACER_Z1)//defined(KT350)

#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON 
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_BACK,KEY_HOMEPAGE,KEY_MENU}
//#define TPD_KEYS_DIM            {{33,501,80,50},{161,501,80,50},{289,501,80,50}}
#define TPD_KEYS_DIM            {{40,505,80,50},{160,505,80,50},{280,505,80,50}}

#endif
#elif defined(ACER_Z2)
#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON 
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_BACK,KEY_HOMEPAGE,KEY_MENU}
//#define TPD_KEYS_DIM            {{33,501,80,50},{161,501,80,50},{289,501,80,50}}
#define TPD_KEYS_DIM            {{40,510,80,60},{160,510,80,60},{280,510,80,60}}

#endif
#elif defined(I706M_V5)
#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON 
#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOME, KEY_MENU, KEY_BACK,KEY_SEARCH}
#define TPD_KEYS_DIM            {{40,550,80,50},{120,550,80,50},{200,550,80,50},{280,550,80,50}}
#endif

#else

#ifdef TPD_HAVE_BUTTON 
#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOME, KEY_MENU, KEY_BACK,KEY_SEARCH}
#define TPD_KEYS_DIM            {{40,505,80,50},{120,505,80,50},{200,505,80,50},{280,505,80,50}}
#endif
#endif

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#define CYTTSP_USE_IRQ
/* ****************************************************************************
 * Prototypes for static functions
 * ************************************************************************** */
static void cyttsp_xy_worker(struct work_struct *work);
static void cyttsp_irq(void);
static int cyttsp_inlist(u16 prev_track[], u8 curr_track_id, u8 *prev_loc, u8 num_touches);
static int cyttsp_next_avail_inlist(u16 curr_track[], u8 *new_loc, u8 num_touches);
static int cyttsp_putbl(struct cyttsp *ts, int show, int show_status, int show_version, int show_cid);
static int __devinit cyttsp_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit cyttsp_remove(struct i2c_client *client);
static int cyttsp_resume(struct i2c_client *client);
static int cyttsp_suspend(struct i2c_client *client, pm_message_t message);

/* Static variables */
static struct cyttsp_gen3_xydata_t g_xy_data;
static struct cyttsp_bootloader_data_t g_bl_data;
static struct cyttsp_sysinfo_data_t g_sysinfo_data;
static struct cyttsp_gen3_xydata_t g_wake_data;
static const struct i2c_device_id cyttsp_id[] = {
	{ CYTTSP_I2C_NAME, 0 },  { }
};

#define TPD_SLAVE_ADDR (0x48>>1)//0x24 for 7bit address

static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
//static unsigned short force[] = {0, TPD_SLAVE_ADDR, I2C_CLIENT_END,I2C_CLIENT_END};//longxw ?
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(CYTTSP_I2C_NAME, TPD_SLAVE_ADDR)};


static struct i2c_driver cyttsp_driver = {
	.driver.name = CYTTSP_I2C_NAME,
	.probe = cyttsp_probe,
	.remove = __devexit_p(cyttsp_remove),
	.detect = tpd_i2c_detect, 
//	.suspend = cyttsp_suspend,
//	.resume = cyttsp_resume,
	.id_table = cyttsp_id,
	//.address_data = &addr_data,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver");
MODULE_AUTHOR("Cypress");


 static u8 bl_cmd[] = {
	CYTTSP_BL_FILE0, CYTTSP_BL_CMD, CYTTSP_BL_EXIT,
	CYTTSP_BL_KEY0, CYTTSP_BL_KEY1, CYTTSP_BL_KEY2,
	CYTTSP_BL_KEY3, CYTTSP_BL_KEY4, CYTTSP_BL_KEY5,
	CYTTSP_BL_KEY6, CYTTSP_BL_KEY7};

static ssize_t cyttsp_irq_status(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev,
	                                         struct i2c_client, dev);
	struct cyttsp *ts = i2c_get_clientdata(client);
	return sprintf(buf, "%u\n", atomic_read(&ts->irq_enabled));
}

static ssize_t cyttsp_irq_enable(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev,
	                                         struct i2c_client, dev);
	struct cyttsp *ts = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;
/*
	struct qtm_obj_message *msg;
*/

	if (size > 2)
		return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
		return err;

	switch (value) {
	case 0:
		if (atomic_cmpxchg(&ts->irq_enabled, 1, 0)) {
			pr_info("touch irq disabled!\n");
			//disable_irq_nosync(ts->client->irq);
			mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		}
		err = size;
		break;
	case 1:
		if (!atomic_cmpxchg(&ts->irq_enabled, 0, 1)) {
			pr_info("touch irq enabled!\n");
/*
			msg = cyttsp_read_msg(ts);
			if (msg == NULL)
				pr_err("%s: Cannot read message\n", __func__);
*/
			//enable_irq(ts->client->irq);
			mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		}
		err = size;
		break;
	default:
		pr_info("cyttsp_irq_enable failed -> irq_enabled = %d\n",
		atomic_read(&ts->irq_enabled));
		err = -EINVAL;
		break;
	}

	return err;
}

static DEVICE_ATTR(irq_enable, S_IRUGO|S_IWUSR|S_IWGRP, cyttsp_irq_status, cyttsp_irq_enable);

#define TPD_RES_X (320)
#define TPD_RES_Y (480)

static void tpd_down(int x, int y,int id)
{


#ifdef TPD_HAVE_BUTTON
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {   
        printk("tpd_down,get_boot_mode()=%d\n",get_boot_mode());
	    if(y > 480) {
            tpd_button(x, y, 1);  
	}
	else
	{
	    input_report_key(tpd->dev, BTN_TOUCH, 1);
	}
    }
    else
    {
	    input_report_key(tpd->dev, BTN_TOUCH, 1);
    }
#else
	input_report_key(tpd->dev, BTN_TOUCH, 1);
#endif

	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);//curr_mt_z[id]
	//input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, curr_tool_width);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	CYTTSP_MT_SYNC(tpd->dev);
}

static void tpd_up(int x, int y)
{


#ifdef TPD_HAVE_BUTTON
        if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
        {   
            printk("tpd_up,get_boot_mode()=%d\n",get_boot_mode());
            tpd_button(x, y, 0); 

        }   
        else
        {
            input_report_key(tpd->dev, BTN_TOUCH, 0);
        }
#else
        input_report_key(tpd->dev, BTN_TOUCH, 0);
#endif
        CYTTSP_MT_SYNC(tpd->dev);

}
static void tpd_down_eng(int x, int y)
{


#ifdef TPD_HAVE_BUTTON
   if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {   
     //input_report_key(tpd->dev, BTN_TOUCH, 1);
     //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
     //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
     //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
     //pr_tp("D[%4d %4d %4d] ", x, y, p);
     //input_mt_sync(tpd->dev);
        printk("tpd_down_eng");
        tpd_button(x, y, 1);  
        if(y > TPD_RES_Y)
            msleep(50);
    }
#endif
}

static void tpd_up_eng(int x, int y)
{
#if 0
    if(*count > 0)
    {
	    //if(FACTORY_BOOT == bootMode)
	    //   input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_report_abs(tpd->dev, ABS_PRESSURE, 0);
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
        input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
        //pr_tp("U[%4d %4d %4d] ", x, y, 0);
        input_mt_sync(tpd->dev);
        TPD_UP_DEBUG_TRACK(x, y);
        (*count)--;
        
        if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
        {   
        
            tpd_button(x, y, 0); 
        }   
        return 1;
    }
#endif

#ifdef TPD_HAVE_BUTTON
        if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
        {   
       // input_report_abs(tpd->dev, ABS_PRESSURE, 0);
       // input_report_key(tpd->dev, BTN_TOUCH, 0);
       // input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
       // input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
       // input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
       // pr_tp("U[%4d %4d %4d] ", x, y, 0);
       // input_mt_sync(tpd->dev);
            printk("tpd_up_eng");

            tpd_button(x, y, 0); 

        }   
#endif

}
/* The cyttsp_xy_worker function reads the XY coordinates and sends them to
 * the input layer.  It is scheduled from the interrupt (or timer).
 */
void cyttsp_xy_worker(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work,struct cyttsp,work);
	u8 id, tilt, reverse_x, reverse_y;
	u8 i, loc;
	u8 prev_touches;
	u8 curr_touches;
	u16 temp_track[CYTTSP_NUM_MT_TOUCH_ID];
	u16 send_track[CYTTSP_NUM_MT_TOUCH_ID];
	u16 curr_track[CYTTSP_NUM_TRACK_ID];
	u16 curr_st_touch[CYTTSP_NUM_ST_TOUCH_ID];
	u16 curr_mt_touch[CYTTSP_NUM_MT_TOUCH_ID];
	u16 curr_mt_pos[CYTTSP_NUM_TRACK_ID][2];	/* if NOT CYTTSP_USE_TRACKING_ID then only uses CYTTSP_NUM_MT_TOUCH_ID positions */
	u8 curr_mt_z[CYTTSP_NUM_TRACK_ID];			/* if NOT CYTTSP_USE_TRACKING_ID then only uses CYTTSP_NUM_MT_TOUCH_ID positions */
	u8 curr_tool_width;
	u16 st_x1, st_y1;
	u8 st_z1;
	u16 st_x2, st_y2;
	u8 st_z2;
	s32 retval;

	/* get event data from CYTTSP device */
	i = CYTTSP_NUM_RETRY;
	do {
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE,
				8, (u8 *)&g_xy_data);
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+8,
				8, (u8 *)&g_xy_data+8);
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+16,
				8, (u8 *)&g_xy_data+16);
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+24,
				6, (u8 *)&g_xy_data+24);
	}
	while ((retval < CYTTSP_OPERATIONAL) && --i);

	/* return immediately on failure to read device on the i2c bus */
	if (retval < CYTTSP_OPERATIONAL) {
		goto exit_xy_worker;
	}

	/* Get the current number of touches and return if there are no touches */
	if ((GET_BOOTLOADERMODE(g_xy_data.tt_mode)==1) ||
		(GET_HSTMODE(g_xy_data.hst_mode) != CYTTSP_OPERATIONAL)) {
		u8 host_reg, tries;
		/* the TTSP device has suffered spurious reset or mode switch */
		printk("[cyttsp_xy_worker]Spurious error in operational mode (tt_mode=0x%02X  hst_mode=0x%02X)\n", \
			g_xy_data.tt_mode, g_xy_data.hst_mode);
		printk("[cyttsp_xy_worker]Resetting TTSP Device; Terminating active tracks\n");
		/* terminate all active tracks */
		curr_touches = CYTTSP_NOTOUCH; 
		/* reset the TTSP part and take it back out of Bootloader mode */
		host_reg = CYTTSP_SOFT_RESET_MODE;
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
			sizeof(host_reg), &host_reg);
		tries = 0;
		do {
			mdelay(100);

			/* set arg2 to non-0 to activate */
			retval = cyttsp_putbl(ts, 1, false, false, false);
		}
		while (!(retval < CYTTSP_OPERATIONAL) &&
			!GET_BOOTLOADERMODE(g_bl_data.bl_status) && 
			!(g_bl_data.bl_file == CYTTSP_OPERATE_MODE + CYTTSP_LOW_POWER_MODE) &&
			tries++ < 50);
		printk("[cyttsp_xy_worker]tries is %d, in the work\n", tries);
		/* switch back to operational mode */
		if (GET_BOOTLOADERMODE(g_bl_data.bl_status)) {
			printk("still in the bootloader mode\n");
            #if 1
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(bl_cmd), bl_cmd);
            #else
            retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+4, 4, bl_cmd+4);
			if (retval < 0)
				cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+8, 3, bl_cmd+8);
			if (retval < 0)
				cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 4, bl_cmd);
            #endif
			if (retval < 0)
				cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
			tries = 0;
			do {
				mdelay(100);
				cyttsp_putbl(ts, 2, false, false, false);
			}
			while (GET_BOOTLOADERMODE(g_bl_data.bl_status) && 
				tries++ < 50);
		}
		if (!(retval < CYTTSP_OPERATIONAL)) {
			printk("[cyttsp_xy_worker]switch to CYTTSP_OPERATE_MODE\n");
			host_reg = CYTTSP_OPERATE_MODE/* + CYTTSP_LOW_POWER_MODE*/;
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
				sizeof(host_reg), &host_reg);
			/* wait for TTSP Device to complete switch to Operational mode */
			mdelay(500);
		}
	}
	else if ((curr_touches = GET_NUM_TOUCHES(g_xy_data.tt_stat)) > CYTTSP_NUM_MT_TOUCH_ID) {
		/* if the number of fingers on the touch surface is more than the maximum
		 * then there will be no new track information even for the orginal
		 * touches. Therefore, ignore this touch event.
		 */
		 printk("[cyttsp_xy_worker]the number of fingers on the touch surface is more than the maximum\n");
		 goto exit_xy_worker;
	}
	else if (IS_LARGE_AREA(g_xy_data.tt_stat)==1) {
		/* terminate all active tracks */
		curr_touches = CYTTSP_NOTOUCH;
		printk("[cyttsp_xy_worker]Large object detected. Terminating active tracks\n");
	}

	cyttsp_xdebug("prev_touches is %d, curr_touches is %d \n", prev_touches, curr_touches);
	/* set tool size */
	curr_tool_width = CYTTSP_SMALL_TOOL_WIDTH;
    
#if 1//LK@add for ghost
       if(curr_touches == CYTTSP_GEN2_2TOUCH)
       {
           if((g_xy_data.x1 == g_xy_data.x2) && (g_xy_data.y1 == g_xy_data.y2))
           {
		       printk("[cyttsp_xy_worker]prev_touches is %d, curr_touches is %d \n", prev_touches, curr_touches);
               printk("[cyttsp_xy_worker]g_xy_data.x1=%d\n g_xy_data.y1=%d\n",g_xy_data.x1,g_xy_data.y1);
               printk("[cyttsp_xy_worker]g_xy_data.x2=%d\n g_xy_data.y2=%d\n",g_xy_data.x2,g_xy_data.y2);
		 goto exit_xy_worker;
           }
       }
#endif	
    /* translate Gen2 interface data into comparable Gen3 data */
	if(ts->platform_data->gen == CYTTSP_GEN2) {
		struct cyttsp_gen2_xydata_t *pxy_gen2_data;
		pxy_gen2_data = (struct cyttsp_gen2_xydata_t *)(&g_xy_data);

		/* use test data? */
		cyttsp_testdat(&g_xy_data, &tt_gen2_testray, sizeof(struct cyttsp_gen3_xydata_t));

		if (pxy_gen2_data->evnt_idx == CYTTSP_GEN2_NOTOUCH) {
			curr_touches = 0;
		}
		else if (curr_touches == CYTTSP_GEN2_GHOST) {
			//curr_touches = 0;
			/* stuff artificial track ID1 and ID2 */
	        g_xy_data.touch12_id = 0x12;
	        g_xy_data.z1 = CYTTSP_MAXZ;
	        g_xy_data.z2 = CYTTSP_MAXZ;
		}
		else if (curr_touches == CYTTSP_GEN2_2TOUCH) {
			g_xy_data.touch12_id = 0x12;	/* stuff artificial track ID1 and ID2 */
			g_xy_data.z1 = CYTTSP_MAXZ;
			g_xy_data.z2 = CYTTSP_MAXZ;
			curr_touches--;			/* 2 touches */
		}
		else if (curr_touches == CYTTSP_GEN2_1TOUCH) {
			g_xy_data.touch12_id = 0x12;	/* stuff artificial track ID1 and ID2 */
			g_xy_data.z1 = CYTTSP_MAXZ;
			g_xy_data.z2 = CYTTSP_NOTOUCH;
			if (pxy_gen2_data->evnt_idx == CYTTSP_GEN2_TOUCH2) {
				/* push touch 2 data into touch1 (first finger up; second finger down) */
				g_xy_data.touch12_id = 0x20;	/* stuff artificial track ID1 for touch 2 info */
				g_xy_data.x1 = g_xy_data.x2;	/* stuff touch 1 with touch 2 coordinate data */
				g_xy_data.y1 = g_xy_data.y2;
			}
		}
		else {
			curr_touches = 0;
		}
	}
	else {
		/* use test data? */
		cyttsp_testdat(&g_xy_data, &tt_gen3_testray, sizeof(struct cyttsp_gen3_xydata_t));
	}
	
	/* clear current active track ID array and count previous touches */
	for (id = 0, prev_touches = CYTTSP_NOTOUCH; id < CYTTSP_NUM_TRACK_ID; id++) {
		curr_track[id] = CYTTSP_NOTOUCH;
		prev_touches += ts->active_track[id];
	}
		
	/* send no events if there were no previous touches and no new touches */
	if ((prev_touches == CYTTSP_NOTOUCH) && 
		((curr_touches == CYTTSP_NOTOUCH) || (curr_touches > CYTTSP_NUM_MT_TOUCH_ID))) {
		printk("prev_touches:%d, curr_touches:%d \n", prev_touches, curr_touches);
		goto exit_xy_worker;
	}

	cyttsp_xdebug("prev=%d  curr=%d\n", prev_touches, curr_touches);

	/* clear current single touches array */
	for (id = 0; id < CYTTSP_NUM_ST_TOUCH_ID; id++) {
		curr_st_touch[id] = CYTTSP_IGNORE_TOUCH;
	}

	/* clear single touch positions */
	st_x1 = CYTTSP_NOTOUCH;
	st_y1 = CYTTSP_NOTOUCH;
	st_z1 = CYTTSP_NOTOUCH;
	st_x2 = CYTTSP_NOTOUCH;
	st_y2 = CYTTSP_NOTOUCH;
	st_z2 = CYTTSP_NOTOUCH;

	/* clear current multi-touches array and multi-touch positions/z */
	for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
		curr_mt_touch[id] = CYTTSP_IGNORE_TOUCH;
	}

	if (ts->platform_data->use_trk_id) {
		for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
			curr_mt_pos[id][CYTTSP_XPOS] = 0;
			curr_mt_pos[id][CYTTSP_YPOS] = 0;
			curr_mt_z[id] = 0;
		}
	}
	else {
		for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) {
			curr_mt_pos[id][CYTTSP_XPOS] = 0;
			curr_mt_pos[id][CYTTSP_YPOS] = 0;
			curr_mt_z[id] = 0;
		}
	}

	/* Determine if display is tilted */
	if (FLIP_DATA(ts->platform_data->flags)) {
		tilt = true;
	}
	else {
		tilt = false;
	}
	/* Check for switch in origin */
	if (REVERSE_X(ts->platform_data->flags)) {
		reverse_x = true;
	}
	else {
		reverse_x = false;
	}
	if (REVERSE_Y(ts->platform_data->flags)) {
		reverse_y = true;
	}
	else {
		reverse_y = false;
	}
    
	cyttsp_xdebug("2:prev_touches is %d, curr_touches is %d \n", prev_touches, curr_touches);

	if (curr_touches) {
		struct cyttsp_gen2_xydata_t *pxy_gen2_data;
		struct cyttsp_gen3_xydata_t *pxy_gen3_data;
		switch (ts->platform_data->gen) {
			case CYTTSP_GEN2: {
				pxy_gen2_data = (struct cyttsp_gen2_xydata_t *)(&g_xy_data);
				cyttsp_xdebug("TTSP Gen2 report:\n");
				cyttsp_xdebug("%02X %02X %02X  %04X %04X %02X  %02X  %04X %04X %02X\n", \
						pxy_gen2_data->hst_mode, pxy_gen2_data->tt_mode, pxy_gen2_data->tt_stat, \
						pxy_gen2_data->x1, pxy_gen2_data->y1, pxy_gen2_data->z1, \
						pxy_gen2_data->evnt_idx, \
						pxy_gen2_data->x2, pxy_gen2_data->y2, pxy_gen2_data->tt_undef1);
				cyttsp_xdebug("%02X %02X %02X\n", \
						pxy_gen2_data->gest_cnt, pxy_gen2_data->gest_id, pxy_gen2_data->gest_set);
				break;
			}
			case CYTTSP_GEN3:
			default: {
				pxy_gen3_data = (struct cyttsp_gen3_xydata_t *)(&g_xy_data);
				cyttsp_xdebug("TTSP Gen3 report:\n");
				cyttsp_xdebug("%02X %02X %02X  %04X %04X %02X  %02X  %04X %04X %02X\n", \
						pxy_gen3_data->hst_mode, pxy_gen3_data->tt_mode, pxy_gen3_data->tt_stat, \
						pxy_gen3_data->x1, pxy_gen3_data->y1, pxy_gen3_data->z1, \
						pxy_gen3_data->touch12_id, \
						pxy_gen3_data->x2, pxy_gen3_data->y2, pxy_gen3_data->z2);
				cyttsp_xdebug("%02X %02X %02X  %04X %04X %02X  %02X  %04X %04X %02X\n", \
						pxy_gen3_data->gest_cnt, pxy_gen3_data->gest_id, pxy_gen3_data->gest_set, \
						pxy_gen3_data->x3, pxy_gen3_data->y3, pxy_gen3_data->z3, \
						pxy_gen3_data->touch34_id, \
						pxy_gen3_data->x4, pxy_gen3_data->y4, pxy_gen3_data->z4);
				break;
			}
		}
	}
	cyttsp_xdebug("3:prev_touches is %d, curr_touches is %d \n", prev_touches, curr_touches);

	/* process the touches */
	switch (curr_touches) {
		case 4: {
			g_xy_data.x4 = be16_to_cpu(g_xy_data.x4);
			g_xy_data.y4 = be16_to_cpu(g_xy_data.y4);
			if (tilt) { 
				FLIP_XY(g_xy_data.x4, g_xy_data.y4);
			}
			if (reverse_x) {
				g_xy_data.x4 = INVERT_X(g_xy_data.x4, ts->platform_data->maxx);
			}
			if (reverse_y) {
				g_xy_data.y4 = INVERT_X(g_xy_data.y4, ts->platform_data->maxy);
			}
			id = GET_TOUCH4_ID(g_xy_data.touch34_id);
			if (ts->platform_data->use_trk_id) {
				curr_mt_pos[CYTTSP_MT_TOUCH4_IDX][CYTTSP_XPOS] = g_xy_data.x4;
				curr_mt_pos[CYTTSP_MT_TOUCH4_IDX][CYTTSP_YPOS] = g_xy_data.y4;
				curr_mt_z[CYTTSP_MT_TOUCH4_IDX] = g_xy_data.z4;
			}
			else {
				curr_mt_pos[id][CYTTSP_XPOS] = g_xy_data.x4;
				curr_mt_pos[id][CYTTSP_YPOS] = g_xy_data.y4;
				curr_mt_z[id] = g_xy_data.z4;
			}
			curr_mt_touch[CYTTSP_MT_TOUCH4_IDX] = id;
			curr_track[id] = CYTTSP_TOUCH;
			if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
				if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] == id) {
					st_x1 = g_xy_data.x4;
					st_y1 = g_xy_data.y4;
					st_z1 = g_xy_data.z4;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				else if (ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] == id) {
					st_x2 = g_xy_data.x4;
					st_y2 = g_xy_data.y4;
					st_z2 = g_xy_data.z4;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
				}
			}
			cyttsp_xdebug("4th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n\n", \
				g_xy_data.x4, g_xy_data.y4, g_xy_data.z4, (g_xy_data.touch34_id & 0x0F));
			/* do not break */
		}
		case 3: {
			g_xy_data.x3 = be16_to_cpu(g_xy_data.x3);
			g_xy_data.y3 = be16_to_cpu(g_xy_data.y3);
			if (tilt) { 
				FLIP_XY(g_xy_data.x3, g_xy_data.y3);
			}
			if (reverse_x) {
				g_xy_data.x3 = INVERT_X(g_xy_data.x3, ts->platform_data->maxx);
			}
			if (reverse_y) {
				g_xy_data.y3 = INVERT_X(g_xy_data.y3, ts->platform_data->maxy);
			}
			id = GET_TOUCH3_ID(g_xy_data.touch34_id);
			if (ts->platform_data->use_trk_id) {
				curr_mt_pos[CYTTSP_MT_TOUCH3_IDX][CYTTSP_XPOS] = g_xy_data.x3;
				curr_mt_pos[CYTTSP_MT_TOUCH3_IDX][CYTTSP_YPOS] = g_xy_data.y3;
				curr_mt_z[CYTTSP_MT_TOUCH3_IDX] = g_xy_data.z3;
			}
			else {
				curr_mt_pos[id][CYTTSP_XPOS] = g_xy_data.x3;
				curr_mt_pos[id][CYTTSP_YPOS] = g_xy_data.y3;
				curr_mt_z[id] = g_xy_data.z3;
			}
			curr_mt_touch[CYTTSP_MT_TOUCH3_IDX] = id;
			curr_track[id] = CYTTSP_TOUCH;
			if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
				if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] == id) {
					st_x1 = g_xy_data.x3;
					st_y1 = g_xy_data.y3;
					st_z1 = g_xy_data.z3;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				else if (ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] == id) {
					st_x2 = g_xy_data.x3;
					st_y2 = g_xy_data.y3;
					st_z2 = g_xy_data.z3;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
				}
			}
			cyttsp_xdebug("3rd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n", \
				g_xy_data.x3, g_xy_data.y3, g_xy_data.z3, ((g_xy_data.touch34_id >> 4) & 0x0F));
			/* do not break */
		}
		case 2: {
			g_xy_data.x2 = be16_to_cpu(g_xy_data.x2);
			g_xy_data.y2 = be16_to_cpu(g_xy_data.y2);
			if (tilt) {
				FLIP_XY(g_xy_data.x2, g_xy_data.y2);
			}
			if (reverse_x) {
				g_xy_data.x2 = INVERT_X(g_xy_data.x2, ts->platform_data->maxx);
			}
			if (reverse_y) {
				g_xy_data.y2 = INVERT_X(g_xy_data.y2, ts->platform_data->maxy);
			}
			id = GET_TOUCH2_ID(g_xy_data.touch12_id);
			if (ts->platform_data->use_trk_id) {
				curr_mt_pos[CYTTSP_MT_TOUCH2_IDX][CYTTSP_XPOS] = g_xy_data.x2;
				curr_mt_pos[CYTTSP_MT_TOUCH2_IDX][CYTTSP_YPOS] = g_xy_data.y2;
				curr_mt_z[CYTTSP_MT_TOUCH2_IDX] = g_xy_data.z2;
			}
			else {
				curr_mt_pos[id][CYTTSP_XPOS] = g_xy_data.x2;
				curr_mt_pos[id][CYTTSP_YPOS] = g_xy_data.y2;
				curr_mt_z[id] = g_xy_data.z2;
			}
			curr_mt_touch[CYTTSP_MT_TOUCH2_IDX] = id;
			curr_track[id] = CYTTSP_TOUCH;
			if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
				if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] == id) {
					st_x1 = g_xy_data.x2;
					st_y1 = g_xy_data.y2;
					st_z1 = g_xy_data.z2;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				else if (ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] == id) {
					st_x2 = g_xy_data.x2;
					st_y2 = g_xy_data.y2;
					st_z2 = g_xy_data.z2;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
				}
			}
			cyttsp_xdebug("2nd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n", \
				g_xy_data.x2, g_xy_data.y2, g_xy_data.z2, (g_xy_data.touch12_id & 0x0F));
			/* do not break */
		}
		case 1:	{
			g_xy_data.x1 = be16_to_cpu(g_xy_data.x1);
			g_xy_data.y1 = be16_to_cpu(g_xy_data.y1);
			if (tilt) {
				FLIP_XY(g_xy_data.x1, g_xy_data.y1);
			}
			if (reverse_x) {
				g_xy_data.x1 = INVERT_X(g_xy_data.x1, ts->platform_data->maxx);
			}
			if (reverse_y) {
				g_xy_data.y1 = INVERT_X(g_xy_data.y1, ts->platform_data->maxy);
			}
			id = GET_TOUCH1_ID(g_xy_data.touch12_id);
			if (ts->platform_data->use_trk_id) {
				curr_mt_pos[CYTTSP_MT_TOUCH1_IDX][CYTTSP_XPOS] = g_xy_data.x1;
				curr_mt_pos[CYTTSP_MT_TOUCH1_IDX][CYTTSP_YPOS] = g_xy_data.y1;
				curr_mt_z[CYTTSP_MT_TOUCH1_IDX] = g_xy_data.z1;
			}
			else {
				curr_mt_pos[id][CYTTSP_XPOS] = g_xy_data.x1;
				curr_mt_pos[id][CYTTSP_YPOS] = g_xy_data.y1;
				curr_mt_z[id] = g_xy_data.z1;
			}
			curr_mt_touch[CYTTSP_MT_TOUCH1_IDX] = id;
			curr_track[id] = CYTTSP_TOUCH;
			if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
				if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] == id) {
					st_x1 = g_xy_data.x1;
					st_y1 = g_xy_data.y1;
					st_z1 = g_xy_data.z1;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				else if (ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] == id) {
					st_x2 = g_xy_data.x1;
					st_y2 = g_xy_data.y1;
					st_z2 = g_xy_data.z1;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
				}
			}
			cyttsp_xdebug("1st XYZ:% 3d,% 3d,% 3d  ID:% 2d\n", \
				g_xy_data.x1, g_xy_data.y1, g_xy_data.z1, ((g_xy_data.touch12_id >> 4) & 0x0F));
			break;
		}
		case 0:
		default:{
			break;
		}
	}

	/* handle Single Touch signals */
	if (ts->platform_data->use_st) {
		cyttsp_xdebug("ST STEP 0 - ST1 ID=%d  ST2 ID=%d\n", \
			curr_st_touch[CYTTSP_ST_FINGER1_IDX], curr_st_touch[CYTTSP_ST_FINGER2_IDX]);
		if (curr_st_touch[CYTTSP_ST_FINGER1_IDX] > CYTTSP_NUM_TRACK_ID) {
			/* reassign finger 1 and 2 positions to new tracks */
			if (curr_touches > 0) {
				/* reassign st finger1 */
				if (ts->platform_data->use_trk_id) {
					id = CYTTSP_MT_TOUCH1_IDX;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = curr_mt_touch[id];
				}
				else {
					id = GET_TOUCH1_ID(g_xy_data.touch12_id);
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				st_x1 = curr_mt_pos[id][CYTTSP_XPOS];
				st_y1 = curr_mt_pos[id][CYTTSP_YPOS];
				st_z1 = curr_mt_z[id];
				cyttsp_xdebug("ST STEP 1 - ST1 ID=%3d\n", curr_st_touch[CYTTSP_ST_FINGER1_IDX]);
				if (curr_touches > 1) {
					if (curr_st_touch[CYTTSP_ST_FINGER2_IDX] > CYTTSP_NUM_TRACK_ID) {
						/* reassign st finger2 */
						if (curr_touches > 1) {
							if (ts->platform_data->use_trk_id) {
								id = CYTTSP_MT_TOUCH2_IDX;
								curr_st_touch[CYTTSP_ST_FINGER2_IDX] = curr_mt_touch[id];
							}
							else {
								id = GET_TOUCH2_ID(g_xy_data.touch12_id);
								curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
							}
							st_x2 = curr_mt_pos[id][CYTTSP_XPOS];
							st_y2 = curr_mt_pos[id][CYTTSP_YPOS];
							st_z2 = curr_mt_z[id];
							cyttsp_xdebug("ST STEP 2 - ST2 ID=%3d\n", curr_st_touch[CYTTSP_ST_FINGER2_IDX]);
						}
					}
				}
			}
		}
		else if (curr_st_touch[CYTTSP_ST_FINGER2_IDX] > CYTTSP_NUM_TRACK_ID) {
			if (curr_touches > 1) {
				/* reassign st finger2 */
				if (ts->platform_data->use_trk_id) {
					id = CYTTSP_MT_TOUCH2_IDX;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = curr_mt_touch[id];	/* reassign st finger2 */
				}
				else {
					id = GET_TOUCH2_ID(g_xy_data.touch12_id);
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;			/* reassign st finger2 */
				}
				st_x2 = curr_mt_pos[id][CYTTSP_XPOS];
				st_y2 = curr_mt_pos[id][CYTTSP_YPOS];
				st_z2 = curr_mt_z[id];
				cyttsp_xdebug("ST STEP 3 - ST2 ID=%3d\n", curr_st_touch[CYTTSP_ST_FINGER2_IDX]);
			}
		}
		/* if the first touch is missing and there is a second touch, 
		 * then set the first touch to second touch and terminate second touch
		 */
		if ((curr_st_touch[CYTTSP_ST_FINGER1_IDX] > CYTTSP_NUM_TRACK_ID) && 
		    (curr_st_touch[CYTTSP_ST_FINGER2_IDX] < CYTTSP_NUM_TRACK_ID)) {
			st_x1 = st_x2;
			st_y1 = st_y2;
			st_z1 = st_z2;
			curr_st_touch[CYTTSP_ST_FINGER1_IDX] = curr_st_touch[CYTTSP_ST_FINGER2_IDX];
			curr_st_touch[CYTTSP_ST_FINGER2_IDX] = CYTTSP_IGNORE_TOUCH;
		}
		/* if the second touch ends up equal to the first touch, then just report a single touch */
		if (curr_st_touch[CYTTSP_ST_FINGER1_IDX] == curr_st_touch[CYTTSP_ST_FINGER2_IDX]) {
			curr_st_touch[CYTTSP_ST_FINGER2_IDX] = CYTTSP_IGNORE_TOUCH;
		}
		/* set Single Touch current event signals */
		if (curr_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
			input_report_abs(ts->input, ABS_X, st_x1);
			input_report_abs(ts->input, ABS_Y, st_y1);
			input_report_abs(ts->input, ABS_PRESSURE, st_z1);
			input_report_key(ts->input, BTN_TOUCH, CYTTSP_TOUCH);
			input_report_abs(ts->input, ABS_TOOL_WIDTH, curr_tool_width);
			cyttsp_xdebug("ST ->  F1:%3d  X:%3d  Y:%3d  Z:%3d  \n", \
				curr_st_touch[CYTTSP_ST_FINGER1_IDX], st_x1, st_y1, st_z1);
			if (curr_st_touch[CYTTSP_ST_FINGER2_IDX] < CYTTSP_NUM_TRACK_ID) {
				input_report_key(ts->input, BTN_2, CYTTSP_TOUCH);
				input_report_abs(ts->input, ABS_HAT0X, st_x2);
				input_report_abs(ts->input, ABS_HAT0Y, st_y2);
				cyttsp_xdebug("ST ->  F2:%3d  X:%3d  Y:%3d  Z:%3d  \n", \
					curr_st_touch[CYTTSP_ST_FINGER2_IDX], st_x2, st_y2, st_z2);
			}
			else {
				input_report_key(ts->input, BTN_2, CYTTSP_NOTOUCH);
			}
		}
		else {
			input_report_abs(ts->input, ABS_PRESSURE, CYTTSP_NOTOUCH);
			input_report_key(ts->input, BTN_TOUCH, CYTTSP_NOTOUCH);
			input_report_key(ts->input, BTN_2, CYTTSP_NOTOUCH);
		}
		/* update platform data for the current single touch information */
		ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] = curr_st_touch[CYTTSP_ST_FINGER1_IDX];
		ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] = curr_st_touch[CYTTSP_ST_FINGER2_IDX];

	}

	/* handle Multi-touch signals */
	if (ts->platform_data->use_mt) {
		if (ts->platform_data->use_trk_id) {
			/* terminate any previous touch where the track is missing from the current event */
			for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) {
            //printk("[cyttsp_xy_worker]use_trk_id110->id:%d;ts->active_track[id]:%d;curr_track[id]:%d\n",id,ts->active_track[id],curr_track[id]);

				if ((ts->active_track[id] != CYTTSP_NOTOUCH) && (curr_track[id] == CYTTSP_NOTOUCH)) {
					#if 1
					tpd_up(curr_mt_pos[id][CYTTSP_XPOS], curr_mt_pos[id][CYTTSP_YPOS]);
					#else
					//input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
					//input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CYTTSP_NOTOUCH);
					//input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, curr_tool_width);
					//input_report_abs(ts->input, ABS_MT_POSITION_X, curr_mt_pos[id][CYTTSP_XPOS]);
					//input_report_abs(ts->input, ABS_MT_POSITION_Y, curr_mt_pos[id][CYTTSP_YPOS]);
					input_report_key(ts->input, BTN_TOUCH, 0);
					CYTTSP_MT_SYNC(ts->input);
                    
                    tpd_up_eng(curr_mt_pos[id][CYTTSP_XPOS], curr_mt_pos[id][CYTTSP_YPOS]);
					#endif
                    //printk("[cyttsp_xy_worker]use_trk_id111->id:%d active_track:%d curr_track[id]:%d X:%3d Y:%3d \n", \
					//	id, ts->active_track[id], curr_track[id], curr_mt_pos[id][CYTTSP_XPOS], curr_mt_pos[id][CYTTSP_YPOS]);
				}
			}
			/* set Multi-Touch current event signals */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
            	//printk("[cyttsp_xy_worker]use_trk_id221->id:%d;curr_mt_touch[id]:%d\n",id,curr_mt_touch[id]);

				if (curr_mt_touch[id] < CYTTSP_NUM_TRACK_ID) {
					#if 1
					tpd_down(curr_mt_pos[id][CYTTSP_XPOS], curr_mt_pos[id][CYTTSP_YPOS],curr_mt_touch[id]);
					#else
					input_report_key(ts->input, BTN_TOUCH, 1);//LK@
					input_report_abs(ts->input, ABS_MT_TRACKING_ID, curr_mt_touch[id]);//???should add it?
					input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 1);//curr_mt_z[id]
					//input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, curr_tool_width);
					input_report_abs(ts->input, ABS_MT_POSITION_X, 
					curr_mt_pos[id][CYTTSP_XPOS]);
					input_report_abs(ts->input, ABS_MT_POSITION_Y, 
					curr_mt_pos[id][CYTTSP_YPOS]);
					CYTTSP_MT_SYNC(ts->input);
                    
                    tpd_down_eng(curr_mt_pos[id][CYTTSP_XPOS], curr_mt_pos[id][CYTTSP_YPOS]);
					#endif
                    //printk("[cyttsp_xy_worker]use_trk_id222->id:%d curr_mt_touch[id]:%d X:%3d Y:%3d \n", \
					//	id, curr_mt_touch[id], curr_mt_pos[id][CYTTSP_XPOS], curr_mt_pos[id][CYTTSP_YPOS]);

				}
			}
		}
		else {
			/* set temporary track array elements to voids */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				temp_track[id] = CYTTSP_IGNORE_TOUCH;
				send_track[id] = CYTTSP_IGNORE_TOUCH;
			}

			/* get what is currently active */
			for (i = 0, id = 0; id < CYTTSP_NUM_TRACK_ID && i < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				if (curr_track[id] == CYTTSP_TOUCH) {
					temp_track[i] = id;
					i++; /* only increment counter if track found */
				}
			}
			cyttsp_xdebug("T1: t0=%d, t1=%d, t2=%d, t3=%d\n", \
				temp_track[0], temp_track[1], temp_track[2], temp_track[3]);
			cyttsp_xdebug("T1: p0=%d, p1=%d, p2=%d, p3=%d\n", \
				ts->prev_mt_touch[0], ts->prev_mt_touch[1], \
				ts->prev_mt_touch[2], ts->prev_mt_touch[3]);

			/* pack in still active previous touches */
			for (id = 0, prev_touches = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				if (temp_track[id] < CYTTSP_NUM_TRACK_ID) {
					if (cyttsp_inlist(ts->prev_mt_touch, temp_track[id], &loc,
						 CYTTSP_NUM_MT_TOUCH_ID)) {
						loc &= CYTTSP_NUM_MT_TOUCH_ID-1;
						send_track[loc] = temp_track[id];
						prev_touches++;
						cyttsp_xdebug("in list s[%d]=%d t[%d]=%d, loc=%d p=%d\n", \
							loc, send_track[loc], \
							id, temp_track[id], loc, prev_touches);
					}
					else {
						cyttsp_xdebug("is not in list s[%d]=%d t[%d]=%d loc=%d\n", \
							id, send_track[id], id, temp_track[id], loc);
					}
				}
			}
			cyttsp_xdebug("S1: s0=%d, s1=%d, s2=%d, s3=%d p=%d\n", \
				send_track[0], send_track[1], send_track[2], send_track[3], \
				prev_touches);

			/* pack in new touches */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				if (temp_track[id] < CYTTSP_NUM_TRACK_ID) {
					if (!cyttsp_inlist(send_track, temp_track[id], &loc, CYTTSP_NUM_MT_TOUCH_ID)) {
						cyttsp_xdebug("not in list t[%d]=%d, loc=%d\n", \
							id, temp_track[id], loc);
						if (cyttsp_next_avail_inlist(send_track, &loc, CYTTSP_NUM_MT_TOUCH_ID)) {
							loc &= CYTTSP_NUM_MT_TOUCH_ID-1;
							send_track[loc] = temp_track[id];
							cyttsp_xdebug("put in list s[%d]=%d t[%d]=%d\n", 
								loc, send_track[loc], id, temp_track[id]);
						}
					}
					else {
						cyttsp_xdebug("is in list s[%d]=%d t[%d]=%d loc=%d\n", \
							id, send_track[id], id, temp_track[id], loc);
					}
				}
			}
			cyttsp_xdebug("S2: s0=%d, s1=%d, s2=%d, s3=%d\n", \
				send_track[0], send_track[1], send_track[2], send_track[3]);

			/* synchronize motion event signals for each current touch */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				/* z will either be 0 (NOTOUCH) or some pressure (TOUCH) */
				cyttsp_xdebug("MT0 prev[%d]=%d temp[%d]=%d send[%d]=%d\n", \
					id, ts->prev_mt_touch[id], \
					id, temp_track[id], \
					id, send_track[id]);
				cyttsp_xdebug("MT0 curr_mt_z[send_track[%d]=%d\n", \
					id, curr_mt_z[send_track[id]]);
				if (send_track[id] < CYTTSP_NUM_TRACK_ID) {
                if(!((curr_mt_pos[send_track[id]][CYTTSP_XPOS] == 65534) ||(curr_mt_pos[send_track[id]][CYTTSP_YPOS] == 65534)))
                {
                    input_report_key(ts->input, BTN_TOUCH, 1);//LK@
					input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 1);	//curr_mt_z[send_track[id]]
					//input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, curr_tool_width);
					input_report_abs(ts->input, ABS_MT_POSITION_X, curr_mt_pos[send_track[id]][CYTTSP_XPOS]);
					input_report_abs(ts->input, ABS_MT_POSITION_Y, curr_mt_pos[send_track[id]][CYTTSP_YPOS]);
					CYTTSP_MT_SYNC(ts->input);

                    tpd_down_eng(curr_mt_pos[send_track[id]][CYTTSP_XPOS], curr_mt_pos[send_track[id]][CYTTSP_YPOS]);
                                    
					cyttsp_xdebug("MT1->TID:%2d X:%3d Y:%3d Z:%3d act touch-sent\n", \
						send_track[id], curr_mt_pos[send_track[id]][CYTTSP_XPOS], \
						curr_mt_pos[send_track[id]][CYTTSP_YPOS],  curr_mt_z[send_track[id]]);
                }
				}
				else if (ts->prev_mt_touch[id] < CYTTSP_NUM_TRACK_ID) {
                if(!((ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_XPOS] == 65534) ||(ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_YPOS] == 65534)))
                {
					//input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CYTTSP_NOTOUCH);	/* void out this touch */
					//input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, curr_tool_width);
					//input_report_abs(ts->input, ABS_MT_POSITION_X, ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_XPOS]);
					//input_report_abs(ts->input, ABS_MT_POSITION_Y, ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_YPOS]);
					input_report_key(ts->input, BTN_TOUCH, 0);//LK@
					CYTTSP_MT_SYNC(ts->input);
                    
                    tpd_up_eng(ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_XPOS], ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_YPOS]);

					cyttsp_xdebug("MT2->TID:%2d X:%3d Y:%3d Z:%3d lift off-sent\n", \
						ts->prev_mt_touch[id], \
						ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_XPOS], \
						ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_YPOS], \
						CYTTSP_NOTOUCH);
                }
				}
				else {
					/* do not stuff any signals for this previously and currently void touches */

					cyttsp_xdebug("MT3-> send[%d]=%d - No touch - NOT sent\n", \
							id, send_track[id]);
				}
			}

			/* save current posted tracks to previous track memory */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
                if (send_track[id] < CYTTSP_NUM_TRACK_ID) { //LK@the send track id will make buffer incorrect, kernel panic.pay attention.
				ts->prev_mt_touch[id] = send_track[id];
				ts->prev_mt_pos[send_track[id]][CYTTSP_XPOS] = curr_mt_pos[send_track[id]][CYTTSP_XPOS];
				ts->prev_mt_pos[send_track[id]][CYTTSP_YPOS] = curr_mt_pos[send_track[id]][CYTTSP_YPOS];
				cyttsp_xdebug("MT4->TID:%2d X:%3d Y:%3d Z:%3d save for previous\n", \
					send_track[id], \
					ts->prev_mt_pos[send_track[id]][CYTTSP_XPOS], \
					ts->prev_mt_pos[send_track[id]][CYTTSP_YPOS], \
					CYTTSP_NOTOUCH);
                }
			}
		}
	}

	/* handle gestures */
	if (ts->platform_data->use_gestures) {
		if (g_xy_data.gest_id) {
			input_report_key(ts->input, BTN_3, CYTTSP_TOUCH);
			input_report_abs(ts->input, ABS_HAT1X, g_xy_data.gest_id);
			input_report_abs(ts->input, ABS_HAT2Y, g_xy_data.gest_cnt);
		}
	}

	/* signal the view motion event */
	input_sync(ts->input);

	/* update platform data for the current multi-touch information */
	for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) {
		ts->active_track[id] = curr_track[id];
	}

exit_xy_worker:
	if(cyttsp_disable_touch)
	{
		cyttsp_debug("Not enabling touch\n");
	}
	else 
	{
		#ifndef CYTTSP_USE_IRQ
			/* restart event timer */
			mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		#else
			/* re-enable the interrupt after processing */
            cyttsp_xdebug("[cyttso_xy_worker]exit_xy_worker,unmask EINT\n");
            spin_lock_irq(&ts->lock);
			mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
            spin_unlock_irq(&ts->lock);
		#endif
	}
	return;
}

static int cyttsp_inlist(u16 prev_track[], u8 curr_track_id, u8 *prev_loc, u8 num_touches)
{
	u8 id =0;

	*prev_loc = CYTTSP_IGNORE_TOUCH;

		cyttsp_xdebug("IN p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);
	for (id = 0, *prev_loc = CYTTSP_IGNORE_TOUCH;
		(id < num_touches); id++) {
		cyttsp_xdebug("p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);
		if (prev_track[id] == curr_track_id) {
			*prev_loc = id;
			break;
		}
	}
		cyttsp_xdebug("OUT p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);

	return ((*prev_loc < CYTTSP_NUM_TRACK_ID) ? true : false);
}

static int cyttsp_next_avail_inlist(u16 curr_track[], u8 *new_loc, u8 num_touches)
{
	u8 id;

	for (id = 0, *new_loc = CYTTSP_IGNORE_TOUCH;
		(id < num_touches); id++) {
		if (curr_track[id] > CYTTSP_NUM_TRACK_ID) {
			*new_loc = id;
			break;
		}
	}

	return ((*new_loc < CYTTSP_NUM_TRACK_ID) ? true : false);
}

/* Timer function used as dummy interrupt driver */
static void cyttsp_timer(unsigned long handle)
{
	struct cyttsp *ts = this_ts;

	cyttsp_xdebug("TTSP Device timer event\n");

	/* schedule motion signal handling */
	schedule_work(&ts->work);

	return;
}



/* ************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function
 * ************************************************************************ */
static void cyttsp_irq(void)
{
	struct cyttsp *ts = this_ts;

	/* disable further interrupts until this interrupt is processed */
	//disable_irq_nosync(ts->client->irq);
	spin_lock_irq(&ts->lock);
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	spin_unlock_irq(&ts->lock);

	/* schedule motion signal handling */
	schedule_work(&ts->work);
}

/* ************************************************************************
 * Probe initialization functions
 * ************************************************************************ */
static int cyttsp_putbl(struct cyttsp *ts, int show, int show_status, int show_version, int show_cid)
{
	int retval = CYTTSP_OPERATIONAL;

	int num_bytes = (show_status * 3) + (show_version * 6) + (show_cid * 3);

	if (show_cid) {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t);
	}
	else if (show_version) {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t) - 3;
	}
	else {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t) - 9;
	}

	if (show) {
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			8, (u8 *)&g_bl_data);
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+8,
			8, (u8 *)&(g_bl_data.ttspver_lo));
		if (show_status) {
			cyttsp_debug("BL%d: f=%02X s=%02X err=%02X bl=%02X%02X bld=%02X%02X\n", \
				show, \
				g_bl_data.bl_file, g_bl_data.bl_status, g_bl_data.bl_error, \
				g_bl_data.blver_hi, g_bl_data.blver_lo, \
				g_bl_data.bld_blver_hi, g_bl_data.bld_blver_lo);
		}
		if (show_version) {
			cyttsp_debug("BL%d: ttspver=0x%02X%02X appid=0x%02X%02X appver=0x%02X%02X\n", \
				show, \
				g_bl_data.ttspver_hi, g_bl_data.ttspver_lo, \
				g_bl_data.appid_hi, g_bl_data.appid_lo, \
				g_bl_data.appver_hi, g_bl_data.appver_lo);
		}
		if (show_cid) {
			cyttsp_debug("BL%d: cid=0x%02X%02X%02X\n", \
				show, \
				g_bl_data.cid_0, g_bl_data.cid_1, g_bl_data.cid_2);
		}
		mdelay(CYTTSP_DELAY_DFLT);
	}

	return retval;
}

#ifdef CYTTSP_INCLUDE_LOAD_FILE
#define CYTTSP_MAX_I2C_LEN	256
#define CYTTSP_MAX_TRY		10
#define CYTTSP_BL_PAGE_SIZE	16
#define CYTTSP_BL_NUM_PAGES	5
static int cyttsp_i2c_write_block_data(struct i2c_client *client, u8 command,
			       u8 length, const u8 *values)
{
	int retval = CYTTSP_OPERATIONAL;

	u8 dataray[CYTTSP_MAX_I2C_LEN];
	u8 try;
	dataray[0] = command;
	if (length) {
		memcpy(&dataray[1], values, length);
	}

	try = CYTTSP_MAX_TRY;
	do {
		retval = i2c_master_send(client, dataray, length+1);
		mdelay(CYTTSP_DELAY_DFLT*2);
	}
	while ((retval != length+1) && try--);

	return retval;
}

static int cyttsp_i2c_write_block_data_chunks(struct cyttsp *ts, u8 command,
			       u8 length, const u8 *values)
{
	int retval = CYTTSP_OPERATIONAL;
	int block = 1;

    u8 i,j;
       
	u8 dataray[CYTTSP_MAX_I2C_LEN];
      
    //printk("[cyttsp_i2c_write_block_data_chunks]000:length=%x\n",length);

    dataray[0] = CYTTSP_REG_BASE;
	/* first page already includes the bl page offset */
    memcpy(&dataray[1], values, CYTTSP_BL_PAGE_SIZE+1);
    
    //for(i=0;i<CYTTSP_BL_PAGE_SIZE+2;i++)
    //       printk("[cyttsp_i2c_write_block_data_chunks]000:dataray[%d]=%x\n",i,dataray[i]);
    for(i=0;i<3;i++)
    {
        retval = i2c_master_send(ts->client, dataray, CYTTSP_BL_PAGE_SIZE+2);
        //printk("[cyttsp_i2c_write_block_data_chunks]000:retval=%d;i=%d\n",retval,i);

        if(retval >= 0)
            break;
    }
    //printk("[cyttsp_i2c_write_block_data_chunks]111:retval=%d\n",retval);
     
	mdelay(10);
	values += CYTTSP_BL_PAGE_SIZE+1;
	length -= CYTTSP_BL_PAGE_SIZE+1;

       //printk("[cyttsp_i2c_write_block_data_chunks]222:length=%x,block=%x,retval=%d\n",length,block,retval);

	/* rem blocks require bl page offset stuffing */
	while (length && (block < CYTTSP_BL_NUM_PAGES) && !(retval < CYTTSP_OPERATIONAL)) {
                  //printk("[cyttsp_i2c_write_block_data_chunks]333:length=%x,block=%x,retval=%d\n",length,block,retval);

            dataray[0] = CYTTSP_REG_BASE;
            dataray[1] = CYTTSP_BL_PAGE_SIZE*block;

            memcpy(&dataray[2], values, 
            length >= CYTTSP_BL_PAGE_SIZE ? CYTTSP_BL_PAGE_SIZE : length);
            for(i=0;i<3;i++)
            {
                retval = i2c_master_send(ts->client, dataray, (length >= CYTTSP_BL_PAGE_SIZE ? CYTTSP_BL_PAGE_SIZE+2 : length+2));
                //printk("[cyttsp_i2c_write_block_data_chunks]444:retval=%d;i=%d\n",retval,i);

                if(retval >= 0)
                    break;

            }
            mdelay(1);//LK@modify 10->1
            values += CYTTSP_BL_PAGE_SIZE;
            length = (length >= CYTTSP_BL_PAGE_SIZE ? length - CYTTSP_BL_PAGE_SIZE : 0);
            block++;
	}

	return retval;
}

static int cyttsp_bootload_app(struct cyttsp *ts)
{
	int retval = CYTTSP_OPERATIONAL;
	int i, j, tries = 0;
	u8 host_reg;

	printk("[cyttsp_bootload_app]load new firmware \n");
	/* reset TTSP Device back to bootloader mode */
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
		sizeof(host_reg), &host_reg);
	/* wait for TTSP Device to complete reset back to bootloader */
//	mdelay(CYTTSP_DELAY_DFLT);
	do {
		mdelay(100);
		cyttsp_putbl(ts, 3, true, false, false);
	} while (g_bl_data.bl_status != 0x10 &&
		g_bl_data.bl_status != 0x11 &&
		tries++ < 20);
    
	cyttsp_debug("load file -- tts_ver=0x%02X%02X  app_id=0x%02X%02X  app_ver=0x%02X%02X\n", \
		cyttsp_fw_tts_verh, cyttsp_fw_tts_verl, \
		cyttsp_fw_app_idh, cyttsp_fw_app_idl, \
		cyttsp_fw_app_verh, cyttsp_fw_app_verl);

	/* download new TTSP Application to the Bootloader
	 *
	 */
        if (!(retval < CYTTSP_OPERATIONAL)) {
            i = 0;
            /* send bootload initiation command */
            if (cyttsp_fw[i].Command == CYTTSP_BL_INIT_LOAD) {
                g_bl_data.bl_file = 0;
                g_bl_data.bl_status = 0;
                g_bl_data.bl_error = 0;

                for(j=0; j<3; j++)
                {
                    retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, cyttsp_fw[i].Length, cyttsp_fw[i].Block);
                    tries = 0;
                    do {
                        mdelay(1000);
                        cyttsp_putbl(ts,4, true, false, false);
                    }
                    while (g_bl_data.bl_error != 0x20 && tries++ < 12);  //max 12s  

                    if(g_bl_data.bl_error == 0x20)
                        break;

                }
                cyttsp_debug("BL CMD=0x%3x, DNLD Rec=% 3d, Len=% 3d, Addr=%04X, TryCount=%d,tries=%d\n", 
                    cyttsp_fw[i].Command,cyttsp_fw[i].Record, cyttsp_fw[i].Length, cyttsp_fw[i].Address,j,tries);	

                /* delay to allow bootloader to get ready for block writes */
                i++;
                tries = 0;

			/* send bootload firmware load blocks - 
			 * kernel limits transfers to I2C_SMBUS_BLOCK_MAX(32) bytes
			 */
                if (!(retval < CYTTSP_OPERATIONAL)) {
                    while (cyttsp_fw[i].Command == CYTTSP_BL_WRITE_BLK) {
                        for(j=0; j<3; j++)
                        {
                            retval = cyttsp_i2c_write_block_data_chunks(ts,CYTTSP_REG_BASE, cyttsp_fw[i].Length, cyttsp_fw[i].Block);
                            tries = 0;
                            /* bootloader requires delay after odd block addresses */
                            do{
                                mdelay(10);
                                cyttsp_putbl(ts,5, false, false, false);//close this log, if you want to debug please open this log cyttsp_putbl(ts,5, true, false, false)
                            }
                            while (g_bl_data.bl_error != 0x20 && tries++ < 10);   //max 100ms                

                            if(g_bl_data.bl_error == 0x20)
                                break;
                        }
                        cyttsp_debug("BL CMD=0x%3x, DNLD Rec=% 3d, Len=% 3d, Addr=%04X, TryCount=%d, tries=%d\n", 
                            cyttsp_fw[i].Command,cyttsp_fw[i].Record, cyttsp_fw[i].Length, cyttsp_fw[i].Address,j,tries);	

                        i++;
                        if (retval < CYTTSP_OPERATIONAL) {
                            cyttsp_debug("BL fail Rec=%3d retval=%d\n",cyttsp_fw[i-1].Record, retval);
                            break;
                        }
                        else {
                            /* reset TTSP I2C counter */
                            retval = cyttsp_i2c_write_block_data(ts->client,CYTTSP_REG_BASE, 0, NULL);
                            tries = 0;
                            do{
                                mdelay(10);
                                /* set arg2 to non-0 to activate */
                                cyttsp_putbl(ts,6, true, false, false);
                            }
                            while (g_bl_data.bl_status != 0x10 && 
                                g_bl_data.bl_status != 0x11 && 
                                tries++ < 10);
                        }
                    }
                    if (!(retval < CYTTSP_OPERATIONAL)) {
                        while (i < cyttsp_fw_records) {
                            for(j=0; j<3; j++)
                            {
                                retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
                                    cyttsp_fw[i].Length, cyttsp_fw[i].Block);
                            
                                tries = 0;
                                do {
                                    mdelay(1000);
                                    cyttsp_putbl(ts,7, true, false, false);
                                }
                                while (g_bl_data.bl_status != 0x10 && 
                                    g_bl_data.bl_status != 0x11 && 
                                    tries++ < 12);

                                if((g_bl_data.bl_status == 0x10) || (g_bl_data.bl_status == 0x11))
                                    break;
                            }
                            cyttsp_debug("BL CMD=0x%3x, DNLD Rec=% 3d, Len=% 3d, Addr=%04X,TryCount=%d, tries=%d\n", 
                                cyttsp_fw[i].Command,cyttsp_fw[i].Record, cyttsp_fw[i].Length, cyttsp_fw[i].Address,j,tries);						

                            i++;

                            if (retval < CYTTSP_OPERATIONAL) {
                                break;
                            }
                        }
                    }
                }
            }
        }

	/* Do we need to reset TTSP Device back to bootloader mode?? */
	/*
	*/
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
		sizeof(host_reg), &host_reg);
	/* wait for TTSP Device to complete reset back to bootloader */
	/*
	*/
        tries = 0;
        do {
            mdelay(100);
            cyttsp_putbl(ts,8, true, true, true);
        }
        while (g_bl_data.bl_status != 0x10 && 
            g_bl_data.bl_status != 0x11 && 
            tries++ < 20);
	/* set arg2 to non-0 to activate */
    
       for(j = 0; j < 3; j++)
       {
            if(!((cyttsp_fw_app_verh == g_bl_data.appver_hi) && (cyttsp_fw_app_verl == g_bl_data.appver_lo)))
            {
                     mdelay(1);
                	retval = cyttsp_putbl(ts, 9, true, true, true);
            }
            else
            {
                break;
            }
       }
        
       cyttsp_app_ver = ((g_bl_data.appver_hi <<8) | g_bl_data.appver_lo);//LK@add
       
       //LK@add to check the downloaded firmware version is the same as the fw file?
       if(!(retval < CYTTSP_OPERATIONAL))
       {
           if(!((cyttsp_fw_app_verh == g_bl_data.appver_hi) && (cyttsp_fw_app_verl == g_bl_data.appver_lo)))
           {
               printk("[cyttsp_bootload_app]cyttsp_fw_app_verh=%x,cyttsp_fw_app_verl=%x\n", cyttsp_fw_app_verh, cyttsp_fw_app_verl);
               printk("[cyttsp_bootload_app]g_bl_data.appver_hi=%x,g_bl_data.appver_lo=%x\n", g_bl_data.appver_hi, g_bl_data.appver_lo);

               retval = -1;
           }
       }
        
	return retval;
}
#else 
static int cyttsp_bootload_app(struct cyttsp *ts)
{
	cyttsp_debug("no-load new firmware \n");
	return CYTTSP_OPERATIONAL;
}
#endif /* CYTTSP_INCLUDE_LOAD_FILE */

#if defined(__TPD_FW_UPDATE_THREAD__)
static struct task_struct *tpd_fw_update_thread = NULL;
static int tpd_fw_update_flag = 0;
static int tpd_fw_update_preflag = 0;
static int tpd_fw_update_complete = 0;
static DECLARE_WAIT_QUEUE_HEAD(tpd_fw_update_waiter);

#ifdef MODTIMER
struct timer_list   tpd_fw_update_timer; 
#define CHECK_FWUPDATE_TIMER_COUNT   (2*HZ)    // 2s, in power on period, it's precise enough
#endif
static void tpd_set_pretrigger(void)
{

    printk("[tpd_set_pretrigger] \n");

    tpd_fw_update_preflag = 1;
	
}

static void tpd_clear_pretrigger(void)
{

    printk("[tpd_clear_pretrigger] \n");

    tpd_fw_update_preflag = 0;
	
}


static int tpd_check_pretrigger(void)
{

    printk("[tpd_check_pretrigger] \n");

    return  tpd_fw_update_preflag;
	
}


static void tpd_fw_update_trigger(void)
{
    printk("[tpd_fw_update_trigger] \n");

    tpd_fw_update_flag = 1; 
    tpd_clear_pretrigger();
    wake_up_interruptible(&tpd_fw_update_waiter);
   
}

static int tpd_fw_update_thread_handler(void *unused)
{
    int retval = 0;
    int error = 0;
    int i;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

    printk("[tpd_fw_update_thread_handler] \n");
    do
    {
  
       set_current_state(TASK_INTERRUPTIBLE);
       wait_event_interruptible(tpd_fw_update_waiter, tpd_fw_update_flag != 0);
    	tpd_fw_update_flag = 0;
       set_current_state(TASK_RUNNING);
       if(0 == tpd_fw_update_complete)
       {
           printk("[tpd_fw_update_thread_handler], tpd_fw_update_complete=%d \n",tpd_fw_update_complete);

           tpd_fw_update_complete = 1;

           #ifdef CYTTSP_USE_IRQ
	    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
           #endif

           for(i=0; i<2; i++)
           {
               error = cyttsp_bootload_app(this_ts);
               if(error >= 0)
               {
                    printk("[tpd_fw_update_thread_handler]cyttsp_bootload_app complete, i=%d!! \n",i);
                    //go out of bootloader mode
                    #if 1
                    error = i2c_smbus_write_i2c_block_data(this_ts->client, CYTTSP_REG_BASE, sizeof(bl_cmd), bl_cmd);
                    #else
                    error = i2c_smbus_write_i2c_block_data(this_ts->client, CYTTSP_REG_BASE+4, 4, bl_cmd+4);
                    if (error < 0)
                        cyttsp_debug("error:%d, exit bootloader failed!!! \n", error);
                    error = i2c_smbus_write_i2c_block_data(this_ts->client, CYTTSP_REG_BASE+8, 3, bl_cmd+8);
                    if (error < 0)
                        cyttsp_debug("error:%d, exit bootloader failed!!! \n", error);
                    error = i2c_smbus_write_i2c_block_data(this_ts->client, CYTTSP_REG_BASE, 4, bl_cmd);
                    #endif
                    if (error < 0)
                        cyttsp_debug("error:%d, exit bootloader failed!!! \n", error);

                    /* wait for TTSP Device to complete switch to Operational mode */
                    mdelay(500);
                    
                    break;
                }
                else
                {
                    printk("[tpd_fw_update_thread_handler]cyttsp_bootload_app error, i=%d!! \n",i);
                }
            }
            #ifdef CYTTSP_USE_IRQ
	     mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
            #endif

        }
       else
       {
           printk("[tpd_fw_update_thread_handler], stop to update fw: tpd_fw_update_complete=%d \n",tpd_fw_update_complete);
       }
    } while (!kthread_should_stop());
    return 0;
}


#ifdef MODTIMER
void tpd_fw_update_timer_callback(unsigned long arg)
{
        tpd_fw_update_flag = 1; 
        wake_up_interruptible(&tpd_fw_update_waiter);

        printk("[tpd_fw_update_timer_callback]\n");

}
#endif

static void tpd_fw_update_thread_init(void)
{

#ifdef MODTIMER
    init_timer(&tpd_fw_update_timer);
    tpd_fw_update_timer.expires	= jiffies + CHECK_FWUPDATE_TIMER_COUNT;
    tpd_fw_update_timer.function = tpd_fw_update_timer_callback;
    tpd_fw_update_timer.data = ((unsigned long) 0 );
    add_timer(&tpd_fw_update_timer);
    //mod_timer(&tpd_fw_update_timer, jiffies + CHECK_FWUPDATE_TIMER_COUNT);
#endif

    tpd_fw_update_thread = kthread_run(tpd_fw_update_thread_handler, 0, "tpd_fw_update");
    if (IS_ERR(tpd_fw_update_thread))
    {
        printk("[%s]: failed to create tpd_fw_update_thread_init thread\n", __FUNCTION__);
    }

	printk("[tpd_fw_update_thread_init] : done\n" );
}
#endif

unsigned int tpd_cyttsp_version_check()
{
    return cyttsp_app_ver;
}

static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval = CYTTSP_OPERATIONAL;
	u8 host_reg;
	int tries;
	int mod, num, tempi=0;
       unsigned int i;

		hwPowerDown(TPD_POWER_SOURCE,"TP");//this line will affect fw update,should add it
		msleep(100);
		hwPowerOn(TPD_POWER_SOURCE,VOL_2800,"TP");
		msleep(100);

		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		
		mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_CTP_RST_PIN, GPIO_PULL_DOWN);
		
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		msleep(10);   
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(30);//LK@10->30
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		msleep(100);

		mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

       for(i=0; i<5; i++)
       {
	    retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);
	    if (retval > 0)
	    {
		printk("[cyttsp_power_on] check i2c read success, retval=%d,i=%d \n", retval,i);
              break;
           }
	    else
	    {
		printk("[cyttsp_power_on] check i2c read failed, retval=%d,i=%d\n", retval,i);    
              if(i == 4)
                  return retval;
           }
        
        }
	/* check if the TTSP device has a bootloader installed */
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
		sizeof(host_reg), &host_reg);
	tries = 0;
	do {
		mdelay(100);

		/* set arg2 to non-0 to activate */
		retval = cyttsp_putbl(ts, 1, true, true, true);
		printk("[cyttsp_power_on] retval = %d *****1111***\n", retval);
	}
	while (!(retval < CYTTSP_OPERATIONAL) &&
		!GET_BOOTLOADERMODE(g_bl_data.bl_status) && 
		!(g_bl_data.bl_file == CYTTSP_OPERATE_MODE + CYTTSP_LOW_POWER_MODE) &&
		tries++ < 10);
    
       cyttsp_app_ver = ((g_bl_data.appver_hi <<8) | g_bl_data.appver_lo);//LK@add

	   printk("[cyttsp_power_on]tries = %d;cyttsp_app_ver=%x\n", tries,cyttsp_app_ver);
	/* is bootloader missing? */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_xdebug("Retval=%d  Check if bootloader is missing...\n", retval);
		if (!GET_BOOTLOADERMODE(g_bl_data.bl_status)) {
			/* skip all bootloader and sys info and go straight to operational mode */
			if (!(retval < CYTTSP_OPERATIONAL)) {
				cyttsp_xdebug("Bootloader is missing (retval = %d)\n", retval);
				host_reg = CYTTSP_OPERATE_MODE/* + CYTTSP_LOW_POWER_MODE*/;
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
					sizeof(host_reg), &host_reg);
				/* wait for TTSP Device to complete switch to Operational mode */
				mdelay(500);//LK@1000->500
				goto bypass;//if no bootloader, go to operation mode directly
			}
		}
	}

#if 0//LK@no to go out of bootloader, check if we should load firmware first 
	/* take TTSP out of bootloader mode; go to TrueTouch operational mode */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_xdebug("exit bootloader; go operational\n");
		//retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(bl_cmd), bl_cmd);
		//retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 8, bl_cmd);
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+4, 4, bl_cmd+4);
		if (retval < 0)
			cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+8, 3, bl_cmd+8);
		if (retval < 0)
			cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 4, bl_cmd);
		if (retval < 0)
			cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
		
		tries = 0;
		do {
			mdelay(1000);
			cyttsp_putbl(ts,4, true, false, false);
			cyttsp_info("BL%d: f=%02X s=%02X err=%02X bl=%02X%02X bld=%02X%02X\n", \
				104, \
				g_bl_data.bl_file, g_bl_data.bl_status, g_bl_data.bl_error, \
				g_bl_data.blver_hi, g_bl_data.blver_lo, \
				g_bl_data.bld_blver_hi, g_bl_data.bld_blver_lo);
		}
		while (GET_BOOTLOADERMODE(g_bl_data.bl_status) && 
			tries++ < 10);
	}
#endif


	if (!(retval < CYTTSP_OPERATIONAL) &&
		cyttsp_app_load()) {
		//mdelay(1000);//LK@mask
		/*if (CYTTSP_DIFF(g_bl_data.ttspver_hi, cyttsp_tts_verh())  ||
			CYTTSP_DIFF(g_bl_data.ttspver_lo, cyttsp_tts_verl())  ||
			CYTTSP_DIFF(g_bl_data.appid_hi, cyttsp_app_idh())  ||
			CYTTSP_DIFF(g_bl_data.appid_lo, cyttsp_app_idl())  ||
			CYTTSP_DIFF(g_bl_data.appver_hi, cyttsp_app_verh())  ||
			CYTTSP_DIFF(g_bl_data.appver_lo, cyttsp_app_verl())  ||
			CYTTSP_DIFF(g_bl_data.cid_0, cyttsp_cid_0())  ||
			CYTTSP_DIFF(g_bl_data.cid_1, cyttsp_cid_1())  ||
			CYTTSP_DIFF(g_bl_data.cid_2, cyttsp_cid_2())  ||
			cyttsp_force_fw_load()) */
             #ifdef CYTTSP_INCLUDE_LOAD_FILE
             #if defined(ACER_Z1)
             //bootloader ver will not be erased when fw update fail, but app ver will be erased.
             if((g_bl_data.appver_hi == cyttsp_fw_app_verh_ofilm) ||
                (g_bl_data.blver_hi == cyttsp_fw_app_verh_ofilm))
             {
                 cyttsp_fw_app_verh = cyttsp_fw_app_verh_ofilm;
                 cyttsp_fw_app_verl = cyttsp_fw_app_verl_ofilm;

             }
             #endif
             #endif
            cyttsp_debug("TP IC Bootloader ver=0x%x, TP IC appver =0x%x, FW file appver=0x%x \n", \
		((g_bl_data.blver_hi <<8) | g_bl_data.blver_lo),((g_bl_data.appver_hi <<8) | g_bl_data.appver_lo), ((cyttsp_fw_app_verh<<8) | cyttsp_fw_app_verl));

             if((((g_bl_data.appver_hi <<8) | g_bl_data.appver_lo) < ((cyttsp_fw_app_verh<<8) | cyttsp_fw_app_verl)) ||
                 (((g_bl_data.appver_hi <<8) | g_bl_data.appver_lo) == APP_ERROR_CODE) ||
                    cyttsp_force_fw_load()) 
                  {
                     #ifdef CYTTSP_INCLUDE_LOAD_FILE
                     #if defined(ACER_Z1)
                     //bootloader ver will not be erased when fw update fail, but app ver will be erased.
                     if((g_bl_data.appver_hi == cyttsp_fw_app_verh_ofilm) ||
                        (g_bl_data.blver_hi == cyttsp_fw_app_verh_ofilm))
                     {
                         memset(cyttsp_fw, 0x00, sizeof(cyttsp_fw));
                         memcpy(cyttsp_fw, cyttsp_fw_ofilm, sizeof(cyttsp_fw));
                     }
                    #endif
                    #endif
                    
			cyttsp_debug("blttsp=0x%02X%02X flttsp=0x%02X%02X force=%d\n", \
				g_bl_data.ttspver_hi, g_bl_data.ttspver_lo, \
				cyttsp_tts_verh(), cyttsp_tts_verl(), cyttsp_force_fw_load());
			cyttsp_debug("blappid=0x%02X%02X flappid=0x%02X%02X\n", \
				g_bl_data.appid_hi, g_bl_data.appid_lo, \
				cyttsp_app_idh(), cyttsp_app_idl());
			cyttsp_debug("blappver=0x%02X%02X flappver=0x%02X%02X\n", \
				g_bl_data.appver_hi, g_bl_data.appver_lo, \
				cyttsp_app_verh(), cyttsp_app_verl());
			cyttsp_debug("blcid=0x%02X%02X%02X flcid=0x%02X%02X%02X\n", \
				g_bl_data.cid_0, g_bl_data.cid_1, g_bl_data.cid_2, \
				cyttsp_cid_0(), cyttsp_cid_1(), cyttsp_cid_2());
			/* enter bootloader to load new app into TTSP Device */
			//retval = cyttsp_bootload_app(ts);
            //cyttsp_debug("cyttsp_bootload_app retval=%d! \n", retval);
            #if defined(__TPD_FW_UPDATE_THREAD__)
            tpd_fw_update_thread_init();
            //tpd_set_pretrigger();//use modtimer,mask this line
            #endif

            #if 0//LK@,remove to tpd_fw_update_thread_handler
			/* take TTSP device out of bootloader mode; switch back to TrueTouch operational mode */
			if (!(retval < CYTTSP_OPERATIONAL)) {
				//retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(bl_cmd), bl_cmd);
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+4, 4, bl_cmd+4);
				if (retval < 0)
					cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+8, 3, bl_cmd+8);
				if (retval < 0)
					cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 4, bl_cmd);
				if (retval < 0)
					cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);
				
				/* wait for TTSP Device to complete switch to Operational mode */
				mdelay(1000);
			}
            #endif
             }
             else//app ver is the newest, no need to update, exit bootloader mode
             {
            		cyttsp_debug("app ver is the newest, no need to update, exit bootloader mode! \n", retval);				

			/* take TTSP device out of bootloader mode; switch back to TrueTouch operational mode */
			if (!(retval < CYTTSP_OPERATIONAL)) {
               		cyttsp_debug("exit bootloader; go operational\n");
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(bl_cmd), bl_cmd);
				if (retval < 0)
					cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);				
				/* wait for TTSP Device to complete switch to Operational mode */
				mdelay(500);
                }

             }
	}
    else//cyttsp_app_load()==0
    {
        cyttsp_debug("cyttsp_app_load()=0\n");
		/* take TTSP device out of bootloader mode; switch back to TrueTouch operational mode */
		if (!(retval < CYTTSP_OPERATIONAL)) {
            cyttsp_debug("exit bootloader; go operational\n");
		    retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(bl_cmd), bl_cmd);
		    if (retval < 0)
				cyttsp_debug("retval:%d, exit bootloader failed!!! \n", retval);				
		    /* wait for TTSP Device to complete switch to Operational mode */
		    mdelay(500);
		}
    }

bypass:
	/* switch to System Information mode to read versions and set interval registers */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_debug("switch to sysinfo mode \n");
		host_reg = CYTTSP_SYSINFO_MODE;
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
			sizeof(host_reg), &host_reg);
		/* wait for TTSP Device to complete switch to SysInfo mode */
		mdelay(500);
		if (!(retval < CYTTSP_OPERATIONAL)) {
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
				8, (u8 *)&g_sysinfo_data);
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+8, 
				8, (u8 *)&g_sysinfo_data+8);
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+16, 
				8, (u8 *)&g_sysinfo_data+16);
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+24, 
				8, (u8 *)&g_sysinfo_data+24);
			cyttsp_debug("SI2: hst_mode=0x%02X mfg_cmd=0x%02X mfg_stat=0x%02X\n", \
				g_sysinfo_data.hst_mode, g_sysinfo_data.mfg_cmd, \
				g_sysinfo_data.mfg_stat);
			cyttsp_debug("SI2: bl_ver=0x%02X%02X\n", \
				g_sysinfo_data.bl_verh, g_sysinfo_data.bl_verl);
			cyttsp_debug("SI2: sysinfo act_int=0x%02X tch_tmout=0x%02X lp_int=0x%02X\n", \
				g_sysinfo_data.act_intrvl, g_sysinfo_data.tch_tmout, \
				g_sysinfo_data.lp_intrvl);
			cyttsp_info("SI%d: tver=%02X%02X a_id=%02X%02X aver=%02X%02X\n", \
				102, \
				g_sysinfo_data.tts_verh, g_sysinfo_data.tts_verl, \
				g_sysinfo_data.app_idh, g_sysinfo_data.app_idl, \
				g_sysinfo_data.app_verh, g_sysinfo_data.app_verl);
			cyttsp_info("SI%d: c_id=%02X%02X%02X\n", \
				103, \
				g_sysinfo_data.cid[0], g_sysinfo_data.cid[1], g_sysinfo_data.cid[2]);
			if (!(retval < CYTTSP_OPERATIONAL) &&
				(CYTTSP_DIFF(ts->platform_data->act_intrvl, CYTTSP_ACT_INTRVL_DFLT)  ||
				CYTTSP_DIFF(ts->platform_data->tch_tmout, CYTTSP_TCH_TMOUT_DFLT) ||
				CYTTSP_DIFF(ts->platform_data->lp_intrvl, CYTTSP_LP_INTRVL_DFLT))) {
				if (!(retval < CYTTSP_OPERATIONAL)) {
					u8 intrvl_ray[sizeof(ts->platform_data->act_intrvl) + 
						sizeof(ts->platform_data->tch_tmout) + 
						sizeof(ts->platform_data->lp_intrvl)];
					u8 i = 0;

					intrvl_ray[i++] = ts->platform_data->act_intrvl;
					intrvl_ray[i++] = ts->platform_data->tch_tmout;
					intrvl_ray[i++] = ts->platform_data->lp_intrvl;

					cyttsp_debug("SI2: platinfo act_intrvl=0x%02X tch_tmout=0x%02X lp_intrvl=0x%02X\n", \
						ts->platform_data->act_intrvl, ts->platform_data->tch_tmout, \
						ts->platform_data->lp_intrvl);
					// set intrvl registers
					//retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_ACT_INTRVL, 
						//sizeof(intrvl_ray), intrvl_ray);
					num = sizeof(intrvl_ray) / 4;
					mod = sizeof(intrvl_ray) % 4;

					while(num > 0) {
						retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+tempi, 
												4, intrvl_ray+tempi);
						if (retval < 0)
							cyttsp_debug("retval:%d, set intrvl registers failed!!! \n", retval);
						num--;
						tempi+=4;
					}
					if(mod != 0) {
						retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+tempi, 
												mod, intrvl_ray+tempi);
						if (retval < 0)
							cyttsp_debug("retval:%d, set intrvl registers failed!!! \n", retval);
					}

					mdelay(CYTTSP_DELAY_SYSINFO);
				}
			}
		}
		/* switch back to Operational mode */
		if (!(retval < CYTTSP_OPERATIONAL)) {
			cyttsp_debug("switch back to operational mode \n");
			host_reg = CYTTSP_OPERATE_MODE/* + CYTTSP_LOW_POWER_MODE*/;
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
				sizeof(host_reg), &host_reg);
			/* wait for TTSP Device to complete switch to Operational mode */
			mdelay(500);
            
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);
			if (retval > 0)
				printk("cyttsp_power_on test read success, retval is 0x%x, host_mode is %d \n", retval, host_reg);
			else
				printk("cyttsp_power_on test read failed, retval is %d\n", retval);

			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+1, sizeof(host_reg), &host_reg);
			if (retval > 0)
				printk("cyttsp_power_on test read success, retval is 0x%x, tt_mode is %d \n", retval, host_reg);
			else
				printk("cyttsp_power_on test read failed, retval is %d\n", retval);
		}
	}
	/* init gesture setup; 
	 * this is required even if not using gestures
	 * in order to set the active distance */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		u8 gesture_setup;
		cyttsp_debug("init gesture setup \n");
		gesture_setup = ts->platform_data->gest_set;
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_GEST_SET, 
			sizeof(gesture_setup), &gesture_setup);
		mdelay(CYTTSP_DELAY_DFLT);
	}

	if (!(retval < CYTTSP_OPERATIONAL)) {
		ts->platform_data->power_state = CYTTSP_ACTIVE_STATE;
	}
	else {
		ts->platform_data->power_state = CYTTSP_IDLE_STATE;
	}
	cyttsp_debug("Retval=%d Power state is %s\n", retval, (ts->platform_data->power_state == CYTTSP_ACTIVE_STATE) ? "ACTIVE" : "IDLE");

      #if 0//defined(__TPD_FW_UPDATE_THREAD__)//use modtimer, mask this line
      if(1 == tpd_check_pretrigger())
      {
          tpd_fw_update_trigger();
      }
      #endif
	return retval;
}

/* cyttsp_initialize: Driver Initialization. This function takes
 * care of the following tasks:
 * 1. Create and register an input device with input layer
 * 2. Take CYTTSP device out of bootloader mode; go operational
 * 3. Start any timers/Work queues.  */
static int cyttsp_initialize(struct i2c_client *client, struct cyttsp *ts)
{
	int retval = CYTTSP_OPERATIONAL;
	u8 id;	

	//ts->input = tpd->dev;
	//tpd->dev->phys = ts->phys;
	//tpd->dev->dev.parent = &client->dev;

	/* init the touch structures */
	ts->num_prev_st_touch = CYTTSP_NOTOUCH;
	for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) {
		ts->active_track[id] = CYTTSP_NOTOUCH;
	}
	for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
		ts->prev_mt_touch[id] = CYTTSP_IGNORE_TOUCH;
		ts->prev_mt_pos[id][CYTTSP_XPOS] = 0;
		ts->prev_mt_pos[id][CYTTSP_YPOS] = 0;
	}
	for (id = 0; id < CYTTSP_NUM_ST_TOUCH_ID; id++) {
		ts->prev_st_touch[id] = CYTTSP_IGNORE_TOUCH;
	}
	
	/* Prepare our worker structure prior to setting up the timer/ISR */
	//INIT_WORK(&ts->work,cyttsp_xy_worker);
	//spin_lock_init(&ts->lock);

	/* Power on the chip and make sure that I/Os are set as specified
	 * in the platform 
	 */
	retval = cyttsp_power_on(ts);
	if (retval < 0) {
		cyttsp_debug("error cyttsp_power_on!!!!!\n");
		goto error_power_on;
	}
	/* Prepare our worker structure prior to setting up the timer/ISR */
	INIT_WORK(&ts->work,cyttsp_xy_worker);
	spin_lock_init(&ts->lock);
    
	ts->input = tpd->dev;
	/* Timer or Interrupt setup */
#ifndef CYTTSP_USE_IRQ
		cyttsp_info("Setting up timer\n");
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
		mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
#else
		cyttsp_info("Setting up interrupt\n");
		mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, cyttsp_irq, 1);
    	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif

	atomic_set(&ts->irq_enabled, 1);
	device_create_file(&ts->client->dev, &dev_attr_irq_enable);

	printk("%s: Successful initialized\n", CYTTSP_I2C_NAME);

error_power_on:
	return retval;
}

static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, CYTTSP_I2C_NAME);
    return 0;
}

static void tpd_re_init(void)
{
	int retval = CYTTSP_OPERATIONAL;
	u8 host_reg;
       struct cyttsp *ts = this_ts;
       

        printk("[tpd_re_init]\n");

        hwPowerDown(TPD_POWER_SOURCE,"TP");
        msleep(100);
        hwPowerOn(TPD_POWER_SOURCE,VOL_2800,"TP");
        msleep(100);

        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);

        mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_CTP_RST_PIN, GPIO_PULL_DOWN);

        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
        msleep(10);   
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
        msleep(30);//LK@10->30
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
        msleep(100);

        mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
        mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
       #if 1
       retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(bl_cmd), bl_cmd);
       #else
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+4, 4, bl_cmd+4);
	if (retval < 0)
		printk("[tpd_re_init]retval:%d, exit bootloader failed1!!! \n", retval);
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE+8, 3, bl_cmd+8);
	if (retval < 0)
		printk("[tpd_re_init]retval:%d, exit bootloader failed2!!! \n", retval);
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 4, bl_cmd);
       #endif
	if (retval < 0)
		printk("[tpd_re_init]retval:%d, exit bootloader failed!!! \n", retval);

}


static int tpd_check_i2c(void)
{
    int retval = CYTTSP_OPERATIONAL;
    u8 host_reg;
    u8 i;
    struct cyttsp *ts = this_ts;

    
    for(i=0; i<5; i++)
    {
        retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);
        if (retval > 0)
        {
            //printk("[tpd_check_i2c] check i2c read success, retval is %d,i=%d \n", retval,i);
            break;
        }
        else
        {
            printk("[tpd_check_i2c] check i2c read failed, retval is %d,i=%d\n", retval,i);
        }
    }

    if(retval > 0)//check i2c ack
    {
        retval = 0;
    }
    else
    {
        retval = 1;
    }

    //printk("[tpd_check_i2c] the final retval:%d\n",retval);
    
    return retval;
}

#if defined(__TPD_I2C_CHECK_SW_WORKAROUD__)
#define CHECK_I2C_TIMER_COUNT   2000    // 2s   
#define BAT_MS_TO_NS(x) (x * 1000 * 1000)
static struct hrtimer tpd_i2c_check_timer;
static struct task_struct *tpd_i2c_check_thread = NULL;
static int tpd_i2c_check_flag = 0;
static int i2c_check_halt = 0;
static DECLARE_WAIT_QUEUE_HEAD(tpd_i2c_check_waiter);

static int tpd_i2c_check_sw_thread_handler(void *unused)
{
    ktime_t ktime;
    int retval = 0;
    int error = 0;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

    printk("[tpd_i2c_check_sw_thread_handler] \n");
    do
    {
    	ktime = ktime_set(0, BAT_MS_TO_NS(CHECK_I2C_TIMER_COUNT));       
    
       set_current_state(TASK_INTERRUPTIBLE);
       wait_event_interruptible(tpd_i2c_check_waiter, tpd_i2c_check_flag != 0);
    	tpd_i2c_check_flag = 0;
       set_current_state(TASK_RUNNING);
       if(0 == i2c_check_halt)
       {
           //printk("[tpd_i2c_check_sw_thread_handler], i2c_check_halt=%d \n",i2c_check_halt);
           error = tpd_check_i2c();
           if(error > 0)
           {
               printk("[tpd_i2c_check_sw_thread_handler], tpd_check_i2c error!! \n");
               tpd_re_init();
           }
        }
       else
       {
           printk("[tpd_i2c_check_sw_thread_handler], stop to check i2c: i2c_check_halt=%d \n",i2c_check_halt);
       }
        hrtimer_start(&tpd_i2c_check_timer, ktime, HRTIMER_MODE_REL);    
		
	} while (!kthread_should_stop());
    
    return 0;
}

static enum hrtimer_restart tpd_i2c_check_sw_workaround(struct hrtimer *timer)
{
	tpd_i2c_check_flag = 1; 
	wake_up_interruptible(&tpd_i2c_check_waiter);

       //printk("[tpd_i2c_check_sw_workaround] \n");
	
    return HRTIMER_NORESTART;
}

static void tpd_i2c_check_sw_workaround_init(void)
{
    ktime_t ktime;

    ktime = ktime_set(0, BAT_MS_TO_NS(CHECK_I2C_TIMER_COUNT));
    hrtimer_init(&tpd_i2c_check_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    tpd_i2c_check_timer.function = tpd_i2c_check_sw_workaround;    
    hrtimer_start(&tpd_i2c_check_timer, ktime, HRTIMER_MODE_REL);

    tpd_i2c_check_thread = kthread_run(tpd_i2c_check_sw_thread_handler, 0, "tpd_i2c_check_sw_workaround");
    if (IS_ERR(tpd_i2c_check_thread))
    {
        printk("[%s]: failed to create tpd_i2c_check_sw_workaround thread\n", __FUNCTION__);
    }

	printk("[tpd_i2c_check_sw_workaround_init] : done\n" );
}

#endif

/* I2C driver probe function */
static int __devinit cyttsp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cyttsp *ts;
	int error;
	int retval = CYTTSP_OPERATIONAL;

	/* allocate and clear memory */
	this_ts = kzalloc (sizeof(struct cyttsp),GFP_KERNEL);
	if (this_ts == NULL) {
		printk("err kzalloc for cyttsp\n");
		retval = -ENOMEM;
	}

	ts = this_ts;
	if (!(retval < CYTTSP_OPERATIONAL)) 
	{
		printk("cyttsp_probe enter!\n");
		/* register driver_data */
        client->addr = (client->addr & I2C_MASK_FLAG) |I2C_ENEXT_FLAG;// | I2C_DMA_FLAG;// |I2C_HS_FLAG;//LK@add
        client->timing = 200;//LK@add
		ts->client = client;
		ts->platform_data = &cyttsp_data;
		//i2c_set_clientdata(client,ts);

		error = cyttsp_initialize(client, ts);
		
		if (error) 
		{
			printk("err cyttsp_initialize\n");
			if (ts != NULL) {
				/* deallocate memory */
				kfree(ts);
				ts = NULL;
			}
/*
			i2c_del_driver(&cyttsp_driver);
*/
			retval = -ENODEV;
		}
		else
		{
			printk("ok cyttsp_initialize\n");
			cyttsp_openlog();

            #if defined(__TPD_I2C_CHECK_SW_WORKAROUD__)
            tpd_i2c_check_sw_workaround_init();
            #endif
            tpd_load_status = 1;
		}
	}
	//tpd_load_status = 1;
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (!(retval < CYTTSP_OPERATIONAL)) {
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ts->early_suspend.suspend = cyttsp_early_suspend;
		ts->early_suspend.resume = cyttsp_late_resume;
		register_early_suspend(&ts->early_suspend);
	}
#endif
*/

	printk("cyttsp_probe %s\n", (retval < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS");

	return retval;
}

/* Function to manage power-on resume */
static int cyttsp_resume(struct i2c_client *client)
{
	struct cyttsp *ts;
	u8 wake_mode = CYTTSP_OPERATIONAL;
	int retval = CYTTSP_OPERATIONAL;


    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);
    msleep(1);
    mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ONE);
    msleep(1);
    mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);
    msleep(1);
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);





	ts = this_ts;
	if (ts->platform_data->use_sleep && (ts->platform_data->power_state != CYTTSP_ACTIVE_STATE)) {
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			sizeof(wake_mode), &wake_mode);
		if (!(retval < CYTTSP_OPERATIONAL)) {
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE,
				8, (u8 *)&g_wake_data);
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+8,
				8, (u8 *)&g_wake_data+8);
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+16,
				8, (u8 *)&g_wake_data+16);
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE+24,
				6, (u8 *)&g_wake_data+24);
		}
	}

	if (!(retval < CYTTSP_OPERATIONAL) && 
		(GET_HSTMODE(g_wake_data.hst_mode) == CYTTSP_OPERATIONAL)) {
		ts->platform_data->power_state = CYTTSP_ACTIVE_STATE;

		/* re-enable the interrupt after resuming */
#ifdef CYTTSP_USE_IRQ
			spin_lock_irq(&ts->lock);
			mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
			spin_unlock_irq(&ts->lock);
#endif
	}
	else {
		retval = -ENODEV;
	}

	cyttsp_debug("Wake Up %s\n", (retval < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS" );
#if defined(__TPD_I2C_CHECK_SW_WORKAROUD__)
    i2c_check_halt = 0;
    hrtimer_start(&tpd_i2c_check_timer, ktime_set(0, BAT_MS_TO_NS(CHECK_I2C_TIMER_COUNT)), HRTIMER_MODE_REL);
#endif
	return retval;
}


/* Function to manage low power suspend */
static int cyttsp_suspend(struct i2c_client *client, pm_message_t message)
{
	struct cyttsp *ts; 
	u8 sleep_mode = CYTTSP_OPERATIONAL;
	int retval = CYTTSP_OPERATIONAL;

	cyttsp_debug("Enter Sleep\n");
#if defined(__TPD_I2C_CHECK_SW_WORKAROUD__)
    i2c_check_halt = 1;
    hrtimer_cancel(&tpd_i2c_check_timer);
#endif

	ts = this_ts; 

	/* disable worker */
	//disable_irq_nosync(ts->client->irq);
	
	retval = cancel_work_sync(&ts->work);
#ifdef CYTTSP_USE_IRQ
		spin_lock_irq(&ts->lock);
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		spin_unlock_irq(&ts->lock);
#endif

	if (!(retval < CYTTSP_OPERATIONAL)) {
//		if (ts->platform_data->use_sleep && 
//			(ts->platform_data->power_state == CYTTSP_ACTIVE_STATE)) {
		//	if (ts->platform_data->use_sleep & CYTTSP_USE_DEEP_SLEEP_SEL) {
				sleep_mode = CYTTSP_DEEP_SLEEP_MODE;
	//		}
		//	else {
		//		sleep_mode = CYTTSP_LOW_POWER_MODE;
		//	}
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
				sizeof(sleep_mode), &sleep_mode);
//		}
	}

	if (!(retval < CYTTSP_OPERATIONAL)) {
		if (sleep_mode == CYTTSP_DEEP_SLEEP_MODE) {
			ts->platform_data->power_state = CYTTSP_SLEEP_STATE;
		}
		else if (sleep_mode == CYTTSP_LOW_POWER_MODE) {
			ts->platform_data->power_state = CYTTSP_LOW_PWR_STATE;
		}
	}

	cyttsp_debug("Sleep Power state is %s\n", \
		(ts->platform_data->power_state == CYTTSP_ACTIVE_STATE) ? "ACTIVE" : \
		((ts->platform_data->power_state == CYTTSP_SLEEP_STATE) ? "SLEEP" : "LOW POWER"));

	return retval;
}

/* registered in driver struct */
static int __devexit cyttsp_remove(struct i2c_client *client)
{
	struct cyttsp *ts;

	cyttsp_alert("Unregister\n");

	/* clientdata registered on probe */
	ts = this_ts;
	if(ts != NULL) {
		device_remove_file(&ts->client->dev, &dev_attr_irq_enable);

		/* Start cleaning up by removing any delayed work and the timer */
		if (cancel_delayed_work((struct delayed_work *)&ts->work)<0) {
			cyttsp_alert("error: could not remove work from workqueue\n");
		}

	/* free up timer or irq */
		#ifndef CYTTSP_USE_IRQ
		del_timer(&ts->timer);
		#endif
	}

	/* housekeeping */
	if (ts != NULL) {
		kfree(ts);
	}

	cyttsp_alert("Leaving\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_early_suspend(struct early_suspend *handler)
{
	struct cyttsp *ts;

	ts = container_of(handler, struct cyttsp, early_suspend);
	cyttsp_suspend(ts->client, PMSG_SUSPEND);
}

static void cyttsp_late_resume(struct early_suspend *handler)
{
	struct cyttsp *ts;

	ts = container_of(handler, struct cyttsp, early_suspend);
	cyttsp_resume(ts->client);
}
#endif  /* CONFIG_HAS_EARLYSUSPEND */

static int tpd_local_init(void)
{
    if(i2c_add_driver(&cyttsp_driver)!=0)
    {
        TPD_DMESG("unable to add i2c driver.\n");
        return -1;
    }
    if(tpd_load_status == 0)
    {
    	TPD_DMESG("add error touch panel driver.\n");
    	i2c_del_driver(&cyttsp_driver);
    	return -1;
    }

#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif

    tpd_type_cap = 1;
    return 0;
}

static struct tpd_driver_t tpd_device_driver = {
		.tpd_device_name = "cy8ctst242",
		.tpd_local_init = tpd_local_init,
		.suspend = cyttsp_early_suspend,
		.resume = cyttsp_late_resume,
#ifdef TPD_HAVE_BUTTON
		.tpd_have_button = 1,
#else
		.tpd_have_button = 0,
#endif		
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
    printk("MediaTek cy8ctst242 touch panel driver init\n");
	    i2c_register_board_info(0, &i2c_tpd, 1);
		if(tpd_driver_add(&tpd_device_driver) < 0)
			TPD_DMESG("add generic driver failed\n");
    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
    TPD_DMESG("MediaTek cy8ctma300 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

