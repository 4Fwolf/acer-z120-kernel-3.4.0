/* Copyright Statement:
 *	a12 mudong wuli 
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */


#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/dma-mapping.h>


//#ifdef MT6575
//#include <mach/mt6575_pm_ldo.h>
#include <mach/mt_pm_ldo.h>
//#include <mach/mt6575_typedefs.h>
#include <mach/mt_typedefs.h>
//#include <mach/mt6575_boot.h>
#include <mach/mt_boot.h>
//#endif

#include "cust_gpio_usage.h"

//LK@add
//#define __TPD_I2C_CHECK_SW_WORKAROUD__
#define __FIRMWARE_UPDATE__
#if defined(ACER_Z1)
#define __FIRMWARE_AUTO_UPDATE__
#include "msg2133_fw_ACER_Z1.h"
#define TPD_POWER_SOURCE         MT65XX_POWER_LDO_VGP
#define TPD_RES_X (320)
#define TPD_RES_Y (480)
#elif defined(ACER_Z2)
#define __FIRMWARE_AUTO_UPDATE__
#include "msg2133_fw_ACER_Z2.h"
#define TPD_POWER_SOURCE         MT65XX_POWER_LDO_VGP
#define TPD_RES_X (320)
#define TPD_RES_Y (480)
#elif defined(ACER_Z3)
#define POWER_MODE1
#define TPD_POWER_SOURCE         MT65XX_POWER_LDO_VGP
#define TPD_RES_X (480)
#define TPD_RES_Y (800)
#endif


extern struct tpd_device *tpd;

struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static void tpd_eint_interrupt_handler(void);
static void tpd_re_init();	 /* for ESD protect */
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

static int __devinit tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_i2c_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static int tpd_initialize(struct i2c_client * client);

extern int tpd_firmware_version[2];   
static int tpd_flag = 0;
static int tpd_halt=0;
static int point_num = 0;
static int p_point_num = 0;
static BOOTMODE bootMode = NORMAL_BOOT;  //  add for TP button,by tyd-lg

#if defined(MSG2133_PS_SUPPORT)
static int tpd_ps_flag=0; //enable--1;disanle--0
static unsigned char tpd_ps_value=1;// 1->far,0->close
#endif

static int last_key_value;//LK@add
static int last_point_num;

#define HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY//LK@modify
#if defined(ACER_Z2)
static const u16 tpd_keys_local[]={
					KEY_MENU,				
					KEY_HOMEPAGE,				
					KEY_BACK,								  
				  };
#elif defined(ACER_Z3)
static const u16 tpd_keys_local[]={
					KEY_MENU,				
					KEY_BACK,				
					KEY_HOMEPAGE,									  
				  };
#endif
static int key_value;
#define MAX_KEY_NUM (sizeof(tpd_keys_local) / sizeof(tpd_keys_local[0]))
#endif

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

//#define TPD_CLOSE_POWER_IN_SLEEP
//#define TP_DEBUG_MSG


#define TOUCH_ADDR_MSG20XX   (0x4C>>1)

#define TPD_OK 0
#define TPD_NG 0x1

#define TPD_PACKET_HEAD_CMD    0x54
#define TPD_PACKET_HEAD_RSP    0x52

#define TPD_PACKET_CMD_VER    0x10

#define TPD_PACKET_CMD_MODE_READ    0x20
#define TPD_PACKET_CMD_MODE_WRITE   0x21
#define TPD_MODE_ACTIVE             0x10
#define TPD_MODE_FAST               0x20
#define TPD_MODE_FREEZE             0x90

#define TPD_PACKET_CMD_FORMAT_READ    0x60
#define TPD_PACKET_CMD_FORMAT_WRITE   0x61
#define TPD_PACKET_FORMAT_1           0x00
#define TPD_PACKET_FORMAT_2           0x01
#define TPD_PACKET_FORMAT_3           0x03


struct TpdPacketT
{
    U8 head;
    U8 command;
    U8 data_1;
    U8 data_2;
};

// debug macros
#if defined(TP_DEBUG_MSG)
#define pr_tp(format, args...) printk("<0>" format, ##args)
#define pr_k(format, args...) printk("<0>" format, ##args)
#define pr_ch(format, args...)                      \
    printk("<0>" "%s <%d>,%s(),cheehwa_print:\n\t"  \
           format,__FILE__,__LINE__,__func__, ##args)

#else
#define pr_tp(format, args...)  do {} while (0)
#define pr_ch(format, args...)  do {} while (0)
#undef pr_k(format, args...)
#define pr_k(format, args...)  do {} while (0)
#endif

#if 0//def TP_PROXIMITY_SENSOR
char ps_data_state[1] = {0};
enum
{
    DISABLE_CTP_PS,
    ENABLE_CTP_PS,
    RESET_CTP_PS
};
#endif
///int SMC_SWITCH=0;

struct touch_info
{
    unsigned short y[3];
    unsigned short x[3];
    unsigned short p[3];
    unsigned short count;
};

typedef struct
{
    unsigned short pos_x;
    unsigned short pos_y;
    unsigned short pos_x2;
    unsigned short pos_y2;
    unsigned short temp2;
    unsigned short temp;
    short dst_x;
    short dst_y;
    unsigned char checksum;
} SHORT_TOUCH_STATE;


static const struct i2c_device_id msg2133_tpd_id[] = {{"msg2133", 0}, {}};
//unsigned short force[] = {0, TOUCH_ADDR_MSG20XX, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata msg2133_i2c_tpd={ I2C_BOARD_INFO("msg2133", TOUCH_ADDR_MSG20XX)};

static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
        .name = "msg2133",
       // .owner = THIS_MODULE,
    },
    .probe = tpd_i2c_probe,
    .remove = __devexit_p(tpd_i2c_remove),
    .id_table = msg2133_tpd_id,
    .detect = tpd_i2c_detect,
    //.address_data = &addr_data,
};
#if 0
static hw_module_info hw_info = {
	.type = HW_MODULE_TYPE_CTP,
	.id = TOUCH_ADDR_MSG20XX,
	.priority = HW_MODULE_PRIORITY_CTP,
	.name = "MSG2133",
	.vendor = "Mstar",
	.more = "320 x 480"
};
#endif




static void tp_power_on(void)
{

#if 0
    hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_2800, "TP"); 
    // for enable/reset pin
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    //msleep(10);
    
    msleep(500);//at least 350ms
#endif

	hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");     // CTP_VDD_2.8V  
}


static void tp_power_off(void)
{
	hwPowerDown(TPD_POWER_SOURCE,"TP");
}

static void tp_rst_high(void)
{
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
}


static void tp_rst_low(void)
{
    // for enable/reset pin
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
}

#if 1//for android 4.0
#define FW_ADDR_MSG21XX   (0xC4>>1)
#else//for android 2.3
#define FW_ADDR_MSG21XX   0xC4
#endif

#if defined (__FIRMWARE_UPDATE__) || defined(__FIRMWARE_AUTO_UPDATE__)
#if 1//for android 4.0
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
#else//for android 2.3
#define FW_ADDR_MSG21XX_TP   0x4C
#define FW_UPDATE_ADDR_MSG21XX   0x92
#endif

#define TP_DEBUG	printk
static  char *fw_version;
static u8 temp[94][1024];
static u8 auto_temp[94][1024];
static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;

static struct i2c_client     *msg21xx_i2c_client;


static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u8 size)
{
   //according to your platform.
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			//.addr = addr,
                     .addr = ((addr & I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),//LK@test
			.flags = I2C_M_RD,
			.len = size,
			.buf = read_data,
			.timing = 400,
		},
	};

	rc = i2c_transfer(i2c_client->adapter, msgs, 1);
	if( rc < 0 )
    {
		printk("HalTscrCReadI2CSeq error %d\n", rc);
	}
}

static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    //according to your platform.
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			//.addr = addr,
                     .addr = ((addr & I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),//LK@test
			.flags = 0,
			.len = size,
			.buf = data,
			.timing = 400,
		},
	};
	rc = i2c_transfer(i2c_client->adapter, msgs, 1);
	if( rc < 0 )
    {
		printk("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc,addr);
	}
}

static void dbbusDWIICEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}

static void dbbusDWIICStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);

    // Delay some interval to guard the next transaction
    //udelay ( 200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode(void)
{
    u8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };

    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);
}

static u8 drvISP_Read(u8 n, u8* pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    u8 Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0};
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1);
    udelay ( 100 );        // delay about 100us*****
    if (n == 1)
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
        //*pDataToRead = dbbus_rx_data[1];   //poro:2012-04-27
        *pDataToRead = (dbbus_rx_data[0]>dbbus_rx_data[1])?dbbus_rx_data[0]:dbbus_rx_data[1];   //poro:2012-04-27
        //*pDataToRead = dbbus_rx_data[0];   //mark

		
    }
    else
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, pDataToRead, n);
    }

    return 0;
}

static void drvISP_WriteEnable(void)
{
    u8 bWriteData[2] =
    {
        0x10, 0x06
    };
    u8 bWriteData1 = 0x12;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
}


static void drvISP_ExitIspMode(void)
{
    u8 bWriteData = 0x24;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1);
}

static u8 drvISP_ReadStatus(void)
{
    u8 bReadData = 0;
    u8 bWriteData[2] =
    {
        0x10, 0x05
    };
    u8 bWriteData1 = 0x12;

    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    udelay ( 100 );        // delay about 100us*****
    drvISP_Read(1, &bReadData);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    return bReadData;
}


static void drvISP_ChipErase()
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
		u32 timeOutCount=0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 3);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    udelay ( 100 );        // delay about 100us*****
    timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
		   timeOutCount++;
	     if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	  }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

  	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
		udelay ( 100 );         // delay about 100us*****
		timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
		   timeOutCount++;
	     if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
	  }
}

static void drvISP_Program(u16 k, u8* pDataToWrite)
{
    u16 i = 0;
    u16 j = 0;
    //u16 n = 0;
    u8 TX_data[133];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
		u32 timeOutCount=0;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);
        for (i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        udelay ( 100 );        // delay about 100us*****
       
        timeOutCount=0;
				while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
				{
		   			timeOutCount++;
	     			if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	  		}
        
        
        
        drvISP_WriteEnable();
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, TX_data, 133);   //write 133 byte per cycle
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    }
}


static void drvISP_Verify(u16 k, u8* pDataToVerify)
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index=0;
    u32 timeOutCount;
    for (j = 0; j < 128; j++)   //128*8 cycle
    {
        bWriteData[2] = (u8)((addr + j * 8) >> 16);
        bWriteData[3] = (u8)((addr + j * 8) >> 8);
        bWriteData[4] = (u8)(addr + j * 8);
        udelay ( 100 );        // delay about 100us*****
        
        
        timeOutCount=0;
				while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
				{
		   		timeOutCount++;
	     		if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	  		}
        
        
        
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
        udelay ( 100 );        // delay about 100us*****
        drvISP_Read(8, RX_data);
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
        for (i = 0; i < 8; i++)   //log out if verify error
        {
        if((RX_data[i]!=0)&&index<10)
		{
        //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
        index++;
		}
            if (RX_data[i] != pDataToVerify[8 * j + i])
            {
                TP_DEBUG("k=%d,j=%d,i=%d===Update Firmware Error===\r\n",k,j,i);
                TP_DEBUG("(RX_data[i]=%x,pDataToVerify[8 * j + i]=%x\r\n",RX_data[i],pDataToVerify[8 * j + i]);
                //break;//add for test
            }
        }
    }
}

static ssize_t firmware_update_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", fw_version);
}

static void _HalTscrHWReset(void)
{

#ifdef POWER_MODE1
	tp_rst_low();
	mdelay(10);  /* Note that the RST must be in LOW 10ms at least */
	tp_rst_high();
	/* Enable the interrupt service thread/routine for INT after 50ms */
	mdelay(50);
#else
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        //mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
        mdelay(10);  /* Note that the RST must be in LOW 10ms at least */
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    /* Enable the interrupt service thread/routine for INT after 50ms */
    mdelay(50);
#endif
}


static ssize_t firmware_update_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    TP_DEBUG("firmware_update_store start\n");

    _HalTscrHWReset();
    //1.Erase TP Flash first
    mdelay(300);//mark

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    //mdelay(300);


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);



    //set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;     
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();



    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay(300);

    //2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();



    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);



    //set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for (i = 0; i < 94; i++)   // total  94 KB : 1 byte per R/W
    {
        drvISP_Program(i, temp[i]);    // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
        
    drvISP_ExitIspMode();
    
    TP_DEBUG("firmware_update_store OK\n");
    _HalTscrHWReset();
    mdelay(300);//mark
    
    FwDataCnt = 0;
    return size;
}

static DEVICE_ATTR(update, S_IRUGO|S_IWUSR|S_IWGRP, firmware_update_show, firmware_update_store);//chmod 664,S_IRUGO|S_IWUSR|S_IWGRP
//static DEVICE_ATTR(update, 0777, firmware_update_show, firmware_update_store);//chmod 664,S_IRUGO|S_IWUSR|S_IWGRP

/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    u16 k=0,i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
	u32 addr = 0;
		u32 timeOutCount=0;
	for (k = 0; k < 94; i++)   // total  94 KB : 1 byte per R/W
   {
      addr = k * 1024;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        bWriteData[2] = (u8)((addr + j * 128) >> 16);
        bWriteData[3] = (u8)((addr + j * 128) >> 8);
        bWriteData[4] = (u8)(addr + j * 128);
        udelay ( 100 );        // delay about 100us*****

        timeOutCount=0;
				while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
				{
		   		timeOutCount++;
	     		if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	  		}
        
        
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
        udelay ( 100 );       // delay about 100us*****
        drvISP_Read(128, RX_data);
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
        for (i = 0; i < 128; i++)   //log out if verify error
        {
            if (RX_data[i] != 0xFF)
            {
                TP_DEBUG("k=%d,j=%d,i=%d===============erase not clean================",k,j,i);
            }
        }
    }
	}
   TP_DEBUG("read finish\n");
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{

    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    _HalTscrHWReset();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();



    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);



    //set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();


    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
    TP_DEBUG("chip erase+\n");
    drvISP_ChipErase();
    TP_DEBUG("chip erase-\n");
    drvISP_ExitIspMode();
    return size;
}

static DEVICE_ATTR(clear, S_IRUGO|S_IWUSR|S_IWGRP, firmware_clear_show, firmware_clear_store);
//static DEVICE_ATTR(clear, 0777, firmware_clear_show, firmware_clear_store);

/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    TP_DEBUG("*** firmware_version_show fw_version = %s***\n", fw_version);
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;

    fw_version = kzalloc(sizeof(char), GFP_KERNEL);
		//SM-BUS GET FW VERSION
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x74;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

    TP_DEBUG("***major = %d ***\n", major);
    TP_DEBUG("***minor = %d ***\n", minor);
    sprintf(fw_version,"%03d%03d", major, minor);
    TP_DEBUG("***fw_version = %s ***\n", fw_version);


    return size;
}
static DEVICE_ATTR(version, S_IRUGO|S_IWUSR|S_IWGRP, firmware_version_show, firmware_version_store);
//static DEVICE_ATTR(version, 0777, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{

    int i;
	TP_DEBUG("***FwDataCnt = %d ***\n", FwDataCnt);
    //for (i = 0; i < 1024; i++)
    {
        memcpy(temp[FwDataCnt], buf, 1024);
    }
    FwDataCnt++;
    return size;
}
static DEVICE_ATTR(data, S_IRUGO|S_IWUSR|S_IWGRP, firmware_data_show, firmware_data_store);//chmod 664
//static DEVICE_ATTR(data, 0777, firmware_data_show, firmware_data_store);//chmod 664
#endif


#if 0//def TP_PROXIMITY_SENSOR
static ssize_t show_proximity_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
    static char temp=2;
    buf = ps_data_state;
    if(temp!=*buf)
    {
    printk("proximity_sensor_show: buf=%d\n\n", *buf);
    temp=*buf;
    }    return 1;
}

static ssize_t store_proximity_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    U8 ps_store_data[4];

    if(buf != NULL && size != 0)
    {
        if(DISABLE_CTP_PS == *buf)
        {
            printk("DISABLE_CTP_PS buf=%d,size=%d\n", *buf, size);
            ps_store_data[0] = 0x52;
            ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x80;
            ps_store_data[3] = 0xa1;
            i2c_write(TOUCH_ADDR_MSG20XX, &ps_store_data[0], 4);
            msleep(2000);
            printk("RESET_CTP_PS buf=%d\n", *buf);
            mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef POWER_MODE1
			// for enable/reset pin
			tp_rst_low();
			msleep(100);
			tp_rst_high();
			msleep(500);
#else
            mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
            msleep(100);
            mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
            mdelay(500);
#endif
            mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        }
        else if(ENABLE_CTP_PS == *buf)
        {
            printk("ENABLE_CTP_PS buf=%d,size=%d\n", *buf, size);
            ps_store_data[0] = 0x52;
            ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x80;
            ps_store_data[3] = 0xa0;
            i2c_write(TOUCH_ADDR_MSG20XX, &ps_store_data[0], 4);
        }
    }

    return size;
}
static DEVICE_ATTR(proximity_sensor, S_IRUGO|S_IWUSR|S_IWGRP, show_proximity_sensor, store_proximity_sensor);
#endif



#if defined(MSG2133_PS_SUPPORT)
extern void msg2133_ps_set_hwm_data(int ps_data);

 int msg2133_set_ps_data_to_alsps(unsigned char val)
{

    printk("[msg2133_set_ps_data_to_alsps]val=%d\n", val);

    msg2133_ps_set_hwm_data(val);
	
}

unsigned char msg2133_get_ps_value(void)
{
    printk("[msg2133_get_ps_value]tpd_ps_value=%d\n", tpd_ps_value);

    return tpd_ps_value;
}
EXPORT_SYMBOL (msg2133_get_ps_value);

void msg2133_ps_enable(int enable)
{

    unsigned char bWriteData[4] =
    {
        0x52, 0x01, 0x24, 0xA5
    };

	if(enable==1)
	{
		bWriteData[3] = 0xA5;
        tpd_ps_flag=1;

	}
	else
	{	
		bWriteData[3] = 0x01;
        tpd_ps_flag=0;

	}

       printk("[msg2133_ps_enable]tpd_ps_flag=%d\n", tpd_ps_flag);
       
       i2c_client->addr = TOUCH_ADDR_MSG20XX;
       i2c_client->addr = ((i2c_client->addr & I2C_MASK_FLAG ) | I2C_ENEXT_FLAG );//LK@add
       i2c_master_send(i2c_client, &bWriteData[0], 4);	

}
EXPORT_SYMBOL (msg2133_ps_enable);
#endif

static  void tpd_down(int x, int y, int id)
{
    //if(FACTORY_BOOT == bootMode)
    //		input_report_key(tpd->dev, BTN_TOUCH, 1);

    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    //pr_tp("D[%4d %4d %4d] ", x, y, p);
    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id); //LK@add the finger num serial
    input_mt_sync(tpd->dev);
    TPD_DOWN_DEBUG_TRACK(x, y);
    
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(x, y, 1);  
    }
}

static  int tpd_up(int x, int y, int id)
{

        //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
        //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
       // input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
        //pr_tp("U[%4d %4d %4d] ", x, y, 0);
        input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
        input_mt_sync(tpd->dev);
        TPD_UP_DEBUG_TRACK(x, y);
        
        if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
        {   
            tpd_button(x, y, 0); 
        }   
        return 1;
    //}
    //return 0;
}
unsigned char tpd_check_sum(unsigned char *pval)
{
    int i, sum = 0;

    for(i = 0; i < 7; i++)
    {
        sum += pval[i];
    }

    return (unsigned char)((-sum) & 0xFF);
}

static bool msg2133_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
    //    pr_ch("The msg_i2c_client->addr=0x%x\n",i2c_client->addr);
    i2c_client->addr = TOUCH_ADDR_MSG20XX;
    i2c_client->addr = ((i2c_client->addr & I2C_MASK_FLAG ) | I2C_ENEXT_FLAG );//LK@add
    i2c_client->timing = 200;

    ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);

    if(ret <= 0)
    {
	 tpd_re_init();
        printk("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}

static int tpd_touchinfo(struct touch_info *cinfo)
{
    SHORT_TOUCH_STATE ShortTouchState;
    BYTE reg_val[8] = {0};
    unsigned int  temp = 0;
    int index = 0;
    
    msg2133_i2c_read(reg_val, 8);

    ShortTouchState.pos_x = ((reg_val[1] & 0xF0) << 4) | reg_val[2];
    ShortTouchState.pos_y = ((reg_val[1] & 0x0F) << 8) | reg_val[3];
    ShortTouchState.dst_x = ((reg_val[4] & 0xF0) << 4) | reg_val[5];
    ShortTouchState.dst_y = ((reg_val[4] & 0x0F) << 8) | reg_val[6];

    if((ShortTouchState.dst_x) & 0x0800)
    {
        ShortTouchState.dst_x |= 0xF000;
    }

    if((ShortTouchState.dst_y) & 0x0800)
    {
        ShortTouchState.dst_y |= 0xF000;
    }

    ShortTouchState.pos_x2 = ShortTouchState.pos_x + ShortTouchState.dst_x;
    ShortTouchState.pos_y2 = ShortTouchState.pos_y + ShortTouchState.dst_y;
    TPD_DEBUG("[tpd_touchinfo] reg_val[0]=%x,X= %d ,Y=%d\n", reg_val[0], ShortTouchState.pos_x, ShortTouchState.pos_x);
    temp = tpd_check_sum(reg_val);
  //  printk("check_sum=%d,reg_val_7=%d\n", temp, reg_val[7]);

    if(temp == reg_val[7])
    {
        TPD_DEBUG("[tpd_touchinfo] \nreg_val[1]=0x%x\nreg_val[2]=0x%x\nreg_val[4]=0x%x\nreg_val[5]=0x%x\nreg_val[6]=0x%x\nreg_val[7]=0x%x\n", reg_val[1], reg_val[2], reg_val[4], reg_val[5], reg_val[6], reg_val[7]);

        if(reg_val[0] == 0x52) //CTP  ID
        {	
            if(reg_val[1] == 0xFF&& reg_val[4] == 0xFF)
            {
             #if defined(MSG2133_PS_SUPPORT)
		        if(reg_val[5] == 0x80 && tpd_ps_flag == 1 ) // close and ps enable
		        {
		            //msg2133_set_ps_data_to_alsps(0);
		            tpd_ps_value = 0;
		            printk("[tpd_touchinfo] ps close\n");
		        }
		        else if(reg_val[5] == 0x40 && tpd_ps_flag == 1 ) // leave and and ps enable
		        {
		        
		            //msg2133_set_ps_data_to_alsps(1);
		            tpd_ps_value = 1;
		            printk("[tpd_touchinfo] ps leave\n");
		        }
			    else
		        {
		        #ifdef HAVE_TOUCH_KEY
			        key_value = reg_val[5] & 0xf;
                #endif
	                point_num = 0;
		        }
		     #else
                #ifdef HAVE_TOUCH_KEY
                     key_value = reg_val[5] & 0xf;
                #endif
                     point_num = 0;
					 
					 //printk("[tpd_touchinfo]key_value=0x%x,point_num=0x%x\n",key_value,point_num);

             #endif
		        //input_sync(tpd->dev);
            }
            else if(ShortTouchState.pos_x > 2047 || ShortTouchState.pos_y > 2047)
            {
            	return  false;
            }
            else if((ShortTouchState.dst_x == 0) && (ShortTouchState.dst_y == 0))
            {
                //cinfo->x[0] =(ShortTouchState.pos_y * TPD_RES_X) / 2048;
                //cinfo->y[0] = (ShortTouchState.pos_x * TPD_RES_Y) / 2048;
                cinfo->x[0] =(ShortTouchState.pos_x * TPD_RES_X) / 2048;
                cinfo->y[0] = (ShortTouchState.pos_y * TPD_RES_Y) / 2048;
                if(cinfo->x[0] < 1)
                    cinfo->x[0] = 1;
                if(cinfo->x[0] > (TPD_RES_X-1))
                    cinfo->x[0] = TPD_RES_X-1;
                if(cinfo->y[0] < 1)
                    cinfo->y[0] = 1;
                if(cinfo->y[0] > (TPD_RES_Y-1))
                    cinfo->y[0] = TPD_RES_Y-1;
                point_num = 1;
            }
            else
            {
                if(ShortTouchState.pos_x2 > 2047 || ShortTouchState.pos_y2 > 2047)
                    return false;
                //cinfo->x[0] = (ShortTouchState.pos_y * TPD_RES_X) / 2048;
                //cinfo->y[0] = (ShortTouchState.pos_x * TPD_RES_Y) / 2048;
                //cinfo->x[1] = (ShortTouchState.pos_y2 * TPD_RES_X) / 2048;
                //cinfo->y[1] = (ShortTouchState.pos_x2 * TPD_RES_Y) / 2048;

                cinfo->x[0] = (ShortTouchState.pos_x * TPD_RES_X) / 2048;
                cinfo->y[0] = (ShortTouchState.pos_y * TPD_RES_Y) / 2048;
                cinfo->x[1] = (ShortTouchState.pos_x2 * TPD_RES_X) / 2048;
                cinfo->y[1] = (ShortTouchState.pos_y2 * TPD_RES_Y) / 2048;
                point_num = 2;
            }
			//}}A:
        }
        else//no touch down or up, no need to report key in event handler
        {
            TPD_DEBUG("[tpd_touchinfo]no touch!\n");
            return  false;
        }


        return true;
    }
    else
    {
        pr_tp("tpd_check_sum_ XXXX\n");
        return  false;
    }

}

static int touch_event_handler(void *unused)
{
    struct touch_info cinfo;
    int touch_state = 3;
    unsigned long time_eclapse;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);
    int last_key = 0;
    int key;
    int current_key;

    do
    {
        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        while ( tpd_halt )
        {
            tpd_flag = 0;
            msleep(20);
        }
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        set_current_state(TASK_RUNNING);

        if(tpd_touchinfo(&cinfo))
        {
            TPD_DEBUG("point_num = %d\n",point_num);
            TPD_DEBUG("MSG_X1 = %d,MSG_Y1 = %d\n", cinfo.x[0], cinfo.y[0]);
            
            if(point_num == 1)
            {
			/*	if(0 == strncmp(MTK_LCM_PHYSICAL_ROTATION, "180", 3))
				{//synchronize with LCM rotate by vito
					cinfo.x[0] = TPD_RES_X - 1 - cinfo.x[0];

					cinfo.y[0] = TPD_RES_Y - 1 - cinfo.y[0];	
				}
			*/
					
                tpd_down(cinfo.x[0], cinfo.y[0], 1);
                input_sync(tpd->dev);
            }
            else if(point_num == 2)
            {
			/*	if(0 == strncmp(MTK_LCM_PHYSICAL_ROTATION, "180", 3))
				{//synchronize with LCM rotate by vito
					cinfo.x[0] = TPD_RES_X - 1 - cinfo.x[0];

					cinfo.y[0] = TPD_RES_Y - 1 - cinfo.y[0];	
					
					cinfo.x[1] = TPD_RES_X - 1 - cinfo.x[1];

					cinfo.y[1] = TPD_RES_Y - 1 - cinfo.y[1];	
				 }*/
                TPD_DEBUG("MSG_X2 = %d,MSG_Y2 = %d\n", cinfo.x[1], cinfo.y[1]);
            
                tpd_down(cinfo.x[0], cinfo.y[0], 0);
                tpd_down(cinfo.x[1], cinfo.y[1], 1);
                input_sync(tpd->dev);
            }
            else if(point_num == 0)
            {
#ifdef HAVE_TOUCH_KEY
            TPD_DEBUG("[touch_event_handler]: key_value=%d,last_key_value=%d,last_point_num=%d\n",key_value,last_key_value,last_point_num);
			if (key_value)//key pressed
			{
			       current_key = key_value;
				key = MAX_KEY_NUM - 1;
				while (current_key >>= 1)
				{
					key--;
				}
				last_key = key;
                
                TPD_DEBUG("[touch_event_handler]:tpd press key=%d;tpd_keys_local[key]=%d",key,tpd_keys_local[key]);

				input_report_key(tpd->dev, tpd_keys_local[key], 1);
                input_mt_sync(tpd->dev);

			}
			else if(last_key_value)//key release, check the lastest key value and release it
			{
			       TPD_DEBUG("[touch_event_handler]:tpd release last_key=%d;tpd_keys_local[last_key]=%d\n",last_key,tpd_keys_local[last_key]);

                   input_report_key(tpd->dev, tpd_keys_local[last_key], 0);
                   input_mt_sync(tpd->dev);

			}
                    else//not key release, should be touch sreen release
                    {
                        if(1 == last_point_num)
                        {
                            tpd_up(cinfo.x[0], cinfo.y[0], 0);
                            TPD_DEBUG("[touch_event_handler]tpd release sreen cinfo.x[0]=%d;cinfo.y[0]=%d\n",cinfo.x[0],cinfo.y[0]);

                        }
                        else if(2 == last_point_num)
                        {
                            tpd_up(cinfo.x[0], cinfo.y[0], 0);
                            tpd_up(cinfo.x[1], cinfo.y[1], 1);
                            TPD_DEBUG("[touch_event_handler]:tpd release sreen cinfo.x[0]=%d;cinfo.y[0]=%d\n",cinfo.x[0],cinfo.y[0]);
                            TPD_DEBUG("[touch_event_handler]:tpd release sreen cinfo.x[1]=%d;cinfo.y[1]=%d\n",cinfo.x[1],cinfo.y[1]);
                        }

                    }
                     last_key_value = key_value;
#else
                 if(1 == last_point_num)
                 {
                        tpd_up(cinfo.x[0], cinfo.y[0], 0);
                        TPD_DEBUG("[touch_event_handler]tpd release sreen cinfo.x[0]=%d;cinfo.y[0]=%d\n",cinfo.x[0],cinfo.y[0]);

                }
                else if(2 == last_point_num)
                {
                        tpd_up(cinfo.x[0], cinfo.y[0], 0);
                        tpd_up(cinfo.x[1], cinfo.y[1], 1);
                        TPD_DEBUG("[touch_event_handler]:tpd release sreen cinfo.x[0]=%d;cinfo.y[0]=%d\n",cinfo.x[0],cinfo.y[0]);
                        TPD_DEBUG("[touch_event_handler]:tpd release sreen cinfo.x[1]=%d;cinfo.y[1]=%d\n",cinfo.x[1],cinfo.y[1]);
                }
#endif

                input_sync(tpd->dev);
            }
            last_point_num = point_num;
        }
    }
    while(!kthread_should_stop());

    return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

static void tpd_eint_interrupt_handler(void)
{
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
    TPD_DEBUG("TPD interrupt has been triggered\n");
}

static void tpd_re_init(void)
{

    printk("[tpd_re_init]\n");

#ifdef POWER_MODE1
    //power on
	tp_power_on();
	
    // for enable/reset pin
	tp_rst_low();
    msleep(10);
	tp_rst_high();
    msleep(500);
#else
    hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_2800, "TP");   
	
    // for enable/reset pin
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(500);
#endif

    /* set INT mode */
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    msleep(10);
}

unsigned int tpd_msg2133_version_check()
{
    int retval = TPD_OK;
    unsigned int ic_ver=0;
    unsigned char i = 0;
    struct TpdPacketT packet = 
    {
        TPD_PACKET_HEAD_CMD,
        TPD_PACKET_CMD_FORMAT_WRITE,
        0x00,
        0x00
    };
    struct TpdPacketT pr;
    char buf[4];
    buf[0] = 0x53;
    buf[1] = (0x74>>8)&0xFF;        
    buf[2] = (0x74)&0xFF;   

    i2c_client->addr = TOUCH_ADDR_MSG20XX;
    i2c_client->addr = ((i2c_client->addr & I2C_MASK_FLAG ) | I2C_ENEXT_FLAG );//LK@add

    for(i=0; i<5; i++)
    {
        retval = i2c_master_send(i2c_client, buf, 3);
        retval = i2c_master_recv(i2c_client, buf, 4);
        //printk("[tpd_msg2133_version_check]:retval=%d,i=%d\n",retval,i);
        if(retval > 0)
        {
            //printk("[tpd_msg2133_version_check]:retval=%d,i=%d,break\n",retval,i);
            break;
        }
    }
    if(retval < 0)//i2c error
    {
            printk("[tpd_msg2133_version_check]:retval=%d,fw in the ic is error, to update fw directly!\n",retval,i);
            return 0;

    }
    ic_ver = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);

    //printk("Firmware IC internal Version:0x%02x,0x%02x,0x%02x,0x%02x,0x%x\n",buf[0],buf[1],buf[2],buf[3],ic_ver);
    return ic_ver;
}

#if defined(__FIRMWARE_AUTO_UPDATE__)

#if defined(ACER_Z1)//Z1 use two type of film for mstar/sunrise module
typedef enum
{
    TP_FW_1,
    TP_FW_JSR
}TP_FW_TYPE;

unsigned char tp_firmware_type = TP_FW_1;
#endif

//LK@, transfer fw file data to 94*1024 format, should be called after tpd_version_check(), before firmware_update_auto()
static void firmware_data_transfer()
{
    int i;

#if defined(ACER_Z1)
    if(tp_firmware_type == TP_FW_1)
    {
        for(i = 0; i < 94; i++)
        {
            memcpy(auto_temp[i], &MSG_2133_BIN[i*1024], 1024);
        }
    }
    else if(tp_firmware_type == TP_FW_JSR)
    {
        for(i = 0; i < 94; i++)
        {
            memcpy(auto_temp[i], &MSG_2133_BIN_JSR[i*1024], 1024);
        }
    }
#else
    for(i = 0; i < 94; i++)
    {
        memcpy(auto_temp[i], &MSG_2133_BIN[i*1024], 1024);
    }
#endif

    printk("firmware_data_transfer");

}

//this function should be called after tpd_check_i2c()
static int tpd_version_check()
{
    int retval = TPD_OK;
    unsigned int ic_ver=0, fw_ver=0;
    unsigned char i = 0;
    struct TpdPacketT packet = 
    {
        TPD_PACKET_HEAD_CMD,
        TPD_PACKET_CMD_FORMAT_WRITE,
        0x00,
        0x00
    };
    struct TpdPacketT pr;
    char buf[4];
    buf[0] = 0x53;
    buf[1] = (0x74>>8)&0xFF;        
    buf[2] = (0x74)&0xFF;   

    i2c_client->addr = TOUCH_ADDR_MSG20XX;
    i2c_client->addr = ((i2c_client->addr & I2C_MASK_FLAG ) | I2C_ENEXT_FLAG );//LK@add

    for(i=0; i<5; i++)
    {
        retval = i2c_master_send(i2c_client, buf, 3);
        retval = i2c_master_recv(i2c_client, buf, 4);
        printk("[tpd_version_check]:retval=%d,i=%d\n",retval,i);
        if(retval > 0)
        {
            printk("[tpd_version_check]:retval=%d,i=%d,break\n",retval,i);
            break;
        }
    }
    if(retval < 0)//i2c error
    {
        #if defined(ACER_Z1)//defalult use TP_FW_1
            tp_firmware_type = TP_FW_1;
        #endif
            printk("[tpd_version_check]:retval=%d,fw in the ic is error, to update fw directly!\n",retval,i);
            return 0;//fw in the ic is error, to update fw directly,because tpd_check_i2c has confirmed the TP is connected

    }

#if defined(ACER_Z1)
    ic_ver = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);

    printk("Firmware IC internal Version:0x%02x,0x%02x,0x%02x,0x%02x,0x%x\n",buf[0],buf[1],buf[2],buf[3],ic_ver);

    if(buf[0] == 0x01)
    {
        tp_firmware_type = TP_FW_1;
        fw_ver = ((MSG_2133_BIN[0x3077] << 24) + (MSG_2133_BIN[0x3076] << 16) + (MSG_2133_BIN[0x3075] << 8) + MSG_2133_BIN[0x3074]);
        printk("Firmware in the SW Version:0x%02x,0x%02x,0x%02x,0x%02x,0x%x\n",MSG_2133_BIN[0x3077],MSG_2133_BIN[0x3076],MSG_2133_BIN[0x3075],MSG_2133_BIN[0x3074],fw_ver);
    }
    else if(buf[0] == 0x03)
    {
        tp_firmware_type = TP_FW_JSR;
        fw_ver = ((MSG_2133_BIN_JSR[0x3077] << 24) + (MSG_2133_BIN_JSR[0x3076] << 16) + (MSG_2133_BIN_JSR[0x3075] << 8) + MSG_2133_BIN_JSR[0x3074]);
        printk("Firmware in the SW Version:0x%02x,0x%02x,0x%02x,0x%02x,0x%x\n",MSG_2133_BIN_JSR[0x3077],MSG_2133_BIN_JSR[0x3076],MSG_2133_BIN_JSR[0x3075],MSG_2133_BIN_JSR[0x3074],fw_ver);
    }

#else
    ic_ver = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);
    fw_ver = ((MSG_2133_BIN[0x3077] << 24) + (MSG_2133_BIN[0x3076] << 16) + (MSG_2133_BIN[0x3075] << 8) + MSG_2133_BIN[0x3074]);
    
    printk("Firmware IC internal Version:0x%02x,0x%02x,0x%02x,0x%02x,0x%x\n",buf[0],buf[1],buf[2],buf[3],ic_ver);
    printk("Firmware in the SW Version:0x%02x,0x%02x,0x%02x,0x%02x,0x%x\n",MSG_2133_BIN[0x3077],MSG_2133_BIN[0x3076],MSG_2133_BIN[0x3075],MSG_2133_BIN[0x3074],fw_ver);
#endif

    if(ic_ver < fw_ver)
    {
        retval = 0;
    }
    else
    {
        retval = 1;
    }

    return retval;
}


static int drvISP_Verify_auto(u16 k, u8* pDataToVerify)
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index=0;
    u32 timeOutCount;

        
    printk("drvISP_Verify_auto \n");

    for (j = 0; j < 128; j++)   //128*8 cycle
    {
        bWriteData[2] = (u8)((addr + j * 8) >> 16);
        bWriteData[3] = (u8)((addr + j * 8) >> 8);
        bWriteData[4] = (u8)(addr + j * 8);
        udelay ( 100 );        // delay about 100us*****
        
        
        timeOutCount=0;
	 while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	 {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }
     
        
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
        udelay ( 100 );        // delay about 100us*****
        drvISP_Read(8, RX_data);
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
        for (i = 0; i < 8; i++)   //log out if verify error
        {
            if((RX_data[i]!=0)&&index<10)
	     {
                //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
	     }
            if (RX_data[i] != pDataToVerify[8 * j + i])
            {
                printk("k=%d,j=%d,i=%d===Update Firmware Error===\r\n",k,j,i);
                printk("(RX_data[i]=%x,pDataToVerify[8 * j + i]=%x\r\n",RX_data[i],pDataToVerify[8 * j + i]);
                return 1;
            }
               
        }
    }
    
    return 0;
}

static int firmware_update_auto()
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    unsigned char retval = 0;
    
    printk("firmware_update_auto start\n");

    _HalTscrHWReset();
    //1.Erase TP Flash first
    mdelay(300);//mark

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    //mdelay(300);


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);



    //set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;     
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();



    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay(300);

    //2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();



    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);



    //set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    //firmware_data_transfer();//LK@move to firmware_auto_downloader(),2012-10-19
    
    for (i = 0; i < 94; i++)   // total  94 KB : 1 byte per R/W
    {
        drvISP_Program(i, auto_temp[i]);    // program to slave's flash
        retval = drvISP_Verify_auto ( i, auto_temp[i] ); //verify data
        if(retval == 1)
        {
            drvISP_ExitIspMode();
            FwDataCnt = 0;
            printk("firmware_update_auto fail!\n");
            return retval;
        }
    }

    drvISP_ExitIspMode();

    printk("firmware_update_auto OK\n");
    _HalTscrHWReset();
    mdelay(300);

    FwDataCnt = 0;

    return retval;
}

#define AUTO_UPDATE_TRY_COUNT   5 //LK@add 2012-10-19
static int firmware_auto_downloader()
{

    int ret = 0;
    unsigned char i;
    
    printk("firmware_auto_downloader \n");
    
    if(tpd_version_check() == 0)
    {
        firmware_data_transfer();
        for(i = 0; i < AUTO_UPDATE_TRY_COUNT; i++)
        {
            ret = firmware_update_auto();
            if(ret == 1)
            {
                printk("[firmware_auto_downloader]: ret =%d,i=%d, firmware update fail, try one time again!\n",ret,i);
            }
            else
            {
                printk("[firmware_auto_downloader]: ret =%d,i=%d, firmware update OK!\n",ret,i);
                break;
            }
        }
    }
    else
    {
        printk("[firmware_auto_downloader]: the firmware ver in the TP is the latest one, no need to update\n");
    }
    return ret;
}
#endif


static int tpd_check_i2c(void)
{
    int retval = TPD_OK;
   
    struct TpdPacketT packet = 
    {
        TPD_PACKET_HEAD_CMD,
        TPD_PACKET_CMD_FORMAT_WRITE,
        0x00,
        0x00
    };
    struct TpdPacketT pr;
    char txbuf[3];
    unsigned char i = 0;
    
    i2c_client->addr = FW_ADDR_MSG21XX;//FW_ADDR_MSG21XX
    i2c_client->addr = ((i2c_client->addr & I2C_MASK_FLAG ) | I2C_ENEXT_FLAG );
    txbuf[0] = 0x10;
    txbuf[1] = 0x11;
    txbuf[2] = 0xE2;
    for(i=0; i<5; i++)//check ack for 5 times
    {
         retval = i2c_master_send(i2c_client, txbuf, 3);
         //printk("[tpd_check_i2c] tx retval:%d;i=%d\n",retval,i);
         if(retval > 0)
         {
             //printk("[tpd_check_i2c] tx retval:%d;i=%d,break\n",retval,i);
             break;
         }
         else
         {
             printk("[tpd_check_i2c] error: tx retval:%d;i=%d\n",retval,i);
         }
    }
 
    if(retval > 0)//check i2c ack
    {
         retval = TPD_OK;
    }
    else
    {
         retval = TPD_NG;
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
    int retval = TPD_OK;
    unsigned int ret = 0;
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
           ret = tpd_msg2133_version_check();
           //printk("[tpd_i2c_check_sw_thread_handler], tpd_msg2133_version_check version is =0x%x\n",ret);
        
           if(ret == 0)
           {
               printk("[tpd_i2c_check_sw_thread_handler], tpd_msg2133_version_check i2c error!! \n");
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
    
    printk("[tpd_i2c_check_sw_workaround_init]\n" );
    tpd_i2c_check_thread = kthread_run(tpd_i2c_check_sw_thread_handler, 0, "tpd_i2c_check_sw_workaround");
    if (IS_ERR(tpd_i2c_check_thread))
    {
        printk("[%s]: failed to create tpd_i2c_check_sw_workaround thread\n", __FUNCTION__);
    }

	printk("[tpd_i2c_check_sw_workaround_init] : done\n" );
}

#endif

static int __devinit tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = TPD_OK;
    int error = 0;
    //   char data;
    //    client->timing = 400;
    
    i2c_client = client;

    //i2c_client->addr |= I2C_ENEXT_FLAG; 
    //i2c_client->timing = 100;
    
    printk("msg2133 tpd_i2c_probe ,the i2c addr=0x%x", client->addr);

#ifdef POWER_MODE1
    //power on
	tp_power_on();
	
    // for enable/reset pin
	tp_rst_low();
    msleep(10);
	tp_rst_high();
    msleep(500);
#else
    hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_2800, "TP"); 
    // for enable/reset pin
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    //msleep(10);
    
    msleep(500);//at least 350ms
#endif

    error = tpd_check_i2c();

    if(error)
    {
        printk("tpd_check_i2c error\n");
        tpd_load_status = 0;
        return -1;
    }


#if defined(__FIRMWARE_AUTO_UPDATE__)
    error = firmware_auto_downloader();
    if(error)
    {
        printk("firmware_auto_downloader error\n");
        tpd_load_status = 0;
        return -1;
    }
#endif

#ifdef HAVE_TOUCH_KEY
    int retry;
	for(retry = 0; retry < MAX_KEY_NUM; retry++)
	{
		input_set_capability(tpd->dev,EV_KEY,tpd_keys_local[retry]);
	}
#endif

    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    
    //msleep(10);
	
    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    msleep(100);
    tpd_load_status = 1;
    
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if(IS_ERR(thread))
    {
        retval = PTR_ERR(thread);
        printk(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
    }
    /*
        if (TRUE==msg2133_i2c_read(data,1))
        {
            pr_ch("Mstar 's TP\n");
            SMC_SWITCH=1;
        }
    */
        
#if defined(__TPD_I2C_CHECK_SW_WORKAROUD__)
    tpd_i2c_check_sw_workaround_init();
#endif

    	/*frameware upgrade*/
#ifdef __FIRMWARE_UPDATE__
	//firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    //firmware_class = class_create(THIS_MODULE, client->name);
    firmware_class = class_create(THIS_MODULE, "mtk-tpd");//for apk update firmware,need set related files permission to 0777

    if (IS_ERR(firmware_class))
        printk("Failed to create class(firmware)!\n");
    firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");
    if (IS_ERR(firmware_cmd_dev))
        printk("Failed to create device(firmware_cmd_dev)!\n");

    // version
    if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
        printk("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
        printk("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
        printk("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
	// clear
    if (device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
        printk("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);

	dev_set_drvdata(firmware_cmd_dev, NULL);
#endif

#if 0//def TP_PROXIMITY_SENSOR

    if(device_create_file(firmware_cmd_dev, &dev_attr_proximity_sensor) < 0) // /sys/class/mtk-tpd/device/proximity_sensor
    {
        printk("Failed to create device file(%s)!\n", dev_attr_proximity_sensor.attr.name);
    }
    printk("create device file:%s\n", dev_attr_proximity_sensor.attr.name);
#endif

    printk("Touch Panel Device MSG2133 Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
    return 0;
}

static int __devexit tpd_i2c_remove(struct i2c_client *client)
{
    printk("TPD removed\n");
    return 0;
}


static int tpd_local_init(void)
{
 	bootMode = get_boot_mode();
	if(SW_REBOOT == bootMode)
		bootMode = NORMAL_BOOT;

    printk(" MSG2133 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

    if(i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        printk("unable to add i2c driver.\n");
        return -1;
    }
    
    if(tpd_load_status == 0)
    {
    	printk("add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
    
#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif
#if 0
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif
#endif
    printk("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;
    return 0;
}

static int tpd_resume(struct i2c_client *client)
{
    int retval = TPD_OK;
    
    printk("TPD resume\n");

#if defined(MSG2133_PS_SUPPORT)
	if(tpd_ps_flag==1)
		return 0;
#endif

    
    tpd_halt = 1;
#ifdef TPD_CLOSE_POWER_IN_SUSPEND
#ifdef POWER_MODE1
    //power on
	tp_power_on();
#else
    hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");
#endif
    msleep(1);
#endif

#ifdef POWER_MODE1
	tp_rst_high();
#else
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif

    msleep(200);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

    tpd_halt = 0;
#if defined(__TPD_I2C_CHECK_SW_WORKAROUD__)
    i2c_check_halt = 0;
    hrtimer_start(&tpd_i2c_check_timer, ktime_set(0, BAT_MS_TO_NS(CHECK_I2C_TIMER_COUNT)), HRTIMER_MODE_REL);
#endif
    return retval;
}

static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
    int retval = TPD_OK;
    
    printk("TPD suspend\n");

#if defined(MSG2133_PS_SUPPORT)
	if(tpd_ps_flag==1)
		return 0;
#endif

#if defined(__TPD_I2C_CHECK_SW_WORKAROUD__)
    i2c_check_halt = 1;
    hrtimer_cancel(&tpd_i2c_check_timer);
#endif
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    
#ifdef POWER_MODE1
	tp_rst_low();
#else
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
#endif
    
#ifdef TPD_CLOSE_POWER_IN_SUSPEND
    msleep(10);
#ifdef POWER_MODE1
	tp_power_off();
#else
    hwPowerDown(TPD_POWER_SOURCE, "TP");
#endif
#endif
    return retval;
}


static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "MSG2133",
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    printk("MediaTek MSG2133 touch panel driver init\n");
    i2c_register_board_info(0, &msg2133_i2c_tpd, 1);
    if(tpd_driver_add(&tpd_device_driver) < 0)
    {
        printk("add MSG2133 driver failed\n");
    }

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    printk("MediaTek MSG2133 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
