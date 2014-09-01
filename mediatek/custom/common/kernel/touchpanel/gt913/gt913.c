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
#include "cust_gpio_usage.h"

#include "gt913.h"         
#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif
#ifdef MT6577
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

extern struct tpd_device *tpd;

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct i2c_client*);
#endif

struct i2c_client *i2c_client_point = NULL;
struct task_struct *thread = NULL;
u8 gtp_rawdiff_mode = 0;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
 
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);

 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 
 #if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
static u8 config[GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
#if GTP_HAVE_TOUCH_KEY
	static const u16 touch_key_array[] = GTP_KEY_TAB;
	#define GTP_MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif
#define TPD_OK 0

struct touch_info {
    u32 y[5];
    u32 x[5];
    u32 p[5];
    int count;
};
 
#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = {KEY_BACK,KEY_HOMEPAGE,KEY_MENU};//TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] ={{67,840,100,80},{240,840,100,80},{450,840,100,80}}; //TPD_KEYS_DIM;
#endif

 static const struct i2c_device_id tpd_id[] = {{"goodix",0},{}};
// unsigned short force[] = {0,0x94,I2C_CLIENT_END,I2C_CLIENT_END}; 
// static const unsigned short * const forces[] = { force, NULL };
// static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata goodix_i2c_tpd={ I2C_BOARD_INFO("goodix", (0xba>>1))};
  
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "goodix",
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = tpd_id,
  .detect = tpd_detect,
 // .address_data = &addr_data,
 };
 
static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
 	 input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p);

	 printk("D[%4d %4d %4d] ", x, y, p);
	 input_mt_sync(tpd->dev);
	 TPD_DOWN_DEBUG_TRACK(x,y);
	 if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(x, y, 1);  
    }
 }
 
static  int tpd_up(int x, int y,int *count) {
	//	 input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
		 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	//	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	//	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 printk("release up ");
		 input_mt_sync(tpd->dev);
		 TPD_UP_DEBUG_TRACK(x,y);
		// (*count)--;
		if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(x, y, 0); 
    }       
	    return 0;
 }

/*******************************************************	
Function:
	Read data from the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
	
Output:
	numbers of i2c_msgs to transfer
*********************************************************/
static int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buffer[I2C_DEVICE_ADDRESS_LEN];
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            .addr = ((client->addr &I2C_MASK_FLAG)),
            .flags = 0,
            .buf = buffer,
            .len = I2C_DEVICE_ADDRESS_LEN,
            .timing = I2C_MASTER_CLOCK
        },
        {
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            .addr = ((client->addr &I2C_MASK_FLAG)),
            .flags = I2C_M_RD,
            .timing = I2C_MASTER_CLOCK
        },
    };

    if (rxbuf == NULL)
        return -1;

    GTP_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        msg[1].buf = &rxbuf[offset];

        if (left > MAX_TRANSACTION_LENGTH)
        {
            msg[1].len = MAX_TRANSACTION_LENGTH;
            left -= MAX_TRANSACTION_LENGTH;
            offset += MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while (i2c_transfer(client->adapter, &msg[0], 2) != 2)
        {
            retry++;

            if (retry == 20)
            {
                GTP_DEBUG("I2C read 0x%X length=%d failed\n", addr + offset, len);
                TPD_DMESG("I2C read 0x%X length=%d failed\n", addr + offset, len);
                return -1;
            }
        }
    }

    return 0;
}

static int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg =
    {
        //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
        .addr = ((client->addr &I2C_MASK_FLAG)),
        .flags = 0,
        .buf = buffer,
        .timing = I2C_MASTER_CLOCK,
    };


    if (txbuf == NULL)
        return -1;

 //   GTP_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        retry = 0;

        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        if (left > MAX_I2C_TRANSFER_SIZE)
        {
            memcpy(&buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE);
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy(&buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left);
            msg.len = left + I2C_DEVICE_ADDRESS_LEN;
            left = 0;
        }

   //     GTP_DEBUG("byte left %d offset %d\n", left, offset);

        while (i2c_transfer(client->adapter, &msg, 1) != 1)
        {
            retry++;

            if (retry == 20)
            {
                GTP_DEBUG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                TPD_DMESG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return -1;
            }
            else
                GTP_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1]);

        }

    }

    return 0;
}
int gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    u16 addr = 0;
    int ret = 0;

    addr = (buf[0]<<8) |buf[1];	

     ret = i2c_read_bytes(client,addr,&buf[2],len-2);

    if(ret == 0)
    {
	ret = 2;
    }
     return ret;
}

/*******************************************************	
Function:
	write data to the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
Output:
	numbers of i2c_msgs to transfer.
*********************************************************/
int gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    u16 addr = 0;
    int ret = 0;

    addr = (buf[0]<<8) |buf[1];	
    ret = i2c_write_bytes(client, addr, &buf[2], len-2);
    if(ret == 0)
    {
	ret = 1;
    }
  //  GTP_DEBUG("gtp_i2c_write ret=%d",ret);

    return ret;

	
}
/*******************************************************
Function:
	Reset chip Function.

Input:
	ms:reset time.
	
Output:
	None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client,s32 ms)
{
    GTP_DEBUG_FUNC();
	
    GTP_GPIO_OUTPUT(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
    msleep(ms);
    GTP_GPIO_OUTPUT(GPIO_CTP_EINT_PIN,(i2c_client_point->addr == 0x14));
    msleep(2);
    GTP_GPIO_OUTPUT(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
    msleep(6);//INT zcf 3ms->6ms
    GTP_GPIO_AS_INT(GPIO_CTP_EINT_PIN);
     if (GTP_GPIO_GET_VALUE(GPIO_CTP_EINT_PIN))
    {
        GTP_DEBUG("There is pull up resisitor attached on the INT pin~!");
        GTP_GPIO_OUTPUT(GPIO_CTP_EINT_PIN, 0);
        msleep(60);
        GTP_GPIO_AS_INT(GPIO_CTP_EINT_PIN);
    }else
	{   
	    msleep(60);
    	}
    return;
}

/*******************************************************
Function:
	Send config Function.

Input:
	client:	i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 1;
#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

 //   GTP_DEBUG_ARRAY(config,GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH);
    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif

    return ret;
}
/*******************************************************
Function:
	GTP initialize function.

Input:
	ts:	i2c client private struct.
	
Output:
	Executive outcomes.0---succeed.
*******************************************************/
static int gtp_init_panel(struct i2c_client *client)
{
    int ret = -1;
  
#if GTP_DRIVER_SEND_CFG
    int i;
    u8 check_sum = 0;
    u8 rd_cfg_buf[16];

    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 *send_cfg_buf[3] = {cfg_info_group1, cfg_info_group2, cfg_info_group3};
    u8 cfg_info_len[3] = {sizeof(cfg_info_group1)/sizeof(cfg_info_group1[0]), 
                          sizeof(cfg_info_group2)/sizeof(cfg_info_group2[0]),
                          sizeof(cfg_info_group3)/sizeof(cfg_info_group3[0])};
 //   GTP_DEBUG("len1=%d,len2=%d,len3=%d",cfg_info_len[0],cfg_info_len[1],cfg_info_len[2]);
    if ((!cfg_info_len[1]) && (!cfg_info_len[2]))
    {	
        rd_cfg_buf[GTP_ADDR_LENGTH] = 0; 
    }
    else
    {
        rd_cfg_buf[0] = GTP_REG_SENSOR_ID >> 8;
        rd_cfg_buf[1] = GTP_REG_SENSOR_ID & 0xff;
        ret = gtp_i2c_read(client, rd_cfg_buf, 3);
        if (ret < 0)
        {
            GTP_ERROR("Read SENSOR ID failed,default use group1 config!");
            rd_cfg_buf[GTP_ADDR_LENGTH] = 0;
        }
        rd_cfg_buf[GTP_ADDR_LENGTH] &= 0x03;
    }
    GTP_DEBUG("SENSOR ID:%d", rd_cfg_buf[GTP_ADDR_LENGTH]);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[rd_cfg_buf[GTP_ADDR_LENGTH]], GTP_CONFIG_LENGTH);

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);
#endif  //endif GTP_CUSTOM_CFG

        config[TRIGGER_LOC] &= 0xfc; 
        config[TRIGGER_LOC] |= 0x01;

/*
    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
*/
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < GTP_CONFIG_LENGTH; i++)
    {
        check_sum += config[i];
    }
    config[GTP_CONFIG_LENGTH] = (~check_sum) + 1;
    
#else //else DRIVER NEED NOT SEND CONFIG

    ret = gtp_i2c_read(client, config, GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("GTP read resolution & max_touch_num failed, use default value!");
    }
#endif //endif GTP_DRIVER_SEND_CFG

    GTP_DEBUG_FUNC();

    ret = gtp_send_cfg(client);
    if (ret < 0)
    {
        GTP_ERROR("Send config error.");
    }

    GTP_DEBUG("gtp_init_panel pass ");

    msleep(10);

    return 0;
}

/*******************************************************
Function:
	Read goodix touchscreen version function.

Input:
	client:	i2c client struct.
	version:address to store version info
	
Output:
	Executive outcomes.0---succeed.
*******************************************************/
int gtp_read_version(struct i2c_client *client, u16* version)
{
    int ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf) + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed"); 
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }

    GTP_INFO("IC VERSION:%02x%02x%02x%02x_%02x%02x", 
              buf[5], buf[3], buf[4], buf[2], buf[7], buf[6]);

    return ret;
}

/*******************************************************
Function:
	I2c test Function.

Input:
	client:i2c client.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static int gtp_i2c_test(struct i2c_client *client)
{
	u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	u8 retry = 0;
	int ret = -1;

	GTP_DEBUG_FUNC();

	while(retry++ < 5)
	{
		ret = gtp_i2c_read(client, test, 3);
		if (ret > 0)
		{
			return ret;
		}
		GTP_ERROR("GTP i2c test failed time %d,ret=%d.",retry,ret);
		msleep(10);
	}
	return ret;
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 { 
	int i = 0;
	u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
	s32 ret = -1;
	u8  finger = 0;
	u8* coor_data = NULL;
	u8  key_value = 0;
	static u8 pre_touch = 0;

	p_point_num = point_num;
	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));

	ret = gtp_i2c_read(i2c_client_point, point_data, 12);

	if (ret < 0)
	{
		GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
		return 0;
	}
	/*
	TPD_DMESG("data: \n");

	for (i = 0;  i<=11; i++)
	{
		printk("%x ", point_data[i]);
		if (i % 8 == 0)
		printk("\n");
	}
	printk("\n");
*/
	finger = point_data[GTP_ADDR_LENGTH];    
	if((finger & 0x80) == 0)
	{
		ret = gtp_i2c_write(i2c_client_point, end_cmd, 3);
		if (ret < 0)
		{
			GTP_INFO("I2C write error!"); 
		}
		return 0;

	}

	point_num = finger & 0x0f;
	if (point_num > GTP_MAX_TOUCH)
	{
		return 0;
	}

	if (point_num > 1)
	{
		u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

		ret = gtp_i2c_read(i2c_client_point, buf, 2 + 8 * (point_num - 1)); 
		memcpy(&point_data[12], &buf[2], 8 * (point_num - 1));
	}
#if GTP_HAVE_TOUCH_KEY
	key_value = point_data[3 + 8 * point_num];

	for (i = 0; i < GTP_MAX_KEY_NUM; i++)
	{
		//point_num = 0;
		input_report_key(tpd->dev, touch_key_array[i], key_value & (0x01<<i));   
		GTP_DEBUG("touch key :key_value = %x", key_value);
	}
#endif

	GTP_DEBUG("pre_touch:%02x, finger:%02x.point_num:%d.", pre_touch, finger,point_num);
	if (point_num)
	{
		for (i = 0; i < point_num; i++)
		{
			coor_data = &point_data[i * 8 + 3];
			cinfo->x[i]  = coor_data[1] | coor_data[2] << 8;
			cinfo->y[i]  = coor_data[3] | coor_data[4] << 8;
			cinfo->p[i]  =coor_data[0];
		}
		cinfo->count = point_num;
	}

	pre_touch = point_num;
	input_report_key(tpd->dev, BTN_TOUCH, (point_num || key_value));
	
       if(gtp_rawdiff_mode == 0)
       {
		ret = gtp_i2c_write(i2c_client_point, end_cmd, 3);
		if (ret < 0)
		{
			GTP_INFO("I2C write error!"); 
		}
       }
	return true;

 };

static int touch_event_handler(void *unused)
 {
	struct touch_info cinfo = {0};
	struct touch_info pinfo = {0};      
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		//	msleep(15);
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);


		if (tpd_touchinfo(&cinfo, &pinfo)) 
		{
		//	GTP_DEBUG("point_num = %d\n",point_num);

			if(point_num >0) 
			{   
			       int i = 0;

			       for(i=0;i < point_num;i++)
			       {
					tpd_down(cinfo.x[i], cinfo.y[i],  cinfo.p[i]); 
				}
/*
				tpd_down(cinfo.x[0], cinfo.y[0],  cinfo.p[0]); 
				if(point_num>1)
				{
					tpd_down(cinfo.x[1], cinfo.y[1],  cinfo.p[1]);
					if(point_num >2) 
					{
						tpd_down(cinfo.x[2], cinfo.y[2],  cinfo.y[0]);
					} 
					if(point_num>3) 
					{
						tpd_down(cinfo.x[3], cinfo.y[3], 1);
					}
					if(point_num > 4)
					{
						tpd_down(cinfo.x[4], cinfo.y[4], 1);
					}
				}
				*/
				input_sync(tpd->dev);
				GTP_DEBUG("press --->\n");

			}
			else
			{
				GTP_DEBUG("release --->\n"); 
				tpd_up(pinfo.x[0], pinfo.y[0], 0);
				//input_mt_sync(tpd->dev);
				input_sync(tpd->dev);
			}
		}

       }while(!kthread_should_stop());
 
	 return 0;
 }
 

static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);	
	return 0;
}
 
 static void tpd_eint_interrupt_handler(void)
 {
	// TPD_DMESG("TPD 3 has been triggered\n");
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	char data[3];
	int ret = -1;
	u16 version_info;

	i2c_client_point = client;
	//client->timing = 300;
//	TPD_DMESG("ENTER tpd_probe=%d\n",client);
#ifdef MT6577
	//power on, need confirm with SA
	//hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	    hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_2800, "TP");      
#endif
	gtp_reset_guitar(client,20);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

	ret = gtp_i2c_test(client);
//	GTP_ERROR("ret=%d",ret);
	if (ret < 0)
	{
		GTP_ERROR("I2C communication ERROR!");
	}
	gtp_rawdiff_mode = 0;
#if GTP_AUTO_UPDATE
       ret = gup_init_update_proc(client);
       if (ret < 0)
       {
          GTP_ERROR("Create update thread error.");
       }
#endif

	ret = gtp_init_panel(client);
//	GTP_ERROR("gtp_init_panel ret=%d",ret);

	if (ret < 0)
	{
		GTP_ERROR("GTP init panel failed.");
	}
#if 0
	ret = gtp_read_version(client, &version_info);
//	GTP_ERROR("gtp_read_version ret=%d",ret);
	if (ret < 0)
	{
		GTP_ERROR("Read version failed.");
	}
#endif
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{ 
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}


	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 0); 
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	tpd_load_status = 1;
#if GTP_CREATE_WR_NODE

       init_wr_node(client);
#endif	
	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
	return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 {
	GTP_DEBUG("TPD removed\n");
#if GTP_CREATE_WR_NODE
       uninit_wr_node();
#endif
	return 0;
 }
 
 
 static int tpd_local_init(void)
 {

	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	if(tpd_load_status == 0) 
	{
		TPD_DMESG("himax add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	//TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
	return 0; 
 }


 static int tpd_resume(struct i2c_client *client)
 {
    u8 retry = 0;
    int ret = -1;
//    GTP_DEBUG("TPD wake up\n");

#if GTP_POWER_CTRL_SLEEP
    while(retry++ < 5)
    {
        gtp_reset_guitar(i2c_client_point,20);
        ret = gtp_send_cfg(i2c_client_point);
        if (ret > 0)
        {
            mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

            GTP_DEBUG("Wakeup sleep send config success.");
            return ret;
        }
    }
#else
    while(retry++ < 10)
    {
        ret = gtp_i2c_test(i2c_client_point);
        if (ret > 0)
        {
            mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
            GTP_DEBUG("GTP wakeup sleep.");
            return ret;
        }
        gtp_reset_guitar(i2c_client_point,20);
    }
#endif

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;

 }
 
 static int tpd_suspend(struct i2c_client *client, pm_message_t message)
 {
	s8 retry = 0;
	int ret = -1;	
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

//	GTP_DEBUG("TPD enter sleep\n");
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);

	while(retry++ < 5)
	{
		ret = gtp_i2c_write(i2c_client_point, i2c_control_buf, 3);
	//	GTP_DEBUG("GTP enter sleep ret=%d",ret);

		if (ret > 0)
		{
			GTP_DEBUG("GTP enter sleep!");
			return TPD_OK;
		}
		msleep(10);
	}
	return TPD_OK;
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "goodix",
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
	 printk("MediaTek GOODIX_TS touch panel driver init\n");
        i2c_register_board_info(0, &goodix_i2c_tpd, 1);
	 if(tpd_driver_add(&tpd_device_driver) < 0)
	 {
		 TPD_DMESG("add HIMAX_TS driver failed\n");
	 }
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) 
 {
	 TPD_DMESG("MediaTek GOODIX_TS touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);

