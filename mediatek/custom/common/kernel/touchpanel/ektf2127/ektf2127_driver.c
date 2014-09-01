/* drivers/input/touchscreen/ektf2k.c - ELAN EKTF2K touchscreen driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
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

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "tpd.h"
#include <cust_eint.h>
#include "tpd_custom_ektf2127.h"
#include "cust_gpio_usage.h"
#include "ektf2127_driver.h"


#ifdef MT6575 
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif

#ifdef MT6577
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif


uint16_t checksum_err=0;
uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;
int X_RESOLUTION=0x00;
int Y_RESOLUTION=0x00;
int FW_ID=0x00;
int work_lock=0x00;
int button_state = 0;
static int key_pressed = -1;

extern struct tpd_device *tpd;
static int tpd_flag = 0;
struct i2c_client *elan_i2c_client = NULL;
struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

struct elan_ktf2k_i2c_platform_data ts_elan_ktf2k_data[] = {
        {
                .version = 0x0001,
                .abs_x_min = 0,
                .abs_x_max = TP_EKTF_WIDTH,   //LG 9.7" Dpin 2368, Spin 2112
                .abs_y_min = 0,
                .abs_y_max = TP_EKTF_HEIGHT,   //LG 9.7" Dpin 1728, Spin 1600
                .intr_gpio = GPIO_CTP_EINT_PIN,//TEGRA_GPIO_PJ7,
        },
};

static struct elan_ktf2k_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf2k_ts_poll(struct i2c_client *client);
static int __hello_packet_handler(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf2k_ts_late_resume(struct early_suspend *h);
#endif
//////////////////////////////////////////////////////////////////////////////////////////////
// For Firmware Update 
int elan_iap_open(struct inode *inode, struct file *filp){ 
	ELAN_PRINT("into elan_iap_open\n");
		if (private_ts == NULL)  ELAN_PRINT("private_ts is NULL~~~");
		
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp){    
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
    int ret;
    char *tmp;
    ELAN_PRINT("into elan_iap_write\n");

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }
	
    ret = i2c_master_send(private_ts->client, tmp, count);
    if (ret != count) ELAN_PRINT("ELAN i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    return ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
    char *tmp;
    int ret;  
    long rc;
    printk("[ELAN]into elan_iap_read\n");
   
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

    ret = i2c_master_recv(private_ts->client, tmp, count);

    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    return ret;
}

static long elan_iap_ioctl( struct file *filp,    unsigned int cmd, unsigned long arg){

	int __user *ip = (int __user *)arg;
	printk("[ELAN]into elan_iap_ioctl\n");
	printk("cmd value %x\n",cmd);
	
	switch (cmd) {        
		case IOCTL_I2C_SLAVE: 
			private_ts->client->addr = (int __user)arg;
			break;   
		case IOCTL_MAJOR_FW_VER:            
			break;        
		case IOCTL_MINOR_FW_VER:            
			break;        
		case IOCTL_RESET:
			break;
		case IOCTL_IAP_MODE_LOCK:
			work_lock=1;
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			work_lock=0;
			//Elan modify
			//if (gpio_get_value(private_ts->intr_gpio))
			if (mt_get_gpio_in(private_ts->intr_gpio))
    			{
        			enable_irq(private_ts->client->irq);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(private_ts->client);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			__fw_packet_handler(private_ts->client);
			return FW_ID;
			break;
		case IOCTL_I2C_INT:
			put_user(gpio_get_value(private_ts->intr_gpio), ip);
			break;			
		default:            
			break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {    
        .open =         elan_iap_open,    
        .write =        elan_iap_write,    
        .read = 	elan_iap_read,    
        .release =	elan_iap_release,    
	.unlocked_ioctl=elan_iap_ioctl, 
 };

// End Firmware Update
///////////////////////////////////////////////////////////////////////////////////

static ssize_t elan_ktf2k_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

        //Elan modify
	//ret = gpio_get_value(ts->intr_gpio);
	ret = mt_get_gpio_in(ts->intr_gpio);
	ELAN_PRINT("GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		ELAN_PRINT(KERN_ERR "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		ELAN_PRINT(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

////////////////////////////////////////////////////////////////////////////////
// Start Firmware Update in Driver
#ifdef IAP_PORTION
static uint8_t I2C_DATA = 0x2a>>1;/*I2C devices address*/

#define PAGERETRY  30

static uint8_t file_fw_data[] = {
#include "fw_data.i"
};

static int request_firmware(struct i2c_client *client);

enum {
	PageSize		= 132,
	PageNum			= 249,
	ACK_Fail		= 0x00,
	ACK_OK	    = 0xAA,
	ACK_REWRITE = 0x55,
};

enum {
	E_FD = -1,
};

static int EnterISPMode(struct i2c_client *client, uint8_t *isp_cmd, int cmd_len)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, isp_cmd, cmd_len);
	if (len != cmd_len) {
		printk("[ELAN] ERROR: EnterISPMode fail! len=%d", len);
		return -1;
	} else
		printk("[ELAN] IAPMode write data successfully!");

	return 0;
}

static int WritePage(uint8_t *szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage, byte);
	if (len != byte) {
		printk("[ELAN] ERROR: write page error, write error. err=%d", len);
		return -1;
	}

	return 0;
}

static int GetAckData(struct i2c_client *client)
{
	int len = 0;
	char buff[4] = {0};

	len = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		printk("[ELAN] ERROR: read data error, write 50 times error. len=%d\r", len);
		return -1;
	}
	
	printk("%s(): buf[0] = 0x%x, buf[1] = 0x%x, buf[2] = 0x%x, buf[3] = 0x%x", __func__, buff[0], buff[1], buff[2], buff[3]);

	if (buff[0] == 0xaa || buff[1] == 0xaa)
		printk("[ELAN] 0x%2x,0x%2x", buff[0], buff[1]);
	else 
		printk("[ELAN] GetAckData ERROR 0x%2x,0x%2x", buff[0], buff[1]);
		
	if (buff[0] == 0xaa || buff[1] == 0xaa) {
		return ACK_OK;
	} else if (buff[0] == 0x55) {
		return ACK_REWRITE;
	} else {
		return ACK_Fail;
	}

	printk("^&&@@#!@!@");

	return 0;
}

static void print_progress(int page, int ic_num, int j)
{
	int i, percent, page_tatol, percent_tatol;
	char str[256];

	str[0] = '\0';
	for (i = 0; i < ((page) / 10); i++) {
		str[i] = '#';
		str[i + 1] = '\0';
	}

	page_tatol = page + 249 * (ic_num - j);
	percent = ((100 * page) / (249));
	percent_tatol = ((100 * page_tatol) / (249 * ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249 * ic_num))
		percent_tatol = 100;

	printk("\rprogress %s| %d%%", str, percent);

	if (page == (249))
		printk("\n");
}

static int Update_FW(void)
{
	int res = 0;
	int iPage = 0, ackcnt = 0;
	int i = 0;
	uint8_t data;
	uint8_t *szBuff = NULL;
	int byte_count;
	int curIndex = 0;
	uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};

	//printk("[ELAN] Update_FW: ic_num=%d", ic_num);

		data = I2C_DATA;
		printk("[ELAN] data=0x%x\n", data);
		
		private_ts->client->addr = data;	//Set I2C slave address

			if(RECOVERY != 0x80) {
					EnterISPMode(private_ts->client, isp_cmd, 4);	 		 //Step 1 enter ISP mode
			} else {
				RECOVERY = 0x00;
				goto write_fw;
			}

write_fw:
		mdelay(600);
		res = i2c_master_send(private_ts->client, &data, sizeof(data));
		if (res != sizeof(data)) {
			printk("[ELAN] dummy %d",res);
		} else {
			printk("[elan] data = %x, res = %d", data, res);
		}
		
		//mdelay(10);
		printk("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");

PAGE_REWRITE:
		for (iPage = 1; iPage <= PageNum; iPage++) {
			for (byte_count = 1; byte_count <= 17; byte_count++) {
				if (byte_count != 17) {
					printk("[ELAN] byte %d\n", byte_count);
					printk("[elan] curIndex =%d\n", curIndex);

					szBuff = file_fw_data + curIndex;
					curIndex = curIndex + 8;
					res = WritePage(szBuff, 8);
				} else {
					printk("byte %d\n", byte_count);

					printk("curIndex =%d\n", curIndex);
					szBuff = file_fw_data + curIndex;
					curIndex = curIndex + 4;

					res = WritePage(szBuff, 4);
				}
				//msleep(100);
				printk("sleeping 100ms...");
			}

			res = GetAckData(private_ts->client);
			if (ACK_OK != res) {
				//msleep(600);
				printk("[ELAN] ERROR: GetAckData fail! res=%d\n", res);
				ackcnt = ackcnt + 1;
				if (ackcnt == PAGERETRY) {
					printk("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
					return E_FD;
				} else {
					printk("[ELAN] ---%d--- page ReWrite %d times!\n", iPage, ackcnt);
					goto PAGE_REWRITE;
				}
			} else
				ackcnt = 0;

			printk("IC address : 0x%02x\n", private_ts->client->addr);
			print_progress(iPage, 1, 1);
			msleep(10);
		} // end of for(iPage = 1; iPage <= PageNum; iPage++)
		
		mdelay(300);
		printk("[ELAN] update 0x%02x Firmware successfully!!!\n",  data);

		//reset
		//ektf2040_reset();
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0x0); //setup RST pin to always GPIO mode.
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(100);

		res = elan_ktf2k_ts_poll(private_ts->client);
		if (res < 0)
			printk("poll int failed in %s(): %d", __FUNCTION__, __LINE__);

		printk("[ELAN] read Hello packet data!");

		private_ts->client->addr = 0x2a>>1;
		__hello_packet_handler(private_ts->client);

		printk("[ELAN] Update ALL Firmware successfully!!!!!!!");
		mdelay(2000);
	
  	return 0;
}

static int request_firmware(struct i2c_client *client)
{
	int ret;
	int New_FW_ID;
	int New_FW_VER;
	printk("firmware checking done.");
	//printk("11111111111111111111");

	//lock work function
	//disable_irq(CUST_EINT_TOUCH_PANEL_NUM);
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	//cancel_work_sync(&private_ts->work);  //longxuewei
	mdelay(20);

	//printk("22222222222222222222");

	//lock suspend
	//power_lock = 1;

	//printk("333333333333333333");
//printk(" [7bd0]=0x%02x,  [7bd1]=0x%02x, [7bd2]=0x%02x, [7bd3]=0x%02x\n", file_fw_data[31696],file_fw_data[31697],file_fw_data[31698],file_fw_data[31699]);
	New_FW_ID = (file_fw_data[31699]<<8  | file_fw_data[31698]) ;              

	New_FW_VER = (file_fw_data[31697]<<8  | file_fw_data[31696]) ; 

	printk(" FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);          

	printk(" FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);

	if ((FW_VERSION>New_FW_VER)||(FW_VERSION==New_FW_VER))
	{
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		return 1;
	}		

	ret = Update_FW();
	if (ret != 0) {
		printk("[elan] update firmware failed in %s()", __FUNCTION__);
		return ret;
	}

	//power_lock = 0;
	//enable_irq(CUST_EINT_TOUCH_PANEL_NUM);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	return 0;
}

#endif	//end IAP_PORTION
// End Firmware Update in Driver 

////////////////////////////////////////////////////////////////////////////////////


static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 10;

	do {
		//Elan modify
		//status = gpio_get_value(ts->intr_gpio);
		status = mt_get_gpio_in(ts->intr_gpio);
		ELAN_PRINT("%s: status = %d\n", __func__, status);
		retry--;
		mdelay(20);
	} while (status == 1 && retry > 0);

	ELAN_PRINT("%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;
	ELAN_PRINT("%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;
//ELAN_PRINT("addr=%x,timing=%d\n", client->addr,client->timing);
	if ((i2c_master_send(client, cmd, 4)) != 4) {
		ELAN_PRINT("%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;

	else {
		if (i2c_master_recv(client, buf, size) != size ||buf[0] != CMD_S_PKT)
		{
			TPD_DMESG("%s: %d,buf[0]=%x,size=%d,\n", __func__,__LINE__, buf[0],size);
			return -EINVAL;
		}
	}

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[4] = { 0 };
	uint8_t buf_recv1[4] = { 0 };

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		TPD_DMESG(" %s: failed!\n", __func__);
		return -EINVAL;
	}
	rc = i2c_master_recv(client, buf_recv, 4);
	ELAN_PRINT(" %s: hello packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
	{
		rc = elan_ktf2k_ts_poll(client);
		if (rc < 0) {
			dev_err(&client->dev, "[elan] %s: failed!\n", __func__);
			return -EINVAL;
		}

		rc = i2c_master_recv(client, buf_recv1, 4);
		ELAN_PRINT(" %s: recovery hello packet %2x:%2X:%2x:%2x\n", __func__, buf_recv1[0], buf_recv1[1], buf_recv1[2], buf_recv1[3]);
		RECOVERY=0x80;
		return RECOVERY;
	}
	return 0;
}


static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t buf_recv[4] = {0};
#ifdef ELAN_PRT_BOOT_VER //Elan add, Print Boot Code Version, Paul@20120517 
	uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01};/* Get BootCode Version*/
	int boot_ver = 0;
#endif //Elan end

// Firmware version
	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;
// X Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;

	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution =minor;
	X_RESOLUTION = ts->x_resolution;
// Y Resolution	
	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;

	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution =minor;
	Y_RESOLUTION = ts->y_resolution;
// Firmware ID
	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

#ifdef ELAN_PRT_BOOT_VER //Elan add, Print Boot Code Version, Paul@20120517 
// Boot Code Version
	rc = elan_ktf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
	{
		TPD_DMESG("[elan] %s: fail to get boot code version! error = %d.\n",
			__func__, rc);
		return rc;
	}
	TPD_DMESG("[elan] %s: dump bootcode data - %02x, %02x, %02x, %02x\n",
		__func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	boot_ver = (major << 8) | minor;
	TPD_DMESG("[elan] %s: boot version: 0x%4.4x\n",
			__func__, boot_ver);
#endif //Elan end

	TPD_DMESG("[elan] %s: firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);
	TPD_DMESG("[elan] %s: firmware ID: 0x%4.4x\n",
			__func__, ts->fw_id);
	TPD_DMESG("[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, ts->x_resolution, ts->y_resolution);
	
	return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];
	//Elan add
	#ifdef REVERSE_X_AXIS
        *x = ELAN_X_MAX - *x;
    #endif
	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];
	#ifdef REVERSE_Y_AXIS
        *y = ELAN_Y_MAX - *y;
    #endif
	return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	int rc;
	mdelay(500);

	rc = __hello_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;

	ELAN_PRINT(" %s: hello packet got.\n", __func__);
// for firmware update
	if(rc==0x80)
	{
		return rc;
	}
// end firmware update

	rc = __fw_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;
	ELAN_PRINT(" %s: firmware checking done.\n", __func__);

hand_shake_failed:
	return rc;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	ELAN_PRINT(" %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	ELAN_PRINT(" dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	power_state = buf[1];
	dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	ELAN_PRINT(" power state = %s\n",power_state == PWR_STATE_DEEP_SLEEP ?
		"Deep Sleep" : "Normal/Idle");

	return power_state;
}



static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf)
{
	int rc, ret, bytes_to_recv = PACKET_SIZE;
	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);
	rc = i2c_master_recv(client, buf, 8);
	//ELAN_PRINT(" %x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	if (rc != 8) 
		return -1;
	mdelay(1);

	rc = i2c_master_recv(client, buf+ 8, 8);
	//ELAN_PRINT(" %x %x %x %x %x %x %x %x\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
	if (rc != 8) 
		return -1;

	rc = i2c_master_recv(client, buf+ 16, 2);
	//ELAN_PRINT(" %x %x \n", buf[16], buf[17]);
	if (rc != 2) 
		return -1;
	msleep(1);
	return rc;


}

#ifdef SOFTKEY_AXIS_VER //SOFTKEY is reported via AXIS
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	//struct input_dev *idev = ts->input_dev;
	struct input_dev *idev = tpd->dev;  //longxuewei
	uint16_t x, y;
	uint16_t fbits=0, checksum=0;
	uint8_t i, num;

	num = buf[IDX_NUM] & 0x7; 
	ELAN_PRINT(" buf[0] = 0x%x\n",buf[0]);


	switch (buf[0]) {
	    case NORMAL_PKT:
		fbits = buf[1] & 0xF8;
		if (num == 0) {
			ELAN_PRINT("no press\n");
			
			if(key_pressed < 0){
				//input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
				//input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
				input_report_key(idev, BTN_TOUCH, 0);
				input_mt_sync(idev);
			}
			else{
				ELAN_PRINT(" KEY_RELEASE: key_code:%d\n",OSD_mapping[key_pressed].key_event);
				input_report_key(idev, OSD_mapping[key_pressed].key_event, 0);
				key_pressed = -1;
			}
			input_sync(idev);
		} else {
			uint8_t idx;
			uint8_t reported = 0;

			ELAN_PRINT("%d fingers\n", num);
			idx=IDX_FINGER;

			for (i = 0; i < FINGER_NUM; i++) {
				ELAN_PRINT("fbits = 0x%x\n",fbits);
				if ((fbits & FINGER_ID)) 
				{
					elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
					//ELAN_PRINT("before--Finger%d, x:%d y:%d\n",(i+1),x,y);
					x = ( x * TP_EKTF_WIDTH)/ELAN_X_MAX;
					y= ( y * TP_EKTF_HEIGHT)/ELAN_Y_MAX;
					// ELAN_PRINT("after---Finger%d, x:%d y:%d\n",(i+1),x,y);
				     	if(y< TP_EKTF_HEIGHT )
				     	{
						input_report_key(idev, BTN_TOUCH, 1);
						input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
						input_report_abs(idev, ABS_MT_POSITION_X, x);
						input_report_abs(idev, ABS_MT_POSITION_Y, y);
						input_report_abs(idev, ABS_MT_TRACKING_ID, i);
						input_mt_sync(idev);
					}
					else
					{
				    		int i=0;
				    		for(i=0;i<TPD_KEY_COUNT;i++){
					    	if((x > OSD_mapping[i].left_x) && (x < OSD_mapping[i].right_x)){
							ELAN_PRINT(" KEY_PRESS: key_code:%d\n",OSD_mapping[i].key_event);				    		
					    		input_report_key(idev, OSD_mapping[i].key_event, 1);
					    		key_pressed = i;
					    	}
				    		}
					
					}
					reported++;
			     	} // end if finger status
				fbits = fbits >> 1;
				idx += 3;
			} // end for 
			if (reported)
				input_sync(idev);
			else {
				input_mt_sync(idev);
				input_sync(idev);
			}

		}

		break;
	   default:
		ELAN_PRINT(" %s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	   } 

	return;
}
#elif defined(SOFTKEY_BTN_VER) //SOFTKEY is reported via BTN bit
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	//struct input_dev *idev = ts->input_dev;
	struct input_dev *idev = tpd->dev;  //longxuewei
	uint16_t x, y;
	uint16_t fbits=0, checksum=0;
	uint8_t i, num;
	uint8_t button_content = 0;

	num = buf[IDX_NUM] & 0x7; 
	//ELAN_PRINT("[elan] buf[0] = 0x%x\n",buf[0]);

	switch (buf[0]) {
	   case NORMAL_PKT:
		fbits = buf[1] & 0xF8;
		if (num == 0) {
			ELAN_PRINT( "no press\n");
			button_content = buf[BUTTON_ID_INDEX];
			ELAN_PRINT("button buf[%d]=%0x, button_content=%0x\n", BUTTON_ID_INDEX, buf[BUTTON_ID_INDEX], 
						button_content);
			if (button_content & ELAN_KEY_0) {  //0x04
				ELAN_PRINT("ELAN_KEY_0 press\n");
				button_state=ELAN_KEY_0;
				input_report_key(idev, TPD_KEYS[0], 1);
			} else if (button_content & ELAN_KEY_1) { //0x01
				ELAN_PRINT("ELAN_KEY_1 press\n");
				button_state=ELAN_KEY_1;
				input_report_key(idev, TPD_KEYS[1], 1);
			} else if (button_content & ELAN_KEY_2) {  //0x02
				ELAN_PRINT("ELAN_KEY_2 press\n");
				button_state=ELAN_KEY_2;
				input_report_key(idev, TPD_KEYS[2], 1);
			} else if(button_content & ELAN_KEY_3) {
				ELAN_PRINT("ELAN_KEY_3 press\n");
				button_state=ELAN_KEY_3;
				input_report_key(idev, TPD_KEYS[3], 1);			
			} else if (button_state == ELAN_KEY_0) {
				ELAN_PRINT("ELAN_KEY_0 release\n");
				button_state=0;
				input_report_key(idev, TPD_KEYS[0], 0);
			} else if (button_state == ELAN_KEY_1) {
				ELAN_PRINT("ELAN_KEY_1 release\n");
				button_state=0;
				input_report_key(idev, TPD_KEYS[1], 0);				
			} else if (button_state == ELAN_KEY_2) {
				ELAN_PRINT("ELAN_KEY_2 release\n");
				button_state=0;
				input_report_key(idev, TPD_KEYS[2], 0);
				input_mt_sync(idev);				
			} else if (button_state == ELAN_KEY_3) {
				ELAN_PRINT("ELAN_KEY_3 release\n");
				button_state=0;
				input_report_key(idev, TPD_KEYS[3], 0);						
			} else {
				//TOUCH release
				ELAN_PRINT("no press, release\n");
				
				//input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
				//input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
				input_report_key(idev, BTN_TOUCH, 0);
				input_mt_sync(idev);
			}
			input_sync(idev);
		} else {
			uint8_t idx;
			uint8_t reported = 0;

			ELAN_PRINT(" %d fingers\n", num);
            		idx=IDX_FINGER;

			for (i = 0; i < FINGER_NUM; i++) {
			  //ELAN_PRINT(" fbits = 0x%x\n",fbits);
			  if ((fbits & FINGER_ID)) {
			     elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
				//ELAN_PRINT("before--- Finger%d, x:%d y:%d\n",(i+1),x,y);
				x = ( x * TP_EKTF_WIDTH)/ELAN_X_MAX;
				y= ( y * TP_EKTF_HEIGHT)/ELAN_Y_MAX;
				//ELAN_PRINT("after--- Finger%d, x:%d y:%d\n",(i+1),x,y);	
				input_report_key(idev, BTN_TOUCH, 1);
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
				input_report_abs(idev, ABS_MT_POSITION_X, x);
				input_report_abs(idev, ABS_MT_POSITION_Y, y);
    				input_report_abs(idev, ABS_MT_TRACKING_ID, i);
				input_mt_sync(idev);
				reported++;
 			  } // end if finger status

			  fbits = fbits >> 1;
			  idx += 3;
			} // end for
			if (reported) {				
				input_sync(idev);
			}
			else {
				input_mt_sync(idev);
				input_sync(idev);
			}
		}

		break;
	   default:
		ELAN_PRINT("%s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	   } 
	return;
}
#else // no SOFTKEY defined
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	//struct input_dev *idev = ts->input_dev;
	struct input_dev *idev = tpd->dev;  //longxuewei
	uint16_t x, y;
	uint16_t fbits=0, checksum=0;
	uint8_t i, num;
	uint8_t button_content = 0;

	num = buf[IDX_NUM] & 0x7; 
	//ELAN_PRINT("buf[0] = 0x%x\n",buf[0]);

	switch (buf[0]) {
	   case NORMAL_PKT:
		fbits = buf[1] & 0xF8;
		if (num == 0) {
			ELAN_PRINT( "no press\n");
			if(key_pressed < 0){
				//input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
				//input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
				input_report_key(idev, BTN_TOUCH, 0);
				input_mt_sync(idev);
				input_sync(idev);
				ELAN_PRINT("up!!\n");
			}
		} else {
			uint8_t idx;
			uint8_t reported = 0;

			ELAN_PRINT(" %d fingers\n", num);
            		idx=IDX_FINGER;

			for (i = 0; i < FINGER_NUM; i++) {
			  if ((fbits & FINGER_ID)) {
				elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
			   	//ELAN_PRINT( " before---Finger%d, x:%d y:%d\n",(i+1),x,y);
				x = ( x * TP_EKTF_WIDTH)/ELAN_X_MAX;
				y= ( y * TP_EKTF_HEIGHT)/ELAN_Y_MAX;
				//ELAN_PRINT( " after---Finger%d, x:%d y:%d\n",(i+1),x,y);

			     	if(y< TP_EKTF_HEIGHT ) {   
					input_report_key(idev, BTN_TOUCH, 1);
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					input_report_abs(idev, ABS_MT_TRACKING_ID, i);
				//ELAN_PRINT("[%d] down:x=%d ,y =%d|||fbits=%d, idx=%d,reported=%d\n",i,x,y,fbits,idx,reported);
					input_mt_sync(idev);
					reported++;
			  	} // end if border
			   } // end if finger status

			  fbits = fbits >> 1;
			  idx += 3;
			} // end for
			if (reported) {	
				input_sync(idev);
			}
			else {
				input_mt_sync(idev);
				input_sync(idev);
			}
		}
		
		break;
	   default:
		ELAN_PRINT("%s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	   } 
	return;
}
#endif
/////////////////////////////////////////////////////////////////////////////////////////
/*
static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf2k_ts_data *ts =
		container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[PACKET_SIZE] = { 0 };
	ELAN_PRINT("%s\n", "elan_ktf2k_ts_work_func");
	if(work_lock==0)
	{
		//Elan modify
		//if (gpio_get_value(ts->intr_gpio))
		if (mt_get_gpio_in(ts->intr_gpio))
		{
			ELAN_PRINT("%s : Detected a gitter on INT pin\n", __func__);
			enable_irq(ts->client->irq);
			return;
		}
		rc = elan_ktf2k_ts_recv_data(ts->client, buf);
		if (rc < 0)
		{
			enable_irq(ts->client->irq);
			return;
		}
		ELAN_PRINT("jeeray---->%x, %x ,%x, %x\n", buf[0], buf[1], buf[2], buf[7]);
		elan_ktf2k_ts_report_data(ts->client, buf);
		enable_irq(ts->client->irq);
	}
	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
#ifdef NON_MTK_IRQ
	struct elan_ktf2k_ts_data *ts = dev_id;
#else
	struct elan_ktf2k_ts_data *ts = private_ts;
#endif
	struct i2c_client *client = ts->client;

	ELAN_PRINT(" %s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);
	

	return IRQ_HANDLED;
}
*/
////////////////////////////////////////////////////////////////////////////////////////////

 static void elan_ktf2k_eint_interrupt_handler(void)
 {
	 ELAN_PRINT("%s\n", __func__);
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);	 
 }

static int touch_event_handler(void *unused)
 {
	int rc;
 	uint8_t buf[PACKET_SIZE] = { 0 };

	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
 
	 do
	 {
		 set_current_state(TASK_INTERRUPTIBLE); 
		  wait_event_interruptible(waiter,tpd_flag!=0);
						 
		tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);
		 
		if(work_lock==0)
		{	
			rc = elan_ktf2k_ts_recv_data(elan_i2c_client, buf);
			if (rc >= 0)
			{
			//ELAN_PRINT("jeeray---->%x, %x ,%x, %x\n", buf[0], buf[1], buf[2], buf[7]);
			elan_ktf2k_ts_report_data(elan_i2c_client, buf);
			}
		}

 	}while(!kthread_should_stop());
 
	 return 0;
 }
/////////////////////////////////////////////////////////////////////////////////////////////////
static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;
         //  Elan modify START
#ifdef NON_MTK_IRQ        
	err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
			IRQF_TRIGGER_LOW, client->name, ts);
#else       
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, //elan_ktf2k_ts_irq_handler, 1);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, elan_ktf2k_eint_interrupt_handler, 1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif		
	ELAN_PRINT("elan_ktf2k_ts_register_interrupt !\n");	

	return err;
}

static int elan_ktf2k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	int retval = 0;
	struct elan_ktf2k_i2c_platform_data *pdata;
	struct elan_ktf2k_ts_data *ts;

	TPD_DMESG("[elan] %s start........\n", __func__);

	hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");      //MT65XX_POWER_LDO_VGP=2.8v	

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0x0); //setup RST pin to always GPIO mode.
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(100);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT); //Setup EINT pin to interrupt mode
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	//mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TPD_DMESG("[elan] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	//printk(KERN_ERR "[elan] %s start.......1.\n", __func__);

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TPD_DMESG("[elan] %s: allocate elan_ktf2k_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}


	//printk(KERN_ERR "[elan] %s start........2\n", __func__);
/*
	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		printk(KERN_ERR "[elan] %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}
*/

	
	//printk(KERN_ERR "[elan] %s start........3\n", __func__);

	//INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);
	elan_i2c_client=client;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

//Elan add START
       // printk(KERN_ERR "[elan] %s: pdata->intr_gpio = %d \n", __func__, (pdata != NULL)?pdata->intr_gpio:0);
       // printk(KERN_ERR "[elan] %s: ts->client->irq = %d \n", __func__, ts->client->irq);       
       // printk(KERN_ERR "[elan] %s: client->timing = %d \n", __func__, client->timing);     
//printk(KERN_ERR "[elan] ts->intr_gpio = %d \n", ts->intr_gpio);  
//Elan add END

	//printk(KERN_ERR "[elan] %s start........4\n", __func__);
	ts->intr_gpio=GPIO_CTP_EINT_PIN;
	if (likely(pdata != NULL)) {
		ts->intr_gpio = pdata->intr_gpio;
	}

	//printk(KERN_ERR "[elan] %s start.......5.\n", __func__);
	client->addr |= I2C_ENEXT_FLAG; //longxuewei add
        client->timing = 400;
	err = elan_ktf2k_ts_setup(client);
	if (err < 0) {
		TPD_DMESG("No Elan chip inside\n");
		err = -ENODEV;
		goto err_detect_failed;
	}

	tpd_load_status = 1;//ocean
	//printk(KERN_ERR "[elan] %s start........6\n", __func__);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		TPD_DMESG("[elan] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "elan-touchscreen";  


	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_X, 0,  TP_EKTF_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0,  TP_EKTF_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,  TP_EKTF_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TP_EKTF_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);	// james Max Finger number is 5

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	
/*
	//Elan add for SOFTKEY START
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	//Elan add for SOFTKEY END
*/

	err = input_register_device(ts->input_dev);
	if (err) {
		TPD_DMESG("[elan]%s: unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	elan_ktf2k_ts_register_interrupt(ts->client);

	if (mt_get_gpio_in(ts->intr_gpio) == 0) {
		TPD_DMESG("[elan]%s: handle missed interrupt\n", __func__);
		//elan_ktf2k_ts_irq_handler(client->irq, ts);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = elan_ktf2k_ts_early_suspend;
	ts->early_suspend.resume = elan_ktf2k_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	private_ts = ts;

	elan_ktf2k_touch_sysfs_init();

	TPD_DMESG("[elan] Start touchscreen %s in interrupt mode\n",
		ts->input_dev->name);
// Firmware Update
  ts->firmware.minor = MISC_DYNAMIC_MINOR;
  ts->firmware.name = "elan-iap";
  ts->firmware.fops = &elan_touch_fops;
  ts->firmware.mode = S_IFREG|S_IRWXUGO; 

  if (misc_register(&ts->firmware) < 0)
  	TPD_DMESG("[ELAN]misc_register failed!!");
  else
    TPD_DMESG("[ELAN]misc_register finished!!");


#ifdef IAP_PORTION
	err = request_firmware(client);
	if(err <0 )
	{
		TPD_DMESG("[elan] ----> request_firmware failed\n");
	}
	else if(err==0)
	{
		TPD_DMESG("[elan] ----> request_firmware ok\n");
	}
	else
	{
		TPD_DMESG("[elan] ----> The FW_VER is the newest,do not need update firmware \n");
	}
#endif


	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}

// End Firmware Update	

	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);

err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}



static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	//unregister_early_suspend(&ts->early_suspend);
	//free_irq(client->irq, ts);

	//if (ts->elan_wq)
	//	destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}
 
//static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int elan_ktf2k_ts_detect(struct i2c_client *client, struct i2c_board_info *info) {
    printk(KERN_ERR "[elan] %s: info->addr = 0x%x\n", __func__, info->addr);

    strcpy(info->type, ELAN_KTF2K_NAME);
    info->platform_data = ts_elan_ktf2k_data;
info->irq = CUST_EINT_TOUCH_PANEL_NUM;
    return 0;
}


static int elan_ktf2k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;

	ELAN_PRINT(" %s: enter\n", __func__);

	//disable_irq(client->irq);

	//rc = cancel_work_sync(&ts->work);
	//if (rc)
	//	enable_irq(client->irq);
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
	ELAN_PRINT(" %s: finish\n", __func__);
	return 0;
}

static int elan_ktf2k_ts_resume(struct i2c_client *client)
{

	int rc = 0, retry = 5;

	ELAN_PRINT("%s: enter\n", __func__);

	do {
		rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_NORMAL);
		rc = elan_ktf2k_ts_get_power_state(client);
		if (rc != PWR_STATE_NORMAL)
			ELAN_PRINT("%s: wake up tp failed! err = %d\n",
				__func__, rc);
		else
			break;
	} while (--retry);

	//enable_irq(client->irq);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts;
	ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);
	ELAN_PRINT(" %s: enter\n", __func__);
	elan_ktf2k_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void elan_ktf2k_ts_late_resume(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts;
	ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);
	ELAN_PRINT(" %s: enter\n", __func__);
	elan_ktf2k_ts_resume(ts->client);
}
#endif


static const struct i2c_device_id elan_ktf2k_ts_id[] = {
	{ ELAN_KTF2K_NAME, 0 },
	{ }
};
//static unsigned short force[] = {0x00, 0x2A, I2C_CLIENT_END, I2C_CLIENT_END};   
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 
static struct i2c_board_info __initdata elan_tpd_i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (0x2A>>1))};//ocean

static struct i2c_driver ektf2k_ts_driver = {
	.probe		= elan_ktf2k_ts_probe,
	.remove		= elan_ktf2k_ts_remove,
	.detect 	= elan_ktf2k_ts_detect, //Elan add
	.id_table	= elan_ktf2k_ts_id,
	.driver		= {
		.name = ELAN_KTF2K_NAME,
	},
//	.address_data = &addr_data,                        
}; 


int tpd_local_init(void){
  TPD_DMESG("Elan EKTF2K I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	if(i2c_add_driver(&ektf2k_ts_driver)!=0)
	{
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	if(tpd_load_status == 0) 
	{
		TPD_DMESG("ft5306 add error touch panel driver.\n");
		i2c_del_driver(&ektf2k_ts_driver);
		return -1;
	}

#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif 	
	tpd_type_cap = 1;
	 TPD_DMESG( "Elan EKTF2K %s OK\n", __func__);	
	return 0;
}

 

static struct tpd_driver_t tpd_device_driver = {
                .tpd_device_name = "elan-tpd",
                .tpd_local_init = tpd_local_init,
               // .suspend = elan_ktf2k_ts_suspend,
              //  .resume = elan_ktf2k_ts_resume,
#ifdef TPD_HAVE_BUTTON
                .tpd_have_button = 1,
#else
                .tpd_have_button = 0,
#endif
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	TPD_DMESG("Elan EKTF2K touch panel driver init\n");
	i2c_register_board_info(0, &elan_tpd_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
	        printk("add Elan EKTF2K driver failed\n");
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
    TPD_DMESG("Elan EKTF2K touch panel driver exit\n");
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("ELAN EKTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");
