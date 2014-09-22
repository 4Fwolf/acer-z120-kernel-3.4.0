/*
 * drivers/leds/leds-mt65xx.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * mt65xx leds driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/leds-mt65xx.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>

#include <cust_leds.h>

#if defined (CONFIG_ARCH_MT6573)
#include <mach/mt6573_pwm.h>
#include <mach/mt6573_gpio.h>
#include <mach/pmu6573_sw.h>

#elif defined (CONFIG_ARCH_MT6516)
#include <mach/mt_pwm.h>
#include <mach/mt_gpio.h>

#elif defined (CONFIG_ARCH_MT6575) || defined (CONFIG_ARCH_MT6575T)
#include <mach/mt_pwm.h>
#include <mach/mt_gpio.h>
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pmic_feature_api.h>
#include <mach/mt_boot.h>

#elif defined (CONFIG_ARCH_MT6577)
#include <mach/mt_pwm.h>
#include <mach/mt_gpio.h>
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pmic_feature_api.h>
#include <mach/mt_boot.h>

#endif

#ifndef CONTROL_BL_TEMPERATURE
#define CONTROL_BL_TEMPERATURE
#endif

#if defined(ACER_Z1) || defined(ACER_Z2) //LK@for backligtht control
volatile unsigned int g_ledCurrPulseCount=1;
#endif
/****************************************************************************
 * LED Variable Settings
 ***************************************************************************/
#define NLED_OFF 0
#define NLED_ON 1
#define NLED_BLINK 2
#define MIN_FRE_OLD_PWM 32 // the min frequence when use old mode pwm by kHz
#define PWM_DIV_NUM 8
#define ERROR_BL_LEVEL 0xFFFFFFFF
struct nled_setting
{
	u8 nled_mode; //0, off; 1, on; 2, blink;
	u32 blink_on_time ;
	u32 blink_off_time;
};
 
struct cust_mt65xx_led* bl_setting = NULL;
unsigned int bl_brightness = 102;
unsigned int bl_duty = 21;
unsigned int bl_div = CLK_DIV1;
unsigned int bl_frequency = 32000;


int time_array[PWM_DIV_NUM]={256,512,1024,2048,4096,8192,16384,32768};
u8 div_array[PWM_DIV_NUM] = {1,2,4,8,16,32,64,128};
unsigned int backlight_PWM_div = CLK_DIV1;// this para come from cust_leds.


/****************************************************************************
 * DEBUG MACROS
 ***************************************************************************/
static int debug_enable = 1;
#define LEDS_DEBUG(format, args...) do{ \
	if(debug_enable) \
	{\
		printk(KERN_EMERG format,##args);\
	}\
}while(0)

/****************************************************************************
 * structures
 ***************************************************************************/
struct mt65xx_led_data {
	struct led_classdev cdev;
	struct cust_mt65xx_led cust;
	struct work_struct work;
	int level;
	int delay_on;
	int delay_off;
};


/****************************************************************************
 * function prototypes
 ***************************************************************************/


extern void mt_pwm_power_off (U32 pwm_no);
extern S32 mt_set_pwm_disable ( U32 pwm_no ) ;
extern unsigned int brightness_mapping(unsigned int level);
extern int mtkfb_get_backlight_pwm(unsigned int divider, unsigned int *freq);


#if defined (CONFIG_ARCH_MT6573)
extern void upmu_kpled_mode(upmu_kpled_list_enum kpled, upmu_kpled_mode_enum val);
extern void upmu_kpled_enable(upmu_kpled_list_enum kpled, kal_bool enable);

#elif defined (CONFIG_ARCH_MT6516)
/* import functions */
extern void pmic_bl_dim_duty(kal_uint8 duty);
extern ssize_t  mt6326_bl_Enable(void);
extern ssize_t  mt6326_bl_Disable(void);
extern ssize_t mt6326_kpled_Enable(void);
extern ssize_t mt6326_kpled_Disable(void);
extern ssize_t mt6326_kpled_dim_duty_Full(void);
extern ssize_t mt6326_kpled_dim_duty_0(void);

#endif


/* export functions */
int mt65xx_leds_brightness_set(enum mt65xx_led_type type, enum led_brightness level);

static int brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div);
static int brightness_set_gpio(int gpio_num, enum led_brightness level);
static int mt65xx_led_set_cust(struct cust_mt65xx_led *cust, int level);
static void mt65xx_led_work(struct work_struct *work);
static void mt65xx_led_set(struct led_classdev *led_cdev, enum led_brightness level);
static int  mt65xx_blink_set(struct led_classdev *led_cdev,
			     unsigned long *delay_on,
			     unsigned long *delay_off);
#ifdef CONTROL_BL_TEMPERATURE
int setMaxbrightness(int max_level, int enable);
//static int control_backlight_brightness(int level);
#endif

/****************************************************************************
 * global variables
 ***************************************************************************/
static struct mt65xx_led_data *g_leds_data[MT65XX_LED_TYPE_TOTAL];
struct wake_lock leds_suspend_lock;

/****************************************************************************
 * add API for temperature control
 ***************************************************************************/

#ifdef CONTROL_BL_TEMPERATURE

//define int limit for brightness limitation
static unsigned  int limit = 255;
static unsigned  int limit_flag = 0;  
static unsigned  int last_level = 0; 
static unsigned  int current_level = 0; 
static DEFINE_MUTEX(bl_level_limit_mutex);


//this API add for control the power and temperature
//if enabe=1, the value of brightness will smaller  than max_level, whatever lightservice transfers to driver
int setMaxbrightness(int max_level, int enable)
{

	struct cust_mt65xx_led *cust_led_list = get_cust_led_list();
	mutex_lock(&bl_level_limit_mutex);
	if (1 == enable)
	{
		limit_flag = 1;
		limit = max_level;
		mutex_unlock(&bl_level_limit_mutex);
		LEDS_DEBUG("[LED] setMaxbrightness limit happen and release lock!!\n");
		printk("setMaxbrightness enable:last_level=%d\n", last_level);
		printk("setMaxbrightness enable:current_level=%d\n", current_level);
		//if (limit < last_level){
		if (0 != current_level){
		printk("mt65xx_leds_set_cust in setMaxbrightness:value control start! limit=%d\n", limit);
		mt65xx_led_set_cust(&cust_led_list[MT65XX_LED_TYPE_LCD], limit);
		}
	}
	else
	{
		limit_flag = 0;
		limit = 255;
		mutex_unlock(&bl_level_limit_mutex);
		LEDS_DEBUG("[LED] setMaxbrightness limit closed and and release lock!!\n");
		printk("setMaxbrightness disable:last_level=%d\n", last_level);
		printk("setMaxbrightness enable:current_level=%d\n", current_level);
		//if (last_level != 0){
		if (0 != current_level){
		printk("control temperature close:limit=%d\n", limit);
		mt65xx_led_set_cust(&cust_led_list[MT65XX_LED_TYPE_LCD], last_level);
		
		//printk("mt65xx_leds_set_cust in setMaxbrightness:value control close!\n");
		}
	}
 	
	LEDS_DEBUG("[LED] setMaxbrightness limit_flag = %d, limit=%d, current_level=%d\n",limit_flag, limit, current_level);
	
	return 0;
	
}
/*static int control_backlight_brightness(int level)
{
	
	if (limit_flag == 1){
			if (level>limit){
					level = limit;
			printk("mt65xx_leds_set_cust:value control! level=%d,limit=%d\n", level, limit);
			}
			
		}
	else{
			last_level = level;
			printk("control_backlight_brightness:last_level=%d\n", last_level);
	}/
	printk("mt65xx_leds_set_cust:control_backlight_brightness level=%d,limit=%d\n", level, limit);

	return level;
}
*/
#endif

/****************************************************************************
 * internal functions
 ***************************************************************************/
static int brightness_mapto64(int level)
{
        if (level < 30)
                return (level >> 1) + 7;
        else if (level <= 120)
                return (level >> 2) + 14;
        else if (level <= 160)
                return level / 5 + 20;
        else
                return (level >> 3) + 33;
}

int find_time_index(int time)
{	
	int index = 0;	
	while(index < 8)	
	{		
		if(time<time_array[index])			
			return index;		
		else
			index++;
	}	
	return PWM_DIV_NUM-1;
}


static int led_set_pwm(int pwm_num, struct nled_setting* led)
{
	struct pwm_easy_config pwm_setting;
	int time_index = 0;
	pwm_setting.pwm_no = pwm_num;
    
        LEDS_DEBUG("[LED]led_set_pwm: mode=%d,pwm_no=%d\n", led->nled_mode, pwm_num);  
	if((pwm_num != PWM3 && pwm_num != PWM4 && pwm_num != PWM5))
		pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;
	else
		pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK_DIV_BY_1625;
    
	switch (led->nled_mode)
	{
		case NLED_OFF :
			pwm_setting.duty = 0;
			pwm_setting.clk_div = CLK_DIV1;
			pwm_setting.duration = 100;
			break;
            
		case NLED_ON :
			pwm_setting.duty = 30;
			pwm_setting.clk_div = CLK_DIV1;			
			pwm_setting.duration = 100;
			break;
            
		case NLED_BLINK :
			LEDS_DEBUG("[LED]LED blink on time = %d offtime = %d\n",led->blink_on_time,led->blink_off_time);
			time_index = find_time_index(led->blink_on_time + led->blink_off_time);
			LEDS_DEBUG("[LED]LED div is %d\n",time_index);
			pwm_setting.clk_div = time_index;
			pwm_setting.duration = (led->blink_on_time + led->blink_off_time) * MIN_FRE_OLD_PWM / div_array[time_index];
			pwm_setting.duty = (led->blink_on_time*100) / (led->blink_on_time + led->blink_off_time);
	}
	pwm_set_easy_config(&pwm_setting);

#if defined (CONFIG_ARCH_MT6573)
	if(pwm_num == PWM7)
	{
		if(led->nled_mode == NLED_OFF)
		{
			LEDS_DEBUG("[LED]Disable MT6573 keypad led\n");
			upmu_kpled_enable(KPLED,0);
		}else
		{
			LEDS_DEBUG("[LED]Enable MT6573 keypad led\n");
			upmu_kpled_mode(KPLED,0);
			upmu_kpled_enable(KPLED,1);
		}		
	}
#endif	

	return 0;
	
}

#if defined (CONFIG_ARCH_MT6575) || defined (CONFIG_ARCH_MT6575T) || defined (CONFIG_ARCH_MT6577)
#define PMIC_PERIOD_NUM 15
int pmic_period_array[] = {250,500,1000,1250,1666,2000,2500,3333,4000,5000,6666,8000,10000,13333,16000};
u8 pmic_clksel_array[] = {0,0,0,0,0,0,1,1,1,2,2,2,3,3,3};
u8 pmic_freqsel_array[] = {24,26,28,29,30,31,29,30,31,29,30,31,29,30,31};

int find_time_index_pmic(int time_ms) {
	int i;
	for(i=0;i<PMIC_PERIOD_NUM;i++) {
		if(time_ms<=pmic_period_array[i]) {
			return i;
		} else {
			continue;
		}
	}
	return PMIC_PERIOD_NUM-1;
}

static int led_blink_pmic(enum mt65xx_led_pmic pmic_type, struct nled_setting* led) {
	int time_index = 0;
	int duty = 0;
	LEDS_DEBUG("[LED]led_blink_pmic: pmic_type=%d\n", pmic_type);  
	
	if((pmic_type != MT65XX_LED_PMIC_NLED_ISINK4 && pmic_type!= MT65XX_LED_PMIC_NLED_ISINK5) || led->nled_mode != NLED_BLINK) {
		return -1;
	}
				
	LEDS_DEBUG("[LED]LED blink on time = %d offtime = %d\n",led->blink_on_time,led->blink_off_time);
	time_index = find_time_index_pmic(led->blink_on_time + led->blink_off_time);
	LEDS_DEBUG("[LED]LED index is %d clksel=%d freqsel=%d\n", time_index, pmic_clksel_array[time_index], pmic_freqsel_array[time_index]);
	duty=32*led->blink_on_time/(led->blink_on_time + led->blink_off_time);
	switch(pmic_type){
		case MT65XX_LED_PMIC_NLED_ISINK4:
			upmu_isinks_ch4_mode(PMIC_PWM_1);
			upmu_isinks_ch4_step(0x3);
			upmu_isinks_ch4_cabc_en(0);
			upmu_isinks_dim1_duty(duty);
			upmu_isinks_dim1_fsel(pmic_freqsel_array[time_index]);
			pmic_bank1_config_interface(0x2e, pmic_clksel_array[time_index], 0x3, 0x6);
			upmu_top2_bst_drv_ck_pdn(0x0);
			hwBacklightISINKTurnOn(MT65XX_LED_PMIC_NLED_ISINK4);
		break;
		case MT65XX_LED_PMIC_NLED_ISINK5:
			upmu_isinks_ch5_mode(PMIC_PWM_2);
			upmu_isinks_ch5_step(0x3);
			upmu_isinks_ch5_cabc_en(0);
			upmu_isinks_dim2_duty(duty);
			upmu_isinks_dim2_fsel(pmic_freqsel_array[time_index]);
			pmic_bank1_config_interface(0x30, pmic_clksel_array[time_index], 0x3, 0x6);
			upmu_top2_bst_drv_ck_pdn(0x0);
			hwBacklightISINKTurnOn(MT65XX_LED_PMIC_NLED_ISINK5);
		break;
		default:
		break;
	}
			
	return 0;
}
#endif

static int backlight_set_pwm(int pwm_num, u32 level, u32 div, struct PWM_config *config_data)
{
	struct pwm_spec_config pwm_setting;
	pwm_setting.pwm_no = pwm_num;
	pwm_setting.mode = PWM_MODE_FIFO; //new mode fifo and periodical mode
		
	#ifdef CONTROL_BL_TEMPERATURE
	mutex_lock(&bl_level_limit_mutex);
			current_level = level;
			printk("brightness_set_pwm:current_level=%d\n", current_level);
			if(0 == limit_flag){
				last_level = level;
				printk("brightness_set_pwm:last_level=%d\n", last_level);
			}else {
					if(limit < current_level){
						level = limit;
						printk("backlight_set_pwm: control level=%d\n", level);
					}
			}	
	mutex_unlock(&bl_level_limit_mutex);
	#endif
	if(config_data->div)
	{
		pwm_setting.clk_div = config_data->div;
		backlight_PWM_div = config_data->div;
	}
	else
		pwm_setting.clk_div = div;
	if(config_data->clock_source)
		pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	else
		pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK_DIV_BY_1625;
	
	if(config_data->High_duration && config_data->low_duration)
		{
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = config_data->High_duration;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = config_data->low_duration;
		}
	else
		{
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 4;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 4;
		}
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
		
	LEDS_DEBUG("[LED]backlight_set_pwm:duty is %d\n", level);
	LEDS_DEBUG("[LED]backlight_set_pwm:clk_src/div/high/low is %d%d%d%d\n", pwm_setting.clk_src,pwm_setting.clk_div,pwm_setting.PWM_MODE_FIFO_REGS.HDURATION,pwm_setting.PWM_MODE_FIFO_REGS.LDURATION);
	if(level>0 && level <= 32)
	{
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 =  (1 << level) - 1 ;
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0 ;
		pwm_set_spec_config(&pwm_setting);
	}else if(level>32 && level <=64)
	{
		level -= 32;
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 =  0xFFFFFFFF ;
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = (1 << level) - 1;
		pwm_set_spec_config(&pwm_setting);
	}else
	{
		LEDS_DEBUG("[LED]Error level in backlight\n");
		mt_set_pwm_disable(pwm_setting.pwm_no);
		mt_pwm_power_off(pwm_setting.pwm_no);
	}
		//printk("[LED]PWM con register is %x \n", INREG32(PWM_BASE + 0x0150));
	return 0;
}



#if defined (CONFIG_ARCH_MT6516)
/* import functions */
static int brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div)
{
	int tmp_level = level;
	LEDS_DEBUG("[LED]PMIC#%d:%d\n", pmic_type, level);

	if (pmic_type == MT65XX_LED_PMIC_LCD) {
		if (level) 
		{
			level = brightness_mapping(tmp_level);
			if(level == ERROR_BL_LEVEL)
				level = tmp_level<8?1:tmp_level/8;
			mt6326_bl_Enable();
			pmic_bl_dim_duty(level);
		}
		else {
			mt6326_bl_Disable();
		}
		return 0;
	}
	else if (pmic_type == MT65XX_LED_PMIC_BUTTON) {
		if (level) {
			mt6326_kpled_dim_duty_Full();
			mt6326_kpled_Enable();
		}
		else {
			mt6326_kpled_dim_duty_0();
			mt6326_kpled_Disable();
		}
		return 0;
	}

	return -1;
}


#elif  defined (CONFIG_ARCH_MT6573)
static int brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div)
{
	return 0;
}


#elif  defined (CONFIG_ARCH_MT6575) || defined (CONFIG_ARCH_MT6575T)|| defined (CONFIG_ARCH_MT6577)
static int brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div)
{
	int tmp_level = level;
	static bool backlight_init_flag[2] = {false, false};
	static bool led_init_flag[2] = {false, false};
	static bool first_time = true;
	
	LEDS_DEBUG("[LED]PMIC#%d:%d\n", pmic_type, level);

	if (pmic_type == MT65XX_LED_PMIC_LCD_ISINK)
	{
		if(backlight_init_flag[0] == false)
		{
			hwBacklightISINKTuning(1, PMIC_PWM_0, 0x3, 0);
			hwBacklightISINKTuning(2, PMIC_PWM_0, 0x3, 0);
			hwBacklightISINKTuning(3, PMIC_PWM_0, 0x3, 0);
			backlight_init_flag[0] = true;
		}
		
		if (level) 
		{
			level = brightness_mapping(tmp_level);
			if(level == ERROR_BL_LEVEL)
				level = tmp_level/17;
			hwPWMsetting(PMIC_PWM_0, level, div);
			upmu_top2_bst_drv_ck_pdn(0x0);
			hwBacklightISINKTurnOn(1);
			hwBacklightISINKTurnOn(2);
			hwBacklightISINKTurnOn(3);
			bl_duty = level;	
		}
		else 
		{
			hwBacklightISINKTurnOff(1);
			hwBacklightISINKTurnOff(2);
			hwBacklightISINKTurnOff(3);
			bl_duty = level;	
		}
		return 0;
	}
	else if(pmic_type == MT65XX_LED_PMIC_LCD_BOOST)
	{
		/*
		if(backlight_init_flag[1] == false)
		{
			hwBacklightBoostTuning(PMIC_PWM_0, 0xC, 0);
			backlight_init_flag[1] = true;
		}
		*/		
		if (level) 
		{
			#ifdef CONTROL_BL_TEMPERATURE
			mutex_lock(&bl_level_limit_mutex);
			current_level = level;
			printk("brightness_set_pmic:current_level=%d\n", current_level);
			if(0 == limit_flag){
				last_level = level;
				printk("brightness_set_pmic:last_level=%d\n", last_level);
			}else {
					if(limit < current_level){
						level = limit;
						printk("backlight_set_pmic: control level=%d\n", level);
					}
			}	
			mutex_unlock(&bl_level_limit_mutex);
			#endif
			
			level = brightness_mapping(tmp_level);
			if(level == ERROR_BL_LEVEL)
				level = tmp_level/42;
	
			upmu_boost_isink_hw_sel(0x1);
			upmu_boost_mode(3);
			upmu_boost_cabc_en(0);

			switch(level)
			{
				case 0:				
					upmu_boost_vrsel(0x0);
					//hwPWMsetting(PMIC_PWM_0, 0, 0x15);
					break;
				case 1:
					upmu_boost_vrsel(0x1);
					//hwPWMsetting(PMIC_PWM_0, 4, 0x15);
					break;
				case 2:
					upmu_boost_vrsel(0x2);
					//hwPWMsetting(PMIC_PWM_0, 5, 0x15);
					break;
				case 3:
					upmu_boost_vrsel(0x3);
					//hwPWMsetting(PMIC_PWM_0, 6, 0x15);
					break;
				case 4:
					upmu_boost_vrsel(0x5);
					//hwPWMsetting(PMIC_PWM_0, 7, 0x15);
					break;
				case 5:
					upmu_boost_vrsel(0x8);
					//hwPWMsetting(PMIC_PWM_0, 8, 0x15);
					break;
				case 6:
					upmu_boost_vrsel(0xB);
					//hwPWMsetting(PMIC_PWM_0, 9, 0x15);
					break;
				default:
					printk("[LED] invalid brightness level %d->%d\n", tmp_level, level);
					break;
			}
			
			upmu_top2_bst_drv_ck_pdn(0x0);
			upmu_boost_en(0x1);
			bl_duty = level;	
		}
		else 
		{
			upmu_boost_en(0x0);
			bl_duty = level;	
			//upmu_top2_bst_drv_ck_pdn(0x1);
		}
		return 0;
	}
	else if (pmic_type == MT65XX_LED_PMIC_BUTTON) 
	{
		if (level) 
		{
	#if defined(ACER_Z2)
			hwBacklightKPLEDTuning(0x1F, 0x0);//0x09->0x1F,31,100%
	#else
			hwBacklightKPLEDTuning(0x09, 0x0);
	#endif			
			hwBacklightKPLEDTurnOn();
		}
		else 
		{
			hwBacklightKPLEDTurnOff();
		}
		return 0;
	}
	else if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK4)
	{
		if(first_time == true)
		{
			hwBacklightISINKTurnOff(MT65XX_LED_PMIC_NLED_ISINK5);  //sw workround for sync leds status 
			first_time = false;
		}

                hwBacklightISINKTuning(0x0, 0x3, 0x0, 0);  //register mode, ch0_step=4ma ,disable CABC
                hwBacklightISINKTurnOn(0x0);  //turn on ISINK0

		//if(led_init_flag[0] == false)
		{
			hwBacklightISINKTuning(MT65XX_LED_PMIC_NLED_ISINK4, PMIC_PWM_1, 0x0, 0);
			hwPWMsetting(PMIC_PWM_1, 15, 8);
			led_init_flag[0] = true;
		}
		
		if (level) 
		{
			upmu_top2_bst_drv_ck_pdn(0x0);
			hwBacklightISINKTurnOn(MT65XX_LED_PMIC_NLED_ISINK4);
		}
		else 
		{
			hwBacklightISINKTurnOff(MT65XX_LED_PMIC_NLED_ISINK4);
			//upmu_top2_bst_drv_ck_pdn(0x1);
		}
		return 0;
	}
	else if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK5)
	{
		if(first_time == true)
		{
			hwBacklightISINKTurnOff(MT65XX_LED_PMIC_NLED_ISINK4);  //sw workround for sync leds status
			first_time = false;
		}

                hwBacklightISINKTuning(0x0, 0x3, 0x0, 0);  //register mode, ch0_step=4ma ,disable CABC
                hwBacklightISINKTurnOn(0x0);  //turn on ISINK0

		//if(led_init_flag[1] == false)
		{
			hwBacklightISINKTuning(MT65XX_LED_PMIC_NLED_ISINK5, PMIC_PWM_2, 0x0, 0);
			hwPWMsetting(PMIC_PWM_2, 15, 8);
			led_init_flag[1] = true;
		}
		if (level) 
		{
			upmu_top2_bst_drv_ck_pdn(0x0);
			hwBacklightISINKTurnOn(MT65XX_LED_PMIC_NLED_ISINK5);
		}
		else 
		{
			hwBacklightISINKTurnOff(MT65XX_LED_PMIC_NLED_ISINK5);
			//upmu_top2_bst_drv_ck_pdn(0x1);
		}
		return 0;
	}
	
	return -1;
}

#endif


static int brightness_set_gpio(int gpio_num, enum led_brightness level)
{
#if defined(ACER_Z1) || defined(ACER_Z2)//LK@for backligtht control
        int i; 
        int min_level = 30;        //the min value for UI brightness setup 
        int max_level = 255;        //the max value for UI brightness setup 
        int min_bri = 1;        //the min value for adjust the brightness 
        int max_bri = 16;        //the max value for adjust the brightness 
        unsigned int count=0, to_count=0; 
        static int s_bBacklightOn = 1; 
          
        mt_set_gpio_mode(gpio_num, GPIO_MODE_GPIO); 
        mt_set_gpio_dir(gpio_num, GPIO_DIR_OUT); 

        if (level) { 
                if (level < min_level) { 
                        to_count = max_bri; 
                }else if (level > 255) { 
                        to_count = min_bri; 
                }else { 
                        to_count = max_bri-((level-min_level)*(max_bri-min_bri)/(max_level-min_level)); 
                        //LEDS_DEBUG("[LED] infact count=%d\n", to_count); 
                } 
                if (s_bBacklightOn) { 
                        if (g_ledCurrPulseCount > to_count) {   //change brighter 
                                count = to_count+16-g_ledCurrPulseCount; 
                        }else if (g_ledCurrPulseCount < to_count){        //change darker 
                                count = to_count-g_ledCurrPulseCount; 
                        }else { 
                                goto end; 
                        } 
                }else { 
                        count = to_count; 
                } 
                
                //LEDS_DEBUG("[LED] need more count=%d\n", count); 
                if (s_bBacklightOn) { 
                    #if 0
                        mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);   
                        mt65xx_eint_mask(CUST_EINT_ALS_NUM);        
                        mt65xx_eint_mask(CUST_EINT_GSE_1_NUM);       
                        mt65xx_eint_mask(CUST_EINT_GSE_2_NUM);       
                        mt65xx_eint_mask(CUST_EINT_WIFI_NUM);       
                   #endif
                        for (i=0; i<count; i++) { 
                                mt_set_gpio_out(gpio_num, GPIO_OUT_ZERO); 
                                udelay(2); 
                                mt_set_gpio_out(gpio_num, GPIO_OUT_ONE); 
                                udelay(2);         
                        } 
                    #if 0    
                        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
                        mt65xx_eint_unmask(CUST_EINT_ALS_NUM); 
                        mt65xx_eint_unmask(CUST_EINT_GSE_1_NUM); 
                        mt65xx_eint_unmask(CUST_EINT_GSE_2_NUM); 
                        mt65xx_eint_unmask(CUST_EINT_WIFI_NUM); 
                   #endif
                }else {        //first pulse must set high more than 20 us 
                        mt_set_gpio_out(gpio_num, GPIO_OUT_ONE); 
                        udelay(20); 
                    #if 0
                        mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
                        mt65xx_eint_mask(CUST_EINT_ALS_NUM); 
                        mt65xx_eint_mask(CUST_EINT_GSE_1_NUM); 
                        mt65xx_eint_mask(CUST_EINT_GSE_2_NUM); 
                        mt65xx_eint_mask(CUST_EINT_WIFI_NUM); 
                   #endif
                        for (i=1; i<count; i++) { 
                                mt_set_gpio_out(gpio_num, GPIO_OUT_ZERO); 
                                udelay(2); 
                                mt_set_gpio_out(gpio_num, GPIO_OUT_ONE); 
                                udelay(2);                             
                        } 
                   #if 0
                        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
                        mt65xx_eint_unmask(CUST_EINT_ALS_NUM); 
                        mt65xx_eint_unmask(CUST_EINT_GSE_1_NUM); 
                        mt65xx_eint_unmask(CUST_EINT_GSE_2_NUM); 
                        mt65xx_eint_unmask(CUST_EINT_WIFI_NUM); 
                   #endif
                } 
                
                g_ledCurrPulseCount = to_count; 
                //LEDS_DEBUG("[LED]GPIO111#%d:%d:curr count=%d, more count=%d, s_bBacklightOn=%d\n", gpio_num, level, g_ledCurrPulseCount, count, s_bBacklightOn); 
                mdelay(3); 
                s_bBacklightOn = 1; 
        }else { 
                mt_set_gpio_out(gpio_num, GPIO_OUT_ZERO); 
                s_bBacklightOn = 0; 
                mdelay(3); 
        } 
end: 
        //LEDS_DEBUG("[LED]GPIO222#%d:%d:curr count=%d, to count=%d, s_bBacklightOn=%d\n", gpio_num, level, g_ledCurrPulseCount, count, s_bBacklightOn); 

#else

	LEDS_DEBUG("[LED]GPIO#%d:%d\n", gpio_num, level);
	mt_set_gpio_mode(gpio_num, GPIO_MODE_GPIO);
	mt_set_gpio_dir(gpio_num, GPIO_DIR_OUT);

	if (level)
		mt_set_gpio_out(gpio_num, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(gpio_num, GPIO_OUT_ZERO);
#endif
	return 0;
}

static int mt65xx_led_set_cust(struct cust_mt65xx_led *cust, int level)
{
	struct nled_setting led_tmp_setting = {0,0,0};
	int tmp_level;

/*#ifdef CONTROL_BL_TEMPERATURE
	level = control_backlight_brightness(level);
	
#endif*/
	tmp_level = level;
	if (level > LED_FULL)
		level = LED_FULL;
	else if (level < 0)
		level = 0;

    printk("mt65xx_leds_set_cust: set brightness, name:%s, mode:%d, level:%d\n", 
		cust->name, cust->mode, level);
	switch (cust->mode) {
		case MT65XX_LED_MODE_PWM:
			if(strcmp(cust->name,"lcd-backlight") == 0)
			{
				bl_brightness = level;
				if(level == 0)
				{
					mt_set_pwm_disable(cust->data);
					mt_pwm_power_off (cust->data);
				}else
				{
					level = brightness_mapping(tmp_level);
					if(level == ERROR_BL_LEVEL)
						level = brightness_mapto64(tmp_level);
						
					backlight_set_pwm(cust->data, level, bl_div,&cust->config_data);
				}
                bl_duty = level;	
				
			}else
			{
				if(level == 0)
				{
					led_tmp_setting.nled_mode = NLED_OFF;
				}else
				{
					led_tmp_setting.nled_mode = NLED_ON;
				}
				led_set_pwm(cust->data,&led_tmp_setting);
			}
			return 1;
            
		case MT65XX_LED_MODE_GPIO:
			return brightness_set_gpio(cust->data, level);
            
		case MT65XX_LED_MODE_PMIC:
			return brightness_set_pmic(cust->data, level, bl_div);
            
		case MT65XX_LED_MODE_CUST:
            if(strcmp(cust->name,"lcd-backlight") == 0)
			{
			    bl_brightness = level;
            }
			#ifdef CONTROL_BL_TEMPERATURE
			mutex_lock(&bl_level_limit_mutex);
			current_level = level;
			printk("brightness_set_cust:current_level=%d\n", current_level);
			if(0 == limit_flag){
				last_level = level;
				printk("brightness_set_cust:last_level=%d\n", last_level);
			}else {
					if(limit < current_level){
						level = limit;
						printk("backlight_set_cust: control level=%d\n", level);
					}
			}	
			mutex_unlock(&bl_level_limit_mutex);
			#endif
			return ((cust_brightness_set)(cust->data))(level, bl_div);
            
		case MT65XX_LED_MODE_NONE:
		default:
			break;
	}
	return -1;
}

static void mt65xx_led_work(struct work_struct *work)
{
	struct mt65xx_led_data *led_data =
		container_of(work, struct mt65xx_led_data, work);

	LEDS_DEBUG("[LED]%s:%d\n", led_data->cust.name, led_data->level);

	mt65xx_led_set_cust(&led_data->cust, led_data->level);
}

static void mt65xx_led_set(struct led_classdev *led_cdev, enum led_brightness level)
{
	struct mt65xx_led_data *led_data =
		container_of(led_cdev, struct mt65xx_led_data, cdev);

	// do something only when level is changed
	if (led_data->level != level) {
		led_data->level = level;
		if(strcmp(led_data->cust.name,"lcd-backlight"))
		{
				schedule_work(&led_data->work);
		}else
		{
				LEDS_DEBUG("[LED]Set Backlight directly %d at time %lu\n",led_data->level,jiffies);
				mt65xx_led_set_cust(&led_data->cust, led_data->level);	
		}
	}
}

static int  mt65xx_blink_set(struct led_classdev *led_cdev,
			     unsigned long *delay_on,
			     unsigned long *delay_off)
{
	struct mt65xx_led_data *led_data =
		container_of(led_cdev, struct mt65xx_led_data, cdev);
	static int got_wake_lock = 0;
	struct nled_setting nled_tmp_setting = {0,0,0};

	// only allow software blink when delay_on or delay_off changed
	if (*delay_on != led_data->delay_on || *delay_off != led_data->delay_off) {
		led_data->delay_on = *delay_on;
		led_data->delay_off = *delay_off;
		if (led_data->delay_on && led_data->delay_off) { // enable blink
			led_data->level = 255; // when enable blink  then to set the level  (255)
			if(led_data->cust.mode == MT65XX_LED_MODE_PWM && 
			(led_data->cust.data != PWM3 && led_data->cust.data != PWM4 && led_data->cust.data != PWM5))
			{
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time = led_data->delay_off;
				nled_tmp_setting.blink_on_time = led_data->delay_on;
				led_set_pwm(led_data->cust.data,&nled_tmp_setting);
				return 0;
			}
			
			#if defined (CONFIG_ARCH_MT6575) || defined (CONFIG_ARCH_MT6575T)|| defined (CONFIG_ARCH_MT6577)
			else if((led_data->cust.mode == MT65XX_LED_MODE_PMIC) && (led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK4
				|| led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK5))
			{
				if(get_chip_eco_ver() == CHIP_E2) {
					nled_tmp_setting.nled_mode = NLED_BLINK;
					nled_tmp_setting.blink_off_time = led_data->delay_off;
					nled_tmp_setting.blink_on_time = led_data->delay_on;
					led_blink_pmic(led_data->cust.data, &nled_tmp_setting);
					return 0;
				} else {
					wake_lock(&leds_suspend_lock);
				}
			}
			#endif		
			
			else if (!got_wake_lock) {
				wake_lock(&leds_suspend_lock);
				got_wake_lock = 1;
			}
		}
		else if (!led_data->delay_on && !led_data->delay_off) { // disable blink
			if(led_data->cust.mode == MT65XX_LED_MODE_PWM && 
			(led_data->cust.data != PWM3 && led_data->cust.data != PWM4 && led_data->cust.data != PWM5))
			{
				nled_tmp_setting.nled_mode = NLED_OFF;
				led_set_pwm(led_data->cust.data,&nled_tmp_setting);
				return 0;
			}
			
			#if defined (CONFIG_ARCH_MT6575) || defined (CONFIG_ARCH_MT6575T)|| defined (CONFIG_ARCH_MT6577)
			else if((led_data->cust.mode == MT65XX_LED_MODE_PMIC) && (led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK4
				|| led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK5))
			{
				if(get_chip_eco_ver() == CHIP_E2) {
					brightness_set_pmic(led_data->cust.data, 0, 0);
					return 0;
				} else {
					wake_unlock(&leds_suspend_lock);
				}
			}
			#endif	
			
			else if (got_wake_lock) {
				wake_unlock(&leds_suspend_lock);
				got_wake_lock = 0;
			}
		}
		return -1;
	}

	// delay_on and delay_off are not changed
	return 0;
}

/****************************************************************************
 * external functions
 ***************************************************************************/
int mt65xx_leds_brightness_set(enum mt65xx_led_type type, enum led_brightness level)
{
	struct cust_mt65xx_led *cust_led_list = get_cust_led_list();

	LEDS_DEBUG("[LED]#%d:%d\n", type, level);

	if (type < 0 || type >= MT65XX_LED_TYPE_TOTAL)
		return -1;

	if (level > LED_FULL)
		level = LED_FULL;
	else if (level < 0)
		level = 0;

	return mt65xx_led_set_cust(&cust_led_list[type], level);

}

EXPORT_SYMBOL(mt65xx_leds_brightness_set);


static ssize_t show_duty(struct device *dev,struct device_attribute *attr, char *buf)
{
	LEDS_DEBUG("[LED]get backlight duty value is:%d \n",bl_duty);
	return sprintf(buf, "%u\n", bl_duty);
}
static ssize_t store_duty(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	unsigned int level = 0;
	size_t count = 0;
	LEDS_DEBUG("set backlight duty start \n");
	level = simple_strtoul(buf,&pvalue,10);
	count = pvalue - buf;
	if (*pvalue && isspace(*pvalue))
		count++;
    
	if(count == size)
	{
		if(bl_setting->mode == MT65XX_LED_MODE_PMIC)
		{
#if defined (CONFIG_ARCH_MT6573)	

#elif defined (CONFIG_ARCH_MT6516)
			if (level) //duty:0-31
			{
				mt6326_bl_Enable();
				pmic_bl_dim_duty(level);
				
			}else 
			{
				mt6326_bl_Disable();
			}
			
#elif defined (CONFIG_ARCH_MT6575) || defined (CONFIG_ARCH_MT6575T)|| defined (CONFIG_ARCH_MT6577)
			//duty:0-16
			if((level >= 0) && (level <= 15))
			{
				brightness_set_pmic(MT65XX_LED_PMIC_LCD_BOOST, (level*17), bl_div);
			}
			else
			{
				LEDS_DEBUG("duty value is error, please select vaule from [0-15]!\n");
			}
#endif			
		}
		else if(bl_setting->mode == MT65XX_LED_MODE_PWM)
		{
			if(level == 0)
			{
				mt_set_pwm_disable(bl_setting->data);
				mt_pwm_power_off (bl_setting->data);
			}else if(level <= 64)
			{
				backlight_set_pwm(bl_setting->data,level, bl_div,&bl_setting->config_data);
			}	
		}
        
		bl_duty = level;

	}
	
	return size;
}


static DEVICE_ATTR(duty, 0664, show_duty, store_duty);


static ssize_t show_div(struct device *dev,struct device_attribute *attr, char *buf)
{
	LEDS_DEBUG("get backlight div value is:%d \n",bl_div);
	return sprintf(buf, "%u\n", bl_div);
}

static ssize_t store_div(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	unsigned int div = 0;
	size_t count = 0;
	LEDS_DEBUG("set backlight div start \n");
	div = simple_strtoul(buf,&pvalue,10);
	count = pvalue - buf;
	
	if (*pvalue && isspace(*pvalue))
		count++;
		
	if(count == size)
	{
                if(div < 0 || (div > 7))
		{
            LEDS_DEBUG("set backlight div parameter error: %d[div:0~7]\n", div);
            return 0;
		}
        
		if(bl_setting->mode == MT65XX_LED_MODE_PWM)
		{
            LEDS_DEBUG("set PWM backlight div OK: div=%d, duty=%d\n", div, bl_duty);
            backlight_set_pwm(bl_setting->data, bl_duty, div,&bl_setting->config_data);
		}
        else if(bl_setting->mode == MT65XX_LED_MODE_CUST)
        {
            LEDS_DEBUG("set cust backlight div OK: div=%d, brightness=%d\n", div, bl_brightness);
	        ((cust_brightness_set)(bl_setting->data))(bl_brightness, div);
        }
        
		bl_div = div;
	}
	
	return size;
}

static DEVICE_ATTR(div, 0664, show_div, store_div);


static ssize_t show_frequency(struct device *dev,struct device_attribute *attr, char *buf)
{
    if(bl_setting->mode == MT65XX_LED_MODE_PWM)
    {
        bl_frequency = 32000/div_array[bl_div];
    }
    else if(bl_setting->mode == MT65XX_LED_MODE_CUST)
    {
        //mtkfb_get_backlight_pwm(bl_div, &bl_frequency);  		
    }

    LEDS_DEBUG("[LED]get backlight PWM frequency value is:%d \n", bl_frequency);
    
	return sprintf(buf, "%u\n", bl_frequency);
}

static DEVICE_ATTR(frequency, 0664, show_frequency, NULL);



static ssize_t store_pwm_register(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;
	if(buf != NULL && size != 0)
	{
		LEDS_DEBUG("store_pwm_register: size:%d,address:0x%s\n", size,buf);
		reg_address = simple_strtoul(buf,&pvalue,16);
	
		if(*pvalue && (*pvalue == '#'))
		{
			reg_value = simple_strtoul((pvalue+1),NULL,16);
			LEDS_DEBUG("set pwm register:[0x%x]= 0x%x\n",reg_address,reg_value);
			OUTREG32(reg_address,reg_value);
			
		}else if(*pvalue && (*pvalue == '@'))
		{
			LEDS_DEBUG("get pwm register:[0x%x]=0x%x\n",reg_address,INREG32(reg_address));
		}	
	}

	return size;
}

static ssize_t show_pwm_register(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(pwm_register, 0664, show_pwm_register, store_pwm_register);


/****************************************************************************
 * driver functions
 ***************************************************************************/
static int __init mt65xx_leds_probe(struct platform_device *pdev)
{
	int i;
	int ret, rc;
	struct cust_mt65xx_led *cust_led_list = get_cust_led_list();
	LEDS_DEBUG("[LED]%s\n", __func__);

	for (i = 0; i < MT65XX_LED_TYPE_TOTAL; i++) {
		if (cust_led_list[i].mode == MT65XX_LED_MODE_NONE) {
			g_leds_data[i] = NULL;
			continue;
		}

		g_leds_data[i] = kzalloc(sizeof(struct mt65xx_led_data), GFP_KERNEL);
		if (!g_leds_data[i]) {
			ret = -ENOMEM;
			goto err;
		}

		g_leds_data[i]->cust.mode = cust_led_list[i].mode;
		g_leds_data[i]->cust.data = cust_led_list[i].data;
		g_leds_data[i]->cust.name = cust_led_list[i].name;

		g_leds_data[i]->cdev.name = cust_led_list[i].name;
		g_leds_data[i]->cust.config_data = cust_led_list[i].config_data;//bei add

		g_leds_data[i]->cdev.brightness_set = mt65xx_led_set;
		g_leds_data[i]->cdev.blink_set = mt65xx_blink_set;

		INIT_WORK(&g_leds_data[i]->work, mt65xx_led_work);

		ret = led_classdev_register(&pdev->dev, &g_leds_data[i]->cdev);
        
		if(strcmp(g_leds_data[i]->cdev.name,"lcd-backlight") == 0)
		{
			rc = device_create_file(g_leds_data[i]->cdev.dev, &dev_attr_duty);
            if(rc)
            {
                LEDS_DEBUG("[LED]device_create_file duty fail!\n");
            }
            
            rc = device_create_file(g_leds_data[i]->cdev.dev, &dev_attr_div);
            if(rc)
            {
                LEDS_DEBUG("[LED]device_create_file duty fail!\n");
            }
            
            rc = device_create_file(g_leds_data[i]->cdev.dev, &dev_attr_frequency);
            if(rc)
            {
                LEDS_DEBUG("[LED]device_create_file duty fail!\n");
            }
            
	    rc = device_create_file(g_leds_data[i]->cdev.dev, &dev_attr_pwm_register);
            if(rc)
            {
                LEDS_DEBUG("[LED]device_create_file duty fail!\n");
            }
			bl_setting = &g_leds_data[i]->cust;
		}

		if (ret)
			goto err;
		
	}

#ifdef CONTROL_BL_TEMPERATURE

	last_level = 0;  
	limit = 255;
	limit_flag = 0; 
	current_level = 0;
	printk("[LED]led probe last_level = %d, limit = %d, limit_flag = %d, current_level = %d\n",last_level,limit,limit_flag,current_level);
#endif

	return 0;

err:
	if (i) {
		for (i = i-1; i >=0; i--) {
			if (!g_leds_data[i])
				continue;
			led_classdev_unregister(&g_leds_data[i]->cdev);
			cancel_work_sync(&g_leds_data[i]->work);
			kfree(g_leds_data[i]);
			g_leds_data[i] = NULL;
		}
	}

	return ret;
}

static int mt65xx_leds_remove(struct platform_device *pdev)
{
	int i;
	for (i = 0; i < MT65XX_LED_TYPE_TOTAL; i++) {
		if (!g_leds_data[i])
			continue;
		led_classdev_unregister(&g_leds_data[i]->cdev);
		cancel_work_sync(&g_leds_data[i]->work);
		kfree(g_leds_data[i]);
		g_leds_data[i] = NULL;
	}

	return 0;
}

/*
static int mt65xx_leds_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}
*/

static void mt65xx_leds_shutdown(struct platform_device *pdev)
{
	int i;
    struct nled_setting led_tmp_setting = {NLED_OFF,0,0};
    
    LEDS_DEBUG("[LED]%s\n", __func__);
    printk("[LED]mt65xx_leds_shutdown: turn off backlight\n");
    
	for (i = 0; i < MT65XX_LED_TYPE_TOTAL; i++) {
		if (!g_leds_data[i])
			continue;
		switch (g_leds_data[i]->cust.mode) {
		    case MT65XX_LED_MODE_PWM:
			    if(strcmp(g_leds_data[i]->cust.name,"lcd-backlight") == 0)
			    {
					mt_set_pwm_disable(g_leds_data[i]->cust.data);
					mt_pwm_power_off (g_leds_data[i]->cust.data);
			    }else
			    {
				    led_set_pwm(g_leds_data[i]->cust.data,&led_tmp_setting);
			    }
                break;
                
		    case MT65XX_LED_MODE_GPIO:
			    brightness_set_gpio(g_leds_data[i]->cust.data, 0);
                break;
                
		    case MT65XX_LED_MODE_PMIC:
			    brightness_set_pmic(g_leds_data[i]->cust.data, 0, 0);
                break;
                
		    case MT65XX_LED_MODE_CUST:
			    ((cust_brightness_set)(g_leds_data[i]->cust.data))(0, bl_div);
                break;
                
		    case MT65XX_LED_MODE_NONE:
		    default:
			    break;
          }
	}

}


static struct platform_driver mt65xx_leds_driver = {
	.driver		= {
		.name	= "leds-mt65xx",
		.owner	= THIS_MODULE,
	},
	.probe		= mt65xx_leds_probe,
	.remove		= mt65xx_leds_remove,
	//.suspend	= mt65xx_leds_suspend,
	.shutdown   = mt65xx_leds_shutdown,
};

#if 0
static struct platform_device mt65xx_leds_device = {
	.name = "leds-mt65xx",
	.id = -1
};

#endif

static int __init mt65xx_leds_init(void)
{
	int ret;

	LEDS_DEBUG("[LED]%s\n", __func__);

#if 0
	ret = platform_device_register(&mt65xx_leds_device);
	if (ret)
		printk("[LED]mt65xx_leds_init:dev:E%d\n", ret);
#endif
	ret = platform_driver_register(&mt65xx_leds_driver);

	if (ret)
	{
		printk("[LED]mt65xx_leds_init:drv:E%d\n", ret);
//		platform_device_unregister(&mt65xx_leds_device);
		return ret;
	}

	wake_lock_init(&leds_suspend_lock, WAKE_LOCK_SUSPEND, "leds wakelock");
	printk("[LED]add control backlight temperature!\n");

	return ret;
}

static void __exit mt65xx_leds_exit(void)
{
	platform_driver_unregister(&mt65xx_leds_driver);
//	platform_device_unregister(&mt65xx_leds_device);
}

module_param(debug_enable, int,0644);

module_init(mt65xx_leds_init);
module_exit(mt65xx_leds_exit);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("LED driver for MediaTek MT65xx chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("leds-mt65xx");

