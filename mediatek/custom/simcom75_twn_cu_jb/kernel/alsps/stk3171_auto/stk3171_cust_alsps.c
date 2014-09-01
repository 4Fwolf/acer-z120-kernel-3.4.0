
#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#include <stk3171.h>
//#include <mach/mt6577_pm_ldo.h>

#if defined(ACER_Z2)//maliyan
static struct alsps_hw_stk cust_alsps_hw = {
    .i2c_num    = 0,
    //.polling_mode =1,
    .polling_mode_ps =0,
    .polling_mode_als =1,
    .power_id   =MT65XX_POWER_LDO_VGP2,
    .power_vol  = VOL_2800,        
    .i2c_addr   = {0x90, 0x00, 0x00, 0x00},	/*STK31xx*/
    .als_level  = { 100,  5210,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,20000},
    .als_value  = { 31, 1000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000},
   // .ps_threshold = 0xFE,
   .als_cmd_val = 0x49,	/*ALS_GAIN=1, IT_ALS=400ms*/
   .ps_cmd_val = 0x21,	/*SLP=30ms, IT_PS=0.2ms*/
   .ps_gain_setting = 0x09, /*PS_GAIN=8X */
    .ps_threshold_high = 90,
    .ps_threshold_low = 70,
};
#else
static struct alsps_hw_stk cust_alsps_hw = {
    .i2c_num    = 0,
	//.polling_mode =1,
	.polling_mode_ps =1,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/        
    .i2c_addr   = {0x90, 0x00, 0x00, 0x00},	/*STK31xx*/
    .als_level  = {5,  9, 36, 59, 82, 132, 205, 273, 500, 845, 1136, 1545, 2364, 4655, 6982},	/* als_code */
    .als_value  = {0, 10, 40, 65, 90, 145, 225, 300, 550, 930, 1250, 1700, 2600, 5120, 7680, 10240},    /* lux */
   // .ps_threshold = 0xFE,
   .als_cmd_val = 0x49,	/*ALS_GAIN=1, IT_ALS=400ms*/
   .ps_cmd_val = 0x21,	/*SLP=30ms, IT_PS=0.2ms*/
   .ps_gain_setting = 0x09, /*PS_GAIN=8X */
    .ps_threshold_high = 0x78,
    .ps_threshold_low = 0x6E,
};
#endif
struct alsps_hw_stk* stk3171_get_cust_alsps_hw_stk(void) {
    return &cust_alsps_hw;
}
#if defined(ACER_Z2)
int C_CUST_ALS_FACTOR_STK3171=1745; 
#else
int C_CUST_ALS_FACTOR_STK3171=1000; 
#endif
