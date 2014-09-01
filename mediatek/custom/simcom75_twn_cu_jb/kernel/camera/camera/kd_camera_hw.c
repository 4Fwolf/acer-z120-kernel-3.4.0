#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#endif

extern void mt_isp_mclk_ctrl(MINT32 en);
#define CAMERA_POWER_VCAM_AF MT65XX_POWER_LDO_VCAM_AF

//#if defined(S5K5CAGX_YUV)
//extern int s5k5cagetid;
//#endif

#define SENSOR_GT2005_YUV_PWR_ON() do {\
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_2800,mode_name)){\
		PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");\
		goto _kdCISModulePowerOn_exit_; \
		}\
		mdelay(10);\
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)){\
			PK_DBG("[CAMERA SENSOR] Fail to enable digital  power:CAMERA_POWER_VCAM_D\n"); \
			goto _kdCISModulePowerOn_exit_;\
		}\
		mdelay(10);\		
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){\
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");\
			goto _kdCISModulePowerOn_exit_;\
		}\
		mdelay(5);\
		}while (0)   
		
		
#define SENSOR_OV7690_YUV_PWR_ON() do {\
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)){\
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");\
		    goto _kdCISModulePowerOn_exit_;\
		}\
		mdelay(10);\
              if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {\
                  PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");\
                  goto _kdCISModulePowerOn_exit_; \
              }\
	       mdelay(5);  \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name)) {\
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");\
                    goto _kdCISModulePowerOn_exit_;\
                }\
                mdelay(10); \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name)) {\
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_A2\n");\
                    goto _kdCISModulePowerOn_exit_; \
                }\
                mdelay(5); \
           }while (0)

#define SENSOR_MT9P017_RAW_PWR_ON() do {\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name)){\
	       	PK_DBG("[CAMERA SENSOR] Fail to enable camera af power:CAMERA_POWER_VCAM_AF\n");\
	       	goto _kdCISModulePowerOn_exit_;\
       	}\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)){\
	       	PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");\
	       	goto _kdCISModulePowerOn_exit_;\
       	}\
       	mdelay(10);\
	    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)){\
		PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");\
		goto _kdCISModulePowerOn_exit_;\
	    }\
        mdelay(10);\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){\
       		PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");\
       		goto _kdCISModulePowerOn_exit_;\
       	}\
       	mdelay(5);\
       	}while (0)

#define SENSOR_OV2659_YUV_PWR_ON() do {\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)){\
	       	PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");\
	       	goto _kdCISModulePowerOn_exit_;\
       	}\
       	mdelay(10);\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){\
       		PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");\
       		goto _kdCISModulePowerOn_exit_;\
       	}\
       	mdelay(5);\
       	}while (0)

#define SENSOR_OV2655_YUV_PWR_ON() do {\			
  		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)){\
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");\
		    goto _kdCISModulePowerOn_exit_;\
		}\
		mdelay(10);\
              if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {\
                  PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");\
                  goto _kdCISModulePowerOn_exit_; \
              }\
	       mdelay(5);  \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name)) {\
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");\
                    goto _kdCISModulePowerOn_exit_;\
                }\
                mdelay(10); \               
           }while (0)

#define SENSOR_OV5640_RAW_PWR_ON() do {\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name)){\
	       	PK_DBG("[CAMERA SENSOR] Fail to enable camera af power:CAMERA_POWER_VCAM_AF\n");\
	       	goto _kdCISModulePowerOn_exit_;\
       	}\
	       mdelay(10);\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)){\
	       	PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");\
	       	goto _kdCISModulePowerOn_exit_;\
       	}\
       	mdelay(10);\
	       if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name)){\
		PK_DBG("[CAMERA SENSOR] Fail to digital  power:CAMERA_POWER_VCAM_D\n");\
		goto _kdCISModulePowerOn_exit_;\
	       }\
	       mdelay(10);\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){\
       		PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");\
       		goto _kdCISModulePowerOn_exit_;\
       	}\
       	mdelay(5);\
       	}while (0)
#define SENSOR_OV5647_RAW_PWR_ON() do {	\
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_2800,mode_name)) {                                           \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");                          \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(10);                                                                                                 \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");                            \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(5);                                                                                                  \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                           \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(10);                                                                                                  \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_A2\n");                           \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(5);                                                                                                  \
            }while (0)
#define SENSOR_GC0329_YUV_PWR_ON() do {									\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)){										\
	       	PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");						\
	       	goto _kdCISModulePowerOn_exit_;												\
       	}																							\
       	mdelay(10); 																						\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){ 										\
       		PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");							\
       		goto _kdCISModulePowerOn_exit_; 																		\
       	}																										\
       	mdelay(5);																							\
       	}while (0)

#define SENSOR_GC0311_YUV_PWR_ON() do {									\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)){										\
	       	PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");						\
	       	goto _kdCISModulePowerOn_exit_;												\
       	}																							\
       	mdelay(10); 																						\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){ 										\
       		PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");							\
       		goto _kdCISModulePowerOn_exit_; 																		\
       	}																										\
       	mdelay(5);																							\
       	}while (0)

#define SENSOR_BF3903_YUV_PWR_ON() do {									\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_2800,mode_name)){										\
	       	PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");						\
	       	goto _kdCISModulePowerOn_exit_;												\
       	}																							\
       	mdelay(10); 																						\
       	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){ 										\
       		PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");							\
       		goto _kdCISModulePowerOn_exit_; 																		\
       	}																										\
       	mdelay(5);																							\
       	}while (0)
#if defined(ACER_Z2)//maliyan
#define SENSOR_OV3660_YUV_PWR_ON() do {                                                                                     \
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)){                                                \
		PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                              \
		goto _kdCISModulePowerOn_exit_;                                                                             \
		}                                                                                                               \
		mdelay(5);                                                                                                     \
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){                                                 \
		PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");                                \
		goto _kdCISModulePowerOn_exit_;                                                                             \
		}                                                                                                               \
		mdelay(10);                                                                                                      \
		/*if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1500,mode_name)){                                                 \
		PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");                               \
		goto _kdCISModulePowerOn_exit_;                                                                             \
		}                                                                                                               \
		mdelay(10);   //  IOVDD use 1.8V,  DVDD use internel   LDO*/                                                                                              \
		}while (0)       	
#else
#define SENSOR_OV3660_YUV_PWR_ON() do {                                                                                     \
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)){                                                \
		PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");                              \
		goto _kdCISModulePowerOn_exit_;                                                                             \
		}                                                                                                               \
		mdelay(5);                                                                                                     \
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)){                                                 \
		PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");                                \
		goto _kdCISModulePowerOn_exit_;                                                                             \
		}                                                                                                               \
		mdelay(5);                                                                                                      \
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name)){                                                 \
		PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                               \
		goto _kdCISModulePowerOn_exit_;                                                                             \
		}                                                                                                               \
		mdelay(10);                                                                                                      \
		}while (0)
#endif
//amy0511 add begain 

#define SENSOR_HI253_YUV_PWR_ON() do {                                                                                   \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_2800,mode_name)) {                                           \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");                          \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(10);                                                                                                 \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");                            \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(2);                                                                                                  \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                           \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(5);                                                                                                  \
            }while (0)
#if defined(ACER_Z2)//maliyan
#define SENSOR_S5K5CAGX_YUV_PWR_ON() do {  \
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_A\n");                           \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                 \
                mdelay(1);                                                                                                 \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1500,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_D2\n");                            \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(2);                                                                                                  \
                 if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)) {                                           \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                          \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                } mdelay(5);                                                                                                  \
              }while (0)
#else
#define SENSOR_S5K5CAGX_YUV_PWR_ON() do {  \
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                           \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                 \
                mdelay(1);                                                                                                 \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");                            \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(2);                                                                                                  \
                 if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)) {                                           \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");                          \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                } mdelay(5);                                                                                                  \
              }while (0)
#endif

#define SENSOR_HI542_RAW_PWR_ON() do {                                                                                   \
               /* if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_2800,mode_name)) {                                           \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                          \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(5);    */                                                                                              \            
             if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");                            \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(5);                                                                                                  \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");                           \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(5);                                                                                                  \
            }while (0)
#define SENSOR_S5K4E1GA_RAW_PWR_ON() do {  \
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                           \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                 \
                mdelay(3);                                                                                                 \
                if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {                                            \
                    PK_DBG("[CAMERA SENSOR] Fail to enable analog power:CAMERA_POWER_VCAM_A\n");                            \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(3);                                                                                                  \
                 if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)) {                                           \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D2\n");                          \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                } mdelay(10);                                                                                                  \
              }while (0)
#define SENSOR_PWR_OFF() do {\
    	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {\
            PK_DBG("[CAMERA SENSOR] Fail to disable CAMERA_POWER_VCAM_A power\n");\
        }\
       mdelay(5);\
        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {\
            PK_DBG("[CAMERA SENSOR] Fail to disable CAMERA_POWER_VCAM_D power\n");\
        }\
       mdelay(5);\
        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name)){\
            PK_DBG("[CAMERA SENSOR] Fail to disable CAMERA_POWER_VCAM_D2 power\n");\
        }\
       mdelay(5);\
        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name)){\
            PK_DBG("[CAMERA SENSOR] Fail to disable CAMERA_POWER_VCAM_AF af\n");\
        }\
	}while (0)

#if defined(MT6575)

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
	u32 pinSetIdx = 0;//default main sensor

	#define IDX_PS_CMRST 0
	#define IDX_PS_CMPDN 4

	#define IDX_PS_MODE 1
	#define IDX_PS_ON   2
	#define IDX_PS_OFF  3
	u32 pinSet[3][8] = {
                    //for main sensor 
	#if defined(MT9P017_RAW) 
		#if defined(SIMCOM_I3000)//LK@I3000
			#if defined(YINGYE_I3000)			
			{
				GPIO_CAMERA_CMRST_PIN,
				GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
				GPIO_OUT_ONE,                   /* ON state */
				GPIO_OUT_ZERO,                  /* OFF state */
				GPIO_CAMERA_CMPDN_PIN,    
				GPIO_CAMERA_CMPDN_PIN_M_GPIO,
				GPIO_OUT_ONE,
				GPIO_OUT_ZERO,
			},
			#else
			{
				GPIO_CAMERA_CMRST_PIN,
				GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
				GPIO_OUT_ONE,                   /* ON state */
				GPIO_OUT_ZERO,                  /* OFF state */
				GPIO_CAMERA_INVALID,    //GPIO_CAMERA_CMPDN_PIN not used
				GPIO_CAMERA_CMPDN_PIN_M_GPIO,
				GPIO_OUT_ONE,
				GPIO_OUT_ZERO,
			},
			#endif
		#else
			{
				GPIO_CAMERA_CMRST_PIN,
				GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
				GPIO_OUT_ONE,                   /* ON state */
				GPIO_OUT_ZERO,                  /* OFF state */
				GPIO_CAMERA_CMPDN_PIN,    
				GPIO_CAMERA_CMPDN_PIN_M_GPIO,
				GPIO_OUT_ONE,
				GPIO_OUT_ZERO,
			},
		#endif
	#elif defined (GT2005_YUV) 
		{
			GPIO_CAMERA_CMRST_PIN,
			GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
			GPIO_OUT_ONE,                   /* ON state */
			GPIO_OUT_ZERO,                  /* OFF state */
			GPIO_CAMERA_CMPDN_PIN,    
			GPIO_CAMERA_CMPDN_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
	#elif defined(S5K5CAGX_YUV)&&(!defined(ACER_Z2))//maliyan
		{
			GPIO_CAMERA_CMRST_PIN,
			GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
			GPIO_OUT_ONE,                   /* ON state */
			GPIO_OUT_ZERO,                  /* OFF state */
			GPIO_CAMERA_CMPDN_PIN,
			GPIO_CAMERA_CMPDN_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
	#else
		{
			GPIO_CAMERA_CMRST_PIN,
			GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
			GPIO_OUT_ONE,                   /* ON state */
			GPIO_OUT_ZERO,                  /* OFF state */
			GPIO_CAMERA_CMPDN_PIN,   
			GPIO_CAMERA_CMPDN_PIN_M_GPIO,
			GPIO_OUT_ZERO,
			GPIO_OUT_ONE,
		},
	#endif

                    //for sub sensor 
                #if defined(OV7690_YUV)
                    #if defined(SIMCOM_I3200)
                    {
                        GPIO_CAMERA_INVALID,    //reset pin not used
                        GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ZERO,
                        GPIO_OUT_ONE,
                    },
                    #else
                    {
                        GPIO_CAMERA_INVALID,    //reset pin not used,LK@2012-5-2,OV7690 has no reset pin
                        GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ZERO,
                        GPIO_OUT_ONE,
                    },
                    #endif
                #else
                    {
                        GPIO_CAMERA_CMRST1_PIN,
                        GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ZERO,
                        GPIO_OUT_ONE,
                    },
                #endif
                    //for main_2 sensor 
                    {
                        GPIO_CAMERA_2_CMRST_PIN,
                        GPIO_CAMERA_2_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,                   /* ON state */
                        GPIO_OUT_ZERO,                  /* OFF state */
                        GPIO_CAMERA_2_CMPDN_PIN,
                        GPIO_CAMERA_2_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ZERO,
                        GPIO_OUT_ONE,
                    }
                   };

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_SECOND_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }
#if defined(SIMCOM_I706M) || defined(ACER_Z1)|| defined(ACER_Z2) //maliyan
	//if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5CAGX_YUV,currSensorName))) &&(pinSetIdx==1))
	{
		//return 0;
	}
	if( (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV3660_YUV,currSensorName)))&&(pinSetIdx ==0))
	{
		pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON] = GPIO_OUT_ZERO;
		pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF] = GPIO_OUT_ONE;
	}
        #if defined (ACER_Z2)//maliyan
        else if( (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5CAGX_YUV,currSensorName)))&&(pinSetIdx ==0))
        {
		pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON] = GPIO_OUT_ZERO;
		pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF] = GPIO_OUT_ONE;
	}
        #else
	else if( (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5CAGX_YUV,currSensorName)))&&(pinSetIdx ==0))
	{
		pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON] = GPIO_OUT_ONE;
		pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF] = GPIO_OUT_ZERO;
	}
        #endif
	if( (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4E1GA_RAW,currSensorName)))&&(pinSetIdx ==0))
	{
		pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON] = GPIO_OUT_ONE;
		pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF] = GPIO_OUT_ZERO;
	}
#endif
    //power ON
    if (On) {
        	//in case	
		#if 0  //defined (S5K5CAGX_YUV)
			if((DUAL_CAMERA_SUB_SENSOR == SensorIdx)&&(s5k5cagetid==1))
			{
				SENSOR_S5K5CAGX_YUV_PWR_ON();
				mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
				mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT);
				mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ONE);
				mdelay(1);
				mt_set_gpio_mode(GPIO_CAMERA_CMRST_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
				mt_set_gpio_dir(GPIO_CAMERA_CMRST_PIN,GPIO_DIR_OUT);
				mt_set_gpio_out(GPIO_CAMERA_CMRST_PIN,GPIO_OUT_ONE);
				mdelay(10);
				mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
				mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT);
				mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO);
				mdelay(200);
			}
		#endif


        if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GT2005_YUV,currSensorName))) {
            SENSOR_GT2005_YUV_PWR_ON();
        } 
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2659_YUV,currSensorName))) {
            SENSOR_OV2659_YUV_PWR_ON();
        }
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2655_YUV,currSensorName))) {
            SENSOR_OV2655_YUV_PWR_ON();
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5640_RAW,currSensorName))) {
            SENSOR_OV5640_RAW_PWR_ON();            
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_MT9P017_RAW,currSensorName))) {
            SENSOR_MT9P017_RAW_PWR_ON();
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV7690_YUV,currSensorName))) {
            SENSOR_OV7690_YUV_PWR_ON();
        }  //amy0320 add 
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5647_RAW,currSensorName))) {
			printk("SENSOR_OV5647_RAW_PWR_ON'");
            SENSOR_OV5647_RAW_PWR_ON();            
        }
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC0329_YUV,currSensorName))) 
	{
		printk("SENSOR_GC0329_YUV_PWR_ON'");
		SENSOR_GC0329_YUV_PWR_ON();
	} 	
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_BF3903_YUV,currSensorName))) 
	{
		printk("SENSOR_BF3903_YUV_PWR_ON'");
		SENSOR_BF3903_YUV_PWR_ON();
	} 	
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV3660_YUV,currSensorName))) 
	{
		printk("SENSOR_OV3660_YUV_PWR_ON'");
                #if defined(ACER_Z2)//maliyan
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
            mdelay(3);
        }
                #endif
		SENSOR_OV3660_YUV_PWR_ON();
	}
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5CAGX_YUV,currSensorName))) 
	{
		printk("SENSOR_S5K5CAGX_YUV_PWR_ON'");
                #if defined(ACER_Z2)//maliyan
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
            mdelay(3);
             }
            #endif
		SENSOR_S5K5CAGX_YUV_PWR_ON();
	}
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI253_YUV,currSensorName))) 
	{
		printk("SENSOR_HI253_YUV_PWR_ON '");
		SENSOR_HI253_YUV_PWR_ON();  //amy0511 add begain 
	}
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC0311_YUV,currSensorName))) 
	{
		printk("SENSOR_GC0311_YUV_PWR_ON'");
		SENSOR_GC0311_YUV_PWR_ON();
	}
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI542_RAW,currSensorName))) 
	{
		printk("SENSOR_HI542_RAW_PWR_ON '");
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
              if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
              if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
              if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		}
		mt_isp_mclk_ctrl(0);
		mdelay(2);
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)) {                                           \
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power:CAMERA_POWER_VCAM_D\n");                          \
                    goto _kdCISModulePowerOn_exit_;                                                                         \
                }                                                                                                           \
                mdelay(2);   
		//RST pin
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		}
		mdelay(5);
		SENSOR_HI542_RAW_PWR_ON();  //amy0511 add begain 
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
              if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
              if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
              if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		}
              mdelay(2);
	      mt_isp_mclk_ctrl(1);
		mdelay(20); 
	}
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4E1GA_RAW,currSensorName))) 
	{
		printk("SENSOR_S5K4E1GA_RAW_PWR_ON'");
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
            mdelay(3);
            }
		SENSOR_S5K4E1GA_RAW_PWR_ON();
	}
	else
	{
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
		mdelay(3);
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}  
		mdelay(50);
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}        
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		} 
		// wait power to be stable 
		mdelay(5); 
		// default is nothing
	}

        //disable inactive sensor
	#if 0  //defined(S5K5CAGX_YUV)
		if((DUAL_CAMERA_SUB_SENSOR == SensorIdx)&&(s5k5cagetid==1))
		{
		}
		else
		{
	#endif
		        if (GPIO_CAMERA_INVALID != pinSet[1-pinSetIdx][IDX_PS_CMRST]) {
		            if(mt_set_gpio_mode(pinSet[1-pinSetIdx][IDX_PS_CMRST],pinSet[1-pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		            if(mt_set_gpio_dir(pinSet[1-pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		            if(mt_set_gpio_out(pinSet[1-pinSetIdx][IDX_PS_CMRST],pinSet[1-pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
		        }        
		        if (GPIO_CAMERA_INVALID != pinSet[1-pinSetIdx][IDX_PS_CMPDN]) {
		            if(mt_set_gpio_mode(pinSet[1-pinSetIdx][IDX_PS_CMPDN],pinSet[1-pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
		            if(mt_set_gpio_dir(pinSet[1-pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
		            if(mt_set_gpio_out(pinSet[1-pinSetIdx][IDX_PS_CMPDN],pinSet[1-pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		        }        
	#if 0 //defined(S5K5CAGX_YUV)
		}
	#endif
#if defined (ACER_Z2)
        if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4E1GA_RAW,currSensorName))) 
	{
         if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
            //PDN pin
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
        }
            mdelay(30);
        }
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5CAGX_YUV,currSensorName))) 
	{
	//PDN pin
           if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
            //mdelay(10);
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
            mdelay(1);
        }
        //RST pin
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(10);
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(15);
        }
	}
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV3660_YUV,currSensorName)))  
	{
	//PDN pin
           if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
            //mdelay(10);
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
            mdelay(1);
        }
        //RST pin
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(10);
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(15);
        }
	}
#elif defined(HI542_RAW)
//do nothing , reset and pwdn has been set before
#else
        //enable active sensor
        //RST pin
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(10);
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(1);
        }
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
            //PDN pin
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
        }
#endif
    }
    else {//power OFF

        //PK_DBG("[OFF]sensorIdx:%d \n",SensorIdx);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
        }
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    	    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }

	SENSOR_PWR_OFF();
    }//

	return 0;

_kdCISModulePowerOn_exit_:

	SENSOR_PWR_OFF();
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);
#else 
#error Error !! forget to implement power control for image sensor

#endif 



