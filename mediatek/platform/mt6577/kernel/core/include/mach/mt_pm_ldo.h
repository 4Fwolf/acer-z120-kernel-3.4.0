#ifndef _MT_PMIC_LDO_H_
#define _MT_PMIC_LDO_H_

#include <mach/mt_typedefs.h>

#define MAX_DEVICE      5
#define MAX_MOD_NAME    32

#define NON_OP "NOP"

/* Debug message event */
#define DBG_PMAPI_NONE		    0x00000000	
#define DBG_PMAPI_CG			0x00000001	
#define DBG_PMAPI_PLL			0x00000002	
#define DBG_PMAPI_SUB			0x00000004	
#define DBG_PMAPI_PMIC			0x00000008	
#define DBG_PMAPI_ALL			0xFFFFFFFF	
	
#define DBG_PMAPI_MASK	   (DBG_PMAPI_ALL)

typedef enum MT65XX_POWER_TAG {
	/* LDO 1*/
	MT65XX_POWER_LDO_VM12_1 = 0,
	MT65XX_POWER_LDO_VM12_2,	
	MT65XX_POWER_LDO_VM12_INT,
	MT65XX_POWER_LDO_VIO28,
	MT65XX_POWER_LDO_VSIM,
	MT65XX_POWER_LDO_VSIM2,
	MT65XX_POWER_LDO_VUSB,
	MT65XX_POWER_LDO_VCAMD,
	MT65XX_POWER_LDO_VCAM_IO,
	MT65XX_POWER_LDO_VCAM_AF,
	MT65XX_POWER_LDO_VMC,
	MT65XX_POWER_LDO_VMCH,
	MT65XX_POWER_LDO_VGP,
	MT65XX_POWER_LDO_VGP2,
	MT65XX_POWER_LDO_VIBR,
	MT65XX_POWER_LDO_VRF,
	MT65XX_POWER_LDO_VTCXO,
	MT65XX_POWER_LDO_VA1,
	MT65XX_POWER_LDO_VA2,
	MT65XX_POWER_LDO_VCAMA,
	MT65XX_POWER_LDO_VRTC,
    MT65XX_POWER_COUNT_END,
    MT65XX_POWER_NONE = -1
} MT65XX_POWER;

typedef enum MT65XX_POWER_VOL_TAG 
{
    VOL_DEFAULT, 
	VOL_1200 = 1200,	
    VOL_1300 = 1300,    
    VOL_1500 = 1500,    
    VOL_1800 = 1800,    
    VOL_2000 = 2000,
    VOL_2100 = 2100,
    VOL_2500 = 2500,    
    VOL_2800 = 2800, 
    VOL_3000 = 3000,
    VOL_3300 = 3300        
} MT65XX_POWER_VOLTAGE;	

typedef struct { 
    DWORD dwPowerCount; 
    BOOL bDefault_on;
    char name[MAX_MOD_NAME];        
    char mod_name[MAX_DEVICE][MAX_MOD_NAME];    
} DEVICE_POWER;

typedef struct
{    
    DEVICE_POWER Power[MT65XX_POWER_COUNT_END];    
} ROOTBUS_HW;

//==============================================================================
// PMIC6329 Exported Function for power service
//==============================================================================
extern void pmic_ldo_enable(MT65XX_POWER powerId, kal_bool powerEnable);
extern void pmic_ldo_vol_sel(MT65XX_POWER powerId, MT65XX_POWER_VOLTAGE powerVolt);

extern bool hwPowerOn(MT65XX_POWER powerId, MT65XX_POWER_VOLTAGE powerVolt, char *mode_name);
extern bool hwPowerDown(MT65XX_POWER powerId, char *mode_name);

#endif // _MT_PMIC_LDO_H_

