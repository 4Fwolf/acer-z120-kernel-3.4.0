/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   boot_mode.c
 *
 * Project:
 * --------
 *   YuSu 
 *
 * Description:
 * ------------
 *   This file implements MTK boot mode.
 *
 * Author:
 * -------
 *   CC Hwang (mtk00702)
 *   Hong-Rong Hsu (mtk02678)
 *   Andy Hsu (mtk04080)
 *
 ****************************************************************************/
#include <common.h>
#include <command.h>
#include <asm/arch/mt65xx.h>
#include <asm/arch/mt65xx_typedefs.h>
#include <asm/arch/boot_mode.h>
#include <asm/mach-types.h>
#include <asm/errno.h>
#include <asm/byteorder.h>



// global variable for specifying boot mode (default = NORMAL)
BOOTMODE g_boot_mode = NORMAL_BOOT;

BOOL meta_mode_check(void)
{	
	//printf(" > check meta mode to bypass power key\n");
	if(g_boot_mode == META_BOOT || g_boot_mode == ADVMETA_BOOT || g_boot_mode ==ATE_FACTORY_BOOT)
	{	
	  return TRUE;
	}
	else
	{	return FALSE;
	}
}


// check the boot mode : (1) meta mode or (2) recovery mode ...
void boot_mode_select(void)
{	
    #ifdef CFG_META_MODE
    if (meta_detection())
    {	
      return;
    }
    #endif   
    #ifdef CFG_FACTORY_MODE
    if (factory_detection())
    {	
      return;
    }
    #endif    
    #ifdef CFG_RECOVERY_MODE
    if(recovery_detection())
    {	
      //**************************************
  		//* CHECK IMAGE
  		//**************************************
  		if(DRV_Reg32(0x40002300)==0xE92D4800)
  		{	
  		  printf(" > do recovery_check\n");
  			//jump(0x40002300); 
  		}
  		else
  		{	
  		  printf(" > bypass recovery_check\n");
  		}
    	return;
    }
    #endif   
}

CHIP_VER get_chip_eco_ver(void)
{   
    return DRV_Reg32(APHW_VER);
}

CHIP_VER get_chip_ver(void)
{
    unsigned int hw_subcode = DRV_Reg32(APHW_SUBCODE);
    unsigned int sw_ver = DRV_Reg32(APSW_VER);
    CHIP_VER ver = CHIP_6575_E2;

    if (0x8A00 == hw_subcode) {
        if (0xE100 == sw_ver) {
            ver = CHIP_6575_E1;
        } else if (0xE201 == sw_ver) {
            if (0x40000000 == (DRV_Reg32(0xC1019100) & 0x40000000)) {
                ver = CHIP_6575_E3;
            } else {
                ver = CHIP_6575_E2;
            }
        }    
    } else if (0x8B00 == hw_subcode) {
        ver = CHIP_6577_E1;
    }
    return ver;
}
