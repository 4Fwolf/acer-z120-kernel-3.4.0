/* Copyright Statement:
 *
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

/* 
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
/*
 * Definitions for stk31xx als/ps sensor chip.
 */
#ifndef __STK3171_H__
#define __STK3171_H__

#include <linux/ioctl.h>

/*ALSPS REGS*/
#define ALS_CMD          0x01
#define ALS_DT1          0x02
#define ALS_DT2          0x03
#define ALS_THDH1        0x04
#define ALS_THDH2        0x05
#define ALS_THDL1        0x06
#define ALS_THDL2        0x07
#define ALSPS_STATUS     0x08
#define PS_CMD           0x09
#define PS_DT            0x0A
#define PS_THDH          0x0B
#define PS_THDL          0x0C
#define PS_GAIN          0x82
#define SW_RESET	0x80

/*ALS Command*/
#define SD_ALS      (1      << 0)
#define INT_ALS     (1      << 1)
#define IT_ALS      (0x03   << 2)
#define THD_ALS     (0x03   << 4)
#define GAIN_ALS    (0x03   << 6)

/*Proximity sensor command*/
#define SD_PS       (1      << 0)
#define INT_PS      (1      << 1)
#define IT_PS       (0x03   << 2)
#define DR_PS       (1      << 4)
#define SLP_PS      (0x03   << 5)
#define INTM_PS     (1      << 7)

#define CONFIG_STK_ALS_CHANGE_THRESHOLD	5
/* STK calibration */
#define STK_AUTO_CT_CALI_SATU
//#define STK_MANUAL_GREYCARD_CALI
//#define STK_AUTO_CT_CALI_NO_SATU

#define MAX(x,y) ((x)>(y)? (x):(y))
#define MIN(x,y) ((x)>(y)? (y):(x))

#if (defined(STK_MANUAL_GREYCARD_CALI) || defined(STK_AUTO_CT_CALI_SATU) || defined(STK_AUTO_CT_CALI_NO_SATU))
#define STK_CALI_SAMPLE_NO		5		
#define STK_THD_H_MAX			250
#define STK_THD_H_MIN			35
#define STK_THD_L_MAX			235
#define STK_THD_L_MIN			20

#define STK_HIGH_THD				0
#define STK_LOW_THD				1
#define STK_CT_AVG				2

#define STK_DATA_MAX			0
#define STK_DATA_AVE				1
#define STK_DATA_MIN				2
   
const static uint16_t cali_sample_time_table[4] = {20, 40, 100, 300};
#define STK_CALI_VER0			0x48
#define STK_CALI_VER1			0x03
#define STK_CALI_FILE "/data/misc/stkpscali.conf"
#define STK_CALI_FILE_SIZE 10

#define STK_MAX_PS_CROSSTALK	200
#define STK_THD_H_ABOVE_CT		30
#define STK_THD_L_ABOVE_CT		15
#define STK_CT_DENOMINATOR		10
#define STK_CT_NUMERATOR		13

#define STK_K_OFFSET			70
#define STK_A_OFFSET			120
#define STK_DEF_CTK				50
#define STK_CTK_MAX				150
#define STK_CTA_MAX				220
#endif

#if defined(STK_AUTO_CT_CALI_SATU)
#define STK_THD_H_BELOW_MAX_CT		15
#define STK_THD_L_BELOW_MAX_CT		30
#define STK_REDETECT_CT				50
#endif

#ifdef STK_MANUAL_GREYCARD_CALI
#define STK_MIN_GREY_PS_DATA			50
#define STK_DIFF_GREY_N_THD_H		15
#define STK_DIFF_GREY_N_THD_L		30
#endif	/*	#ifdef STK_MANUAL_GREYCARD_CALI	*/

//#if (defined(STK_MANUAL_GREYCARD_CALI) || defined(STK_AUTO_CT_CALI_SATU))
//#define SITRONIX_PERMISSION_THREAD
//#endif
struct alsps_hw_stk {
    int i2c_num;                                    /*!< the i2c bus used by ALS/PS */
    int power_id;                                   /*!< the power id of the chip */
    int power_vol;                                  /*!< the power voltage of the chip */
	//int polling_mode;                               /*!< 1: polling mode ; 0:interrupt mode*/
	int polling_mode_ps;                               /*!< 1: polling mode ; 0:interrupt mode*/
	int polling_mode_als;                               /*!< 1: polling mode ; 0:interrupt mode*/
    unsigned char   i2c_addr[C_CUST_I2C_ADDR_NUM];  /*!< i2c address list, some chip will have multiple address */
    unsigned int    als_level[C_CUST_ALS_LEVEL-1];  /*!< (C_CUST_ALS_LEVEL-1) levels divides all range into C_CUST_ALS_LEVEL levels*/
    unsigned int    als_value[C_CUST_ALS_LEVEL];    /*!< the value reported in each level */
    unsigned int    ps_threshold;                   /*!< the threshold of proximity sensor */
    unsigned int	als_cmd_val;
    unsigned int	ps_cmd_val;
    unsigned int	ps_gain_setting;
    unsigned int    ps_threshold_high;
    unsigned int    ps_threshold_low;
	unsigned int    als_window_loss;                /*!< the window loss  */
};


extern struct alsps_hw_stk* stk3171_get_cust_alsps_hw_stk(void);
extern int C_CUST_ALS_FACTOR_STK3171;
#endif

