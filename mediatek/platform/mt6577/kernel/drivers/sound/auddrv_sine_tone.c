/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2009
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


/*******************************************************************************
 *
 * Filename:
 * ---------
 *   auddrv_sine_tone.c
 *
 * Project:
 * --------
 *   Android Audio Driver
 *
 * Description:
 * ------------
 *   Sine tone generation
 *
 * Author:
 * -------
 *   Chipeng    (mtk02308)
 *   Stan Huang (mtk01728)
 *
 *
 *------------------------------------------------------------------------------
 * $Revision$
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/

/*****************************************************************************
*                E X T E R N A L   R E F E R E N C E S
******************************************************************************
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/xlog.h>

#include <linux/pmic6326_sw.h>
#include <mach/mt_typedefs.h>
#include "auddrv_def.h"
#include <linux/spinlock.h>

/*****************************************************************************
*                          C O N S T A N T S
******************************************************************************
*/

#define SOUND_FAKE_READ_SIZE (128)

/*****************************************************************************
*                     C O M P I L E R   F L A G S
******************************************************************************
*/


/*****************************************************************************
*                         D A T A   T Y P E S
******************************************************************************
*/


static short Tone_1k[] = {
    0x0   ,     0x0,          0xa02     ,0xa02,
    0x13a4   ,0x13a3,    0x1c83   ,0x1c83,
    0x2449   ,0x2449,    0x2aac   ,0x2aac,
    0x2f6a   ,0x2f69,    0x3254   ,0x3255,
    0x3352   ,0x3351,    0x3255   ,0x3255,
    0x2f6a   ,0x2f69,    0x2aac   ,0x2aac,
    0x2449   ,0x244a,    0x1c83   ,0x1c83,
    0x13a4   ,0x13a3,    0xa03     ,0xa02,
    0x0         ,0x0     ,    0xf5fd   ,0xf5fd,
    0xec5d   ,0xec5c,    0xe37e   ,0xe37e,
    0xdbb7   ,0xdbb7,    0xd555   ,0xd555,
    0xd097   ,0xd096,    0xcdab   ,0xcdac,
    0xccaf   ,0xccae,    0xcdab   ,0xcdab,
    0xd096   ,0xd097,    0xd554   ,0xd554,
    0xdbb7   ,0xdbb7,    0xe37e   ,0xe37d,
    0xec5d   ,0xec5d,    0xf5fd   ,0xf5fe
};

static char* Tone_ptr = (char*)Tone_1k;
static unsigned int  count = 0;


/*****************************************************************************
*                        F U N C T I O N   D E F I N I T I O N
******************************************************************************
*/

void copy_to_user_fake(char* Read_Data_Ptr , int read_size)
{
   uint32_t copy_size =0;
   char*write_ptr = Read_Data_Ptr;
   unsigned long result;
   PRINTK_AUDDRV("copy_to_user_fake read_size = %d Tone_ptr count = %d\n", read_size, count);
   while(read_size > 0)
   {
      if(count == 0){
         if (read_size >SOUND_FAKE_READ_SIZE){
             copy_size = SOUND_FAKE_READ_SIZE;
         }
         else{
             copy_size = read_size;
         }
         PRINTK_AUDDRV("1 copy_to_user_  copy_size = %d Tone_ptr count = %d read_size =%d \n", copy_size, count, read_size);
         result = copy_to_user( (void __user *)write_ptr, Tone_ptr, copy_size);
         read_size -= copy_size;
         write_ptr+= copy_size;
         count += copy_size;
         count %= SOUND_FAKE_READ_SIZE;
      }
      else if(count > 0 && read_size >= (SOUND_FAKE_READ_SIZE - count)){
         copy_size = SOUND_FAKE_READ_SIZE - count;
         PRINTK_AUDDRV("2 copy_to_user_  copy_size = %d Tone_ptr count = %d read_size =%d \n", copy_size, count, read_size);
         result = copy_to_user( (void __user *)write_ptr, (Tone_ptr+count) , copy_size);
         read_size -= copy_size;
         write_ptr+= copy_size;
         count =0;
      }

      else if(count > 0 && read_size < (SOUND_FAKE_READ_SIZE - count)){
         copy_size = read_size;
         PRINTK_AUDDRV("3 copy_to_user_  copy_size = %d Tone_ptr count = %d read_size =%d \n", copy_size, count, read_size);
         result = copy_to_user( (void __user *)write_ptr, (Tone_ptr+count) , copy_size);
         read_size -= copy_size;
         write_ptr+= copy_size;
         count += copy_size;
         count %= SOUND_FAKE_READ_SIZE;
      }
   }
}




