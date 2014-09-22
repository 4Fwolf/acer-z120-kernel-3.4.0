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

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/xlog.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include "mach/irqs.h"
#include "mach/sync_write.h"
#include "mach/mt_reg_base.h"
#include "mach/mt_clock_manager.h"
#include "mach/mt_typedefs.h"
#include "mach/mt_sc.h"
#include "mach/mt_boot.h"
#include "mach/mt_ca9_power.h"
#include "mach/mt_dcm.h"
#include "mach/pmic_mt6329_sw.h"
#include "mach/upmu_common_sw.h"

#define HW_RESV (0xF1019100)

/*********************************************************************
 * FUNCTION DEFINATIONS
 ********************************************************************/
extern CHIP_VER get_chip_ver(void);
extern ssize_t mt6329_read_byte(u8 cmd, u8 *returnData);

kal_uint32 mt6577_get_bus_freq(void)
{
    kal_uint32 fout = 0, fbdiv = 0, fbsel = 0, prediv = 0, postdiv = 0, ckdiv = 0;

    if ((DRV_Reg32(TOP_CKMUXSEL) & 0x3) == 0x0) // Using CLKSQ
    {
        fout = 26000;
    }
    else if ((DRV_Reg32(TOP_CKMUXSEL) & 0x3) == 0x2) // Using WPLL
    {
        fout = 197000;
    }
    else
    {
        fbdiv = (DRV_Reg32(MAINPLL_CON0) & 0x7F00) >> 8;

        fbsel = (DRV_Reg32(MAINPLL_CON0) & 0x0030) >> 4;
        if (fbsel == 0)
            fbsel = 1;
        else if (fbsel == 1)
            fbsel = 2;
        else
            fbsel = 4;

        prediv = (DRV_Reg32(MAINPLL_CON0) & 0x00C0) >> 6;
        if (prediv == 0)
            prediv = 1;
        else if (prediv == 1)
            prediv = 2;
        else
            prediv = 4;

        postdiv = (DRV_Reg32(MAINPLL_CON1) & 0x0030) >> 4;
        if (postdiv == 0)
            postdiv = 1;
        else if (postdiv == 1)
            postdiv = 2;
        else
            postdiv = 4;

        fout = 26000 * (fbdiv + 1) * fbsel / prediv / postdiv; // KHz
    }

    ckdiv = (DRV_Reg32(TOP_CKDIV0) & 0x00000018) >> 3;

    switch (ckdiv)
    {
        case 0:
            fout = fout; // bus clock will not be divided
            break;
        case 1:
            fout = fout / 4; // bus clock will be divided by 4
            break;
        case 2:
            fout = fout / 5; // bus clock will be divided by 5
            break;
        case 3:
            fout = fout / 6; // bus clock will be divided by 6
            break;
        default:
            break;
    }

    return fout;
}
EXPORT_SYMBOL(mt6577_get_bus_freq);

void chip_dep_init(void)
{
    scu_control(SCU_IC_STANDBY_ON); // enable GIC auto gated

    scu_control(SCU_STANDBY_ON); // enable SCU auto gated

    mt_ca9_power_ctrl(CA9_DYNA_CLK_GATING_ENABLE); // enable dynamic clock gating

    mt_l2c_power_ctrl(L2C_STANDBY_ENABLE | L2C_DYNA_CLK_GATING_ENABLE); // enable standby mode and high-level dynamic clock gating

    scu_set_cpu_pwr_status(SCU_CPU_PWR_NORMAL); // current cpu power state
}

unsigned int mt_pmic_cpu_max_volt(void)
{
    unsigned int volt, index, volt_bias;
    
    if ((DRV_Reg32(HW_RESV) & (0x1 << 23)) && ((DRV_Reg32(HW_RESV) & (0x1 << 20)) == 0))
    {
        volt = 0x19; // P.K. DVS_VOL_11
    }
    else
    {
        if ((DRV_Reg32(HW_RESV) & (0x1 << 21)))
            volt_bias = 2;
        else if ((DRV_Reg32(HW_RESV) & (0x1 << 22)))
            volt_bias = 1;
        else
            volt_bias = 0;

        index = (DRV_Reg32(HW_RESV) & 0xE000) >> 13;
        if (index == 0x0)
            volt = 0x17 + volt_bias;
        else if (index == 0x1)
            volt = 0x16 + volt_bias;
        else if (index == 0x2)
            volt = 0x15 + volt_bias;
        else if (index == 0x3)
            volt = 0x14 + volt_bias;
        else if (index == 0x4)
            volt = 0x13 + volt_bias;
        else if (index == 0x5)
            volt = 0x18 + volt_bias;
        else if (index == 0x6)
            volt = 0x19 + volt_bias;
        else if (index == 0x7)
            volt = 0x19 + volt_bias;
        else
            volt = 0x13 + volt_bias;
    }
    
    return volt;
}

void mt_pmic_low_power_init(void)
{
    unsigned int volt = 0;
    
    /********************
    * PMIC VPROC setting
    *********************/

    upmu_buck_vosel_srclken_0(BUCK_VPROC, 0x08); // VPROC 0.9V in sleep mode

    if (get_chip_ver() >= CHIP_6577_E1)
    {
        volt = mt_pmic_cpu_max_volt();

        if ((DRV_Reg32(HW_RESV) & (0x1 << 23)) && ((DRV_Reg32(HW_RESV) & (0x1 << 20)) == 0))
        {
            upmu_buck_vosel_dvs_00(BUCK_VPROC, volt);
            upmu_buck_vosel_dvs_01(BUCK_VPROC, 0x0F); // 1.075V DVS_VOL_01
            upmu_buck_vosel_dvs_10(BUCK_VPROC, 0x13); // 1.175V DVS_VOL_10
            upmu_buck_vosel_dvs_11(BUCK_VPROC, 0x17); // 1.275V DVS_VOL_11
        }
        else
        {
            if (DRV_Reg32(HW_RESV) & (0x1 << 12))
            {
                if ((DRV_Reg32(HW_RESV) & (0x1 << 17)) && ((DRV_Reg32(HW_RESV) & (0x1 << 16)) == 0))
                {
                    upmu_buck_vosel_dvs_00(BUCK_VPROC, volt);
                    upmu_buck_vosel_dvs_01(BUCK_VPROC, 0x0F); // 1.075V DVS_VOL_01
                    upmu_buck_vosel_dvs_10(BUCK_VPROC, 0x13); // 1.175V DVS_VOL_10
                    upmu_buck_vosel_dvs_11(BUCK_VPROC, 0x17); // 1.275V DVS_VOL_11
                }
                else
                {
                    upmu_buck_vosel_dvs_00(BUCK_VPROC, 0x08); // 0.900V DVS_VOL_00
                    upmu_buck_vosel_dvs_01(BUCK_VPROC, 0x0F); // 1.075V DVS_VOL_01
                    upmu_buck_vosel_dvs_10(BUCK_VPROC, 0x13); // 1.175V DVS_VOL_10
                    upmu_buck_vosel_dvs_11(BUCK_VPROC, volt);
                }
            }
            else
            {
                upmu_buck_vosel_dvs_00(BUCK_VPROC, 0x08); // 0.900V DVS_VOL_00
                upmu_buck_vosel_dvs_01(BUCK_VPROC, 0x0F); // 1.075V DVS_VOL_01
                upmu_buck_vosel_dvs_10(BUCK_VPROC, 0x13); // 1.175V DVS_VOL_10
                upmu_buck_vosel_dvs_11(BUCK_VPROC, volt);
            }
        }
    }
    else if (get_chip_ver() >= CHIP_6575_E2)
    {
        upmu_buck_vosel_dvs_00(BUCK_VPROC, 0x0B);
        upmu_buck_vosel_dvs_01(BUCK_VPROC, 0x0F);
        upmu_buck_vosel_dvs_10(BUCK_VPROC, 0x13);

        if ((DRV_Reg32(HW_RESV) & (0x1 << 29)))
        {
            upmu_buck_vosel_dvs_11(BUCK_VPROC, 0x17);
        }
        else
        {
            upmu_buck_vosel_dvs_11(BUCK_VPROC, 0x16);
        }
    }
    else if (get_chip_ver() >= CHIP_6575_E1)
    {
        upmu_buck_vosel_dvs_00(BUCK_VPROC, 0x13);

        if ((DRV_Reg32(HW_RESV) & (0x1 << 29)))
        {
            upmu_buck_vosel_dvs_01(BUCK_VPROC, 0x17);
        }
        else
        {
            upmu_buck_vosel_dvs_01(BUCK_VPROC, 0x16);
        }

        upmu_buck_vosel_dvs_10(BUCK_VPROC, 0x13);

        if ((DRV_Reg32(HW_RESV) & (0x1 << 29)))
        {
            upmu_buck_vosel_dvs_11(BUCK_VPROC, 0x17);
        }
        else
        {
            upmu_buck_vosel_dvs_11(BUCK_VPROC, 0x16);
        }
    }
    else
    {
        upmu_buck_vosel_dvs_00(BUCK_VPROC, 0x16);
        upmu_buck_vosel_dvs_01(BUCK_VPROC, 0x16);
        upmu_buck_vosel_dvs_10(BUCK_VPROC, 0x16);
        upmu_buck_vosel_dvs_11(BUCK_VPROC, 0x16);
    }

    DRV_WriteReg32(SC_AP_DVFS_CON, ((DRV_Reg32(SC_AP_DVFS_CON) & 0xFFFFFFFC) | 0x03)); // set cpu to top voltage

    upmu_buck_ctrl(BUCK_VPROC, 0x3); // VPROC controlled by SRCLKEN and AP_DVFS_CON1/0

    /********************
    * PMIC VCORE setting
    *********************/

    if ((DRV_Reg32(HW_RESV) & (0x1 << 19)))
    {
        upmu_buck_vosel(BUCK_VCORE, UPMU_VOLT_0_8_0_0_V); // VCORE 0.8V in sleep mode
    }
    else
    {
        upmu_buck_vosel(BUCK_VCORE, UPMU_VOLT_0_9_0_0_V); // VCORE 0.9V in sleep mode
    }

    /********************
    * PMIC Other setting
    *********************/

    pmic_config_interface(0x8B, 0x08, 0x1F, 0x0); // VM12_INT 0.9V in sleep mode
    pmic_config_interface(0x8C, 0x10, 0x1F, 0x0); // VM12_INT_LOW_BOUND
    pmic_config_interface(0x8F, 0x01, 0x01, 0x4); // VM12_INT Tracking VPROC
    pmic_config_interface(0x90, 0x01, 0x01, 0x0); // VM12_INT_LP_SEL HW control

    pmic_config_interface(0x85, 0x01, 0x01, 0x0); // VM12_1_LP_SEL HW control
    pmic_config_interface(0x89, 0x01, 0x01, 0x0); // VM12_2_LP_SEL HW control

    pmic_config_interface(0xA9, 0x01, 0x01, 0x0); // VMC_LP_SEL HW control
    pmic_config_interface(0xAD, 0x01, 0x01, 0x0); // VMCH_LP_SEL HW control

    pmic_config_interface(0xC6, 0x01, 0x01, 0x0); // VA1_LP_SEL HW control

    pmic_config_interface(0xC1, 0x01, 0x01, 0x1); // VTCXO_ON_CTRL HW control

    pmic_config_interface(0x4F, 0x01, 0x01, 0x6); // BUCK clock keep 2MHz select
    pmic_config_interface(0x4F, 0x01, 0x01, 0x7); // OSC10M and 2M auto select function enable
}

void mt_pmic_low_power_reg_dump(void)
{
    kal_uint8 i, value;

    for (i = 0x44; i <= 0x4F; i++)
    {
        mt6329_read_byte(i, &value);
        xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "[pmic_dump_register] Reg[0x%X]=0x%X\n", i, value);
    }

    for (i = 0x57; i <= 0x59; i++)
    {
        mt6329_read_byte(i, &value);
        xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "[pmic_dump_register] Reg[0x%X]=0x%X\n", i, value);
    }

    for (i = 0x85; i <= 0x90; i++)
    {
        mt6329_read_byte(i, &value);
        xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "[pmic_dump_register] Reg[0x%X]=0x%X\n", i, value);
    }

    mt6329_read_byte(0xA9, &value);
    xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "[pmic_dump_register] Reg[0x%X]=0x%X\n", 0xA9, value);

    mt6329_read_byte(0xAD, &value);
    xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "[pmic_dump_register] Reg[0x%X]=0x%X\n", 0xAD, value);

    mt6329_read_byte(0xC1, &value);
    xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "[pmic_dump_register] Reg[0x%X]=0x%X\n", 0xC1, value);

    mt6329_read_byte(0xC6, &value);
    xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "[pmic_dump_register] Reg[0x%X]=0x%X\n", 0xC6, value);

    mt6329_read_byte(0xD1, &value);
    xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "[pmic_dump_register] Reg[0x%X]=0x%X\n", 0xD1, value);
}

static int pmic_reg_dump_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    mt_pmic_low_power_reg_dump();
    return 0;
}

static int cpu_speed_dump_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    unsigned int speed;

    DRV_WriteReg32(0xF0001008, 0xA);

    DRV_WriteReg32(0xF0007400, 0x4800);
    DRV_WriteReg32(0xF0007404, 0x003B);
    DRV_WriteReg32(0xF0007400, 0x8800);

    udelay(500);
    while (DRV_Reg32(0xF0007404) & 0x8000);

    speed = ((DRV_Reg32(0xF0007410) * 26) / 2049) * 4;
    DRV_WriteReg32(0xF0007400, 0x8800);

    DRV_WriteReg32(0xF0001008, 0x0);

    printk("******************\n");
    printk("CA9 Speed = %d\n", speed);
    printk("******************\n");

    return 0;
}

void mt_power_management_init(void)
{
    struct proc_dir_entry *entry = NULL;
    struct proc_dir_entry *pm_init_dir = NULL;

    #if !defined (CONFIG_MT6577_FPGA)
    xlog_printk(ANDROID_LOG_INFO, "Power/PM_INIT", "Chip Version = 0x%x, Bus Frequency = %d KHz\n", get_chip_ver(), mt6577_get_bus_freq());

    chip_dep_init(); // set specific chip setting

    slp_mod_init(); // sleep controller init

    mt6577_clk_mgr_init(); // clock manager init, including clock gating init

    mt6577_pm_log_init(); // power management log init

    mt6577_dcm_init(); // dynamic clock management init

    pm_init_dir = proc_mkdir("pm_init", NULL);
    if (!pm_init_dir)
    {
        pr_err("[%s]: mkdir /proc/pm_init failed\n", __FUNCTION__);
    }
    else
    {
        entry = create_proc_entry("pmic_reg_dump", S_IRUGO, pm_init_dir);
        if (entry)
        {
            entry->read_proc = pmic_reg_dump_read;
        }

        entry = create_proc_entry("cpu_speed_dump", S_IRUGO, pm_init_dir);
        if (entry)
        {
            entry->read_proc = cpu_speed_dump_read;
        }
    }
    #endif
}
