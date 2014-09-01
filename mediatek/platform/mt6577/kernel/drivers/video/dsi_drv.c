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
#ifdef BUILD_UBOOT
#define ENABLE_DSI_INTERRUPT 0 

#include <asm/arch/disp_drv_platform.h>
#include <asm/arch/mt6577_gpio.h>

#else

#define ENABLE_DSI_INTERRUPT 1 

#include <linux/delay.h>
#include <disp_drv_log.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/spinlock.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_irq.h>
#include <mach/mt_gpio.h>
#include "mach/mt_clock_manager.h"
#include <mach/mt_boot.h>

#include "lcd_reg.h"
#include "lcd_drv.h"

#include "dpi_reg.h"
#include "dpi_drv.h"

#include "dsi_reg.h"
#include "dsi_drv.h"
#endif

#if ENABLE_DSI_INTERRUPT
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <mach/irqs.h>
#include "mtkfb.h"
static wait_queue_head_t _dsi_wait_queue;
static wait_queue_head_t _dsi_dcs_read_wait_queue;
#endif

/*
#define PLL_BASE			(0xF0060000)
#define DSI_PHY_BASE		(0xF0060B00)
#define DSI_BASE            	(0xF0140000)
*/

//#define PWR_OFF                 (APCONFIG_BASE + 0x0304)
//#define GRAPH1_PDN              (1 << 3)
#define G1_MEM_PDN              (APCONFIG_BASE + 0x0060)
#define G1_MEM_DSI              (1)
#ifdef BUILD_UBOOT
#define GRAPH1SYS_CG_SET        (GMC1_BASE + 0x320)
#define GRAPH1SYS_CG_CLR        (GMC1_BASE + 0x340)
#else
#define GRAPH1SYS_CG_SET        (MMSYS1_CONFIG_BASE + 0x320)
#define GRAPH1SYS_CG_CLR        (MMSYS1_CONFIG_BASE + 0x340)
#endif
#define GRAPH1SYS_CG_DSI        (1 << 28)

#if !(defined(CONFIG_MT6577_FPGA) || defined(BUILD_UBOOT))
#define DSI_MIPI_API
#endif

static PDSI_REGS const DSI_REG = (PDSI_REGS)(DSI_BASE);
static PDSI_PHY_REGS const DSI_PHY_REG = (PDSI_PHY_REGS)(MIPI_CONFG_BASE+0x800);
static PDSI_CMDQ_REGS const DSI_CMDQ_REG = (PDSI_CMDQ_REGS)(DSI_BASE+0x180);
static PLCD_REGS const LCD_REG = (PLCD_REGS)(LCD_BASE);
static PDPI_REGS const DPI_REG = (PDPI_REGS)(DPI_BASE);

static MIPITX_CFG0_REG mipitx_con0;
static MIPITX_CFG1_REG mipitx_con1;
static MIPITX_CFG3_REG mipitx_con3;
static MIPITX_CFG6_REG mipitx_con6;
static MIPITX_CFG8_REG mipitx_con8;
static MIPITX_CFG9_REG mipitx_con9;

typedef struct
{
	DSI_REGS regBackup;
	unsigned int bit_time_ns;
	unsigned int vfp_period_us;
	unsigned int vsa_vs_period_us;
	unsigned int vsa_hs_period_us;
	unsigned int vsa_ve_period_us;
	unsigned int vbp_period_us;
    void (*pIntCallback)(DISP_INTERRUPT_EVENTS);
} DSI_CONTEXT;

static bool s_isDsiPowerOn = FALSE;
bool is_needReConfig = true;
static DSI_CONTEXT _dsiContext;

#ifndef BUILD_UBOOT

DEFINE_SPINLOCK(g_handle_esd_lock);

static bool dsi_esd_recovery = false;
static bool dsi_noncont_clk_enabled = false;
static unsigned int dsi_noncont_clk_period = 1;
static bool dsi_int_te_enabled = false;
static unsigned int dsi_int_te_period = 1;
static unsigned int dsi_dpi_isr_count = 0;
unsigned long g_handle_esd_flag;

#endif

#ifdef BUILD_UBOOT
static long int get_current_time_us(void)
{
    return 0;       ///TODO: fix me
}
#else
static long int get_current_time_us(void)
{
    struct timeval t;
    do_gettimeofday(&t);
    return (t.tv_sec & 0xFFF) * 1000000 + t.tv_usec;
}
#endif
static void lcm_mdelay(UINT32 ms)
{
    udelay(1000 * ms);
}

#if ENABLE_DSI_INTERRUPT
static irqreturn_t _DSI_InterruptHandler(int irq, void *dev_id)
{   
    DSI_INT_STATUS_REG status = DSI_REG->DSI_INTSTA;

    if (status.RD_RDY)
    {        
        ///write clear RD_RDY interrupt
        DSI_REG->DSI_INTSTA.RD_RDY = 1;   

        /// write clear RD_RDY interrupt must be before DSI_RACK
        /// because CMD_DONE will raise after DSI_RACK, 
        /// so write clear RD_RDY after that will clear CMD_DONE too
        
		do
        {
            ///send read ACK
            DSI_REG->DSI_RACK.DSI_RACK = 1;
        } while(DSI_REG->DSI_STA.BUSY);

		wake_up_interruptible(&_dsi_dcs_read_wait_queue);
        if(_dsiContext.pIntCallback)
            _dsiContext.pIntCallback(DISP_DSI_READ_RDY_INT);            
    }

    if (status.CMD_DONE)
    {
        DSI_REG->DSI_INTSTA.CMD_DONE = 1;
    	// Go back to LP mode.
    	DSI_clk_HS_mode(0);
        wake_up_interruptible(&_dsi_wait_queue);
        if(_dsiContext.pIntCallback)
            _dsiContext.pIntCallback(DISP_DSI_CMD_DONE_INT);            
    }

    return IRQ_HANDLED;
}
#endif


static BOOL _IsEngineBusy(void)
{
	DSI_STATUS_REG status;

	status = DSI_REG->DSI_STA;
	
	if (status.BUSY || status.ERR_MSG)
		return TRUE;

	return FALSE;
}

#if 0
static BOOL _IsCMDQBusy(void)
{
	DSI_INT_STATUS_REG INT_status;

	INT_status=DSI_REG->DSI_INTSTA;

	if (!INT_status.CMD_DONE)
	{
		DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", " DSI CMDQ status BUSY !!\n");
	
		return TRUE;
	}

	return FALSE;
}
#endif

static void _WaitForEngineNotBusy(void)
{
    int timeOut;
#if ENABLE_DSI_INTERRUPT
    long int time;
    static const long WAIT_TIMEOUT = 2 * HZ;    // 2 sec
#endif

	if (DSI_REG->DSI_MODE_CTRL.MODE)
		return ;

	timeOut = 20;

#if ENABLE_DSI_INTERRUPT
	time = get_current_time_us();

    if (in_interrupt())
    {
        // perform busy waiting if in interrupt context
		while(_IsEngineBusy()) {
			msleep(1);
			if (--timeOut < 0)	{
						
				DISP_LOG_PRINT(ANDROID_LOG_ERROR, "DSI", " Wait for DSI engine not busy timeout!!!(Wait %d us)\n", get_current_time_us() - time);
				DSI_DumpRegisters();
				DSI_Reset();
				
				break;
			}
		}
    }
    else
    {
        while (_IsEngineBusy())
        {
            long ret = wait_event_interruptible_timeout(_dsi_wait_queue, 
                                                        !_IsEngineBusy(),
                                                        WAIT_TIMEOUT);
            if (0 == ret) {
                DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", " Wait for DSI engine not busy timeout, and reset DSI!!!\n");
				DSI_DumpRegisters();
				DSI_Reset();
            }
        }
    }
#else

    while(_IsEngineBusy()) {
		msleep(1);
		if (--timeOut < 0)	{
					
			DISP_LOG_PRINT(ANDROID_LOG_ERROR, "DSI", " Wait for DSI engine not busy timeout!!!\n");
			DSI_DumpRegisters();
			DSI_Reset();
			
			break;
		}
	}

#endif    
}

static void _BackupDSIRegisters(void)
{
    DSI_REGS *regs = &(_dsiContext.regBackup);
	
    //memcpy((void*)&(_dsiContext.regBackup), (void*)DSI_BASE, sizeof(DSI_REGS));

    OUTREG32(&regs->DSI_INTEN, AS_UINT32(&DSI_REG->DSI_INTEN));
    OUTREG32(&regs->DSI_MODE_CTRL, AS_UINT32(&DSI_REG->DSI_MODE_CTRL));
    OUTREG32(&regs->DSI_TXRX_CTRL, AS_UINT32(&DSI_REG->DSI_TXRX_CTRL));
    OUTREG32(&regs->DSI_PSCTRL, AS_UINT32(&DSI_REG->DSI_PSCTRL));

    OUTREG32(&regs->DSI_VSA_NL, AS_UINT32(&DSI_REG->DSI_VSA_NL));		
    OUTREG32(&regs->DSI_VBP_NL, AS_UINT32(&DSI_REG->DSI_VBP_NL));		
    OUTREG32(&regs->DSI_VFP_NL, AS_UINT32(&DSI_REG->DSI_VFP_NL));		
    OUTREG32(&regs->DSI_VACT_NL, AS_UINT32(&DSI_REG->DSI_VACT_NL));		

    OUTREG32(&regs->DSI_LINE_NB, AS_UINT32(&DSI_REG->DSI_LINE_NB));		
    OUTREG32(&regs->DSI_HSA_NB, AS_UINT32(&DSI_REG->DSI_HSA_NB));		
    OUTREG32(&regs->DSI_HBP_NB, AS_UINT32(&DSI_REG->DSI_HBP_NB));		
    OUTREG32(&regs->DSI_HFP_NB, AS_UINT32(&DSI_REG->DSI_HFP_NB));		

    OUTREG32(&regs->DSI_RGB_NB, AS_UINT32(&DSI_REG->DSI_RGB_NB));		
    OUTREG32(&regs->DSI_HSA_WC, AS_UINT32(&DSI_REG->DSI_HSA_WC));		
    OUTREG32(&regs->DSI_HBP_WC, AS_UINT32(&DSI_REG->DSI_HBP_WC));		
    OUTREG32(&regs->DSI_HFP_WC, AS_UINT32(&DSI_REG->DSI_HFP_WC));		
	
    OUTREG32(&regs->DSI_MEM_CONTI, AS_UINT32(&DSI_REG->DSI_MEM_CONTI));

    OUTREG32(&regs->DSI_PHY_TIMECON0, AS_UINT32(&DSI_REG->DSI_PHY_TIMECON0));
    OUTREG32(&regs->DSI_PHY_TIMECON1, AS_UINT32(&DSI_REG->DSI_PHY_TIMECON1));
    OUTREG32(&regs->DSI_PHY_TIMECON2, AS_UINT32(&DSI_REG->DSI_PHY_TIMECON2));
    OUTREG32(&regs->DSI_PHY_TIMECON3, AS_UINT32(&DSI_REG->DSI_PHY_TIMECON3));
	
}

static void _RestoreDSIRegisters(void)
{
    DSI_REGS *regs = &(_dsiContext.regBackup);

    OUTREG32(&DSI_REG->DSI_INTEN, AS_UINT32(&regs->DSI_INTEN));	
    OUTREG32(&DSI_REG->DSI_MODE_CTRL, AS_UINT32(&regs->DSI_MODE_CTRL));	
    OUTREG32(&DSI_REG->DSI_TXRX_CTRL, AS_UINT32(&regs->DSI_TXRX_CTRL));	
    OUTREG32(&DSI_REG->DSI_PSCTRL, AS_UINT32(&regs->DSI_PSCTRL));	

    OUTREG32(&DSI_REG->DSI_VSA_NL, AS_UINT32(&regs->DSI_VSA_NL));		
    OUTREG32(&DSI_REG->DSI_VBP_NL, AS_UINT32(&regs->DSI_VBP_NL));		
    OUTREG32(&DSI_REG->DSI_VFP_NL, AS_UINT32(&regs->DSI_VFP_NL));		
    OUTREG32(&DSI_REG->DSI_VACT_NL, AS_UINT32(&regs->DSI_VACT_NL));		

    OUTREG32(&DSI_REG->DSI_LINE_NB, AS_UINT32(&regs->DSI_LINE_NB));		
    OUTREG32(&DSI_REG->DSI_HSA_NB, AS_UINT32(&regs->DSI_HSA_NB));		
    OUTREG32(&DSI_REG->DSI_HBP_NB, AS_UINT32(&regs->DSI_HBP_NB));		
    OUTREG32(&DSI_REG->DSI_HFP_NB, AS_UINT32(&regs->DSI_HFP_NB));		

    OUTREG32(&DSI_REG->DSI_RGB_NB, AS_UINT32(&regs->DSI_RGB_NB));		
    OUTREG32(&DSI_REG->DSI_HSA_WC, AS_UINT32(&regs->DSI_HSA_WC));		
    OUTREG32(&DSI_REG->DSI_HBP_WC, AS_UINT32(&regs->DSI_HBP_WC));		
    OUTREG32(&DSI_REG->DSI_HFP_WC, AS_UINT32(&regs->DSI_HFP_WC));		

    OUTREG32(&DSI_REG->DSI_MEM_CONTI, AS_UINT32(&regs->DSI_MEM_CONTI));		

    OUTREG32(&DSI_REG->DSI_PHY_TIMECON0, AS_UINT32(&regs->DSI_PHY_TIMECON0));		
    OUTREG32(&DSI_REG->DSI_PHY_TIMECON1, AS_UINT32(&regs->DSI_PHY_TIMECON1));
    OUTREG32(&DSI_REG->DSI_PHY_TIMECON2, AS_UINT32(&regs->DSI_PHY_TIMECON2));		
    OUTREG32(&DSI_REG->DSI_PHY_TIMECON3, AS_UINT32(&regs->DSI_PHY_TIMECON3));		
	DSI_Reset();
}

static void _ResetBackupedDSIRegisterValues(void)
{
    DSI_REGS *regs = &_dsiContext.regBackup;
    memset((void*)regs, 0, sizeof(DSI_REGS));
}

DSI_STATUS DSI_Init(BOOL isDsiPoweredOn)
{
    DSI_STATUS ret = DSI_STATUS_OK;

    memset(&_dsiContext, 0, sizeof(_dsiContext));

    if (isDsiPoweredOn) {
        _BackupDSIRegisters();
		is_needReConfig = false;
    } else {
        _ResetBackupedDSIRegisterValues();
		is_needReConfig = true;
    }

	ret = DSI_PowerOn();

	OUTREG32(&DSI_REG->DSI_MEM_CONTI, (DSI_WMEM_CONTI<<16)|(DSI_RMEM_CONTI));	

    ASSERT(ret == DSI_STATUS_OK);

    mipitx_con0=DSI_PHY_REG->MIPITX_CON0;
    mipitx_con1=DSI_PHY_REG->MIPITX_CON1;
    mipitx_con3=DSI_PHY_REG->MIPITX_CON3;
    mipitx_con6=DSI_PHY_REG->MIPITX_CON6;
    mipitx_con8=DSI_PHY_REG->MIPITX_CON8;
    mipitx_con9=DSI_PHY_REG->MIPITX_CON9;

#if ENABLE_DSI_INTERRUPT

    init_waitqueue_head(&_dsi_wait_queue);
    init_waitqueue_head(&_dsi_dcs_read_wait_queue);	

	// Clear INT status.
	DSI_REG->DSI_INTSTA.RD_RDY= 1;
	DSI_REG->DSI_INTSTA.CMD_DONE = 1;

    if (request_irq(MT_DSI_IRQ_ID,
        _DSI_InterruptHandler, IRQF_TRIGGER_LOW, MTKFB_DRIVER, NULL) < 0)
    {
        DISP_LOG_PRINT(ANDROID_LOG_ERROR, "DSI", "fail to request DSI irq\n"); 
        return DSI_STATUS_ERROR;
    }

	//mt65xx_irq_unmask(MT_DSI_IRQ_ID);
    DSI_REG->DSI_INTEN.CMD_DONE=1;
    DSI_REG->DSI_INTEN.RD_RDY=1;
#endif
    
    return DSI_STATUS_OK;
}


DSI_STATUS DSI_Deinit(void)
{
    DSI_STATUS ret = DSI_PowerOff();

    ASSERT(ret == DSI_STATUS_OK);

    return DSI_STATUS_OK;
}

#ifdef BUILD_UBOOT
DSI_STATUS DSI_PowerOn(void)
{

    if (!s_isDsiPowerOn)
    {
#if 0   // FIXME
        BOOL ret = hwEnableClock(MT65XX_PDN_MM_DSI, "DSI");
        ASSERT(ret);
#else
		MASKREG32(0xC2080028, 0x00000020, 0x00000020);
		printf("[DISP] - uboot - DSI_PowerOn. 0x%8x,0x%8x,0x%8x\n", INREG32(0xC2080000), INREG32(0xC2080004), INREG32(0xC2080008));

#endif        
        _RestoreDSIRegisters();
	//_WaitForEngineNotBusy();		
        s_isDsiPowerOn = TRUE;
    }

    return DSI_STATUS_OK;
}


DSI_STATUS DSI_PowerOff(void)
{

    if (s_isDsiPowerOn)
    {
        BOOL ret = TRUE;
        //_WaitForEngineNotBusy();
        _BackupDSIRegisters();
#if 0   // FIXME
        ret = hwDisableClock(MT65XX_PDN_MM_DSI, "DSI");
        ASSERT(ret);
#else
        MASKREG32(0xC2080018, 0x00000020, 0x00000020);
		printf("[DISP] - uboot - DSI_PowerOff. 0x%8x,0x%8x,0x%8x\n", INREG32(0xC2080000), INREG32(0xC2080004), INREG32(0xC2080008));
#endif        
        s_isDsiPowerOn = FALSE;
    }
    
    return DSI_STATUS_OK;
}

#else
DSI_STATUS DSI_PowerOn(void)
{
#ifndef CONFIG_MT6577_FPGA
    if (!s_isDsiPowerOn)
    {
#if 1   // FIXME
        //BOOL ret = hwEnableClock(MT65XX_PDN_MM_DSI, "DSI");
        BOOL ret = enable_clock(MT65XX_PDN_MM_DSI, "DSI");
        ASSERT(!ret);
#endif        
		if(is_needReConfig)
        	_RestoreDSIRegisters();
        s_isDsiPowerOn = TRUE;
    }
#endif
    return DSI_STATUS_OK;
}


DSI_STATUS DSI_PowerOff(void)
{
#ifndef CONFIG_MT6577_FPGA
    if (s_isDsiPowerOn)
    {
        BOOL ret = TRUE;
        _WaitForEngineNotBusy();
        _BackupDSIRegisters();
#if 1   // FIXME
        //ret = hwDisableClock(MT65XX_PDN_MM_DSI, "DSI");
        ret = disable_clock(MT65XX_PDN_MM_DSI, "DSI");
        ASSERT(!ret);
#endif        
        s_isDsiPowerOn = FALSE;
    }
#endif
    return DSI_STATUS_OK;
}
#endif

DSI_STATUS DSI_WaitForNotBusy(void)
{
    _WaitForEngineNotBusy();

    return DSI_STATUS_OK;
}


DSI_STATUS DSI_EnableClk(void)
{
	_WaitForEngineNotBusy();

	DSI_REG->DSI_START.DSI_START=0;
	DSI_REG->DSI_START.DSI_START=1;

    return DSI_STATUS_OK;
}


DSI_STATUS DSI_DisableClk(void)
{
	DSI_REG->DSI_START.DSI_START=0;

    return DSI_STATUS_OK;
}


DSI_STATUS DSI_Reset(void)
{
	DSI_REG->DSI_COM_CTRL.DSI_RESET = 1;
	lcm_mdelay(5);
	DSI_REG->DSI_COM_CTRL.DSI_RESET = 0;

    return DSI_STATUS_OK;
}


DSI_STATUS DSI_SetMode(unsigned int mode)
{

	DSI_REG->DSI_MODE_CTRL.MODE = mode;

	return DSI_STATUS_OK;
}

DSI_STATUS DSI_EnableInterrupt(DISP_INTERRUPT_EVENTS eventID)
{
#if ENABLE_DSI_INTERRUPT
    switch(eventID)
    {
        case DISP_DSI_READ_RDY_INT:
            DSI_REG->DSI_INTEN.RD_RDY = 1;
            break;
        case DISP_DSI_CMD_DONE_INT:
            DSI_REG->DSI_INTEN.CMD_DONE = 1;
            break;
        default:
            return DSI_STATUS_ERROR;
    }

    return DSI_STATUS_OK;
#else
    ///TODO: warning log here
    return DSI_STATUS_ERROR;
#endif
}


DSI_STATUS DSI_SetInterruptCallback(void (*pCB)(DISP_INTERRUPT_EVENTS))
{
    _dsiContext.pIntCallback = pCB;

    return DSI_STATUS_OK;
}

DSI_STATUS DSI_handle_TE(void)
{

	unsigned int data_array;

	//data_array=0x00351504;
	//DSI_set_cmdq(&data_array, 1, 1);

	//lcm_mdelay(10);

	// RACT	
	//data_array=1;
	//OUTREG32(&DSI_REG->DSI_RACK, data_array);

	// TE + BTA
	data_array=0x24;
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] DSI_handle_TE TE + BTA !! \n");
	OUTREG32(&DSI_CMDQ_REG->data0, data_array);

	//DSI_CMDQ_REG->data0.byte0=0x24;
	//DSI_CMDQ_REG->data0.byte1=0;
	//DSI_CMDQ_REG->data0.byte2=0;
	//DSI_CMDQ_REG->data0.byte3=0;

	DSI_REG->DSI_CMDQ_SIZE.CMDQ_SIZE=1;

	DSI_REG->DSI_START.DSI_START=0;
	DSI_REG->DSI_START.DSI_START=1;

	// wait TE Trigger status
//	do
//	{
		lcm_mdelay(10);

		data_array=INREG32(&DSI_REG->DSI_INTSTA);
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] DSI INT state : %x !! \n", data_array);
	
		data_array=INREG32(&DSI_REG->DSI_TRIG_STA);
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] DSI TRIG TE status check : %x !! \n", data_array);
//	} while(!(data_array&0x4));

	// RACT	
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] DSI Set RACT !! \n");
	data_array=1;
	OUTREG32(&DSI_REG->DSI_RACK, data_array);

	return DSI_STATUS_OK;

}

void DSI_Config_VDO_Timing(LCM_PARAMS *lcm_params)
{
	unsigned int line_byte;
	unsigned int horizontal_sync_active_byte;
	unsigned int horizontal_backporch_byte;
	unsigned int horizontal_frontporch_byte;
	unsigned int rgb_byte;
	unsigned int dsiTmpBufBpp;

	#define LINE_PERIOD_US				(8 * line_byte * _dsiContext.bit_time_ns / 1000)

	if(lcm_params->dsi.data_format.format == LCM_DSI_FORMAT_RGB565)
		dsiTmpBufBpp = 2;
	else
		dsiTmpBufBpp = 3;

	OUTREG32(&DSI_REG->DSI_VSA_NL, lcm_params->dsi.vertical_sync_active);
	OUTREG32(&DSI_REG->DSI_VBP_NL, lcm_params->dsi.vertical_backporch);
	OUTREG32(&DSI_REG->DSI_VFP_NL, lcm_params->dsi.vertical_frontporch);
	OUTREG32(&DSI_REG->DSI_VACT_NL, lcm_params->dsi.vertical_active_line);

	line_byte							=	(lcm_params->dsi.horizontal_sync_active \
											+ lcm_params->dsi.horizontal_backporch \
											+ lcm_params->dsi.horizontal_frontporch \
											+ lcm_params->dsi.horizontal_active_pixel) * dsiTmpBufBpp;
	
	horizontal_sync_active_byte 		=	(lcm_params->dsi.horizontal_sync_active * dsiTmpBufBpp - 4);

	if (lcm_params->dsi.mode == SYNC_EVENT_VDO_MODE)
		horizontal_backporch_byte		=	((lcm_params->dsi.horizontal_backporch + lcm_params->dsi.horizontal_sync_active)* dsiTmpBufBpp - 4);
	else
		horizontal_backporch_byte		=	(lcm_params->dsi.horizontal_backporch * dsiTmpBufBpp - 4);
	
	horizontal_frontporch_byte			=	(lcm_params->dsi.horizontal_frontporch * dsiTmpBufBpp - 6);
	rgb_byte							=	(lcm_params->dsi.horizontal_active_pixel * dsiTmpBufBpp + 6);					
	
	OUTREG32(&DSI_REG->DSI_LINE_NB, line_byte);
	OUTREG32(&DSI_REG->DSI_HSA_NB, horizontal_sync_active_byte);
	OUTREG32(&DSI_REG->DSI_HBP_NB, horizontal_backporch_byte);
	OUTREG32(&DSI_REG->DSI_HFP_NB, horizontal_frontporch_byte);
	OUTREG32(&DSI_REG->DSI_RGB_NB, rgb_byte);

	OUTREG32(&DSI_REG->DSI_HSA_WC, (horizontal_sync_active_byte-6));
	OUTREG32(&DSI_REG->DSI_HBP_WC, (horizontal_backporch_byte-6));
	OUTREG32(&DSI_REG->DSI_HFP_WC, (horizontal_frontporch_byte-6)); 

	_dsiContext.vfp_period_us 		= LINE_PERIOD_US * lcm_params->dsi.vertical_frontporch / 1000;
	_dsiContext.vsa_vs_period_us	= LINE_PERIOD_US * 1 / 1000;
	_dsiContext.vsa_hs_period_us	= LINE_PERIOD_US * (lcm_params->dsi.vertical_sync_active - 2) / 1000;
	_dsiContext.vsa_ve_period_us	= LINE_PERIOD_US * 1 / 1000;
	_dsiContext.vbp_period_us		= LINE_PERIOD_US * lcm_params->dsi.vertical_backporch / 1000;

	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - video timing, mode = %d \n", lcm_params->dsi.mode);
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - VSA : %d %d(us)\n", DSI_REG->DSI_VSA_NL, (_dsiContext.vsa_vs_period_us+_dsiContext.vsa_hs_period_us+_dsiContext.vsa_ve_period_us));
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - VBP : %d %d(us)\n", DSI_REG->DSI_VBP_NL, _dsiContext.vbp_period_us);
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - VFP : %d %d(us)\n", DSI_REG->DSI_VFP_NL, _dsiContext.vfp_period_us);
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - VACT: %d \n", DSI_REG->DSI_VACT_NL);

	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - HLB : %d \n", DSI_REG->DSI_LINE_NB);
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - HSA : %d \n", DSI_REG->DSI_HSA_NB);
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - HBP : %d \n", DSI_REG->DSI_HBP_NB);
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - HFP : %d \n", DSI_REG->DSI_HFP_NB);
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] kernel - RGB : %d \n", DSI_REG->DSI_RGB_NB);

}


void DSI_PHY_clk_setting(unsigned int div1, unsigned int div2, unsigned int lane_no)
{

	unsigned int tx_ldo_en=0x1;
	unsigned int tx_ldo_vocal=0x4;
	unsigned int rg_pll_pr_div=0;
	unsigned int rg_pll_fb_div2=0;
	unsigned int po_div2_en=0x00;

	_dsiContext.bit_time_ns = 1000 * (1000 * (div2 ? div2*0x02 : 0x1)) / (26 * (div1+0x01)) / lane_no;

	if (div1>0x1e) //vco high speed mode
	{
		rg_pll_pr_div=1;
		rg_pll_fb_div2=1;
	}
	else
	{
		rg_pll_pr_div=0;
		rg_pll_fb_div2=0;
	}
	
	//mipi TX/PLL enable
	mipitx_con0.PLL_EN=1;
	mipitx_con0.PLL_CLKR_EN=1;
	// Leave it to be sync. with MIPITX I/O.
	//mipitx_con0.RG_LNT_LPTX_BIAS_EN=1;
	mipitx_con0.RG_LNT_LPTX_BIAS_EN=0;
	mipitx_con0.RG_LNT_HSTX_EDGE_SEL=0;
	mipitx_con0.RG_DSI_PHY_CK_PSEL=0;
#if defined(MTK_HDMI_SUPPORT)
	mipitx_con0.RG_DPI_EN=1;
#else
	mipitx_con0.RG_DPI_EN=0;
#endif
	mipitx_con0.RG_LNT_HSTX_BIAS_EN=1;
	mipitx_con0.RG_LPTX_SWBYPASS=0;
	mipitx_con0.RG_PRBS_EN=0;
#if defined(MTK_HDMI_SUPPORT)
	mipitx_con0.RG_DPI_CKSEL=2;
#else
	mipitx_con0.RG_DPI_CKSEL=3;
#endif
	mipitx_con0.RG_CLK_SEL=0;
	
	// mipitx pll setting
	mipitx_con1.RG_PLL_DIV1=(div1&0x3F);
	mipitx_con1.RG_PLL_DIV2=(div2&0xF);
	mipitx_con1.RG_DSI_CK_SEL=(lane_no-1);
	mipitx_con1.RG_DSI_CK_DSEL=0;
	
	//mipitx pll pdiv2/prdiv
	mipitx_con3.PG_PLL_PRDIV=rg_pll_pr_div;
	mipitx_con3.RG_PLL_VCOCAL_CKCTL=0;
	mipitx_con3.RG_PLL_FBSEL=rg_pll_fb_div2;
	mipitx_con3.RG_PLL_CDIV=0;
	mipitx_con3.RG_PLL_ACCEN=0;
	//mipitx_con3.RG_PLL_AUTOK_LOAD=0;
	//mipitx_con3.RG_PLL_LOAD_RSTB=0;
	//mipitx_con3.RG_PLL_AUTOK_EN=0;
	mipitx_con3.RG_PLL_PODIV2=po_div2_en;

	mipitx_con8.RG_LNTC_HS_CZ=8;
	mipitx_con8.RG_LNT0_HS_CZ=8;
	mipitx_con8.RG_LNT1_HS_CZ=8;
	mipitx_con8.RG_LNT_CIRE=2;

	//unsigned int mipitx_con9=(tx_ldo_vocal<<0x0b)|(tx_ldo_en<<0x03);
	mipitx_con9.RG_TXLDO_FBCAL=0;
	mipitx_con9.RG_TXLDO_EN=tx_ldo_en;
	mipitx_con9.RG_TXLDO_IBCAL=0;
	mipitx_con9.RG_TXLDO_OCCAL=0;
	mipitx_con9.RG_TXLDO_PDDOS_EN=0;
	mipitx_con9.RG_TXLDO_VOCAL=tx_ldo_vocal;
		
	DSI_PHY_REG->MIPITX_CON0=mipitx_con0;
	DSI_PHY_REG->MIPITX_CON1=mipitx_con1;
	DSI_PHY_REG->MIPITX_CON3=mipitx_con3;	
	DSI_PHY_REG->MIPITX_CON6=mipitx_con6;
	DSI_PHY_REG->MIPITX_CON8=mipitx_con8;
	DSI_PHY_REG->MIPITX_CON9=mipitx_con9;

	// wait pll stable
	mdelay(1);
}

#ifdef BUILD_UBOOT
void DSI_PHY_clk_switch(bool on)
{
	MIPITX_CFG0_REG mipitx_con0=DSI_PHY_REG->MIPITX_CON0;

	if(on)
		mipitx_con0.PLL_EN=1;
	else
		mipitx_con0.PLL_EN=0;

	DSI_PHY_REG->MIPITX_CON0=mipitx_con0;	
}
#else
void DSI_PHY_clk_switch(bool on)
{
	if(on)
	{
#ifdef DSI_MIPI_API
		enable_mipi(MT65XX_MIPI_TX, "DSI");
#endif
		mipitx_con0.PLL_EN=1;
		DSI_PHY_REG->MIPITX_CON0=mipitx_con0;	
	}
	else
	{
		mipitx_con0=DSI_PHY_REG->MIPITX_CON0;
		mipitx_con1=DSI_PHY_REG->MIPITX_CON1;
		mipitx_con3=DSI_PHY_REG->MIPITX_CON3;
		mipitx_con6=DSI_PHY_REG->MIPITX_CON6;
		mipitx_con8=DSI_PHY_REG->MIPITX_CON8;
		mipitx_con9=DSI_PHY_REG->MIPITX_CON9;
#ifdef DSI_MIPI_API 
		disable_mipi(MT65XX_MIPI_TX, "DSI");
#else
		OUTREG32(&DSI_PHY_REG->MIPITX_CON0, 0);
		OUTREG32(&DSI_PHY_REG->MIPITX_CON6, 0);
		OUTREG32(&DSI_PHY_REG->MIPITX_CON9, 0);
#endif
	}	
}
#endif

void DSI_PHY_TIMCONFIG(LCM_PARAMS *lcm_params)
{
	DSI_PHY_TIMCON0_REG timcon0;
	DSI_PHY_TIMCON1_REG timcon1;	
	DSI_PHY_TIMCON2_REG timcon2;
	DSI_PHY_TIMCON3_REG timcon3;
	unsigned int div1 = lcm_params->dsi.pll_div1;
	unsigned int div2 = lcm_params->dsi.pll_div2;
	unsigned int lane_no = lcm_params->dsi.LANE_NUM;

	unsigned int div2_real=div2 ? div2*0x02 : 0x1;
	unsigned int cycle_time = (8 * 1000 * div2_real)/ (26 * (div1+0x01));
	unsigned int ui = (1000 * div2_real)/ (26 * (div1+0x01)) + 1;
	unsigned int hs_trail_m, hs_trail_n;

	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] - kernel - DSI_PHY_TIMCONFIG, Cycle Time = %d(ns), Unit Interval = %d(ns). div1 = %d, div2 = %d, lane# = %d \n", cycle_time, ui, div1, div2, lane_no); 		

	#define NS_TO_CYCLE(n, c)	((n) / c + (( (n) % c) ? 1 : 0))

	hs_trail_m=lane_no;
	hs_trail_n= (lcm_params->dsi.HS_TRAIL == 0) ? NS_TO_CYCLE(((lane_no * 4 * ui) + 60), cycle_time) : lcm_params->dsi.HS_TRAIL;

	// +6 is recommended from designer becauase of HW latency
	timcon0.HS_TRAIL	= ((hs_trail_m > hs_trail_n) ? hs_trail_m : hs_trail_n) + 6;
	timcon0.HS_ZERO 	= (lcm_params->dsi.HS_ZERO == 0) ? NS_TO_CYCLE((105 + 6 * ui), cycle_time) : lcm_params->dsi.HS_ZERO;
	timcon0.HS_ZERO -= 5;
	if(timcon0.HS_ZERO < 1)
		timcon0.HS_ZERO = 1;

	timcon0.HS_PRPR 	= (lcm_params->dsi.HS_PRPR == 0) ? NS_TO_CYCLE((40 + 4 * ui + 12), cycle_time) : lcm_params->dsi.HS_PRPR;
	// HS_PRPR can't be 1.
	if (timcon0.HS_PRPR < 2)
		timcon0.HS_PRPR = 2;
	timcon0.LPX 		= (lcm_params->dsi.LPX == 0) ? NS_TO_CYCLE(50, cycle_time) : lcm_params->dsi.LPX;

	timcon1.TA_SACK 	= (lcm_params->dsi.TA_SACK == 0) ? 1 : lcm_params->dsi.TA_SACK;
	timcon1.TA_GET 		= (lcm_params->dsi.TA_GET == 0) ? (5 * NS_TO_CYCLE(50, cycle_time)) : lcm_params->dsi.TA_GET;
	timcon1.TA_SURE 	= (lcm_params->dsi.TA_SURE == 0) ? (3 * NS_TO_CYCLE(50, cycle_time) / 2) : lcm_params->dsi.TA_SURE;
	timcon1.TA_GO 		= (lcm_params->dsi.TA_GO == 0) ? (4 * NS_TO_CYCLE(50, cycle_time)) : lcm_params->dsi.TA_GO;


	timcon2.CLK_TRAIL 	= (lcm_params->dsi.CLK_TRAIL == 0) ? NS_TO_CYCLE(60, cycle_time) : lcm_params->dsi.CLK_TRAIL;
	// CLK_TRAIL can't be 1.
	if (timcon2.CLK_TRAIL < 2)
		timcon2.CLK_TRAIL = 2;
	timcon2.CLK_ZERO 	= (lcm_params->dsi.CLK_ZERO == 0) ? NS_TO_CYCLE((300 - 38), cycle_time) : lcm_params->dsi.CLK_ZERO;
	timcon2.LPX_WAIT 	= (lcm_params->dsi.LPX_WAIT == 0) ? 1 : lcm_params->dsi.LPX_WAIT;
	timcon2.CONT_DET 	= lcm_params->dsi.CONT_DET;

	timcon3.CLK_HS_PRPR	= (lcm_params->dsi.CLK_HS_PRPR == 0) ? NS_TO_CYCLE((38 + 95) / 2, cycle_time) : lcm_params->dsi.CLK_HS_PRPR;

	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] - kernel - DSI_PHY_TIMCONFIG, HS_TRAIL = %d, HS_ZERO = %d, HS_PRPR = %d, LPX = %d, TA_SACK = %d, TA_GET = %d, TA_SURE = %d, TA_GO = %d, CLK_TRAIL = %d, CLK_ZERO = %d, CLK_HS_PRPR = %d \n", \
			timcon0.HS_TRAIL, timcon0.HS_ZERO, timcon0.HS_PRPR, timcon0.LPX, timcon1.TA_SACK, timcon1.TA_GET, timcon1.TA_SURE, timcon1.TA_GO, timcon2.CLK_TRAIL, timcon2.CLK_ZERO, timcon3.CLK_HS_PRPR);		

	DSI_REG->DSI_PHY_TIMECON0=timcon0;
	DSI_REG->DSI_PHY_TIMECON1=timcon1;
	DSI_REG->DSI_PHY_TIMECON2=timcon2;
	DSI_REG->DSI_PHY_TIMECON3=timcon3;
	
}



void DSI_clk_ULP_mode(bool enter)
{
	DSI_PHY_LCCON_REG tmp_reg1;
	//DSI_PHY_REG_ANACON0	tmp_reg2;

	tmp_reg1=DSI_REG->DSI_PHY_LCCON;
	//tmp_reg2=DSI_PHY_REG->ANACON0;

	if(enter) {

		tmp_reg1.LC_HS_TX_EN=0;
		OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
		tmp_reg1.LC_ULPM_EN=1;
		OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
		//tmp_reg2.PLL_EN=0;
		//OUTREG32(&DSI_PHY_REG->ANACON0, AS_UINT32(&tmp_reg2));

	}
	else {

		//tmp_reg2.PLL_EN=1;
		//OUTREG32(&DSI_PHY_REG->ANACON0, AS_UINT32(&tmp_reg2));
		lcm_mdelay(1);
		tmp_reg1.LC_ULPM_EN=0;
		OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
		tmp_reg1.LC_WAKEUP_EN=1;
		OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
		tmp_reg1.LC_WAKEUP_EN=0;
		OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);

	}
}


void DSI_clk_HS_mode(bool enter)
{
	DSI_PHY_LCCON_REG tmp_reg1 = DSI_REG->DSI_PHY_LCCON;

	if(enter && !DSI_clk_HS_state()) {
		tmp_reg1.LC_HS_TX_EN=1;
		OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&tmp_reg1));
		//lcm_mdelay(1);
	}
	else if (!enter && DSI_clk_HS_state()) {
		tmp_reg1.LC_HS_TX_EN=0;
		OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&tmp_reg1));
		//lcm_mdelay(1);

	}
}	


bool DSI_clk_HS_state(void)
{
	return DSI_REG->DSI_PHY_LCCON.LC_HS_TX_EN ? TRUE : FALSE;
}


void DSI_lane0_ULP_mode(bool enter)
{
	DSI_PHY_LD0CON_REG tmp_reg1;

	tmp_reg1=DSI_REG->DSI_PHY_LD0CON;

	if(enter) {
		// suspend
		tmp_reg1.L0_HS_TX_EN=0;
		OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
		tmp_reg1.L0_ULPM_EN=1;
		OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
	}
	else {
		// resume
		tmp_reg1.L0_ULPM_EN=0;
		OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
		tmp_reg1.L0_WAKEUP_EN=1;
		OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
		tmp_reg1.L0_WAKEUP_EN=0;
		OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&tmp_reg1));
		lcm_mdelay(1);
	}
}

#ifndef BUILD_UBOOT

// called by DPI ISR
void DSI_handle_esd_recovery(void)
{
	long int dsi_current_time;
	unsigned int state;
	
	dsi_dpi_isr_count++;

	if (dsi_noncont_clk_enabled && (dsi_dpi_isr_count % dsi_noncont_clk_period)==0) {
		//DSI_handle_noncont_clk();
		{		
			if (!DSI_REG->DSI_MODE_CTRL.MODE)
				return ;
			
			dsi_current_time = get_current_time_us();

			state = DSI_REG->DSI_STATE_DBG3.TCON_STATE;	
		
			switch(state)
			{
				case DSI_VDO_VSA_VS_STATE:
					while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VSA_HS_STATE)
					{
						if(get_current_time_us() - dsi_current_time > _dsiContext.vsa_vs_period_us)
						{
							xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VSA_HS_STATE, _dsiContext.vsa_vs_period_us);
							return ;
						}			
					}
					break;
			
				case DSI_VDO_VSA_HS_STATE:
					while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VSA_VE_STATE)
					{
						if(get_current_time_us() - dsi_current_time > _dsiContext.vsa_hs_period_us)
						{
							xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VSA_VE_STATE, _dsiContext.vsa_hs_period_us);
							return ;
						}			
					}
					break;
			
				case DSI_VDO_VSA_VE_STATE:
					while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VBP_STATE)
					{
						if(get_current_time_us() - dsi_current_time > _dsiContext.vsa_ve_period_us)
						{
							xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VBP_STATE, _dsiContext.vsa_ve_period_us);
							return ;
						}			
					}
					break;
			
				case DSI_VDO_VBP_STATE:
					xlog_printk(ANDROID_LOG_WARN, "DSI", "Can't do clock switch in DSI_VDO_VBP_STATE !!!\n");
					break;
			
				case DSI_VDO_VACT_STATE:
					while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VFP_STATE)
					{
						if(get_current_time_us() - dsi_current_time > _dsiContext.vfp_period_us )
						{
							xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VFP_STATE, _dsiContext.vfp_period_us );
							return ;
						}			
					}
					break;
			
				case DSI_VDO_VFP_STATE:
					while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VSA_VS_STATE)
					{
						if(get_current_time_us() - dsi_current_time > _dsiContext.vfp_period_us )
						{
							xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VSA_VS_STATE, _dsiContext.vfp_period_us );
							return ;
						}			
					}
					break;
			
				default :
					xlog_printk(ANDROID_LOG_ERROR, "DSI", "invalid state = %x \n", state);
					return ;
			}
			
			// Clock switch HS->LP->HS
			if(DSI_REG->DSI_STATE_DBG0.CTL_STATE_0 == 0x1)
			{
				spin_lock_irqsave(&g_handle_esd_lock, g_handle_esd_flag);
				
				DSI_clk_HS_mode(0);
				DSI_clk_HS_mode(1);

				spin_unlock_irqrestore(&g_handle_esd_lock, g_handle_esd_flag);

			}		

			return ;
		}		
	}

	if (dsi_int_te_enabled && (dsi_dpi_isr_count % dsi_int_te_period)==0) {
		spin_lock_irqsave(&g_handle_esd_lock, g_handle_esd_flag);

		//mt65xx_irq_mask(MT6575_DSI_IRQ_ID);
		disable_irq(MT_DSI_IRQ_ID);

		dsi_esd_recovery = DSI_handle_int_TE();

		//mt65xx_irq_unmask(MT6575_DSI_IRQ_ID);
		enable_irq(MT_DSI_IRQ_ID);

		spin_unlock_irqrestore(&g_handle_esd_lock, g_handle_esd_flag);
	}	   
	
}


// called by "esd_recovery_kthread"
bool DSI_esd_check(void)
{
	bool result = false;

	if(dsi_esd_recovery)
		result = true;
	else
		result = false;

	dsi_esd_recovery = false;

	return result;
}


void DSI_set_int_TE(bool enable, unsigned int period)
{
	dsi_int_te_enabled = enable;
	if(period<1)
		period = 1;
	dsi_int_te_period = period;
	dsi_dpi_isr_count = 0;
}


// called by DPI ISR.
bool DSI_handle_int_TE(void)
{
	DSI_T0_INS t0;
	long int dsi_current_time;

	if (!DSI_REG->DSI_MODE_CTRL.MODE)
		return false;

	dsi_current_time = get_current_time_us();

	if(DSI_REG->DSI_STATE_DBG3.TCON_STATE == DSI_VDO_VFP_STATE)
	{
		udelay(_dsiContext.vfp_period_us / 2);

		if ((DSI_REG->DSI_STATE_DBG3.TCON_STATE == DSI_VDO_VFP_STATE) && DSI_REG->DSI_STATE_DBG0.CTL_STATE_0 == 0x1)
		{			
			// Can't do int. TE check while INUSE FB number is not 0 because later disable/enable DPI will set INUSE FB to number 0.
			if(DPI_REG->STATUS.FB_INUSE != 0)
				return false;
			
			DSI_clk_HS_mode(0);

			DSI_REG->DSI_COM_CTRL.DSI_RESET = 1;
			DPI_DisableClk();
			DSI_SetMode(CMD_MODE);
			DSI_REG->DSI_COM_CTRL.DSI_RESET = 0;
			//DSI_Reset();

			t0.CONFG = 0x20;		///TE
			t0.Data0 = 0;
			t0.Data_ID = 0;
			t0.Data1 = 0;
			
			OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t0));
			OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);

			// Enable RD_RDY INT for polling it's status later
			DSI_REG->DSI_INTEN.RD_RDY =  1;

			DSI_EnableClk();

	        while(DSI_REG->DSI_INTSTA.RD_RDY == 0)  // polling RD_RDY
	        {
				if(get_current_time_us() - dsi_current_time > _dsiContext.vfp_period_us)
				{
					xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for internal TE time-out for %d (us)!!!\n", _dsiContext.vfp_period_us);

					///do necessary reset here
					DSI_REG->DSI_RACK.DSI_RACK = 1;
					DSI_Reset();

					return true;
				}
	        }

			// Write clear RD_RDY
			DSI_REG->DSI_INTSTA.RD_RDY = 1;
			DSI_REG->DSI_RACK.DSI_RACK = 1;
			// Write clear CMD_DONE
			DSI_REG->DSI_INTSTA.CMD_DONE = 1;

			// Restart video mode. (with VSA ahead)
			DSI_SetMode(SYNC_PULSE_VDO_MODE);
			DSI_clk_HS_mode(1);
			DPI_EnableClk();
			DSI_EnableClk();
		}
	
	}

	return false;

}


void DSI_set_noncont_clk(bool enable, unsigned int period)
{
	dsi_noncont_clk_enabled = enable;
	if(period<1)
		period = 1;
	dsi_noncont_clk_period = period;
	dsi_dpi_isr_count = 0;
}


// called by DPI ISR.
void DSI_handle_noncont_clk(void)
{
	unsigned int state;
	long int dsi_current_time;

	if (!DSI_REG->DSI_MODE_CTRL.MODE)
		return ;

	state = DSI_REG->DSI_STATE_DBG3.TCON_STATE;

	dsi_current_time = get_current_time_us();

	switch(state)
	{
		case DSI_VDO_VSA_VS_STATE:
			while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VSA_HS_STATE)
			{
				if(get_current_time_us() - dsi_current_time > _dsiContext.vsa_vs_period_us)
				{
					xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VSA_HS_STATE, _dsiContext.vsa_vs_period_us);
					return ;
				}			
			}
			break;

		case DSI_VDO_VSA_HS_STATE:
			while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VSA_VE_STATE)
			{
				if(get_current_time_us() - dsi_current_time > _dsiContext.vsa_hs_period_us)
				{
					xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VSA_VE_STATE, _dsiContext.vsa_hs_period_us);
					return ;
				}			
			}
			break;

		case DSI_VDO_VSA_VE_STATE:
			while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VBP_STATE)
			{
				if(get_current_time_us() - dsi_current_time > _dsiContext.vsa_ve_period_us)
				{
					xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VBP_STATE, _dsiContext.vsa_ve_period_us);
					return ;
				}			
			}
			break;

		case DSI_VDO_VBP_STATE:
			xlog_printk(ANDROID_LOG_WARN, "DSI", "Can't do clock switch in DSI_VDO_VBP_STATE !!!\n");
			break;

		case DSI_VDO_VACT_STATE:
			while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VFP_STATE)
			{
				if(get_current_time_us() - dsi_current_time > _dsiContext.vfp_period_us )
				{
					xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VFP_STATE, _dsiContext.vfp_period_us );
					return ;
				}			
			}
			break;

		case DSI_VDO_VFP_STATE:
			while(DSI_REG->DSI_STATE_DBG3.TCON_STATE != DSI_VDO_VSA_VS_STATE)
			{
				if(get_current_time_us() - dsi_current_time > _dsiContext.vfp_period_us )
				{
					xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for %x state timeout %d (us)!!!\n", DSI_VDO_VSA_VS_STATE, _dsiContext.vfp_period_us );
					return ;
				}			
			}
			break;

		default :
			xlog_printk(ANDROID_LOG_ERROR, "DSI", "invalid state = %x \n", state);
			return ;
	}

	// Clock switch HS->LP->HS
	DSI_clk_HS_mode(0);
	udelay(1);
	DSI_clk_HS_mode(1);

}
#endif


void DSI_set_cmdq_V3(LCM_setting_table_V3 *para_tbl, unsigned int size, unsigned char force_update)
{
	UINT32 i, layer, layer_state, lane_num;
	UINT32 goto_addr, mask_para, set_para;
	UINT32 fbPhysAddr, fbVirAddr;
	DSI_T0_INS t0;	
	DSI_T1_INS t1;	
	DSI_T2_INS t2;	

	UINT32 index = 0;

	unsigned char data_id, cmd, count;
	unsigned char *para_list;

	do {
		data_id = para_tbl[index].id;
		cmd = para_tbl[index].cmd;
		count = para_tbl[index].count;
		para_list = para_tbl[index].para_list;

		if (data_id == REGFLAG_ESCAPE_ID && cmd == REGFLAG_DELAY_MS_V3)
		{
			udelay(1000*count);
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI_set_cmdq_V3[%d]. Delay %d (ms) \n", index, count);

			continue;
		}

		_WaitForEngineNotBusy();
		
		if (count > 59)
		{
			UINT32 pixel = count/3 + ((count%3) ? 1 : 0);
			
			LCD_REG_LAYER	fb_layer_info;
			LCD_REG_DSI_DC	dsi_info;
			bool			lcd_w2m, lcm_te_enable;

			// backup layer state.
			layer_state = AS_UINT32(&LCD_REG->WROI_CONTROL) & 0xFC000000;

			// backup FB layer info.
			memcpy((void *)&fb_layer_info, (void *)&LCD_REG->LAYER[FB_LAYER], sizeof(LCD_REG_LAYER));

			// backup LCD-DSI I/F configuration.
			dsi_info = LCD_REG->DS_DSI_CON;

			// backup lane number.
			lane_num = DSI_REG->DSI_TXRX_CTRL.LANE_NUM;

			// backup lcd_w2m
			lcd_w2m = LCD_REG->WROI_CONTROL.W2M;

			// backup TE enable
			lcm_te_enable = LCD_REG->TEARING_CFG.ENABLE;

			// HW limitation.
			// LP type-1 command can't go with 2 lanes. So we must switch to lane mode.
			DSI_REG->DSI_TXRX_CTRL.LANE_NUM = 1;
			DSI_PHY_REG->MIPITX_CON1.RG_DSI_CK_SEL = 0;

			// Modify LCD-DSI configuration
			LCD_REG->DS_DSI_CON.DC_DSI = TRUE;
			// Let LSB of RGB(BGR in buffer) first.
			LCD_REG->DS_DSI_CON.RGB_SWAP = LCD_DSI_IF_FMT_COLOR_ORDER_BGR;
			// Let parameters be in unit of byte.
			LCD_REG->DS_DSI_CON.CLR_FMT = LCD_DSI_IF_FORMAT_RGB888;
			// Disable W2M
			LCD_REG->WROI_CONTROL.W2M = 0;

			// HW limitation
			// It makes package numbers > 1.
			LCD_REG->DS_DSI_CON.PACKET_SIZE = 256;

			// Disable TE
			LCD_TE_Enable(0);

			// Start of Enable only one layer (FB layer) to push data to DSI
			LCD_CHECK_RET(LCD_LayerEnable(LCD_LAYER_ALL, FALSE));
			LCD_CHECK_RET(LCD_LayerEnable(FB_LAYER, TRUE));
			LCD_CHECK_RET(LCD_SetRoiWindow(0, 0, pixel, 1));
			LCD_CHECK_RET(LCD_SetBackgroundColor(0));
#ifdef BUILD_UBOOT
			LCD_REG->LAYER[FB_LAYER].ADDRESS = para_list;
#else      
			// operates on FB layer
			{
				extern void disp_get_fb_address(UINT32 *fbVirAddr, UINT32 *fbPhysAddr);
				disp_get_fb_address(&fbVirAddr ,&fbPhysAddr);

				// copy parameters to FB layer buffer.
				memcpy((void *)fbVirAddr, (void *)para_list, count);
				//LCD_REG->LAYER[FB_LAYER].ADDRESS = fbPhysAddr;
			}
#endif
			LCD_CHECK_RET(LCD_LayerSetFormat(FB_LAYER, LCD_LAYER_FORMAT_RGB888));
			LCD_CHECK_RET(LCD_LayerSetPitch(FB_LAYER, pixel*3));
			LCD_CHECK_RET(LCD_LayerSetOffset(FB_LAYER, 0, 0));
			LCD_CHECK_RET(LCD_LayerSetSize(FB_LAYER, pixel, 1));
			// End of Enable only one layer (FB layer) to push data to DSI

			t1.CONFG = 1;
			t1.Data_ID = data_id;	
			t1.mem_start0 = (cmd&0xFF);
			t1.mem_start1 = (cmd>>8);

			OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t1));
			OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);	

			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] - kernel - DSI_set_cmdq_V3[%d]. command(0x%x) parameter count = %d > 59, pixel = %d \n", index, cmd, count, pixel);
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] - kernel - command queue only support 16 x 4 bytes. Header used 4 byte. DCS used 1 byte. If parameter > 59 byte, work around by Type-1 command. \n");
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "para_list[%d] = {", count);
			for (i = 0; i < count; i++)
				DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "0x%02x, ", para_list[i]);
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "} \n");

#ifdef BUILD_UBOOT
			if(force_update)
			{
				LCD_CHECK_RET(LCD_StartTransfer(FALSE));
				DSI_EnableClk();

				while(DSI_REG->DSI_STA.BUSY || DSI_REG->DSI_STATE_DBG4.EXE_STATE != 1) {
					printk("[DISP - uboot - DSI Busy : %d, DSI_STATE_DBG4.EXE_STATE = %d, LCD Busy : %d \n", DSI_REG->DSI_STA.BUSY, DSI_REG->DSI_STATE_DBG4.EXE_STATE, LCD_REG->STATUS.BUSY);
					if (DSI_REG->DSI_STATE_DBG4.EXE_STATE == 2 && !LCD_REG->STATUS.BUSY) {
						DSI_Reset();
					}				
				}
								
			}
#else      
			//LCD_M4U_On(0);

			if(force_update)
			{
				LCD_CHECK_RET(LCD_StartTransfer(FALSE));		
				DSI_EnableClk();

				while (_IsEngineBusy())
				{
					long ret = wait_event_interruptible_timeout(_dsi_wait_queue, 
																!_IsEngineBusy(),
																1 * HZ);
					if (0 == ret) {
						DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", "[%s] Wait for DSI engine not busy timeout!!! Busy : %d, DSI_STATE_DBG4.EXE_STATE = %d \n", __func__, DSI_REG->DSI_STA.BUSY, DSI_REG->DSI_STATE_DBG4.EXE_STATE);
						if (DSI_REG->DSI_STATE_DBG4.EXE_STATE == 2) {
							DSI_Reset();
						}
					}
				}
			}

			//LCD_M4U_On(1);
#endif
			// restore FB layer info.
			memcpy((void *)&LCD_REG->LAYER[FB_LAYER], (void *)&fb_layer_info, sizeof(LCD_REG_LAYER));

			// restore LCD-DSI I/F configuration.
			LCD_REG->DS_DSI_CON = dsi_info;

			// restore lane number.
			DSI_REG->DSI_TXRX_CTRL.LANE_NUM = lane_num;
			DSI_PHY_REG->MIPITX_CON1.RG_DSI_CK_SEL = (lane_num - 1);
			
			// restore layer state.
			for(layer=LCD_LAYER_0; layer<LCD_LAYER_NUM; layer++)
			{
				if(layer_state&(0x80000000>>layer))
					LCD_CHECK_RET(LCD_LayerEnable(layer, TRUE));
				else
					LCD_CHECK_RET(LCD_LayerEnable(layer, FALSE));
			}

			// restore lcd_w2m
			if (lcd_w2m)
				LCD_REG->WROI_CONTROL.W2M = 1;

			// restore TE
			if(lcm_te_enable)
				LCD_TE_Enable(1);

		}
		else
		{
			//for(i = 0; i < sizeof(DSI_CMDQ_REG->data0) / sizeof(DSI_CMDQ); i++)
			//	OUTREG32(&DSI_CMDQ_REG->data0[i], 0);
			memset(&DSI_CMDQ_REG->data0, 0, sizeof(DSI_CMDQ_REG->data0));
		
			if (count > 1)
			{
				t2.CONFG = 2;
				t2.Data_ID = data_id;
				t2.WC16 = count+1;

				OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t2));

				goto_addr = (UINT32)(&DSI_CMDQ_REG->data0[1].byte0);
				mask_para = (0xFF<<((goto_addr&0x3)*8));
				set_para = (cmd<<((goto_addr&0x3)*8));
				MASKREG32(goto_addr&(~0x3), mask_para, set_para);
				
				for(i=0; i<count; i++)
				{
					goto_addr = (UINT32)(&DSI_CMDQ_REG->data0[1].byte1) + i;
					mask_para = (0xFF<<((goto_addr&0x3)*8));
					set_para = (para_list[i]<<((goto_addr&0x3)*8));
					MASKREG32(goto_addr&(~0x3), mask_para, set_para);			
				}

				OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 2+(count)/4); 		
			}
			else
			{
				t0.CONFG = 0;
				t0.Data0 = cmd;
				if (count)
				{
					t0.Data_ID = data_id;
					t0.Data1 = para_list[0];
				}
				else
				{
					t0.Data_ID = data_id;
					t0.Data1 = 0;
				}
				OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t0));
				OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);
			}
			
			for (i = 0; i < AS_UINT32(&DSI_REG->DSI_CMDQ_SIZE); i++)
				DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI_set_cmdq_V3[%d]. DSI_CMDQ+%04x : 0x%08x\n", index, i*4, INREG32(DSI_BASE + 0x180 + i*4));

			if(force_update)
				DSI_EnableClk();
		}

	} while (++index < size);

}


void DSI_set_cmdq_V2(unsigned cmd, unsigned char count, unsigned char *para_list, unsigned char force_update)
{
	UINT32 i, layer, layer_state, lane_num;
	UINT32 goto_addr, mask_para, set_para;
	UINT32 fbPhysAddr, fbVirAddr;
	DSI_T0_INS t0;	
	DSI_T1_INS t1;	
	DSI_T2_INS t2;	

    _WaitForEngineNotBusy();
    
	if (count > 59)
	{
		UINT32 pixel = count/3 + ((count%3) ? 1 : 0);
		
		LCD_REG_LAYER 	fb_layer_info;
		LCD_REG_DSI_DC	dsi_info;
		bool			lcd_w2m, lcm_te_enable;

		// backup layer state.
		layer_state = AS_UINT32(&LCD_REG->WROI_CONTROL) & 0xFC000000;

		// backup FB layer info.
		memcpy((void *)&fb_layer_info, (void *)&LCD_REG->LAYER[FB_LAYER], sizeof(LCD_REG_LAYER));

		// backup LCD-DSI I/F configuration.
		dsi_info = LCD_REG->DS_DSI_CON;

		// backup lane number.
		lane_num = DSI_REG->DSI_TXRX_CTRL.LANE_NUM;

		// backup lcd_w2m
		lcd_w2m = LCD_REG->WROI_CONTROL.W2M;

		// backup TE enable
		lcm_te_enable = LCD_REG->TEARING_CFG.ENABLE;

		// HW limitation.
		// LP type-1 command can't go with 2 lanes. So we must switch to lane mode.
		DSI_REG->DSI_TXRX_CTRL.LANE_NUM = 1;
		DSI_PHY_REG->MIPITX_CON1.RG_DSI_CK_SEL = 0;

		// Modify LCD-DSI configuration
		LCD_REG->DS_DSI_CON.DC_DSI = TRUE;
		// Let LSB of RGB(BGR in buffer) first.
		LCD_REG->DS_DSI_CON.RGB_SWAP = LCD_DSI_IF_FMT_COLOR_ORDER_BGR;
		// Let parameters be in unit of byte.
		LCD_REG->DS_DSI_CON.CLR_FMT = LCD_DSI_IF_FORMAT_RGB888;
		// Disable W2M
		LCD_REG->WROI_CONTROL.W2M = 0;

		// HW limitation
		// It makes package numbers > 1.
		LCD_REG->DS_DSI_CON.PACKET_SIZE = 256;

		// Disable TE
		LCD_TE_Enable(0);

		// Start of Enable only one layer (FB layer) to push data to DSI
		LCD_CHECK_RET(LCD_LayerEnable(LCD_LAYER_ALL, FALSE));
		LCD_CHECK_RET(LCD_LayerEnable(FB_LAYER, TRUE));
		LCD_CHECK_RET(LCD_SetRoiWindow(0, 0, pixel, 1));
		LCD_CHECK_RET(LCD_SetBackgroundColor(0));
#ifdef BUILD_UBOOT
		LCD_REG->LAYER[FB_LAYER].ADDRESS = para_list;
#else      
		// operates on FB layer
		{
		    extern void disp_get_fb_address(UINT32 *fbVirAddr, UINT32 *fbPhysAddr);
            disp_get_fb_address(&fbVirAddr ,&fbPhysAddr);

		    // copy parameters to FB layer buffer.
		    memcpy((void *)fbVirAddr, (void *)para_list, count);
		    //LCD_REG->LAYER[FB_LAYER].ADDRESS = fbPhysAddr;
		}
#endif
		LCD_CHECK_RET(LCD_LayerSetFormat(FB_LAYER, LCD_LAYER_FORMAT_RGB888));
		LCD_CHECK_RET(LCD_LayerSetPitch(FB_LAYER, pixel*3));
		LCD_CHECK_RET(LCD_LayerSetOffset(FB_LAYER, 0, 0));
		LCD_CHECK_RET(LCD_LayerSetSize(FB_LAYER, pixel, 1));
		// End of Enable only one layer (FB layer) to push data to DSI

		t1.CONFG = 1;
		if (cmd < 0xB0)
			t1.Data_ID = DSI_DCS_LONG_PACKET_ID;	
		else
			t1.Data_ID = DSI_GERNERIC_LONG_PACKET_ID;
		t1.mem_start0 = (cmd&0xFF);
		t1.mem_start1 = (cmd>>8);

		OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t1));
		OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);	

		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] - kernel - DSI_set_cmdq_V2. command(0x%x) parameter count = %d > 59, pixel = %d \n", cmd, count, pixel);
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[DISP] - kernel - command queue only support 16 x 4 bytes. Header used 4 byte. DCS used 1 byte. If parameter > 59 byte, work around by Type-1 command. \n");
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "para_list[%d] = {", count);
		for (i = 0; i < count; i++)
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "0x%02x, ", para_list[i]);
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "} \n");

#ifdef BUILD_UBOOT
		if(force_update)
		{
			LCD_CHECK_RET(LCD_StartTransfer(FALSE));
			DSI_EnableClk();

			while(DSI_REG->DSI_STA.BUSY || DSI_REG->DSI_STATE_DBG4.EXE_STATE != 1) {
				printk("[DISP - uboot - DSI Busy : %d, DSI_STATE_DBG4.EXE_STATE = %d, LCD Busy : %d \n", DSI_REG->DSI_STA.BUSY, DSI_REG->DSI_STATE_DBG4.EXE_STATE, LCD_REG->STATUS.BUSY);
				if (DSI_REG->DSI_STATE_DBG4.EXE_STATE == 2 && !LCD_REG->STATUS.BUSY) {
					DSI_Reset();
				}				
			}
							
		}
#else      
		//LCD_M4U_On(0);

		if(force_update)
		{
			LCD_CHECK_RET(LCD_StartTransfer(FALSE));		
			DSI_EnableClk();

			while (_IsEngineBusy())
	        {
	            long ret = wait_event_interruptible_timeout(_dsi_wait_queue, 
	                                                        !_IsEngineBusy(),
	                                                        1 * HZ);
	            if (0 == ret) {
	                DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", "[%s] Wait for DSI engine not busy timeout!!! Busy : %d, DSI_STATE_DBG4.EXE_STATE = %d \n", __func__, DSI_REG->DSI_STA.BUSY, DSI_REG->DSI_STATE_DBG4.EXE_STATE);
					if (DSI_REG->DSI_STATE_DBG4.EXE_STATE == 2) {
						DSI_Reset();
					}
	            }
	        }
		}

		//LCD_M4U_On(1);
#endif
		// restore FB layer info.
		memcpy((void *)&LCD_REG->LAYER[FB_LAYER], (void *)&fb_layer_info, sizeof(LCD_REG_LAYER));

		// restore LCD-DSI I/F configuration.
		LCD_REG->DS_DSI_CON = dsi_info;

		// restore lane number.
		DSI_REG->DSI_TXRX_CTRL.LANE_NUM = lane_num;
		DSI_PHY_REG->MIPITX_CON1.RG_DSI_CK_SEL = (lane_num - 1);
		
		// restore layer state.
		for(layer=LCD_LAYER_0; layer<LCD_LAYER_NUM; layer++)
		{
			if(layer_state&(0x80000000>>layer))
				LCD_CHECK_RET(LCD_LayerEnable(layer, TRUE));
			else
				LCD_CHECK_RET(LCD_LayerEnable(layer, FALSE));
		}

		// restore lcd_w2m
		if (lcd_w2m)
			LCD_REG->WROI_CONTROL.W2M = 1;

		// restore TE
		if(lcm_te_enable)
			LCD_TE_Enable(1);

	}
	else
	{
	if (cmd < 0xB0)
	{
		if (count > 1)
		{
			t2.CONFG = 2;
			t2.Data_ID = DSI_DCS_LONG_PACKET_ID;
			t2.WC16 = count+1;

			OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t2));

			goto_addr = (UINT32)(&DSI_CMDQ_REG->data0[1].byte0);
			mask_para = (0xFF<<((goto_addr&0x3)*8));
			set_para = (cmd<<((goto_addr&0x3)*8));
			MASKREG32(goto_addr&(~0x3), mask_para, set_para);
			
			for(i=0; i<count; i++)
			{
				goto_addr = (UINT32)(&DSI_CMDQ_REG->data0[1].byte1) + i;
				mask_para = (0xFF<<((goto_addr&0x3)*8));
				set_para = (para_list[i]<<((goto_addr&0x3)*8));
				MASKREG32(goto_addr&(~0x3), mask_para, set_para);			
			}

			OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 2+(count)/4); 		
		}
		else
		{
			t0.CONFG = 0;
			t0.Data0 = cmd;
			if (count)
			{
				t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_1;
				t0.Data1 = para_list[0];
			}
			else
			{
				t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_0;
				t0.Data1 = 0;
			}
			OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t0));
			OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);
		}
	}
	else
	{
		if (count > 1)
		{
			t2.CONFG = 2;
			t2.Data_ID = DSI_GERNERIC_LONG_PACKET_ID;
			t2.WC16 = count+1;

			OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t2));

			goto_addr = (UINT32)(&DSI_CMDQ_REG->data0[1].byte0);
			mask_para = (0xFF<<((goto_addr&0x3)*8));
			set_para = (cmd<<((goto_addr&0x3)*8));
			MASKREG32(goto_addr&(~0x3), mask_para, set_para);
			
			for(i=0; i<count; i++)
			{
				goto_addr = (UINT32)(&DSI_CMDQ_REG->data0[1].byte1) + i;
				mask_para = (0xFF<<((goto_addr&0x3)*8));
				set_para = (para_list[i]<<((goto_addr&0x3)*8));
				MASKREG32(goto_addr&(~0x3), mask_para, set_para);			
			}

			OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 2+(count)/4);

		}
		else
		{
			t0.CONFG = 0;
			t0.Data0 = cmd;
			if (count)
			{
				t0.Data_ID = DSI_GERNERIC_SHORT_PACKET_ID_2;
				t0.Data1 = para_list[0];
			}
			else
			{
				t0.Data_ID = DSI_GERNERIC_SHORT_PACKET_ID_1;
				t0.Data1 = 0;
			}
			OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t0));
			OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);
		}
	}
		
	for (i = 0; i < AS_UINT32(&DSI_REG->DSI_CMDQ_SIZE); i++)
        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI_set_cmdq_V2. DSI_CMDQ+%04x : 0x%08x\n", i*4, INREG32(DSI_BASE + 0x180 + i*4));

		if(force_update)
			DSI_EnableClk();
	}

}


void DSI_set_cmdq(unsigned int *pdata, unsigned int queue_size, unsigned char force_update)
{
	UINT32 i;

	ASSERT(queue_size<=16);

    _WaitForEngineNotBusy();

	for(i=0; i<queue_size; i++)
		OUTREG32(&DSI_CMDQ_REG->data0[i], AS_UINT32((pdata+i)));

	OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, queue_size);

    //for (i = 0; i < queue_size; i++)
    //    printk("[DISP] - kernel - DSI_set_cmdq. DSI_CMDQ+%04x : 0x%08x\n", i*4, INREG32(DSI_BASE + 0x180 + i*4));

	if(force_update)
		DSI_EnableClk();
	
}


DSI_STATUS DSI_Write_T0_INS(DSI_T0_INS *t0)
{
    OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(t0));	

	OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);
	OUTREG32(&DSI_REG->DSI_START, 0);
	OUTREG32(&DSI_REG->DSI_START, 1);

	return DSI_STATUS_OK;	
}


DSI_STATUS DSI_Write_T1_INS(DSI_T1_INS *t1)
{
    OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(t1));	

	OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);
	OUTREG32(&DSI_REG->DSI_START, 0);
	OUTREG32(&DSI_REG->DSI_START, 1);

	return DSI_STATUS_OK;
}


DSI_STATUS DSI_Write_T2_INS(DSI_T2_INS *t2)
{
	unsigned int i;
	
	OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(t2));

	for(i=0;i<((t2->WC16-1)>>2)+1;i++)
	    OUTREG32(&DSI_CMDQ_REG->data0[1+i], AS_UINT32((t2->pdata+i)));

	OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, (((t2->WC16-1)>>2)+2));
	OUTREG32(&DSI_REG->DSI_START, 0);
	OUTREG32(&DSI_REG->DSI_START, 1);

	return DSI_STATUS_OK;
}


DSI_STATUS DSI_Write_T3_INS(DSI_T3_INS *t3)
{
    OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(t3));	

	OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);
	OUTREG32(&DSI_REG->DSI_START, 0);
	OUTREG32(&DSI_REG->DSI_START, 1);

	return DSI_STATUS_OK;
}

DSI_STATUS DSI_TXRX_Control(bool cksm_en, 
                                  bool ecc_en, 
                                  unsigned char lane_num, 
                                  unsigned char vc_num,
                                  bool null_packet_en,
                                  bool err_correction_en,
                                  bool dis_eotp_en,
                                  unsigned int  max_return_size)
{
    DSI_TXRX_CTRL_REG tmp_reg;
    
    tmp_reg=DSI_REG->DSI_TXRX_CTRL;

    ///TODO: parameter checking
    tmp_reg.CKSM_EN=cksm_en;
    tmp_reg.ECC_EN=ecc_en;
    tmp_reg.LANE_NUM=lane_num;
    tmp_reg.VC_NUM=vc_num;
    tmp_reg.CORR_EN = err_correction_en;
    tmp_reg.DIS_EOT = dis_eotp_en;
    tmp_reg.NULL_EN = null_packet_en;
    tmp_reg.MAX_RTN_SIZE = max_return_size;

    OUTREG32(&DSI_REG->DSI_TXRX_CTRL, AS_UINT32(&tmp_reg));
#ifndef BUILD_UBOOT	
	if(!is_needReConfig)
		is_needReConfig = true;
	else
#endif
		DSI_Reset();
	return DSI_STATUS_OK;
}


DSI_STATUS DSI_PS_Control(unsigned int ps_type, unsigned int ps_wc)
{
    DSI_PSCTRL_REG tmp_reg;
    tmp_reg=DSI_REG->DSI_PSCTRL;
    
    ///TODO: parameter checking
    ASSERT(ps_type <= PACKED_PS_18BIT_RGB666);
    tmp_reg.DSI_PS_SEL=ps_type;
    tmp_reg.DSI_PS_WC=ps_wc;
    
    OUTREG32(&DSI_REG->DSI_PSCTRL, AS_UINT32(&tmp_reg));   
    return DSI_STATUS_OK;
}


DSI_STATUS DSI_ConfigVDOTiming(unsigned int dsi_vsa_nl,
                                    unsigned int dsi_vbp_nl,
                                    unsigned int dsi_vfp_nl,
                                    unsigned int dsi_vact_nl,
                                    unsigned int dsi_line_nb,
                                    unsigned int dsi_hsa_nb,
                                    unsigned int dsi_hbp_nb,
                                    unsigned int dsi_hfp_nb,
                                    unsigned int dsi_rgb_nb,
                                    unsigned int dsi_hsa_wc,
                                    unsigned int dsi_hbp_wc,
                                    unsigned int dsi_hfp_wc)
{
    ///TODO: parameter checking
	OUTREG32(&DSI_REG->DSI_VSA_NL, AS_UINT32(&dsi_vsa_nl));
	OUTREG32(&DSI_REG->DSI_VBP_NL, AS_UINT32(&dsi_vbp_nl));
	OUTREG32(&DSI_REG->DSI_VFP_NL, AS_UINT32(&dsi_vfp_nl));
	OUTREG32(&DSI_REG->DSI_VACT_NL, AS_UINT32(&dsi_vact_nl));

	OUTREG32(&DSI_REG->DSI_LINE_NB, AS_UINT32(&dsi_line_nb));
	OUTREG32(&DSI_REG->DSI_HSA_NB, AS_UINT32(&dsi_hsa_nb));
	OUTREG32(&DSI_REG->DSI_HBP_NB, AS_UINT32(&dsi_hbp_nb));
	OUTREG32(&DSI_REG->DSI_HFP_NB, AS_UINT32(&dsi_hfp_nb));
	OUTREG32(&DSI_REG->DSI_RGB_NB, AS_UINT32(&dsi_rgb_nb));

	OUTREG32(&DSI_REG->DSI_HSA_WC, AS_UINT32(&dsi_hsa_wc));
	OUTREG32(&DSI_REG->DSI_HBP_WC, AS_UINT32(&dsi_hbp_wc));
	OUTREG32(&DSI_REG->DSI_HFP_WC, AS_UINT32(&dsi_hfp_wc));	

    return DSI_STATUS_OK;
}

void DSI_write_lcm_cmd(unsigned int cmd)
{
	DSI_T0_INS *t0_tmp=0;
	DSI_CMDQ_CONFG CONFG_tmp;

	CONFG_tmp.type=SHORT_PACKET_RW;
	CONFG_tmp.BTA=DISABLE_BTA;
	CONFG_tmp.HS=LOW_POWER;
	CONFG_tmp.CL=CL_8BITS;
	CONFG_tmp.TE=DISABLE_TE;
	CONFG_tmp.RPT=DISABLE_RPT;

	t0_tmp->CONFG = *((unsigned char *)(&CONFG_tmp));
	t0_tmp->Data_ID= (cmd&0xFF);
	t0_tmp->Data0 = 0x0;
	t0_tmp->Data1 = 0x0;	

	DSI_Write_T0_INS(t0_tmp);
}


void DSI_write_lcm_regs(unsigned int addr, unsigned int *para, unsigned int nums)
{
	DSI_T2_INS *t2_tmp=0;
	DSI_CMDQ_CONFG CONFG_tmp;

	CONFG_tmp.type=LONG_PACKET_W;
	CONFG_tmp.BTA=DISABLE_BTA;
	CONFG_tmp.HS=LOW_POWER;
	CONFG_tmp.CL=CL_8BITS;
	CONFG_tmp.TE=DISABLE_TE;
	CONFG_tmp.RPT=DISABLE_RPT;

	t2_tmp->CONFG = *((unsigned char *)(&CONFG_tmp));
	t2_tmp->Data_ID = (addr&0xFF);
	t2_tmp->WC16 = nums;	
	t2_tmp->pdata = para;	

	DSI_Write_T2_INS(t2_tmp);

}

UINT32 DSI_dcs_read_lcm_reg(UINT8 cmd)
{
    UINT32 max_try_count = 5;
    UINT32 recv_data;
    UINT32 recv_data_cnt;
    unsigned int read_timeout_ms;
    unsigned char packet_type;
    DSI_T0_INS t0;  
#if ENABLE_DSI_INTERRUPT
    static const long WAIT_TIMEOUT = 2 * HZ;    // 2 sec
    long ret;
#endif

	if (DSI_REG->DSI_MODE_CTRL.MODE)
		return 0;

    do
    {
       if(max_try_count == 0)
          return 0;

       max_try_count--;
       recv_data = 0;
       recv_data_cnt = 0;
       read_timeout_ms = 20;
        
       _WaitForEngineNotBusy();

       t0.CONFG = 0x04;        ///BTA
       t0.Data0 = cmd;
       t0.Data_ID = DSI_DCS_READ_PACKET_ID;
       t0.Data1 = 0;

       OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t0));
       OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);

       ///clear read ACK 
       DSI_REG->DSI_RACK.DSI_RACK = 1;
       DSI_REG->DSI_INTSTA.RD_RDY = 1;
       DSI_REG->DSI_INTSTA.CMD_DONE = 1;
       DSI_REG->DSI_INTEN.RD_RDY =  1;
       DSI_REG->DSI_INTEN.CMD_DONE=  1;

       OUTREG32(&DSI_REG->DSI_START, 0);
       OUTREG32(&DSI_REG->DSI_START, 1);

       /// the following code is to
       /// 1: wait read ready
       /// 2: ack read ready
       /// 3: wait for CMDQ_DONE
       /// 3: read data
#if ENABLE_DSI_INTERRUPT
        ret = wait_event_interruptible_timeout(_dsi_dcs_read_wait_queue, 
                                                       !_IsEngineBusy(),
                                                       WAIT_TIMEOUT);
        if (0 == ret) {
            DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", " Wait for DSI engine read ready timeout!!!\n");

				DSI_DumpRegisters();
				
				///do necessary reset here
				DSI_REG->DSI_RACK.DSI_RACK = 1;
				DSI_Reset();

                return 0;
            }
#else
    #ifdef DSI_DRV_DEBUG_LOG_ENABLE
        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " Start polling DSI read ready!!!\n");
    #endif
        while(DSI_REG->DSI_INTSTA.RD_RDY == 0)  ///read clear
        {
            ///keep polling
            msleep(1);
            read_timeout_ms --;
            
            if(read_timeout_ms == 0)
            {
                DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " Polling DSI read ready timeout!!!\n");
                DSI_DumpRegisters();

                ///do necessary reset here
                DSI_REG->DSI_RACK.DSI_RACK = 1;
                DSI_Reset();
                return 0;
            }
        }
    #ifdef DSI_DRV_DEBUG_LOG_ENABLE
        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " End polling DSI read ready!!!\n");
    #endif

        DSI_REG->DSI_RACK.DSI_RACK = 1;

       while(DSI_REG->DSI_STA.BUSY)
       {
           ///DSI READ ACK HW bug workaround
    #ifdef DSI_DRV_DEBUG_LOG_ENABLE
           DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI is busy: 0x%x !!!\n", DSI_REG->DSI_STA.BUSY);
    #endif
           DSI_REG->DSI_RACK.DSI_RACK = 1;
       }


       ///clear interrupt status
       DSI_REG->DSI_INTSTA.RD_RDY = 1;
	   DSI_REG->DSI_INTSTA.CMD_DONE = 1;
       ///STOP DSI
       OUTREG32(&DSI_REG->DSI_START, 0);

#endif
   
       DSI_REG->DSI_INTEN.RD_RDY =  0;

    #ifdef DSI_DRV_DEBUG_LOG_ENABLE
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_STA : 0x%x \n", DSI_REG->DSI_RX_STA);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_SIZE : 0x%x \n", DSI_REG->DSI_CMDQ_SIZE.CMDQ_SIZE);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_DATA0 : 0x%x \n", DSI_CMDQ_REG->data0[0].byte0);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_DATA1 : 0x%x \n", DSI_CMDQ_REG->data0[0].byte1);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_DATA2 : 0x%x \n", DSI_CMDQ_REG->data0[0].byte2);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_DATA3 : 0x%x \n", DSI_CMDQ_REG->data0[0].byte3);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE0 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE0);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE1 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE1);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE2 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE2);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE3 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE3);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE4 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE4);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE5 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE5);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE6 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE6);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE7 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE7);
    #endif
       packet_type = DSI_REG->DSI_RX_DATA.BYTE0;
           
    #ifdef DSI_DRV_DEBUG_LOG_ENABLE
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI read packet_type is 0x%x \n",packet_type);
    #endif
       if(DSI_REG->DSI_RX_STA.LONG == 1)
       {
           recv_data_cnt = DSI_REG->DSI_RX_DATA.BYTE1 + DSI_REG->DSI_RX_DATA.BYTE2 * 16;
           if(recv_data_cnt > 4)
           {
    #ifdef DSI_DRV_DEBUG_LOG_ENABLE
              DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", " DSI read long packet data  exceeds 4 bytes \n");
    #endif
              recv_data_cnt = 4;
           }
           memcpy((void*)&recv_data, (void*)&DSI_REG->DSI_RX_DATA.BYTE4, recv_data_cnt);
       }
       else
       {
           memcpy((void*)&recv_data, (void*)&DSI_REG->DSI_RX_DATA.BYTE1, 2);
       }
    
    #ifdef DSI_DRV_DEBUG_LOG_ENABLE
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI read 0x%x data is 0x%x \n",cmd,  recv_data);
    #endif
   }while(packet_type != 0x1C && packet_type != 0x21 && packet_type != 0x22);
   /// here: we may receive a ACK packet which packet type is 0x02 (incdicates some error happened)
   /// therefore we try re-read again until no ACK packet
   /// But: if it is a good way to keep re-trying ???
   
   return recv_data;
}

/// return value: the data length we got
UINT32 DSI_dcs_read_lcm_reg_v2(UINT8 cmd, UINT8 *buffer, UINT8 buffer_size)
{
    UINT32 max_try_count = 5;
    UINT32 recv_data_cnt;
    unsigned int read_timeout_ms;
    unsigned char packet_type;
    DSI_T0_INS t0;  

#if ENABLE_DSI_INTERRUPT
    static const long WAIT_TIMEOUT = 2 * HZ;    // 2 sec
    long ret;
#endif

	if (DSI_REG->DSI_MODE_CTRL.MODE)
		return 0;

    if (buffer == NULL || buffer_size == 0)
        return 0;

    do
    {
       if(max_try_count == 0)
	  return 0;
       max_try_count--;
       recv_data_cnt = 0;
       read_timeout_ms = 20;
        
       _WaitForEngineNotBusy();

       t0.CONFG = 0x04;        ///BTA
       t0.Data0 = cmd;
       if (buffer_size < 0x3)
           t0.Data_ID = DSI_DCS_READ_PACKET_ID;
       else
           t0.Data_ID = DSI_GERNERIC_READ_LONG_PACKET_ID;
       t0.Data1 = 0;

       OUTREG32(&DSI_CMDQ_REG->data0[0], AS_UINT32(&t0));
       OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);

       ///clear read ACK 
       DSI_REG->DSI_RACK.DSI_RACK = 1;
       DSI_REG->DSI_INTSTA.RD_RDY = 1;
       DSI_REG->DSI_INTSTA.CMD_DONE = 1;
       DSI_REG->DSI_INTEN.RD_RDY =  1;
       DSI_REG->DSI_INTEN.CMD_DONE=  1;

       OUTREG32(&DSI_REG->DSI_START, 0);
       OUTREG32(&DSI_REG->DSI_START, 1);

       /// the following code is to
       /// 1: wait read ready
       /// 2: ack read ready
       /// 3: wait for CMDQ_DONE
       /// 3: read data
#if ENABLE_DSI_INTERRUPT
       ret = wait_event_interruptible_timeout(_dsi_dcs_read_wait_queue, 
                                                       !_IsEngineBusy(),
                                                       WAIT_TIMEOUT);
        if (0 == ret) {
            xlog_printk(ANDROID_LOG_WARN, "DSI", " Wait for DSI engine read ready timeout!!!\n");

				DSI_DumpRegisters();
				
				///do necessary reset here
				DSI_REG->DSI_RACK.DSI_RACK = 1;
				DSI_Reset();

                return 0;
            }
#else
    #ifdef DDI_DRV_DEBUG_LOG_ENABLE
        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " Start polling DSI read ready!!!\n");
    #endif
        while(DSI_REG->DSI_INTSTA.RD_RDY == 0)  ///read clear
        {
            ///keep polling
            msleep(1);
            read_timeout_ms --;
            
            if(read_timeout_ms == 0)
            {
                DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " Polling DSI read ready timeout!!!\n");
                DSI_DumpRegisters();

                ///do necessary reset here
                DSI_REG->DSI_RACK.DSI_RACK = 1;
                DSI_Reset();
                return 0;
            }
        }
    #ifdef DDI_DRV_DEBUG_LOG_ENABLE
        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " End polling DSI read ready!!!\n");
    #endif

        DSI_REG->DSI_RACK.DSI_RACK = 1;

       while(DSI_REG->DSI_STA.BUSY)
       {
           ///DSI READ ACK HW bug workaround
    #ifdef DDI_DRV_DEBUG_LOG_ENABLE
           DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI is busy: 0x%x !!!\n", DSI_REG->DSI_STA.BUSY);
    #endif
           DSI_REG->DSI_RACK.DSI_RACK = 1;
       }


       ///clear interrupt status
       DSI_REG->DSI_INTSTA.RD_RDY = 1;
	   DSI_REG->DSI_INTSTA.CMD_DONE = 1;
       ///STOP DSI
       OUTREG32(&DSI_REG->DSI_START, 0);

#endif
   
       DSI_REG->DSI_INTEN.RD_RDY =  0;

    #ifdef DDI_DRV_DEBUG_LOG_ENABLE
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_STA : 0x%x \n", DSI_REG->DSI_RX_STA);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_SIZE : 0x%x \n", DSI_REG->DSI_CMDQ_SIZE.CMDQ_SIZE);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_DATA0 : 0x%x \n", DSI_CMDQ_REG->data0[0].byte0);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_DATA1 : 0x%x \n", DSI_CMDQ_REG->data0[0].byte1);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_DATA2 : 0x%x \n", DSI_CMDQ_REG->data0[0].byte2);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_CMDQ_DATA3 : 0x%x \n", DSI_CMDQ_REG->data0[0].byte3);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE0 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE0);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE1 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE1);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE2 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE2);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE3 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE3);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE4 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE4);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE5 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE5);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE6 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE6);
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI_RX_DATA.BYTE7 : 0x%x \n", DSI_REG->DSI_RX_DATA.BYTE7);
    #endif
       packet_type = DSI_REG->DSI_RX_DATA.BYTE0;
           
    #ifdef DDI_DRV_DEBUG_LOG_ENABLE
       DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", " DSI read packet_type is 0x%x \n",packet_type);
    #endif
       if(DSI_REG->DSI_RX_STA.LONG == 1)
       {
           recv_data_cnt = DSI_REG->DSI_RX_DATA.BYTE1 + DSI_REG->DSI_RX_DATA.BYTE2 * 16;
           if(recv_data_cnt > 10)
           {
    #ifdef DDI_DRV_DEBUG_LOG_ENABLE
              DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", " DSI read long packet data  exceeds 4 bytes \n");
    #endif
              recv_data_cnt = 10;
           }

          if(recv_data_cnt > buffer_size)
          {
#ifdef DDI_DRV_DEBUG_LOG_ENABLE
              DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", " DSI read long packet data  exceeds buffer size: %d\n", buffer_size);
#endif
              recv_data_cnt = buffer_size;
           }
           memcpy((void*)buffer, (void*)&DSI_REG->DSI_RX_DATA.BYTE4, recv_data_cnt);
       }
       else
       {
             if(recv_data_cnt > buffer_size)
             {
#ifdef DDI_DRV_DEBUG_LOG_ENABLE
                 DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", " DSI read short packet data  exceeds buffer size: %d\n", buffer_size);
#endif
                 recv_data_cnt = buffer_size;
             }
           memcpy((void*)buffer, (void*)&DSI_REG->DSI_RX_DATA.BYTE1, 2);
       }
    
   }while(packet_type != 0x1C && packet_type != 0x21 && packet_type != 0x22);
   /// here: we may receive a ACK packet which packet type is 0x02 (incdicates some error happened)
   /// therefore we try re-read again until no ACK packet
   /// But: if it is a good way to keep re-trying ???
   
   return recv_data_cnt;
}

UINT32 DSI_read_lcm_reg()
{
    return 0;
}


DSI_STATUS DSI_write_lcm_fb(unsigned int addr, bool long_length)
{
	DSI_T1_INS *t1_tmp=0;
	DSI_CMDQ_CONFG CONFG_tmp;

	CONFG_tmp.type=FB_WRITE;
	CONFG_tmp.BTA=DISABLE_BTA;
	CONFG_tmp.HS=HIGH_SPEED;

	if(long_length)
		CONFG_tmp.CL=CL_16BITS;
	else
		CONFG_tmp.CL=CL_8BITS;		

	CONFG_tmp.TE=DISABLE_TE;
	CONFG_tmp.RPT=DISABLE_RPT;


	t1_tmp->CONFG = *((unsigned char *)(&CONFG_tmp));
	t1_tmp->Data_ID= 0x39;
	t1_tmp->mem_start0 = (addr&0xFF);	

	if(long_length)
		t1_tmp->mem_start1 = ((addr>>8)&0xFF);

	return DSI_Write_T1_INS(t1_tmp);	

	
}


DSI_STATUS DSI_read_lcm_fb(void)
{
	// TBD
	return DSI_STATUS_OK;
}

DSI_STATUS DSI_enable_MIPI_txio(bool en)
{
    if(en)
    {   	
        *(volatile unsigned int *) (INFRA_SYS_CFG_BASE+0x890) |= 0x00000100;    // enable MIPI TX IO    
        mipitx_con0.RG_LNT_LPTX_BIAS_EN = 1;
    }
    else
    {
		// Set MIPI TX pad output low state
		mt_set_gpio_out(GPIO5, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO6, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO7, GPIO_OUT_ZERO);		
		mt_set_gpio_out(GPIO8, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO9, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO11, GPIO_OUT_ZERO);		

		mt_set_gpio_mode(GPIO5, GPIO_MODE_GPIO);
		mt_set_gpio_mode(GPIO6, GPIO_MODE_GPIO);
		mt_set_gpio_mode(GPIO7, GPIO_MODE_GPIO);
		mt_set_gpio_mode(GPIO8, GPIO_MODE_GPIO);
		mt_set_gpio_mode(GPIO9, GPIO_MODE_GPIO);
		mt_set_gpio_mode(GPIO11, GPIO_MODE_GPIO);

		mt_set_gpio_dir(GPIO5, GPIO_DIR_OUT);
		mt_set_gpio_dir(GPIO6, GPIO_DIR_OUT);
		mt_set_gpio_dir(GPIO7, GPIO_DIR_OUT);
		mt_set_gpio_dir(GPIO8, GPIO_DIR_OUT);
		mt_set_gpio_dir(GPIO9, GPIO_DIR_OUT);
		mt_set_gpio_dir(GPIO11, GPIO_DIR_OUT);		
	
        *(volatile unsigned int *) (INFRA_SYS_CFG_BASE+0x890) &= ~0x00000100;   // disable MIPI TX IO   
        mipitx_con0.RG_LNT_LPTX_BIAS_EN = 0;
    }

	DSI_PHY_REG->MIPITX_CON0=mipitx_con0;

	return DSI_STATUS_OK;
}


bool Need_Wait_ULPS(void)
{
	if(((INREG32(DSI_BASE + 0x14C)>> 24) & 0xFF) != 0x04) {

#ifdef DDI_DRV_DEBUG_LOG_ENABLE
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[%s]:true \n", __func__);
#endif
		return TRUE;

	} else {

#ifdef DDI_DRV_DEBUG_LOG_ENABLE
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[%s]:false \n", __func__);
#endif
		return FALSE;

	}

}


DSI_STATUS Wait_ULPS_Mode(void)
{
	DSI_PHY_LCCON_REG lccon_reg=DSI_REG->DSI_PHY_LCCON;
	DSI_PHY_LD0CON_REG ld0con=DSI_REG->DSI_PHY_LD0CON;

	lccon_reg.LC_ULPM_EN =1;
	ld0con.L0_ULPM_EN=1;
	OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&lccon_reg));
	OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&ld0con));

#ifdef DDI_DRV_DEBUG_LOG_ENABLE
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[%s]:enter \n", __func__);
#endif

	while(((INREG32(DSI_BASE + 0x14C)>> 24) & 0xFF) != 0x04)
	{
		lcm_mdelay(5);
#ifdef DDI_DRV_DEBUG_LOG_ENABLE
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI+%04x : 0x%08x \n", DSI_BASE, INREG32(DSI_BASE + 0x14C));
#endif
	}

#ifdef DDI_DRV_DEBUG_LOG_ENABLE
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[%s]:exit \n", __func__);
#endif

	return DSI_STATUS_OK;

}


DSI_STATUS Wait_WakeUp(void)
{
	DSI_PHY_LCCON_REG lccon_reg=DSI_REG->DSI_PHY_LCCON;
	DSI_PHY_LD0CON_REG ld0con=DSI_REG->DSI_PHY_LD0CON;

#ifdef DDI_DRV_DEBUG_LOG_ENABLE
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[%s]:enter \n", __func__);
#endif

	lccon_reg.LC_ULPM_EN =0;
	ld0con.L0_ULPM_EN=0;
	OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&lccon_reg));
	OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&ld0con));

	lcm_mdelay(1);//Wait 1ms for LCM Spec

	lccon_reg.LC_WAKEUP_EN =1;
	ld0con.L0_WAKEUP_EN=1;
	OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&lccon_reg));
	OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&ld0con));

	while(((INREG32(DSI_BASE + 0x148)>> 8) & 0xFF) != 0x01)
	{
		lcm_mdelay(5);
#ifdef DDI_DRV_DEBUG_LOG_ENABLE
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[soso]DSI+%04x : 0x%08x \n", DSI_BASE, INREG32(DSI_BASE + 0x148));
#endif
	}

#ifdef DDI_DRV_DEBUG_LOG_ENABLE
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "[%s]:exit \n", __func__);
#endif

	lccon_reg.LC_WAKEUP_EN =0;
	ld0con.L0_WAKEUP_EN=0;
	OUTREG32(&DSI_REG->DSI_PHY_LCCON, AS_UINT32(&lccon_reg));
	OUTREG32(&DSI_REG->DSI_PHY_LD0CON, AS_UINT32(&ld0con));

	return DSI_STATUS_OK;

}

// -------------------- Retrieve Information --------------------

DSI_STATUS DSI_DumpRegisters(void)
{
    UINT32 i;

    DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "---------- Start dump DSI registers ----------\n");
    
    for (i = 0; i < sizeof(DSI_REGS); i += 4)
    {
        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI+%04x : 0x%08x\n", i, INREG32(DSI_BASE + i));
    }

    for (i = 0; i < sizeof(DSI_PHY_REGS); i += 4)
    {
        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI_PHY+%04x(%p) : 0x%08x\n", i, (UINT32*)(MIPI_CONFG_BASE+0x800+i), INREG32((MIPI_CONFG_BASE+0x800+i)));
    }

    return DSI_STATUS_OK;
}


static LCM_PARAMS lcm_params_for_clk_setting;


DSI_STATUS DSI_FMDesense_Query(void)
{
    return DSI_STATUS_OK;
}

DSI_STATUS DSI_FM_Desense(unsigned long freq)
{
    ///need check
    DSI_Change_CLK(freq);
    return DSI_STATUS_OK;
}

DSI_STATUS DSI_Reset_CLK(void)
{
    extern LCM_PARAMS *lcm_params;

    _WaitForEngineNotBusy();
	DSI_PHY_TIMCONFIG(lcm_params);
	DSI_PHY_clk_setting(lcm_params->dsi.pll_div1, lcm_params->dsi.pll_div2, lcm_params->dsi.LANE_NUM);
	return DSI_STATUS_OK;
}

DSI_STATUS DSI_Get_Default_CLK(unsigned int *clk)
{
    extern LCM_PARAMS *lcm_params;
	unsigned int div2_real = lcm_params->dsi.pll_div2 ? lcm_params->dsi.pll_div2 : 0x1;

    *clk = 13 * (lcm_params->dsi.pll_div1 + 1) / div2_real;
    return DSI_STATUS_OK;
}

DSI_STATUS DSI_Get_Current_CLK(unsigned int *clk)
{
    if(mipitx_con1.RG_PLL_DIV2 == 0)
        *clk = 26 * (mipitx_con1.RG_PLL_DIV1 + 1);
    else
        *clk = 13 * (mipitx_con1.RG_PLL_DIV1 + 1) / mipitx_con1.RG_PLL_DIV2;

    return DSI_STATUS_OK;
}

DSI_STATUS DSI_Change_CLK(unsigned int clk)
{
    extern LCM_PARAMS *lcm_params;

    if(clk > 1000)
        return DSI_STATUS_ERROR;
    memcpy((void *)&lcm_params_for_clk_setting, (void *)lcm_params, sizeof(LCM_PARAMS));

    for(lcm_params_for_clk_setting.dsi.pll_div2 = 15; lcm_params_for_clk_setting.dsi.pll_div2 > 0; lcm_params_for_clk_setting.dsi.pll_div2--)
    {
        for(lcm_params_for_clk_setting.dsi.pll_div1 = 0; lcm_params_for_clk_setting.dsi.pll_div1 < 39; lcm_params_for_clk_setting.dsi.pll_div1++)
        {
            if((13 * (lcm_params_for_clk_setting.dsi.pll_div1 + 1) / lcm_params_for_clk_setting.dsi.pll_div2) >= clk)
                goto end;
        }
    }

    if(lcm_params_for_clk_setting.dsi.pll_div2 == 0)
    {
        for(lcm_params_for_clk_setting.dsi.pll_div1 = 0; lcm_params_for_clk_setting.dsi.pll_div1 < 39; lcm_params_for_clk_setting.dsi.pll_div1++)
        {
            if((26 * (lcm_params_for_clk_setting.dsi.pll_div1 + 1)) >= clk)
                goto end;
        }
    }

end:
    _WaitForEngineNotBusy();
	DSI_PHY_TIMCONFIG(&lcm_params_for_clk_setting);
	DSI_PHY_clk_setting(lcm_params_for_clk_setting.dsi.pll_div1, lcm_params_for_clk_setting.dsi.pll_div2, lcm_params_for_clk_setting.dsi.LANE_NUM);

    return DSI_STATUS_OK;
}


