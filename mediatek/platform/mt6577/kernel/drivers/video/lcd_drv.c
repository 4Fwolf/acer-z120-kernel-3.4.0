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
#if defined(BUILD_UBOOT)
#define ENABLE_LCD_INTERRUPT 0 

#include <asm/arch/disp_drv_platform.h>
#else
    #if defined(CONFIG_MT6577_FPGA)
        #define ENABLE_LCD_INTERRUPT 0
    #else
        #define ENABLE_LCD_INTERRUPT 1
    #endif
#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_irq.h>
#include <mach/mt_clock_manager.h>

#include "lcd_reg.h"
#include "lcd_drv.h"
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <disp_drv_log.h>
#include <linux/dma-mapping.h>
#include "dsi_drv.h"

#ifdef MTK_M4U_SUPPORT
#include <mach/m4u.h>
#endif
#include <linux/hrtimer.h>
#if ENABLE_LCD_INTERRUPT
//#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
//#include <asm/tcm.h>
#include <mach/irqs.h>
static wait_queue_head_t _lcd_wait_queue;
#endif
static wait_queue_head_t _vsync_wait_queue;
atomic_t lcd_vsync = ATOMIC_INIT(0);
atomic_t wait_lcd_vsync = ATOMIC_INIT(0);
static struct hrtimer hrtimer_vsync;
#define VSYNC_US_TO_NS(x) (x * 1000)
unsigned int vsync_timer = 0;
unsigned int vsync_cnt = 0;
unsigned int timer_cnt = 0;
unsigned int te_cnt = 0;
#include "debug.h"
#include <asm/current.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#endif
#define LCD_OUTREG32(addr, data)	\
		{\
		OUTREG32(addr, data);}

#define LCD_OUTREG8(addr, data)	\
		{\
		OUTREG8(addr, data);}


#define LCD_OUTREG16(addr, data)	\
		{\
		OUTREG16(addr, data);}

#define LCD_MASKREG32(addr, mask, data)	\
		{\
		MASKREG32(addr, mask, data);}

#if defined(MTK_M4U_SUPPORT)
	M4U_EXPORT_FUNCTION_STRUCT _m4u_lcdc_func = {0};
	EXPORT_SYMBOL(_m4u_lcdc_func);
#endif

static size_t dbi_log_on = false;
#define DBI_LOG(fmt, arg...) \
    do { \
        if (dbi_log_on) DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", fmt, ##arg);    \
    }while (0)

#define DBI_FUNC()	\
	do { \
		if(dbi_log_on) DISP_LOG_PRINT(ANDROID_LOG_INFO, "LCD", "[Func]%s\n", __func__);  \
	}while (0)

void dbi_log_enable(int enable)
{
    dbi_log_on = enable;
	DBI_LOG("lcd log %s\n", enable?"enabled":"disabled");
}

static PLCD_REGS const LCD_REG = (PLCD_REGS)(LCD_BASE);
static const UINT32 TO_BPP[LCD_FB_FORMAT_NUM] = {2, 3, 4};
unsigned int wait_time = 0;
typedef struct
{
    LCD_FB_FORMAT fbFormat;
    UINT32 fbPitchInBytes;
    LCD_REG_SIZE roiWndSize;
    LCD_OUTPUT_MODE outputMode;
    LCD_REGS regBackup;
    void (*pIntCallback)(DISP_INTERRUPT_EVENTS);
} LCD_CONTEXT;

static LCD_CONTEXT _lcdContext = {0};
static int wst_step_LCD = -1;//for LCD&FM de-sense
static bool is_get_default_write_cycle = FALSE;
static unsigned int default_write_cycle = 0;
static UINT32 default_wst = 0;
static bool limit_w2m_speed = false;
static bool limit_w2tvr_speed = false;
static bool lcd_esd_check = false;
#ifdef BUILD_UBOOT
UINT32 const NLI_ARB_CS = 0xC100d014;
UINT32 const DPI_PAD_CON = 0xC2080900;

static UINT32 const INFRA_DRV5 = 0xC0001814;
static UINT32 const INFRA_DRV6 = 0xC0001818;
static UINT32 const INFRA_DRV12 = 0xC0001830;
#else
static UINT32 const NLI_ARB_CS = 0xf100d014;
static UINT32 const DPI_PAD_CON = 0xf2080900;

static UINT32 const INFRA_DRV5 = 0xF0001814;
static UINT32 const INFRA_DRV6 = 0xF0001818;
static UINT32 const INFRA_DRV12 = 0xF0001830;
#endif
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
LCD_STATUS LCD_Init_IO_pad(LCM_PARAMS *lcm_params)
{
	ASSERT(lcm_params->dbi.port < 2);
	if(lcm_params->type == LCM_TYPE_DBI){
		MASKREG32(NLI_ARB_CS, 0xF << (lcm_params->dbi.port << 2), lcm_params->dbi.port << (lcm_params->dbi.port << 2)); // NLI_ARB chip select
		MASKREG32(NLI_ARB_CS, 0x00040000, lcm_params->io_select_mode << 18); // NLI_ARB chip select
		if(1 == lcm_params->io_select_mode){// if DBI share DPI data pin
			MASKREG32(DPI_PAD_CON, 0x00000001, 0x00000000);// DPI PAD Mode should be other mode, not DPI mode
		}
	}
	else if(lcm_params->type == LCM_TYPE_DPI){
		MASKREG32(DPI_PAD_CON, 0x00000001, 0x00000001);// DPI mode
		MASKREG32(DPI_PAD_CON, 0x00000006, (lcm_params->dpi.i2x_en << 1) | (lcm_params->dpi.i2x_edge << 2));// DPI mode
	}
	else{}//DSI do nothing
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_Set_DrivingCurrent(LCM_PARAMS *lcm_params)
{
	if(1 == lcm_params->io_select_mode){// if DBI share DPI data pin
		MASKREG32(INFRA_DRV12, 0x07777000,(((lcm_params->dbi.io_driving_current >> 4) - 1) << 24)|
										  (((lcm_params->dbi.io_driving_current >> 4) - 1) << 20)|
										  (((lcm_params->dbi.io_driving_current >> 4) - 1) << 16)|
										  (((lcm_params->dbi.io_driving_current >> 4) - 1) << 12));
	}
	else{
		MASKREG32(INFRA_DRV6, 0x000000777,(((lcm_params->dbi.io_driving_current >> 4) - 1) << 0)|
										  (((lcm_params->dbi.io_driving_current >> 4) - 1) << 4)|
										  (((lcm_params->dbi.io_driving_current >> 4) - 1) << 8));
	}

	MASKREG32(INFRA_DRV5, 0x77770000,(((lcm_params->dbi.io_driving_current >> 4) - 1) << 28)|
										  (((lcm_params->dbi.io_driving_current >> 4) - 1) << 24)|
										  (((lcm_params->dbi.io_driving_current >> 4) - 1) << 20)|
										  (((lcm_params->dbi.io_driving_current >> 4) - 1) << 16));
	return LCD_STATUS_OK;
}

#if ENABLE_LCD_INTERRUPT
static irqreturn_t _LCD_InterruptHandler(int irq, void *dev_id)
{   
    LCD_REG_INTERRUPT status = LCD_REG->INT_STATUS;

    if (status.COMPLETED)
    {
#ifdef CONFIG_MTPROF_APPLAUNCH  // eng enable, user disable   	
        LOG_PRINT(ANDROID_LOG_INFO, "AppLaunch", "LCD frame buffer update done !\n");
#endif
        wake_up_interruptible(&_lcd_wait_queue);

        if(_lcdContext.pIntCallback)
            _lcdContext.pIntCallback(DISP_LCD_TRANSFER_COMPLETE_INT);

        DBG_OnLcdDone();
    }

    if (status.SYNC)// this is TE mode 0 interrupt
    {
        if(_lcdContext.pIntCallback)
            _lcdContext.pIntCallback(DISP_LCD_SYNC_INT);
#ifndef BUILD_UBOOT    
		if(atomic_read(&wait_lcd_vsync))//judge if wait vsync
		{
			if(-1 != hrtimer_try_to_cancel(&hrtimer_vsync)){
				atomic_set(&wait_lcd_vsync, 0);
				atomic_set(&lcd_vsync, 1);
			
				hrtimer_start(&hrtimer_vsync, ktime_set(0, VSYNC_US_TO_NS(vsync_timer)), HRTIMER_MODE_REL);
				wake_up_interruptible(&_vsync_wait_queue);
				te_cnt++;
//				printk("TE signal, cancel timer\n");
			}
//			printk("TE signal, and wake up\n");
		}
#endif         
		DBG_OnTeDelayDone();
		lcd_esd_check = false;
    }
#if 0  //TE mode 1
	if(status.HTT)
	{
        if(_lcdContext.pIntCallback)
            _lcdContext.pIntCallback(DISP_LCD_HTT_INT);
        
		DBG_OnTeDelayDone();
	}
#endif
	LCD_OUTREG32(&LCD_REG->INT_STATUS, 0);
    return IRQ_HANDLED;
}
#endif


static BOOL _IsEngineBusy(void)
{
    LCD_REG_STATUS status;

    status = LCD_REG->STATUS;
    if (status.RUN || 
        status.WAIT_CMDQ ||  
        status.WAIT_HTT|| 
        status.WAIT_SYNC || 
        status.BUSY ||
        status.GMC) 
        return TRUE;

    return FALSE;
}


BOOL LCD_IsBusy(void)
{
	return _IsEngineBusy();
}


static void _WaitForEngineNotBusy(void)
{
#if ENABLE_LCD_INTERRUPT
    static const long WAIT_TIMEOUT = 2 * HZ;    // 2 sec
    if (in_interrupt())
    {
        // perform busy waiting if in interrupt context
        while(_IsEngineBusy()) {}
    }
    else
    {
        while (_IsEngineBusy())
        {
            long ret = wait_event_interruptible_timeout(_lcd_wait_queue, 
                                                        !_IsEngineBusy(),
                                                        WAIT_TIMEOUT);
            if (0 == ret) {
				LCD_REG_DSI_DC tmp_reg;
				if(_IsEngineBusy()){
                	DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", "[WARNING] Wait for LCD engine not busy timeout, LCD Hang, %s!!!\n", current->comm); 
					LCD_DumpRegisters();
					tmp_reg = LCD_REG->DS_DSI_CON;
					if(LCD_REG->STATUS.WAIT_SYNC){
                    	DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", "reason is LCD can't wait TE signal!!!\n"); 
						LCD_TE_Enable(FALSE);
					// Can't receive VSYNC, RX may be dead.
					//DSI_handle_esd_recovery(METHOD_LCM_VSYNC);
					lcd_esd_check = true;
					}
					OUTREG16(&LCD_REG->START, 0);
					OUTREG16(&LCD_REG->START, 0x1);
					if(1 == tmp_reg.DC_DSI){
						DSI_DumpRegisters();
						DSI_Reset();
					}
//					OUTREG16(&LCD_REG->START, 0);
//					OUTREG16(&LCD_REG->START, 0x8000);
				}
				else
                	DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", "[WARNING] Wait for LCD engine not busy timeout, LCD not Hang but no IRQ!!!, %s\n", current->comm); 
            }
        }
    }
#else
    while(_IsEngineBusy()) {
	    udelay(50);//sleep 1ms
		wait_time++;
		if(wait_time>20000){ //timeout
			LCD_REG_DSI_DC tmp_reg;
			printk("[WARNING] Wait for LCD engine not busy timeout!!!\n");
			LCD_DumpRegisters();
			tmp_reg = LCD_REG->DS_DSI_CON;
			if(LCD_REG->STATUS.WAIT_SYNC){
				printk("reason is LCD can't wait TE signal!!!\n");
				LCD_TE_Enable(FALSE);
			}
			OUTREG16(&LCD_REG->START, 0);
			OUTREG16(&LCD_REG->START, 0x1);
			OUTREG16(&LCD_REG->START, 0);
			OUTREG16(&LCD_REG->START, 0x8000);
	
			if(1 == tmp_reg.DC_DSI){
				DSI_DumpRegisters();
				DSI_Reset();
				DSI_clk_HS_mode(1);
				DSI_CHECK_RET(DSI_EnableClk());
			}
			wait_time = 0;
		}
    }
    wait_time = 0;
#endif    
}

unsigned int vsync_wait_time = 0;
void LCD_WaitTE(void)
{
#ifndef BUILD_UBOOT	
	atomic_set(&wait_lcd_vsync, 1);
//	hrtimer_start(&hrtimer_vsync, ktime_set(0, VSYNC_US_TO_NS(vsync_timer)), HRTIMER_MODE_REL);
	wait_event_interruptible(_vsync_wait_queue, atomic_read(&lcd_vsync));
	atomic_set(&lcd_vsync, 0);
	vsync_cnt++;
//	atomic_set(&wait_lcd_vsync, 0);
#endif
}

#ifndef BUILD_UBOOT
void LCD_GetVsyncCnt()
{
	printk("Vsync_call_count = %d, timer_vsync_count = %d, te_count = %d\n", vsync_cnt, timer_cnt, te_cnt);
}

enum hrtimer_restart lcd_te_hrtimer_func(struct hrtimer *timer)
{
	long long ret;
	if(atomic_read(&wait_lcd_vsync))
	{
		atomic_set(&wait_lcd_vsync, 0);
		atomic_set(&lcd_vsync, 1);
		wake_up_interruptible(&_vsync_wait_queue);
		timer_cnt++;
//		printk("hrtimer Vsync, and wake up\n");
	}
	ret = hrtimer_forward_now(timer, ktime_set(0, VSYNC_US_TO_NS(vsync_timer)));
    return HRTIMER_RESTART;
}
#endif
void LCD_InitVSYNC(unsigned int vsync_interval)
{
#ifndef BUILD_UBOOT
    ktime_t ktime;
	vsync_timer = vsync_interval;
	ktime = ktime_set(0, VSYNC_US_TO_NS(vsync_timer));
	hrtimer_init(&hrtimer_vsync, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_vsync.function = lcd_te_hrtimer_func;
	hrtimer_start(&hrtimer_vsync, ktime, HRTIMER_MODE_REL);
#endif
}

void LCD_PauseVSYNC(bool enable)
{
#ifndef BUILD_UBOOT
	if(enable){
		if(0 <= hrtimer_cancel(&hrtimer_vsync)){
			printk("[LCD_PauseVSYNC], cancel vsync timer]\n");
			if(atomic_read(&wait_lcd_vsync)){
				atomic_set(&wait_lcd_vsync, 0);
				atomic_set(&lcd_vsync, 1);
				wake_up_interruptible(&_vsync_wait_queue);
				timer_cnt++;
				printk("[LCD_PauseVSYNC], wait vsync so need wakeup event to avoid hang\n");
			}
		}
	}
	else{
		printk("[LCD_PauseVSYNC], restart vsync timer]\n");
		hrtimer_start(&hrtimer_vsync, ktime_set(0, VSYNC_US_TO_NS(vsync_timer)), HRTIMER_MODE_REL);
	}
#endif
}

static void _BackupLCDRegisters(void)
{
//    memcpy((void*)&(_lcdContext.regBackup), (void*)LCD_REG, sizeof(LCD_REGS));
    LCD_REGS *regs = &(_lcdContext.regBackup);
    UINT32 i;
    LCD_OUTREG32(&regs->INT_ENABLE, AS_UINT32(&LCD_REG->INT_ENABLE));
    LCD_OUTREG32(&regs->SERIAL_CFG, AS_UINT32(&LCD_REG->SERIAL_CFG));

	for(i = 0; i < ARY_SIZE(LCD_REG->SIF_TIMING); ++i)
    {
        LCD_OUTREG32(&regs->SIF_TIMING[i], AS_UINT32(&LCD_REG->SIF_TIMING[i]));
    }

    for(i = 0; i < ARY_SIZE(LCD_REG->PARALLEL_CFG); ++i)
    {
        LCD_OUTREG32(&regs->PARALLEL_CFG[i], AS_UINT32(&LCD_REG->PARALLEL_CFG[i]));
    }

    LCD_OUTREG32(&regs->TEARING_CFG, AS_UINT32(&LCD_REG->TEARING_CFG));
    LCD_OUTREG32(&regs->PARALLEL_DW, AS_UINT32(&LCD_REG->PARALLEL_DW));
	LCD_OUTREG32(&regs->CALC_HTT, AS_UINT32(&LCD_REG->CALC_HTT));
	LCD_OUTREG32(&regs->SYNC_LCM_SIZE, AS_UINT32(&LCD_REG->SYNC_LCM_SIZE));
	LCD_OUTREG32(&regs->SYNC_CNT, AS_UINT32(&LCD_REG->SYNC_CNT));
	LCD_OUTREG32(&regs->GMC_CON, AS_UINT32(&LCD_REG->GMC_CON));

    for(i = 0; i < ARY_SIZE(LCD_REG->WROI_W2M_ADDR); ++i)
    {
        LCD_OUTREG32(&regs->WROI_W2M_ADDR[i], AS_UINT32(&LCD_REG->WROI_W2M_ADDR[i]));
    }

	LCD_OUTREG32(&regs->W2M_PITCH, AS_UINT32(&LCD_REG->W2M_PITCH));
    LCD_OUTREG32(&regs->WROI_W2M_OFFSET, AS_UINT32(&LCD_REG->WROI_W2M_OFFSET));
    LCD_OUTREG32(&regs->WROI_W2M_CONTROL, AS_UINT32(&LCD_REG->WROI_W2M_CONTROL));
    LCD_OUTREG32(&regs->WROI_CONTROL, AS_UINT32(&LCD_REG->WROI_CONTROL));
    LCD_OUTREG32(&regs->WROI_OFFSET, AS_UINT32(&LCD_REG->WROI_OFFSET));
    LCD_OUTREG32(&regs->WROI_CMD_ADDR, AS_UINT32(&LCD_REG->WROI_CMD_ADDR));
    LCD_OUTREG32(&regs->WROI_DATA_ADDR, AS_UINT32(&LCD_REG->WROI_DATA_ADDR));
    LCD_OUTREG32(&regs->WROI_SIZE, AS_UINT32(&LCD_REG->WROI_SIZE));
    LCD_OUTREG32(&regs->WROI_HW_REFRESH, AS_UINT32(&LCD_REG->WROI_HW_REFRESH));
    LCD_OUTREG32(&regs->WROI_DC, AS_UINT32(&LCD_REG->WROI_DC));
    LCD_OUTREG32(&regs->WROI_BG_COLOR, AS_UINT32(&LCD_REG->WROI_BG_COLOR));
    LCD_OUTREG32(&regs->DS_DSI_CON, AS_UINT32(&LCD_REG->DS_DSI_CON));

    for(i = 0; i < ARY_SIZE(LCD_REG->LAYER); ++i)
    {
        LCD_OUTREG32(&regs->LAYER[i].CONTROL, AS_UINT32(&LCD_REG->LAYER[i].CONTROL));
        LCD_OUTREG32(&regs->LAYER[i].COLORKEY, AS_UINT32(&LCD_REG->LAYER[i].COLORKEY));
        LCD_OUTREG32(&regs->LAYER[i].OFFSET, AS_UINT32(&LCD_REG->LAYER[i].OFFSET));
        LCD_OUTREG32(&regs->LAYER[i].ADDRESS, AS_UINT32(&LCD_REG->LAYER[i].ADDRESS));
        LCD_OUTREG32(&regs->LAYER[i].SIZE, AS_UINT32(&LCD_REG->LAYER[i].SIZE));
        LCD_OUTREG32(&regs->LAYER[i].SCRL_OFFSET, AS_UINT32(&LCD_REG->LAYER[i].SCRL_OFFSET));
        LCD_OUTREG32(&regs->LAYER[i].WINDOW_OFFSET, AS_UINT32(&LCD_REG->LAYER[i].WINDOW_OFFSET));
        LCD_OUTREG32(&regs->LAYER[i].WINDOW_PITCH, AS_UINT32(&LCD_REG->LAYER[i].WINDOW_PITCH));
        LCD_OUTREG32(&regs->LAYER[i].DB_ADD, AS_UINT32(&LCD_REG->LAYER[i].DB_ADD));
    }

	LCD_OUTREG32(&regs->DITHER_CON, AS_UINT32(&LCD_REG->DITHER_CON));
    
	for(i = 0; i < ARY_SIZE(LCD_REG->COEF_ROW); ++i)
    {
        LCD_OUTREG32(&regs->COEF_ROW[i], AS_UINT32(&LCD_REG->COEF_ROW[i]));
    }

    for(i = 0; i < ARY_SIZE(LCD_REG->GAMMA); ++i)
    {
        LCD_OUTREG32(&regs->GAMMA[i], AS_UINT32(&LCD_REG->GAMMA[i]));
    }
}


static void _RestoreLCDRegisters(void)
{
    LCD_REGS *regs = &(_lcdContext.regBackup);
    UINT32 i;
    LCD_OUTREG32(&LCD_REG->INT_ENABLE, AS_UINT32(&regs->INT_ENABLE));
    LCD_OUTREG32(&LCD_REG->SERIAL_CFG, AS_UINT32(&regs->SERIAL_CFG));

	for(i = 0; i < ARY_SIZE(LCD_REG->SIF_TIMING); ++i)
    {
        LCD_OUTREG32(&LCD_REG->SIF_TIMING[i], AS_UINT32(&regs->SIF_TIMING[i]));
    }

    for(i = 0; i < ARY_SIZE(LCD_REG->PARALLEL_CFG); ++i)
    {
        LCD_OUTREG32(&LCD_REG->PARALLEL_CFG[i], AS_UINT32(&regs->PARALLEL_CFG[i]));
    }

    LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&regs->TEARING_CFG));
    LCD_OUTREG32(&LCD_REG->PARALLEL_DW, AS_UINT32(&regs->PARALLEL_DW));
	LCD_OUTREG32(&LCD_REG->CALC_HTT, AS_UINT32(&regs->CALC_HTT));
	LCD_OUTREG32(&LCD_REG->SYNC_LCM_SIZE, AS_UINT32(&regs->SYNC_LCM_SIZE));
	LCD_OUTREG32(&LCD_REG->SYNC_CNT, AS_UINT32(&regs->SYNC_CNT));
	LCD_OUTREG32(&LCD_REG->GMC_CON, AS_UINT32(&regs->GMC_CON));

    for(i = 0; i < ARY_SIZE(LCD_REG->WROI_W2M_ADDR); ++i)
    {
        LCD_OUTREG32(&LCD_REG->WROI_W2M_ADDR[i], AS_UINT32(&regs->WROI_W2M_ADDR[i]));
    }
	
	LCD_OUTREG32(&LCD_REG->W2M_PITCH, AS_UINT32(&regs->W2M_PITCH));
    LCD_OUTREG32(&LCD_REG->WROI_W2M_OFFSET, AS_UINT32(&regs->WROI_W2M_OFFSET));
    LCD_OUTREG32(&LCD_REG->WROI_W2M_CONTROL, AS_UINT32(&regs->WROI_W2M_CONTROL));
    LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&regs->WROI_CONTROL));
    LCD_OUTREG32(&LCD_REG->WROI_OFFSET, AS_UINT32(&regs->WROI_OFFSET));
    LCD_OUTREG32(&LCD_REG->WROI_CMD_ADDR, AS_UINT32(&regs->WROI_CMD_ADDR));
    LCD_OUTREG32(&LCD_REG->WROI_DATA_ADDR, AS_UINT32(&regs->WROI_DATA_ADDR));
    LCD_OUTREG32(&LCD_REG->WROI_SIZE, AS_UINT32(&regs->WROI_SIZE));
    LCD_OUTREG32(&LCD_REG->WROI_HW_REFRESH, AS_UINT32(&regs->WROI_HW_REFRESH));
    LCD_OUTREG32(&LCD_REG->WROI_DC, AS_UINT32(&regs->WROI_DC));
    LCD_OUTREG32(&LCD_REG->WROI_BG_COLOR, AS_UINT32(&regs->WROI_BG_COLOR));
    LCD_OUTREG32(&LCD_REG->DS_DSI_CON, AS_UINT32(&regs->DS_DSI_CON));

    for(i = 0; i < ARY_SIZE(LCD_REG->LAYER); ++i)
    {
        LCD_OUTREG32(&LCD_REG->LAYER[i].CONTROL, AS_UINT32(&regs->LAYER[i].CONTROL));
        LCD_OUTREG32(&LCD_REG->LAYER[i].COLORKEY, AS_UINT32(&regs->LAYER[i].COLORKEY));
        LCD_OUTREG32(&LCD_REG->LAYER[i].OFFSET, AS_UINT32(&regs->LAYER[i].OFFSET));
        LCD_OUTREG32(&LCD_REG->LAYER[i].ADDRESS, AS_UINT32(&regs->LAYER[i].ADDRESS));
        LCD_OUTREG32(&LCD_REG->LAYER[i].SIZE, AS_UINT32(&regs->LAYER[i].SIZE));
        LCD_OUTREG32(&LCD_REG->LAYER[i].SCRL_OFFSET, AS_UINT32(&regs->LAYER[i].SCRL_OFFSET));
        LCD_OUTREG32(&LCD_REG->LAYER[i].WINDOW_OFFSET, AS_UINT32(&regs->LAYER[i].WINDOW_OFFSET));
        LCD_OUTREG32(&LCD_REG->LAYER[i].WINDOW_PITCH, AS_UINT32(&regs->LAYER[i].WINDOW_PITCH));
        LCD_OUTREG32(&LCD_REG->LAYER[i].DB_ADD, AS_UINT32(&regs->LAYER[i].DB_ADD));
    }

	LCD_OUTREG32(&LCD_REG->DITHER_CON, AS_UINT32(&regs->DITHER_CON));

    for(i = 0; i < ARY_SIZE(LCD_REG->COEF_ROW); ++i)
    {
        LCD_OUTREG32(&LCD_REG->COEF_ROW[i], AS_UINT32(&regs->COEF_ROW[i]));
    }

    for(i = 0; i < ARY_SIZE(LCD_REG->GAMMA); ++i)
    {
        LCD_OUTREG32(&LCD_REG->GAMMA[i], AS_UINT32(&regs->GAMMA[i]));
    }
}


static void _ResetBackupedLCDRegisterValues(void)
{
    LCD_REGS *regs = &_lcdContext.regBackup;
    memset((void*)regs, 0, sizeof(LCD_REGS));

    LCD_OUTREG32(&regs->SERIAL_CFG, 0x00003000);
    LCD_OUTREG32(&regs->PARALLEL_CFG[0], 0x00FC0000);
    LCD_OUTREG32(&regs->PARALLEL_CFG[1], 0x00300000);
    LCD_OUTREG32(&regs->PARALLEL_CFG[2], 0x00300000);
}


// ---------------------------------------------------------------------------
//  LCD Controller API Implementations
// ---------------------------------------------------------------------------

LCD_STATUS LCD_Init(void)
{
    LCD_STATUS ret = LCD_STATUS_OK;

    memset(&_lcdContext, 0, sizeof(_lcdContext));

    // LCD controller would NOT reset register as default values
    // Do it by SW here
    //
    _ResetBackupedLCDRegisterValues();

    ret = LCD_PowerOn();

	LCD_OUTREG32(&LCD_REG->SYNC_LCM_SIZE, 0x00010001);
	LCD_OUTREG32(&LCD_REG->SYNC_CNT, 0x1);

    ASSERT(ret == LCD_STATUS_OK);

#if ENABLE_LCD_INTERRUPT
    if (request_irq(MT_LCD_IRQ_ID,
        _LCD_InterruptHandler, IRQF_TRIGGER_LOW, "mtklcd", NULL) < 0)
    {
        DBI_LOG("[LCD][ERROR] fail to request LCD irq\n"); 
        return LCD_STATUS_ERROR;
    }
//	mt65xx_irq_unmask(MT_LCD_IRQ_ID);
//	enable_irq(MT_LCD_IRQ_ID);
    init_waitqueue_head(&_lcd_wait_queue);
    init_waitqueue_head(&_vsync_wait_queue);
    LCD_REG->INT_ENABLE.COMPLETED = 1;
//	LCD_REG->INT_ENABLE.REG_COMPLETED = 1;
	LCD_REG->INT_ENABLE.CMDQ_COMPLETED = 1;
	LCD_REG->INT_ENABLE.HTT = 1;
    LCD_REG->INT_ENABLE.SYNC = 1;
#endif
    
    return LCD_STATUS_OK;
}


LCD_STATUS LCD_Deinit(void)
{
    LCD_STATUS ret = LCD_PowerOff();

    ASSERT(ret == LCD_STATUS_OK);

    return LCD_STATUS_OK;
}

static BOOL s_isLcdPowerOn = FALSE;

#ifdef BUILD_UBOOT
LCD_STATUS LCD_PowerOn(void)
{
    if (!s_isLcdPowerOn)
    {
		LCD_MASKREG32(0xC0001040, 0x02, 0x0);
		LCD_MASKREG32(0xC2080020, 0x80, 0x80);
		LCD_MASKREG32(0xC2080028, 0x00000280, 0x00000280);
		LCD_MASKREG32(0xC2080024, 0x00000020, 0x00000020);
		printf("0x%8x,0x%8x,0x%8x\n", INREG32(0xC2080000), INREG32(0xC2080004), INREG32(0xC2080008));
        _RestoreLCDRegisters();
        s_isLcdPowerOn = TRUE;
    }

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_PowerOff(void)
{
    if (s_isLcdPowerOn)
    {
        int ret = 1;
        _WaitForEngineNotBusy();
        _BackupLCDRegisters();
#if 1   // FIXME
		LCD_MASKREG32(0xC0001040, 0x02, 0x02);
		LCD_MASKREG32(0xC2080010, 0x80, 0x80);
		LCD_MASKREG32(0xC2080018, 0x00000280, 0x00000280);
		LCD_MASKREG32(0xC2080014, 0x00000020, 0x00000020);
		printf("0x%8x,0x%8x,0x%8x\n", INREG32(0xC2080000), INREG32(0xC2080004), INREG32(0xC2080008));
#endif        
        s_isLcdPowerOn = FALSE;
    }

    return LCD_STATUS_OK;
}

#else
LCD_STATUS LCD_PowerOn(void)
{
    DBI_FUNC();
#ifndef CONFIG_MT6577_FPGA
#if 1
    if (!s_isLcdPowerOn)
    {
		int ret = 0;
#if 1   // FIXME
	DBI_LOG("lcd will be power on\n");
      	ret = enable_clock(MT65XX_PDN_MM_SMI_LARB1_EMI, "LCD");
		ret += enable_clock(MT65XX_PDN_MM_SMI_LARB1, "LCD");
		ret += enable_clock(MT65XX_PDN_MM_LCD_EMI, "LCD");
		ret += enable_clock(MT65XX_PDN_MM_LCD, "LCD");
		if(ret > 0)
		{
			DBI_LOG("[LCD]power manager API return FALSE\n");
		}
#endif        
        _RestoreLCDRegisters();
        s_isLcdPowerOn = TRUE;
    }

#endif
#endif
    return LCD_STATUS_OK;
}


LCD_STATUS LCD_PowerOff(void)
{
    DBI_FUNC();
#ifndef CONFIG_MT6577_FPGA
    if (s_isLcdPowerOn)
    {
        int ret = 1;
        _WaitForEngineNotBusy();
        _BackupLCDRegisters();
	DBI_LOG("lcd will be power off\n");
#if 1   // FIXME
        ret = disable_clock(MT65XX_PDN_MM_LCD, "LCD");
		ret += disable_clock(MT65XX_PDN_MM_LCD_EMI, "LCD");
		ret += disable_clock(MT65XX_PDN_MM_SMI_LARB1, "LCD");
		ret += disable_clock(MT65XX_PDN_MM_SMI_LARB1_EMI, "LCD");
		if(ret > 0)
		{
			DBI_LOG("[LCD]power manager API return FALSE\n");
		}
#endif        
        s_isLcdPowerOn = FALSE;
    }
#endif
    return LCD_STATUS_OK;
}
#endif

LCD_STATUS LCD_WaitForNotBusy(void)
{
    _WaitForEngineNotBusy();
    return LCD_STATUS_OK;
}
EXPORT_SYMBOL(LCD_WaitForNotBusy);


LCD_STATUS LCD_EnableInterrupt(DISP_INTERRUPT_EVENTS eventID)
{
#if ENABLE_LCD_INTERRUPT
    switch(eventID)
    {
        case DISP_LCD_TRANSFER_COMPLETE_INT:
            LCD_REG->INT_ENABLE.COMPLETED = 1;
            break;
        case DISP_LCD_REG_COMPLETE_INT:
            LCD_REG->INT_ENABLE.REG_COMPLETED = 1;
            break;
        case DISP_LCD_CDMQ_COMPLETE_INT:
            LCD_REG->INT_ENABLE.CMDQ_COMPLETED = 1;
            break;
        case DISP_LCD_HTT_INT:
            LCD_REG->INT_ENABLE.HTT = 1;
            break;
        case DISP_LCD_SYNC_INT:
            LCD_REG->INT_ENABLE.SYNC = 1;
            break;
        default:
            return LCD_STATUS_ERROR;
    }

    return LCD_STATUS_OK;
#else
    ///TODO: warning log here
    return LCD_STATUS_ERROR;
#endif
}


LCD_STATUS LCD_SetInterruptCallback(void (*pCB)(DISP_INTERRUPT_EVENTS))
{
    _lcdContext.pIntCallback = pCB;

    return LCD_STATUS_OK;
}


// -------------------- LCD Controller Interface --------------------

LCD_STATUS LCD_ConfigParallelIF(LCD_IF_ID id,
                                LCD_IF_PARALLEL_BITS ifDataWidth,
                                LCD_IF_PARALLEL_CLK_DIV clkDivisor,
                                UINT32 writeSetup,
                                UINT32 writeHold,
                                UINT32 writeWait,
                                UINT32 readSetup,
								UINT32 readHold,
                                UINT32 readLatency,
                                UINT32 waitPeriod,
								UINT32 chw)
{
    ASSERT(id >= LCD_IF_PARALLEL_0 && id <= LCD_IF_PARALLEL_2);
    ASSERT(writeSetup <= 16U);
    ASSERT(writeHold <= 16U);
    ASSERT(writeWait <= 64U);
    ASSERT(readSetup <= 16U);
	ASSERT(readHold <= 16U);
    ASSERT(readLatency <= 64U);
    ASSERT(chw <= 16U);

    if (0 == writeHold)   writeHold = 1;
    if (0 == writeWait)   writeWait = 1;
    if (0 == readLatency) readLatency = 1;

    _WaitForEngineNotBusy();

    // (1) Config Data Width
    {
        LCD_REG_PCNFDW pcnfdw = LCD_REG->PARALLEL_DW;

        switch(id)
        {
        case LCD_IF_PARALLEL_0: pcnfdw.PCNF0_DW = (UINT32)ifDataWidth; pcnfdw.PCNF0_CHW = chw;break;
        case LCD_IF_PARALLEL_1: pcnfdw.PCNF1_DW = (UINT32)ifDataWidth; pcnfdw.PCNF1_CHW = chw;break;
        case LCD_IF_PARALLEL_2: pcnfdw.PCNF2_DW = (UINT32)ifDataWidth; pcnfdw.PCNF2_CHW = chw;break;
        default : ASSERT(0);
        };

        LCD_OUTREG32(&LCD_REG->PARALLEL_DW, AS_UINT32(&pcnfdw));
    }

    // (2) Config Timing
    {
        UINT32 i;
        LCD_REG_PCNF config;
        
        i = (UINT32)id - LCD_IF_PARALLEL_0;
        config = LCD_REG->PARALLEL_CFG[i];

        config.C2WS = writeSetup;
        config.C2WH = writeHold - 1;
        config.WST  = writeWait - 1;
        config.C2RS = readSetup;
        config.C2RH = readHold;
        config.RLT  = readLatency - 1;

        LCD_OUTREG32(&LCD_REG->PARALLEL_CFG[i], AS_UINT32(&config));
    }

    // (3) Config Delay Between Commands
    {
        LCD_REG_WROI_CON ctrl = LCD_REG->WROI_CONTROL;
//        ctrl.PERIOD = waitPeriod;
        LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));
    }

    // TODO: modify this for 6573
	//#warning "iopad selection should be refined"
	#if 0
    LCD_OUTREG32(MMSYS1_CONFIG_BASE+0x0400, 0x3);
	#endif
    return LCD_STATUS_OK;
}



LCD_STATUS LCD_ConfigIfFormat(LCD_IF_FMT_COLOR_ORDER order,
                              LCD_IF_FMT_TRANS_SEQ transSeq,
                              LCD_IF_FMT_PADDING padding,
                              LCD_IF_FORMAT format,
                              LCD_IF_WIDTH busWidth)
{
    LCD_REG_WROI_CON ctrl = LCD_REG->WROI_CONTROL;


    ctrl.RGB_ORDER  = order;
    ctrl.BYTE_ORDER = transSeq;
    ctrl.PADDING    = padding;
    ctrl.DATA_FMT   = (UINT32)format;
    ctrl.IF_FMT   = (UINT32)busWidth;
    ctrl.IF_24 = 0;
    if(busWidth == LCD_IF_WIDTH_24_BITS)
	{
	    ctrl.IF_24 = 1;
	}
    LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));


    return LCD_STATUS_OK;
}

LCD_STATUS LCD_ConfigSerialIF(LCD_IF_ID id,
                              LCD_IF_SERIAL_BITS bits,
                              UINT32 three_wire,
                              UINT32 sdi,
                              BOOL   first_pol,
                              BOOL   sck_def,
                              UINT32 div2,
                              UINT32 hw_cs,
                              UINT32 css,
                              UINT32 csh,
                              UINT32 rd_1st,
                              UINT32 rd_2nd,
                              UINT32 wr_1st,
                              UINT32 wr_2nd)
{
    LCD_REG_SCNF config;
	LCD_REG_SIF_TIMING sif_timing;
	unsigned int offset = 0;
	unsigned int sif_id = 0;
    
	ASSERT(id >= LCD_IF_SERIAL_0 && id <= LCD_IF_SERIAL_1);

    _WaitForEngineNotBusy();

    memset(&config, 0, sizeof(config));
	
	if(id == LCD_IF_SERIAL_1){
		offset = 8;
		sif_id = 1;
	}
	
	LCD_MASKREG32(&config, 0x07 << offset, bits << offset);
	LCD_MASKREG32(&config, 0x08 << offset, three_wire << (offset + 3));
	LCD_MASKREG32(&config, 0x10 << offset, sdi << (offset + 4));
	LCD_MASKREG32(&config, 0x20 << offset, first_pol << (offset + 5));
	LCD_MASKREG32(&config, 0x40 << offset, sck_def << (offset + 6));
	LCD_MASKREG32(&config, 0x80 << offset, div2 << (offset + 7));

	config.HW_CS = hw_cs;
//	config.SIZE_0 = bits;
    LCD_OUTREG32(&LCD_REG->SERIAL_CFG, AS_UINT32(&config));

	sif_timing.WR_2ND = wr_2nd;
	sif_timing.WR_1ST = wr_1st;
	sif_timing.RD_2ND = rd_2nd;
	sif_timing.RD_1ST = rd_1st;
	sif_timing.CSH = csh;
	sif_timing.CSS = css;
	
	LCD_OUTREG32(&LCD_REG->SIF_TIMING[sif_id], AS_UINT32(&sif_timing));

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_SetResetSignal(BOOL high)
{
    LCD_REG->RESET = high ? 1 : 0;

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_ConfigDSIIfFormat(LCD_DSI_IF_FMT_COLOR_ORDER order,
                              LCD_DSI_IF_FMT_TRANS_SEQ transSeq,
                              LCD_DSI_IF_FMT_PADDING padding,
                              LCD_DSI_IF_FORMAT format,
                              UINT32 packet_size,
                              bool DC_DSI)
{
	LCD_REG_DSI_DC config;

	config.DC_DSI		= DC_DSI;	
	config.BYTE_SWAP 	= transSeq;
	config.RGB_SWAP		= order;
	config.PAD_MSB		= padding;
	config.CLR_FMT		= format;
	config.PACKET_SIZE	= packet_size;

	OUTREG32(&LCD_REG->DS_DSI_CON, AS_UINT32(&config));

	return LCD_STATUS_OK;
}


// -------------------- Command Queue --------------------

LCD_STATUS LCD_CmdQueueEnable(BOOL enabled)
{
    LCD_REG_WROI_CON ctrl;

//    _WaitForEngineNotBusy();

    ctrl = LCD_REG->WROI_CONTROL;
    ctrl.ENC = enabled ? 1 : 0;
    LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));

    return LCD_STATUS_OK;
}

/*
LCD_STATUS LCD_CmdQueueSelect(LCD_CMDQ_ID id)
{
    LCD_REG_WROI_CON ctrl;

    _WaitForEngineNotBusy();
    ctrl = LCD_REG->WROI_CONTROL;

    switch(id)
    {
    case LCD_CMDQ_0 :
    case LCD_CMDQ_1 :
        ctrl.COM_SEL = (UINT32)id;
        break;
    default : ASSERT(0);
    }

    LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));

    return LCD_STATUS_OK;
}
*/

LCD_STATUS LCD_CmdQueueSetWaitPeriod(UINT32 period)
{
    LCD_REG_WROI_CON ctrl;

    ASSERT(period < 1024);
    
//    _WaitForEngineNotBusy();

    ctrl = LCD_REG->WROI_CONTROL;
//    ctrl.PERIOD = period;
    LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_CmdQueueWrite(UINT32 *cmds, UINT32 cmdCount)
{
    LCD_REG_WROI_CON ctrl;
    UINT32 i;

    ASSERT(cmdCount >= 0 && cmdCount < ARY_SIZE(LCD_REG->CMDQ));
    
//    _WaitForEngineNotBusy();
    ctrl = LCD_REG->WROI_CONTROL;
    ctrl.COMMAND = cmdCount - 1;
    LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));

    for (i = 0; i < cmdCount; ++ i)
    {
        LCD_REG->CMDQ[i] = cmds[i];
    }

    return LCD_STATUS_OK;
}


// -------------------- Layer Configurations --------------------

LCD_STATUS LCD_LayerEnable(LCD_LAYER_ID id, BOOL enable)
{
    LCD_REG_WROI_CON ctrl;

//    _WaitForEngineNotBusy();
	DBI_LOG("LCD_LayerEnable: %d, %d\n", id, enable);
    ctrl = LCD_REG->WROI_CONTROL;
    
    switch(id)
    {
    case LCD_LAYER_0 : ctrl.EN0 = enable ? 1 : 0; break;
    case LCD_LAYER_1 : ctrl.EN1 = enable ? 1 : 0; break;
    case LCD_LAYER_2 : ctrl.EN2 = enable ? 1 : 0; break;
    case LCD_LAYER_3 : ctrl.EN3 = enable ? 1 : 0; break;
    case LCD_LAYER_4 : ctrl.EN4 = enable ? 1 : 0; break;
    case LCD_LAYER_5 : ctrl.EN5 = enable ? 1 : 0; break;
    case LCD_LAYER_ALL :
        LCD_MASKREG32(&ctrl, 0xFC000000, enable ? 0xFC000000 : 0);
        break;
    default : ASSERT(0);
    }

    LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));
    
    return LCD_STATUS_OK;
}

LCD_STATUS LCD_ConfigDither(int lrs, int lgs, int lbs, int dbr, int dbg, int dbb)
{
	LCD_REG_DITHER_CON ctrl;

//	_WaitForEngineNotBusy();

	ctrl = LCD_REG->DITHER_CON;

	ctrl.LFSR_R_SEED = lrs;
	ctrl.LFSR_G_SEED = lgs;
	ctrl.LFSR_B_SEED = lbs;
	ctrl.DB_R = dbr;
	ctrl.DB_G = dbg;
	ctrl.DB_B = dbb;

	OUTREG32(&LCD_REG->DITHER_CON, AS_UINT32(&ctrl));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerEnableDither(LCD_LAYER_ID id, UINT32 enable)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        return LCD_STATUS_OK;
    }
    else if (id < LCD_LAYER_NUM)
    {
		LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
        ctrl.DITHER_EN = enable;
        OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
    }
    
    return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetAddress(LCD_LAYER_ID id, UINT32 address)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);
    ASSERT((address & 0x3) == 0);   // layer address should be 8-byte-aligned

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_REG->LAYER[id].ADDRESS = address;
    }
    
    return LCD_STATUS_OK;
}


UINT32 LCD_LayerGetAddress(LCD_LAYER_ID id)
{
    ASSERT(id < LCD_LAYER_NUM);

    return LCD_REG->LAYER[id].ADDRESS;
}


LCD_STATUS LCD_LayerSetSize(LCD_LAYER_ID id, UINT32 width, UINT32 height)
{
    LCD_REG_SIZE size;
    size.WIDTH = (UINT16)width;
    size.HEIGHT = (UINT16)height;

    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_OUTREG32(&LCD_REG->LAYER[id].SIZE, AS_UINT32(&size));
    }
    
    return LCD_STATUS_OK;
}


LCD_STATUS LCD_LayerSetPitch(LCD_LAYER_ID id, UINT32 pitch)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_OUTREG16(&LCD_REG->LAYER[id].WINDOW_PITCH, pitch);
    }
    
    return LCD_STATUS_OK;
}


LCD_STATUS LCD_LayerSetOffset(LCD_LAYER_ID id, UINT32 x, UINT32 y)
{
    LCD_REG_COORD offset;
    offset.X = (UINT16)x;
    offset.Y = (UINT16)y;

    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();
    
    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_OUTREG32(&LCD_REG->LAYER[id].OFFSET, AS_UINT32(&offset));
    }
    
    return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetWindowOffset(LCD_LAYER_ID id, UINT32 x, UINT32 y)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();
    
    if (id < LCD_LAYER_NUM)
    {
        OUTREG32(&LCD_REG->LAYER[id].WINDOW_OFFSET, (y<<16 | x));
    }
    
    return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetFormat(LCD_LAYER_ID id, LCD_LAYER_FORMAT format)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();
    
    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
        ctrl.CLRDPT = (UINT32)format;
        LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
    }
    
    return LCD_STATUS_OK;
}


LCD_STATUS LCD_LayerEnableByteSwap(LCD_LAYER_ID id, BOOL enable)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        UINT32 i;
        for (i = 0; i < ARY_SIZE(LCD_REG->LAYER); ++ i)
        {
            LCD_LayerEnableByteSwap((LCD_LAYER_ID)i, enable);
        }
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
        ctrl.SWP = enable ? 1 : 0;
        LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
    }

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_LayerSetRotation(LCD_LAYER_ID id, LCD_LAYER_ROTATION rotation)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
	    UINT32 i;
        LCD_REG_LAYER_CON ctrl;
	    for(i = LCD_LAYER_0;i < LCD_LAYER_NUM;i++){
	        ctrl = LCD_REG->LAYER[i].CONTROL;
	        ctrl.ROTATE = (UINT32)rotation;
            LCD_OUTREG32(&LCD_REG->LAYER[i].CONTROL, AS_UINT32(&ctrl));
	    }
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
        ctrl.ROTATE = (UINT32)rotation;
        LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
    }

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_LayerSetAlphaBlending(LCD_LAYER_ID id, BOOL enable, UINT8 alpha)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
        ctrl.OPAEN = enable ? 1 : 0;
        ctrl.OPA   = alpha;
        LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
    }
    
    return LCD_STATUS_OK;
}


LCD_STATUS LCD_LayerSetSourceColorKey(LCD_LAYER_ID id, BOOL enable, UINT32 colorKey)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
        ctrl.SRC_KEY_EN = enable ? 1 : 0;
        LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
        LCD_REG->LAYER[id].COLORKEY= colorKey;
    }
    
    return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetDestColorKey(LCD_LAYER_ID id, BOOL enable, UINT32 colorKey)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
        ctrl.DST_KEY_EN = enable ? 1 : 0;
        LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
        LCD_REG->LAYER[id].COLORKEY = colorKey;
    }
    
    return LCD_STATUS_OK;
}


LCD_STATUS LCD_Layer3D_Prepare(LCD_LAYER_ID id)
{
	unsigned int src_idx, dst_idx;

	src_idx = id;

	if(src_idx&0x1)
		dst_idx = src_idx-1;
	else
		dst_idx = src_idx+1;

	memcpy((void *)&LCD_REG->LAYER[dst_idx], (void *)&LCD_REG->LAYER[src_idx], sizeof(LCD_REG_LAYER));

	return LCD_STATUS_OK;
}


LCD_STATUS LCD_Layer3D_Enable(LCD_LAYER_ID id, BOOL enable)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

	if (LCD_LAYER_ALL == id)
	{
		NOT_IMPLEMENTED();
	}
	else if (id < LCD_LAYER_NUM)
	{
		LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
		ctrl.THREE_D= enable ? 1 : 0;
		LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
	}

	return LCD_STATUS_OK;

}

LCD_STATUS LCD_Layer3D_R1st(LCD_LAYER_ID id, BOOL r_first)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

	if (LCD_LAYER_ALL == id)
	{
		NOT_IMPLEMENTED();
	}
	else if (id < LCD_LAYER_NUM)
	{
		LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
		ctrl.R_FIRST= r_first ? 1 : 0;
		LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
	}

	return LCD_STATUS_OK;

}

LCD_STATUS LCD_Layer3D_landscape(LCD_LAYER_ID id, BOOL landscape)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

	if (LCD_LAYER_ALL == id)
	{
		NOT_IMPLEMENTED();
	}
	else if (id < LCD_LAYER_NUM)
	{
		LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
		ctrl.LANDSCAPE= landscape ? 1 : 0;
		LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
	}

	return LCD_STATUS_OK;

}


LCD_STATUS LCD_LayerSet3D(LCD_LAYER_ID id, BOOL enable, BOOL r_first, BOOL landscape)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

//    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        LCD_REG_LAYER_CON ctrl = LCD_REG->LAYER[id].CONTROL;
        ctrl.THREE_D= enable ? 1 : 0;
        ctrl.R_FIRST= r_first ? 1 : 0;
        ctrl.LANDSCAPE= landscape ? 1 : 0;
        LCD_OUTREG32(&LCD_REG->LAYER[id].CONTROL, AS_UINT32(&ctrl));
    }
    
    return LCD_STATUS_OK;
}


BOOL LCD_Is3DEnabled(void)
{
	LCD_REG_WROI_CON  con;
	LCD_REG_LAYER_CON con0, con1;

	con = LCD_REG->WROI_CONTROL;
		
	if ((con.EN0 && con.EN1))
	{
		con0 = LCD_REG->LAYER[0].CONTROL;
		con1 = LCD_REG->LAYER[1].CONTROL;
		if ((con0.THREE_D && con1.THREE_D))
			return TRUE;
	}

	if ((con.EN2 && con.EN3))
	{
		con0 = LCD_REG->LAYER[2].CONTROL;
		con1 = LCD_REG->LAYER[3].CONTROL;
		if ((con0.THREE_D && con1.THREE_D))
			return TRUE;
	}

	if ((con.EN4 && con.EN5))
	{
		con0 = LCD_REG->LAYER[4].CONTROL;
		con1 = LCD_REG->LAYER[5].CONTROL;
		if ((con0.THREE_D && con1.THREE_D))
			return TRUE;
	}

	return FALSE;
}


BOOL LCD_Is3DLandscapeMode(void)
{
	LCD_REG_WROI_CON  con;
	LCD_REG_LAYER_CON con0, con1;

	con = LCD_REG->WROI_CONTROL;

	if ((con.EN0 && con.EN1))
	{
		con0 = LCD_REG->LAYER[0].CONTROL;
		con1 = LCD_REG->LAYER[1].CONTROL;
		if ((con0.LANDSCAPE && con1.LANDSCAPE))
			return TRUE;
	}

	if ((con.EN2 && con.EN3))
	{
		con0 = LCD_REG->LAYER[2].CONTROL;
		con1 = LCD_REG->LAYER[3].CONTROL;
		if ((con0.LANDSCAPE && con1.LANDSCAPE))
			return TRUE;
	}

	if ((con.EN4 && con.EN5))
	{
		con0 = LCD_REG->LAYER[4].CONTROL;
		con1 = LCD_REG->LAYER[5].CONTROL;
		if ((con0.LANDSCAPE && con1.LANDSCAPE))
			return TRUE;
	}

	return FALSE;
}


// -------------------- HW Trigger Configurations --------------------

LCD_STATUS LCD_LayerSetTriggerMode(LCD_LAYER_ID id, LCD_LAYER_TRIGGER_MODE mode)
{
    LCD_WROI_HWREF hwref;
    LCD_REG_WROI_DC dc;

    BOOL enHwTrigger = (LCD_SW_TRIGGER != mode);
    BOOL enDirectCouple = (LCD_HW_TRIGGER_DIRECT_COUPLE == mode);

//    _WaitForEngineNotBusy();

    hwref = LCD_REG->WROI_HW_REFRESH;
    dc = LCD_REG->WROI_DC;
        
    switch(id)
    {
    case LCD_LAYER_0 :
        hwref.EN0 = enHwTrigger ? 1 : 0;
        dc.EN0 = enDirectCouple ? 1 : 0;
        break;
    case LCD_LAYER_1 : 
        hwref.EN1 = enHwTrigger ? 1 : 0;
        dc.EN1 = enDirectCouple ? 1 : 0;
        break;
    case LCD_LAYER_2 : 
        hwref.EN2 = enHwTrigger ? 1 : 0;
        dc.EN2 = enDirectCouple ? 1 : 0;
        break;
    case LCD_LAYER_3 : 
        hwref.EN3 = enHwTrigger ? 1 : 0;
        dc.EN3 = enDirectCouple ? 1 : 0;
        break;
    case LCD_LAYER_4 : 
        hwref.EN4 = enHwTrigger ? 1 : 0;
        dc.EN4 = enDirectCouple ? 1 : 0;
        break;
    case LCD_LAYER_5 : 
        hwref.EN5 = enHwTrigger ? 1 : 0;
        dc.EN5 = enDirectCouple ? 1 : 0;
        break;
    case LCD_LAYER_ALL :
        LCD_MASKREG32(&hwref, 0xFC000000, enHwTrigger ? 0xFC000000 : 0);
        LCD_MASKREG32(&dc, 0xFC000000, enDirectCouple ? 0xFC000000 : 0);
        break;
    default : ASSERT(0);
    }

    LCD_OUTREG32(&LCD_REG->WROI_HW_REFRESH, AS_UINT32(&hwref));
    LCD_OUTREG32(&LCD_REG->WROI_DC, AS_UINT32(&dc));

    return LCD_STATUS_OK;
}

LCD_STATUS LCD_EnableHwTrigger(BOOL enable)
{
    LCD_WROI_HWREF hwref = LCD_REG->WROI_HW_REFRESH;
    hwref.HWEN = enable ? 1 : 0;
    LCD_OUTREG32(&LCD_REG->WROI_HW_REFRESH, AS_UINT32(&hwref));

    // Disable all layers direct-couple
    //
    if (!enable && (INREG32(&LCD_REG->WROI_DC) & 0xFC000000))
    {
        LCD_MASKREG32(&LCD_REG->WROI_DC, 0xFC000000, 0);
    }

    return LCD_STATUS_OK;
}


// -------------------- ROI Window Configurations --------------------

LCD_STATUS LCD_SetBackgroundColor(UINT32 bgColor)
{
//    _WaitForEngineNotBusy();
    LCD_REG->WROI_BG_COLOR = bgColor;
    
    return LCD_STATUS_OK;
}

LCD_STATUS LCD_GetRoiWindow(UINT32 *x, UINT32 *y, UINT32 *width, UINT32 *height)
{
	*x = LCD_REG->WROI_OFFSET.X;
	*y = LCD_REG->WROI_OFFSET.Y;
	*width = LCD_REG->WROI_SIZE.WIDTH;
	*height = LCD_REG->WROI_SIZE.HEIGHT;

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_SetRoiWindow(UINT32 x, UINT32 y, UINT32 width, UINT32 height)
{
    LCD_REG_COORD offset;
    LCD_REG_SIZE size;
    
    offset.X = (UINT16)x;
    offset.Y = (UINT16)y;
    size.WIDTH = (UINT16)width;
    size.HEIGHT = (UINT16)height;

//    _WaitForEngineNotBusy();
    LCD_OUTREG32(&LCD_REG->WROI_OFFSET, AS_UINT32(&offset));
    LCD_OUTREG32(&LCD_REG->WROI_SIZE, AS_UINT32(&size));

    _lcdContext.roiWndSize = size;
        
    return LCD_STATUS_OK;
}

// DSI Related Configurations
LCD_STATUS LCD_EnableDCtoDsi(BOOL enable)
{
    LCD_REG_DSI_DC tmp_reg;
    
	tmp_reg=LCD_REG->DS_DSI_CON;
    tmp_reg.DC_DSI = enable;
    
    LCD_OUTREG32(&LCD_REG->DS_DSI_CON, AS_UINT32(&tmp_reg));

    return LCD_STATUS_OK;
}

// -------------------- Output to Memory Configurations --------------------

LCD_STATUS LCD_SetOutputMode(LCD_OUTPUT_MODE mode)
{
    LCD_REG_WROI_CON    roiCtrl;
    LCD_REG_WROI_W2MCON w2mCtrl;
//    LCD_REGS *backup_regs;

    _WaitForEngineNotBusy();

    roiCtrl = LCD_REG->WROI_CONTROL;
    w2mCtrl = LCD_REG->WROI_W2M_CONTROL;

    w2mCtrl.W2LCM     = (mode & LCD_OUTPUT_TO_LCM) ? 1 : 0;
    roiCtrl.W2M       = (mode & LCD_OUTPUT_TO_MEM) ? 1 : 0;
    w2mCtrl.DC_TV_ROT = (mode & LCD_OUTPUT_TO_TVROT) ? 1 : 0;

    LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&roiCtrl));
    LCD_OUTREG32(&LCD_REG->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));

    _lcdContext.outputMode = mode;

#if 0
    //write to backup regs to avoid inconsistencies
    backup_regs = &(_lcdContext.regBackup);
    roiCtrl = backup_regs->WROI_CONTROL;
    w2mCtrl = backup_regs->WROI_W2M_CONTROL;

    w2mCtrl.W2LCM     = (mode & LCD_OUTPUT_TO_LCM) ? 1 : 0;
    roiCtrl.W2M       = (mode & LCD_OUTPUT_TO_MEM) ? 1 : 0;
    w2mCtrl.DC_TV_ROT = (mode & LCD_OUTPUT_TO_TVROT) ? 1 : 0;


    LCD_OUTREG32(&backup_regs->WROI_CONTROL, AS_UINT32(&roiCtrl));
    LCD_OUTREG32(&backup_regs->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));
#endif
    return LCD_STATUS_OK;    
}
EXPORT_SYMBOL(LCD_SetOutputMode);

LCD_STATUS LCD_SetOutputAlpha(unsigned int alpha)
{
    LCD_REG_WROI_W2MCON w2mCtrl;

//    _WaitForEngineNotBusy();

    w2mCtrl = LCD_REG->WROI_W2M_CONTROL;

    w2mCtrl.OUT_ALPHA = alpha;

    OUTREG32(&LCD_REG->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));

    return LCD_STATUS_OK;    
}

LCD_STATUS LCD_WaitDPIIndication(BOOL enable)
{
    LCD_REG_WROI_W2MCON w2mCtrl;

//    _WaitForEngineNotBusy();

    w2mCtrl = LCD_REG->WROI_W2M_CONTROL;
    w2mCtrl.DLY_EN = enable ? 1 : 0;
    LCD_OUTREG32(&LCD_REG->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));
    
    return LCD_STATUS_OK;
}
EXPORT_SYMBOL(LCD_WaitDPIIndication);


LCD_STATUS LCD_FBSetFormat(LCD_FB_FORMAT format)
{
    LCD_REG_WROI_W2MCON w2mCtrl;
//    LCD_REGS *backup_regs;

//    _WaitForEngineNotBusy();

    w2mCtrl = LCD_REG->WROI_W2M_CONTROL;
    w2mCtrl.W2M_FMT = (UINT32)format;
    LCD_OUTREG32(&LCD_REG->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));

    _lcdContext.fbFormat = format;

#if 0
    //write to backup regs to avoid inconsistencies
    backup_regs = &(_lcdContext.regBackup);
    w2mCtrl = backup_regs->WROI_W2M_CONTROL;
    w2mCtrl.W2M_FMT = (UINT32)format;

    LCD_OUTREG32(&backup_regs->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));
#endif
    return LCD_STATUS_OK;
}
EXPORT_SYMBOL(LCD_FBSetFormat);

LCD_STATUS LCD_FBSetPitch(UINT32 pitchInByte)
{
//    LCD_REGS *backup_regs;
    _lcdContext.fbPitchInBytes = pitchInByte;

    LCD_REG->W2M_PITCH = pitchInByte;

#if 0
    //write to backup regs to avoid inconsistencies
    backup_regs = &(_lcdContext.regBackup);
    backup_regs->W2M_PITCH = pitchInByte;
#endif
    return LCD_STATUS_OK;
}
EXPORT_SYMBOL(LCD_FBSetPitch);

LCD_STATUS LCD_WriteBackUp(LCD_FB_ID id, hdmiBackupLCDFB backupFB)
{
    LCD_REGS *backup_regs = &(_lcdContext.regBackup);
    
	_lcdContext.fbPitchInBytes = backupFB.pitchInBytes;
    _lcdContext.fbFormat = backupFB.format;
    _lcdContext.outputMode = backupFB.mode;
    //write to backup regs to avoid inconsistencies
	switch(id)
	{	
		case LCD_FB_0 : // do nothing
        break;
    	
		case LCD_FB_1 :
        	backup_regs->WROI_W2M_CONTROL.FB1_EN = backupFB.enable ? 1 : 0;
		break;

    	case LCD_FB_2 :
	        backup_regs->WROI_W2M_CONTROL.FB2_EN = backupFB.enable ? 1 : 0;
        break;

    	default:
        ASSERT(0);
	}
	backup_regs->WROI_W2M_OFFSET.x = (UINT16)backupFB.x;
	backup_regs->WROI_W2M_OFFSET.y = (UINT16)backupFB.y;
	backup_regs->WROI_W2M_ADDR[id] = backupFB.address;	
    backup_regs->W2M_PITCH = backupFB.pitchInBytes; //pitch
    backup_regs->WROI_W2M_CONTROL.W2M_FMT = (UINT32)backupFB.format;
    backup_regs->WROI_W2M_CONTROL.W2LCM = (backupFB.mode & LCD_OUTPUT_TO_LCM) ? 1 : 0;
    backup_regs->WROI_CONTROL.W2M = (backupFB.mode & LCD_OUTPUT_TO_MEM) ? 1 : 0;
    backup_regs->WROI_W2M_CONTROL.DC_TV_ROT = (backupFB.mode & LCD_OUTPUT_TO_TVROT) ? 1 : 0;
 
 	return LCD_STATUS_OK;
}
EXPORT_SYMBOL(LCD_WriteBackUp);

LCD_STATUS LCD_FBEnable(LCD_FB_ID id, BOOL enable)
{
    LCD_REG_WROI_W2MCON w2mCtrl;
//    LCD_REGS *backup_regs;

//    _WaitForEngineNotBusy();

    w2mCtrl = LCD_REG->WROI_W2M_CONTROL;

    switch(id)
    {
    case LCD_FB_0 : // do nothing
        break;
        
    case LCD_FB_1 :
        w2mCtrl.FB1_EN = enable ? 1 : 0;
        break;

    case LCD_FB_2 :
        w2mCtrl.FB2_EN = enable ? 1 : 0;
        break;

    default:
        ASSERT(0);
    }

    LCD_OUTREG32(&LCD_REG->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));

#if 0
    //write to backup regs to avoid inconsistencies
    backup_regs = &(_lcdContext.regBackup);
    w2mCtrl = backup_regs->WROI_W2M_CONTROL;

    switch(id)
    {
    case LCD_FB_0 : // do nothing
        break;
        
    case LCD_FB_1 :
        w2mCtrl.FB1_EN = enable ? 1 : 0;
        break;

    case LCD_FB_2 :
        w2mCtrl.FB2_EN = enable ? 1 : 0;
        break;

    default:
        ASSERT(0);
    }


    LCD_OUTREG32(&backup_regs->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));
#endif
    return LCD_STATUS_OK;
}
EXPORT_SYMBOL(LCD_FBEnable);

LCD_STATUS LCD_FBReset(void)
{
    LCD_REG_WROI_W2MCON w2mCtrl;

//    _WaitForEngineNotBusy();

    w2mCtrl = LCD_REG->WROI_W2M_CONTROL;
    w2mCtrl.FBSEQ_RST = 1;
    OUTREG32(&LCD_REG->WROI_W2M_CONTROL, AS_UINT32(&w2mCtrl));

    return LCD_STATUS_OK;
}

LCD_STATUS LCD_FBSetAddress(LCD_FB_ID id, UINT32 address)
{
//    LCD_REGS *backup_regs;
    ASSERT(id < LCD_FB_NUM);

//    _WaitForEngineNotBusy();
    LCD_REG->WROI_W2M_ADDR[id] = address;

#if 0
    //write to backup regs to avoid inconsistencies
    backup_regs = &(_lcdContext.regBackup);
    backup_regs->WROI_W2M_ADDR[id] = address;
#endif
    return LCD_STATUS_OK;
}
EXPORT_SYMBOL(LCD_FBSetAddress);

LCD_STATUS LCD_FBSetStartCoord(UINT32 x, UINT32 y)
{
//    LCD_REGS *backup_regs;
    LCD_REG_COORD offset;
    offset.X = (UINT16)x;
    offset.Y = (UINT16)y;

//    _WaitForEngineNotBusy();

    //WDT_SW_MM_PERI_RESET(MM_PERI_LCD);
    
    LCD_OUTREG32(&LCD_REG->WROI_W2M_OFFSET, AS_UINT32(&offset));

#if 0
    //write to backup regs to avoid inconsistencies
    backup_regs = &(_lcdContext.regBackup);
    LCD_OUTREG32(&backup_regs->WROI_W2M_OFFSET, AS_UINT32(&offset));
#endif
    return LCD_STATUS_OK;
}
EXPORT_SYMBOL(LCD_FBSetStartCoord);
// -------------------- Color Matrix --------------------

LCD_STATUS LCD_EnableColorMatrix(LCD_IF_ID id, BOOL enable)
{
//#warning "the color matrix is different on MT6573"
    switch(id)
    {
    case LCD_IF_PARALLEL_0 :
    case LCD_IF_PARALLEL_1 :
    case LCD_IF_PARALLEL_2 :
    {
        UINT32 i = id - LCD_IF_PARALLEL_0;
        LCD_REG_PCNF config = LCD_REG->PARALLEL_CFG[i];
        LCD_OUTREG32(&LCD_REG->PARALLEL_CFG[i], AS_UINT32(&config));
        break;
    }
    case LCD_IF_SERIAL_0 :
    case LCD_IF_SERIAL_1 :
    {
        LCD_REG_SCNF config = LCD_REG->SERIAL_CFG;

        LCD_OUTREG32(&LCD_REG->SERIAL_CFG, AS_UINT32(&config));
        break;
    }
    case LCD_IF_ALL :
    {
        LCD_EnableColorMatrix(LCD_IF_PARALLEL_0, enable);
        LCD_EnableColorMatrix(LCD_IF_PARALLEL_1, enable);
        LCD_EnableColorMatrix(LCD_IF_PARALLEL_2, enable);
        LCD_EnableColorMatrix(LCD_IF_SERIAL_0, enable);
        break;
    }
    default: ASSERT(0);
    }

    return LCD_STATUS_OK;
}

/** Input: const S2_8 mat[9], fixed ponit signed 2.8 format
           |                      |
           | mat[0] mat[1] mat[2] |
           | mat[3] mat[4] mat[5] |
           | mat[6] mat[7] mat[8] |
           |                      |
*/
LCD_STATUS LCD_SetColorMatrix(const S2_8 mat[9])
{
//#warning "the color matrix is different on MT6573"
    UINT32 i, j = 0;

    for (i = 0; i < ARY_SIZE(LCD_REG->COEF_ROW); ++ i)
    {
        LCD_REG_COEF_ROW row = LCD_REG->COEF_ROW[i];
        row.COL0 = mat[j++];
        row.COL1 = mat[j++];
        row.COL2 = mat[j++];
        LCD_OUTREG32(&LCD_REG->COEF_ROW[i], AS_UINT32(&row));
    }

    return LCD_STATUS_OK;
}

// -------------------- Tearing Control --------------------

LCD_STATUS LCD_TE_Enable(BOOL enable)
{
    LCD_REG_TECON tecon = LCD_REG->TEARING_CFG;
    tecon.ENABLE = enable ? 1 : 0;
    LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&tecon));

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_TE_SetMode(LCD_TE_MODE mode)
{
    LCD_REG_TECON tecon = LCD_REG->TEARING_CFG;
    tecon.MODE = (LCD_TE_MODE_VSYNC_OR_HSYNC == mode) ? 1 : 0;
    LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&tecon));

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_TE_SetEdgePolarity(BOOL polarity)
{
    LCD_REG_TECON tecon = LCD_REG->TEARING_CFG;
    tecon.EDGE_SEL = (polarity ? 1 : 0);
    LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&tecon));

    return LCD_STATUS_OK;
}


LCD_STATUS LCD_TE_ConfigVHSyncMode(UINT32 hsDelayCnt,
                                   UINT32 vsWidthCnt,
                                   LCD_TE_VS_WIDTH_CNT_DIV vsWidthCntDiv)
{
/*    LCD_REG_TECON tecon = LCD_REG->TEARING_CFG;
    tecon.HS_MCH_CNT = (hsDelayCnt ? hsDelayCnt - 1 : 0);
    tecon.VS_WLMT = (vsWidthCnt ? vsWidthCnt - 1 : 0);
    tecon.VS_CNT_DIV = vsWidthCntDiv;
    LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&tecon));
*/
    return LCD_STATUS_OK;
}

// -------------------- Operations --------------------

LCD_STATUS LCD_SelectWriteIF(LCD_IF_ID id)
{
    LCD_REG_CMD_ADDR cmd_addr;
	LCD_REG_DAT_ADDR dat_addr;

    switch(id)
    {
    case LCD_IF_PARALLEL_0 : cmd_addr.addr = 0; break;
    case LCD_IF_PARALLEL_1 : cmd_addr.addr = 2; break;
    case LCD_IF_PARALLEL_2 : cmd_addr.addr = 4; break;
    case LCD_IF_SERIAL_0   : cmd_addr.addr = 8; break;
    case LCD_IF_SERIAL_1   : cmd_addr.addr = 0xA; break;
    default:
        ASSERT(0);
    }
	dat_addr.addr = cmd_addr.addr + 1;
    LCD_OUTREG16(&LCD_REG->WROI_CMD_ADDR, AS_UINT16(&cmd_addr));
    LCD_OUTREG16(&LCD_REG->WROI_DATA_ADDR, AS_UINT16(&dat_addr));

    return LCD_STATUS_OK;
}


__inline static void _LCD_WriteIF(DWORD baseAddr, UINT32 value, LCD_IF_MCU_WRITE_BITS bits)
{
    switch(bits)
    {
    case LCD_IF_MCU_WRITE_8BIT :
        LCD_OUTREG8(baseAddr, value);
        break;
        
    case LCD_IF_MCU_WRITE_16BIT :
        LCD_OUTREG16(baseAddr, value);
        break;
        
    case LCD_IF_MCU_WRITE_32BIT :
        LCD_OUTREG32(baseAddr, value);
        break;

    default:
        ASSERT(0);
    }
}


LCD_STATUS LCD_WriteIF(LCD_IF_ID id, LCD_IF_A0_MODE a0,
                       UINT32 value, LCD_IF_MCU_WRITE_BITS bits)
{
    DWORD baseAddr = 0;

    switch(id)
    {
    case LCD_IF_PARALLEL_0 : baseAddr = (DWORD)&LCD_REG->PCMD0; break;
    case LCD_IF_PARALLEL_1 : baseAddr = (DWORD)&LCD_REG->PCMD1; break;
    case LCD_IF_PARALLEL_2 : baseAddr = (DWORD)&LCD_REG->PCMD2; break;
    case LCD_IF_SERIAL_0   : baseAddr = (DWORD)&LCD_REG->SCMD0; break;
    case LCD_IF_SERIAL_1   : baseAddr = (DWORD)&LCD_REG->SCMD1; break;
    default:
        ASSERT(0);
    }

    if (LCD_IF_A0_HIGH == a0)
    {
        baseAddr += LCD_A0_HIGH_OFFSET;
    }

    _LCD_WriteIF(baseAddr, value, bits);

    return LCD_STATUS_OK;
}


__inline static UINT32 _LCD_ReadIF(DWORD baseAddr, LCD_IF_MCU_WRITE_BITS bits)
{
    switch(bits)
    {
    case LCD_IF_MCU_WRITE_8BIT :
        return (UINT32)INREG8(baseAddr);
        
    case LCD_IF_MCU_WRITE_16BIT :
        return (UINT32)INREG16(baseAddr);
        
    case LCD_IF_MCU_WRITE_32BIT :
        return (UINT32)INREG32(baseAddr);

    default:
        ASSERT(0);
    }
}


LCD_STATUS LCD_ReadIF(LCD_IF_ID id, LCD_IF_A0_MODE a0,
                      UINT32 *value, LCD_IF_MCU_WRITE_BITS bits)
{
    DWORD baseAddr = 0;

    if (NULL == value) return LCD_STATUS_ERROR;

    switch(id)
    {
    case LCD_IF_PARALLEL_0 : baseAddr = (DWORD)&LCD_REG->PCMD0; break;
    case LCD_IF_PARALLEL_1 : baseAddr = (DWORD)&LCD_REG->PCMD1; break;
    case LCD_IF_PARALLEL_2 : baseAddr = (DWORD)&LCD_REG->PCMD2; break;
    case LCD_IF_SERIAL_0   : baseAddr = (DWORD)&LCD_REG->SCMD0; break;
    case LCD_IF_SERIAL_1   : baseAddr = (DWORD)&LCD_REG->SCMD1; break;
    default:
        ASSERT(0);
    }

    if (LCD_IF_A0_HIGH == a0)
    {
        baseAddr += LCD_A0_HIGH_OFFSET;
    }

    *value = _LCD_ReadIF(baseAddr, bits);

    return LCD_STATUS_OK;
}

bool LCD_IsLayerEnable(LCD_LAYER_ID id)
{
    LCD_REG_WROI_CON ctrl;

//    _WaitForEngineNotBusy();
	bool ret = false;
    ctrl = LCD_REG->WROI_CONTROL;
    
    switch(id)
    {
    case LCD_LAYER_0 :if(ctrl.EN0 == 1)ret = true;break;
    case LCD_LAYER_1 :if(ctrl.EN1 == 1)ret = true;break;   
	case LCD_LAYER_2 :if(ctrl.EN2 == 1)ret = true;break;   
	case LCD_LAYER_3 :if(ctrl.EN3 == 1)ret = true;break;  
	case LCD_LAYER_4 :if(ctrl.EN4 == 1)ret = true;break;   
	case LCD_LAYER_5 :if(ctrl.EN5 == 1)ret = true;break;
    default : ASSERT(0);
    }
    return ret;
}

LCD_STATUS LCD_StartTransfer(BOOL blocking)
{
    LCD_REG_SIZE mainWndSize = _lcdContext.roiWndSize;

    if (_lcdContext.outputMode & LCD_OUTPUT_TO_MEM)
    {
        UINT32 bpp = TO_BPP[_lcdContext.fbFormat];

        // Set pitch in pixel here according to frame buffer BPP
        ASSERT(_lcdContext.fbPitchInBytes);
        ASSERT(_lcdContext.fbPitchInBytes % bpp == 0);
        mainWndSize.WIDTH = (UINT16)(_lcdContext.fbPitchInBytes / bpp);
    }

    //LCD_OUTREG32(&LCD_REG->MWIN_SIZE, AS_UINT32(&mainWndSize));
	LCD_SetGMCThrottle();
    _WaitForEngineNotBusy();
    DBG_OnTriggerLcd();
    DBI_FUNC();

	LCD_OUTREG32(&LCD_REG->START, 0);
    LCD_OUTREG32(&LCD_REG->START, (1 << 15));

    if (blocking)
    {
        _WaitForEngineNotBusy();
    }

    return LCD_STATUS_OK;
}

// -------------------- Retrieve Information --------------------

LCD_OUTPUT_MODE LCD_GetOutputMode(void)
{
	return _lcdContext.outputMode;
}
LCD_STATE  LCD_GetState(void)
{
    if (!s_isLcdPowerOn)
    {
        return LCD_STATE_POWER_OFF;
    }

    if (_IsEngineBusy())
    {
        return LCD_STATE_BUSY;
    }

    return LCD_STATE_IDLE;
}


LCD_STATUS LCD_DumpRegisters(void)
{
    UINT32 i;

    DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", "---------- Start dump LCD registers ----------\n");
    
    for (i = 0; i < offsetof(LCD_REGS, COEF_ROW); i += 4)
    {
        DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", "LCD+%04x : 0x%08x\n", i, INREG32(LCD_BASE + i));
    }
	
    for (i = 0x400; i < 0x470; i += 4)
    {
        DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", "LCD+%04x : 0x%08x\n", i, INREG32(LCD_BASE + i));
    }

    for (i = offsetof(LCD_REGS, CMDQ); i < sizeof(LCD_REGS); i += 4)
    {
        DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", "LCD+%04x : 0x%08x\n", i, INREG32(LCD_BASE + i));
    }

    return LCD_STATUS_OK;
}

#if !defined(MTK_M4U_SUPPORT)
#ifdef BUILD_UBOOT
static unsigned long v2p(unsigned long va)
{
    return va;
}
#else
static unsigned long v2p(unsigned long va)
{
    unsigned long pageOffset = (va & (PAGE_SIZE - 1));
    pgd_t *pgd;
    pmd_t *pmd;
    pte_t *pte;
    unsigned long pa;

    pgd = pgd_offset(current->mm, va); /* what is tsk->mm */
    pmd = pmd_offset(pgd, va);
    pte = pte_offset_map(pmd, va);
    
    pa = (pte_val(*pte) & (PAGE_MASK)) | pageOffset;


    return pa;
}
#endif
#endif
LCD_STATUS LCD_Get_VideoLayerSize(unsigned int id, unsigned int *width, unsigned int *height)
{
    ASSERT(id < LCD_LAYER_NUM || LCD_LAYER_ALL == id);

    _WaitForEngineNotBusy();

    if (LCD_LAYER_ALL == id)
    {
        NOT_IMPLEMENTED();
    }
    else if (id < LCD_LAYER_NUM)
    {
        *width = LCD_REG->LAYER[id].SIZE.WIDTH;
        *height = LCD_REG->LAYER[id].SIZE.HEIGHT;
    }

    return LCD_STATUS_OK;
}

LCD_STATUS LCD_Capture_Layerbuffer(unsigned int layer_id, unsigned int pvbuf, unsigned int bpp)
{
    unsigned int ppbuf = 0;
    UINT16 offset_x, offset_y, w, h;
    LCD_OUTPUT_MODE mode;
    if(pvbuf == 0 || bpp == 0)
    {
        DBI_LOG("LCD_Capture_Layerbuffer, ERROR, parameters wrong: pvbuf=0x%08x, bpp=%d\n", pvbuf, bpp);
        return LCD_STATUS_ERROR;
    }

    if(layer_id >= LCD_LAYER_NUM)
    {
        DBI_LOG("LCD_Capture_Layerbuffer, ERROR, parameters wrong: layer_id=%d\n", layer_id);
        return LCD_STATUS_ERROR;
    }

    if(!LCD_IsLayerEnable(layer_id))
    {
        DBI_LOG("LCD_Capture_Layerbuffer, ERROR, the layer%d is not enabled\n", layer_id);
        return LCD_STATUS_ERROR;
    }

    LCD_WaitForNotBusy();

    _BackupLCDRegisters();

    // begin capture
    if(bpp == 32)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_ARGB8888));
    else if(bpp == 16)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_RGB565));
    else if(bpp == 24)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_RGB888));
    else
        DBI_LOG("mtkfb_capture_Videobuffer, fb color format not support\n");

	offset_x = LCD_REG->LAYER[layer_id].OFFSET.X;
	offset_y = LCD_REG->LAYER[layer_id].OFFSET.Y;
	w = LCD_REG->LAYER[layer_id].SIZE.WIDTH;
	h = LCD_REG->LAYER[layer_id].SIZE.HEIGHT;
#if defined(MTK_M4U_SUPPORT)
	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("[LCDC init M4U] M4U not been init for LCDC\n");
		return LCD_STATUS_ERROR;
	}
	
	_m4u_lcdc_func.m4u_alloc_mva(M4U_CLNTMOD_LCDC, pvbuf, w*h*bpp/8, &ppbuf);
#else
    ppbuf = v2p(pvbuf);
#endif
	ASSERT(ppbuf != 0);

	mode = LCD_GetOutputMode();
	
	LCD_CHECK_RET(LCD_LayerEnable(LCD_LAYER_ALL, 0));
	LCD_CHECK_RET(LCD_LayerEnable(layer_id, 1));
	LCD_CHECK_RET(LCD_LayerSetRotation(layer_id, LCD_LAYER_ROTATE_0));
	LCD_CHECK_RET(LCD_SetRoiWindow(offset_x, offset_y, w, h));
	LCD_CHECK_RET(LCD_FBSetPitch((LCD_REG->LAYER[layer_id].SIZE.WIDTH)*bpp/8));

	LCD_CHECK_RET(LCD_FBSetStartCoord(0, 0));
	LCD_CHECK_RET(LCD_FBReset());
    
	LCD_CHECK_RET(LCD_FBSetAddress(LCD_FB_0, ppbuf));
	LCD_CHECK_RET(LCD_FBEnable(LCD_FB_0, TRUE));

    LCD_CHECK_RET(LCD_SetOutputMode(LCD_OUTPUT_TO_MEM));
    LCD_TE_Enable(FALSE);

	LCD_SetOutputAlpha(0xff);

//	LCD_CHECK_RET(LCD_SetRoiWindow(offset_x, offset_y, h, w));

	LCD_MASKREG32(0xf20a10a0, 0x1, 0); //disable DC to DSI when write to memory partially
#if defined(MTK_M4U_SUPPORT)
	_m4u_lcdc_func.m4u_dma_cache_maint(M4U_CLNTMOD_LCDC, (void *)pvbuf, w*h*bpp/8, DMA_BIDIRECTIONAL);
#endif

    LCD_CHECK_RET(LCD_StartTransfer(TRUE));
 
	LCD_SetOutputMode(mode);
    // capture end
    _RestoreLCDRegisters();
#if defined(MTK_M4U_SUPPORT)
    _m4u_lcdc_func.m4u_dealloc_mva(M4U_CLNTMOD_LCDC, pvbuf, w*h*bpp/8, ppbuf);
#endif
    return LCD_STATUS_OK;   
}


LCD_STATUS LCD_Capture_Videobuffer(unsigned int pvbuf, unsigned int bpp, unsigned int video_rotation)
{
    unsigned int ppbuf = 0;
    UINT16 offset_x, offset_y, w, h;
    LCD_OUTPUT_MODE mode;
    if(pvbuf == 0 || bpp == 0)
    {
        DBI_LOG("LCD_Capture_Videobuffer, ERROR, parameters wrong: pvbuf=0x%08x, bpp=%d\n", pvbuf, bpp);
        return LCD_STATUS_OK;
    }
    
    LCD_WaitForNotBusy();

    _BackupLCDRegisters();

    // begin capture
    if(bpp == 32)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_ARGB8888));
    else if(bpp == 16)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_RGB565));
    else if(bpp == 24)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_RGB888));
    else
        DBI_LOG("mtkfb_capture_Videobuffer, fb color format not support\n");

	offset_x = LCD_REG->LAYER[LCD_LAYER_2].OFFSET.X;
	offset_y = LCD_REG->LAYER[LCD_LAYER_2].OFFSET.Y;
	w = LCD_REG->LAYER[LCD_LAYER_2].SIZE.WIDTH;
	h = LCD_REG->LAYER[LCD_LAYER_2].SIZE.HEIGHT;
#if defined(MTK_M4U_SUPPORT)
	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("[LCDC init M4U] M4U not been init for LCDC\n");
		return LCD_STATUS_ERROR;
	}
	
	_m4u_lcdc_func.m4u_alloc_mva(M4U_CLNTMOD_LCDC, pvbuf, w*h*bpp/8, &ppbuf);
#else
    ppbuf = v2p(pvbuf);
#endif
	mode = LCD_GetOutputMode();
	
	LCD_CHECK_RET(LCD_LayerEnable(LCD_LAYER_ALL, 0));
	LCD_CHECK_RET(LCD_LayerEnable(LCD_LAYER_2, 1));
	LCD_CHECK_RET(LCD_LayerSetRotation(LCD_LAYER_2, LCD_LAYER_ROTATE_0));
	LCD_CHECK_RET(LCD_SetRoiWindow(offset_x, offset_y, w, h));
	LCD_CHECK_RET(LCD_FBSetPitch((LCD_REG->LAYER[LCD_LAYER_2].SIZE.WIDTH)*bpp/8));

	switch(video_rotation)
	{
		case 0:break;
		case 1:
			LCD_CHECK_RET(LCD_LayerSetRotation(LCD_LAYER_2, LCD_LAYER_ROTATE_90));
			LCD_CHECK_RET(LCD_SetRoiWindow(offset_x, offset_y, h, w));
			LCD_CHECK_RET(LCD_FBSetPitch((LCD_REG->LAYER[LCD_LAYER_2].SIZE.HEIGHT)*bpp/8));
			break;
		case 2:
			LCD_CHECK_RET(LCD_LayerSetRotation(LCD_LAYER_2, LCD_LAYER_ROTATE_180));break;
		case 3:
			LCD_CHECK_RET(LCD_LayerSetRotation(LCD_LAYER_2, LCD_LAYER_ROTATE_270));
			LCD_CHECK_RET(LCD_SetRoiWindow(offset_x, offset_y, h, w));
			LCD_CHECK_RET(LCD_FBSetPitch((LCD_REG->LAYER[LCD_LAYER_2].SIZE.HEIGHT)*bpp/8));
			break;
		default:
		    DBI_LOG("[LCD_Capture_videobuffer] video_rotation = %d error\n", video_rotation);
	}

	LCD_CHECK_RET(LCD_FBSetStartCoord(0, 0));
	LCD_CHECK_RET(LCD_FBReset());
    
	LCD_CHECK_RET(LCD_FBSetAddress(LCD_FB_0, ppbuf));
	LCD_CHECK_RET(LCD_FBEnable(LCD_FB_0, TRUE));

    LCD_CHECK_RET(LCD_SetOutputMode(LCD_OUTPUT_TO_MEM));
    LCD_TE_Enable(FALSE);

	LCD_SetOutputAlpha(0xff);

//	LCD_CHECK_RET(LCD_SetRoiWindow(offset_x, offset_y, h, w));

	LCD_MASKREG32(0xf20a10a0, 0x1, 0); //disable DC to DSI when write to memory partially
#if defined(MTK_M4U_SUPPORT)
	_m4u_lcdc_func.m4u_dma_cache_maint(M4U_CLNTMOD_LCDC, (void *)pvbuf, w*h*bpp/8, DMA_BIDIRECTIONAL);
#endif

	if(dbi_log_on)
		LCD_DumpRegisters();

    LCD_CHECK_RET(LCD_StartTransfer(TRUE));
 
	LCD_SetOutputMode(mode);
    // capture end
    _RestoreLCDRegisters();
#if defined(MTK_M4U_SUPPORT)
    _m4u_lcdc_func.m4u_dealloc_mva(M4U_CLNTMOD_LCDC, pvbuf, w*h*bpp/8, ppbuf);
#endif
    return LCD_STATUS_OK;   
}

#define ALIGN_TO(x, n)  \
	(((x) + ((n) - 1)) & ~((n) - 1))

LCD_STATUS LCD_Capture_Framebuffer(unsigned int pvbuf, unsigned int bpp)
{
    unsigned int ppbuf = 0;
    LCD_OUTPUT_MODE mode;
	UINT16 offset_x, offset_y, w, h, i;
    if(pvbuf == 0 || bpp == 0)
    {
        DBI_LOG("LCD_Capture_Framebuffer, ERROR, parameters wrong: pvbuf=0x%08x, bpp=%d\n", pvbuf, bpp);
        return LCD_STATUS_OK;
    }

#if defined(MTK_M4U_SUPPORT)
	if (!_m4u_lcdc_func.isInit)
	{
        unsigned int t;
		unsigned int i,j;
		unsigned int w_xres = DISP_GetScreenWidth();
	    unsigned int w = ALIGN_TO(DISP_GetScreenWidth(), 32);
	    unsigned int h = DISP_GetScreenHeight();
		unsigned int pixel_bpp = bpp/8;
    	unsigned int fbsize = w*h*pixel_bpp;
		unsigned int fbv = (unsigned int)ioremap_cached((unsigned int)LCD_REG->LAYER[FB_LAYER].ADDRESS, fbsize);
		unsigned short* fbvt = (unsigned short*)fbv;

		DBI_LOG("[LCDC init M4U] M4U not been init for LCDC, this should only happened in recovery mode\n");
		for(i = 0;i < h; i++){
			for(j = 0;j < w_xres; j++){
	    		t = fbvt[j + i * w];
 	         	*(unsigned short*)(pvbuf+i*w_xres*pixel_bpp+j*pixel_bpp) = t;
    		}
		}
		iounmap((void *)fbv);
		return LCD_STATUS_OK;
	}
	else
		_m4u_lcdc_func.m4u_alloc_mva(M4U_CLNTMOD_LCDC, pvbuf, DISP_GetScreenHeight()*DISP_GetScreenWidth()*bpp/8, &ppbuf);
#else
    ppbuf = v2p(pvbuf);
#endif

    LCD_WaitForNotBusy();

    _BackupLCDRegisters();

    // begin capture
    if(bpp == 32)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_ARGB8888));
    else if(bpp == 16)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_RGB565));
    else if(bpp == 24)
        LCD_CHECK_RET(LCD_FBSetFormat(LCD_FB_FORMAT_RGB888));
    else
        DBI_LOG("mtkfb_capture_framebuffer, fb color format not support\n");

	mode = LCD_GetOutputMode();
    LCD_CHECK_RET(LCD_LayerSetRotation(LCD_LAYER_ALL, LCD_LAYER_ROTATE_0));
	
	//zaikuo wang, add to capture framebuffer for LCM PHYSICAL rotate 180
	if(0 == strncmp(MTK_LCM_PHYSICAL_ROTATION, "180", 3)){
		for(i = LCD_LAYER_0;i < LCD_LAYER_NUM;i++){
			offset_x = LCD_REG->LAYER[i].OFFSET.X;
			offset_y = LCD_REG->LAYER[i].OFFSET.Y;
			w = LCD_REG->LAYER[i].SIZE.WIDTH;
			h = LCD_REG->LAYER[i].SIZE.HEIGHT;
			offset_x = DISP_GetScreenWidth() - (offset_x + w);
			offset_y = DISP_GetScreenHeight() - (offset_y + h);
			LCD_CHECK_RET(LCD_LayerSetOffset(i, offset_x, offset_y));
		}
	}
    LCD_CHECK_RET(LCD_FBSetPitch(DISP_GetScreenWidth()*bpp/8));
    LCD_CHECK_RET(LCD_FBSetStartCoord(0, 0));
    LCD_CHECK_RET(LCD_FBSetAddress(LCD_FB_0, ppbuf));
    LCD_CHECK_RET(LCD_FBEnable(LCD_FB_0, TRUE));
    
    LCD_CHECK_RET(LCD_SetOutputMode(LCD_OUTPUT_TO_MEM));

    LCD_TE_Enable(FALSE);
	LCD_SetOutputAlpha(0xff);
#if defined(MTK_M4U_SUPPORT)
	_m4u_lcdc_func.m4u_dma_cache_maint(M4U_CLNTMOD_LCDC, (unsigned int*)pvbuf, DISP_GetScreenHeight()*DISP_GetScreenWidth()*bpp/8, DMA_BIDIRECTIONAL);
#endif
	if(dbi_log_on)
		LCD_DumpRegisters();

    LCD_CHECK_RET(LCD_StartTransfer(TRUE));
	LCD_SetOutputMode(mode);
    // capture end
    _RestoreLCDRegisters();
#if defined(MTK_M4U_SUPPORT)
    _m4u_lcdc_func.m4u_dealloc_mva(M4U_CLNTMOD_LCDC, pvbuf, DISP_GetScreenHeight()*DISP_GetScreenWidth()*bpp/8, ppbuf);
#endif
    return LCD_STATUS_OK;    
}

LCD_STATUS LCD_FMDesense_Query()
{
    return LCD_STATUS_OK;
}

LCD_STATUS LCD_FM_Desense(LCD_IF_ID id, unsigned long freq)
{
    UINT32 a,b;
	UINT32 c,d;
	UINT32 wst,c2wh,chw,write_cycles;
	LCD_REG_PCNF config;
//	LCD_REG_WROI_CON ctrl;    
	LCD_REG_PCNFDW pcnfdw;

	OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));
    DBI_LOG("[enter LCD_FM_Desense]:parallel IF = 0x%x, ctrl = 0x%x\n",
		INREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));   
	wst = config.WST;
	c2wh = config.C2WH;
	// Config Delay Between Commands
//	OUTREG32(&ctrl, AS_UINT32(&LCD_REG->WROI_CONTROL));
	OUTREG32(&pcnfdw, AS_UINT32(&LCD_REG->PARALLEL_DW));

	switch(id)
    {
        case LCD_IF_PARALLEL_0: chw = pcnfdw.PCNF0_CHW; break;
        case LCD_IF_PARALLEL_1: chw = pcnfdw.PCNF1_CHW; break;
        case LCD_IF_PARALLEL_2: chw = pcnfdw.PCNF2_CHW; break;
        default : ASSERT(0);
    }

	a = 13000 - freq * 10 - 20;
	b = 13000 - freq * 10 + 20;
    write_cycles = wst + c2wh + chw + 2;//this is 6573 E1, E2 will change
	c = (a * write_cycles)%13000;
	d = (b * write_cycles)%13000;
	a = (a * write_cycles)/13000;
	b = (b * write_cycles)/13000;
	
	if((b > a)||(c == 0)||(d == 0)){//need modify setting to avoid interference
	    DBI_LOG("[LCD_FM_Desense] need to modify lcd setting, freq = %ld\n",freq);
	    wst -= wst_step_LCD;
		wst_step_LCD = 0 - wst_step_LCD;

		config.WST = wst;
		LCD_WaitForNotBusy();
		OUTREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id], AS_UINT32(&config));
	}
	else{
		DBI_LOG("[LCD_FM_Desense] not need to modify lcd setting, freq = %ld\n",freq);
	}
	DBI_LOG("[leave LCD_FM_Desense]:parallel = 0x%x\n", INREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));   
    return LCD_STATUS_OK;  
}

LCD_STATUS LCD_Reset_WriteCycle(LCD_IF_ID id)
{
	LCD_REG_PCNF config;
	UINT32 wst;
	DBI_LOG("[enter LCD_Reset_WriteCycle]:parallel = 0x%x\n", INREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));   
	if(wst_step_LCD > 0){//have modify lcd setting, so when fm turn off, we must decrease wst to default setting
	    DBI_LOG("[LCD_Reset_WriteCycle] need to reset lcd setting\n");
	    OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));
		wst = config.WST;
		wst -= wst_step_LCD;
		wst_step_LCD = 0 - wst_step_LCD;

		config.WST = wst;
		LCD_WaitForNotBusy();
		OUTREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id], AS_UINT32(&config));
	}
	else{
		DBI_LOG("[LCD_Reset_WriteCycle] parallel is default setting, not need to reset it\n");
	}
	DBI_LOG("[leave LCD_Reset_WriteCycle]:parallel = 0x%x\n", INREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));   
	return LCD_STATUS_OK; 
}

LCD_STATUS LCD_Get_Default_WriteCycle(LCD_IF_ID id, unsigned int *write_cycle)
{
	UINT32 wst,c2wh,chw;
	LCD_REG_PCNF config;
//	LCD_REG_WROI_CON ctrl;    
	LCD_REG_PCNFDW pcnfdw;
    
	if(is_get_default_write_cycle){
	    *write_cycle = default_write_cycle;
	    return LCD_STATUS_OK;
	}

	OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));
    DBI_LOG("[enter LCD_Get_Default_WriteCycle]:parallel IF = 0x%x, ctrl = 0x%x\n",
		INREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));   
	wst = config.WST;
	c2wh = config.C2WH;
	// Config Delay Between Commands
//	OUTREG32(&ctrl, AS_UINT32(&LCD_REG->WROI_CONTROL));
	OUTREG32(&pcnfdw, AS_UINT32(&LCD_REG->PARALLEL_DW));
	switch(id)
    {
        case LCD_IF_PARALLEL_0: chw = pcnfdw.PCNF0_CHW; break;
        case LCD_IF_PARALLEL_1: chw = pcnfdw.PCNF1_CHW; break;
        case LCD_IF_PARALLEL_2: chw = pcnfdw.PCNF2_CHW; break;
        default : ASSERT(0);
    }
    *write_cycle = wst + c2wh + chw + 2;
	default_write_cycle = *write_cycle;
	default_wst = wst;
	is_get_default_write_cycle = TRUE;
	DBI_LOG("[leave LCD_Get_Default_WriteCycle]:Default_Write_Cycle = %d\n", *write_cycle);   
    return LCD_STATUS_OK;  
}

LCD_STATUS LCD_Get_Current_WriteCycle(LCD_IF_ID id, unsigned int *write_cycle)
{
	UINT32 wst,c2wh,chw;
	LCD_REG_PCNF config;
//	LCD_REG_WROI_CON ctrl;       
    LCD_REG_PCNFDW pcnfdw;

	OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));
    DBI_LOG("[enter LCD_Get_Current_WriteCycle]:parallel IF = 0x%x, ctrl = 0x%x\n",
		INREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));   
	wst = config.WST;
	c2wh = config.C2WH;
	// Config Delay Between Commands
//	OUTREG32(&ctrl, AS_UINT32(&LCD_REG->WROI_CONTROL));
	OUTREG32(&pcnfdw, AS_UINT32(&LCD_REG->PARALLEL_DW));
	switch(id)
    {
        case LCD_IF_PARALLEL_0: chw = pcnfdw.PCNF0_CHW; break;
        case LCD_IF_PARALLEL_1: chw = pcnfdw.PCNF1_CHW; break;
        case LCD_IF_PARALLEL_2: chw = pcnfdw.PCNF2_CHW; break;
        default : ASSERT(0);
    }

    *write_cycle = wst + c2wh + chw + 2;//this is 6573 E1, E2 will change
	DBI_LOG("[leave LCD_Get_Current_WriteCycle]:Default_Write_Cycle = %d\n", *write_cycle);   
    return LCD_STATUS_OK;  
}

LCD_STATUS LCD_Change_WriteCycle(LCD_IF_ID id, unsigned int write_cycle)
{
	UINT32 wst;
	LCD_REG_PCNF config;

	OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));
    DBI_LOG("[enter LCD_Change_WriteCycle]:parallel IF = 0x%x, ctrl = 0x%x\n",
		INREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id]),INREG32(&LCD_REG->WROI_CONTROL));   

    DBI_LOG("[LCD_Change_WriteCycle] modify lcd setting\n");
	wst = write_cycle - default_write_cycle + default_wst;

	config.WST = wst;
	LCD_WaitForNotBusy();
	OUTREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id], AS_UINT32(&config));
	DBI_LOG("[leave LCD_Change_WriteCycle]:parallel = 0x%x\n", INREG32(&LCD_REG->PARALLEL_CFG[(UINT32)id]));   
    return LCD_STATUS_OK;  
}

#if defined(MTK_M4U_SUPPORT)
LCD_STATUS LCD_InitM4U()
{
	M4U_PORT_STRUCT M4uPort;
	DBI_LOG("[LCDC driver]%s\n", __func__);

	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("[LCDC init M4U] M4U not been init for LCDC\n");
		return LCD_STATUS_ERROR;
	}

	M4uPort.ePortID = M4U_PORT_LCD_R;
	M4uPort.Virtuality = 1; 					   
	M4uPort.Security = 0;
	M4uPort.Distance = 1;
	M4uPort.Direction = 0;

	_m4u_lcdc_func.m4u_config_port(&M4uPort);

	M4uPort.ePortID = M4U_PORT_LCD_W;
	M4uPort.Virtuality = 1; 					   
	M4uPort.Security = 0;
	M4uPort.Distance = 1;
	M4uPort.Direction = 0;

	_m4u_lcdc_func.m4u_config_port(&M4uPort);
//    _m4u_lcdc_func.m4u_dump_reg(M4U_CLNTMOD_LCDC);
	
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_AllocUIMva(unsigned int va, unsigned int *mva, unsigned int size)
{
    if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("[LCDC init M4U] M4U not been init for LCDC\n");
		return LCD_STATUS_ERROR;
	}
    _m4u_lcdc_func.m4u_alloc_mva(M4U_CLNTMOD_LCDC_UI, va, size, mva);
#if 0
	_m4u_lcdc_func.m4u_insert_tlb_range(M4U_CLNTMOD_LCDC_UI,
                                  		 *mva,
                      					 *mva + size - 1,
                      					 RT_RANGE_HIGH_PRIORITY,
										 0);
#endif
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_AllocOverlayMva(unsigned int va, unsigned int *mva, unsigned int size)
{
    if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("[LCDC init M4U] M4U not been init for LCDC\n");
		return LCD_STATUS_ERROR;
	}
	_m4u_lcdc_func.m4u_alloc_mva(M4U_CLNTMOD_LCDC, va, size, mva);
	_m4u_lcdc_func.m4u_insert_tlb_range(M4U_CLNTMOD_LCDC,
                                  		 *mva,
                      					 *mva + size - 1,
                      					 RT_RANGE_HIGH_PRIORITY,
										 0);
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_DeallocMva(unsigned int va, unsigned int mva, unsigned int size)
{
    if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("[TV]Error, M4U has not init func for TV-out\n");
		return LCD_STATUS_ERROR;
	}
    _m4u_lcdc_func.m4u_invalid_tlb_range(M4U_CLNTMOD_LCDC, mva, mva + size - 1);
    _m4u_lcdc_func.m4u_dealloc_mva(M4U_CLNTMOD_LCDC, va, size, mva);
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_M4UPowerOn(void)
{
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_M4UPowerOff(void)
{
	return LCD_STATUS_OK;
}

int m4u_alloc_mva_stub(M4U_MODULE_ID_ENUM eModuleID, const unsigned int BufAddr, const unsigned int BufSize, unsigned int *pRetMVABuf)
{
	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("%s, Error, M4U has not init func\n", __func__);
		return LCD_STATUS_ERROR;
	}
	return _m4u_lcdc_func.m4u_alloc_mva(eModuleID, BufAddr, BufSize, pRetMVABuf);
}
EXPORT_SYMBOL(m4u_alloc_mva_stub);
  
int m4u_dealloc_mva_stub(M4U_MODULE_ID_ENUM eModuleID, const unsigned int BufAddr, const unsigned int BufSize, const unsigned int MVA)
{
	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("%s, Error, M4U has not init func\n", __func__);
		return LCD_STATUS_ERROR;
	}
	return _m4u_lcdc_func.m4u_dealloc_mva(eModuleID, BufAddr, BufSize, MVA);
}
EXPORT_SYMBOL(m4u_dealloc_mva_stub);
                  							
int m4u_insert_tlb_range_stub(M4U_MODULE_ID_ENUM eModuleID, 
                  unsigned int MVAStart, 
                  const unsigned int MVAEnd, 
                  M4U_RANGE_PRIORITY_ENUM ePriority,
                  unsigned int entryCount)
{
	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("%s, Error, M4U has not init func\n", __func__);
		return LCD_STATUS_ERROR;
	}

	return _m4u_lcdc_func.m4u_insert_tlb_range(eModuleID, MVAStart, MVAEnd, ePriority, entryCount);				  
}
EXPORT_SYMBOL(m4u_insert_tlb_range_stub);
                        
int m4u_invalid_tlb_range_stub(M4U_MODULE_ID_ENUM eModuleID, 
                  unsigned int MVAStart, 
                  unsigned int MVAEnd)
{
	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("%s, Error, M4U has not init func\n", __func__);
		return LCD_STATUS_ERROR;
	}

	return _m4u_lcdc_func.m4u_invalid_tlb_range(eModuleID, MVAStart, MVAEnd);				  
}
EXPORT_SYMBOL(m4u_invalid_tlb_range_stub);
             
int m4u_invalid_tlb_all_stub(M4U_MODULE_ID_ENUM eModuleID);  
int m4u_manual_insert_entry_stub(M4U_MODULE_ID_ENUM eModuleID, unsigned int EntryMVA, bool Lock); 
  
int m4u_config_port_stub(M4U_PORT_STRUCT* pM4uPort)
{
	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("%s, Error, M4U has not init func\n", __func__);
		return LCD_STATUS_ERROR;
	}

	return _m4u_lcdc_func.m4u_config_port(pM4uPort);
}
EXPORT_SYMBOL(m4u_config_port_stub);

LCD_STATUS LCD_M4U_On(bool enable)
{
	M4U_PORT_STRUCT M4uPort;
	DBI_LOG("[LCDC driver]%s\n", __func__);

	if (!_m4u_lcdc_func.isInit)
	{
		DBI_LOG("[LCDC M4U ON] M4U not been init for LCDC\n");
		return LCD_STATUS_ERROR;
	}

	M4uPort.ePortID = M4U_PORT_LCD_R;
	M4uPort.Virtuality = enable; 					   
	M4uPort.Security = 0;
	M4uPort.Distance = 1;
	M4uPort.Direction = 0;

	_m4u_lcdc_func.m4u_config_port(&M4uPort);

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_DumpM4U(){
	_m4u_lcdc_func.m4u_dump_reg(M4U_CLNTMOD_LCDC);
	_m4u_lcdc_func.m4u_dump_info(M4U_CLNTMOD_LCDC_UI);
	return LCD_STATUS_OK;
}

#endif

LCD_STATUS LCD_W2M_NeedLimiteSpeed(BOOL enable)
{
	limit_w2m_speed = enable;
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_W2TVR_NeedLimiteSpeed(BOOL enable)
{
	limit_w2tvr_speed = enable;
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_SetGMCThrottle()
{
    UINT16 i, pitch, h;

	UINT32 total_req = 0;
	INT32 throttle_cnt = 0;

	if(!limit_w2m_speed && !limit_w2tvr_speed){
		LCD_OUTREG32(&LCD_REG->GMC_CON, (throttle_cnt << 16) | (0 << 4) | 0x4);
		return LCD_STATUS_OK;
	}

	for(i=0;i<LCD_LAYER_NUM;i++){
		if(LCD_IsLayerEnable((LCD_LAYER_ID)i)){
	  	    pitch = LCD_REG->LAYER[(LCD_LAYER_ID)i].WINDOW_PITCH;
   		    h = LCD_REG->LAYER[(LCD_LAYER_ID)i].SIZE.HEIGHT;
			total_req += pitch * h;
		}
	}

	if(total_req != 0)
		throttle_cnt = 34666666/total_req - 2;//34666666 is calculated by 130MHz/60fps*16
	
	if(throttle_cnt < 0)
		throttle_cnt = 0;	
	LCD_OUTREG32(&LCD_REG->GMC_CON, (throttle_cnt << 16) | (1 << 4) | 0x4);
	
	return LCD_STATUS_OK;
}


// called by "esd_recovery_kthread"
BOOL LCD_esd_check(void)
{
	UINT32 x, y, width, height;

	// Enable TE interrupt
	//LCD_TE_SetMode(LCD_TE_MODE_VSYNC_ONLY);
	//LCD_TE_SetEdgePolarity(LCM_POLARITY_RISING);
	LCD_TE_Enable(TRUE);

	// Backup ROI
	LCD_CHECK_RET(LCD_GetRoiWindow(&x, &y, &width, &height));

	// Set ROI = 0
	LCD_CHECK_RET(LCD_SetRoiWindow(0, 0, 0, 0));

	// Switch to unuse port
	LCD_CHECK_RET(LCD_SelectWriteIF(LCD_IF_PARALLEL_2));

	// Write to LCM
	LCD_CHECK_RET(LCD_SetOutputMode(LCD_OUTPUT_TO_LCM));

	// Blocking Trigger
	// This is to cheat LCDC to wait TE interrupt 
	LCD_CHECK_RET(LCD_StartTransfer(TRUE));

	// Restore ROI
	LCD_CHECK_RET(LCD_SetRoiWindow(x, y, width, height));

	// Disable TE interrupt
	LCD_TE_Enable(FALSE);

	// Write to memory	
	LCD_CHECK_RET(LCD_SetOutputMode(LCD_OUTPUT_TO_MEM));

	return lcd_esd_check;
	
}

