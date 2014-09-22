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
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/xlog.h>

#include <mach/mt_boot.h>
#include <mach/mt_reg_base.h>
#include <mach/irqs.h>
#include <mach/mtk_rtc.h>

#include <rtc-mt.h>		/* custom file */

#define XLOG_MYTAG	"Power/RTC"
#define RTC_NAME	"mt-rtc"

#define RTC_RELPWR_WHEN_XRST	1	/* BBPU = 0 when xreset_rstb goes low */

/* RTC registers */
#define RTC_BBPU	(RTC_BASE + 0x0000)
#define RTC_IRQ_STA	(RTC_BASE + 0x0004)
#define RTC_IRQ_EN	(RTC_BASE + 0x0008)
#define RTC_CII_EN	(RTC_BASE + 0x000c)
#define RTC_AL_MASK	(RTC_BASE + 0x0010)
#define RTC_TC_SEC	(RTC_BASE + 0x0014)
#define RTC_TC_MIN	(RTC_BASE + 0x0018)
#define RTC_TC_HOU	(RTC_BASE + 0x001c)
#define RTC_TC_DOM	(RTC_BASE + 0x0020)
#define RTC_TC_DOW	(RTC_BASE + 0x0024)
#define RTC_TC_MTH	(RTC_BASE + 0x0028)
#define RTC_TC_YEA	(RTC_BASE + 0x002c)
#define RTC_AL_SEC	(RTC_BASE + 0x0030)
#define RTC_AL_MIN	(RTC_BASE + 0x0034)
#define RTC_AL_HOU	(RTC_BASE + 0x0038)
#define RTC_AL_DOM	(RTC_BASE + 0x003c)
#define RTC_AL_DOW	(RTC_BASE + 0x0040)
#define RTC_AL_MTH	(RTC_BASE + 0x0044)
#define RTC_AL_YEA	(RTC_BASE + 0x0048)
#define RTC_OSC32CON	(RTC_BASE + 0x004c)
#define RTC_POWERKEY1	(RTC_BASE + 0x0050)
#define RTC_POWERKEY2	(RTC_BASE + 0x0054)
#define RTC_PDN1	(RTC_BASE + 0x0058)
#define RTC_PDN2	(RTC_BASE + 0x005c)
#define RTC_SPAR0	(RTC_BASE + 0x0060)
#define RTC_SPAR1	(RTC_BASE + 0x0064)
#define RTC_PROT	(RTC_BASE + 0x0068)
#define RTC_DIFF	(RTC_BASE + 0x006c)
#define RTC_CALI	(RTC_BASE + 0x0070)
#define RTC_WRTGR	(RTC_BASE + 0x0074)
#define RTC_CON		(RTC_BASE + 0x0078)

#define RTC_BBPU_PWREN		(1U << 0)	/* BBPU = 1 when alarm occurs */
#define RTC_BBPU_BBPU		(1U << 2)	/* 1: power on, 0: power down */
#define RTC_BBPU_AUTO		(1U << 3)	/* BBPU = 0 when xreset_rstb goes low */
#define RTC_BBPU_RELOAD		(1U << 5)
#define RTC_BBPU_CBUSY		(1U << 6)
#define RTC_BBPU_KEY		(0x43 << 8)

#define RTC_IRQ_STA_AL		(1U << 0)
#define RTC_IRQ_STA_LP		(1U << 3)

#define RTC_IRQ_EN_AL		(1U << 0)
#define RTC_IRQ_EN_ONESHOT	(1U << 2)
#define RTC_IRQ_EN_LP		(1U << 3)
#define RTC_IRQ_EN_ONESHOT_AL	(RTC_IRQ_EN_ONESHOT | RTC_IRQ_EN_AL)

#define RTC_OSC32CON_LNBUFEN	(1U << 11)	/* ungate 32K to ABB */

#define RTC_CON_F32KOB		(1U << 5)	/* 0: RTC_GPIO exports 32K */

/* we map HW YEA 0 (2000) to 1968 not 1970 because 2000 is the leap year */
#define RTC_MIN_YEAR		1968
#define RTC_NUM_YEARS		128
//#define RTC_MAX_YEAR		(RTC_MIN_YEAR + RTC_NUM_YEARS - 1)

#define RTC_MIN_YEAR_OFFSET	(RTC_MIN_YEAR - 1900)

/*
 * RTC_PDN1:
 *     bit 0 - 3  : Android bits
 *     bit 4 - 5  : Recovery bits (0x10: factory data reset)
 *     bit 6      : Bypass PWRKEY bit
 *     bit 7      : Power-On Time bit
 *     bit 8      : RTC_GPIO_USER_WIFI bit
 *     bit 9      : RTC_GPIO_USER_GPS bit
 *     bit 10     : RTC_GPIO_USER_BT bit
 *     bit 11     : RTC_GPIO_USER_FM bit
 *     bit 12     : RTC_GPIO_USER_PMIC bit
 *     bit 13 - 14: reserved bits
 *     bit 15     : Debug bit
 */

/*
 * RTC_PDN2:
 *     bit 0 - 3 : MTH in power-on time
 *     bit 4     : Power-On Alarm bit
 *     bit 5 - 6 : UART bits
 *     bit 7     : reserved bit
 *     bit 8 - 14: YEA in power-on time
 *     bit 15    : Power-On Logo bit
 */

/*
 * RTC_SPAR0:
 *     bit 0 - 5 : SEC in power-on time
 *     bit 6 - 15: reserved bits
 */

/*
 * RTC_SPAR1:
 *     bit 0 - 5  : MIN in power-on time
 *     bit 6 - 10 : HOU in power-on time
 *     bit 11 - 15: DOM in power-on time
 */

#define rtc_read(addr)		(*(volatile u16 *)(addr))
#define rtc_write(addr, val)	(*(volatile u16 *)(addr) = (u16)(val))

#define rtc_busy_wait()					\
do {							\
	while (rtc_read(RTC_BBPU) & RTC_BBPU_CBUSY);	\
} while (0)

#define rtc_xinfo(fmt, args...)		\
	xlog_printk(ANDROID_LOG_INFO, XLOG_MYTAG, fmt, ##args)

#define rtc_xerror(fmt, args...)	\
	xlog_printk(ANDROID_LOG_ERROR, XLOG_MYTAG, fmt, ##args)

#define rtc_xfatal(fmt, args...)	\
	xlog_printk(ANDROID_LOG_FATAL, XLOG_MYTAG, fmt, ##args)

static struct rtc_device *rtc;
static DEFINE_SPINLOCK(rtc_lock);

static int rtc_show_time = 0;
static int rtc_show_alarm = 1;

static void rtc_write_trigger(void)
{
	rtc_write(RTC_WRTGR, 1);
	rtc_busy_wait();
}

static void rtc_writeif_unlock(void)
{
	rtc_write(RTC_PROT, 0x586a);
	rtc_write_trigger();
	rtc_write(RTC_PROT, 0x9136);
	rtc_write_trigger();
}

static void rtc_xosc_write(u16 val, bool reload)
{
	u16 bbpu;

	rtc_write(RTC_OSC32CON, 0x1a57);
	rtc_busy_wait();
	rtc_write(RTC_OSC32CON, 0x2b68);
	rtc_busy_wait();

	rtc_write(RTC_OSC32CON, val);
	rtc_busy_wait();

	if (reload) {
		bbpu = rtc_read(RTC_BBPU) | RTC_BBPU_KEY | RTC_BBPU_RELOAD;
		rtc_write(RTC_BBPU, bbpu);
		rtc_write_trigger();
	}
}

void rtc_gpio_enable_32k(rtc_gpio_user_t user)
{
	u16 con, pdn1;
	unsigned long flags;

	if (user < RTC_GPIO_USER_WIFI || user > RTC_GPIO_USER_PMIC)
		return;

	spin_lock_irqsave(&rtc_lock, flags);
	pdn1 = rtc_read(RTC_PDN1);
	if (!(pdn1 & RTC_GPIO_USER_MASK)) {	/* first user */
		con = rtc_read(RTC_CON) & ~RTC_CON_F32KOB;
		rtc_write(RTC_CON, con);
		rtc_write_trigger();
	}

	pdn1 |= (1U << user);
	rtc_write(RTC_PDN1, pdn1);
	rtc_write_trigger();
	spin_unlock_irqrestore(&rtc_lock, flags);

	rtc_xinfo("RTC_GPIO user %d enables 32k (0x%x)\n", user, pdn1);
}
EXPORT_SYMBOL(rtc_gpio_enable_32k);

void rtc_gpio_disable_32k(rtc_gpio_user_t user)
{
	u16 con, pdn1;
	unsigned long flags;

	if (user < RTC_GPIO_USER_WIFI || user > RTC_GPIO_USER_PMIC)
		return;

	spin_lock_irqsave(&rtc_lock, flags);
	pdn1 = rtc_read(RTC_PDN1) & ~(1U << user);
	rtc_write(RTC_PDN1, pdn1);
	rtc_write_trigger();

	if (!(pdn1 & RTC_GPIO_USER_MASK)) {	/* no users */
		con = rtc_read(RTC_CON) | RTC_CON_F32KOB;
		rtc_write(RTC_CON, con);
		rtc_write_trigger();
	}
	spin_unlock_irqrestore(&rtc_lock, flags);

	rtc_xinfo("RTC_GPIO user %d disables 32k (0x%x)\n", user, pdn1);
}
EXPORT_SYMBOL(rtc_gpio_disable_32k);

void rtc_enable_abb_32k(void)
{
	u16 con;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	con = rtc_read(RTC_OSC32CON) | RTC_OSC32CON_LNBUFEN;
	rtc_xosc_write(con, true);
	spin_unlock_irqrestore(&rtc_lock, flags);

	rtc_xinfo("enable ABB 32k (0x%x)\n", con);
}

void rtc_disable_abb_32k(void)
{
	u16 con;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	con = rtc_read(RTC_OSC32CON) & ~RTC_OSC32CON_LNBUFEN;
	rtc_xosc_write(con, true);
	spin_unlock_irqrestore(&rtc_lock, flags);

	rtc_xinfo("disable ABB 32k (0x%x)\n", con);
}

void rtc_enable_writeif(void)
{
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	rtc_write(RTC_PROT, 0x586a);
	rtc_write_trigger();
	rtc_write(RTC_PROT, 0x9136);
	rtc_write_trigger();
	spin_unlock_irqrestore(&rtc_lock, flags);
}

void rtc_disable_writeif(void)
{
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	rtc_write(RTC_PROT, 0);
	rtc_write_trigger();
	spin_unlock_irqrestore(&rtc_lock, flags);
}

void rtc_mark_recovery(void)
{
	u16 pdn1;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	pdn1 = rtc_read(RTC_PDN1) & ~0x0030;
	rtc_write(RTC_PDN1, pdn1 | 0x0010);
	rtc_write_trigger();
	spin_unlock_irqrestore(&rtc_lock, flags);
}

u16 rtc_rdwr_uart_bits(u16 *val)
{
	u16 pdn2;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	if (val) {
		pdn2 = rtc_read(RTC_PDN2) & ~0x0060;
		pdn2 |= (*val & 0x0003) << 5;
		rtc_write(RTC_PDN2, pdn2);
		rtc_write_trigger();
	}
	pdn2 = rtc_read(RTC_PDN2);
	spin_unlock_irqrestore(&rtc_lock, flags);

	return (pdn2 & 0x0060) >> 5;
}

void rtc_bbpu_power_down(void)
{
	u16 bbpu, con;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	rtc_writeif_unlock();

	/* disable 32K export if there are no RTC_GPIO users */
	if (!(rtc_read(RTC_PDN1) & RTC_GPIO_USER_MASK)) {
		con = rtc_read(RTC_CON) | RTC_CON_F32KOB;
		rtc_write(RTC_CON, con);
		rtc_write_trigger();
	}

	/* pull PWRBB low */
	bbpu = RTC_BBPU_KEY | RTC_BBPU_AUTO | RTC_BBPU_PWREN;
	rtc_write(RTC_BBPU, bbpu);
	rtc_write_trigger();
	spin_unlock_irqrestore(&rtc_lock, flags);
}

void rtc_read_pwron_alarm(struct rtc_wkalrm *alm)
{
	u16 pdn1, pdn2, spar0, spar1;
	unsigned long flags;
	struct rtc_time *tm;

	if (alm == NULL)
		return;
	tm = &alm->time;

	spin_lock_irqsave(&rtc_lock, flags);
	pdn1 = rtc_read(RTC_PDN1);
	pdn2 = rtc_read(RTC_PDN2);
	spar0 = rtc_read(RTC_SPAR0);
	spar1 = rtc_read(RTC_SPAR1);
	spin_unlock_irqrestore(&rtc_lock, flags);

	alm->enabled = (pdn1 & 0x0080 ? (pdn2 & 0x8000 ? 3 : 2) : 0);
	alm->pending = !!(pdn2 & 0x0010);	/* return Power-On Alarm bit */
	tm->tm_year = ((pdn2 & 0x7f00) >> 8) + RTC_MIN_YEAR_OFFSET;
	tm->tm_mon = (pdn2 & 0x000f) - 1;
	tm->tm_mday = (spar1 & 0xf800) >> 11;
	tm->tm_hour = (spar1 & 0x07c0) >> 6;
	tm->tm_min = spar1 & 0x003f;
	tm->tm_sec = spar0 & 0x003f;

	if (rtc_show_alarm) {
		rtc_xinfo("power-on = %04d/%02d/%02d %02d:%02d:%02d (%d)(%d)\n",
		          tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		          tm->tm_hour, tm->tm_min, tm->tm_sec,
		          alm->enabled, alm->pending);
	}
}

#ifndef USER_BUILD_KERNEL
static void rtc_lp_exception(void)
{
	u16 bbpu, irqsta, irqen, osc32;
	u16 pwrkey1, pwrkey2, prot, con, sec1, sec2;

	bbpu = rtc_read(RTC_BBPU);
	irqsta = rtc_read(RTC_IRQ_STA);
	irqen = rtc_read(RTC_IRQ_EN);
	osc32 = rtc_read(RTC_OSC32CON);
	pwrkey1 = rtc_read(RTC_POWERKEY1);
	pwrkey2 = rtc_read(RTC_POWERKEY2);
	prot = rtc_read(RTC_PROT);
	con = rtc_read(RTC_CON);
	sec1 = rtc_read(RTC_TC_SEC);
	mdelay(2000);
	sec2 = rtc_read(RTC_TC_SEC);

	rtc_xfatal("!!! 32K WAS STOPPED !!!\n"
	           "RTC_BBPU      = 0x%x\n"
	           "RTC_IRQ_STA   = 0x%x\n"
	           "RTC_IRQ_EN    = 0x%x\n"
	           "RTC_OSC32CON  = 0x%x\n"
	           "RTC_POWERKEY1 = 0x%x\n"
	           "RTC_POWERKEY2 = 0x%x\n"
	           "RTC_PROT      = 0x%x\n"
	           "RTC_CON       = 0x%x\n"
	           "RTC_TC_SEC    = %02d\n"
	           "RTC_TC_SEC    = %02d\n",
	           bbpu, irqsta, irqen, osc32,
	           pwrkey1, pwrkey2, prot, con, sec1, sec2);
}
#endif

static void rtc_tasklet_handler(unsigned long data)
{
	u16 irqsta, pdn1, pdn2, spar0, spar1;
	bool pwron_alm = false;

	spin_lock(&rtc_lock);
	irqsta = rtc_read(RTC_IRQ_STA);		/* read clear */
	if (unlikely(!(irqsta & RTC_IRQ_STA_AL))) {
#ifndef USER_BUILD_KERNEL
		if (irqsta & RTC_IRQ_STA_LP)
			rtc_lp_exception();
#endif
		spin_unlock(&rtc_lock);
		enable_irq(MT_RTC_IRQ_ID);
		return;
	}

#if RTC_RELPWR_WHEN_XRST
{
	/* set AUTO bit because AUTO = 0 when PWREN = 1 and alarm occurs */
	u16 bbpu = rtc_read(RTC_BBPU) | RTC_BBPU_KEY | RTC_BBPU_AUTO;
	rtc_write(RTC_BBPU, bbpu);
	rtc_write_trigger();
}
#endif

	pdn1 = rtc_read(RTC_PDN1);
	pdn2 = rtc_read(RTC_PDN2);
	spar0 = rtc_read(RTC_SPAR0);
	spar1 = rtc_read(RTC_SPAR1);
	if (pdn1 & 0x0080) {	/* power-on time is available */
		u16 now_sec, now_min, now_hou, now_dom, now_mth, now_yea;
		u16 irqen, sec, min, hou, dom, mth, yea;
		unsigned long now_time, time;

		now_sec = rtc_read(RTC_TC_SEC);
		now_min = rtc_read(RTC_TC_MIN);
		now_hou = rtc_read(RTC_TC_HOU);
		now_dom = rtc_read(RTC_TC_DOM);
		now_mth = rtc_read(RTC_TC_MTH);
		now_yea = rtc_read(RTC_TC_YEA) + RTC_MIN_YEAR;
		if (rtc_read(RTC_TC_SEC) < now_sec) {	/* SEC has carried */
			now_sec = rtc_read(RTC_TC_SEC);
			now_min = rtc_read(RTC_TC_MIN);
			now_hou = rtc_read(RTC_TC_HOU);
			now_dom = rtc_read(RTC_TC_DOM);
			now_mth = rtc_read(RTC_TC_MTH);
			now_yea = rtc_read(RTC_TC_YEA) + RTC_MIN_YEAR;
		}

		sec = spar0 & 0x003f;
		min = spar1 & 0x003f;
		hou = (spar1 & 0x07c0) >> 6;
		dom = (spar1 & 0xf800) >> 11;
		mth = pdn2 & 0x000f;
		yea = ((pdn2 & 0x7f00) >> 8) + RTC_MIN_YEAR;

		now_time = mktime(now_yea, now_mth, now_dom, now_hou, now_min, now_sec);
		time = mktime(yea, mth, dom, hou, min, sec);

		if (now_time >= time - 1 && now_time <= time + 4) {	/* power on */
			rtc_write(RTC_PDN1, pdn1 & ~0x0080);
			rtc_write(RTC_PDN2, pdn2 | 0x0010);
			rtc_write_trigger();
			pwron_alm = true;
		} else if (now_time < time) {	/* set power-on alarm */
			rtc_write(RTC_AL_YEA, yea - RTC_MIN_YEAR);
			rtc_write(RTC_AL_MTH, mth);
			rtc_write(RTC_AL_DOM, dom);
			rtc_write(RTC_AL_HOU, hou);
			rtc_write(RTC_AL_MIN, min);
			rtc_write(RTC_AL_SEC, sec);
			rtc_write(RTC_AL_MASK, 0x0010);		/* mask DOW */
			rtc_write_trigger();
			irqen = rtc_read(RTC_IRQ_EN) | RTC_IRQ_EN_ONESHOT_AL;
			rtc_write(RTC_IRQ_EN, irqen);
			rtc_write_trigger();
		}
	}
	spin_unlock(&rtc_lock);

	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);

	if (rtc_show_alarm)
		rtc_xinfo("%s time is up\n", pwron_alm ? "power-on" : "alarm");

	enable_irq(MT_RTC_IRQ_ID);
}
static DECLARE_TASKLET(rtc_tasklet, rtc_tasklet_handler, 0);

static irqreturn_t rtc_irq_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
	disable_irq_nosync(MT_RTC_IRQ_ID);

	tasklet_schedule(&rtc_tasklet);

	return IRQ_HANDLED;
}

#if RTC_OVER_TIME_RESET
static void rtc_reset_to_deftime(struct rtc_time *tm)
{
	unsigned long flags;

	tm->tm_year = RTC_DEFAULT_YEA - 1900;
	tm->tm_mon = RTC_DEFAULT_MTH - 1;
	tm->tm_mday = RTC_DEFAULT_DOM;
	tm->tm_wday = 1;
	tm->tm_hour = 0;
	tm->tm_min = 0;
	tm->tm_sec = 0;

	spin_lock_irqsave(&rtc_lock, flags);
	rtc_write(RTC_TC_YEA, RTC_DEFAULT_YEA - RTC_MIN_YEAR);
	rtc_write(RTC_TC_MTH, RTC_DEFAULT_MTH);
	rtc_write(RTC_TC_DOM, RTC_DEFAULT_DOM);
	rtc_write(RTC_TC_DOW, 1);
	rtc_write(RTC_TC_HOU, 0);
	rtc_write(RTC_TC_MIN, 0);
	rtc_write(RTC_TC_SEC, 0);
	rtc_write_trigger();
	spin_unlock_irqrestore(&rtc_lock, flags);

	rtc_xerror("reset to default date %04d/%02d/%02d\n",
	           RTC_DEFAULT_YEA, RTC_DEFAULT_MTH, RTC_DEFAULT_DOM);
}
#endif

static int rtc_ops_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time, flags;

	spin_lock_irqsave(&rtc_lock, flags);
	tm->tm_sec = rtc_read(RTC_TC_SEC);
	tm->tm_min = rtc_read(RTC_TC_MIN);
	tm->tm_hour = rtc_read(RTC_TC_HOU);
	tm->tm_mday = rtc_read(RTC_TC_DOM);
	tm->tm_mon = rtc_read(RTC_TC_MTH);
	tm->tm_year = rtc_read(RTC_TC_YEA);
	if (rtc_read(RTC_TC_SEC) < tm->tm_sec) {	/* SEC has carried */
		tm->tm_sec = rtc_read(RTC_TC_SEC);
		tm->tm_min = rtc_read(RTC_TC_MIN);
		tm->tm_hour = rtc_read(RTC_TC_HOU);
		tm->tm_mday = rtc_read(RTC_TC_DOM);
		tm->tm_mon = rtc_read(RTC_TC_MTH);
		tm->tm_year = rtc_read(RTC_TC_YEA);
	}
	spin_unlock_irqrestore(&rtc_lock, flags);

	tm->tm_year += RTC_MIN_YEAR_OFFSET;
	tm->tm_mon--;
	rtc_tm_to_time(tm, &time);
#if RTC_OVER_TIME_RESET
	if (unlikely(time > (unsigned long)LONG_MAX)) {
		rtc_reset_to_deftime(tm);
		rtc_tm_to_time(tm, &time);
	}
#endif
	tm->tm_wday = (time / 86400 + 4) % 7;	/* 1970/01/01 is Thursday */

	if (rtc_show_time) {
		rtc_xinfo("read tc time = %04d/%02d/%02d (%d) %02d:%02d:%02d\n",
		          tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		          tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	}

	return 0;
}

static int rtc_ops_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time, flags;

	rtc_tm_to_time(tm, &time);
	if (time > (unsigned long)LONG_MAX)
		return -EINVAL;

	tm->tm_year -= RTC_MIN_YEAR_OFFSET;
	tm->tm_mon++;

	rtc_xinfo("set tc time = %04d/%02d/%02d %02d:%02d:%02d\n",
	          tm->tm_year + RTC_MIN_YEAR, tm->tm_mon, tm->tm_mday,
	          tm->tm_hour, tm->tm_min, tm->tm_sec);

	spin_lock_irqsave(&rtc_lock, flags);
	rtc_write(RTC_TC_YEA, tm->tm_year);
	rtc_write(RTC_TC_MTH, tm->tm_mon);
	rtc_write(RTC_TC_DOM, tm->tm_mday);
	rtc_write(RTC_TC_HOU, tm->tm_hour);
	rtc_write(RTC_TC_MIN, tm->tm_min);
	rtc_write(RTC_TC_SEC, tm->tm_sec);
	rtc_write_trigger();
	spin_unlock_irqrestore(&rtc_lock, flags);

	return 0;
}

static int rtc_ops_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	u16 irqen, pdn2;
	unsigned long flags;
	struct rtc_time *tm = &alm->time;

	spin_lock_irqsave(&rtc_lock, flags);
	irqen = rtc_read(RTC_IRQ_EN);
	tm->tm_sec = rtc_read(RTC_AL_SEC);
	tm->tm_min = rtc_read(RTC_AL_MIN);
	tm->tm_hour = rtc_read(RTC_AL_HOU);
	tm->tm_mday = rtc_read(RTC_AL_DOM);
	tm->tm_mon = rtc_read(RTC_AL_MTH);
	tm->tm_year = rtc_read(RTC_AL_YEA);
	pdn2 = rtc_read(RTC_PDN2);
	spin_unlock_irqrestore(&rtc_lock, flags);

	alm->enabled = !!(irqen & RTC_IRQ_EN_AL);
	alm->pending = !!(pdn2 & 0x0010);	/* return Power-On Alarm bit */
	tm->tm_year += RTC_MIN_YEAR_OFFSET;
	tm->tm_mon--;

	return 0;
}

static void rtc_save_pwron_time(bool enable, struct rtc_time *tm, bool logo)
{
	u16 pdn1, pdn2, spar0, spar1;

	pdn2 = rtc_read(RTC_PDN2) & 0x00f0;
	pdn2 |= (tm->tm_year << 8) | tm->tm_mon;
	if (logo)
		pdn2 |= 0x8000;

	spar1 = (tm->tm_mday << 11) | (tm->tm_hour << 6) | tm->tm_min;
	spar0 = rtc_read(RTC_SPAR0) & 0xffc0;
	spar0 |= tm->tm_sec;

	rtc_write(RTC_PDN2, pdn2);
	rtc_write(RTC_SPAR1, spar1);
	rtc_write(RTC_SPAR0, spar0);
	if (enable) {
		pdn1 = rtc_read(RTC_PDN1) | 0x0080;
		rtc_write(RTC_PDN1, pdn1);
	} else {
		pdn1 = rtc_read(RTC_PDN1) & ~0x0080;
		rtc_write(RTC_PDN1, pdn1);
	}
	rtc_write_trigger();
}

static int rtc_ops_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	u16 irqsta, irqen, pdn2;
	unsigned long time, flags;
	struct rtc_time *tm = &alm->time;

	rtc_tm_to_time(tm, &time);
	if (time > (unsigned long)LONG_MAX)
		return -EINVAL;

	tm->tm_year -= RTC_MIN_YEAR_OFFSET;
	tm->tm_mon++;

	rtc_xinfo("set al time = %04d/%02d/%02d %02d:%02d:%02d (%d)\n",
	          tm->tm_year + RTC_MIN_YEAR, tm->tm_mon, tm->tm_mday,
	          tm->tm_hour, tm->tm_min, tm->tm_sec, alm->enabled);

	spin_lock_irqsave(&rtc_lock, flags);
	if (alm->enabled == 2) {	/* enable power-on alarm */
		rtc_save_pwron_time(true, tm, false);
	} else if (alm->enabled == 3) {		/* enable power-on alarm with logo */
		rtc_save_pwron_time(true, tm, true);
	} else if (alm->enabled == 4) {		/* disable power-on alarm */
		alm->enabled = 0;
		rtc_save_pwron_time(false, tm, false);
	}

	/* disable alarm and clear Power-On Alarm bit */
	irqen = rtc_read(RTC_IRQ_EN) & ~RTC_IRQ_EN_AL;
	pdn2 = rtc_read(RTC_PDN2) & ~0x0010;
	rtc_write(RTC_IRQ_EN, irqen);
	rtc_write(RTC_PDN2, pdn2);
	rtc_write_trigger();
	irqsta = rtc_read(RTC_IRQ_STA);		/* read clear */

	if (alm->enabled) {
		rtc_write(RTC_AL_YEA, tm->tm_year);
		rtc_write(RTC_AL_MTH, tm->tm_mon);
		rtc_write(RTC_AL_DOM, tm->tm_mday);
		rtc_write(RTC_AL_HOU, tm->tm_hour);
		rtc_write(RTC_AL_MIN, tm->tm_min);
		rtc_write(RTC_AL_SEC, tm->tm_sec);
		rtc_write(RTC_AL_MASK, 0x0010);		/* mask DOW */
		rtc_write_trigger();
		irqen = rtc_read(RTC_IRQ_EN) | RTC_IRQ_EN_ONESHOT_AL;
		rtc_write(RTC_IRQ_EN, irqen);
		rtc_write_trigger();
	}
	spin_unlock_irqrestore(&rtc_lock, flags);

	return 0;
}

static struct rtc_class_ops rtc_ops = {
	.read_time	= rtc_ops_read_time,
	.set_time	= rtc_ops_set_time,
	.read_alarm	= rtc_ops_read_alarm,
	.set_alarm	= rtc_ops_set_alarm,
};

static int rtc_pdrv_probe(struct platform_device *pdev)
{
	u16 irqen;
	int r;
	unsigned long flags;

	/* only enable LPD interrupt in engineering build */
	spin_lock_irqsave(&rtc_lock, flags);
	rtc_writeif_unlock();
#ifndef USER_BUILD_KERNEL
	irqen = rtc_read(RTC_IRQ_EN) | RTC_IRQ_EN_LP;
#else
	irqen = rtc_read(RTC_IRQ_EN) & ~RTC_IRQ_EN_LP;
#endif
	rtc_write(RTC_IRQ_EN, irqen);
	rtc_write_trigger();
	spin_unlock_irqrestore(&rtc_lock, flags);

	/* register rtc device (/dev/rtc0) */
	rtc = rtc_device_register(RTC_NAME, &pdev->dev, &rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		rtc_xerror("register rtc device failed (%ld)\n", PTR_ERR(rtc));
		return PTR_ERR(rtc);
	}

	r = request_irq(MT_RTC_IRQ_ID, rtc_irq_handler, IRQF_TRIGGER_LOW,
	                RTC_NAME, NULL);
	if (r) {
		rtc_xerror("register IRQ failed (%d)\n", r);
		rtc_device_unregister(rtc);
		return r;
	}

	return 0;
}

/* should never be called */
static int rtc_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rtc_pdrv = {
	.probe		= rtc_pdrv_probe,
	.remove		= rtc_pdrv_remove,
	.driver		= {
		.name	= RTC_NAME,
		.owner	= THIS_MODULE,
	},
};

static struct platform_device rtc_pdev = {
	.name	= RTC_NAME,
	.id	= -1,
};

static int __init rtc_mod_init(void)
{
	int r;

	r = platform_device_register(&rtc_pdev);
	if (r) {
		rtc_xerror("register device failed (%d)\n", r);
		return r;
	}

	r = platform_driver_register(&rtc_pdrv);
	if (r) {
		rtc_xerror("register driver failed (%d)\n", r);
		platform_device_unregister(&rtc_pdev);
		return r;
	}

	return 0;
}

/* should never be called */
static void __exit rtc_mod_exit(void)
{
}

static int __init rtc_late_init(void)
{
	u16 irqen, pdn1;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	irqen = rtc_read(RTC_IRQ_EN);
	pdn1 = rtc_read(RTC_PDN1);
	spin_unlock_irqrestore(&rtc_lock, flags);

	rtc_xinfo("RTC_IRQ_EN = 0x%x, RTC_PDN1 = 0x%x\n", irqen, pdn1);

	return 0;
}

module_init(rtc_mod_init);
module_exit(rtc_mod_exit);

late_initcall(rtc_late_init);

module_param(rtc_show_time, int, 0644);
module_param(rtc_show_alarm, int, 0644);

MODULE_AUTHOR("Terry Chang <terry.chang@mediatek.com>");
MODULE_DESCRIPTION("MT6577 RTC Driver v3.0");
MODULE_LICENSE("GPL");
