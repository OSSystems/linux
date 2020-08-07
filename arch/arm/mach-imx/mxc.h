/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2004-2007, 2010-2015 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 */

#ifndef __ASM_ARCH_MXC_H__
#define __ASM_ARCH_MXC_H__

#include <linux/types.h>

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

#define MXC_CPU_MX1		1
#define MXC_CPU_MX21		21
#define MXC_CPU_MX25		25
#define MXC_CPU_MX27		27
#define MXC_CPU_MX31		31
#define MXC_CPU_MX35		35
#define MXC_CPU_MX51		51
#define MXC_CPU_MX53		53
#define MXC_CPU_IMX6SL		0x60
#define MXC_CPU_IMX6DL		0x61
#define MXC_CPU_IMX6SX		0x62
#define MXC_CPU_IMX6Q		0x63
#define MXC_CPU_IMX6UL		0x64
#define MXC_CPU_IMX6ULL		0x65
/* virtual cpu id for i.mx6ulz */
#define MXC_CPU_IMX6ULZ		0x6b
#define MXC_CPU_IMX6SLL		0x67
#define MXC_CPU_IMX7D		0x72
#define MXC_CPU_IMX7ULP		0xff

#define IMX_DDR_TYPE_DDR3		0
#define IMX_DDR_TYPE_LPDDR2		1
#define IMX_DDR_TYPE_LPDDR3		2
#define IMX_MMDC_DDR_TYPE_LPDDR3	3

#define IMX_LPDDR2_1CH_MODE            0
#define IMX_LPDDR2_2CH_MODE            1

#ifndef __ASSEMBLY__
extern unsigned int __mxc_cpu_type;

static inline bool cpu_is_imx6sl(void)
{
#ifdef CONFIG_SOC_IMX6SL
	return __mxc_cpu_type == MXC_CPU_IMX6SL;
#else
	return false;
#endif
}

static inline bool cpu_is_imx6dl(void)
{
#ifdef CONFIG_SOC_IMX6Q /* Q means Q/DL in this case */
	return __mxc_cpu_type == MXC_CPU_IMX6DL;
#else
	return false;
#endif
}

static inline bool cpu_is_imx6sx(void)
{
#ifdef CONFIG_SOC_IMX6SX
	return __mxc_cpu_type == MXC_CPU_IMX6SX;
#else
	return false;
#endif
}

static inline bool cpu_is_imx6ul(void)
{
#ifdef CONFIG_SOC_IMX6UL
	return __mxc_cpu_type == MXC_CPU_IMX6UL;
#else
	return false;
#endif
}

static inline bool cpu_is_imx6ull(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6ULL;
}

static inline bool cpu_is_imx6ulz(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6ULZ;
}

static inline bool cpu_is_imx6sll(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SLL;
}

static inline bool cpu_is_imx6q(void)
{
#ifdef CONFIG_SOC_IMX6Q
	return __mxc_cpu_type == MXC_CPU_IMX6Q;
#else
	return false;
#endif
}

static inline bool cpu_is_imx6(void)
{
#ifdef CONFIG_SOC_IMX6
	return __mxc_cpu_type == MXC_CPU_IMX6Q ||
		__mxc_cpu_type == MXC_CPU_IMX6DL ||
		__mxc_cpu_type == MXC_CPU_IMX6SL ||
		__mxc_cpu_type == MXC_CPU_IMX6SX ||
		__mxc_cpu_type == MXC_CPU_IMX6UL ||
		__mxc_cpu_type == MXC_CPU_IMX6ULL ||
		__mxc_cpu_type == MXC_CPU_IMX6SLL ||
		__mxc_cpu_type == MXC_CPU_IMX6ULZ;
#else
	return false;
#endif
}

static inline bool cpu_is_imx7d(void)
{
#ifdef CONFIG_SOC_IMX7D
	return __mxc_cpu_type == MXC_CPU_IMX7D;
#else
	return false;
#endif
}

static inline bool cpu_is_imx7ulp(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX7ULP;
}

struct cpu_op {
	u32 cpu_rate;
};

int tzic_enable_wake(void);

extern struct cpu_op *(*get_cpu_op)(int *op);
#endif

#define imx_readl	readl_relaxed
#define imx_readw	readw_relaxed
#define imx_writel	writel_relaxed
#define imx_writew	writew_relaxed

#endif /*  __ASM_ARCH_MXC_H__ */
