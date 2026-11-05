// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Copyright (C) 2013 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 */

#define pr_fmt(fmt) "psci: " fmt

#include <linux/init.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/psci.h>
#include <linux/mm.h>

#include <uapi/linux/psci.h>

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <asm/barrier.h>
#include <asm/cpu_ops.h>
#include <asm/errno.h>
#include <asm/smp_plat.h>

static int __init cpu_psci_cpu_init(unsigned int cpu)
{
	return 0;
}

static int __init cpu_psci_cpu_prepare(unsigned int cpu)
{
	if (!psci_ops.cpu_on) {
		pr_err("no cpu_on method, not booting CPU%d\n", cpu);
		return -ENODEV;
	}

	return 0;
}

#ifdef CONFIG_ARCH_MEDIATEK
/*
 * MT6755: the stock kernel powers a secondary core's MTCMOS domain through the
 * SPM right after PSCI CPU_ON (mt_psci_cpu_boot -> spm_mtcmos_ctrl_cpu).
 * Without it the core never starts. Only CPU1 is known so far.
 */
#define MT6755_SPM_PHYS	0x10006000
#define MT6755_SPM_ST1	0x180
#define MT6755_SPM_ST2	0x184

static const struct {
	u32 pwr_con;	/* PWR_CON register offset */
	u32 l1pdn;	/* L1 cache power-down register offset */
	u32 st_bit;	/* power status bit in ST1/ST2 */
	u32 cl_bit;	/* status bit of the cluster this core is in */
} mt6755_cpu_pwr[8] = {
	[1] = { 0x20c, 0x24c, 10, 8 },
	[2] = { 0x210, 0x250, 11, 8 },
	[3] = { 0x214, 0x254, 12, 8 },
	[4] = { 0x21c, 0x260, 16, 15 },
	[5] = { 0x220, 0x264, 17, 15 },
	[6] = { 0x224, 0x268, 18, 15 },
	[7] = { 0x228, 0x26c, 19, 15 },
};

static void __iomem *mt6755_spm;
static DEFINE_SPINLOCK(mt6755_spm_lock);

static void mt6755_rmw(u32 off, u32 clr, u32 set)
{
	u32 v = readl_relaxed(mt6755_spm + off);

	writel_relaxed((v & ~clr) | set, mt6755_spm + off);
	dsb(sy);
}

static bool mt6755_present(void)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-sysirq");
	bool found = np != NULL;

	of_node_put(np);
	return found;
}

static int mt6755_smp_power_on(unsigned int cpu)
{
	unsigned long flags;
	u32 v, pc, l1, bit;
	int ret = 0;

	if (!mt6755_present()) {
		pr_info("mt6755: sysirq node not found, hook skipped\n");
		return 0;
	}
	msleep(10);	/* give the firmware a moment after CPU_ON */
	if (cpu >= ARRAY_SIZE(mt6755_cpu_pwr) || !mt6755_cpu_pwr[cpu].pwr_con) {
		pr_warn("mt6755: no SPM power-on sequence for CPU%u yet\n", cpu);
		return -ENODEV;
	}
	if (!mt6755_spm) {
		mt6755_spm = ioremap(MT6755_SPM_PHYS, SZ_4K);
		if (!mt6755_spm)
			return -ENOMEM;
	}

	pc = mt6755_cpu_pwr[cpu].pwr_con;
	l1 = mt6755_cpu_pwr[cpu].l1pdn;
	bit = BIT(mt6755_cpu_pwr[cpu].st_bit);

	spin_lock_irqsave(&mt6755_spm_lock, flags);
	writel_relaxed(0x0b160001, mt6755_spm);
	dsb(sy);

	if (!(readl_relaxed(mt6755_spm + MT6755_SPM_ST1) & BIT(mt6755_cpu_pwr[cpu].cl_bit))) {
		pr_err("mt6755: cluster of CPU%u is not powered, not handled\n", cpu);
		ret = -ENODEV;
		goto out;
	}

	mt6755_rmw(pc, 0, BIT(2));	/* PWR_ON */
	mt6755_rmw(pc, 0, BIT(3));	/* PWR_ON_2ND */
	if (readl_relaxed_poll_timeout_atomic(mt6755_spm + MT6755_SPM_ST1, v, v & bit, 10, 1000000) ||
	    readl_relaxed_poll_timeout_atomic(mt6755_spm + MT6755_SPM_ST2, v, v & bit, 10, 1000000)) {
		pr_err("mt6755: CPU%u domain did not power up\n", cpu);
		ret = -ETIMEDOUT;
		goto out;
	}
	mt6755_rmw(pc, BIT(1), 0);	/* ISO off */
	mt6755_rmw(l1, BIT(0), 0);	/* L1 cache power-down off */
	if (readl_relaxed_poll_timeout_atomic(mt6755_spm + l1, v, !(v & BIT(8)), 10, 1000000)) {
		pr_err("mt6755: CPU%u L1 power-down ack timeout\n", cpu);
		ret = -ETIMEDOUT;
		goto out;
	}
	udelay(10);
	mt6755_rmw(pc, 0, BIT(6));	/* SRAM_ISOINT_B on */
	mt6755_rmw(pc, BIT(5), 0);	/* SRAM_CKISO off */
	mt6755_rmw(pc, BIT(4), 0);	/* CLK_DIS off */
	mt6755_rmw(pc, 0, BIT(0));	/* RST_B on */
out:
	spin_unlock_irqrestore(&mt6755_spm_lock, flags);
	if (!ret)
		pr_info("mt6755: CPU%u powered up through SPM\n", cpu);
	return ret;
}
#else
static inline int mt6755_smp_power_on(unsigned int cpu) { return 0; }
#endif

static int cpu_psci_cpu_boot(unsigned int cpu)
{
	phys_addr_t pa_secondary_entry = __pa_symbol(secondary_entry);
	int err = psci_ops.cpu_on(cpu_logical_map(cpu), pa_secondary_entry);
	if (err)
		pr_err("failed to boot CPU%d (%d)\n", cpu, err);

	if (!err && mt6755_smp_power_on(cpu))
		pr_err("MT6755: SPM power-on of CPU%d failed\n", cpu);

	return err;
}

#ifdef CONFIG_HOTPLUG_CPU
static bool cpu_psci_cpu_can_disable(unsigned int cpu)
{
	return !psci_tos_resident_on(cpu);
}

static int cpu_psci_cpu_disable(unsigned int cpu)
{
	/* Fail early if we don't have CPU_OFF support */
	if (!psci_ops.cpu_off)
		return -EOPNOTSUPP;

	/* Trusted OS will deny CPU_OFF */
	if (psci_tos_resident_on(cpu))
		return -EPERM;

	return 0;
}

static void cpu_psci_cpu_die(unsigned int cpu)
{
	/*
	 * There are no known implementations of PSCI actually using the
	 * power state field, pass a sensible default for now.
	 */
	u32 state = PSCI_POWER_STATE_TYPE_POWER_DOWN <<
		    PSCI_0_2_POWER_STATE_TYPE_SHIFT;

	psci_ops.cpu_off(state);
}

static int cpu_psci_cpu_kill(unsigned int cpu)
{
	int err;
	unsigned long start, end;

	if (!psci_ops.affinity_info)
		return 0;
	/*
	 * cpu_kill could race with cpu_die and we can
	 * potentially end up declaring this cpu undead
	 * while it is dying. So, try again a few times.
	 */

	start = jiffies;
	end = start + msecs_to_jiffies(100);
	do {
		err = psci_ops.affinity_info(cpu_logical_map(cpu), 0);
		if (err == PSCI_0_2_AFFINITY_LEVEL_OFF) {
			pr_info("CPU%d killed (polled %d ms)\n", cpu,
				jiffies_to_msecs(jiffies - start));
			return 0;
		}

		usleep_range(100, 1000);
	} while (time_before(jiffies, end));

	pr_warn("CPU%d may not have shut down cleanly (AFFINITY_INFO reports %d)\n",
			cpu, err);
	return -ETIMEDOUT;
}
#endif

const struct cpu_operations cpu_psci_ops = {
	.name		= "psci",
	.cpu_init	= cpu_psci_cpu_init,
	.cpu_prepare	= cpu_psci_cpu_prepare,
	.cpu_boot	= cpu_psci_cpu_boot,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_can_disable = cpu_psci_cpu_can_disable,
	.cpu_disable	= cpu_psci_cpu_disable,
	.cpu_die	= cpu_psci_cpu_die,
	.cpu_kill	= cpu_psci_cpu_kill,
#endif
};

