// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017 Linaro Ltd.
 */
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/devcoredump.h>
#include <linux/list.h>
#include <linux/module.h>

/* 候选 50：禁止运行时挂起（临时稳定化开关）*/
unsigned int joan_no_rpm;
module_param(joan_no_rpm, uint, 0644);
MODULE_PARM_DESC(joan_no_rpm, "joan: forbid venus runtime suspend (0=normal, 1=always on)");

/* 候选 36b：收到 HFI 系统错误后不做恢复（避免实验时把内核卡死） */
unsigned int joan_no_recover;
module_param(joan_no_recover, uint, 0644);
MODULE_PARM_DESC(joan_no_recover, "joan: skip venus sys-error recovery (debug only)");
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ioctl.h>

#include "core.h"
#include "firmware.h"
#include "pm_helpers.h"
#include "hfi_venus_io.h"

#ifndef JOAN_VTRACE
#define JOAN_VTRACE(fmt, ...) do { \
	pr_info("joan-venus-trace: " fmt "\n", ##__VA_ARGS__); \
} while (0)
#endif


static void venus_coredump(struct venus_core *core)
{
	struct device *dev;
	phys_addr_t mem_phys;
	size_t mem_size;
	void *mem_va;
	void *data;

	dev = core->dev;
	mem_phys = core->fw.mem_phys;
	mem_size = core->fw.mem_size;

	mem_va = memremap(mem_phys, mem_size, MEMREMAP_WC);
	if (!mem_va)
		return;

	data = vmalloc(mem_size);
	if (!data) {
		memunmap(mem_va);
		return;
	}

	memcpy(data, mem_va, mem_size);
	memunmap(mem_va);
	dev_coredumpv(dev, data, mem_size, GFP_KERNEL);
}

static void venus_event_notify(struct venus_core *core, u32 event)
{
	struct venus_inst *inst;

	switch (event) {
	case EVT_SYS_WATCHDOG_TIMEOUT:
	case EVT_SYS_ERROR:
		break;
	default:
		return;
	}

	mutex_lock(&core->lock);
	set_bit(0, &core->sys_error);
	set_bit(0, &core->dump_core);
	list_for_each_entry(inst, &core->instances, list)
		inst->ops->event_notify(inst, EVT_SESSION_ERROR, NULL);
	mutex_unlock(&core->lock);

	disable_irq_nosync(core->irq);
	schedule_delayed_work(&core->work, msecs_to_jiffies(10));
}

static const struct hfi_core_ops venus_core_ops = {
	.event_notify = venus_event_notify,
};

#define RPM_WAIT_FOR_IDLE_MAX_ATTEMPTS 10

static void venus_sys_error_handler(struct work_struct *work)
{
	struct venus_core *core =
			container_of(work, struct venus_core, work.work);
	int ret, i, max_attempts = RPM_WAIT_FOR_IDLE_MAX_ATTEMPTS;

	if (joan_no_recover) {
		JOAN_VTRACE("sys_error: joan_no_recover=1, skip recovery");
		core->state = CORE_UNINIT;
		return;
	}
	const char *err_msg = "";
	bool failed = false;

	ret = pm_runtime_get_sync(core->dev);
	if (ret < 0) {
		err_msg = "resume runtime PM";
		max_attempts = 0;
		failed = true;
	}

	core->ops->core_deinit(core);
	core->state = CORE_UNINIT;

	for (i = 0; i < max_attempts; i++) {
		/*
		 * joan 修复：decoder/encoder 视频设备是各自模块 probe 时才注册的，
		 * 只加载 venus-dec（或只加载 venus-enc）时另一个指针为 NULL。
		 * 原代码直接 pm_runtime_active() 会空指针，把整个内核拖死
		 * （实测 oops 在 venus_sys_error_handler+0x74，ldr w1,[x0,#0x19c]）。
		 */
		if ((!core->dev_dec || !pm_runtime_active(core->dev_dec)) &&
		    (!core->dev_enc || !pm_runtime_active(core->dev_enc)))
			break;
		msleep(10);
	}

	mutex_lock(&core->lock);

	venus_shutdown(core);

	if (test_bit(0, &core->dump_core)) {
		venus_coredump(core);
		clear_bit(0, &core->dump_core);
	}

	pm_runtime_put_sync(core->dev);

	for (i = 0; i < max_attempts; i++) {
		if (!core->pmdomains ||
		    !pm_runtime_active(core->pmdomains->pd_devs[0]))
			break;
		usleep_range(1000, 1500);
	}

	hfi_reinit(core);

	ret = pm_runtime_get_sync(core->dev);
	if (ret < 0) {
		err_msg = "resume runtime PM";
		failed = true;
	}

	ret = venus_boot(core);
	if (ret && !failed) {
		err_msg = "boot Venus";
		failed = true;
	}

	ret = hfi_core_resume(core, true);
	if (ret && !failed) {
		err_msg = "resume HFI";
		failed = true;
	}

	enable_irq(core->irq);

	mutex_unlock(&core->lock);

	ret = hfi_core_init(core);
	if (ret && !failed) {
		err_msg = "init HFI";
		failed = true;
	}

	pm_runtime_put_sync(core->dev);

	if (failed) {
		disable_irq_nosync(core->irq);
		dev_warn_ratelimited(core->dev,
				     "System error has occurred, recovery failed to %s\n",
				     err_msg);
		schedule_delayed_work(&core->work, msecs_to_jiffies(10));
		return;
	}

	dev_warn(core->dev, "system error has occurred (recovered)\n");

	mutex_lock(&core->lock);
	clear_bit(0, &core->sys_error);
	wake_up_all(&core->sys_err_done);
	mutex_unlock(&core->lock);
}

static u32 to_v4l2_codec_type(u32 codec)
{
	switch (codec) {
	case HFI_VIDEO_CODEC_H264:
		return V4L2_PIX_FMT_H264;
	case HFI_VIDEO_CODEC_H263:
		return V4L2_PIX_FMT_H263;
	case HFI_VIDEO_CODEC_MPEG1:
		return V4L2_PIX_FMT_MPEG1;
	case HFI_VIDEO_CODEC_MPEG2:
		return V4L2_PIX_FMT_MPEG2;
	case HFI_VIDEO_CODEC_MPEG4:
		return V4L2_PIX_FMT_MPEG4;
	case HFI_VIDEO_CODEC_VC1:
		return V4L2_PIX_FMT_VC1_ANNEX_G;
	case HFI_VIDEO_CODEC_VP8:
		return V4L2_PIX_FMT_VP8;
	case HFI_VIDEO_CODEC_VP9:
		return V4L2_PIX_FMT_VP9;
	case HFI_VIDEO_CODEC_DIVX:
	case HFI_VIDEO_CODEC_DIVX_311:
		return V4L2_PIX_FMT_XVID;
	default:
		return 0;
	}
}

static int venus_enumerate_codecs(struct venus_core *core, u32 type)
{
	const struct hfi_inst_ops dummy_ops = {};
	struct venus_inst *inst;
	u32 codec, codecs;
	unsigned int i;
	int ret;

	if (core->res->hfi_version != HFI_VERSION_1XX)
		return 0;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	mutex_init(&inst->lock);
	inst->core = core;
	inst->session_type = type;
	if (type == VIDC_SESSION_TYPE_DEC)
		codecs = core->dec_codecs;
	else
		codecs = core->enc_codecs;

	ret = hfi_session_create(inst, &dummy_ops);
	if (ret)
		goto err;

	for (i = 0; i < MAX_CODEC_NUM; i++) {
		codec = (1UL << i) & codecs;
		if (!codec)
			continue;

		ret = hfi_session_init(inst, to_v4l2_codec_type(codec));
		if (ret)
			goto done;

		ret = hfi_session_deinit(inst);
		if (ret)
			goto done;
	}

done:
	hfi_session_destroy(inst);
err:
	mutex_destroy(&inst->lock);
	kfree(inst);

	return ret;
}

static void venus_assign_register_offsets(struct venus_core *core)
{
	if (IS_IRIS2(core) || IS_IRIS2_1(core) || IS_AR50_LITE(core)) {
		core->cpu_base = core->base + CPU_BASE_V6;
		core->cpu_cs_base = core->base + CPU_CS_BASE_V6;
		core->cpu_ic_base = core->base + CPU_IC_BASE_V6;
		core->wrapper_base = core->base + WRAPPER_BASE_V6;
		core->wrapper_tz_base = core->base + WRAPPER_TZ_BASE_V6;
		if (IS_AR50_LITE(core)) {
			core->vbif_base = NULL;
			core->aon_base = NULL;
		} else {
			core->vbif_base = core->base + VBIF_BASE;
			core->aon_base = core->base + AON_BASE_V6;
		}
	} else {
		core->vbif_base = core->base + VBIF_BASE;
		core->cpu_base = core->base + CPU_BASE;
		core->cpu_cs_base = core->base + CPU_CS_BASE;
		core->cpu_ic_base = core->base + CPU_IC_BASE;
		core->wrapper_base = core->base + WRAPPER_BASE;
		core->wrapper_tz_base = NULL;
		core->aon_base = NULL;
	}
}

static irqreturn_t venus_isr_thread(int irq, void *dev_id)
{
	struct venus_core *core = dev_id;
	irqreturn_t ret;

	ret = hfi_isr_thread(irq, dev_id);

	if (ret == IRQ_HANDLED && venus_fault_inject_ssr())
		hfi_core_trigger_ssr(core, HFI_TEST_SSR_SW_ERR_FATAL);

	return ret;
}

#if defined(CONFIG_OF_DYNAMIC)
static int venus_add_video_core(struct venus_core *core, const char *node_name,
				const char *compat)
{
	struct of_changeset *ocs = core->ocs;
	struct device *dev = core->dev;
	struct device_node *np, *enp;
	int ret;

	if (!node_name)
		return 0;

	enp = of_find_node_by_name(dev->of_node, node_name);
	if (enp) {
		of_node_put(enp);
		return 0;
	}

	np = of_changeset_create_node(ocs, dev->of_node, node_name);
	if (!np) {
		dev_err(dev, "Unable to create new node\n");
		return -ENODEV;
	}

	ret = of_changeset_add_prop_string(ocs, np, "compatible", compat);
	if (ret)
		dev_err(dev, "unable to add %s\n", compat);

	of_node_put(np);

	return ret;
}

static int venus_add_dynamic_nodes(struct venus_core *core)
{
	struct device *dev = core->dev;
	int ret;

	core->ocs = kmalloc(sizeof(*core->ocs), GFP_KERNEL);
	if (!core->ocs)
		return -ENOMEM;

	of_changeset_init(core->ocs);

	ret = venus_add_video_core(core, core->res->dec_nodename, "venus-decoder");
	if (ret)
		goto err;

	ret = venus_add_video_core(core, core->res->enc_nodename, "venus-encoder");
	if (ret)
		goto err;

	ret = of_changeset_apply(core->ocs);
	if (ret) {
		dev_err(dev, "applying changeset fail ret %d\n", ret);
		goto err;
	}

	return 0;
err:
	of_changeset_destroy(core->ocs);
	kfree(core->ocs);
	core->ocs = NULL;
	return ret;
}

static void venus_remove_dynamic_nodes(struct venus_core *core)
{
	if (core->ocs) {
		of_changeset_revert(core->ocs);
		of_changeset_destroy(core->ocs);
		kfree(core->ocs);
	}
}
#else
static int venus_add_dynamic_nodes(struct venus_core *core)
{
	return 0;
}

static void venus_remove_dynamic_nodes(struct venus_core *core) {}
#endif

/* 只读 MMCC，避免错误清理前隐式访问 Venus/VBIF 寄存器。 */
void joan_venus_mmcc_probe(const char *tag)
{
	struct device_node *np;
	void __iomem *mmcc;

	np = of_find_compatible_node(NULL, NULL, "qcom,mmcc-msm8998");
	if (!np) {
		JOAN_VTRACE("mmcc[%s]: node not found", tag);
	} else {
		mmcc = of_iomap(np, 0);
		of_node_put(np);
		if (!mmcc) {
			JOAN_VTRACE("mmcc[%s]: of_iomap failed", tag);
		} else {
			JOAN_VTRACE("mmcc[%s]: gdscr(0x1024)=%#x core(0x1028)=%#x ahb(0x1030)=%#x axi(0x1034)=%#x maxi(0x1038)=%#x",
				    tag, readl(mmcc + 0x1024), readl(mmcc + 0x1028),
				    readl(mmcc + 0x1030), readl(mmcc + 0x1034),
				    readl(mmcc + 0x1038));
			iounmap(mmcc);
		}
	}
}

static void joan_probe_branch(void __iomem *base, u32 off, const char *name,
			      const char *tag)
{
	u32 value = readl(base + off);

	JOAN_VTRACE("clk[%s]: %-26s off=%#06x value=%#010x read-only",
		    tag, name, off, value);
}

/* 时钟由 CCF 管理；取证不得直接写 CBCR 绕过引用计数。 */
static void joan_venus_clk_probe(const char *tag)
{
	struct device_node *np;
	void __iomem *mmcc, *gcc;

	np = of_find_compatible_node(NULL, NULL, "qcom,mmcc-msm8998");
	if (!np) {
		JOAN_VTRACE("clk[%s]: no mmcc node", tag);
	} else {
		mmcc = of_iomap(np, 0);
		of_node_put(np);
		if (!mmcc) {
			JOAN_VTRACE("clk[%s]: mmcc iomap failed", tag);
		} else {
			joan_probe_branch(mmcc, 0x5024, "mmcc:mnoc_ahb", tag);
			joan_probe_branch(mmcc, 0xf004, "mmcc:mnoc_maxi", tag);
			joan_probe_branch(mmcc, 0xf064, "mmcc:vmem_maxi", tag);
			joan_probe_branch(mmcc, 0xf068, "mmcc:vmem_ahb", tag);
			joan_probe_branch(mmcc, 0x1024, "mmcc:video_top_gdscr", tag);
			iounmap(mmcc);
		}
	}

	np = of_find_compatible_node(NULL, NULL, "qcom,gcc-msm8998");
	if (!np) {
		JOAN_VTRACE("clk[%s]: no gcc node", tag);
		return;
	}
	gcc = of_iomap(np, 0);
	of_node_put(np);
	if (!gcc) {
		JOAN_VTRACE("clk[%s]: gcc iomap failed", tag);
		return;
	}
	/* gcc 侧只读：dtsi 明确警告写 gcc 受保护时钟可能触发复位 */
	joan_probe_branch(gcc, 0x9000, "gcc:mmss_sys_noc_axi", tag);
	joan_probe_branch(gcc, 0x9004, "gcc:mmss_noc_cfg_ahb", tag);
	iounmap(gcc);
}

static void joan_venus_supplier_probe(struct venus_core *core, const char *tag)
{
#ifdef CONFIG_PM
	struct device_node *np;
	struct platform_device *pdev;
	struct device *supplier;
	enum rpm_status status;
	unsigned long flags;
	bool enabled;
	int usage;

	np = of_parse_phandle(core->dev->of_node, "iommus", 0);
	if (!np) {
		pr_info("joan-venus-trace: supplier[%s]: no IOMMU node\n", tag);
		return;
	}
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev) {
		pr_info("joan-venus-trace: supplier[%s]: no IOMMU device\n", tag);
		return;
	}

	/* Read software state without taking a runtime PM usage reference. */
	supplier = &pdev->dev;
	spin_lock_irqsave(&supplier->power.lock, flags);
	status = supplier->power.runtime_status;
	enabled = pm_runtime_enabled(supplier);
	usage = atomic_read(&supplier->power.usage_count);
	spin_unlock_irqrestore(&supplier->power.lock, flags);
	pr_info("joan-venus-trace: supplier[%s]: %s rpm_status=%u enabled=%u usage=%d\n",
		tag, dev_name(supplier), (unsigned int)status, enabled, usage);
	put_device(supplier);
#endif
}

/* 仅验证已知的硬件版本寄存器；不能把全零窗口解释为 ARM9 状态。 */
int joan_venus_hwversion_probe(struct venus_core *core, const char *tag)
{
	u32 version;

	if (!of_device_is_compatible(core->dev->of_node, "qcom,msm8998-venus"))
		return 0;

	JOAN_VTRACE("hwver[%s]: before readl(wrapper+%#x)", tag,
		    WRAPPER_HW_VERSION);
	version = readl(core->wrapper_base + WRAPPER_HW_VERSION);
	JOAN_VTRACE("hwver[%s]: value=%#x", tag, version);

	if ((version & WRAPPER_HW_VERSION_MAJOR_VERSION_MASK) !=
	    (3U << WRAPPER_HW_VERSION_MAJOR_VERSION_SHIFT)) {
		dev_err(core->dev, "invalid Venus 3.x MMIO window at %s (%#x)\n",
			tag, version);
		return -EIO;
	}

	joan_venus_supplier_probe(core, tag);
	return 0;
}

/*
 * 取证辅助：按模块参数给的地址列表逐个读 venus 寄存器块。
 *   joan_probe_list="0x80004,0x80000"   —— 逗号分隔的十六进制偏移
 *   joan_probe_write=1                  —— 每个偏移先读，再把 bit0 置 1 写回再读
 * 设了 joan_probe_list 时，probe 在扫完后直接返回 -ENODEV（只做扫描）。
 * 每条标记独立打印，便于按最后一条 before 定位未返回的读取。
 */
static char *joan_probe_list;
module_param(joan_probe_list, charp, 0644);
MODULE_PARM_DESC(joan_probe_list, "joan: comma-separated hex offsets to read inside venus reg block");

static bool joan_probe_write;
module_param(joan_probe_write, bool, 0644);
MODULE_PARM_DESC(joan_probe_write, "joan: also write bit0 back after reading");

/*
 * joan_probe_poke="0x80004=0x1" —— 逗号分隔的 off=val（都按 16 进制），
 * 在每个值上做一次 **纯写**（posted，不复读），用于验证「必须先写某寄存器
 * 才能让寄存器块响应」这类假设。下游 msm-4.4 venus_boot.c 原文：
 *   "Force the VBIF clk to be on to avoid AXI bridge halt ack failure"
 *   reg = readl_relaxed(reg_base + VIDC_VENUS_VBIF_CLK_ON);  reg |= FORCE_ON;
 */
static char *joan_probe_poke;
module_param(joan_probe_poke, charp, 0644);
MODULE_PARM_DESC(joan_probe_poke, "joan: comma-separated off=val hex pairs, written (no readback) before scan");

/*
 * joan_vbif_forceon=1 —— A/B 开关：probe 里 pm_runtime_get_sync 之后是否立刻
 * 对 VBIF_CLK_ON 写 FORCE_ON。默认关（见调用点注释）。
 */
static bool joan_vbif_forceon;
module_param(joan_vbif_forceon, bool, 0644);
MODULE_PARM_DESC(joan_vbif_forceon, "joan: force VBIF_CLK_ON at probe (A point), default 0");

/* 1：仅上电和 HW_VERSION；2：PAS 后退出；0：继续原有 probe。 */
static unsigned int joan_probe_stop;
module_param(joan_probe_stop, uint, 0444);
MODULE_PARM_DESC(joan_probe_stop, "joan: stop at runtime PM (1) or after PAS (2), default 0");

/*
 * joan_noc_clks=1 —— H9 实验：下游 msm8998 vidc 每个上电周期要开 12 个时钟，
 * mainline 只开 4 个（core/iface/bus/mbus）。VBIF 挂在 MMSS NoC 上，
 * 若 NoC 侧时钟（mnoc_ahb / gcc mmss_sys_noc_axi / bimc_smmu_*）没开，
 * 访问 VBIF 寄存器窗就会挂死互连。
 *
 * 这里用 clk_get(NULL, "<全局时钟名>") 直接取来开，**不动 DT/驱动资源表**，
 * 目的是用最小改动做单变量验证。
 */
static bool joan_noc_clks;
module_param(joan_noc_clks, bool, 0644);
MODULE_PARM_DESC(joan_noc_clks, "joan: enable the extra MMSS NoC/SMMU clocks mainline venus lacks");

static const char * const joan_noc_clk_names[] = {
	"gcc_mmss_sys_noc_axi_clk",
	"gcc_mmss_noc_cfg_ahb_clk",
	"mnoc_ahb_clk",
	"mnoc_maxi_clk",
	"bimc_smmu_ahb_clk",
	"bimc_smmu_axi_clk",
	"vmem_ahb_clk",
	"vmem_maxi_clk",
};

static void joan_noc_clk_enable(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(joan_noc_clk_names); i++) {
		const char *n = joan_noc_clk_names[i];
		struct clk *c = clk_get(NULL, n);
		int ret;

		if (IS_ERR(c)) {
			JOAN_VTRACE("noc_clk: %s clk_get err %ld", n, PTR_ERR(c));
			continue;
		}
		ret = clk_prepare_enable(c);
		JOAN_VTRACE("noc_clk: %s rate=%lu prepare_enable ret=%d",
			    n, clk_get_rate(c), ret);
		/* 故意不 clk_put：诊断用，保持住引用直到下次重启 */
	}
}

static void joan_venus_addr_poke(struct venus_core *core)
{
	char *buf, *p, *tok;

	if (!joan_probe_poke)
		return;

	buf = kstrdup(joan_probe_poke, GFP_KERNEL);
	if (!buf)
		return;

	p = buf;
	while ((tok = strsep(&p, ",")) != NULL) {
		char *eq;
		unsigned int off, val;

		if (!*tok)
			continue;
		eq = strchr(tok, '=');
		if (!eq)
			continue;
		*eq++ = '\0';
		if (kstrtouint(tok, 16, &off) || kstrtouint(eq, 16, &val)) {
			JOAN_VTRACE("poke: bad token");
			continue;
		}

		JOAN_VTRACE("poke: before writel(base+%#08x, %#010x) [posted]", off, val);
		writel_relaxed(val, core->base + off);
		JOAN_VTRACE("poke: writel done base+%#08x (no readback)", off);
	}

	kfree(buf);
}

static void joan_venus_addr_scan(struct venus_core *core)
{
	char *buf, *p, *tok;

	if (!joan_probe_list)
		return;

	buf = kstrdup(joan_probe_list, GFP_KERNEL);
	if (!buf)
		return;

	JOAN_VTRACE("scan: list=%s write=%d", joan_probe_list, joan_probe_write);

	p = buf;
	while ((tok = strsep(&p, ",")) != NULL) {
		unsigned int off;
		u32 before, after;

		if (!*tok)
			continue;
		if (kstrtouint(tok, 16, &off)) {
			JOAN_VTRACE("scan: bad token '%s'", tok);
			continue;
		}

		JOAN_VTRACE("scan: before readl(base+%#08x)", off);
		before = readl(core->base + off);
		JOAN_VTRACE("scan: readl(base+%#08x)=%#010x", off, before);

		if (joan_probe_write) {
			writel(before | BIT(0), core->base + off);
			udelay(50);
			after = readl(core->base + off);
			JOAN_VTRACE("scan: write(base+%#08x, %#010x) -> %#010x",
				    off, before | BIT(0), after);
		}
	}

	kfree(buf);
}

static int venus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct venus_core *core;
	bool runtime_resumed = false;
	bool irq_requested = false;
	int ret, pm_ret = 0;

	if (joan_probe_stop > 2)
		return -EINVAL;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;

	core->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(core->base))
		return PTR_ERR(core->base);

	/*
	 * ★ v2 修正（2026-10-10）：joan_venus_addr_scan() 原先放在这里（上电之前），
	 * 实测证明「venus 未被 pm_runtime 上电时，任何寄存器访问都会挂死总线」
	 * ——连已知可读的 wrapper 0xe0000 也不例外，所以那时扫出来的「不可访问」
	 * 全是假象。扫描改到 pm_runtime_get_sync() 之后（见下面 powered-scan 段）。
	 */

	joan_venus_clk_probe("pre-probe");

	core->video_path = devm_of_icc_get(dev, "video-mem");
	if (IS_ERR(core->video_path))
		return PTR_ERR(core->video_path);

	core->cpucfg_path = devm_of_icc_get(dev, "cpu-cfg");
	if (IS_ERR(core->cpucfg_path))
		return PTR_ERR(core->cpucfg_path);

	core->irq = platform_get_irq(pdev, 0);
	if (core->irq < 0)
		return core->irq;

	core->res = of_device_get_match_data(dev);
	if (!core->res)
		return -ENODEV;

	mutex_init(&core->pm_lock);

	core->pm_ops = venus_pm_get(core->res->hfi_version);
	if (!core->pm_ops)
		return -ENODEV;

	if (core->pm_ops->core_get) {
		ret = core->pm_ops->core_get(core);
		if (ret)
			return ret;
	}

	ret = dma_set_mask_and_coherent(dev, core->res->dma_mask);
	if (ret)
		goto err_core_put;

	dma_set_max_seg_size(dev, UINT_MAX);

	INIT_LIST_HEAD(&core->instances);
	mutex_init(&core->lock);
	INIT_DELAYED_WORK(&core->work, venus_sys_error_handler);
	init_waitqueue_head(&core->sys_err_done);

	ret = hfi_create(core, &venus_core_ops);
	if (ret)
		goto err_core_put;

	ret = devm_request_threaded_irq(dev, core->irq, hfi_isr, venus_isr_thread,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"venus", core);
	if (ret)
		goto err_hfi_destroy;
	irq_requested = true;

	venus_assign_register_offsets(core);

	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret)
		goto err_hfi_destroy;

	platform_set_drvdata(pdev, core);

	pm_runtime_enable(dev);

	JOAN_VTRACE("probe: before pm_runtime_get_sync");
	ret = pm_runtime_get_sync(dev);
	JOAN_VTRACE("probe: pm_runtime_get_sync ret=%d", ret);
	if (ret < 0)
		goto err_runtime_disable;
	runtime_resumed = true;

	joan_venus_mmcc_probe("A-after-runtime-pm");
	ret = joan_venus_hwversion_probe(core, "A-after-runtime-pm");
	if (ret)
		goto err_runtime_disable;
	if (joan_probe_stop == 1) {
		JOAN_VTRACE("probe: diagnostic stop=1 before firmware/TZ");
		ret = -ECANCELED;
		goto err_runtime_disable;
	}

	/*
	 * ★ 2026-10-10 A/B 开关：上电后（pm_runtime_get_sync 之后）是否立刻对
	 * VBIF_CLK_ON 做一次 force-on。
	 *
	 * 默认**关闭**：实测发现「在 A 点写 base+0x80004」之后，紧接着的
	 * VBIF 首次读（scan 的 readl(base+0x80000)）会挂死总线；
	 * 而把这次写交给 joan_probe_poke 去做（同样地址、同样值）时读是通的
	 * （§16.13.C 可复现）。⇒ 用 joan_vbif_forceon=1 单独打开做对照，
	 * 确认「A 点写」本身是否就是问题。
	 */
	JOAN_VTRACE("probe: vbif_forceon=%d noc_clks=%d vbif_base=%px base=%px",
		    joan_vbif_forceon, joan_noc_clks, core->vbif_base, core->base);
	if (joan_noc_clks)
		joan_noc_clk_enable();
	if (joan_vbif_forceon && !IS_V6(core) && core->vbif_base) {
		JOAN_VTRACE("vbif_clk_on[A]: force-on -> %px+%#x",
			    core->vbif_base, VBIF_CLK_ON);
		writel(VBIF_CLK_ON_FORCE_ON, core->vbif_base + VBIF_CLK_ON);
		JOAN_VTRACE("vbif_clk_on[A]: posted write returned");
	}

	/*
	 * ★ powered-scan：此时 GDSC + 4 路 video 时钟已开，扫描结果才有效。
	 * 设了 joan_probe_list 时扫完就退出（走正常错误回收路径）。
	 * 先跑 joan_probe_poke（纯写，用于验证「必须先写才响应」的假设）。
	 */
	joan_venus_addr_poke(core);
	joan_venus_addr_scan(core);
	if (joan_probe_list) {
		ret = -ENODEV;
		goto err_runtime_disable;
	}

	JOAN_VTRACE("probe: before venus_firmware_init");
	ret = venus_firmware_init(core);
	JOAN_VTRACE("probe: venus_firmware_init ret=%d use_tz=%u", ret, core->use_tz);
	if (ret)
		goto err_runtime_disable;

	JOAN_VTRACE("probe: before venus_boot");
	ret = venus_boot(core);
	JOAN_VTRACE("probe: venus_boot ret=%d", ret);
	if (ret)
		goto err_firmware_deinit;

	ret = joan_venus_hwversion_probe(core, "B-after-PAS");
	if (ret)
		goto err_venus_shutdown;
	if (joan_probe_stop == 2) {
		JOAN_VTRACE("probe: diagnostic stop=2 before remote state/HFI");
		ret = -ECANCELED;
		goto err_venus_shutdown;
	}

	JOAN_VTRACE("probe: before venus_firmware_cfg");
	ret = venus_firmware_cfg(core);
	JOAN_VTRACE("probe: venus_firmware_cfg ret=%d", ret);
	if (ret)
		goto err_venus_shutdown;

	JOAN_VTRACE("probe: before hfi_core_resume");
	ret = hfi_core_resume(core, true);
	JOAN_VTRACE("probe: hfi_core_resume ret=%d", ret);
	if (ret)
		goto err_venus_shutdown;

	JOAN_VTRACE("probe: before hfi_core_init");
	ret = hfi_core_init(core);
	JOAN_VTRACE("probe: hfi_core_init ret=%d", ret);
	if (ret)
		goto err_venus_shutdown;

	ret = venus_firmware_check(core);
	if (ret)
		goto err_core_deinit;

	if (core->res->dec_nodename || core->res->enc_nodename) {
		ret = venus_add_dynamic_nodes(core);
		if (ret)
			goto err_core_deinit;
	}

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret)
		goto err_remove_dynamic_nodes;

	ret = venus_enumerate_codecs(core, VIDC_SESSION_TYPE_DEC);
	if (ret)
		goto err_of_depopulate;

	ret = venus_enumerate_codecs(core, VIDC_SESSION_TYPE_ENC);
	if (ret)
		goto err_of_depopulate;

	/*
	 * 候选 50（临时稳定化开关）：msm8998 上「空闲掉电 -> 恢复」路径仍然会触发
	 * 固件 System error（power_on/boot_core 都返回 0，随后报错并无法恢复）。
	 * joan_no_rpm=1 时禁止运行时挂起，核心保持上电，换取稳定；
	 * 默认 0，等恢复路径修好后再打开。
	 */
	if (joan_no_rpm) {
		pm_runtime_forbid(dev);
		JOAN_VTRACE("probe: runtime PM forbidden (joan_no_rpm=1)");
	}
	ret = pm_runtime_put_sync(dev);
	if (ret) {
		pm_runtime_get_noresume(dev);
		goto err_of_depopulate;
	}

	venus_dbgfs_init(core);

	return 0;

err_of_depopulate:
	of_platform_depopulate(dev);
err_remove_dynamic_nodes:
	venus_remove_dynamic_nodes(core);
err_core_deinit:
	hfi_core_deinit(core, false);
err_venus_shutdown:
	/* 先移除 IRQ，防止固件关闭或掉电时 handler 继续访问 MMIO。 */
	devm_free_irq(dev, core->irq, core);
	irq_requested = false;
	JOAN_VTRACE("probe: before error venus_shutdown");
	pm_ret = venus_shutdown(core);
	JOAN_VTRACE("probe: error venus_shutdown ret=%d", pm_ret);
err_firmware_deinit:
	venus_firmware_deinit(core);
err_runtime_disable:
	if (irq_requested) {
		devm_free_irq(dev, core->irq, core);
		irq_requested = false;
	}
	if (runtime_resumed) {
		/* CORE_UNINIT 下不会等待 HFI；正常关闭 clocks，再交给 genpd。 */
		JOAN_VTRACE("probe: before error runtime suspend");
		pm_ret = pm_runtime_put_sync_suspend(dev);
		JOAN_VTRACE("probe: error runtime suspend ret=%d active=%d",
			    pm_ret, pm_runtime_active(dev));
	} else {
		pm_runtime_put_noidle(dev);
	}
	pm_runtime_disable(dev);
	if (!runtime_resumed || pm_ret >= 0)
		pm_runtime_set_suspended(dev);
	v4l2_device_unregister(&core->v4l2_dev);
err_hfi_destroy:
	if (irq_requested)
		devm_free_irq(dev, core->irq, core);
	hfi_destroy(core);
err_core_put:
	if (core->pm_ops->core_put)
		core->pm_ops->core_put(core);
	return ret;
}

static void venus_remove(struct platform_device *pdev)
{
	struct venus_core *core = platform_get_drvdata(pdev);
	const struct venus_pm_ops *pm_ops = core->pm_ops;
	struct device *dev = core->dev;
	int ret;

	cancel_delayed_work_sync(&core->work);
	ret = pm_runtime_get_sync(dev);
	WARN_ON(ret < 0);

	ret = hfi_core_deinit(core, true);
	WARN_ON(ret);

	venus_shutdown(core);
	of_platform_depopulate(dev);

	venus_firmware_deinit(core);

	venus_remove_dynamic_nodes(core);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	if (pm_ops->core_put)
		pm_ops->core_put(core);

	v4l2_device_unregister(&core->v4l2_dev);

	hfi_destroy(core);

	mutex_destroy(&core->pm_lock);
	mutex_destroy(&core->lock);
	venus_dbgfs_deinit(core);
}

static void venus_core_shutdown(struct platform_device *pdev)
{
	struct venus_core *core = platform_get_drvdata(pdev);

	pm_runtime_get_sync(core->dev);
	venus_shutdown(core);
	venus_firmware_deinit(core);
	pm_runtime_put_sync(core->dev);
}

static __maybe_unused int venus_runtime_suspend(struct device *dev)
{
	struct venus_core *core = dev_get_drvdata(dev);
	const struct venus_pm_ops *pm_ops = core->pm_ops;
	int ret;

	ret = hfi_core_suspend(core);
	if (ret)
		return ret;

	if (pm_ops->core_power) {
		ret = pm_ops->core_power(core, POWER_OFF);
		if (ret)
			return ret;
	}

	ret = icc_set_bw(core->cpucfg_path, 0, 0);
	if (ret)
		goto err_cpucfg_path;

	ret = icc_set_bw(core->video_path, 0, 0);
	if (ret)
		goto err_video_path;

	return ret;

err_video_path:
	icc_set_bw(core->cpucfg_path, kbps_to_icc(1000), 0);
err_cpucfg_path:
	if (pm_ops->core_power)
		pm_ops->core_power(core, POWER_ON);

	return ret;
}

void venus_close_common(struct venus_inst *inst, struct file *filp)
{
	/*
	 * Make sure we don't have IRQ/IRQ-thread currently running
	 * or pending execution, which would race with the inst destruction.
	 */
	synchronize_irq(inst->core->irq);

	v4l2_m2m_ctx_release(inst->m2m_ctx);
	v4l2_m2m_release(inst->m2m_dev);
	hfi_session_destroy(inst);
	v4l2_fh_del(&inst->fh, filp);
	v4l2_fh_exit(&inst->fh);
	v4l2_ctrl_handler_free(&inst->ctrl_handler);

	mutex_destroy(&inst->lock);
	mutex_destroy(&inst->ctx_q_lock);
}
EXPORT_SYMBOL_GPL(venus_close_common);

static __maybe_unused int venus_runtime_resume(struct device *dev)
{
	struct venus_core *core = dev_get_drvdata(dev);
	const struct venus_pm_ops *pm_ops = core->pm_ops;
	int ret;

	ret = icc_set_bw(core->video_path, kbps_to_icc(20000), 0);
	if (ret)
		return ret;

	ret = icc_set_bw(core->cpucfg_path, kbps_to_icc(1000), 0);
	if (ret)
		return ret;

	if (pm_ops->core_power) {
		ret = pm_ops->core_power(core, POWER_ON);
		if (ret)
			return ret;
	}

	return hfi_core_resume(core, false);
}

static const struct dev_pm_ops venus_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(venus_runtime_suspend, venus_runtime_resume, NULL)
};

static const struct freq_tbl msm8916_freq_table[] = {
	{ 352800, 228570000 },	/* 1920x1088 @ 30 + 1280x720 @ 30 */
	{ 244800, 160000000 },	/* 1920x1088 @ 30 */
	{ 108000, 100000000 },	/* 1280x720 @ 30 */
};

static const struct reg_val msm8916_reg_preset[] = {
	{ 0xe0020, 0x05555556 },
	{ 0xe0024, 0x05555556 },
	{ 0x80124, 0x00000003 },
};

static const struct venus_resources msm8916_res = {
	.freq_tbl = msm8916_freq_table,
	.freq_tbl_size = ARRAY_SIZE(msm8916_freq_table),
	.reg_tbl = msm8916_reg_preset,
	.reg_tbl_size = ARRAY_SIZE(msm8916_reg_preset),
	.clks = { "core", "iface", "bus", },
	.clks_num = 3,
	.max_load = 352800, /* 720p@30 + 1080p@30 */
	.hfi_version = HFI_VERSION_1XX,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xddc00000 - 1,
	.fwname = "qcom/venus-1.8/venus.mbn",
	.dec_nodename = "video-decoder",
	.enc_nodename = "video-encoder",
};

static const struct freq_tbl msm8996_freq_table[] = {
	{ 1944000, 520000000 },	/* 4k UHD @ 60 (decode only) */
	{  972000, 520000000 },	/* 4k UHD @ 30 */
	{  489600, 346666667 },	/* 1080p @ 60 */
	{  244800, 150000000 },	/* 1080p @ 30 */
	{  108000,  75000000 },	/* 720p @ 30 */
};

static const struct reg_val msm8996_reg_preset[] = {
	{ 0x80010, 0xffffffff },
	{ 0x80018, 0x00001556 },
	{ 0x8001C, 0x00001556 },
};

static const struct venus_resources msm8996_res = {
	.freq_tbl = msm8996_freq_table,
	.freq_tbl_size = ARRAY_SIZE(msm8996_freq_table),
	.reg_tbl = msm8996_reg_preset,
	.reg_tbl_size = ARRAY_SIZE(msm8996_reg_preset),
	.clks = {"core", "iface", "bus", "mbus" },
	.clks_num = 4,
	.vcodec0_clks = { "core" },
	.vcodec1_clks = { "core" },
	.vcodec_clks_num = 1,
	.max_load = 2563200,
	.hfi_version = HFI_VERSION_3XX,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xddc00000 - 1,
	.fwname = "qcom/venus-4.2/venus.mbn",
};

static const struct freq_tbl msm8998_freq_table[] = {
	{ 1728000, 533000000 },	/* 4k UHD @ 60 (decode only) */
	{ 1036800, 444000000 },	/* 2k @ 120 */
	{  829440, 355200000 },	/* 4k @ 44 */
	{  489600, 269330000 },/* 4k @ 30 */
	{  108000, 200000000 },	/* 1080p @ 60 */
};

static const struct reg_val msm8998_reg_preset[] = {
	{ 0x80124, 0x00000003 },
	{ 0x80550, 0x01111111 },
	{ 0x80560, 0x01111111 },
	{ 0x80568, 0x01111111 },
	{ 0x80570, 0x01111111 },
	{ 0x80580, 0x01111111 },
	{ 0x80588, 0x01111111 },
	{ 0xe2010, 0x00000000 },
};

static const struct venus_resources msm8998_res = {
	.freq_tbl = msm8998_freq_table,
	.freq_tbl_size = ARRAY_SIZE(msm8998_freq_table),
	.reg_tbl = msm8998_reg_preset,
	.reg_tbl_size = ARRAY_SIZE(msm8998_reg_preset),
	.clks = { "core", "iface", "bus", "mbus" },
	.clks_num = 4,
	.vcodec0_clks = { "core" },
	.vcodec1_clks = { "core" },
	.vcodec_clks_num = 1,
	.max_load = 2563200,
	.hfi_version = HFI_VERSION_3XX,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xddc00000 - 1,
	/* MSM8998 downstream venus_ns / venus_sec_non_pixel IOVA pools. */
	.cp_start = 0,
	.cp_size = 0x70800000,
	.cp_nonpixel_start = 0x1000000,
	.cp_nonpixel_size = 0x24800000,
	.fwname = "qcom/venus-4.4/venus.mbn",
};

static const struct freq_tbl sdm660_freq_table[] = {
	{ 979200, 518400000 },
	{ 489600, 441600000 },
	{ 432000, 404000000 },
	{ 244800, 320000000 },
	{ 216000, 269330000 },
	{ 108000, 133330000 },
};

static const struct reg_val sdm660_reg_preset[] = {
	{ 0x80010, 0x001f001f },
	{ 0x80018, 0x00000156 },
	{ 0x8001c, 0x00000156 },
};

static const struct bw_tbl sdm660_bw_table_enc[] = {
	{  979200,  1044000, 0, 2446336, 0 },	/* 4k UHD @ 30 */
	{  864000,   887000, 0, 2108416, 0 },	/* 720p @ 240 */
	{  489600,   666000, 0, 1207296, 0 },	/* 1080p @ 60 */
	{  432000,   578000, 0, 1058816, 0 },	/* 720p @ 120 */
	{  244800,   346000, 0,  616448, 0 },	/* 1080p @ 30 */
	{  216000,   293000, 0,  534528, 0 },	/* 720p @ 60 */
	{  108000,   151000, 0,  271360, 0 },	/* 720p @ 30 */
};

static const struct bw_tbl sdm660_bw_table_dec[] = {
	{  979200,  2365000, 0, 1892000, 0 },	/* 4k UHD @ 30 */
	{  864000,  1978000, 0, 1554000, 0 },	/* 720p @ 240 */
	{  489600,  1133000, 0,  895000, 0 },	/* 1080p @ 60 */
	{  432000,   994000, 0,  781000, 0 },	/* 720p @ 120 */
	{  244800,   580000, 0,  460000, 0 },	/* 1080p @ 30 */
	{  216000,   501000, 0,  301000, 0 },	/* 720p @ 60 */
	{  108000,   255000, 0,  202000, 0 },	/* 720p @ 30 */
};

static const struct venus_resources sdm660_res = {
	.freq_tbl = sdm660_freq_table,
	.freq_tbl_size = ARRAY_SIZE(sdm660_freq_table),
	.reg_tbl = sdm660_reg_preset,
	.reg_tbl_size = ARRAY_SIZE(sdm660_reg_preset),
	.bw_tbl_enc = sdm660_bw_table_enc,
	.bw_tbl_enc_size = ARRAY_SIZE(sdm660_bw_table_enc),
	.bw_tbl_dec = sdm660_bw_table_dec,
	.bw_tbl_dec_size = ARRAY_SIZE(sdm660_bw_table_dec),
	.clks = {"core", "iface", "bus", "bus_throttle" },
	.clks_num = 4,
	.vcodec0_clks = { "vcodec0_core" },
	.vcodec1_clks = { "vcodec0_core" },
	.vcodec_clks_num = 1,
	.vcodec_num = 1,
	.max_load = 1036800,
	.hfi_version = HFI_VERSION_3XX,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.cp_start = 0,
	.cp_size = 0x79000000,
	.cp_nonpixel_start = 0x1000000,
	.cp_nonpixel_size = 0x28000000,
	.dma_mask = 0xd9000000 - 1,
	.fwname = "qcom/venus-4.4/venus.mdt",
};

static const struct freq_tbl sdm845_freq_table[] = {
	{ 3110400, 533000000 },	/* 4096x2160@90 */
	{ 2073600, 444000000 },	/* 4096x2160@60 */
	{ 1944000, 404000000 },	/* 3840x2160@60 */
	{  972000, 330000000 },	/* 3840x2160@30 */
	{  489600, 200000000 },	/* 1920x1080@60 */
	{  244800, 100000000 },	/* 1920x1080@30 */
};

static const struct bw_tbl sdm845_bw_table_enc[] = {
	{ 1944000, 1612000, 0, 2416000, 0 },	/* 3840x2160@60 */
	{  972000,  951000, 0, 1434000, 0 },	/* 3840x2160@30 */
	{  489600,  723000, 0,  973000, 0 },	/* 1920x1080@60 */
	{  244800,  370000, 0,	495000, 0 },	/* 1920x1080@30 */
};

static const struct bw_tbl sdm845_bw_table_dec[] = {
	{ 2073600, 3929000, 0, 5551000, 0 },	/* 4096x2160@60 */
	{ 1036800, 1987000, 0, 2797000, 0 },	/* 4096x2160@30 */
	{  489600, 1040000, 0, 1298000, 0 },	/* 1920x1080@60 */
	{  244800,  530000, 0,  659000, 0 },	/* 1920x1080@30 */
};

static const struct venus_resources sdm845_res = {
	.freq_tbl = sdm845_freq_table,
	.freq_tbl_size = ARRAY_SIZE(sdm845_freq_table),
	.bw_tbl_enc = sdm845_bw_table_enc,
	.bw_tbl_enc_size = ARRAY_SIZE(sdm845_bw_table_enc),
	.bw_tbl_dec = sdm845_bw_table_dec,
	.bw_tbl_dec_size = ARRAY_SIZE(sdm845_bw_table_dec),
	.clks = {"core", "iface", "bus" },
	.clks_num = 3,
	.vcodec0_clks = { "core", "bus" },
	.vcodec1_clks = { "core", "bus" },
	.vcodec_clks_num = 2,
	.max_load = 3110400,	/* 4096x2160@90 */
	.hfi_version = HFI_VERSION_4XX,
	.vpu_version = VPU_VERSION_AR50,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xe0000000 - 1,
	.fwname = "qcom/venus-5.2/venus.mbn",
};

static const struct venus_resources sdm845_res_v2 = {
	.freq_tbl = sdm845_freq_table,
	.freq_tbl_size = ARRAY_SIZE(sdm845_freq_table),
	.bw_tbl_enc = sdm845_bw_table_enc,
	.bw_tbl_enc_size = ARRAY_SIZE(sdm845_bw_table_enc),
	.bw_tbl_dec = sdm845_bw_table_dec,
	.bw_tbl_dec_size = ARRAY_SIZE(sdm845_bw_table_dec),
	.clks = {"core", "iface", "bus" },
	.clks_num = 3,
	.vcodec0_clks = { "vcodec0_core", "vcodec0_bus" },
	.vcodec1_clks = { "vcodec1_core", "vcodec1_bus" },
	.vcodec_clks_num = 2,
	.vcodec_pmdomains = (const char *[]) { "venus", "vcodec0", "vcodec1" },
	.vcodec_pmdomains_num = 3,
	.opp_pmdomain = (const char *[]) { "cx" },
	.opp_pmdomain_num = 1,
	.vcodec_num = 2,
	.max_load = 3110400,	/* 4096x2160@90 */
	.hfi_version = HFI_VERSION_4XX,
	.vpu_version = VPU_VERSION_AR50,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xe0000000 - 1,
	.cp_start = 0,
	.cp_size = 0x70800000,
	.cp_nonpixel_start = 0x1000000,
	.cp_nonpixel_size = 0x24800000,
	.fwname = "qcom/venus-5.2/venus.mbn",
	.dec_nodename = "video-core0",
	.enc_nodename = "video-core1",
};

static const struct freq_tbl sc7180_freq_table[] = {
	{  0, 500000000 },
	{  0, 434000000 },
	{  0, 340000000 },
	{  0, 270000000 },
	{  0, 150000000 },
};

static const struct bw_tbl sc7180_bw_table_enc[] = {
	{  972000,  750000, 0, 0, 0 },	/* 3840x2160@30 */
	{  489600,  451000, 0, 0, 0 },	/* 1920x1080@60 */
	{  244800,  234000, 0, 0, 0 },	/* 1920x1080@30 */
};

static const struct bw_tbl sc7180_bw_table_dec[] = {
	{ 1036800, 1386000, 0, 1875000, 0 },	/* 4096x2160@30 */
	{  489600,  865000, 0, 1146000, 0 },	/* 1920x1080@60 */
	{  244800,  530000, 0,  583000, 0 },	/* 1920x1080@30 */
};

static const struct venus_resources sc7180_res = {
	.freq_tbl = sc7180_freq_table,
	.freq_tbl_size = ARRAY_SIZE(sc7180_freq_table),
	.bw_tbl_enc = sc7180_bw_table_enc,
	.bw_tbl_enc_size = ARRAY_SIZE(sc7180_bw_table_enc),
	.bw_tbl_dec = sc7180_bw_table_dec,
	.bw_tbl_dec_size = ARRAY_SIZE(sc7180_bw_table_dec),
	.clks = {"core", "iface", "bus" },
	.clks_num = 3,
	.vcodec0_clks = { "vcodec0_core", "vcodec0_bus" },
	.vcodec_clks_num = 2,
	.vcodec_pmdomains = (const char *[]) { "venus", "vcodec0" },
	.vcodec_pmdomains_num = 2,
	.opp_pmdomain = (const char *[]) { "cx" },
	.opp_pmdomain_num = 1,
	.vcodec_num = 1,
	.hfi_version = HFI_VERSION_4XX,
	.vpu_version = VPU_VERSION_AR50,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xe0000000 - 1,
	.cp_start = 0,
	.cp_size = 0x70800000,
	.cp_nonpixel_start = 0x1000000,
	.cp_nonpixel_size = 0x24800000,
	.fwname = "qcom/venus-5.4/venus.mbn",
	.dec_nodename = "video-decoder",
	.enc_nodename = "video-encoder",
};

static const struct freq_tbl sm8250_freq_table[] = {
	{ 0, 444000000 },
	{ 0, 366000000 },
	{ 0, 338000000 },
	{ 0, 240000000 },
};

static const struct bw_tbl sm8250_bw_table_enc[] = {
	{ 1944000, 1954000, 0, 3711000, 0 },	/* 3840x2160@60 */
	{  972000,  996000, 0, 1905000, 0 },	/* 3840x2160@30 */
	{  489600,  645000, 0,  977000, 0 },	/* 1920x1080@60 */
	{  244800,  332000, 0,	498000, 0 },	/* 1920x1080@30 */
};

static const struct bw_tbl sm8250_bw_table_dec[] = {
	{ 2073600, 2403000, 0, 4113000, 0 },	/* 4096x2160@60 */
	{ 1036800, 1224000, 0, 2079000, 0 },	/* 4096x2160@30 */
	{  489600,  812000, 0,  998000, 0 },	/* 1920x1080@60 */
	{  244800,  416000, 0,  509000, 0 },	/* 1920x1080@30 */
};

static const struct reg_val sm8250_reg_preset[] = {
	{ 0xb0088, 0 },
};

static const struct venus_resources sm8250_res = {
	.freq_tbl = sm8250_freq_table,
	.freq_tbl_size = ARRAY_SIZE(sm8250_freq_table),
	.reg_tbl = sm8250_reg_preset,
	.reg_tbl_size = ARRAY_SIZE(sm8250_reg_preset),
	.bw_tbl_enc = sm8250_bw_table_enc,
	.bw_tbl_enc_size = ARRAY_SIZE(sm8250_bw_table_enc),
	.bw_tbl_dec = sm8250_bw_table_dec,
	.bw_tbl_dec_size = ARRAY_SIZE(sm8250_bw_table_dec),
	.clks = {"core", "iface"},
	.clks_num = 2,
	.resets = { "bus", "core" },
	.resets_num = 2,
	.vcodec0_clks = { "vcodec0_core" },
	.vcodec_clks_num = 1,
	.vcodec_pmdomains = (const char *[]) { "venus", "vcodec0" },
	.vcodec_pmdomains_num = 2,
	.opp_pmdomain = (const char *[]) { "mx", "mmcx" },
	.opp_pmdomain_num = 2,
	.vcodec_num = 1,
	.max_load = 7833600,
	.hfi_version = HFI_VERSION_6XX,
	.vpu_version = VPU_VERSION_IRIS2,
	.num_vpp_pipes = 4,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xe0000000 - 1,
	.fwname = "qcom/vpu-1.0/venus.mbn",
	.dec_nodename = "video-decoder",
	.enc_nodename = "video-encoder",
};

static const struct freq_tbl sc7280_freq_table[] = {
	{ 0, 460000000 },
	{ 0, 424000000 },
	{ 0, 335000000 },
	{ 0, 240000000 },
	{ 0, 133333333 },
};

static const struct bw_tbl sc7280_bw_table_enc[] = {
	{ 1944000, 1896000, 0, 3657000, 0 },	/* 3840x2160@60 */
	{  972000,  968000, 0, 1848000, 0 },	/* 3840x2160@30 */
	{  489600,  618000, 0,  941000, 0 },	/* 1920x1080@60 */
	{  244800,  318000, 0,	480000, 0 },	/* 1920x1080@30 */
};

static const struct bw_tbl sc7280_bw_table_dec[] = {
	{ 2073600, 2128000, 0, 3831000, 0 },	/* 4096x2160@60 */
	{ 1036800, 1085000, 0, 1937000, 0 },	/* 4096x2160@30 */
	{  489600,  779000, 0,  998000, 0 },	/* 1920x1080@60 */
	{  244800,  400000, 0,  509000, 0 },	/* 1920x1080@30 */
};

static const struct reg_val sm7280_reg_preset[] = {
	{ 0xb0088, 0 },
};

static const struct hfi_ubwc_config sc7280_ubwc_config = {
	0, 0, {1, 1, 1, 0, 0, 0}, 8, 32, 14, 0, 0, {0, 0}
};

static const struct venus_resources sc7280_res = {
	.freq_tbl = sc7280_freq_table,
	.freq_tbl_size = ARRAY_SIZE(sc7280_freq_table),
	.reg_tbl = sm7280_reg_preset,
	.reg_tbl_size = ARRAY_SIZE(sm7280_reg_preset),
	.bw_tbl_enc = sc7280_bw_table_enc,
	.bw_tbl_enc_size = ARRAY_SIZE(sc7280_bw_table_enc),
	.bw_tbl_dec = sc7280_bw_table_dec,
	.bw_tbl_dec_size = ARRAY_SIZE(sc7280_bw_table_dec),
	.ubwc_conf = &sc7280_ubwc_config,
	.clks = {"core", "bus", "iface"},
	.clks_num = 3,
	.vcodec0_clks = {"vcodec_core", "vcodec_bus"},
	.vcodec_clks_num = 2,
	.vcodec_pmdomains = (const char *[]) { "venus", "vcodec0" },
	.vcodec_pmdomains_num = 2,
	.opp_pmdomain = (const char *[]) { "cx" },
	.opp_pmdomain_num = 1,
	.vcodec_num = 1,
	.hfi_version = HFI_VERSION_6XX,
	.vpu_version = VPU_VERSION_IRIS2_1,
	.num_vpp_pipes = 1,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xe0000000 - 1,
	.cp_start = 0,
	.cp_size = 0x25800000,
	.cp_nonpixel_start = 0x1000000,
	.cp_nonpixel_size = 0x24800000,
	.fwname = "qcom/vpu-2.0/venus.mbn",
	.dec_nodename = "video-decoder",
	.enc_nodename = "video-encoder",
};

static const struct bw_tbl qcm2290_bw_table_dec[] = {
	{ 352800, 597000, 0, 746000, 0 }, /* 1080p@30 + 720p@30 */
	{ 244800, 413000, 0, 516000, 0 }, /* 1080p@30 */
	{ 216000, 364000, 0, 454000, 0 }, /* 720p@60  */
	{ 108000, 182000, 0, 227000, 0 }, /* 720p@30  */
};

static const struct bw_tbl qcm2290_bw_table_enc[] = {
	{ 352800, 396000, 0, 0, 0 }, /* 1080p@30 + 720p@30 */
	{ 244800, 275000, 0, 0, 0 }, /* 1080p@30 */
	{ 216000, 242000, 0, 0, 0 }, /* 720p@60  */
	{ 108000, 121000, 0, 0, 0 }, /* 720p@30  */
};

static const struct firmware_version min_fw = {
	.major = 6, .minor = 0, .rev = 55,
};

static const struct venus_resources qcm2290_res = {
	.bw_tbl_dec = qcm2290_bw_table_dec,
	.bw_tbl_dec_size = ARRAY_SIZE(qcm2290_bw_table_dec),
	.bw_tbl_enc = qcm2290_bw_table_enc,
	.bw_tbl_enc_size = ARRAY_SIZE(qcm2290_bw_table_enc),
	.clks = { "core", "iface", "bus", "throttle" },
	.clks_num = 4,
	.vcodec0_clks = { "vcodec0_core", "vcodec0_bus" },
	.vcodec_clks_num = 2,
	.vcodec_pmdomains = (const char *[]) { "venus", "vcodec0" },
	.vcodec_pmdomains_num = 2,
	.opp_pmdomain = (const char *[]) { "cx" },
	.opp_pmdomain_num = 1,
	.vcodec_num = 1,
	.hfi_version = HFI_VERSION_4XX,
	.vpu_version = VPU_VERSION_AR50_LITE,
	.max_load = 352800,
	.num_vpp_pipes = 1,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.cp_start = 0,
	.cp_size = 0x70800000,
	.cp_nonpixel_start = 0x1000000,
	.cp_nonpixel_size = 0x24800000,
	.dma_mask = 0xe0000000 - 1,
	.fwname = "qcom/venus-6.0/venus.mbn",
	.dec_nodename = "video-decoder",
	.enc_nodename = "video-encoder",
	.min_fw = &min_fw,
};

static const struct of_device_id venus_dt_match[] = {
	{ .compatible = "qcom,msm8916-venus", .data = &msm8916_res, },
	{ .compatible = "qcom,msm8996-venus", .data = &msm8996_res, },
	{ .compatible = "qcom,msm8998-venus", .data = &msm8998_res, },
	{ .compatible = "qcom,qcm2290-venus", .data = &qcm2290_res, },
	{ .compatible = "qcom,sc7180-venus", .data = &sc7180_res, },
	{ .compatible = "qcom,sc7280-venus", .data = &sc7280_res, },
	{ .compatible = "qcom,sdm660-venus", .data = &sdm660_res, },
	{ .compatible = "qcom,sdm845-venus", .data = &sdm845_res, },
	{ .compatible = "qcom,sdm845-venus-v2", .data = &sdm845_res_v2, },
	{ .compatible = "qcom,sm8250-venus", .data = &sm8250_res, },
	{ }
};
MODULE_DEVICE_TABLE(of, venus_dt_match);

static struct platform_driver qcom_venus_driver = {
	.probe = venus_probe,
	.remove = venus_remove,
	.driver = {
		.name = "qcom-venus",
		.of_match_table = venus_dt_match,
		.pm = &venus_pm_ops,
	},
	.shutdown = venus_core_shutdown,
};
module_platform_driver(qcom_venus_driver);

MODULE_ALIAS("platform:qcom-venus");
MODULE_DESCRIPTION("Qualcomm Venus video encoder and decoder driver");
MODULE_LICENSE("GPL v2");
