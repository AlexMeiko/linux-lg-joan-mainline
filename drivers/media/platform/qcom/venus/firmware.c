// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Linaro Ltd.
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/sizes.h>
#include <linux/soc/qcom/mdt_loader.h>

#include "core.h"
#include "firmware.h"
#include "hfi_venus_io.h"
#include <linux/delay.h>

#ifndef JOAN_VTRACE
#define JOAN_VTRACE(fmt, ...) do { \
	pr_info("joan-venus-trace: " fmt "\n", ##__VA_ARGS__); \
} while (0)
#endif


#define VENUS_PAS_ID			9
#define VENUS_FW_MEM_SIZE		(6 * SZ_1M)
#define VENUS_FW_START_ADDR		0x0

/*
 * ★ 第十七候选（2026-10-26）：只写 WRAPPER_CLOCK_CONFIG(0xe0004)=0
 *
 * 依据：下游 msm8998 vidc 每次上电、时钟使能后、PAS 之前都写
 *   VIDC_WRAPPER_CLOCK_CONFIG(0xe0004)     = 0
 *   VIDC_WRAPPER_CPU_CLOCK_CONFIG(0xe2000) = 0
 * 主线定义了 WRAPPER_CLOCK_CONFIG(hfi_venus_io.h:98) 却在全驱动里从未使用；
 * 0xe2000 已由第十六候选实读 = 0（故本候选只补 0x04）。
 *
 * 0 = 关闭（默认）；1 = PAS 前纯 posted 写（不复读，最安全）；
 * 2 = 读→写→读（多拿一个初值，但多一次同页读）。
 * 注意：0xe1020-0xe1034 属 VBIF 域，读即形态 B（已实测），本候选绝不触碰。
 */
extern unsigned int joan_clock_config;	/* 定义与 module_param 在 hfi_venus.c */

static void venus_reset_cpu(struct venus_core *core)
{
	u32 fw_size = core->fw.mapped_mem_size;
	void __iomem *wrapper_base;

	if (IS_IRIS2(core) || IS_IRIS2_1(core))
		wrapper_base = core->wrapper_tz_base;
	else
		wrapper_base = core->wrapper_base;

	writel(0, wrapper_base + WRAPPER_FW_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_FW_END_ADDR);
	writel(0, wrapper_base + WRAPPER_CPA_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_CPA_END_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_NONPIX_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_NONPIX_END_ADDR);

	if (IS_IRIS2(core) || IS_IRIS2_1(core)) {
		/* Bring XTSS out of reset */
		writel(0, wrapper_base + WRAPPER_TZ_XTSS_SW_RESET);
	} else {
		writel(0x0, wrapper_base + WRAPPER_CPU_CGC_DIS);
		writel(0x0, wrapper_base + WRAPPER_CPU_CLOCK_CONFIG);

		/* Bring ARM9 out of reset */
		writel(0, wrapper_base + WRAPPER_A9SS_SW_RESET);
	}
}

int venus_set_hw_state(struct venus_core *core, bool resume)
{
	int ret;

	JOAN_VTRACE("set_hw_state: enter resume=%d use_tz=%u", resume, core->use_tz);

	if (core->use_tz) {
		JOAN_VTRACE("set_hw_state: before remote_state state=%d id=0", resume);
		ret = qcom_scm_set_remote_state(resume, 0);
		JOAN_VTRACE("set_hw_state: remote_state raw_ret=%d", ret);
		if (resume && ret == -EINVAL)
			ret = 0;
		return ret;
	}

	if (resume) {
		JOAN_VTRACE("set_hw_state: before venus_reset_cpu");
		venus_reset_cpu(core);
		JOAN_VTRACE("set_hw_state: after venus_reset_cpu");
	} else {
		if (IS_IRIS2(core) || IS_IRIS2_1(core))
			writel(WRAPPER_XTSS_SW_RESET_BIT,
			       core->wrapper_tz_base + WRAPPER_TZ_XTSS_SW_RESET);
		else
			writel(WRAPPER_A9SS_SW_RESET_BIT,
			       core->wrapper_base + WRAPPER_A9SS_SW_RESET);
	}

	return 0;
}

static int venus_load_fw(struct venus_core *core, const char *fwname,
			 phys_addr_t *mem_phys, size_t *mem_size)
{
	const struct firmware *mdt;
	struct reserved_mem *rmem;
	struct device_node *node;
	struct device *dev;
	ssize_t fw_size;
	void *mem_va;
	int ret;

	*mem_phys = 0;
	*mem_size = 0;

	dev = core->dev;
	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(dev, "no memory-region specified\n");
		return -EINVAL;
	}

	rmem = of_reserved_mem_lookup(node);
	of_node_put(node);
	if (!rmem) {
		dev_err(dev, "failed to lookup reserved memory-region\n");
		return -EINVAL;
	}

	JOAN_VTRACE("load_fw: request_firmware(%s)", fwname);
	ret = request_firmware(&mdt, fwname, dev);
	JOAN_VTRACE("load_fw: request_firmware ret=%d", ret);
	if (ret < 0)
		return ret;

	fw_size = qcom_mdt_get_size(mdt);
	if (fw_size < 0) {
		ret = fw_size;
		goto err_release_fw;
	}

	*mem_phys = rmem->base;
	*mem_size = rmem->size;

	if (*mem_size < fw_size || fw_size > VENUS_FW_MEM_SIZE) {
		ret = -EINVAL;
		goto err_release_fw;
	}

	mem_va = memremap(*mem_phys, *mem_size, MEMREMAP_WC);
	if (!mem_va) {
		dev_err(dev, "unable to map memory region %pa size %#zx\n", mem_phys, *mem_size);
		ret = -ENOMEM;
		goto err_release_fw;
	}

	JOAN_VTRACE("load_fw: before mdt_load use_tz=%u fw_size=%zd rmem=%pa/%#zx",
		    core->use_tz, fw_size, mem_phys, *mem_size);
	if (core->use_tz)
		ret = qcom_mdt_load(dev, mdt, fwname, VENUS_PAS_ID,
				    mem_va, *mem_phys, *mem_size, NULL);
	else
		ret = qcom_mdt_load_no_init(dev, mdt, fwname, mem_va,
					    *mem_phys, *mem_size, NULL);
	JOAN_VTRACE("load_fw: mdt_load ret=%d", ret);

	memunmap(mem_va);
err_release_fw:
	release_firmware(mdt);
	return ret;
}

static int venus_boot_no_tz(struct venus_core *core, phys_addr_t mem_phys,
			    size_t mem_size)
{
	struct iommu_domain *iommu;
	struct device *dev;
	int ret;

	dev = core->fw.dev;
	if (!dev)
		return -EPROBE_DEFER;

	iommu = core->fw.iommu_domain;
	core->fw.mapped_mem_size = mem_size;

	ret = iommu_map(iommu, VENUS_FW_START_ADDR, mem_phys, mem_size,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "could not map video firmware region\n");
		return ret;
	}

	venus_reset_cpu(core);

	return 0;
}

static int venus_shutdown_no_tz(struct venus_core *core)
{
	const size_t mapped = core->fw.mapped_mem_size;
	struct iommu_domain *iommu;
	size_t unmapped;
	u32 reg;
	struct device *dev = core->fw.dev;
	void __iomem *wrapper_base = core->wrapper_base;
	void __iomem *wrapper_tz_base = core->wrapper_tz_base;

	if (IS_IRIS2(core) || IS_IRIS2_1(core)) {
		/* Assert the reset to XTSS */
		reg = readl(wrapper_tz_base + WRAPPER_TZ_XTSS_SW_RESET);
		reg |= WRAPPER_XTSS_SW_RESET_BIT;
		writel(reg, wrapper_tz_base + WRAPPER_TZ_XTSS_SW_RESET);
	} else {
		/* Assert the reset to ARM9 */
		reg = readl(wrapper_base + WRAPPER_A9SS_SW_RESET);
		reg |= WRAPPER_A9SS_SW_RESET_BIT;
		writel(reg, wrapper_base + WRAPPER_A9SS_SW_RESET);
	}

	iommu = core->fw.iommu_domain;

	if (core->fw.mapped_mem_size && iommu) {
		unmapped = iommu_unmap(iommu, VENUS_FW_START_ADDR, mapped);

		if (unmapped != mapped)
			dev_err(dev, "failed to unmap firmware\n");
		else
			core->fw.mapped_mem_size = 0;
	}

	return 0;
}

int venus_firmware_cfg(struct venus_core *core)
{
	void __iomem *cpu_cs_base = core->cpu_cs_base;

	if (IS_AR50_LITE(core))
		writel(CPU_CS_VCICMD_ARP_OFF, cpu_cs_base + CPU_CS_VCICMD);

	return 0;
}

int venus_boot(struct venus_core *core)
{
	struct device *dev = core->dev;
	const struct venus_resources *res = core->res;
	const char *fwpath = NULL;
	phys_addr_t mem_phys;
	size_t mem_size;
	int ret;

	if (!IS_ENABLED(CONFIG_QCOM_MDT_LOADER) ||
	    (core->use_tz && !qcom_scm_is_available()))
		return -EPROBE_DEFER;

	ret = of_property_read_string_index(dev->of_node, "firmware-name", 0,
					    &fwpath);
	if (ret)
		fwpath = core->res->fwname;

	JOAN_VTRACE("boot: fwpath=%s", fwpath);
	ret = venus_load_fw(core, fwpath, &mem_phys, &mem_size);
	JOAN_VTRACE("boot: venus_load_fw ret=%d mem_phys=%pa mem_size=%#zx",
		    ret, &mem_phys, mem_size);
	if (ret) {
		dev_err(dev, "fail to load video firmware\n");
		return -EINVAL;
	}

	core->fw.mem_size = mem_size;
	core->fw.mem_phys = mem_phys;

	if (core->use_tz &&
	    of_device_is_compatible(dev->of_node, "qcom,msm8998-venus")) {
		JOAN_VTRACE("boot: pre-PAS CPU_CGC_DIS before writel reg=%#x val=0",
			    WRAPPER_BASE + WRAPPER_CPU_CGC_DIS);
		writel(0, core->wrapper_base + WRAPPER_CPU_CGC_DIS);
		JOAN_VTRACE("boot: pre-PAS CPU_CGC_DIS posted write returned");

		/* 第十七候选：补下游 power-on 末尾的 WRAPPER_CLOCK_CONFIG=0 */
		if (joan_clock_config) {
			if (joan_clock_config >= 2) {
				JOAN_VTRACE("clkcfg: before readl(wrapper+%#x)",
					    WRAPPER_CLOCK_CONFIG);
				JOAN_VTRACE("clkcfg: value_before=%#x",
					    readl(core->wrapper_base +
						  WRAPPER_CLOCK_CONFIG));
			}
			JOAN_VTRACE("clkcfg: before writel(wrapper+%#x)=0",
				    WRAPPER_CLOCK_CONFIG);
			writel(0, core->wrapper_base + WRAPPER_CLOCK_CONFIG);
			JOAN_VTRACE("clkcfg: posted write returned");
			if (joan_clock_config >= 2)
				JOAN_VTRACE("clkcfg: value_after=%#x",
					    readl(core->wrapper_base +
						  WRAPPER_CLOCK_CONFIG));
		}
	}

	JOAN_VTRACE("boot: before pas_auth_and_reset(9) use_tz=%u", core->use_tz);
	if (core->use_tz)
		ret = qcom_scm_pas_auth_and_reset(VENUS_PAS_ID);
	else
		ret = venus_boot_no_tz(core, mem_phys, mem_size);
	JOAN_VTRACE("boot: pas_auth_and_reset ret=%d", ret);

	if (ret)
		return ret;

	if (core->use_tz && res->cp_size) {
		/*
		 * Clues for porting using downstream data:
		 * cp_start = 0
		 * cp_size = venus_ns/virtual-addr-pool[0] - yes, address and not size!
		 *   This works, as the non-secure context bank is placed
		 *   contiguously right after the Content Protection region.
		 *
		 * cp_nonpixel_start = venus_sec_non_pixel/virtual-addr-pool[0]
		 * cp_nonpixel_size = venus_sec_non_pixel/virtual-addr-pool[1]
		 */
		pr_info("joan-venus-trace: boot: before mem_protect_video_var cp=%#x/%#x nonpixel=%#x/%#x\n",
			res->cp_start, res->cp_size,
			res->cp_nonpixel_start, res->cp_nonpixel_size);
		ret = qcom_scm_mem_protect_video_var(res->cp_start,
						     res->cp_size,
						     res->cp_nonpixel_start,
						     res->cp_nonpixel_size);
		pr_info("joan-venus-trace: boot: mem_protect_video_var ret=%d\n", ret);
		if (ret) {
			int shutdown_ret;

			pr_info("joan-venus-trace: boot: before CP error pas_shutdown(9)\n");
			shutdown_ret = qcom_scm_pas_shutdown(VENUS_PAS_ID);
			pr_info("joan-venus-trace: boot: CP error pas_shutdown ret=%d\n",
				shutdown_ret);
			dev_err(dev, "set virtual address ranges fail (%d)\n",
				ret);
			return ret;
		}
	}

	return 0;
}

int venus_shutdown(struct venus_core *core)
{
	int ret;

	if (core->use_tz)
		ret = qcom_scm_pas_shutdown(VENUS_PAS_ID);
	else
		ret = venus_shutdown_no_tz(core);

	return ret;
}

int venus_firmware_check(struct venus_core *core)
{
	const struct firmware_version *req = core->res->min_fw;
	const struct firmware_version *run = &core->venus_ver;

	if (!req)
		return 0;

	if (!is_fw_rev_or_newer(core, req->major, req->minor, req->rev))
		goto error;

	return 0;
error:
	dev_err(core->dev, "Firmware v%d.%d.%d < v%d.%d.%d\n",
		run->major, run->minor, run->rev,
		req->major, req->minor, req->rev);

	return -EINVAL;
}

int venus_firmware_init(struct venus_core *core)
{
	struct platform_device_info info;
	struct iommu_domain *iommu_dom;
	struct platform_device *pdev;
	struct device_node *np;
	int ret;

	np = of_get_child_by_name(core->dev->of_node, "video-firmware");
	if (!np) {
		core->use_tz = true;
		JOAN_VTRACE("firmware_init: no video-firmware node -> use_tz=1");
		return 0;
	}
	JOAN_VTRACE("firmware_init: video-firmware node present -> use_tz=0");

	memset(&info, 0, sizeof(info));
	info.fwnode = &np->fwnode;
	info.parent = core->dev;
	info.name = np->name;
	info.dma_mask = DMA_BIT_MASK(32);

	pdev = platform_device_register_full(&info);
	if (IS_ERR(pdev)) {
		of_node_put(np);
		return PTR_ERR(pdev);
	}

	pdev->dev.of_node = np;

	ret = of_dma_configure(&pdev->dev, np, true);
	if (ret) {
		dev_err(core->dev, "dma configure fail\n");
		goto err_unregister;
	}

	core->fw.dev = &pdev->dev;

	iommu_dom = iommu_paging_domain_alloc(core->fw.dev);
	if (IS_ERR(iommu_dom)) {
		dev_err(core->fw.dev, "Failed to allocate iommu domain\n");
		ret = PTR_ERR(iommu_dom);
		goto err_unregister;
	}

	ret = iommu_attach_device(iommu_dom, core->fw.dev);
	if (ret) {
		dev_err(core->fw.dev, "could not attach device\n");
		goto err_iommu_free;
	}

	core->fw.iommu_domain = iommu_dom;

	of_node_put(np);

	return 0;

err_iommu_free:
	iommu_domain_free(iommu_dom);
err_unregister:
	platform_device_unregister(pdev);
	of_node_put(np);
	return ret;
}

void venus_firmware_deinit(struct venus_core *core)
{
	struct iommu_domain *iommu;

	if (!core->fw.dev)
		return;

	iommu = core->fw.iommu_domain;

	iommu_detach_device(iommu, core->fw.dev);

	if (core->fw.iommu_domain) {
		iommu_domain_free(iommu);
		core->fw.iommu_domain = NULL;
	}

	platform_device_unregister(to_platform_device(core->fw.dev));
}
