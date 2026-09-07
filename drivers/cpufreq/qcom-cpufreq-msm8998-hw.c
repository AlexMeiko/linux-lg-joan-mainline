// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8998 OSM/CPRH CPU frequency driver.
 *
 * The OSM v2 block in MSM8998 is not initialized by the boot firmware.  This
 * driver derives the per-device open-loop voltages from QFPROM, programs both
 * CPRH controllers and OSM domains, then exposes the two CPU clusters through
 * the CPUFreq core.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include <asm/cputype.h>

#include "qcom-cpufreq-msm8998.h"

#define MSM8998_OSM_DOMAIN_STRIDE		0x2000
#define MSM8998_OSM_TABLE_STRIDE		0x20
#define MSM8998_OSM_VERSION			0x0000
#define MSM8998_OSM_ENABLE			0x1004
#define MSM8998_OSM_CC_ZERO_BEHAV_CTRL		0x100c
#define MSM8998_OSM_SPM_CC_HYSTERESIS		0x101c
#define MSM8998_OSM_SPM_CC_DCVS_DISABLE		0x1020
#define MSM8998_OSM_SPM_CORE_RET_MAPPING	0x1024
#define MSM8998_OSM_LLM_FREQ_HYSTERESIS		0x102c
#define MSM8998_OSM_LLM_VOLT_HYSTERESIS		0x1030
#define MSM8998_OSM_LLM_DCVS_DISABLE		0x1034
#define MSM8998_OSM_PDN_FSM_CTRL		0x1070
#define MSM8998_OSM_CC_BOOST_TIMER0		0x1074
#define MSM8998_OSM_CC_BOOST_TIMER1		0x1078
#define MSM8998_OSM_CC_BOOST_TIMER2		0x107c
#define MSM8998_OSM_DCVS_BOOST_TIMER0		0x1084
#define MSM8998_OSM_DCVS_BOOST_TIMER1		0x1088
#define MSM8998_OSM_DCVS_BOOST_TIMER2		0x108c
#define MSM8998_OSM_PS_BOOST_TIMER0		0x1094
#define MSM8998_OSM_PS_BOOST_TIMER1		0x1098
#define MSM8998_OSM_PS_BOOST_TIMER2		0x109c
#define MSM8998_OSM_BOOST_SYNC_DELAY		0x10a0
#define MSM8998_OSM_DROOP_CTRL			0x10a4
#define MSM8998_OSM_DROOP_RELEASE_TIMER		0x10a8
#define MSM8998_OSM_DROOP_UNSTALL_TIMER		0x10ac
#define MSM8998_OSM_DROOP_WAIT_TIMER0		0x10b0
#define MSM8998_OSM_DCVS_DROOP_TIMER		0x10b8
#define MSM8998_OSM_DROOP_SYNC_DELAY		0x10bc
#define MSM8998_OSM_PLL_SW_OVERRIDE		0x10c0
#define MSM8998_OSM_INDEX			0x1150
#define MSM8998_OSM_FREQUENCY			0x1154
#define MSM8998_OSM_VOLTAGE			0x1158
#define MSM8998_OSM_OVERRIDE			0x115c
#define MSM8998_OSM_SPARE			0x1164
#define MSM8998_OSM_CYCLE_COUNTER_CTRL		0x1f00
#define MSM8998_OSM_DESIRED			0x1f10
#define OSM_CYCLE_COUNTER_USE_XO_EDGE_EN	BIT(8)
#define MSM8998_OSM_FSM_CC_BOOST		BIT(0)
#define MSM8998_OSM_FSM_PS_BOOST		BIT(1)
#define MSM8998_OSM_FSM_DCVS_BOOST		BIT(2)
#define MSM8998_OSM_FSM_PC_RET_DROOP		BIT(3)
#define MSM8998_OSM_FSM_WFX_DROOP		BIT(4)
#define MSM8998_OSM_FSM_DCVS_DROOP		BIT(5)

#define MSM8998_CPR_CTL				0x004
#define MSM8998_CPR_CTL_LOOP_EN			BIT(0)
#define MSM8998_CPR_TIMER_AUTO_CONT		0x00c
#define MSM8998_CPR_STEP_QUOT			0x014
#define MSM8998_CPR_GCNT(_ro)			(0x0a0 + 4 * (_ro))
#define MSM8998_CPR_SENSOR_OWNER(_sensor)	(0x200 + 4 * (_sensor))
#define MSM8998_CPR_THRESHOLD			0x808
#define MSM8998_CPR_RO_MASK			0x80c
#define MSM8998_CPR_TARGET_QUOT(_ro)		(0x840 + 4 * (_ro))
#define MSM8998_CPR_SAW_ERROR_STEP		0x7a4
#define MSM8998_CPR_MARGIN_TIMERS		0x7a8
#define MSM8998_CPR_MARGIN_ADJ_CTL		0x7f8
#define MSM8998_CPR_CORNER(_corner)		(0x3a00 + 4 * (_corner))
#define MSM8998_CPRH_CTL			0x3aa0
#define MSM8998_CPRH_CTL_OSM_ENABLED		BIT(0)

#define MSM8998_SAW_SIZE			0x1000
#define MSM8998_SAW_VERSION			0xfd0
#define MSM8998_SAW_ID				0xc04
#define MSM8998_SAW_ID_PMIC_ARB_PRESENT	BIT(2)
#define MSM8998_SAW_VCTL			0x900
#define MSM8998_SAW_AVS_CTL			0x904
#define MSM8998_SAW_AVS_LIMIT			0x908
#define MSM8998_SAW_PMIC_STS			0xc18
#define JOAN_SAW_AVS_CTL			0x01010031
#define JOAN_SAW_BOOT_AVS_LIMIT		0x00000000

#define MSM8998_ACD_VERSION			0x00
#define MSM8998_ACD_AUTO_TRANSFER_CONFIG	0x80
#define MSM8998_ACD_AUTO_TRANSFER		0x84
#define MSM8998_ACD_AUTO_TRANSFER_STATUS	0x8c
#define MSM8998_ACD_WRITE_CTL			0x90
#define MSM8998_ACD_WRITE_STATUS		0x94

#define MSM8998_APCS_OSM_CMD_RCG		0x12c
#define MSM8998_APCS_OSM_CFG_RCG		0x130
#define MSM8998_APCS_RCG_UPDATE			BIT(0)
#define MSM8998_APCS_RCG_ROOT_EN		BIT(1)
#define MSM8998_APCS_RCG_DIV_MASK		GENMASK(4, 0)
#define MSM8998_APCS_RCG_SRC_MASK		GENMASK(10, 8)
#define MSM8998_APCS_RCG_DIV_1P5		2
#define MSM8998_APCS_RCG_SRC_GPLL0		FIELD_PREP(MSM8998_APCS_RCG_SRC_MASK, 1)

#define MSM8998_OSM_CLOCK_HZ			200000000U
#define MSM8998_XO_CLOCK_HZ			19200000U
#define MSM8998_AUX_CLOCK_HZ			300000000U
#define MSM8998_OSM_INITIAL_HZ			300000000U
#define MSM8998_POWER_BOOT_HZ			1555200000U
#define MSM8998_PERFORMANCE_BOOT_HZ		1728000000U
#define MSM8998_OSM_TRANSITION_LATENCY_NS	10000U
#define MSM8998_OSM_COUNT_NS(_ns)		((u32)(((u64)MSM8998_OSM_CLOCK_HZ * (_ns)) / NSEC_PER_SEC))

struct msm8998_osm_domain {
	void __iomem *osm;
	void __iomem *cprh;
	void __iomem *acd;
	void __iomem *saw;
	phys_addr_t osm_phys;
	struct msm8998_cpr_corner_data corners[MSM8998_CPR_MAX_CORNER_COUNT];
	struct msm8998_osm_lut_data lut[MSM8998_OSM_TABLE_SIZE];
	struct msm8998_osm_opp opps[MSM8998_OSM_MAX_OPP_COUNT];
	struct msm8998_osm_hw_table_image hw_table;
	struct msm8998_cprh_register_image cprh_image;
	struct msm8998_osm_sequence_image sequence;
	struct msm8998_osm_acd_image acd_image;
	struct cpufreq_frequency_table frequency_table[MSM8998_OSM_MAX_OPP_COUNT + 1];
	cpumask_t cpus;
	unsigned int corner_count;
	unsigned int opp_count;
	unsigned int cluster;
	bool enabled;
};

struct msm8998_osm_drv {
	struct device *dev;
	void __iomem *osm_base;
	void __iomem *apcs_common;
	phys_addr_t osm_phys;
	struct clk *cpr_clk;
	struct clk *aux_clk;
	struct clk *xo_clk;
	bool clocks_enabled;
	bool saw_ready;
	struct msm8998_osm_domain domain[MSM8998_CPR_CLUSTER_COUNT];
};

struct msm8998_saw_config {
	const char *resource_name;
	phys_addr_t phys;
	u32 avs_limit;
};

static const struct msm8998_saw_config joan_saw_configs[] = {
	[MSM8998_CPR_POWER_CLUSTER] = {
		.resource_name = "power-saw",
		.phys = 0x17912000,
		.avs_limit = 0x04200420,
	},
	[MSM8998_CPR_PERFORMANCE_CLUSTER] = {
		.resource_name = "performance-saw",
		.phys = 0x17812000,
		.avs_limit = 0x04700470,
	},
};

struct msm8998_saw_snapshot {
	u32 version;
	u32 id;
	u32 vctl;
	u32 avs_ctl;
	u32 avs_limit;
	u32 pmic_sts;
};

#define JOAN_CORNER(_frequency, _floor, _ceiling, _open_adj, _closed_adj, _range) \
	{ \
		.frequency_hz = (_frequency), \
		.floor_uv = (_floor), \
		.ceiling_uv = (_ceiling), \
		.open_loop_adjust_uv = (_open_adj), \
		.closed_loop_adjust_uv = (_closed_adj), \
		.max_floor_to_ceiling_uv = (_range), \
	}

static const struct msm8998_cpr_corner_config joan_power_corners[] = {
	JOAN_CORNER(300000000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(364800000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(441600000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(518400000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(595200000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(672000000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(748800000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(825600000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(883200000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(960000000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(1036800000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(1094400000, 632000, 900000, -12000, -10000, 32000),
	JOAN_CORNER(1171200000, 632000, 900000, -12000, -11000, 32000),
	JOAN_CORNER(1248000000, 632000, 900000, -12000, -12000, 32000),
	JOAN_CORNER(1324800000, 632000, 900000, -12000, -13000, 32000),
	JOAN_CORNER(1401600000, 632000, 900000, -12000, -14000, 32000),
	JOAN_CORNER(1478400000, 632000, 900000, -16000, -14000, 32000),
	JOAN_CORNER(1555200000, 632000, 900000, -16000, -15000, 32000),
	JOAN_CORNER(1670400000, 712000, 952000, -20000, -21000, 40000),
	JOAN_CORNER(1747200000, 712000, 952000, -24000, -24000, 40000),
	JOAN_CORNER(1824000000, 772000, 1056000, -28000, -26000, 40000),
	JOAN_CORNER(1900800000, 772000, 1056000, -28000, -28000, 40000),
};

static const u32 joan_power_ro_scaling[] = {
	0xa23, 0xaea, 0xa11, 0xaca, 0x9a7, 0xa72, 0x897, 0x9f9,
	0xc75, 0xcb7, 0xc78, 0xb92, 0xbee, 0xba6, 0x7fa, 0xb81,
	0xa23, 0xaea, 0xa11, 0xaca, 0x9a7, 0xa72, 0x897, 0x9f9,
	0xc75, 0xcb7, 0xc78, 0xb92, 0xbee, 0xba6, 0x7fa, 0xb81,
	0x957, 0x9f6, 0x9b3, 0xa4e, 0x94e, 0xa04, 0x8d3, 0x9fb,
	0xace, 0xbe1, 0xbac, 0xb77, 0xb39, 0xa80, 0x7dd, 0xae0,
	0x812, 0x869, 0x8fc, 0x982, 0x8ac, 0x952, 0x8f0, 0x9a1,
	0x7ec, 0x9cf, 0x9b7, 0xaae, 0x9fa, 0x845, 0x764, 0x949,
};

static const struct msm8998_cpr_domain_config joan_power_config = {
	.corners = joan_power_corners,
	.ro_scaling_factor = joan_power_ro_scaling,
	.corner_count = ARRAY_SIZE(joan_power_corners),
	.fmax_corner = { 7, 10, 17, 21 },
	.fuse_closed_loop_adjust_uv = { 20000, 26000, 12000, 30000 },
	.step_uv = 4000,
	.apm_threshold_uv = 800000,
	.apm_crossover_uv = 880000,
	.mem_acc_threshold_uv = 852000,
	.mem_acc_crossover_uv = 852000,
	.scaled_open_loop_as_ceiling = true,
	.allow_quotient_interpolation = true,
};

static const struct msm8998_cpr_corner_config joan_performance_corners[] = {
	JOAN_CORNER(300000000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(345600000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(422400000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(499200000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(576000000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(652800000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(729600000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(806400000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(902400000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(979200000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(1056000000, 568000, 828000, 0, 0, 32000),
	JOAN_CORNER(1132800000, 568000, 828000, -8000, -10000, 32000),
	JOAN_CORNER(1190400000, 632000, 900000, -12000, -10000, 32000),
	JOAN_CORNER(1267200000, 632000, 900000, -12000, -11000, 32000),
	JOAN_CORNER(1344000000, 632000, 900000, -12000, -12000, 32000),
	JOAN_CORNER(1420800000, 632000, 900000, -12000, -12000, 32000),
	JOAN_CORNER(1497600000, 632000, 900000, -12000, -13000, 32000),
	JOAN_CORNER(1574400000, 632000, 900000, -12000, -14000, 32000),
	JOAN_CORNER(1651200000, 632000, 900000, -16000, -14000, 32000),
	JOAN_CORNER(1728000000, 632000, 900000, -16000, -15000, 32000),
	JOAN_CORNER(1804800000, 712000, 952000, -20000, -16000, 40000),
	JOAN_CORNER(1881600000, 712000, 952000, -16000, -16000, 40000),
	JOAN_CORNER(1958400000, 712000, 952000, -16000, -17000, 40000),
	JOAN_CORNER(2035200000, 772000, 1136000, -16000, -15000, 40000),
	JOAN_CORNER(2112000000, 772000, 1136000, -12000, -14000, 40000),
	JOAN_CORNER(2208000000, 772000, 1136000, -28000, -27000, 40000),
	JOAN_CORNER(2265600000, 772000, 1136000, -28000, -27000, 40000),
	JOAN_CORNER(2323200000, 772000, 1136000, -28000, -28000, 40000),
	JOAN_CORNER(2342400000, 772000, 1136000, -28000, -28000, 40000),
	JOAN_CORNER(2361600000, 772000, 1136000, -28000, -28000, 40000),
};

static const u32 joan_performance_ro_scaling[] = {
	0xb29, 0xbf1, 0xb0c, 0xb88, 0xa8b, 0xaee, 0x98e, 0xa47,
	0xa45, 0xa12, 0x8c4, 0xd10, 0xcd9, 0xc41, 0xc5c, 0xa5f,
	0xb29, 0xbf1, 0xb0c, 0xb88, 0xa8b, 0xaee, 0x98e, 0xa47,
	0xa45, 0xa12, 0x8c4, 0xd10, 0xcd9, 0xc41, 0xc5c, 0xa5f,
	0xa2b, 0xac3, 0xa74, 0xad9, 0xa0d, 0xa7d, 0x9a1, 0xa32,
	0x908, 0x977, 0x8c3, 0xc20, 0xbce, 0xbdc, 0xab4, 0x8ff,
	0x76d, 0x7e0, 0x830, 0x8b4, 0x7f2, 0x871, 0x81d, 0x88c,
	0x61d, 0x74e, 0x785, 0x8bb, 0x89d, 0x96d, 0x6e2, 0x5c6,
};

static const struct msm8998_cpr_domain_config joan_performance_config = {
	.corners = joan_performance_corners,
	.ro_scaling_factor = joan_performance_ro_scaling,
	.corner_count = ARRAY_SIZE(joan_performance_corners),
	.fmax_corner = { 7, 11, 19, 29 },
	.fuse_closed_loop_adjust_uv = { 0, 0, 12000, 50000 },
	.step_uv = 4000,
	.apm_threshold_uv = 800000,
	.apm_crossover_uv = 880000,
	.mem_acc_threshold_uv = 852000,
	.mem_acc_crossover_uv = 852000,
	.scaled_open_loop_as_ceiling = true,
	.allow_quotient_interpolation = true,
};

static const struct msm8998_cprh_controller_config joan_power_cprh = {
	.clock_rate_hz = MSM8998_XO_CLOCK_HZ,
	.sensor_time_ns = 1000,
	.loop_time_ns = 5000000,
	.voltage_settling_time_ns = 1760,
	.corner_switch_delay_time_ns = 1042,
	.base_uv = 348000,
	.step_uv = 4000,
	.count_repeat = 1,
	.sensor_count = 6,
	.idle_clocks = 15,
	.step_quot_init_min = 11,
	.step_quot_init_max = 12,
	.consecutive_down = 2,
	.up_threshold = 2,
	.down_threshold = 2,
	.down_error_step_limit = 1,
	.up_error_step_limit = 1,
	.saw_use_unit_mv = true,
	.use_dynamic_step_quot = true,
};

static const struct msm8998_cprh_controller_config joan_performance_cprh = {
	.clock_rate_hz = MSM8998_XO_CLOCK_HZ,
	.sensor_time_ns = 1000,
	.loop_time_ns = 5000000,
	.voltage_settling_time_ns = 1760,
	.corner_switch_delay_time_ns = 1042,
	.base_uv = 348000,
	.step_uv = 4000,
	.count_repeat = 1,
	.sensor_count = 9,
	.idle_clocks = 15,
	.step_quot_init_min = 9,
	.step_quot_init_max = 14,
	.consecutive_down = 2,
	.up_threshold = 2,
	.down_threshold = 2,
	.down_error_step_limit = 1,
	.up_error_step_limit = 1,
	.saw_use_unit_mv = true,
	.use_dynamic_step_quot = true,
};

static const struct msm8998_osm_sequence_config joan_power_sequence = {
	.apm_threshold_uv = 800000,
	.mem_acc_threshold_uv = 852000,
};

static const struct msm8998_osm_sequence_config joan_performance_sequence = {
	.apm_threshold_uv = 800000,
	.mem_acc_threshold_uv = 852000,
};

static const struct msm8998_osm_acd_config joan_acd_config = {
	.tunable_delay = 0x00009611,
	.control = 0x002b5ffd,
	.soft_start_control = 0x00000501,
	.initial_extint = 0x02cf9ae8,
	.final_extint = 0x02cf9afe,
	.auto_transfer_control = 0x00000015,
};

#define JOAN_OSM_LUT(_frequency, _frequency_data, _override, _spare, _vc) \
	{ \
		.frequency_hz = (_frequency), \
		.frequency_data = (_frequency_data), \
		.override_data = (_override), \
		.spare_data = (_spare), \
		.virtual_corner = (_vc), \
	}

static const struct msm8998_osm_lut_config joan_power_lut[] = {
	JOAN_OSM_LUT(300000000, 0x0004000f, 0x01200020, 1, 1),
	JOAN_OSM_LUT(364800000, 0x05040013, 0x01200020, 1, 2),
	JOAN_OSM_LUT(441600000, 0x05040017, 0x02200020, 1, 3),
	JOAN_OSM_LUT(518400000, 0x0504001b, 0x02200020, 1, 4),
	JOAN_OSM_LUT(595200000, 0x0504001f, 0x02200020, 1, 5),
	JOAN_OSM_LUT(672000000, 0x05040023, 0x03200020, 1, 6),
	JOAN_OSM_LUT(748800000, 0x05040027, 0x03200020, 1, 7),
	JOAN_OSM_LUT(825600000, 0x0404002b, 0x03220022, 1, 8),
	JOAN_OSM_LUT(883200000, 0x0404002e, 0x04250025, 1, 9),
	JOAN_OSM_LUT(960000000, 0x04040032, 0x04280028, 1, 10),
	JOAN_OSM_LUT(1036800000, 0x04040036, 0x042b002b, 1, 11),
	JOAN_OSM_LUT(1094400000, 0x04040039, 0x052e002e, 2, 12),
	JOAN_OSM_LUT(1171200000, 0x0404003d, 0x05310031, 2, 13),
	JOAN_OSM_LUT(1248000000, 0x04040041, 0x05340034, 2, 14),
	JOAN_OSM_LUT(1324800000, 0x04040045, 0x06370037, 2, 15),
	JOAN_OSM_LUT(1401600000, 0x04040049, 0x063a003a, 2, 16),
	JOAN_OSM_LUT(1478400000, 0x0404004d, 0x073e003e, 2, 17),
	JOAN_OSM_LUT(1555200000, 0x04040051, 0x07410041, 2, 18),
	JOAN_OSM_LUT(1670400000, 0x04040057, 0x08460046, 2, 19),
	JOAN_OSM_LUT(1747200000, 0x0404005b, 0x08490049, 2, 20),
	JOAN_OSM_LUT(1824000000, 0x0404005f, 0x084c004c, 3, 21),
	JOAN_OSM_LUT(1900800000, 0x04040063, 0x094f004f, 3, 22),
};

static const struct msm8998_osm_lut_config joan_performance_lut[] = {
	JOAN_OSM_LUT(300000000, 0x0004000f, 0x01200020, 1, 1),
	JOAN_OSM_LUT(345600000, 0x05040012, 0x01200020, 1, 2),
	JOAN_OSM_LUT(422400000, 0x05040016, 0x02200020, 1, 3),
	JOAN_OSM_LUT(499200000, 0x0504001a, 0x02200020, 1, 4),
	JOAN_OSM_LUT(576000000, 0x0504001e, 0x02200020, 1, 5),
	JOAN_OSM_LUT(652800000, 0x05040022, 0x03200020, 1, 6),
	JOAN_OSM_LUT(729600000, 0x05040026, 0x03200020, 1, 7),
	JOAN_OSM_LUT(806400000, 0x0504002a, 0x03220022, 1, 8),
	JOAN_OSM_LUT(902400000, 0x0404002f, 0x04260026, 1, 9),
	JOAN_OSM_LUT(979200000, 0x04040033, 0x04290029, 1, 10),
	JOAN_OSM_LUT(1056000000, 0x04040037, 0x052c002c, 1, 11),
	JOAN_OSM_LUT(1132800000, 0x0404003b, 0x052f002f, 1, 12),
	JOAN_OSM_LUT(1190400000, 0x0404003e, 0x05320032, 2, 13),
	JOAN_OSM_LUT(1267200000, 0x04040042, 0x06350035, 2, 14),
	JOAN_OSM_LUT(1344000000, 0x04040046, 0x06380038, 2, 15),
	JOAN_OSM_LUT(1420800000, 0x0404004a, 0x063b003b, 2, 16),
	JOAN_OSM_LUT(1497600000, 0x0404004e, 0x073e003e, 2, 17),
	JOAN_OSM_LUT(1574400000, 0x04040052, 0x07420042, 2, 18),
	JOAN_OSM_LUT(1651200000, 0x04040056, 0x07450045, 2, 19),
	JOAN_OSM_LUT(1728000000, 0x0404005a, 0x08480048, 2, 20),
	JOAN_OSM_LUT(1804800000, 0x0404005e, 0x084b004b, 2, 21),
	JOAN_OSM_LUT(1881600000, 0x04040062, 0x094e004e, 2, 22),
	JOAN_OSM_LUT(1958400000, 0x04040066, 0x09520052, 2, 23),
	JOAN_OSM_LUT(2035200000, 0x0404006a, 0x09550055, 3, 24),
	JOAN_OSM_LUT(2112000000, 0x0404006e, 0x0a580058, 3, 25),
	JOAN_OSM_LUT(2208000000, 0x04040073, 0x0a5c005c, 3, 26),
	JOAN_OSM_LUT(2265600000, 0x04010076, 0x0a5e005e, 3, 26),
	JOAN_OSM_LUT(2265600000, 0x04040076, 0x0a5e005e, 3, 27),
	JOAN_OSM_LUT(2342400000, 0x0401007a, 0x0a620062, 3, 27),
	JOAN_OSM_LUT(2323200000, 0x04040079, 0x0a610061, 3, 28),
	JOAN_OSM_LUT(2419200000, 0x0401007e, 0x0a650065, 3, 28),
	JOAN_OSM_LUT(2342400000, 0x0404007a, 0x0a620062, 3, 29),
	JOAN_OSM_LUT(2438400000, 0x0401007f, 0x0a660066, 3, 29),
	JOAN_OSM_LUT(2361600000, 0x0404007b, 0x0a620062, 3, 30),
	JOAN_OSM_LUT(2457600000, 0x04010080, 0x0a660066, 3, 30),
};

static struct msm8998_osm_drv *msm8998_osm;

static const char * const msm8998_fuse_cell_names[MSM8998_CPR_FUSE_ROW_COUNT] = {
	"row38", "row39", "row67", "row68",
	"row69", "row70", "row71", "row100",
};

static const struct msm8998_cpr_domain_config * const msm8998_domain_configs[] = {
	[MSM8998_CPR_POWER_CLUSTER] = &joan_power_config,
	[MSM8998_CPR_PERFORMANCE_CLUSTER] = &joan_performance_config,
};

static const struct msm8998_osm_lut_config * const msm8998_domain_luts[] = {
	[MSM8998_CPR_POWER_CLUSTER] = joan_power_lut,
	[MSM8998_CPR_PERFORMANCE_CLUSTER] = joan_performance_lut,
};

static const unsigned int msm8998_domain_lut_counts[] = {
	[MSM8998_CPR_POWER_CLUSTER] = ARRAY_SIZE(joan_power_lut),
	[MSM8998_CPR_PERFORMANCE_CLUSTER] = ARRAY_SIZE(joan_performance_lut),
};

static const struct msm8998_cprh_controller_config * const
msm8998_cprh_configs[] = {
	[MSM8998_CPR_POWER_CLUSTER] = &joan_power_cprh,
	[MSM8998_CPR_PERFORMANCE_CLUSTER] = &joan_performance_cprh,
};

static const struct msm8998_osm_sequence_config * const
msm8998_sequence_configs[] = {
	[MSM8998_CPR_POWER_CLUSTER] = &joan_power_sequence,
	[MSM8998_CPR_PERFORMANCE_CLUSTER] = &joan_performance_sequence,
};

static inline void msm8998_osm_write(struct msm8998_osm_domain *domain,
				     u32 offset, u32 value)
{
	writel_relaxed(value, domain->osm + offset);
}

static inline u32 msm8998_osm_read(struct msm8998_osm_domain *domain,
				   u32 offset)
{
	return readl_relaxed(domain->osm + offset);
}

static inline void msm8998_cpr_write(struct msm8998_osm_domain *domain,
				     u32 offset, u32 value)
{
	writel_relaxed(value, domain->cprh + offset);
}

static inline u32 msm8998_cpr_read(struct msm8998_osm_domain *domain,
				   u32 offset)
{
	return readl_relaxed(domain->cprh + offset);
}

static int msm8998_read_fuse_rows(struct device *dev,
				  struct msm8998_cpr_fuse_rows *rows)
{
	u64 *values[MSM8998_CPR_FUSE_ROW_COUNT] = {
		&rows->row38, &rows->row39, &rows->row67, &rows->row68,
		&rows->row69, &rows->row70, &rows->row71, &rows->row100,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(msm8998_fuse_cell_names); i++) {
		struct nvmem_cell *cell;
		void *buffer;
		size_t length;

		cell = devm_nvmem_cell_get(dev, msm8998_fuse_cell_names[i]);
		if (IS_ERR(cell))
			return dev_err_probe(dev, PTR_ERR(cell),
					    "failed to get QFPROM cell %s\n",
					    msm8998_fuse_cell_names[i]);

		buffer = nvmem_cell_read(cell, &length);
		if (IS_ERR(buffer))
			return dev_err_probe(dev, PTR_ERR(buffer),
					    "failed to read QFPROM cell %s\n",
					    msm8998_fuse_cell_names[i]);
		if (length != sizeof(u64)) {
			dev_err(dev, "QFPROM cell %s has %zu bytes, expected 8\n",
				msm8998_fuse_cell_names[i], length);
			kfree(buffer);
			return -EINVAL;
		}

		*values[i] = get_unaligned_le64(buffer);
		kfree(buffer);
	}

	return 0;
}

static int msm8998_prepare_domain(struct msm8998_osm_domain *domain,
				  const struct msm8998_cpr_domain_config *config,
				  const struct msm8998_osm_lut_config *lut_config,
				  unsigned int lut_config_count,
				  const struct msm8998_cprh_controller_config *cprh_config,
				  const struct msm8998_osm_sequence_config *sequence_config,
				  const struct msm8998_cpr_cluster_fuse_data *fuse,
				  u8 force_highest_corner)
{
	unsigned int i;
	int ret;

	ret = msm8998_cpr_build_corners(config, fuse, domain->corners,
					ARRAY_SIZE(domain->corners));
	if (ret)
		return ret;
	domain->corner_count = config->corner_count;

	ret = msm8998_cpr_apply_partial_binning(force_highest_corner,
						 domain->corners,
						 domain->corner_count);
	if (ret)
		return ret;

	ret = msm8998_cpr_append_crossover_corners(config, domain->corners,
						  ARRAY_SIZE(domain->corners),
						  &domain->corner_count);
	if (ret)
		return ret;

	ret = msm8998_osm_resolve_lut(lut_config, lut_config_count,
					 domain->corners, domain->corner_count,
					 domain->lut, ARRAY_SIZE(domain->lut));
	if (ret)
		return ret;

	ret = msm8998_osm_build_hw_table_image(domain->lut, lut_config_count,
					       &domain->hw_table);
	if (ret)
		return ret;

	ret = msm8998_osm_build_opp_table(domain->lut, lut_config_count,
					  domain->opps, ARRAY_SIZE(domain->opps),
					  &domain->opp_count);
	if (ret)
		return ret;

	/* Both joan CPU domains use CPRH hardware closed-loop downstream. */
	ret = msm8998_cprh_build_register_image(cprh_config, domain->corners,
						 domain->corner_count, true,
						 &domain->cprh_image);
	if (ret)
		return ret;

	ret = msm8998_osm_build_tz_sequence_image(sequence_config, domain->lut,
						  lut_config_count, domain->corners,
						  domain->corner_count,
						  &domain->sequence);
	if (ret)
		return ret;

	ret = msm8998_osm_build_acd_image(&joan_acd_config,
					  &domain->acd_image);
	if (ret)
		return ret;

	for (i = 0; i < domain->opp_count; i++) {
		domain->frequency_table[i].driver_data =
			domain->opps[i].table_index;
		domain->frequency_table[i].frequency =
			domain->opps[i].frequency_hz / 1000;
	}
	domain->frequency_table[domain->opp_count].frequency =
		CPUFREQ_TABLE_END;

	cpumask_clear(&domain->cpus);
	for_each_possible_cpu(i) {
		u64 mpidr = cpu_logical_map(i);

		if (MPIDR_AFFINITY_LEVEL(mpidr, 1) == domain->cluster)
			cpumask_set_cpu(i, &domain->cpus);
	}
	if (cpumask_empty(&domain->cpus))
		return -ENODEV;

	return 0;
}

static int msm8998_configure_apcs(struct msm8998_osm_drv *drv)
{
	void __iomem *cmd = drv->apcs_common + MSM8998_APCS_OSM_CMD_RCG;
	u32 command;
	u32 config;
	unsigned int timeout;

	/* HMSS GPLL0 / 1.5 is the validated 200 MHz OSM clock source. */
	config = FIELD_PREP(MSM8998_APCS_RCG_DIV_MASK,
			   MSM8998_APCS_RCG_DIV_1P5) |
		 MSM8998_APCS_RCG_SRC_GPLL0;
	writel_relaxed(config, drv->apcs_common + MSM8998_APCS_OSM_CFG_RCG);
	command = readl_relaxed(cmd);
	command |= MSM8998_APCS_RCG_ROOT_EN | MSM8998_APCS_RCG_UPDATE;
	writel_relaxed(command, cmd);

	for (timeout = 0; timeout < 500; timeout++) {
		command = readl_relaxed(cmd);
		if (!(command & MSM8998_APCS_RCG_UPDATE))
			return command & MSM8998_APCS_RCG_ROOT_EN ? 0 : -EIO;
		udelay(1);
	}

	return -ETIMEDOUT;
}

static void msm8998_setup_cycle_counter(struct msm8998_osm_domain *domain)
{
	u32 ratio = MSM8998_OSM_CLOCK_HZ / MSM8998_XO_CLOCK_HZ;

	msm8998_osm_write(domain, MSM8998_OSM_CYCLE_COUNTER_CTRL,
			  BIT(0) | (ratio - 1) << 1 |
			  OSM_CYCLE_COUNTER_USE_XO_EDGE_EN);
}

static void msm8998_setup_cc_llm_policy(struct msm8998_osm_domain *domain)
{
	u32 timer = MSM8998_OSM_COUNT_NS(1000);
	u32 llm_timer = MSM8998_OSM_COUNT_NS(327675);

	/* Match downstream qcom,pc-override-index = <0 0>. */
	msm8998_osm_write(domain, MSM8998_OSM_CC_ZERO_BEHAV_CTRL, BIT(0));
	msm8998_osm_write(domain, MSM8998_OSM_SPM_CC_HYSTERESIS,
			  timer | timer << 16);
	msm8998_osm_write(domain, MSM8998_OSM_SPM_CC_DCVS_DISABLE, 0);
	/* Treat cores in retention as active, matching qcom,set-ret-inactive. */
	msm8998_osm_write(domain, MSM8998_OSM_SPM_CORE_RET_MAPPING, 0);
	msm8998_osm_write(domain, MSM8998_OSM_LLM_FREQ_HYSTERESIS,
			  llm_timer | llm_timer << 16);
	msm8998_osm_write(domain, MSM8998_OSM_LLM_VOLT_HYSTERESIS,
			  llm_timer | llm_timer << 16);
	msm8998_osm_write(domain, MSM8998_OSM_LLM_DCVS_DISABLE, 0);
}

static void msm8998_setup_fsms(struct msm8998_osm_domain *domain)
{
	u32 value;

	/* Boost FSM */
	value = msm8998_osm_read(domain, MSM8998_OSM_PDN_FSM_CTRL);
	msm8998_osm_write(domain, MSM8998_OSM_PDN_FSM_CTRL,
			  value | MSM8998_OSM_FSM_CC_BOOST);

	value = msm8998_osm_read(domain, MSM8998_OSM_CC_BOOST_TIMER0);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(10000)) |
		 FIELD_PREP(GENMASK(31, 16), MSM8998_OSM_COUNT_NS(5000));
	msm8998_osm_write(domain, MSM8998_OSM_CC_BOOST_TIMER0, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_CC_BOOST_TIMER1);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(10000)) |
		 FIELD_PREP(GENMASK(31, 16), MSM8998_OSM_COUNT_NS(10000));
	msm8998_osm_write(domain, MSM8998_OSM_CC_BOOST_TIMER1, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_CC_BOOST_TIMER2);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(1000));
	msm8998_osm_write(domain, MSM8998_OSM_CC_BOOST_TIMER2, value);

	/* Safe-frequency FSM */
	value = msm8998_osm_read(domain, MSM8998_OSM_PDN_FSM_CTRL);
	msm8998_osm_write(domain, MSM8998_OSM_PDN_FSM_CTRL,
			  value | MSM8998_OSM_FSM_DCVS_BOOST);

	value = msm8998_osm_read(domain, MSM8998_OSM_DCVS_BOOST_TIMER0);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(10000)) |
		 FIELD_PREP(GENMASK(31, 16), MSM8998_OSM_COUNT_NS(5000));
	msm8998_osm_write(domain, MSM8998_OSM_DCVS_BOOST_TIMER0, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_DCVS_BOOST_TIMER1);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(10000)) |
		 FIELD_PREP(GENMASK(31, 16), MSM8998_OSM_COUNT_NS(10000));
	msm8998_osm_write(domain, MSM8998_OSM_DCVS_BOOST_TIMER1, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_DCVS_BOOST_TIMER2);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(1000));
	msm8998_osm_write(domain, MSM8998_OSM_DCVS_BOOST_TIMER2, value);

	/* Power-save FSM */
	value = msm8998_osm_read(domain, MSM8998_OSM_PDN_FSM_CTRL);
	msm8998_osm_write(domain, MSM8998_OSM_PDN_FSM_CTRL,
			  value | MSM8998_OSM_FSM_PS_BOOST);

	value = msm8998_osm_read(domain, MSM8998_OSM_PS_BOOST_TIMER0);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(10000)) |
		 FIELD_PREP(GENMASK(31, 16), MSM8998_OSM_COUNT_NS(5000));
	msm8998_osm_write(domain, MSM8998_OSM_PS_BOOST_TIMER0, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_PS_BOOST_TIMER1);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(10000)) |
		 FIELD_PREP(GENMASK(31, 16), MSM8998_OSM_COUNT_NS(10000));
	msm8998_osm_write(domain, MSM8998_OSM_PS_BOOST_TIMER1, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_PS_BOOST_TIMER2);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(1000));
	msm8998_osm_write(domain, MSM8998_OSM_PS_BOOST_TIMER2, value);

	msm8998_osm_write(domain, MSM8998_OSM_BOOST_SYNC_DELAY, 5);

	/* WFx droop FSM */
	value = msm8998_osm_read(domain, MSM8998_OSM_PDN_FSM_CTRL);
	msm8998_osm_write(domain, MSM8998_OSM_PDN_FSM_CTRL,
			  value | MSM8998_OSM_FSM_WFX_DROOP);
	value = msm8998_osm_read(domain, MSM8998_OSM_DROOP_UNSTALL_TIMER);
	value |= FIELD_PREP(GENMASK(31, 16), MSM8998_OSM_COUNT_NS(500));
	msm8998_osm_write(domain, MSM8998_OSM_DROOP_UNSTALL_TIMER, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_DROOP_WAIT_TIMER0);
	value |= FIELD_PREP(GENMASK(31, 16), MSM8998_OSM_COUNT_NS(250));
	msm8998_osm_write(domain, MSM8998_OSM_DROOP_WAIT_TIMER0, value);

	/* Power-collapse/retention droop FSM */
	value = msm8998_osm_read(domain, MSM8998_OSM_PDN_FSM_CTRL);
	msm8998_osm_write(domain, MSM8998_OSM_PDN_FSM_CTRL,
			  value | MSM8998_OSM_FSM_PC_RET_DROOP);
	value = msm8998_osm_read(domain, MSM8998_OSM_DROOP_UNSTALL_TIMER);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(500));
	msm8998_osm_write(domain, MSM8998_OSM_DROOP_UNSTALL_TIMER, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_DROOP_WAIT_TIMER0);
	value |= FIELD_PREP(GENMASK(15, 0), MSM8998_OSM_COUNT_NS(250));
	msm8998_osm_write(domain, MSM8998_OSM_DROOP_WAIT_TIMER0, value);

	/* DCVS droop FSM */
	value = msm8998_osm_read(domain, MSM8998_OSM_PDN_FSM_CTRL);
	msm8998_osm_write(domain, MSM8998_OSM_PDN_FSM_CTRL,
			  value | MSM8998_OSM_FSM_DCVS_DROOP);

	msm8998_osm_write(domain, MSM8998_OSM_DROOP_SYNC_DELAY, 1);
	msm8998_osm_write(domain, MSM8998_OSM_DROOP_RELEASE_TIMER,
			  MSM8998_OSM_COUNT_NS(5));
	msm8998_osm_write(domain, MSM8998_OSM_DCVS_DROOP_TIMER,
			  MSM8998_OSM_COUNT_NS(500));
	value = msm8998_osm_read(domain, MSM8998_OSM_DROOP_CTRL);
	value |= BIT(31) | FIELD_PREP(GENMASK(22, 16), 2) |
		 FIELD_PREP(GENMASK(6, 0), 8);
	msm8998_osm_write(domain, MSM8998_OSM_DROOP_CTRL, value);
	value = msm8998_osm_read(domain, MSM8998_OSM_PLL_SW_OVERRIDE);
	msm8998_osm_write(domain, MSM8998_OSM_PLL_SW_OVERRIDE, value | BIT(0));
}

static void msm8998_setup_osm_table(struct msm8998_osm_domain *domain)
{
	unsigned int i;

	for (i = 0; i < MSM8998_OSM_TABLE_SIZE; i++) {
		const struct msm8998_osm_hw_table_entry *entry =
			&domain->hw_table.entry[i];
		u32 offset = i * MSM8998_OSM_TABLE_STRIDE;

		msm8998_osm_write(domain, MSM8998_OSM_INDEX + offset,
				  entry->index);
		msm8998_osm_write(domain, MSM8998_OSM_FREQUENCY + offset,
				  entry->frequency_data);
		msm8998_osm_write(domain, MSM8998_OSM_VOLTAGE + offset,
				  entry->voltage_data);
		msm8998_osm_write(domain, MSM8998_OSM_OVERRIDE + offset,
				  entry->override_data);
		msm8998_osm_write(domain, MSM8998_OSM_SPARE + offset,
				  entry->spare_data);
	}

	/* Flush every posted LUT write before programming either domain further. */
	msm8998_osm_read(domain, MSM8998_OSM_VERSION);
}

static int msm8998_snapshot_saw(struct msm8998_osm_drv *drv,
				struct msm8998_saw_snapshot *state,
				const char *stage)
{
	bool versions_ok = true;
	unsigned int i;

	/* Validate both register layouts before reading either v4.1 payload. */
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		state[i].version = readl(drv->domain[i].saw + MSM8998_SAW_VERSION);
		if (((state[i].version >> 28) & 0xf) != 4 ||
		    ((state[i].version >> 16) & 0xfff) != 1)
			versions_ok = false;
	}
	if (!versions_ok) {
		for (i = 0; i < ARRAY_SIZE(drv->domain); i++)
			dev_err(drv->dev, "SAW %s cluster %u: version=%08x, expected 4.1\n",
				stage, i, state[i].version);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		void __iomem *saw = drv->domain[i].saw;

		state[i].id = readl(saw + MSM8998_SAW_ID);
		state[i].vctl = readl(saw + MSM8998_SAW_VCTL);
		state[i].avs_ctl = readl(saw + MSM8998_SAW_AVS_CTL);
		state[i].avs_limit = readl(saw + MSM8998_SAW_AVS_LIMIT);
		state[i].pmic_sts = readl(saw + MSM8998_SAW_PMIC_STS);
	}
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++)
		dev_info(drv->dev,
			 "SAW %s cluster %u: version=%08x id=%08x vctl=%08x avs_ctl=%08x avs_limit=%08x pmic_sts=%08x\n",
			 stage, i, state[i].version, state[i].id, state[i].vctl,
			 state[i].avs_ctl, state[i].avs_limit, state[i].pmic_sts);

	return 0;
}

static int msm8998_setup_saw(struct msm8998_osm_drv *drv)
{
	struct msm8998_saw_snapshot before[MSM8998_CPR_CLUSTER_COUNT];
	struct msm8998_saw_snapshot after[MSM8998_CPR_CLUSTER_COUNT];
	unsigned int i;
	int ret;

	ret = msm8998_snapshot_saw(drv, before, "pre-init");
	if (ret)
		return ret;

	/* No cluster may be active when either SAW is initialized. */
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		struct msm8998_osm_domain *domain = &drv->domain[i];
		u32 enable = msm8998_osm_read(domain, MSM8998_OSM_ENABLE);
		u32 cpr_ctl = msm8998_cpr_read(domain, MSM8998_CPR_CTL);
		u32 cprh_ctl = msm8998_cpr_read(domain, MSM8998_CPRH_CTL);

		dev_info(drv->dev,
			 "SAW pre-init cluster %u: osm_enable=%08x cpr_ctl=%08x cprh_ctl=%08x\n",
			 i, enable, cpr_ctl, cprh_ctl);
		if ((enable & BIT(0)) || (cpr_ctl & MSM8998_CPR_CTL_LOOP_EN) ||
		    (cprh_ctl & MSM8998_CPRH_CTL_OSM_ENABLED))
			return dev_err_probe(drv->dev, -EBUSY,
				"refusing SAW initialization with cluster %u active\n", i);
		if (!(before[i].id & MSM8998_SAW_ID_PMIC_ARB_PRESENT))
			return dev_err_probe(drv->dev, -ENODEV,
				"cluster %u SAW has no PMIC arbiter\n", i);
		if (before[i].vctl || before[i].pmic_sts)
			return dev_err_probe(drv->dev, -EBUSY,
				"cluster %u SAW is not in the audited idle state\n", i);
		if (before[i].avs_ctl == JOAN_SAW_AVS_CTL &&
		    before[i].avs_limit == joan_saw_configs[i].avs_limit)
			continue;
		if (before[i].avs_ctl ||
		    before[i].avs_limit != JOAN_SAW_BOOT_AVS_LIMIT)
			return dev_err_probe(drv->dev, -EBUSY,
				"cluster %u SAW has an unknown AVS configuration\n", i);
	}

	/* Match downstream's AVS_CTL -> barrier -> AVS_LIMIT -> barrier order. */
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		void __iomem *saw = drv->domain[i].saw;

		if (before[i].avs_ctl == JOAN_SAW_AVS_CTL &&
		    before[i].avs_limit == joan_saw_configs[i].avs_limit)
			continue;
		writel_relaxed(JOAN_SAW_AVS_CTL, saw + MSM8998_SAW_AVS_CTL);
		/* Complete AVS control programming before setting its limits. */
		mb();
		writel_relaxed(joan_saw_configs[i].avs_limit,
			       saw + MSM8998_SAW_AVS_LIMIT);
		/* Make both AVS settings visible before initializing CPRH. */
		mb();
	}

	ret = msm8998_snapshot_saw(drv, after, "post-init");
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		if (after[i].version != before[i].version ||
		    after[i].id != before[i].id)
			return dev_err_probe(drv->dev, -EIO,
				"cluster %u SAW identity changed during initialization\n", i);
		if (after[i].avs_ctl != JOAN_SAW_AVS_CTL ||
		    after[i].avs_limit != joan_saw_configs[i].avs_limit)
			return dev_err_probe(drv->dev, -EIO,
				"cluster %u SAW AVS initialization readback mismatch\n", i);
	}
	drv->saw_ready = true;

	return 0;
}

static int msm8998_setup_cprh(struct msm8998_osm_domain *domain)
{
	const struct msm8998_cprh_register_image *image = &domain->cprh_image;
	unsigned int i;

	for (i = 0; i < MSM8998_CPR_RING_OSC_COUNT; i++)
		if (image->gcnt_ro_mask & BIT(i))
			msm8998_cpr_write(domain, MSM8998_CPR_GCNT(i), image->gcnt);

	msm8998_cpr_write(domain, MSM8998_CPR_TIMER_AUTO_CONT,
			  image->cpr_timer_auto_cont);
	msm8998_cpr_write(domain, MSM8998_CPR_CTL,
			  image->cpr_ctl & ~MSM8998_CPR_CTL_LOOP_EN);
	msm8998_cpr_write(domain, MSM8998_CPR_STEP_QUOT, image->cpr_step_quot);
	for (i = 0; i < image->sensor_owner_count; i++)
		msm8998_cpr_write(domain, MSM8998_CPR_SENSOR_OWNER(i), 0);
	msm8998_cpr_write(domain, MSM8998_CPR_THRESHOLD, image->threshold);
	msm8998_cpr_write(domain, MSM8998_CPR_RO_MASK, image->ro_mask);
	for (i = 0; i < MSM8998_CPR_RING_OSC_COUNT; i++)
		if (image->gcnt_ro_mask & BIT(i))
			msm8998_cpr_write(domain, MSM8998_CPR_TARGET_QUOT(i),
					  image->target_quot[i]);
	for (i = 0; i < image->corner_count; i++)
		msm8998_cpr_write(domain, MSM8998_CPR_CORNER(i), image->corner[i]);
	msm8998_cpr_write(domain, MSM8998_CPR_SAW_ERROR_STEP,
			  image->saw_error_step_limit);
	msm8998_cpr_write(domain, MSM8998_CPR_MARGIN_TIMERS,
			  image->margin_temp_core_timers);
	msm8998_cpr_write(domain, MSM8998_CPR_MARGIN_ADJ_CTL,
			  image->margin_adjust_ctl);
	msm8998_cpr_write(domain, MSM8998_CPRH_CTL, image->cprh_ctl);

	/* Start sensing only after every CPRH corner is visible. */
	wmb();
	msm8998_cpr_write(domain, MSM8998_CPR_CTL,
			  image->cpr_ctl | MSM8998_CPR_CTL_LOOP_EN);
	if (!(msm8998_cpr_read(domain, MSM8998_CPR_CTL) &
	      MSM8998_CPR_CTL_LOOP_EN))
		return -EIO;

	return 0;
}

static int msm8998_acd_wait(struct msm8998_osm_domain *domain,
				    u32 offset, u32 mask, unsigned int timeout_us)
{
	unsigned int i;

	for (i = 0; i < timeout_us; i++) {
		if (readl_relaxed(domain->acd + offset) & mask)
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

static int msm8998_acd_local_write(struct msm8998_osm_domain *domain,
				   u32 offset)
{
	u32 command;

	if (offset >= 0x80 || offset & 0x3)
		return -EINVAL;

	writel_relaxed(0, domain->acd + MSM8998_ACD_WRITE_CTL);
	command = (offset / 4) << 1 | BIT(0);
	writel_relaxed(command, domain->acd + MSM8998_ACD_WRITE_CTL);
	/* Match downstream: flush the posted local-transfer request via VERSION. */
	readl_relaxed(domain->acd + MSM8998_ACD_VERSION);
	return msm8998_acd_wait(domain, MSM8998_ACD_WRITE_STATUS,
				BIT(offset / 4), 1000);
}

static int msm8998_setup_acd(struct msm8998_osm_domain *domain)
{
	unsigned int i;

	for (i = 0; i < domain->acd_image.operation_count; i++) {
		const struct msm8998_osm_acd_operation *operation =
			&domain->acd_image.operation[i];

		switch (operation->type) {
		case MSM8998_OSM_ACD_MASTER_WRITE:
			writel_relaxed(operation->value,
				       domain->acd + operation->offset);
			break;
		case MSM8998_OSM_ACD_AUTO_TRANSFER:
			writel_relaxed(operation->value,
				       domain->acd + MSM8998_ACD_AUTO_TRANSFER_CONFIG);
			writel_relaxed(0, domain->acd + MSM8998_ACD_AUTO_TRANSFER);
			writel_relaxed(BIT(0), domain->acd + MSM8998_ACD_AUTO_TRANSFER);
			readl_relaxed(domain->acd + MSM8998_ACD_VERSION);
			if (msm8998_acd_wait(domain,
					     MSM8998_ACD_AUTO_TRANSFER_STATUS,
					     BIT(0), 1000))
				return -ETIMEDOUT;
			break;
		case MSM8998_OSM_ACD_WRITE_THROUGH:
			writel_relaxed(operation->value,
				       domain->acd + operation->offset);
			readl_relaxed(domain->acd + MSM8998_ACD_VERSION);
			if (msm8998_acd_local_write(domain, operation->offset))
				return -ETIMEDOUT;
			break;
		case MSM8998_OSM_ACD_DELAY_US:
			udelay(operation->value);
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static int msm8998_setup_sequence(struct msm8998_osm_domain *domain)
{
	unsigned int i;

	for (i = 0; i < domain->sequence.write_count; i++) {
		const struct msm8998_osm_register_write *write =
			&domain->sequence.write[i];
		int ret;

		switch (write->access) {
		case MSM8998_OSM_ACCESS_MMIO:
			msm8998_osm_write(domain, write->offset, write->value);
			break;
		case MSM8998_OSM_ACCESS_SCM:
			ret = qcom_scm_io_writel(domain->osm_phys + write->offset,
						write->value);
			if (ret)
				return ret;
			break;
		default:
			return -EINVAL;
		}
	}

	/* Flush all non-secure sequence writes before selecting a LUT row. */
	msm8998_osm_read(domain, MSM8998_OSM_VERSION);
	return 0;
}

static void msm8998_request_table_index(struct msm8998_osm_domain *domain,
					u32 table_index)
{
	/* MSM8998 v2 deletes qcom,llm-sw-overr from the downstream DT. */
	msm8998_osm_write(domain, MSM8998_OSM_DESIRED, table_index);
	msm8998_osm_read(domain, MSM8998_OSM_VERSION);
}

static int msm8998_set_frequency(struct msm8998_osm_domain *domain,
				 u32 frequency_hz)
{
	unsigned int i;

	for (i = 0; i < domain->opp_count; i++) {
		if (domain->opps[i].frequency_hz != frequency_hz)
			continue;

		msm8998_request_table_index(domain, domain->opps[i].table_index);
		return 0;
	}

	return -EINVAL;
}

static int msm8998_program_domains(struct msm8998_osm_drv *drv)
{
	unsigned int i;
	int ret;

	if (!drv->saw_ready)
		return dev_err_probe(drv->dev, -EIO,
				     "SAW initialization must complete before CPRH setup\n");

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		ret = msm8998_setup_cprh(&drv->domain[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++)
		msm8998_setup_cycle_counter(&drv->domain[i]);

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++)
		msm8998_setup_osm_table(&drv->domain[i]);

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++)
		msm8998_setup_cc_llm_policy(&drv->domain[i]);

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++)
		msm8998_setup_fsms(&drv->domain[i]);

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		ret = msm8998_setup_sequence(&drv->domain[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static void msm8998_disable_domains(struct msm8998_osm_drv *drv)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		msm8998_osm_write(&drv->domain[i], MSM8998_OSM_ENABLE, 0);
		drv->domain[i].enabled = false;
	}
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++)
		msm8998_osm_read(&drv->domain[i], MSM8998_OSM_VERSION);
}

static int msm8998_enable_domains(struct msm8998_osm_drv *drv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		struct msm8998_osm_domain *domain = &drv->domain[i];

		ret = msm8998_setup_acd(domain);
		if (ret) {
			dev_err(drv->dev, "cluster %u ACD setup failed: %d\n",
				i, ret);
			return ret;
		}

		udelay(5);
		msm8998_osm_write(domain, MSM8998_OSM_ENABLE, 1);
		msm8998_osm_read(domain, MSM8998_OSM_VERSION);
		udelay(50);

		/* These warning readbacks do not establish OSM readiness. */
		if (msm8998_osm_read(domain, MSM8998_OSM_ENABLE) != 1) {
			dev_warn(drv->dev,
				 "OSM post-enable cluster %u: ENABLE readback failed\n",
				 i);
		}
		if (msm8998_osm_read(domain, MSM8998_OSM_VERSION) == 0) {
			dev_warn(drv->dev,
				 "OSM post-enable cluster %u: VERSION is zero\n",
				 i);
		}
	}
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		if (!(msm8998_osm_read(&drv->domain[i], MSM8998_OSM_ENABLE) &
		      BIT(0))) {
			dev_err(drv->dev, "cluster %u OSM failed to enable\n", i);
			msm8998_disable_domains(drv);
			return -EIO;
		}
		drv->domain[i].enabled = true;
	}

	return 0;
}

static int msm8998_set_boot_rates(struct msm8998_osm_drv *drv)
{
	static const u32 boot_rates[] = {
		MSM8998_POWER_BOOT_HZ,
		MSM8998_PERFORMANCE_BOOT_HZ,
	};
	unsigned int i;
	int ret;

	/* The v2 downstream driver changes from 300 MHz to these rates last. */
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		ret = msm8998_set_frequency(&drv->domain[i], boot_rates[i]);
		if (ret)
			return dev_err_probe(drv->dev, ret,
					     "missing v2 boot rate %u for cluster %u\n",
					     boot_rates[i], i);
	}

	return 0;
}

static struct msm8998_osm_domain *msm8998_domain_for_cpu(unsigned int cpu)
{
	struct msm8998_osm_drv *drv = READ_ONCE(msm8998_osm);
	unsigned int i;

	if (!drv || cpu >= nr_cpu_ids)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++)
		if (cpumask_test_cpu(cpu, &drv->domain[i].cpus))
			return &drv->domain[i];

	return NULL;
}

static int msm8998_cpufreq_target_index(struct cpufreq_policy *policy,
					unsigned int index)
{
	struct msm8998_osm_domain *domain = policy->driver_data;
	u32 table_index;

	if (!domain || !domain->enabled || index >= domain->opp_count)
		return -EINVAL;

	table_index = policy->freq_table[index].driver_data;
	if (table_index >= MSM8998_OSM_TABLE_SIZE)
		return -EINVAL;

	msm8998_request_table_index(domain, table_index);
	return 0;
}

static unsigned int msm8998_cpufreq_get(unsigned int cpu)
{
	struct msm8998_osm_domain *domain = msm8998_domain_for_cpu(cpu);
	u32 table_index;

	if (!domain || !domain->enabled)
		return 0;

	table_index = msm8998_osm_read(domain, MSM8998_OSM_DESIRED);
	if (table_index >= MSM8998_OSM_TABLE_SIZE)
		return 0;

	return domain->lut[table_index].config.frequency_hz / 1000;
}

static int msm8998_cpufreq_init(struct cpufreq_policy *policy)
{
	struct msm8998_osm_domain *domain =
		msm8998_domain_for_cpu(policy->cpu);

	if (!domain || !domain->enabled)
		return -ENODEV;

	policy->driver_data = domain;
	policy->freq_table = domain->frequency_table;
	policy->cpuinfo.transition_latency =
		MSM8998_OSM_TRANSITION_LATENCY_NS;
	cpumask_copy(policy->cpus, &domain->cpus);
	return 0;
}

static void msm8998_cpufreq_exit(struct cpufreq_policy *policy)
{
	policy->driver_data = NULL;
}

static struct cpufreq_driver msm8998_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = msm8998_cpufreq_target_index,
	.get = msm8998_cpufreq_get,
	.init = msm8998_cpufreq_init,
	.exit = msm8998_cpufreq_exit,
	.name = "qcom-msm8998-osm",
};

static int msm8998_map_resources(struct platform_device *pdev,
				 struct msm8998_osm_drv *drv)
{
	static const char * const cprh_names[] = {
		"power-cprh", "performance-cprh",
	};
	static const char * const acd_names[] = {
		"power-acd", "performance-acd",
	};
	struct device *dev = &pdev->dev;
	struct resource *resource;
	unsigned int i;

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "osm");
	if (!resource)
		return dev_err_probe(dev, -EINVAL,
				     "missing OSM register resource\n");
	if (resource_size(resource) <
	    MSM8998_OSM_DOMAIN_STRIDE * MSM8998_CPR_CLUSTER_COUNT)
		return dev_err_probe(dev, -EINVAL,
				     "OSM register resource is too small\n");

	drv->osm_base = devm_ioremap_resource(dev, resource);
	if (IS_ERR(drv->osm_base))
		return PTR_ERR(drv->osm_base);
	drv->osm_phys = resource->start;

	drv->apcs_common =
		devm_platform_ioremap_resource_byname(pdev, "apcs-common");
	if (IS_ERR(drv->apcs_common))
		return PTR_ERR(drv->apcs_common);

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		struct msm8998_osm_domain *domain = &drv->domain[i];

		domain->osm = drv->osm_base + i * MSM8998_OSM_DOMAIN_STRIDE;
		domain->osm_phys = drv->osm_phys +
			i * MSM8998_OSM_DOMAIN_STRIDE;
		domain->cprh = devm_platform_ioremap_resource_byname(pdev,
							       cprh_names[i]);
		if (IS_ERR(domain->cprh))
			return PTR_ERR(domain->cprh);
		domain->acd = devm_platform_ioremap_resource_byname(pdev,
							      acd_names[i]);
		if (IS_ERR(domain->acd))
			return PTR_ERR(domain->acd);
		resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						       joan_saw_configs[i].resource_name);
		if (!resource || resource->start != joan_saw_configs[i].phys ||
		    resource_size(resource) != MSM8998_SAW_SIZE)
			return dev_err_probe(dev, -EINVAL,
				"invalid %s register resource\n",
				joan_saw_configs[i].resource_name);
		domain->saw = devm_ioremap_resource(dev, resource);
		if (IS_ERR(domain->saw))
			return PTR_ERR(domain->saw);
		domain->cluster = i;
	}

	return 0;
}

static int msm8998_prepare_domains(struct msm8998_osm_drv *drv)
{
	struct msm8998_cpr_fuse_rows rows;
	struct msm8998_cpr_fuses fuses;
	struct msm8998_cpr_fuse_data fuse_data;
	unsigned int i;
	int ret;

	ret = msm8998_read_fuse_rows(drv->dev, &rows);
	if (ret)
		return ret;
	ret = msm8998_cpr_decode_fuses(&rows, &fuses);
	if (ret)
		return dev_err_probe(drv->dev, ret,
				     "failed to decode CPR fuses\n");
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &fuse_data);
	if (ret)
		return dev_err_probe(drv->dev, ret,
				     "failed to calculate CPR fuse data\n");

	if (fuse_data.speed_bin != 2 || fuse_data.revision != 3)
		return dev_err_probe(drv->dev, -ENODEV,
			"unsupported speed bin %u CPR revision %u\n",
			fuse_data.speed_bin, fuse_data.revision);

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		ret = msm8998_prepare_domain(&drv->domain[i],
				msm8998_domain_configs[i], msm8998_domain_luts[i],
				msm8998_domain_lut_counts[i],
				msm8998_cprh_configs[i],
				msm8998_sequence_configs[i],
				&fuse_data.cluster[i],
				fuse_data.force_highest_corner);
		if (ret)
			return dev_err_probe(drv->dev, ret,
					     "failed to prepare cluster %u\n", i);
	}

	dev_info(drv->dev,
		 "speed bin %u CPR revision %u fuse combo %u%s\n",
		 fuse_data.speed_bin, fuse_data.revision, fuse_data.fuse_combo,
		 fuse_data.force_highest_corner ? " partial binning" : "");
	return 0;
}

static int msm8998_prepare_clocks(struct msm8998_osm_drv *drv)
{
	unsigned long rate;
	int ret;

	drv->cpr_clk = devm_clk_get(drv->dev, "cpr");
	if (IS_ERR(drv->cpr_clk))
		return dev_err_probe(drv->dev, PTR_ERR(drv->cpr_clk),
				     "failed to get CPR clock\n");
	drv->aux_clk = devm_clk_get(drv->dev, "aux");
	if (IS_ERR(drv->aux_clk))
		return dev_err_probe(drv->dev, PTR_ERR(drv->aux_clk),
				     "failed to get auxiliary clock\n");
	drv->xo_clk = devm_clk_get(drv->dev, "xo");
	if (IS_ERR(drv->xo_clk))
		return dev_err_probe(drv->dev, PTR_ERR(drv->xo_clk),
				     "failed to get XO clock\n");

	ret = clk_set_rate(drv->cpr_clk, MSM8998_XO_CLOCK_HZ);
	if (ret)
		return dev_err_probe(drv->dev, ret,
				     "failed to set CPR clock rate\n");
	ret = clk_set_rate(drv->aux_clk, MSM8998_AUX_CLOCK_HZ);
	if (ret)
		return dev_err_probe(drv->dev, ret,
				     "failed to set auxiliary clock rate\n");

	rate = clk_get_rate(drv->cpr_clk);
	if (rate != MSM8998_XO_CLOCK_HZ)
		return dev_err_probe(drv->dev, -EINVAL,
			"CPR clock rate is %lu, expected %u\n", rate,
			MSM8998_XO_CLOCK_HZ);
	rate = clk_get_rate(drv->aux_clk);
	if (rate != MSM8998_AUX_CLOCK_HZ)
		return dev_err_probe(drv->dev, -EINVAL,
			"auxiliary clock rate is %lu, expected %u\n", rate,
			MSM8998_AUX_CLOCK_HZ);
	rate = clk_get_rate(drv->xo_clk);
	if (rate != MSM8998_XO_CLOCK_HZ)
		return dev_err_probe(drv->dev, -EINVAL,
			"XO clock rate is %lu, expected %u\n", rate,
			MSM8998_XO_CLOCK_HZ);

	return 0;
}

static int msm8998_enable_clocks(struct msm8998_osm_drv *drv)
{
	int ret;

	ret = clk_prepare_enable(drv->cpr_clk);
	if (ret)
		return dev_err_probe(drv->dev, ret,
				     "failed to enable CPR clock\n");
	ret = clk_prepare_enable(drv->aux_clk);
	if (ret) {
		clk_disable_unprepare(drv->cpr_clk);
		return dev_err_probe(drv->dev, ret,
				     "failed to enable auxiliary clock\n");
	}
	ret = clk_prepare_enable(drv->xo_clk);
	if (ret) {
		clk_disable_unprepare(drv->aux_clk);
		clk_disable_unprepare(drv->cpr_clk);
		return dev_err_probe(drv->dev, ret,
				     "failed to enable XO clock\n");
	}
	drv->clocks_enabled = true;

	return 0;
}

static void msm8998_disable_clocks(struct msm8998_osm_drv *drv)
{
	if (!drv->clocks_enabled)
		return;

	clk_disable_unprepare(drv->xo_clk);
	clk_disable_unprepare(drv->aux_clk);
	clk_disable_unprepare(drv->cpr_clk);
	drv->clocks_enabled = false;
}

static void msm8998_stop_cprh(struct msm8998_osm_drv *drv)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		struct msm8998_osm_domain *domain = &drv->domain[i];

		msm8998_cpr_write(domain, MSM8998_CPR_CTL,
				   domain->cprh_image.cpr_ctl & ~BIT(0));
		msm8998_cpr_read(domain, MSM8998_CPR_CTL);
	}
}

static int msm8998_validate_board(struct device *dev)
{
	struct device_node *root;
	u32 msm_id[2];
	int ret;

	if (!of_machine_is_compatible("lg,joan") ||
	    !of_machine_is_compatible("qcom,msm8998"))
		return dev_err_probe(dev, -ENODEV,
				     "SAW initialization is restricted to LG Joan\n");

	/* This existing root property selects Joan's MSM8998 v2.1 DTB. */
	root = of_find_node_by_path("/");
	ret = of_property_read_u32_array(root, "qcom,msm-id", msm_id,
					 ARRAY_SIZE(msm_id));
	of_node_put(root);
	if (ret)
		return dev_err_probe(dev, ret, "missing qcom,msm-id board data\n");
	if (msm_id[0] != 292 || msm_id[1] != 0x20001)
		return dev_err_probe(dev, -ENODEV,
				     "SAW initialization requires the MSM8998 v2.1 DTB\n");

	return 0;
}

static int msm8998_osm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msm8998_osm_drv *drv;
	unsigned int i;
	int ret;

	ret = msm8998_validate_board(dev);
	if (ret)
		return ret;
	if (READ_ONCE(msm8998_osm))
		return dev_err_probe(dev, -EBUSY,
				     "an MSM8998 OSM instance is already active\n");

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;
	drv->dev = dev;
	ret = msm8998_map_resources(pdev, drv);
	if (ret)
		return ret;

	ret = msm8998_prepare_domains(drv);
	if (ret)
		return ret;
	ret = msm8998_prepare_clocks(drv);
	if (ret)
		return ret;
	ret = msm8998_enable_clocks(drv);
	if (ret)
		goto disable_clocks;
	ret = msm8998_setup_saw(drv);
	if (ret)
		goto disable_clocks;
	ret = msm8998_program_domains(drv);
	if (ret) {
		dev_err(dev, "failed to program OSM domains: %d\n", ret);
		goto stop_cprh;
	}
	ret = msm8998_configure_apcs(drv);
	if (ret) {
		dev_err(dev, "failed to configure the 200 MHz OSM clock: %d\n",
			ret);
		goto stop_cprh;
	}
	/* Downstream sends the initial LLM pulse after the OSM source is running. */
	for (i = 0; i < ARRAY_SIZE(drv->domain); i++) {
		ret = msm8998_set_frequency(&drv->domain[i], MSM8998_OSM_INITIAL_HZ);
		if (ret) {
			dev_err(dev,
				"missing initial OSM rate %u for cluster %u\n",
				MSM8998_OSM_INITIAL_HZ, i);
			goto stop_cprh;
		}
	}
	ret = msm8998_enable_domains(drv);
	if (ret)
		goto stop_cprh;
	ret = msm8998_set_boot_rates(drv);
	if (ret)
		goto stop_cprh;
	platform_set_drvdata(pdev, drv);
	WRITE_ONCE(msm8998_osm, drv);
	ret = cpufreq_register_driver(&msm8998_cpufreq_driver);
	if (ret) {
		/* OSM now clocks the CPUs, so leave its dependencies enabled. */
		dev_crit(dev,
			 "OSM is running at v2 boot rates but CPUFreq registration failed: %d\n",
			 ret);
		return 0;
	}

	dev_info(dev,
		 "OSM CPUFreq ready: power %u-%u MHz, performance %u-%u MHz\n",
		 drv->domain[0].opps[0].frequency_hz / 1000000,
		 drv->domain[0].opps[drv->domain[0].opp_count - 1].frequency_hz /
			1000000,
		 drv->domain[1].opps[0].frequency_hz / 1000000,
		 drv->domain[1].opps[drv->domain[1].opp_count - 1].frequency_hz /
			1000000);
	return 0;

stop_cprh:
	msm8998_disable_domains(drv);
	msm8998_stop_cprh(drv);
disable_clocks:
	msm8998_disable_clocks(drv);
	return ret;
}

static const struct of_device_id msm8998_osm_match[] = {
	{ .compatible = "lg,joan-osm-cpufreq" },
	{ }
};

static struct platform_driver msm8998_osm_driver = {
	.probe = msm8998_osm_probe,
	.driver = {
		.name = "qcom-msm8998-osm-cpufreq",
		.of_match_table = msm8998_osm_match,
		.suppress_bind_attrs = true,
	},
};

static int __init msm8998_osm_init(void)
{
	return platform_driver_register(&msm8998_osm_driver);
}
arch_initcall(msm8998_osm_init);
