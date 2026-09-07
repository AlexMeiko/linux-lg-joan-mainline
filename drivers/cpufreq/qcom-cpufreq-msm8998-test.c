// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>

#include "qcom-cpufreq-msm8998.h"

static const struct msm8998_cpr_fuse_rows msm8998_cpr_known_rows = {
	.row38 = 0x984889005002005e,
	.row39 = 0x801a03f29e2c79ae,
	.row67 = 0xa13800a6a9b3bbbb,
	.row68 = 0x3c62000008acae4f,
	.row69 = 0x302bae7aaeeec00e,
	.row70 = 0x0000044544558375,
	.row71 = 0x0002100000140fee,
	.row100 = 0x0000000000000000,
};

#define MSM8998_CORNER(_frequency, _floor, _ceiling, _open_adj, _closed_adj, \
		       _range)                                                \
	{                                                                  \
		.frequency_hz = (_frequency),                              \
		.floor_uv = (_floor),                                      \
		.ceiling_uv = (_ceiling),                                  \
		.open_loop_adjust_uv = (_open_adj),                        \
		.closed_loop_adjust_uv = (_closed_adj),                    \
		.max_floor_to_ceiling_uv = (_range),                       \
	}

static const struct msm8998_cpr_corner_config msm8998_power_corners[] = {
	MSM8998_CORNER(300000000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(364800000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(441600000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(518400000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(595200000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(672000000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(748800000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(825600000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(883200000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(960000000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(1036800000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(1094400000, 632000, 900000, -12000, -10000, 32000),
	MSM8998_CORNER(1171200000, 632000, 900000, -12000, -11000, 32000),
	MSM8998_CORNER(1248000000, 632000, 900000, -12000, -12000, 32000),
	MSM8998_CORNER(1324800000, 632000, 900000, -12000, -13000, 32000),
	MSM8998_CORNER(1401600000, 632000, 900000, -12000, -14000, 32000),
	MSM8998_CORNER(1478400000, 632000, 900000, -16000, -14000, 32000),
	MSM8998_CORNER(1555200000, 632000, 900000, -16000, -15000, 32000),
	MSM8998_CORNER(1670400000, 712000, 952000, -20000, -21000, 40000),
	MSM8998_CORNER(1747200000, 712000, 952000, -24000, -24000, 40000),
	MSM8998_CORNER(1824000000, 772000, 1056000, -28000, -26000, 40000),
	MSM8998_CORNER(1900800000, 772000, 1056000, -28000, -28000, 40000),
};

static const u32 msm8998_power_ro_scaling_factor[] = {
	0xa23, 0xaea, 0xa11, 0xaca, 0x9a7, 0xa72, 0x897, 0x9f9,
	0xc75, 0xcb7, 0xc78, 0xb92, 0xbee, 0xba6, 0x7fa, 0xb81,
	0xa23, 0xaea, 0xa11, 0xaca, 0x9a7, 0xa72, 0x897, 0x9f9,
	0xc75, 0xcb7, 0xc78, 0xb92, 0xbee, 0xba6, 0x7fa, 0xb81,
	0x957, 0x9f6, 0x9b3, 0xa4e, 0x94e, 0xa04, 0x8d3, 0x9fb,
	0xace, 0xbe1, 0xbac, 0xb77, 0xb39, 0xa80, 0x7dd, 0xae0,
	0x812, 0x869, 0x8fc, 0x982, 0x8ac, 0x952, 0x8f0, 0x9a1,
	0x7ec, 0x9cf, 0x9b7, 0xaae, 0x9fa, 0x845, 0x764, 0x949,
};

static const struct msm8998_cpr_domain_config msm8998_power_config = {
	.corners = msm8998_power_corners,
	.ro_scaling_factor = msm8998_power_ro_scaling_factor,
	.corner_count = ARRAY_SIZE(msm8998_power_corners),
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

static const int msm8998_power_expected_open_uv[] = {
	640000, 640000, 640000, 640000, 640000, 640000, 640000, 640000,
	652000, 668000, 680000, 680000, 696000, 712000, 724000, 740000,
	752000, 764000, 800000, 824000, 844000, 868000,
};

static const int msm8998_power_expected_floor_uv[] = {
	608000, 608000, 608000, 608000, 608000, 608000, 608000, 608000,
	620000, 636000, 648000, 648000, 664000, 680000, 692000, 708000,
	720000, 732000, 800000, 800000, 804000, 852000,
};

static const u32 msm8998_power_expected_target_quot[] = {
	614, 614, 614, 614, 614, 614, 614, 614, 658, 716, 774,
	774, 810, 846, 881, 917, 956, 991, 1077, 1134, 1194, 1254,
};

static const struct msm8998_cpr_corner_config msm8998_performance_corners[] = {
	MSM8998_CORNER(300000000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(345600000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(422400000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(499200000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(576000000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(652800000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(729600000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(806400000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(902400000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(979200000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(1056000000, 568000, 828000, 0, 0, 32000),
	MSM8998_CORNER(1132800000, 568000, 828000, -8000, -10000, 32000),
	MSM8998_CORNER(1190400000, 632000, 900000, -12000, -10000, 32000),
	MSM8998_CORNER(1267200000, 632000, 900000, -12000, -11000, 32000),
	MSM8998_CORNER(1344000000, 632000, 900000, -12000, -12000, 32000),
	MSM8998_CORNER(1420800000, 632000, 900000, -12000, -12000, 32000),
	MSM8998_CORNER(1497600000, 632000, 900000, -12000, -13000, 32000),
	MSM8998_CORNER(1574400000, 632000, 900000, -12000, -14000, 32000),
	MSM8998_CORNER(1651200000, 632000, 900000, -16000, -14000, 32000),
	MSM8998_CORNER(1728000000, 632000, 900000, -16000, -15000, 32000),
	MSM8998_CORNER(1804800000, 712000, 952000, -20000, -16000, 40000),
	MSM8998_CORNER(1881600000, 712000, 952000, -16000, -16000, 40000),
	MSM8998_CORNER(1958400000, 712000, 952000, -16000, -17000, 40000),
	MSM8998_CORNER(2035200000, 772000, 1136000, -16000, -15000, 40000),
	MSM8998_CORNER(2112000000, 772000, 1136000, -12000, -14000, 40000),
	MSM8998_CORNER(2208000000, 772000, 1136000, -28000, -27000, 40000),
	MSM8998_CORNER(2265600000, 772000, 1136000, -28000, -27000, 40000),
	MSM8998_CORNER(2323200000, 772000, 1136000, -28000, -28000, 40000),
	MSM8998_CORNER(2342400000, 772000, 1136000, -28000, -28000, 40000),
	MSM8998_CORNER(2361600000, 772000, 1136000, -28000, -28000, 40000),
};

static const u32 msm8998_performance_ro_scaling_factor[] = {
	0xb29, 0xbf1, 0xb0c, 0xb88, 0xa8b, 0xaee, 0x98e, 0xa47,
	0xa45, 0xa12, 0x8c4, 0xd10, 0xcd9, 0xc41, 0xc5c, 0xa5f,
	0xb29, 0xbf1, 0xb0c, 0xb88, 0xa8b, 0xaee, 0x98e, 0xa47,
	0xa45, 0xa12, 0x8c4, 0xd10, 0xcd9, 0xc41, 0xc5c, 0xa5f,
	0xa2b, 0xac3, 0xa74, 0xad9, 0xa0d, 0xa7d, 0x9a1, 0xa32,
	0x908, 0x977, 0x8c3, 0xc20, 0xbce, 0xbdc, 0xab4, 0x8ff,
	0x76d, 0x7e0, 0x830, 0x8b4, 0x7f2, 0x871, 0x81d, 0x88c,
	0x61d, 0x74e, 0x785, 0x8bb, 0x89d, 0x96d, 0x6e2, 0x5c6,
};

static const struct msm8998_cpr_domain_config msm8998_performance_config = {
	.corners = msm8998_performance_corners,
	.ro_scaling_factor = msm8998_performance_ro_scaling_factor,
	.corner_count = ARRAY_SIZE(msm8998_performance_corners),
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

static const int msm8998_performance_expected_open_uv[] = {
	656000, 656000, 656000, 656000, 656000, 656000, 656000, 656000,
	656000, 656000, 656000, 656000, 656000, 672000, 684000, 700000,
	716000, 732000, 740000, 756000, 780000, 812000, 844000, 872000,
	904000, 924000, 944000, 968000, 976000, 980000,
};

static const int msm8998_performance_expected_floor_uv[] = {
	624000, 624000, 624000, 624000, 624000, 624000, 624000, 624000,
	624000, 624000, 624000, 624000, 632000, 640000, 652000, 668000,
	684000, 700000, 708000, 724000, 740000, 800000, 804000, 852000,
	864000, 884000, 904000, 928000, 936000, 940000,
};

static const u32 msm8998_performance_expected_target_quot[] = {
	1093, 1093, 1093, 1093, 1093, 1093, 1093, 1093, 1093, 1093,
	1093, 1093, 1097, 1140, 1182, 1228, 1271, 1314, 1360, 1402,
	1479, 1544, 1607, 1676, 1744, 1796, 1845, 1892, 1908, 1924,
};

static const struct msm8998_cprh_controller_config msm8998_power_cprh = {
	.clock_rate_hz = 19200000,
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

static const struct msm8998_cprh_controller_config msm8998_performance_cprh = {
	.clock_rate_hz = 19200000,
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

static const struct msm8998_osm_sequence_config msm8998_power_sequence = {
	.apm_threshold_uv = 800000,
};

static const struct msm8998_osm_sequence_config msm8998_performance_sequence = {
	.apm_threshold_uv = 800000,
	.mem_acc_threshold_uv = 852000,
};

static const struct msm8998_osm_acd_config msm8998_acd_config = {
	.tunable_delay = 0x00009611,
	.control = 0x002b5ffd,
	.soft_start_control = 0x00000501,
	.initial_extint = 0x02cf9ae8,
	.final_extint = 0x02cf9afe,
	.auto_transfer_control = 0x00000015,
};

static const u32 msm8998_power_expected_open_loop_registers[] = {
	0x36004149, 0x36004149, 0x36004149, 0x36004149,
	0x36004149, 0x36004149, 0x36004149, 0x36004149,
	0x360b444c, 0x361a4850, 0x36284b53, 0x36284b53,
	0x36314f57, 0x363a535b, 0x3643565e, 0x364c5a62,
	0x36565d65, 0x365f6068, 0x36747171, 0x36827177,
	0x3691727c, 0x36a07e82, 0x20008585, 0x20007e7e,
};

static const u32 msm8998_performance_expected_open_loop_registers[] = {
	0x3600454d, 0x3600454d, 0x3600454d, 0x3600454d,
	0x3600454d, 0x3600454d, 0x3600454d, 0x3600454d,
	0x3600454d, 0x3600454d, 0x3600454d, 0x3600454d,
	0x3601474d, 0x360c4951, 0x36174c54, 0x36225058,
	0x362d545c, 0x36385860, 0x36435a62, 0x364e5e66,
	0x3661626c, 0x36717174, 0x3681727c, 0x36927e83,
	0x36a3818b, 0x36b08690, 0x36bc8b95, 0x36c8919b,
	0x36cc939d, 0x36d0949e, 0x20008585, 0x20007e7e,
};

#define MSM8998_OSM_LUT(_frequency, _frequency_data, _override, _spare, _vc) \
	{                                                                    \
		.frequency_hz = (_frequency),                                \
		.frequency_data = (_frequency_data),                         \
		.override_data = (_override),                                \
		.spare_data = (_spare),                                      \
		.virtual_corner = (_vc),                                     \
	}

static const struct msm8998_osm_lut_config msm8998_power_lut_config[] = {
	MSM8998_OSM_LUT(300000000, 0x0004000f, 0x01200020, 1, 1),
	MSM8998_OSM_LUT(364800000, 0x05040013, 0x01200020, 1, 2),
	MSM8998_OSM_LUT(441600000, 0x05040017, 0x02200020, 1, 3),
	MSM8998_OSM_LUT(518400000, 0x0504001b, 0x02200020, 1, 4),
	MSM8998_OSM_LUT(595200000, 0x0504001f, 0x02200020, 1, 5),
	MSM8998_OSM_LUT(672000000, 0x05040023, 0x03200020, 1, 6),
	MSM8998_OSM_LUT(748800000, 0x05040027, 0x03200020, 1, 7),
	MSM8998_OSM_LUT(825600000, 0x0404002b, 0x03220022, 1, 8),
	MSM8998_OSM_LUT(883200000, 0x0404002e, 0x04250025, 1, 9),
	MSM8998_OSM_LUT(960000000, 0x04040032, 0x04280028, 1, 10),
	MSM8998_OSM_LUT(1036800000, 0x04040036, 0x042b002b, 1, 11),
	MSM8998_OSM_LUT(1094400000, 0x04040039, 0x052e002e, 2, 12),
	MSM8998_OSM_LUT(1171200000, 0x0404003d, 0x05310031, 2, 13),
	MSM8998_OSM_LUT(1248000000, 0x04040041, 0x05340034, 2, 14),
	MSM8998_OSM_LUT(1324800000, 0x04040045, 0x06370037, 2, 15),
	MSM8998_OSM_LUT(1401600000, 0x04040049, 0x063a003a, 2, 16),
	MSM8998_OSM_LUT(1478400000, 0x0404004d, 0x073e003e, 2, 17),
	MSM8998_OSM_LUT(1555200000, 0x04040051, 0x07410041, 2, 18),
	MSM8998_OSM_LUT(1670400000, 0x04040057, 0x08460046, 2, 19),
	MSM8998_OSM_LUT(1747200000, 0x0404005b, 0x08490049, 2, 20),
	MSM8998_OSM_LUT(1824000000, 0x0404005f, 0x084c004c, 3, 21),
	MSM8998_OSM_LUT(1900800000, 0x04040063, 0x094f004f, 3, 22),
};

static const struct msm8998_osm_lut_config msm8998_performance_lut_config[] = {
	MSM8998_OSM_LUT(300000000, 0x0004000f, 0x01200020, 1, 1),
	MSM8998_OSM_LUT(345600000, 0x05040012, 0x01200020, 1, 2),
	MSM8998_OSM_LUT(422400000, 0x05040016, 0x02200020, 1, 3),
	MSM8998_OSM_LUT(499200000, 0x0504001a, 0x02200020, 1, 4),
	MSM8998_OSM_LUT(576000000, 0x0504001e, 0x02200020, 1, 5),
	MSM8998_OSM_LUT(652800000, 0x05040022, 0x03200020, 1, 6),
	MSM8998_OSM_LUT(729600000, 0x05040026, 0x03200020, 1, 7),
	MSM8998_OSM_LUT(806400000, 0x0504002a, 0x03220022, 1, 8),
	MSM8998_OSM_LUT(902400000, 0x0404002f, 0x04260026, 1, 9),
	MSM8998_OSM_LUT(979200000, 0x04040033, 0x04290029, 1, 10),
	MSM8998_OSM_LUT(1056000000, 0x04040037, 0x052c002c, 1, 11),
	MSM8998_OSM_LUT(1132800000, 0x0404003b, 0x052f002f, 1, 12),
	MSM8998_OSM_LUT(1190400000, 0x0404003e, 0x05320032, 2, 13),
	MSM8998_OSM_LUT(1267200000, 0x04040042, 0x06350035, 2, 14),
	MSM8998_OSM_LUT(1344000000, 0x04040046, 0x06380038, 2, 15),
	MSM8998_OSM_LUT(1420800000, 0x0404004a, 0x063b003b, 2, 16),
	MSM8998_OSM_LUT(1497600000, 0x0404004e, 0x073e003e, 2, 17),
	MSM8998_OSM_LUT(1574400000, 0x04040052, 0x07420042, 2, 18),
	MSM8998_OSM_LUT(1651200000, 0x04040056, 0x07450045, 2, 19),
	MSM8998_OSM_LUT(1728000000, 0x0404005a, 0x08480048, 2, 20),
	MSM8998_OSM_LUT(1804800000, 0x0404005e, 0x084b004b, 2, 21),
	MSM8998_OSM_LUT(1881600000, 0x04040062, 0x094e004e, 2, 22),
	MSM8998_OSM_LUT(1958400000, 0x04040066, 0x09520052, 2, 23),
	MSM8998_OSM_LUT(2035200000, 0x0404006a, 0x09550055, 3, 24),
	MSM8998_OSM_LUT(2112000000, 0x0404006e, 0x0a580058, 3, 25),
	MSM8998_OSM_LUT(2208000000, 0x04040073, 0x0a5c005c, 3, 26),
	MSM8998_OSM_LUT(2265600000, 0x04010076, 0x0a5e005e, 3, 26),
	MSM8998_OSM_LUT(2265600000, 0x04040076, 0x0a5e005e, 3, 27),
	MSM8998_OSM_LUT(2342400000, 0x0401007a, 0x0a620062, 3, 27),
	MSM8998_OSM_LUT(2323200000, 0x04040079, 0x0a610061, 3, 28),
	MSM8998_OSM_LUT(2419200000, 0x0401007e, 0x0a650065, 3, 28),
	MSM8998_OSM_LUT(2342400000, 0x0404007a, 0x0a620062, 3, 29),
	MSM8998_OSM_LUT(2438400000, 0x0401007f, 0x0a660066, 3, 29),
	MSM8998_OSM_LUT(2361600000, 0x0404007b, 0x0a620062, 3, 30),
	MSM8998_OSM_LUT(2457600000, 0x04010080, 0x0a660066, 3, 30),
};

#define MSM8998_OSM_WRITE(_offset, _value, _access) \
	{                                             \
		.offset = (_offset),                    \
		.value = (_value),                      \
		.access = (_access),                    \
	}

static const struct msm8998_osm_register_write
msm8998_power_expected_sequence_writes[] = {
	MSM8998_OSM_WRITE(0x3dc, 10, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x3e0, 11, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x3e4, 19, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x3e8, 20, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x380, 0x5f, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x1048, 18, MSM8998_OSM_ACCESS_MMIO),
	MSM8998_OSM_WRITE(0x420, 23, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x33c, 18, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x37c, 17, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x430, 0x4b9, MSM8998_OSM_ACCESS_SCM),
};

static const struct msm8998_osm_register_write
msm8998_performance_expected_sequence_writes[] = {
	MSM8998_OSM_WRITE(0x460, 31, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x3dc, 11, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x3e0, 12, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x3e4, 22, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x3e8, 23, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x380, 0x6a, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x1048, 21, MSM8998_OSM_ACCESS_MMIO),
	MSM8998_OSM_WRITE(0x420, 30, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x33c, 21, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x37c, 20, MSM8998_OSM_ACCESS_SCM),
	MSM8998_OSM_WRITE(0x430, 0x579, MSM8998_OSM_ACCESS_SCM),
};

#define MSM8998_ACD_OPERATION(_type, _offset, _value) \
	{                                               \
		.type = (_type),                          \
		.offset = (_offset),                      \
		.value = (_value),                        \
	}

static const struct msm8998_osm_acd_operation
msm8998_expected_acd_operations[] = {
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_MASTER_WRITE, 0x08, 0x00009611),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_MASTER_WRITE, 0x04, 0x002b5ffd),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_MASTER_WRITE, 0x28, 0x00000501),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_MASTER_WRITE, 0x30, 0x02cf9ae8),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_MASTER_WRITE, 0x88, 0x00000015),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_AUTO_TRANSFER, 0x80, 0x00001406),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_WRITE_THROUGH, 0x3c, 1),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_WRITE_THROUGH, 0x34, 1),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_WRITE_THROUGH, 0x34, 0),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_DELAY_US, 0x00, 1),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_WRITE_THROUGH, 0x30, 0x02cf9afe),
	MSM8998_ACD_OPERATION(MSM8998_OSM_ACD_MASTER_WRITE, 0x80, 0x00009406),
};

#define MSM8998_EXPECTED_OPP(_frequency, _voltage, _vc, _index) \
	{                                               \
		.frequency_hz = (_frequency),           \
		.open_loop_uv = (_voltage),             \
		.virtual_corner = (_vc),                \
		.table_index = (_index),                 \
	}

static const struct msm8998_osm_opp msm8998_power_expected_opps[] = {
	MSM8998_EXPECTED_OPP(300000000, 640000, 1, 0),
	MSM8998_EXPECTED_OPP(364800000, 640000, 2, 1),
	MSM8998_EXPECTED_OPP(441600000, 640000, 3, 2),
	MSM8998_EXPECTED_OPP(518400000, 640000, 4, 3),
	MSM8998_EXPECTED_OPP(595200000, 640000, 5, 4),
	MSM8998_EXPECTED_OPP(672000000, 640000, 6, 5),
	MSM8998_EXPECTED_OPP(748800000, 640000, 7, 6),
	MSM8998_EXPECTED_OPP(825600000, 640000, 8, 7),
	MSM8998_EXPECTED_OPP(883200000, 652000, 9, 8),
	MSM8998_EXPECTED_OPP(960000000, 668000, 10, 9),
	MSM8998_EXPECTED_OPP(1036800000, 680000, 11, 10),
	MSM8998_EXPECTED_OPP(1094400000, 680000, 12, 11),
	MSM8998_EXPECTED_OPP(1171200000, 696000, 13, 12),
	MSM8998_EXPECTED_OPP(1248000000, 712000, 14, 13),
	MSM8998_EXPECTED_OPP(1324800000, 724000, 15, 14),
	MSM8998_EXPECTED_OPP(1401600000, 740000, 16, 15),
	MSM8998_EXPECTED_OPP(1478400000, 752000, 17, 16),
	MSM8998_EXPECTED_OPP(1555200000, 764000, 18, 17),
	MSM8998_EXPECTED_OPP(1670400000, 800000, 19, 18),
	MSM8998_EXPECTED_OPP(1747200000, 824000, 20, 19),
	MSM8998_EXPECTED_OPP(1824000000, 844000, 21, 20),
	MSM8998_EXPECTED_OPP(1900800000, 868000, 22, 21),
};

static const struct msm8998_osm_opp msm8998_performance_expected_opps[] = {
	MSM8998_EXPECTED_OPP(300000000, 656000, 1, 0),
	MSM8998_EXPECTED_OPP(345600000, 656000, 2, 1),
	MSM8998_EXPECTED_OPP(422400000, 656000, 3, 2),
	MSM8998_EXPECTED_OPP(499200000, 656000, 4, 3),
	MSM8998_EXPECTED_OPP(576000000, 656000, 5, 4),
	MSM8998_EXPECTED_OPP(652800000, 656000, 6, 5),
	MSM8998_EXPECTED_OPP(729600000, 656000, 7, 6),
	MSM8998_EXPECTED_OPP(806400000, 656000, 8, 7),
	MSM8998_EXPECTED_OPP(902400000, 656000, 9, 8),
	MSM8998_EXPECTED_OPP(979200000, 656000, 10, 9),
	MSM8998_EXPECTED_OPP(1056000000, 656000, 11, 10),
	MSM8998_EXPECTED_OPP(1132800000, 656000, 12, 11),
	MSM8998_EXPECTED_OPP(1190400000, 656000, 13, 12),
	MSM8998_EXPECTED_OPP(1267200000, 672000, 14, 13),
	MSM8998_EXPECTED_OPP(1344000000, 684000, 15, 14),
	MSM8998_EXPECTED_OPP(1420800000, 700000, 16, 15),
	MSM8998_EXPECTED_OPP(1497600000, 716000, 17, 16),
	MSM8998_EXPECTED_OPP(1574400000, 732000, 18, 17),
	MSM8998_EXPECTED_OPP(1651200000, 740000, 19, 18),
	MSM8998_EXPECTED_OPP(1728000000, 756000, 20, 19),
	MSM8998_EXPECTED_OPP(1804800000, 780000, 21, 20),
	MSM8998_EXPECTED_OPP(1881600000, 812000, 22, 21),
	MSM8998_EXPECTED_OPP(1958400000, 844000, 23, 22),
	MSM8998_EXPECTED_OPP(2035200000, 872000, 24, 23),
	MSM8998_EXPECTED_OPP(2112000000, 904000, 25, 24),
	MSM8998_EXPECTED_OPP(2208000000, 924000, 26, 25),
	MSM8998_EXPECTED_OPP(2265600000, 944000, 27, 27),
	MSM8998_EXPECTED_OPP(2323200000, 968000, 28, 29),
	MSM8998_EXPECTED_OPP(2342400000, 976000, 29, 31),
	MSM8998_EXPECTED_OPP(2361600000, 980000, 30, 33),
	MSM8998_EXPECTED_OPP(2457600000, 980000, 30, 34),
};

static void msm8998_cpr_raw_fuse_decode_test(struct kunit *test)
{
	static const u8 expected_voltage
		[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
			[MSM8998_CPR_POWER_CLUSTER] = { 41, 42, 38, 51 },
			[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 43, 43, 39, 42 },
		};
	static const u16 expected_quot
		[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
			[MSM8998_CPR_POWER_CLUSTER] = { 555, 697, 1000, 1248 },
			[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 1093, 1093, 1411,
							      1875 },
		};
	static const u8 expected_offset
		[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
			[MSM8998_CPR_POWER_CLUSTER] = { 0, 28, 60, 49 },
			[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 0, 0, 63, 92 },
		};
	struct msm8998_cpr_fuses fuses;
	int cluster;
	int corner;

	KUNIT_EXPECT_EQ(test, msm8998_cpr_decode_fuses(NULL, &fuses), -EINVAL);
	KUNIT_EXPECT_EQ(test,
			msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, NULL),
			-EINVAL);
	KUNIT_ASSERT_EQ(test,
			msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses),
			0);
	KUNIT_EXPECT_EQ(test, fuses.speed_bin, (u8)2);
	KUNIT_EXPECT_EQ(test, fuses.revision, (u8)3);
	KUNIT_EXPECT_EQ(test, fuses.force_highest_corner, (u8)0);

	for (cluster = 0; cluster < MSM8998_CPR_CLUSTER_COUNT; cluster++) {
		for (corner = 0; corner < MSM8998_CPR_FUSE_CORNER_COUNT;
		     corner++) {
			KUNIT_EXPECT_EQ(test,
					fuses.cluster[cluster].init_voltage[corner],
					expected_voltage[cluster][corner]);
			KUNIT_EXPECT_EQ(test,
					fuses.cluster[cluster].target_quot[corner],
					expected_quot[cluster][corner]);
			KUNIT_EXPECT_EQ(test,
					fuses.cluster[cluster].ring_osc[corner],
					(u8)11);
			KUNIT_EXPECT_EQ(test,
					fuses.cluster[cluster].quot_offset[corner],
					expected_offset[cluster][corner]);
		}
	}
}

static void msm8998_cpr_voltage_decode_test(struct kunit *test)
{
	int voltage_uv;

	KUNIT_ASSERT_EQ(test,
			msm8998_cpr_decode_voltage(688000, 0, &voltage_uv), 0);
	KUNIT_EXPECT_EQ(test, voltage_uv, 688000);
	KUNIT_ASSERT_EQ(test,
			msm8998_cpr_decode_voltage(688000, 31, &voltage_uv), 0);
	KUNIT_EXPECT_EQ(test, voltage_uv, 998000);
	KUNIT_ASSERT_EQ(test,
			msm8998_cpr_decode_voltage(688000, 32, &voltage_uv), 0);
	KUNIT_EXPECT_EQ(test, voltage_uv, 688000);
	KUNIT_ASSERT_EQ(test,
			msm8998_cpr_decode_voltage(688000, 63, &voltage_uv), 0);
	KUNIT_EXPECT_EQ(test, voltage_uv, 378000);
	KUNIT_EXPECT_EQ(test,
			msm8998_cpr_decode_voltage(688000, 64, &voltage_uv),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, msm8998_cpr_decode_voltage(688000, 0, NULL),
			-EINVAL);
}

static void msm8998_cpr_known_device_test(struct kunit *test)
{
	static const int expected_fused_uv
		[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
			[MSM8998_CPR_POWER_CLUSTER] = { 598000, 656000, 768000,
							866000 },
			[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 646000, 646000,
							      758000, 956000 },
		};
	static const int expected_adjusted_uv
		[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
			[MSM8998_CPR_POWER_CLUSTER] = { 638000, 680000, 780000,
							896000 },
			[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 654000, 654000,
							      770000, 1008000 },
		};
	static const u16 expected_quot_offset
		[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
			[MSM8998_CPR_POWER_CLUSTER] = { 0, 140, 300, 245 },
			[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 0, 0, 315, 460 },
		};
	static const u16 expected_target_quot
		[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
			[MSM8998_CPR_POWER_CLUSTER] = { 555, 697, 1000, 1248 },
			[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 1093, 1093, 1411,
							      1875 },
		};
	struct msm8998_cpr_fuses fuses;
	struct msm8998_cpr_fuse_data data;
	int cluster;
	int corner;

	KUNIT_ASSERT_EQ(test,
			msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses),
			0);
	KUNIT_ASSERT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, &data),
			0);
	KUNIT_EXPECT_EQ(test, data.speed_bin, (u8)2);
	KUNIT_EXPECT_EQ(test, data.revision, (u8)3);
	KUNIT_EXPECT_EQ(test, data.fuse_combo, (u8)19);

	for (cluster = 0; cluster < MSM8998_CPR_CLUSTER_COUNT; cluster++) {
		const struct msm8998_cpr_cluster_fuse_data *cluster_data =
			&data.cluster[cluster];

		for (corner = 0; corner < MSM8998_CPR_FUSE_CORNER_COUNT;
		     corner++) {
			KUNIT_EXPECT_EQ(test,
					cluster_data->fused_open_loop_uv[corner],
					expected_fused_uv[cluster][corner]);
			KUNIT_EXPECT_EQ(test,
					cluster_data
					->adjusted_fused_open_loop_uv[corner],
					expected_adjusted_uv[cluster][corner]);
			KUNIT_EXPECT_EQ(test, cluster_data->target_quot[corner],
					expected_target_quot[cluster][corner]);
			KUNIT_EXPECT_EQ(test, cluster_data->quot_offset[corner],
					expected_quot_offset[cluster][corner]);
			KUNIT_EXPECT_EQ(test, cluster_data->ring_osc[corner],
					(u8)11);
		}
	}
}

static void msm8998_cpr_power_corner_table_test(struct kunit *test)
{
	struct msm8998_cpr_corner_data corners[MSM8998_CPR_MAX_CORNER_COUNT];
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	unsigned int fuse_corner = 0;
	unsigned int i;
	int ret;

	ret = msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &data);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_build_corners(&msm8998_power_config,
					&data.cluster[MSM8998_CPR_POWER_CLUSTER],
					corners, ARRAY_SIZE(corners));
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_apply_partial_binning(data.force_highest_corner,
						corners,
						msm8998_power_config.corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, ARRAY_SIZE(msm8998_power_corners),
			ARRAY_SIZE(msm8998_power_expected_open_uv));
	KUNIT_ASSERT_EQ(test, ARRAY_SIZE(msm8998_power_corners),
			ARRAY_SIZE(msm8998_power_expected_floor_uv));
	KUNIT_ASSERT_EQ(test, ARRAY_SIZE(msm8998_power_corners),
			ARRAY_SIZE(msm8998_power_expected_target_quot));

	for (i = 0; i < ARRAY_SIZE(msm8998_power_corners); i++) {
		while (i > msm8998_power_config.fmax_corner[fuse_corner])
			fuse_corner++;

		KUNIT_EXPECT_EQ(test, corners[i].frequency_hz,
				msm8998_power_corners[i].frequency_hz);
		KUNIT_EXPECT_EQ(test, corners[i].open_loop_uv,
				msm8998_power_expected_open_uv[i]);
		KUNIT_EXPECT_EQ(test, corners[i].floor_uv,
				msm8998_power_expected_floor_uv[i]);
		KUNIT_EXPECT_EQ(test, corners[i].ceiling_uv,
				msm8998_power_expected_open_uv[i]);
		KUNIT_EXPECT_EQ(test, corners[i].target_quot,
				msm8998_power_expected_target_quot[i]);
		KUNIT_EXPECT_EQ(test, corners[i].fuse_corner, (u8)fuse_corner);
		KUNIT_EXPECT_EQ(test, corners[i].ring_osc, (u8)11);
		KUNIT_EXPECT_FALSE(test, corners[i].use_open_loop);
	}
}

static void msm8998_cpr_performance_corner_table_test(struct kunit *test)
{
	struct msm8998_cpr_corner_data corners[MSM8998_CPR_MAX_CORNER_COUNT];
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	unsigned int fuse_corner = 0;
	unsigned int i;
	int ret;

	ret = msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &data);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_build_corners(&msm8998_performance_config,
					&data.cluster[MSM8998_CPR_PERFORMANCE_CLUSTER],
					corners, ARRAY_SIZE(corners));
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_apply_partial_binning(data.force_highest_corner,
						corners,
						msm8998_performance_config.corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, ARRAY_SIZE(msm8998_performance_corners),
			ARRAY_SIZE(msm8998_performance_expected_open_uv));
	KUNIT_ASSERT_EQ(test, ARRAY_SIZE(msm8998_performance_corners),
			ARRAY_SIZE(msm8998_performance_expected_floor_uv));
	KUNIT_ASSERT_EQ(test, ARRAY_SIZE(msm8998_performance_corners),
			ARRAY_SIZE(msm8998_performance_expected_target_quot));

	for (i = 0; i < ARRAY_SIZE(msm8998_performance_corners); i++) {
		while (i > msm8998_performance_config.fmax_corner[fuse_corner])
			fuse_corner++;

		KUNIT_EXPECT_EQ(test, corners[i].frequency_hz,
				msm8998_performance_corners[i].frequency_hz);
		KUNIT_EXPECT_EQ(test, corners[i].open_loop_uv,
				msm8998_performance_expected_open_uv[i]);
		KUNIT_EXPECT_EQ(test, corners[i].floor_uv,
				msm8998_performance_expected_floor_uv[i]);
		KUNIT_EXPECT_EQ(test, corners[i].ceiling_uv,
				msm8998_performance_expected_open_uv[i]);
		KUNIT_EXPECT_EQ(test, corners[i].target_quot,
				msm8998_performance_expected_target_quot[i]);
		KUNIT_EXPECT_EQ(test, corners[i].fuse_corner, (u8)fuse_corner);
		KUNIT_EXPECT_EQ(test, corners[i].ring_osc, (u8)11);
		KUNIT_EXPECT_FALSE(test, corners[i].use_open_loop);
	}
}

static void msm8998_cpr_partial_binning_test(struct kunit *test)
{
	struct msm8998_cpr_corner_data corners[MSM8998_CPR_MAX_CORNER_COUNT];
	struct msm8998_cpr_corner_data highest;
	struct msm8998_cpr_fuse_rows rows = msm8998_cpr_known_rows;
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	unsigned int i;
	int ret;

	rows.row100 = 1ULL << 45;
	ret = msm8998_cpr_decode_fuses(&rows, &fuses);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, fuses.force_highest_corner, (u8)1);
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &data);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_build_corners(&msm8998_power_config,
					&data.cluster[MSM8998_CPR_POWER_CLUSTER],
					corners, ARRAY_SIZE(corners));
	KUNIT_ASSERT_EQ(test, ret, 0);
	highest = corners[msm8998_power_config.corner_count - 1];

	ret = msm8998_cpr_apply_partial_binning(data.force_highest_corner,
						corners,
						msm8998_power_config.corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	for (i = 0; i < msm8998_power_config.corner_count; i++) {
		KUNIT_EXPECT_EQ(test, corners[i].frequency_hz,
				msm8998_power_corners[i].frequency_hz);
		KUNIT_EXPECT_EQ(test, corners[i].floor_uv, highest.floor_uv);
		KUNIT_EXPECT_EQ(test, corners[i].open_loop_uv,
				highest.open_loop_uv);
		KUNIT_EXPECT_EQ(test, corners[i].ceiling_uv,
				highest.ceiling_uv);
		KUNIT_EXPECT_EQ(test, corners[i].target_quot,
				highest.target_quot);
		KUNIT_EXPECT_EQ(test, corners[i].fuse_corner,
				highest.fuse_corner);
		KUNIT_EXPECT_EQ(test, corners[i].ring_osc, highest.ring_osc);
	}
}

static void msm8998_osm_expect_opps(struct kunit *test,
				    const struct msm8998_osm_opp *actual,
				    unsigned int actual_count,
				    const struct msm8998_osm_opp *expected,
				    unsigned int expected_count)
{
	unsigned int i;

	KUNIT_ASSERT_EQ(test, actual_count, expected_count);
	for (i = 0; i < expected_count; i++) {
		KUNIT_EXPECT_EQ(test, actual[i].frequency_hz,
				expected[i].frequency_hz);
		KUNIT_EXPECT_EQ(test, actual[i].open_loop_uv,
				expected[i].open_loop_uv);
		KUNIT_EXPECT_EQ(test, actual[i].virtual_corner,
				expected[i].virtual_corner);
		KUNIT_EXPECT_EQ(test, actual[i].table_index,
				expected[i].table_index);
	}
}

static void msm8998_osm_power_table_test(struct kunit *test)
{
	struct msm8998_cpr_corner_data corners[MSM8998_CPR_MAX_CORNER_COUNT];
	struct msm8998_osm_lut_data *lut;
	struct msm8998_osm_opp *opps;
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	unsigned int opp_count;
	unsigned int i;
	int ret;

	lut = kunit_kcalloc(test, MSM8998_OSM_TABLE_SIZE, sizeof(*lut),
			    GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, lut);
	opps = kunit_kcalloc(test, MSM8998_OSM_MAX_OPP_COUNT, sizeof(*opps),
			     GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, opps);

	ret = msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &data);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_build_corners(&msm8998_power_config,
					&data.cluster[MSM8998_CPR_POWER_CLUSTER],
					corners, ARRAY_SIZE(corners));
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_apply_partial_binning(data.force_highest_corner,
						corners,
						msm8998_power_config.corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_osm_resolve_lut(msm8998_power_lut_config,
				      ARRAY_SIZE(msm8998_power_lut_config),
				      corners,
				      ARRAY_SIZE(msm8998_power_corners), lut,
				      MSM8998_OSM_TABLE_SIZE);
	KUNIT_ASSERT_EQ(test, ret, 0);

	for (i = 0; i < ARRAY_SIZE(msm8998_power_lut_config); i++)
		KUNIT_EXPECT_EQ(test, lut[i].open_loop_uv,
				msm8998_power_expected_open_uv[i]);
	for (; i < MSM8998_OSM_TABLE_SIZE; i++) {
		KUNIT_EXPECT_EQ(test, lut[i].config.frequency_hz,
				(u32)1900800000);
		KUNIT_EXPECT_EQ(test, lut[i].config.virtual_corner, (u8)22);
		KUNIT_EXPECT_EQ(test, lut[i].open_loop_uv, 868000);
	}

	ret = msm8998_osm_build_opp_table(lut,
					  ARRAY_SIZE(msm8998_power_lut_config),
					  opps, MSM8998_OSM_MAX_OPP_COUNT,
					  &opp_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	msm8998_osm_expect_opps(test, opps, opp_count,
				msm8998_power_expected_opps,
				ARRAY_SIZE(msm8998_power_expected_opps));
}

static void msm8998_osm_performance_table_test(struct kunit *test)
{
	struct msm8998_cpr_corner_data corners[MSM8998_CPR_MAX_CORNER_COUNT];
	struct msm8998_osm_lut_data *lut;
	struct msm8998_osm_opp *opps;
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	unsigned int opp_count;
	unsigned int i;
	int ret;

	lut = kunit_kcalloc(test, MSM8998_OSM_TABLE_SIZE, sizeof(*lut),
			    GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, lut);
	opps = kunit_kcalloc(test, MSM8998_OSM_MAX_OPP_COUNT, sizeof(*opps),
			     GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, opps);

	ret = msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &data);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_build_corners(&msm8998_performance_config,
					&data.cluster[MSM8998_CPR_PERFORMANCE_CLUSTER],
					corners, ARRAY_SIZE(corners));
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_apply_partial_binning(data.force_highest_corner,
						corners,
						msm8998_performance_config.corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_osm_resolve_lut(msm8998_performance_lut_config,
				      ARRAY_SIZE(msm8998_performance_lut_config),
				      corners,
				      ARRAY_SIZE(msm8998_performance_corners), lut,
				      MSM8998_OSM_TABLE_SIZE);
	KUNIT_ASSERT_EQ(test, ret, 0);

	for (i = 0; i < ARRAY_SIZE(msm8998_performance_lut_config); i++) {
		unsigned int virtual_corner =
			msm8998_performance_lut_config[i].virtual_corner;

		KUNIT_EXPECT_EQ(test, lut[i].open_loop_uv,
				msm8998_performance_expected_open_uv[virtual_corner -
								     1]);
	}
	for (; i < MSM8998_OSM_TABLE_SIZE; i++) {
		KUNIT_EXPECT_EQ(test, lut[i].config.frequency_hz,
				(u32)2457600000);
		KUNIT_EXPECT_EQ(test, lut[i].config.virtual_corner, (u8)30);
		KUNIT_EXPECT_EQ(test, lut[i].open_loop_uv, 980000);
	}

	ret = msm8998_osm_build_opp_table(lut,
					  ARRAY_SIZE(msm8998_performance_lut_config),
					  opps, MSM8998_OSM_MAX_OPP_COUNT,
					  &opp_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	msm8998_osm_expect_opps(test, opps, opp_count,
				msm8998_performance_expected_opps,
				ARRAY_SIZE(msm8998_performance_expected_opps));
}

static int
msm8998_cpr_build_known_domain(const struct msm8998_cpr_domain_config *config,
			       enum msm8998_cpr_cluster_id cluster,
			       struct msm8998_cpr_corner_data *corners,
			       unsigned int *corner_count)
{
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	const unsigned int capacity = MSM8998_CPR_MAX_CORNER_COUNT;
	int ret;

	ret = msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses);
	if (ret)
		return ret;
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &data);
	if (ret)
		return ret;
	ret = msm8998_cpr_build_corners(config, &data.cluster[cluster], corners,
					MSM8998_CPR_MAX_CORNER_COUNT);
	if (ret)
		return ret;
	ret = msm8998_cpr_apply_partial_binning(data.force_highest_corner,
						corners, config->corner_count);
	if (ret)
		return ret;

	*corner_count = config->corner_count;
	ret = msm8998_cpr_append_crossover_corners(config, corners, capacity,
						   corner_count);

	return ret;
}

static void msm8998_cpr_crossover_corner_test(struct kunit *test)
{
	struct msm8998_cpr_corner_data *corners;
	unsigned int corner_count;
	int ret;

	corners = kunit_kcalloc(test, MSM8998_CPR_MAX_CORNER_COUNT,
				sizeof(*corners), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, corners);
	ret = msm8998_cpr_build_known_domain(&msm8998_power_config,
					     MSM8998_CPR_POWER_CLUSTER,
					     corners, &corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, corner_count, 24U);
	KUNIT_EXPECT_EQ(test, corners[22].frequency_hz, (u32)0);
	KUNIT_EXPECT_EQ(test, corners[22].floor_uv, 880000);
	KUNIT_EXPECT_EQ(test, corners[22].open_loop_uv, 880000);
	KUNIT_EXPECT_EQ(test, corners[22].ceiling_uv, 880000);
	KUNIT_EXPECT_EQ(test, corners[22].target_quot, (u32)0);
	KUNIT_EXPECT_TRUE(test, corners[22].use_open_loop);
	KUNIT_EXPECT_EQ(test, corners[23].frequency_hz, (u32)0);
	KUNIT_EXPECT_EQ(test, corners[23].floor_uv, 852000);
	KUNIT_EXPECT_EQ(test, corners[23].open_loop_uv, 852000);
	KUNIT_EXPECT_EQ(test, corners[23].ceiling_uv, 852000);
	KUNIT_EXPECT_EQ(test, corners[23].target_quot, (u32)0);
	KUNIT_EXPECT_TRUE(test, corners[23].use_open_loop);
}

static void msm8998_expect_cprh_image(struct kunit *test,
				      const struct msm8998_cpr_domain_config *domain,
				      enum msm8998_cpr_cluster_id cluster,
				      const struct msm8998_cprh_controller_config *controller,
				      const u32 *expected_corners,
				      unsigned int expected_corner_count,
				      u32 expected_base_quot, u32 expected_step_quot,
				      u16 expected_sensor_count)
{
	struct msm8998_cprh_register_image *image;
	struct msm8998_cpr_corner_data *corners;
	unsigned int corner_count;
	unsigned int i;
	int ret;

	corners = kunit_kcalloc(test, MSM8998_CPR_MAX_CORNER_COUNT,
				sizeof(*corners), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, corners);
	image = kunit_kzalloc(test, sizeof(*image), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, image);

	ret = msm8998_cpr_build_known_domain(domain, cluster, corners,
					     &corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, corner_count, expected_corner_count);
	ret = msm8998_cprh_build_register_image(controller, corners,
						corner_count, false, image);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, image->corner_count, (u8)expected_corner_count);
	KUNIT_EXPECT_EQ(test, image->cpr_ctl, (u32)0x0000021f);
	KUNIT_EXPECT_EQ(test, image->cpr_timer_auto_cont, (u32)96000);
	KUNIT_EXPECT_EQ(test, image->cpr_step_quot, expected_step_quot);
	KUNIT_EXPECT_EQ(test, image->gcnt, (u32)19);
	KUNIT_EXPECT_EQ(test, image->gcnt_ro_mask, (u16)BIT(11));
	KUNIT_EXPECT_EQ(test, image->ro_mask, (u16)0xf7ff);
	KUNIT_EXPECT_EQ(test, image->target_quot[11], expected_base_quot);
	KUNIT_EXPECT_EQ(test, image->threshold, (u32)0x00004202);
	KUNIT_EXPECT_EQ(test, image->margin_adjust_ctl, (u32)0x00004080);
	KUNIT_EXPECT_EQ(test, image->saw_error_step_limit, (u32)0x21);
	KUNIT_EXPECT_EQ(test, image->margin_temp_core_timers,
			(u32)0x00840000);
	KUNIT_EXPECT_EQ(test, image->cprh_ctl, (u32)0x080a00af);
	KUNIT_EXPECT_EQ(test, image->sensor_owner_count,
			expected_sensor_count);

	for (i = 0; i < MSM8998_CPR_RING_OSC_COUNT; i++) {
		if (i != 11)
			KUNIT_EXPECT_EQ(test, image->target_quot[i], (u32)0);
	}
	for (i = 0; i < expected_corner_count; i++)
		KUNIT_EXPECT_EQ(test, image->corner[i], expected_corners[i]);

	ret = msm8998_cprh_build_register_image(controller, corners,
						corner_count, true, image);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, image->margin_adjust_ctl, (u32)0x00004090);
	for (i = 0; i < domain->corner_count; i++)
		KUNIT_EXPECT_EQ(test, image->corner[i],
				expected_corners[i] & ~BIT(29));
	for (; i < expected_corner_count; i++)
		KUNIT_EXPECT_EQ(test, image->corner[i], expected_corners[i]);
}

static void msm8998_cprh_register_image_test(struct kunit *test)
{
	msm8998_expect_cprh_image(test, &msm8998_power_config,
				    MSM8998_CPR_POWER_CLUSTER,
		&msm8998_power_cprh, msm8998_power_expected_open_loop_registers,
		ARRAY_SIZE(msm8998_power_expected_open_loop_registers), 614,
		0x30b, 6);
	msm8998_expect_cprh_image(test, &msm8998_performance_config,
				    MSM8998_CPR_PERFORMANCE_CLUSTER,
		&msm8998_performance_cprh,
		msm8998_performance_expected_open_loop_registers,
		ARRAY_SIZE(msm8998_performance_expected_open_loop_registers),
		1093, 0x389, 9);
}

static void msm8998_expect_osm_initialization(
			struct kunit *test,
			const struct msm8998_cpr_domain_config *domain,
			enum msm8998_cpr_cluster_id cluster,
			const struct msm8998_osm_lut_config *lut_config,
			unsigned int config_count, const int *expected_open_uv,
			unsigned int expected_open_count,
			const struct msm8998_osm_sequence_config *sequence_config,
			const struct msm8998_osm_register_write *expected_writes,
			unsigned int expected_write_count,
			const u8 expected_mem_acc_level_vc[4],
			u8 expected_apm_crossover_vc,
			u8 expected_apm_threshold_pre_vc,
			u8 expected_apm_threshold_vc,
			u8 expected_mem_acc_crossover_vc,
			u8 expected_mem_acc_threshold_pre_vc,
			u8 expected_mem_acc_threshold_vc)
{
	struct msm8998_osm_hw_table_image *hardware;
	struct msm8998_osm_sequence_image *sequence;
	struct msm8998_cpr_corner_data *corners;
	struct msm8998_osm_lut_data *lut;
	unsigned int corner_count;
	unsigned int source;
	unsigned int i;
	int ret;

	corners = kunit_kcalloc(test, MSM8998_CPR_MAX_CORNER_COUNT,
				sizeof(*corners), GFP_KERNEL);
	lut = kunit_kcalloc(test, MSM8998_OSM_TABLE_SIZE, sizeof(*lut),
			   GFP_KERNEL);
	hardware = kunit_kzalloc(test, sizeof(*hardware), GFP_KERNEL);
	sequence = kunit_kzalloc(test, sizeof(*sequence), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, corners);
	KUNIT_ASSERT_NOT_NULL(test, lut);
	KUNIT_ASSERT_NOT_NULL(test, hardware);
	KUNIT_ASSERT_NOT_NULL(test, sequence);

	ret = msm8998_cpr_build_known_domain(domain, cluster, corners,
					     &corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_osm_resolve_lut(lut_config, config_count, corners,
				      corner_count, lut, MSM8998_OSM_TABLE_SIZE);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_osm_build_hw_table_image(lut, config_count, hardware);
	KUNIT_ASSERT_EQ(test, ret, 0);

	for (i = 0; i < MSM8998_OSM_TABLE_SIZE; i++) {
		u8 virtual_corner;
		u32 expected_voltage;

		source = i < config_count ? i : config_count - 1;
		virtual_corner = lut_config[source].virtual_corner;
		KUNIT_ASSERT_GT(test, virtual_corner, (u8)0);
		KUNIT_ASSERT_LE(test, virtual_corner, expected_open_count);
		expected_voltage = (virtual_corner - 1) << 16 |
			expected_open_uv[virtual_corner - 1] / 1000;
		KUNIT_EXPECT_EQ(test, hardware->entry[i].index, (u32)i);
		KUNIT_EXPECT_EQ(test, hardware->entry[i].frequency_data,
				lut_config[source].frequency_data);
		KUNIT_EXPECT_EQ(test, hardware->entry[i].voltage_data,
				expected_voltage);
		KUNIT_EXPECT_EQ(test, hardware->entry[i].override_data,
				lut_config[source].override_data);
		KUNIT_EXPECT_EQ(test, hardware->entry[i].spare_data,
				lut_config[source].spare_data);
	}

	ret = msm8998_osm_build_tz_sequence_image(sequence_config, lut,
			config_count, corners, corner_count, sequence);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_TRUE(test, sequence->sequencer_owned_by_tz);
	KUNIT_EXPECT_EQ(test, sequence->write_count, (u8)expected_write_count);
	KUNIT_EXPECT_EQ(test, sequence->apm_crossover_vc,
			expected_apm_crossover_vc);
	KUNIT_EXPECT_EQ(test, sequence->apm_threshold_pre_vc,
			expected_apm_threshold_pre_vc);
	KUNIT_EXPECT_EQ(test, sequence->apm_threshold_vc,
			expected_apm_threshold_vc);
	KUNIT_EXPECT_EQ(test, sequence->mem_acc_crossover_vc,
			expected_mem_acc_crossover_vc);
	KUNIT_EXPECT_EQ(test, sequence->mem_acc_threshold_pre_vc,
			expected_mem_acc_threshold_pre_vc);
	KUNIT_EXPECT_EQ(test, sequence->mem_acc_threshold_vc,
			expected_mem_acc_threshold_vc);
	for (i = 0; i < ARRAY_SIZE(sequence->mem_acc_level_vc); i++)
		KUNIT_EXPECT_EQ(test, sequence->mem_acc_level_vc[i],
				expected_mem_acc_level_vc[i]);
	for (i = 0; i < expected_write_count; i++) {
		KUNIT_EXPECT_EQ(test, sequence->write[i].offset,
				expected_writes[i].offset);
		KUNIT_EXPECT_EQ(test, sequence->write[i].value,
				expected_writes[i].value);
		KUNIT_EXPECT_EQ(test, sequence->write[i].access,
				expected_writes[i].access);
	}
}

static void msm8998_osm_initialization_image_test(struct kunit *test)
{
	static const u8 power_mem_acc_level_vc[] = { 10, 11, 19, 20 };
	static const u8 performance_mem_acc_level_vc[] = { 11, 12, 22, 23 };

	msm8998_expect_osm_initialization(test, &msm8998_power_config,
		MSM8998_CPR_POWER_CLUSTER, msm8998_power_lut_config,
		ARRAY_SIZE(msm8998_power_lut_config),
		msm8998_power_expected_open_uv,
		ARRAY_SIZE(msm8998_power_expected_open_uv),
		&msm8998_power_sequence, msm8998_power_expected_sequence_writes,
		ARRAY_SIZE(msm8998_power_expected_sequence_writes),
		power_mem_acc_level_vc, 23, 17, 18, 0, 0, 0);
	msm8998_expect_osm_initialization(test, &msm8998_performance_config,
		MSM8998_CPR_PERFORMANCE_CLUSTER,
		msm8998_performance_lut_config,
		ARRAY_SIZE(msm8998_performance_lut_config),
		msm8998_performance_expected_open_uv,
		ARRAY_SIZE(msm8998_performance_expected_open_uv),
		&msm8998_performance_sequence,
		msm8998_performance_expected_sequence_writes,
		ARRAY_SIZE(msm8998_performance_expected_sequence_writes),
		performance_mem_acc_level_vc, 30, 20, 21, 31, 22, 23);
}

static void msm8998_osm_acd_image_test(struct kunit *test)
{
	struct msm8998_osm_acd_image *image;
	unsigned int i;
	int ret;

	image = kunit_kzalloc(test, sizeof(*image), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, image);
	ret = msm8998_osm_build_acd_image(&msm8998_acd_config, image);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, image->initial_auto_transfer_mask,
			(u32)0x00001406);
	KUNIT_EXPECT_EQ(test, image->power_collapse_auto_transfer_mask,
			(u32)0x00009406);
	KUNIT_ASSERT_EQ(test, image->operation_count,
			(u8)ARRAY_SIZE(msm8998_expected_acd_operations));
	for (i = 0; i < ARRAY_SIZE(msm8998_expected_acd_operations); i++) {
		KUNIT_EXPECT_EQ(test, image->operation[i].type,
				msm8998_expected_acd_operations[i].type);
		KUNIT_EXPECT_EQ(test, image->operation[i].offset,
				msm8998_expected_acd_operations[i].offset);
		KUNIT_EXPECT_EQ(test, image->operation[i].value,
				msm8998_expected_acd_operations[i].value);
	}
}

static void msm8998_osm_initialization_invalid_test(struct kunit *test)
{
	struct msm8998_osm_sequence_config sequence_config;
	struct msm8998_osm_hw_table_image *hardware;
	struct msm8998_osm_sequence_image *sequence;
	struct msm8998_cpr_corner_data *corners;
	struct msm8998_osm_acd_image *acd;
	struct msm8998_osm_lut_data *lut;
	unsigned int corner_count;
	int ret;

	corners = kunit_kcalloc(test, MSM8998_CPR_MAX_CORNER_COUNT,
				sizeof(*corners), GFP_KERNEL);
	lut = kunit_kcalloc(test, MSM8998_OSM_TABLE_SIZE, sizeof(*lut),
			   GFP_KERNEL);
	hardware = kunit_kzalloc(test, sizeof(*hardware), GFP_KERNEL);
	sequence = kunit_kzalloc(test, sizeof(*sequence), GFP_KERNEL);
	acd = kunit_kzalloc(test, sizeof(*acd), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, corners);
	KUNIT_ASSERT_NOT_NULL(test, lut);
	KUNIT_ASSERT_NOT_NULL(test, hardware);
	KUNIT_ASSERT_NOT_NULL(test, sequence);
	KUNIT_ASSERT_NOT_NULL(test, acd);

	ret = msm8998_cpr_build_known_domain(&msm8998_power_config,
		MSM8998_CPR_POWER_CLUSTER, corners, &corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_osm_resolve_lut(msm8998_power_lut_config,
		ARRAY_SIZE(msm8998_power_lut_config), corners, corner_count,
		lut, MSM8998_OSM_TABLE_SIZE);
	KUNIT_ASSERT_EQ(test, ret, 0);

	ret = msm8998_osm_build_hw_table_image(NULL,
		ARRAY_SIZE(msm8998_power_lut_config), hardware);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_osm_build_hw_table_image(lut, 0, hardware);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_osm_build_hw_table_image(lut,
		ARRAY_SIZE(msm8998_power_lut_config), NULL);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	lut[MSM8998_OSM_TABLE_SIZE - 1].open_loop_uv += 1000;
	ret = msm8998_osm_build_hw_table_image(lut,
		ARRAY_SIZE(msm8998_power_lut_config), hardware);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	lut[MSM8998_OSM_TABLE_SIZE - 1].open_loop_uv -= 1000;
	lut[0].open_loop_uv = 0;
	ret = msm8998_osm_build_hw_table_image(lut,
		ARRAY_SIZE(msm8998_power_lut_config), hardware);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	lut[0].open_loop_uv = corners[0].open_loop_uv;

	sequence_config = msm8998_power_sequence;
	sequence_config.osm_no_tz = true;
	ret = msm8998_osm_build_tz_sequence_image(&sequence_config, lut,
		ARRAY_SIZE(msm8998_power_lut_config), corners, corner_count,
		sequence);
	KUNIT_EXPECT_EQ(test, ret, -EOPNOTSUPP);
	sequence_config = msm8998_power_sequence;
	sequence_config.apm_threshold_uv = 0;
	ret = msm8998_osm_build_tz_sequence_image(&sequence_config, lut,
		ARRAY_SIZE(msm8998_power_lut_config), corners, corner_count,
		sequence);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	sequence_config = msm8998_power_sequence;
	lut[0].config.spare_data = 0;
	ret = msm8998_osm_build_tz_sequence_image(&sequence_config, lut,
		ARRAY_SIZE(msm8998_power_lut_config), corners, corner_count,
		sequence);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	KUNIT_EXPECT_EQ(test, msm8998_osm_build_acd_image(NULL, acd), -EINVAL);
	KUNIT_EXPECT_EQ(test,
		msm8998_osm_build_acd_image(&msm8998_acd_config, NULL),
		-EINVAL);
}

static void msm8998_cpr_crossover_invalid_test(struct kunit *test)
{
	struct msm8998_cpr_corner_data *corners;
	struct msm8998_cpr_domain_config config;
	unsigned int capacity;
	unsigned int corner_count;
	int ret;

	corners = kunit_kcalloc(test, MSM8998_CPR_MAX_CORNER_COUNT,
				sizeof(*corners), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, corners);
	config = msm8998_power_config;
	corner_count = config.corner_count;
	capacity = corner_count + 1;

	ret = msm8998_cpr_append_crossover_corners(&config, corners, capacity,
						   &corner_count);
	KUNIT_EXPECT_EQ(test, ret, -ENOSPC);

	config.apm_crossover_uv = 0;
	config.mem_acc_crossover_uv = 0;
	corner_count = config.corner_count;
	capacity = config.corner_count - 1;
	ret = msm8998_cpr_append_crossover_corners(&config, corners, capacity,
						   &corner_count);
	KUNIT_EXPECT_EQ(test, ret, -ENOSPC);

	config = msm8998_power_config;
	config.apm_crossover_uv = U32_MAX;
	corner_count = config.corner_count;
	capacity = MSM8998_CPR_MAX_CORNER_COUNT;
	ret = msm8998_cpr_append_crossover_corners(&config, corners, capacity,
						   &corner_count);
	KUNIT_EXPECT_EQ(test, ret, -ERANGE);
}

static void msm8998_cprh_invalid_register_image_test(struct kunit *test)
{
	struct msm8998_cpr_corner_data *corners;
	struct msm8998_cprh_register_image *image;
	struct msm8998_cprh_controller_config controller;
	unsigned int corner_count;
	int ret;

	corners = kunit_kcalloc(test, MSM8998_CPR_MAX_CORNER_COUNT,
				sizeof(*corners), GFP_KERNEL);
	image = kunit_kzalloc(test, sizeof(*image), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, corners);
	KUNIT_ASSERT_NOT_NULL(test, image);
	ret = msm8998_cpr_build_known_domain(&msm8998_power_config,
					     MSM8998_CPR_POWER_CLUSTER,
					     corners, &corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	controller = msm8998_power_cprh;

	ret = msm8998_cprh_build_register_image(NULL, corners, corner_count,
						false, image);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cprh_build_register_image(&controller, corners, 0, false,
						image);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cprh_build_register_image(&controller, corners,
						MSM8998_CPR_MAX_CORNER_COUNT + 1,
						false, image);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cprh_build_register_image(&controller, corners, corner_count,
						false, NULL);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	controller = msm8998_power_cprh;
	controller.base_uv = U32_MAX;
	ret = msm8998_cprh_build_register_image(&controller, corners, corner_count,
						false, image);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	controller = msm8998_power_cprh;
	controller.clock_rate_hz = U32_MAX;
	controller.sensor_time_ns = U32_MAX;
	controller.loop_time_ns = 1;
	controller.voltage_settling_time_ns = 0;
	controller.corner_switch_delay_time_ns = 0;
	ret = msm8998_cprh_build_register_image(&controller, corners, corner_count,
						false, image);
	KUNIT_EXPECT_EQ(test, ret, -ERANGE);
	controller = msm8998_power_cprh;
	controller.clock_rate_hz = U32_MAX;
	controller.sensor_time_ns = 1;
	controller.loop_time_ns = U32_MAX;
	controller.voltage_settling_time_ns = 0;
	controller.corner_switch_delay_time_ns = 0;
	ret = msm8998_cprh_build_register_image(&controller, corners, corner_count,
						false, image);
	KUNIT_EXPECT_EQ(test, ret, -ERANGE);

	corners[0].floor_uv = -1;
	ret = msm8998_cprh_build_register_image(&msm8998_power_cprh, corners,
						corner_count, false, image);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cpr_build_known_domain(&msm8998_power_config,
					     MSM8998_CPR_POWER_CLUSTER,
					     corners, &corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);

	corners[0].target_quot = 0;
	ret = msm8998_cprh_build_register_image(&msm8998_power_cprh, corners,
						corner_count, false, image);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cpr_build_known_domain(&msm8998_power_config,
					     MSM8998_CPR_POWER_CLUSTER,
					     corners, &corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	corners[0].target_quot = 1;
	corners[1].target_quot = 0xfff;
	ret = msm8998_cprh_build_register_image(&msm8998_power_cprh, corners,
						corner_count, false, image);
	KUNIT_EXPECT_EQ(test, ret, -ERANGE);
	ret = msm8998_cpr_build_known_domain(&msm8998_power_config,
					     MSM8998_CPR_POWER_CLUSTER,
					     corners, &corner_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	corners[0].open_loop_uv = msm8998_power_cprh.base_uv +
					 256 * msm8998_power_cprh.step_uv;
	corners[0].ceiling_uv = corners[0].open_loop_uv;
	ret = msm8998_cprh_build_register_image(&msm8998_power_cprh, corners,
						corner_count, false, image);
	KUNIT_EXPECT_EQ(test, ret, -ERANGE);
}

static void msm8998_cpr_invalid_fuse_test(struct kunit *test)
{
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	struct msm8998_cpr_fuses valid_fuses;
	int ret;

	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(NULL, &data),
			-EINVAL);
	ret = msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &valid_fuses);
	KUNIT_ASSERT_EQ(test, ret, 0);
	fuses = valid_fuses;
	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, NULL),
			-EINVAL);

	fuses = valid_fuses;
	fuses.speed_bin = 4;
	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, &data),
			-EINVAL);
	fuses = valid_fuses;
	fuses.revision = 8;
	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, &data),
			-EINVAL);
	fuses = valid_fuses;
	fuses.force_highest_corner = 2;
	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, &data),
			-EINVAL);
	fuses = valid_fuses;
	fuses.cluster[0].init_voltage[0] = 64;
	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, &data),
			-EINVAL);
	fuses = valid_fuses;
	fuses.cluster[0].target_quot[0] = 4096;
	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, &data),
			-EINVAL);
	fuses = valid_fuses;
	fuses.cluster[0].ring_osc[0] = 16;
	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, &data),
			-EINVAL);
	fuses = valid_fuses;
	fuses.cluster[0].quot_offset[0] = 128;
	KUNIT_EXPECT_EQ(test, msm8998_cpr_calculate_fuse_data(&fuses, &data),
			-EINVAL);
}

static void msm8998_cpr_invalid_corner_test(struct kunit *test)
{
	struct msm8998_cpr_corner_config corner_config
		[ARRAY_SIZE(msm8998_power_corners)];
	struct msm8998_cpr_corner_data corners[MSM8998_CPR_MAX_CORNER_COUNT];
	struct msm8998_cpr_cluster_fuse_data fuse;
	struct msm8998_cpr_domain_config config;
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	int ret;

	ret = msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &data);
	KUNIT_ASSERT_EQ(test, ret, 0);
	fuse = data.cluster[MSM8998_CPR_POWER_CLUSTER];
	config = msm8998_power_config;
	memcpy(corner_config, msm8998_power_corners, sizeof(corner_config));
	config.corners = corner_config;

	ret = msm8998_cpr_build_corners(NULL, &fuse, corners,
					ARRAY_SIZE(corners));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cpr_build_corners(&config, NULL, corners,
					ARRAY_SIZE(corners));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cpr_build_corners(&config, &fuse, NULL,
					ARRAY_SIZE(corners));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cpr_build_corners(&config, &fuse, corners,
					config.corner_count - 1);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	config.fmax_corner[1] = config.fmax_corner[0];
	ret = msm8998_cpr_build_corners(&config, &fuse, corners,
					ARRAY_SIZE(corners));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	config = msm8998_power_config;
	config.corners = corner_config;
	config.fmax_corner[3]--;
	ret = msm8998_cpr_build_corners(&config, &fuse, corners,
					ARRAY_SIZE(corners));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	config = msm8998_power_config;
	config.corners = corner_config;
	corner_config[0].frequency_hz = 0;
	ret = msm8998_cpr_build_corners(&config, &fuse, corners,
					ARRAY_SIZE(corners));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	corner_config[0] = msm8998_power_corners[0];
	corner_config[1].frequency_hz = corner_config[0].frequency_hz;
	ret = msm8998_cpr_build_corners(&config, &fuse, corners,
					ARRAY_SIZE(corners));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	corner_config[1] = msm8998_power_corners[1];
	corner_config[0].floor_uv = corner_config[0].ceiling_uv + 1;
	ret = msm8998_cpr_build_corners(&config, &fuse, corners,
					ARRAY_SIZE(corners));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	ret = msm8998_cpr_apply_partial_binning(0, NULL,
						config.corner_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cpr_apply_partial_binning(0, corners, 0);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_cpr_apply_partial_binning(2, corners,
						config.corner_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static void msm8998_osm_invalid_table_test(struct kunit *test)
{
	struct msm8998_osm_lut_config *config;
	struct msm8998_cpr_corner_data *corners;
	struct msm8998_osm_lut_data *lut;
	struct msm8998_osm_opp *opps;
	struct msm8998_cpr_fuse_data data;
	struct msm8998_cpr_fuses fuses;
	const unsigned int config_count = ARRAY_SIZE(msm8998_power_lut_config);
	const unsigned int corner_count = ARRAY_SIZE(msm8998_power_corners);
	const unsigned int corner_capacity = MSM8998_CPR_MAX_CORNER_COUNT;
	const unsigned int lut_count = MSM8998_OSM_TABLE_SIZE;
	const unsigned int opp_capacity = MSM8998_OSM_MAX_OPP_COUNT;
	unsigned int opp_count;
	int ret;

	config = kunit_kcalloc(test, config_count, sizeof(*config), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, config);
	corners = kunit_kcalloc(test, corner_capacity, sizeof(*corners), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, corners);
	lut = kunit_kcalloc(test, lut_count, sizeof(*lut), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, lut);
	opps = kunit_kcalloc(test, opp_capacity, sizeof(*opps), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, opps);

	ret = msm8998_cpr_decode_fuses(&msm8998_cpr_known_rows, &fuses);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_calculate_fuse_data(&fuses, &data);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_cpr_build_corners(&msm8998_power_config,
					&data.cluster[MSM8998_CPR_POWER_CLUSTER],
					corners, corner_capacity);
	KUNIT_ASSERT_EQ(test, ret, 0);
	memcpy(config, msm8998_power_lut_config,
	       sizeof(msm8998_power_lut_config));

	ret = msm8998_osm_resolve_lut(NULL, config_count, corners, corner_count,
				      lut, lut_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_osm_resolve_lut(config, 0, corners,
				      corner_count, lut, lut_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_osm_resolve_lut(config, config_count, corners,
				      corner_count, lut, lut_count - 1);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	config[0].frequency_hz = 0;
	ret = msm8998_osm_resolve_lut(config, config_count, corners,
				      corner_count, lut, lut_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	config[0] = msm8998_power_lut_config[0];
	config[0].virtual_corner = 0;
	ret = msm8998_osm_resolve_lut(config, config_count, corners,
				      corner_count, lut, lut_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	config[0] = msm8998_power_lut_config[0];
	config[0].frequency_data &= ~0x00070000;
	ret = msm8998_osm_resolve_lut(config, config_count, corners,
				      corner_count, lut, lut_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	config[0] = msm8998_power_lut_config[0];
	config[0].frequency_data =
		(config[0].frequency_data & ~0x00070000) | 0x00050000;
	ret = msm8998_osm_resolve_lut(config, config_count, corners,
				      corner_count, lut, lut_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	memcpy(config, msm8998_power_lut_config,
	       sizeof(msm8998_power_lut_config));
	ret = msm8998_osm_resolve_lut(config, config_count, corners,
				      corner_count, lut, lut_count);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = msm8998_osm_build_opp_table(NULL, config_count, opps,
					  opp_capacity, &opp_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_osm_build_opp_table(lut, 0, opps, opp_capacity,
					  &opp_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = msm8998_osm_build_opp_table(lut, config_count, opps, 1,
					  &opp_count);
	KUNIT_EXPECT_EQ(test, ret, -ENOSPC);

	lut[0].config.frequency_hz = 0;
	ret = msm8998_osm_build_opp_table(lut, config_count, opps,
					  opp_capacity, &opp_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	lut[0].config = msm8998_power_lut_config[0];
	lut[0].open_loop_uv = 0;
	ret = msm8998_osm_build_opp_table(lut, config_count, opps,
					  opp_capacity, &opp_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	lut[0].open_loop_uv = corners[0].open_loop_uv;
	lut[0].config.frequency_data &= ~0x00070000;
	ret = msm8998_osm_build_opp_table(lut, config_count, opps,
					  opp_capacity, &opp_count);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static struct kunit_case msm8998_cpr_test_cases[] = {
	KUNIT_CASE(msm8998_cpr_raw_fuse_decode_test),
	KUNIT_CASE(msm8998_cpr_voltage_decode_test),
	KUNIT_CASE(msm8998_cpr_known_device_test),
	KUNIT_CASE(msm8998_cpr_power_corner_table_test),
	KUNIT_CASE(msm8998_cpr_performance_corner_table_test),
	KUNIT_CASE(msm8998_cpr_partial_binning_test),
	KUNIT_CASE(msm8998_osm_power_table_test),
	KUNIT_CASE(msm8998_osm_performance_table_test),
	KUNIT_CASE(msm8998_cpr_crossover_corner_test),
	KUNIT_CASE(msm8998_cprh_register_image_test),
	KUNIT_CASE(msm8998_osm_initialization_image_test),
	KUNIT_CASE(msm8998_osm_acd_image_test),
	KUNIT_CASE(msm8998_osm_initialization_invalid_test),
	KUNIT_CASE(msm8998_cpr_crossover_invalid_test),
	KUNIT_CASE(msm8998_cprh_invalid_register_image_test),
	KUNIT_CASE(msm8998_cpr_invalid_fuse_test),
	KUNIT_CASE(msm8998_cpr_invalid_corner_test),
	KUNIT_CASE(msm8998_osm_invalid_table_test),
	{}
};

static struct kunit_suite msm8998_cpr_test_suite = {
	.name = "qcom-cpufreq-msm8998-fuse",
	.test_cases = msm8998_cpr_test_cases,
};

kunit_test_suite(msm8998_cpr_test_suite);

MODULE_LICENSE("GPL");
