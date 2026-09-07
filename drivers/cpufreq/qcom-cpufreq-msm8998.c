// SPDX-License-Identifier: GPL-2.0-only
/*
 * Fuse calculations for the Qualcomm MSM8998 OSM CPU frequency controller.
 *
 * This file deliberately contains no MMIO access. Hardware initialization is
 * added only after these per-device calculations match known downstream data.
 */

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/string.h>

#include "qcom-cpufreq-msm8998.h"

#define MSM8998_CPR_SPEED_BIN_MAX 3
#define MSM8998_CPR_REVISION_MAX 7
#define MSM8998_CPR_VOLTAGE_FUSE_MAX 0x3f
#define MSM8998_CPR_TARGET_QUOT_MAX 0xfff
#define MSM8998_CPR_RING_OSC_MAX 0xf
#define MSM8998_CPR_QUOT_OFFSET_MAX 0x7f
#define MSM8998_CPR_VOLTAGE_STEP_UV 10000
#define MSM8998_CPR_QUOT_OFFSET_SCALE 5
#define MSM8998_CPRH_CORNER_INIT_VOLTAGE_MAX 0xff
#define MSM8998_CPRH_CORNER_FLOOR_VOLTAGE_MAX 0xff
#define MSM8998_CPRH_CORNER_QUOT_DELTA_MAX 0x1ff
#define MSM8998_CPRH_CTL_BASE_VOLTAGE_MAX 0x3ff
#define MSM8998_CPRH_CTL_MODE_SWITCH_DELAY_MAX 0xff
#define MSM8998_CPRH_CTL_VOLTAGE_MULTIPLIER_MAX 0xf
#define MSM8998_CPRH_DELTA_QUOT_STEP 4
#define MSM8998_CPRH_MODE_SWITCH_DELAY_FACTOR 4
#define MSM8998_CPRH_NANOSECONDS_PER_SECOND 1000000000ULL
#define MSM8998_OSM_CORE_COUNT_MASK GENMASK(18, 16)
#define MSM8998_OSM_CORE_COUNT_SHIFT 16
#define MSM8998_OSM_MAX_CORE_COUNT 4
#define MSM8998_OSM_VOLTAGE_MV_MAX 0xfff
#define MSM8998_OSM_VIRTUAL_CORNER_MAX 0x3f
#define MSM8998_OSM_SEQUENCE_MINUS_ONE 0xff
#define MSM8998_OSM_SEQUENCE_REGISTER(_index) (0x300 + (_index) * 4)
#define MSM8998_OSM_SEQUENCE_REGISTER1_ALIAS 0x1048
#define MSM8998_OSM_MEM_ACC_LEVEL_COUNT 3
#define MSM8998_OSM_MEM_ACC_THRESHOLD_COUNT 4
#define MSM8998_OSM_L_VALUE_MASK 0xff
#define MSM8998_OSM_APM_SEQUENCE_VALUE 0x39
#define MSM8998_OSM_ACD_CONTROL 0x4
#define MSM8998_OSM_ACD_TUNABLE_DELAY 0x8
#define MSM8998_OSM_ACD_SOFT_START_CONTROL 0x28
#define MSM8998_OSM_ACD_EXTINT_CONFIG 0x30
#define MSM8998_OSM_ACD_DCVS_SWITCH 0x34
#define MSM8998_OSM_ACD_GFMUX_CONFIG 0x3c
#define MSM8998_OSM_ACD_AUTO_TRANSFER_CONFIG 0x80
#define MSM8998_OSM_ACD_AUTO_TRANSFER_CONTROL 0x88

static const int msm8998_cpr_reference_uv
	[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
		[MSM8998_CPR_POWER_CLUSTER] = { 688000, 756000, 828000,
						1056000 },
		[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 756000, 756000, 828000,
						      1056000 },
	};

static const int msm8998_cpr_fuse_adjust_uv
	[MSM8998_CPR_CLUSTER_COUNT][MSM8998_CPR_FUSE_CORNER_COUNT] = {
		[MSM8998_CPR_POWER_CLUSTER] = { 40000, 24000, 12000, 30000 },
		[MSM8998_CPR_PERFORMANCE_CLUSTER] = { 8000, 0, 12000, 52000 },
	};

static u64 msm8998_cpr_fuse_bits(u64 row, unsigned int lsb, unsigned int width)
{
	return (row >> lsb) & ((1ULL << width) - 1);
}

int msm8998_cpr_decode_fuses(const struct msm8998_cpr_fuse_rows *rows,
			     struct msm8998_cpr_fuses *fuses)
{
	struct msm8998_cpr_cluster_fuses *power;
	struct msm8998_cpr_cluster_fuses *performance;

	if (!rows || !fuses)
		return -EINVAL;

	memset(fuses, 0, sizeof(*fuses));
	power = &fuses->cluster[MSM8998_CPR_POWER_CLUSTER];
	performance = &fuses->cluster[MSM8998_CPR_PERFORMANCE_CLUSTER];

	fuses->speed_bin = msm8998_cpr_fuse_bits(rows->row38, 29, 3);
	fuses->revision = msm8998_cpr_fuse_bits(rows->row39, 51, 3);
	fuses->force_highest_corner =
		msm8998_cpr_fuse_bits(rows->row100, 45, 1);

	power->ring_osc[0] = msm8998_cpr_fuse_bits(rows->row67, 12, 4);
	power->ring_osc[1] = msm8998_cpr_fuse_bits(rows->row67, 8, 4);
	power->ring_osc[2] = msm8998_cpr_fuse_bits(rows->row67, 4, 4);
	power->ring_osc[3] = msm8998_cpr_fuse_bits(rows->row67, 0, 4);
	power->init_voltage[0] = msm8998_cpr_fuse_bits(rows->row67, 34, 6);
	power->init_voltage[1] = msm8998_cpr_fuse_bits(rows->row67, 28, 6);
	power->init_voltage[2] = msm8998_cpr_fuse_bits(rows->row67, 22, 6);
	power->init_voltage[3] = msm8998_cpr_fuse_bits(rows->row67, 16, 6);
	power->target_quot[0] = msm8998_cpr_fuse_bits(rows->row68, 18, 12);
	power->target_quot[1] = msm8998_cpr_fuse_bits(rows->row68, 6, 12);
	power->target_quot[2] = msm8998_cpr_fuse_bits(rows->row67, 58, 6) |
				msm8998_cpr_fuse_bits(rows->row68, 0, 6) << 6;
	power->target_quot[3] = msm8998_cpr_fuse_bits(rows->row67, 46, 12);
	power->quot_offset[1] = msm8998_cpr_fuse_bits(rows->row68, 63, 1) |
				msm8998_cpr_fuse_bits(rows->row69, 0, 6) << 1;
	power->quot_offset[2] = msm8998_cpr_fuse_bits(rows->row68, 56, 7);
	power->quot_offset[3] = msm8998_cpr_fuse_bits(rows->row68, 49, 7);

	performance->ring_osc[0] = msm8998_cpr_fuse_bits(rows->row69, 26, 4);
	performance->ring_osc[1] = msm8998_cpr_fuse_bits(rows->row69, 22, 4);
	performance->ring_osc[2] = msm8998_cpr_fuse_bits(rows->row69, 18, 4);
	performance->ring_osc[3] = msm8998_cpr_fuse_bits(rows->row69, 14, 4);
	performance->init_voltage[0] =
		msm8998_cpr_fuse_bits(rows->row69, 48, 6);
	performance->init_voltage[1] =
		msm8998_cpr_fuse_bits(rows->row69, 42, 6);
	performance->init_voltage[2] =
		msm8998_cpr_fuse_bits(rows->row69, 36, 6);
	performance->init_voltage[3] =
		msm8998_cpr_fuse_bits(rows->row69, 30, 6);
	performance->target_quot[0] =
		msm8998_cpr_fuse_bits(rows->row70, 32, 12);
	performance->target_quot[1] =
		msm8998_cpr_fuse_bits(rows->row70, 20, 12);
	performance->target_quot[2] = msm8998_cpr_fuse_bits(rows->row70, 8, 12);
	performance->target_quot[3] =
		msm8998_cpr_fuse_bits(rows->row69, 60, 4) |
		msm8998_cpr_fuse_bits(rows->row70, 0, 8) << 4;
	performance->quot_offset[1] =
		msm8998_cpr_fuse_bits(rows->row71, 13, 3) |
		msm8998_cpr_fuse_bits(rows->row71, 21, 4) << 3;
	performance->quot_offset[2] = msm8998_cpr_fuse_bits(rows->row71, 6, 7);
	performance->quot_offset[3] =
		msm8998_cpr_fuse_bits(rows->row70, 63, 1) |
		msm8998_cpr_fuse_bits(rows->row71, 0, 6) << 1;

	return 0;
}

int msm8998_cpr_decode_voltage(int reference_uv, u8 fuse, int *voltage_uv)
{
	int direction;
	int steps;

	if (!voltage_uv || fuse > MSM8998_CPR_VOLTAGE_FUSE_MAX)
		return -EINVAL;

	direction = fuse & 0x20 ? -1 : 1;
	steps = fuse & 0x1f;
	*voltage_uv =
		reference_uv + direction * steps * MSM8998_CPR_VOLTAGE_STEP_UV;

	return 0;
}

static int
msm8998_cpr_validate_cluster_fuses(const struct msm8998_cpr_cluster_fuses *fuses)
{
	int i;

	for (i = 0; i < MSM8998_CPR_FUSE_CORNER_COUNT; i++) {
		if (fuses->init_voltage[i] > MSM8998_CPR_VOLTAGE_FUSE_MAX ||
		    fuses->target_quot[i] > MSM8998_CPR_TARGET_QUOT_MAX ||
		    fuses->ring_osc[i] > MSM8998_CPR_RING_OSC_MAX ||
		    fuses->quot_offset[i] > MSM8998_CPR_QUOT_OFFSET_MAX)
			return -EINVAL;
	}

	return 0;
}

int msm8998_cpr_calculate_fuse_data(const struct msm8998_cpr_fuses *fuses,
				    struct msm8998_cpr_fuse_data *data)
{
	int cluster;
	int corner;
	int ret;

	if (!fuses || !data || fuses->speed_bin > MSM8998_CPR_SPEED_BIN_MAX ||
	    fuses->revision > MSM8998_CPR_REVISION_MAX ||
	    fuses->force_highest_corner > 1)
		return -EINVAL;

	for (cluster = 0; cluster < MSM8998_CPR_CLUSTER_COUNT; cluster++) {
		ret = msm8998_cpr_validate_cluster_fuses(&fuses->cluster[cluster]);
		if (ret)
			return ret;
	}

	memset(data, 0, sizeof(*data));
	data->speed_bin = fuses->speed_bin;
	data->revision = fuses->revision;
	data->force_highest_corner = fuses->force_highest_corner;
	data->fuse_combo = fuses->revision + 8 * fuses->speed_bin;

	for (cluster = 0; cluster < MSM8998_CPR_CLUSTER_COUNT; cluster++) {
		const struct msm8998_cpr_cluster_fuses *cluster_fuses =
			&fuses->cluster[cluster];
		struct msm8998_cpr_cluster_fuse_data *cluster_data =
			&data->cluster[cluster];

		for (corner = 0; corner < MSM8998_CPR_FUSE_CORNER_COUNT;
		     corner++) {
			int reference_uv =
				msm8998_cpr_reference_uv[cluster][corner];
			int *fused_uv =
				&cluster_data->fused_open_loop_uv[corner];
			u8 voltage_fuse = cluster_fuses->init_voltage[corner];

			ret = msm8998_cpr_decode_voltage(reference_uv, voltage_fuse,
							 fused_uv);
			if (ret)
				return ret;

			cluster_data->adjusted_fused_open_loop_uv[corner] =
				cluster_data->fused_open_loop_uv[corner] +
				msm8998_cpr_fuse_adjust_uv[cluster][corner];
			cluster_data->target_quot[corner] =
				cluster_fuses->target_quot[corner];
			cluster_data->quot_offset[corner] =
				cluster_fuses->quot_offset[corner] *
				MSM8998_CPR_QUOT_OFFSET_SCALE;
			cluster_data->ring_osc[corner] =
				cluster_fuses->ring_osc[corner];
		}

		for (corner = 1; corner < MSM8998_CPR_FUSE_CORNER_COUNT;
		     corner++) {
			if (cluster_data->adjusted_fused_open_loop_uv[corner] <
			    cluster_data
				    ->adjusted_fused_open_loop_uv[corner - 1])
				cluster_data
					->adjusted_fused_open_loop_uv[corner] =
					cluster_data->adjusted_fused_open_loop_uv
						[corner - 1];
		}
	}

	return 0;
}

static int msm8998_cpr_round_up_uv(int voltage_uv, u32 step_uv, int *rounded_uv)
{
	u64 rounded;

	if (!rounded_uv || voltage_uv <= 0 || !step_uv || step_uv > INT_MAX)
		return -EINVAL;

	rounded = div64_u64((u64)voltage_uv + step_uv - 1, step_uv) * step_uv;
	if (rounded > INT_MAX)
		return -ERANGE;

	*rounded_uv = rounded;

	return 0;
}

static int msm8998_cpr_interpolate(u32 x1, int y1, u32 x2, int y2, u32 x,
				   int *y)
{
	u64 delta;

	if (!y || x1 >= x2 || y1 > y2 || x1 > x || x > x2)
		return -EINVAL;

	delta = (u64)(x2 - x) * (y2 - y1);
	delta = div64_u64(delta, x2 - x1);
	*y = y2 - delta;

	return 0;
}

static int
msm8998_cpr_validate_config(const struct msm8998_cpr_domain_config *config,
			    const struct msm8998_cpr_cluster_fuse_data *fuse,
			    unsigned int corner_capacity)
{
	unsigned int i;

	if (!fuse || !config || !config->corners ||
	    !config->ro_scaling_factor || !config->corner_count ||
	    config->corner_count > MSM8998_CPR_MAX_CORNER_COUNT ||
	    corner_capacity < config->corner_count || !config->step_uv ||
	    config->step_uv > INT_MAX)
		return -EINVAL;

	for (i = 0; i < MSM8998_CPR_FUSE_CORNER_COUNT; i++) {
		if (config->fmax_corner[i] >= config->corner_count ||
		    (i &&
		     config->fmax_corner[i] <= config->fmax_corner[i - 1]) ||
		    fuse->adjusted_fused_open_loop_uv[i] <= 0 ||
		    (i && fuse->adjusted_fused_open_loop_uv[i] <
				  fuse->adjusted_fused_open_loop_uv[i - 1]) ||
		    fuse->ring_osc[i] >= MSM8998_CPR_RING_OSC_COUNT ||
		    !config->ro_scaling_factor
			[i * MSM8998_CPR_RING_OSC_COUNT + fuse->ring_osc[i]])
			return -EINVAL;
	}

	if (config->fmax_corner[MSM8998_CPR_FUSE_CORNER_COUNT - 1] !=
	    config->corner_count - 1)
		return -EINVAL;

	for (i = 0; i < config->corner_count; i++) {
		const struct msm8998_cpr_corner_config *corner =
			&config->corners[i];

		if (!corner->frequency_hz || corner->floor_uv <= 0 ||
		    corner->ceiling_uv <= 0 ||
		    corner->floor_uv > corner->ceiling_uv ||
		    !corner->max_floor_to_ceiling_uv ||
		    (i && corner->frequency_hz <=
				  config->corners[i - 1].frequency_hz))
			return -EINVAL;
	}

	return 0;
}

static int msm8998_cpr_quot_adjustment(u32 ro_scale, int voltage_uv,
				       int *adjustment)
{
	s64 value;

	if (!ro_scale || !adjustment)
		return -EINVAL;

	value = div_s64((s64)ro_scale * voltage_uv, 1000000);
	if (value < INT_MIN || value > INT_MAX)
		return -ERANGE;

	*adjustment = value;

	return 0;
}

static int msm8998_cpr_adjust_quotient(u32 quotient, int adjustment, u32 *adjusted)
{
	s64 value = (s64)quotient + adjustment;

	if (!adjusted || value <= 0 || value > MSM8998_CPR_TARGET_QUOT_MAX)
		return -ERANGE;

	*adjusted = value;

	return 0;
}

static int msm8998_cpr_build_quotients(const struct msm8998_cpr_domain_config *config,
				       const struct msm8998_cpr_cluster_fuse_data *fuse,
				       struct msm8998_cpr_corner_data *corners)
{
	u32 quot_low[MSM8998_CPR_FUSE_CORNER_COUNT] = {};
	u32 quot_high[MSM8998_CPR_FUSE_CORNER_COUNT] = {};
	unsigned int fuse_corner;
	unsigned int i;
	int adjustment;
	int ret;

	if (!config->allow_quotient_interpolation) {
		for (i = 0; i < config->corner_count; i++) {
			int adjust_uv;
			u32 fuse_quot;
			u32 *target_quot;
			u32 ro_scale;

			fuse_corner = corners[i].fuse_corner;
			ro_scale = config->ro_scaling_factor
				[fuse_corner * MSM8998_CPR_RING_OSC_COUNT +
				 fuse->ring_osc[fuse_corner]];
			adjust_uv =
				config->fuse_closed_loop_adjust_uv[fuse_corner] +
				config->corners[i].closed_loop_adjust_uv;
			ret = msm8998_cpr_quot_adjustment(ro_scale, adjust_uv, &adjustment);
			if (ret)
				return ret;
			fuse_quot = fuse->target_quot[fuse_corner];
			target_quot = &corners[i].target_quot;
			ret = msm8998_cpr_adjust_quotient(fuse_quot, adjustment, target_quot);
			if (ret)
				return ret;
			corners[i].ring_osc = fuse->ring_osc[fuse_corner];
		}

		return 0;
	}

	ret = msm8998_cpr_quot_adjustment(config->ro_scaling_factor
					 [fuse->ring_osc[0]],
		config->fuse_closed_loop_adjust_uv[0], &adjustment);
	if (ret)
		return ret;
	ret = msm8998_cpr_adjust_quotient(fuse->target_quot[0], adjustment,
					  &quot_high[0]);
	if (ret)
		return ret;
	quot_low[0] = quot_high[0];
	for (i = 0; i <= config->fmax_corner[0]; i++) {
		corners[i].target_quot = quot_high[0];
		corners[i].ring_osc = fuse->ring_osc[0];
	}

	for (fuse_corner = 1;
	     fuse_corner < MSM8998_CPR_FUSE_CORNER_COUNT; fuse_corner++) {
		quot_high[fuse_corner] = fuse->target_quot[fuse_corner];
		if (fuse->ring_osc[fuse_corner] ==
		    fuse->ring_osc[fuse_corner - 1]) {
			quot_low[fuse_corner] = quot_high[fuse_corner - 1];
		} else {
			if (quot_high[fuse_corner] < fuse->quot_offset[fuse_corner])
				return -ERANGE;
			quot_low[fuse_corner] = quot_high[fuse_corner] -
						 fuse->quot_offset[fuse_corner];
		}
		quot_high[fuse_corner] = max(quot_high[fuse_corner], quot_low[fuse_corner]);
	}

	for (fuse_corner = 1;
	     fuse_corner < MSM8998_CPR_FUSE_CORNER_COUNT; fuse_corner++) {
		int fuse_adjust_uv =
			config->fuse_closed_loop_adjust_uv[fuse_corner];
		u32 ro_scale = config->ro_scaling_factor
			[fuse_corner * MSM8998_CPR_RING_OSC_COUNT +
			 fuse->ring_osc[fuse_corner]];

		ret = msm8998_cpr_quot_adjustment(ro_scale, fuse_adjust_uv, &adjustment);
		if (ret)
			return ret;
		ret = msm8998_cpr_adjust_quotient(quot_high[fuse_corner],
						  adjustment,
						  &quot_high[fuse_corner]);
		if (ret)
			return ret;

		if (fuse->ring_osc[fuse_corner] ==
		    fuse->ring_osc[fuse_corner - 1]) {
			quot_low[fuse_corner] = quot_high[fuse_corner - 1];
		} else {
			fuse_adjust_uv =
				config->fuse_closed_loop_adjust_uv[fuse_corner - 1];
			ret = msm8998_cpr_quot_adjustment(ro_scale, fuse_adjust_uv, &adjustment);
			if (ret)
				return ret;
			ret = msm8998_cpr_adjust_quotient(quot_low[fuse_corner],
							  adjustment,
							  &quot_low[fuse_corner]);
			if (ret)
				return ret;
		}
		quot_high[fuse_corner] = max(quot_high[fuse_corner], quot_low[fuse_corner]);
	}

	for (fuse_corner = 1;
	     fuse_corner < MSM8998_CPR_FUSE_CORNER_COUNT; fuse_corner++) {
		unsigned int low = config->fmax_corner[fuse_corner - 1];
		unsigned int high = config->fmax_corner[fuse_corner];

		for (i = low + 1; i <= high; i++) {
			int quotient;

			ret = msm8998_cpr_interpolate(corners[low].frequency_hz,
						      quot_low[fuse_corner],
						      corners[high].frequency_hz,
						      quot_high[fuse_corner],
						      corners[i].frequency_hz,
						      &quotient);
			if (ret)
				return ret;
			corners[i].target_quot = quotient;
			corners[i].ring_osc = fuse->ring_osc[fuse_corner];
		}
	}

	for (i = 0; i < config->corner_count; i++) {
		int corner_adjust_uv;
		u32 ro_scale;
		u32 adjusted;

		fuse_corner = corners[i].fuse_corner;
		ro_scale = config->ro_scaling_factor
			[fuse_corner * MSM8998_CPR_RING_OSC_COUNT +
			 corners[i].ring_osc];
		corner_adjust_uv = config->corners[i].closed_loop_adjust_uv;
		ret = msm8998_cpr_quot_adjustment(ro_scale, corner_adjust_uv, &adjustment);
		if (ret)
			return ret;
		ret = msm8998_cpr_adjust_quotient(corners[i].target_quot,
						  adjustment, &adjusted);
		if (ret)
			return ret;
		corners[i].target_quot = adjusted;

		if (i && corners[i].ring_osc == corners[i - 1].ring_osc &&
		    corners[i].target_quot < corners[i - 1].target_quot)
			corners[i].target_quot = corners[i - 1].target_quot;
	}

	return 0;
}

static int msm8998_cpr_adjust_threshold(struct msm8998_cpr_corner_data *corners,
					unsigned int corner_count,
					u32 threshold_uv, u32 hysteresis_uv,
					u32 step_uv)
{
	unsigned int i;
	int hysteresis;
	int threshold;
	int step;
	int ret;

	if (!threshold_uv)
		return 0;
	if (threshold_uv > INT_MAX || hysteresis_uv > INT_MAX ||
	    step_uv > INT_MAX)
		return -EINVAL;

	step = step_uv;
	ret = msm8998_cpr_round_up_uv(threshold_uv, step_uv, &threshold);
	if (ret)
		return ret;

	if (hysteresis_uv) {
		ret = msm8998_cpr_round_up_uv(hysteresis_uv, step_uv,
					      &hysteresis);
		if (ret)
			return ret;
	} else {
		hysteresis = 0;
	}

	for (i = 0; i < corner_count; i++) {
		struct msm8998_cpr_corner_data *corner = &corners[i];

		if (threshold <= corner->floor_uv ||
		    threshold > corner->ceiling_uv)
			continue;

		if (corner->open_loop_uv >= threshold) {
			corner->floor_uv =
				max(corner->floor_uv, threshold - hysteresis);
			corner->open_loop_uv =
				max(corner->open_loop_uv, corner->floor_uv);
		} else {
			corner->ceiling_uv = threshold - step;
		}

		if (corner->floor_uv > corner->ceiling_uv ||
		    corner->open_loop_uv < corner->floor_uv ||
		    corner->open_loop_uv > corner->ceiling_uv)
			return -EINVAL;
	}

	return 0;
}

int msm8998_cpr_build_corners(const struct msm8998_cpr_domain_config *config,
			      const struct msm8998_cpr_cluster_fuse_data *fuse,
			      struct msm8998_cpr_corner_data *corners,
			      unsigned int corner_capacity)
{
	unsigned int fuse_corner;
	unsigned int i;
	int ret;

	if (!corners)
		return -EINVAL;

	ret = msm8998_cpr_validate_config(config, fuse, corner_capacity);
	if (ret)
		return ret;

	memset(corners, 0, sizeof(*corners) * config->corner_count);
	for (i = 0, fuse_corner = 0; i < config->corner_count; i++) {
		const struct msm8998_cpr_corner_config *source =
			&config->corners[i];
		struct msm8998_cpr_corner_data *corner = &corners[i];

		while (i > config->fmax_corner[fuse_corner])
			fuse_corner++;

		corner->frequency_hz = source->frequency_hz;
		corner->fuse_corner = fuse_corner;
		ret = msm8998_cpr_round_up_uv(source->floor_uv, config->step_uv,
					      &corner->floor_uv);
		if (ret)
			return ret;
		ret = msm8998_cpr_round_up_uv(source->ceiling_uv,
					      config->step_uv,
					      &corner->ceiling_uv);
		if (ret)
			return ret;
		if (corner->floor_uv > corner->ceiling_uv)
			return -EINVAL;
	}

	for (i = 0; i <= config->fmax_corner[0]; i++)
		corners[i].open_loop_uv = fuse->adjusted_fused_open_loop_uv[0];

	for (fuse_corner = 1; fuse_corner < MSM8998_CPR_FUSE_CORNER_COUNT;
	     fuse_corner++) {
		unsigned int low = config->fmax_corner[fuse_corner - 1];
		unsigned int high = config->fmax_corner[fuse_corner];
		int low_uv = fuse->adjusted_fused_open_loop_uv[fuse_corner - 1];
		int high_uv = fuse->adjusted_fused_open_loop_uv[fuse_corner];

		for (i = low + 1; i <= high; i++) {
			ret = msm8998_cpr_interpolate(corners[low].frequency_hz,
						      low_uv,
						      corners[high].frequency_hz,
						      high_uv,
						      corners[i].frequency_hz,
						      &corners[i].open_loop_uv);
			if (ret)
				return ret;
		}
	}

	for (i = 0; i < config->corner_count; i++) {
		s64 adjusted = (s64)corners[i].open_loop_uv +
			       config->corners[i].open_loop_adjust_uv;

		if (adjusted <= 0 || adjusted > INT_MAX)
			return -ERANGE;
		corners[i].open_loop_uv = adjusted;
	}

	for (i = 1; i < config->corner_count; i++)
		corners[i].open_loop_uv = max(corners[i].open_loop_uv,
					      corners[i - 1].open_loop_uv);

	for (i = 0; i < config->corner_count; i++) {
		ret = msm8998_cpr_round_up_uv(corners[i].open_loop_uv,
					      config->step_uv,
					      &corners[i].open_loop_uv);
		if (ret)
			return ret;
		corners[i].open_loop_uv = clamp(corners[i].open_loop_uv,
						corners[i].floor_uv,
						corners[i].ceiling_uv);
	}

	ret = msm8998_cpr_adjust_threshold(corners, config->corner_count,
					   config->apm_threshold_uv,
					   config->apm_hysteresis_uv,
					   config->step_uv);
	if (ret)
		return ret;
	ret = msm8998_cpr_adjust_threshold(corners, config->corner_count,
					   config->mem_acc_threshold_uv, 0,
					   config->step_uv);
	if (ret)
		return ret;

	if (config->scaled_open_loop_as_ceiling) {
		for (i = 0; i < config->corner_count; i++)
			corners[i].ceiling_uv = corners[i].open_loop_uv;
	}

	for (i = 0; i < config->corner_count; i++) {
		s64 floor = (s64)corners[i].ceiling_uv -
			    config->corners[i].max_floor_to_ceiling_uv;
		int range_floor;

		if (floor <= 0 || floor > INT_MAX)
			return -ERANGE;
		ret = msm8998_cpr_round_up_uv(floor, config->step_uv,
					      &range_floor);
		if (ret)
			return ret;
		corners[i].floor_uv = max(corners[i].floor_uv, range_floor);
		corners[i].open_loop_uv =
			max(corners[i].open_loop_uv, corners[i].floor_uv);
	}

	for (i = 1; i < config->corner_count; i++) {
		if (corners[i].floor_uv >= corners[i - 1].floor_uv)
			continue;

		corners[i].floor_uv = corners[i - 1].floor_uv;
		corners[i].open_loop_uv =
			max(corners[i].open_loop_uv, corners[i].floor_uv);
		corners[i].ceiling_uv =
			max(corners[i].ceiling_uv, corners[i].floor_uv);
	}

	return msm8998_cpr_build_quotients(config, fuse, corners);
}

/* Apply after all per-corner parameters are populated; frequencies stay local. */
int msm8998_cpr_apply_partial_binning(u8 force_highest_corner,
				      struct msm8998_cpr_corner_data *corners,
				      unsigned int corner_count)
{
	struct msm8998_cpr_corner_data highest;
	unsigned int i;

	if (!corners || !corner_count ||
	    corner_count > MSM8998_CPR_MAX_CORNER_COUNT ||
	    force_highest_corner > 1)
		return -EINVAL;

	for (i = 0; i < corner_count; i++) {
		if (!corners[i].frequency_hz || corners[i].floor_uv <= 0 ||
		    corners[i].open_loop_uv < corners[i].floor_uv ||
		    corners[i].ceiling_uv < corners[i].open_loop_uv ||
		    corners[i].fuse_corner >= MSM8998_CPR_FUSE_CORNER_COUNT ||
		    (i && corners[i].frequency_hz <= corners[i - 1].frequency_hz))
			return -EINVAL;
	}

	if (!force_highest_corner)
		return 0;

	highest = corners[corner_count - 1];
	for (i = 0; i < corner_count - 1; i++) {
		u32 frequency_hz = corners[i].frequency_hz;

		corners[i] = highest;
		corners[i].frequency_hz = frequency_hz;
	}

	return 0;
}

int msm8998_cpr_append_crossover_corners(const struct msm8998_cpr_domain_config *config,
					 struct msm8998_cpr_corner_data *corners,
					 unsigned int corner_capacity,
					 unsigned int *corner_count)
{
	const u32 crossover_uv[] = {
		config ? config->apm_crossover_uv : 0,
		config ? config->mem_acc_crossover_uv : 0,
	};
	unsigned int additions = 0;
	unsigned int count;
	unsigned int i;

	if (!config || !corners || !corner_count ||
	    *corner_count != config->corner_count ||
	    *corner_count > MSM8998_CPR_MAX_CORNER_COUNT)
		return -EINVAL;
	if (corner_capacity < *corner_count)
		return -ENOSPC;

	for (i = 0; i < ARRAY_SIZE(crossover_uv); i++) {
		if (crossover_uv[i] > INT_MAX)
			return -ERANGE;
		if (crossover_uv[i])
			additions++;
	}
	if (additions > corner_capacity - *corner_count ||
	    *corner_count + additions > MSM8998_CPR_MAX_CORNER_COUNT)
		return -ENOSPC;

	count = *corner_count;
	for (i = 0; i < ARRAY_SIZE(crossover_uv); i++) {
		struct msm8998_cpr_corner_data *corner;

		if (!crossover_uv[i])
			continue;
		corner = &corners[count++];
		memset(corner, 0, sizeof(*corner));
		corner->floor_uv = crossover_uv[i];
		corner->open_loop_uv = crossover_uv[i];
		corner->ceiling_uv = crossover_uv[i];
		corner->use_open_loop = true;
	}
	*corner_count = count;

	return 0;
}

static int msm8998_cprh_validate_controller(const struct msm8998_cprh_controller_config *config)
{
	if (!config || !config->clock_rate_hz || !config->sensor_time_ns ||
	    !config->loop_time_ns || !config->base_uv || !config->step_uv ||
	    config->base_uv > INT_MAX || config->step_uv > INT_MAX ||
	    !config->sensor_count || config->idle_clocks > 0x1f ||
	    config->count_mode > 0x3 || config->count_repeat > 0x7fffff ||
	    config->step_quot_init_min > 0x3f ||
	    config->step_quot_init_max > 0x3f ||
	    config->consecutive_up > 0xf || config->consecutive_down > 0xf ||
	    config->up_threshold > 0x1f || config->down_threshold > 0x1f ||
	    config->down_error_step_limit > 0x1f ||
	    config->up_error_step_limit > 0x1f)
		return -EINVAL;

	return 0;
}

int msm8998_cprh_build_register_image(const struct msm8998_cprh_controller_config *config,
				      const struct msm8998_cpr_corner_data *corners,
				      unsigned int corner_count, bool closed_loop,
				      struct msm8998_cprh_register_image *image)
{
	u64 base_voltage_steps;
	u64 continuous_count;
	u64 gcnt;
	u64 mode_switch_delay;
	u64 settle_count;
	u64 voltage_multiplier;
	u32 pmic_step_size = 1;
	u64 cycles;
	int base_uv;
	unsigned int i;
	unsigned int ro;
	int ret;

	ret = msm8998_cprh_validate_controller(config);
	if (ret)
		return ret;
	if (!corners || !corner_count ||
	    corner_count > MSM8998_CPR_MAX_CORNER_COUNT || !image)
		return -EINVAL;

	base_uv = config->base_uv;
	base_voltage_steps = DIV_ROUND_UP_ULL(config->base_uv,
					      config->step_uv);
	voltage_multiplier = DIV_ROUND_UP_ULL(config->step_uv, 1000);
	cycles = (u64)config->clock_rate_hz * config->corner_switch_delay_time_ns;
	mode_switch_delay = div64_u64(cycles, MSM8998_CPRH_NANOSECONDS_PER_SECOND);
	mode_switch_delay /= MSM8998_CPRH_MODE_SWITCH_DELAY_FACTOR;
	cycles = (u64)config->clock_rate_hz * config->voltage_settling_time_ns;
	settle_count = div64_u64(cycles, MSM8998_CPRH_NANOSECONDS_PER_SECOND);
	if (config->saw_use_unit_mv)
		pmic_step_size = config->step_uv / 1000;
	cycles = (u64)config->clock_rate_hz * config->sensor_time_ns;
	gcnt = DIV_ROUND_UP_ULL(cycles, MSM8998_CPRH_NANOSECONDS_PER_SECOND) - 1;
	cycles = (u64)config->clock_rate_hz * config->loop_time_ns;
	continuous_count = div64_u64(cycles, MSM8998_CPRH_NANOSECONDS_PER_SECOND);
	if (base_voltage_steps > MSM8998_CPRH_CTL_BASE_VOLTAGE_MAX ||
	    voltage_multiplier > MSM8998_CPRH_CTL_VOLTAGE_MULTIPLIER_MAX ||
	    mode_switch_delay > MSM8998_CPRH_CTL_MODE_SWITCH_DELAY_MAX ||
	    settle_count > 0x7ff || pmic_step_size > 0x1f ||
	    gcnt > U32_MAX || !continuous_count || continuous_count > U32_MAX)
		return -ERANGE;

	memset(image, 0, sizeof(*image));
	image->corner_count = corner_count;
	image->sensor_owner_count = config->sensor_count;
	image->ro_mask = GENMASK(MSM8998_CPR_RING_OSC_COUNT - 1, 0);

	for (ro = 0; ro < MSM8998_CPR_RING_OSC_COUNT; ro++) {
		u32 minimum = UINT_MAX;

		for (i = 0; i < corner_count; i++) {
			if (corners[i].target_quot && corners[i].ring_osc == ro)
				minimum = min(minimum, corners[i].target_quot);
		}
		if (minimum != UINT_MAX) {
			image->target_quot[ro] = minimum;
			image->gcnt_ro_mask |= BIT(ro);
			image->ro_mask &= ~BIT(ro);
		}
	}

	for (i = 0; i < corner_count; i++) {
		u32 delta_quot_steps = 0;
		u64 floor_voltage_steps;
		u64 initial_voltage_steps;
		u32 value;

		if (corners[i].floor_uv <= 0 || corners[i].open_loop_uv <= 0 ||
		    corners[i].ceiling_uv <= 0 ||
		    corners[i].floor_uv < base_uv ||
		    corners[i].open_loop_uv < base_uv ||
		    corners[i].floor_uv > corners[i].open_loop_uv ||
		    corners[i].open_loop_uv > corners[i].ceiling_uv)
			return -EINVAL;
		if (corners[i].frequency_hz) {
			if (!corners[i].target_quot ||
			    corners[i].target_quot > MSM8998_CPR_TARGET_QUOT_MAX ||
			    corners[i].ring_osc >= MSM8998_CPR_RING_OSC_COUNT)
				return -EINVAL;
			delta_quot_steps = DIV_ROUND_UP(corners[i].target_quot -
				image->target_quot[corners[i].ring_osc],
				MSM8998_CPRH_DELTA_QUOT_STEP);
		} else if (corners[i].target_quot || !corners[i].use_open_loop) {
			return -EINVAL;
		}

		initial_voltage_steps = DIV_ROUND_UP_ULL(corners[i].open_loop_uv -
			base_uv, config->step_uv);
		floor_voltage_steps = DIV_ROUND_UP_ULL(corners[i].floor_uv -
			base_uv, config->step_uv);
		if (initial_voltage_steps >
				MSM8998_CPRH_CORNER_INIT_VOLTAGE_MAX ||
		    floor_voltage_steps >
				MSM8998_CPRH_CORNER_FLOOR_VOLTAGE_MAX ||
		    delta_quot_steps > MSM8998_CPRH_CORNER_QUOT_DELTA_MAX)
			return -ERANGE;

		value = initial_voltage_steps | floor_voltage_steps << 8 |
			delta_quot_steps << 16 | corners[i].ring_osc << 25;
		if (!closed_loop || corners[i].use_open_loop)
			value |= BIT(29);
		image->corner[i] = value;
	}

	image->gcnt = gcnt;
	image->cpr_timer_auto_cont = continuous_count;
	image->cpr_ctl = BIT(0) | config->idle_clocks << 1 |
		config->count_mode << 6 | config->count_repeat << 9;
	image->cpr_step_quot = config->step_quot_init_min |
		config->step_quot_init_max << 6;
	image->threshold = config->consecutive_down |
		config->consecutive_up << 4 | config->down_threshold << 8 |
		config->up_threshold << 13;
	image->margin_adjust_ctl = pmic_step_size << 12;
	if (config->use_dynamic_step_quot)
		image->margin_adjust_ctl |= BIT(7);
	if (closed_loop)
		image->margin_adjust_ctl |= BIT(4);
	image->saw_error_step_limit = config->up_error_step_limit |
		config->down_error_step_limit << 5;
	image->margin_temp_core_timers = settle_count << 18;
	image->cprh_ctl = BIT(0) | base_voltage_steps << 1 |
		mode_switch_delay << 17 | voltage_multiplier << 25;

	return 0;
}

int msm8998_osm_resolve_lut(const struct msm8998_osm_lut_config *config,
			    unsigned int config_count,
			    const struct msm8998_cpr_corner_data *corners,
			    unsigned int corner_count,
			    struct msm8998_osm_lut_data *lut,
			    unsigned int lut_count)
{
	unsigned int i;
	unsigned int source;

	if (!config || !config_count || config_count > MSM8998_OSM_TABLE_SIZE ||
	    !corners || !corner_count ||
	    corner_count > MSM8998_CPR_MAX_CORNER_COUNT || !lut ||
	    lut_count < MSM8998_OSM_TABLE_SIZE)
		return -EINVAL;

	memset(lut, 0, sizeof(*lut) * MSM8998_OSM_TABLE_SIZE);
	for (i = 0; i < MSM8998_OSM_TABLE_SIZE; i++) {
		unsigned int core_count;
		unsigned int virtual_corner;

		source = min(i, config_count - 1);
		virtual_corner = config[source].virtual_corner;
		core_count = (config[source].frequency_data &
			      MSM8998_OSM_CORE_COUNT_MASK) >>
			     MSM8998_OSM_CORE_COUNT_SHIFT;
		if (!config[source].frequency_hz || !virtual_corner ||
		    virtual_corner > corner_count || !core_count ||
		    core_count > MSM8998_OSM_MAX_CORE_COUNT ||
		    corners[virtual_corner - 1].open_loop_uv <= 0)
			return -EINVAL;

		lut[i].config = config[source];
		lut[i].open_loop_uv = corners[virtual_corner - 1].open_loop_uv;
	}

	return 0;
}

int msm8998_osm_build_opp_table(const struct msm8998_osm_lut_data *lut,
				unsigned int config_count,
				struct msm8998_osm_opp *opps,
				unsigned int opp_capacity,
				unsigned int *opp_count)
{
	u32 absolute_max_frequency;
	unsigned int count = 0;
	unsigned int core_count;
	unsigned int i;

	if (!lut || !config_count || config_count > MSM8998_OSM_TABLE_SIZE ||
	    !opps || !opp_capacity || !opp_count)
		return -EINVAL;

	*opp_count = 0;
	for (i = 0; i < config_count; i++) {
		core_count = (lut[i].config.frequency_data &
			      MSM8998_OSM_CORE_COUNT_MASK) >>
			     MSM8998_OSM_CORE_COUNT_SHIFT;
		if (!lut[i].config.frequency_hz || lut[i].open_loop_uv <= 0 ||
		    !lut[i].config.virtual_corner ||
		    lut[i].config.virtual_corner > MSM8998_CPR_MAX_CORNER_COUNT ||
		    !core_count || core_count > MSM8998_OSM_MAX_CORE_COUNT)
			return -EINVAL;
	}

	absolute_max_frequency = lut[config_count - 1].config.frequency_hz;
	for (i = 0; i < config_count; i++) {
		core_count = (lut[i].config.frequency_data &
			      MSM8998_OSM_CORE_COUNT_MASK) >>
			     MSM8998_OSM_CORE_COUNT_SHIFT;
		if (core_count != MSM8998_OSM_MAX_CORE_COUNT)
			continue;

		if (count >= opp_capacity)
			return -ENOSPC;
		/* Duplicate frequencies use distinct single/quad-core LUT rows. */
		opps[count].frequency_hz = lut[i].config.frequency_hz;
		opps[count].open_loop_uv = lut[i].open_loop_uv;
		opps[count].virtual_corner = lut[i].config.virtual_corner;
		opps[count].table_index = i;
		count++;

		if (lut[i].config.frequency_hz == absolute_max_frequency) {
			*opp_count = count;
			return 0;
		}
	}

	if (count >= opp_capacity)
		return -ENOSPC;
	/* Preserve the final LUT row as the maximum-frequency fallback. */
	opps[count].frequency_hz = lut[config_count - 1].config.frequency_hz;
	opps[count].open_loop_uv = lut[config_count - 1].open_loop_uv;
	opps[count].virtual_corner = lut[config_count - 1].config.virtual_corner;
	opps[count].table_index = config_count - 1;
	*opp_count = count + 1;

	return 0;
}

static bool msm8998_osm_lut_rows_equal(const struct msm8998_osm_lut_data *left,
					const struct msm8998_osm_lut_data *right)
{
	return left->config.frequency_hz == right->config.frequency_hz &&
		left->config.frequency_data == right->config.frequency_data &&
		left->config.override_data == right->config.override_data &&
		left->config.spare_data == right->config.spare_data &&
		left->config.virtual_corner == right->config.virtual_corner &&
		left->open_loop_uv == right->open_loop_uv;
}

int msm8998_osm_build_hw_table_image(const struct msm8998_osm_lut_data *lut,
				     unsigned int config_count,
				     struct msm8998_osm_hw_table_image *image)
{
	u32 previous_spare = 0;
	u8 previous_virtual_corner = 0;
	unsigned int i;

	if (!lut || !config_count || config_count > MSM8998_OSM_TABLE_SIZE ||
	    !image)
		return -EINVAL;

	memset(image, 0, sizeof(*image));
	for (i = 0; i < MSM8998_OSM_TABLE_SIZE; i++) {
		const struct msm8998_osm_lut_data *row = &lut[i];
		struct msm8998_osm_hw_table_entry *entry = &image->entry[i];
		u32 voltage_mv;
		u8 virtual_corner;

		if (i >= config_count &&
		    !msm8998_osm_lut_rows_equal(row, &lut[config_count - 1]))
			return -EINVAL;
		if (!row->config.frequency_hz || !row->config.virtual_corner ||
		    row->config.virtual_corner > MSM8998_OSM_VIRTUAL_CORNER_MAX + 1 ||
		    row->open_loop_uv < 1000)
			return -EINVAL;

		virtual_corner = row->config.virtual_corner - 1;
		voltage_mv = row->open_loop_uv / 1000;
		if (voltage_mv > MSM8998_OSM_VOLTAGE_MV_MAX)
			return -ERANGE;
		if (i && virtual_corner == previous_virtual_corner &&
		    row->config.spare_data != previous_spare)
			return -EINVAL;

		entry->index = i;
		entry->frequency_data = row->config.frequency_data;
		entry->voltage_data = virtual_corner << 16 | voltage_mv;
		entry->override_data = row->config.override_data;
		entry->spare_data = row->config.spare_data;
		previous_virtual_corner = virtual_corner;
		previous_spare = row->config.spare_data;
	}

	return 0;
}

static int msm8998_osm_find_threshold_vc(
			const struct msm8998_osm_lut_data *lut,
			const struct msm8998_cpr_corner_data *corners,
			unsigned int corner_count, u32 threshold_uv,
			u8 *threshold_pre_vc, u8 *threshold_vc)
{
	unsigned int i;

	if (!threshold_uv || threshold_uv > INT_MAX)
		return -EINVAL;

	for (i = 0; i < MSM8998_OSM_TABLE_SIZE; i++) {
		u8 virtual_corner;

		if (!lut[i].config.virtual_corner ||
		    lut[i].config.virtual_corner > corner_count)
			return -EINVAL;
		virtual_corner = lut[i].config.virtual_corner - 1;
		if (corners[virtual_corner].ceiling_uv < (int)threshold_uv)
			continue;

		*threshold_vc = virtual_corner;
		*threshold_pre_vc = virtual_corner ? virtual_corner - 1 :
			MSM8998_OSM_SEQUENCE_MINUS_ONE;
		return 0;
	}

	*threshold_vc = MSM8998_OSM_TABLE_SIZE - 1;
	*threshold_pre_vc = *threshold_vc - 1;

	return 0;
}

static int msm8998_osm_append_sequence_write(
			struct msm8998_osm_sequence_image *image, u32 offset,
			u32 value, enum msm8998_osm_register_access access)
{
	struct msm8998_osm_register_write *write;

	if (image->write_count >= MSM8998_OSM_TZ_SEQUENCE_MAX_WRITES)
		return -ENOSPC;

	write = &image->write[image->write_count++];
	write->offset = offset;
	write->value = value;
	write->access = access;

	return 0;
}

int msm8998_osm_build_tz_sequence_image(
			const struct msm8998_osm_sequence_config *config,
			const struct msm8998_osm_lut_data *lut,
			unsigned int config_count,
			const struct msm8998_cpr_corner_data *corners,
			unsigned int corner_count,
			struct msm8998_osm_sequence_image *image)
{
	u8 mem_acc_level_map[MSM8998_OSM_MEM_ACC_LEVEL_COUNT] = {};
	u32 current_level;
	unsigned int transition_count = 0;
	unsigned int l_value_count = 0;
	unsigned int i;
	int ret;

	if (!config || !lut || !config_count ||
	    config_count > MSM8998_OSM_TABLE_SIZE || !corners ||
	    !corner_count || corner_count > MSM8998_CPR_MAX_CORNER_COUNT ||
	    !image)
		return -EINVAL;

	memset(image, 0, sizeof(*image));
	if (config->osm_no_tz)
		return -EOPNOTSUPP;
	if (!config->apm_threshold_uv || config->apm_threshold_uv > INT_MAX ||
	    config->mem_acc_threshold_uv > INT_MAX)
		return -EINVAL;

	image->sequencer_owned_by_tz = true;
	if (config->mem_acc_threshold_uv) {
		if (corner_count < 2)
			return -EINVAL;
		image->apm_crossover_vc = corner_count - 2;
		image->mem_acc_crossover_vc = corner_count - 1;
	} else {
		image->apm_crossover_vc = corner_count - 1;
	}

	ret = msm8998_osm_find_threshold_vc(lut, corners, corner_count,
			config->apm_threshold_uv, &image->apm_threshold_pre_vc,
			&image->apm_threshold_vc);
	if (ret)
		return ret;
	if (config->mem_acc_threshold_uv) {
		ret = msm8998_osm_find_threshold_vc(lut, corners, corner_count,
				config->mem_acc_threshold_uv,
				&image->mem_acc_threshold_pre_vc,
				&image->mem_acc_threshold_vc);
		if (ret)
			return ret;
	}

	current_level = lut[0].config.spare_data;
	if (current_level != 1)
		return -EINVAL;
	for (i = 0; i < config_count; i++) {
		u32 level = lut[i].config.spare_data;
		u8 virtual_corner;

		if (level < current_level ||
		    level > MSM8998_OSM_MEM_ACC_LEVEL_COUNT)
			return -EINVAL;
		if (current_level == MSM8998_OSM_MEM_ACC_LEVEL_COUNT)
			break;
		if (level == current_level)
			continue;
		if (level != current_level + 1 ||
		    transition_count >= MSM8998_OSM_MEM_ACC_LEVEL_COUNT - 1 ||
		    !lut[i].config.virtual_corner)
			return -EINVAL;

		virtual_corner = lut[i].config.virtual_corner - 1;
		if (!virtual_corner)
			return -EINVAL;
		mem_acc_level_map[transition_count++] = virtual_corner - 1;
		current_level = level;
	}
	if (transition_count != MSM8998_OSM_MEM_ACC_LEVEL_COUNT - 1 ||
	    current_level != MSM8998_OSM_MEM_ACC_LEVEL_COUNT)
		return -EINVAL;

	image->mem_acc_level_vc[0] = mem_acc_level_map[0];
	image->mem_acc_level_vc[1] = mem_acc_level_map[0] + 1;
	image->mem_acc_level_vc[2] = mem_acc_level_map[1];
	image->mem_acc_level_vc[3] = mem_acc_level_map[1] + 1;

	if (config->mem_acc_threshold_uv) {
		image->mem_acc_level_vc[2] = image->mem_acc_threshold_pre_vc;
		image->mem_acc_level_vc[3] = image->mem_acc_threshold_vc;
		if (image->mem_acc_threshold_pre_vc ==
		    MSM8998_OSM_SEQUENCE_MINUS_ONE) {
			image->mem_acc_level_vc[0] =
				MSM8998_OSM_SEQUENCE_MINUS_ONE;
			image->mem_acc_level_vc[1] =
				MSM8998_OSM_SEQUENCE_MINUS_ONE;
		} else {
			if (image->mem_acc_level_vc[1] >=
			    image->mem_acc_level_vc[2])
				image->mem_acc_level_vc[1] =
					image->mem_acc_level_vc[2] - 1;
			if (image->mem_acc_level_vc[0] >=
			    image->mem_acc_level_vc[1])
				image->mem_acc_level_vc[0] =
					image->mem_acc_level_vc[1] - 1;
		}
	}

	if (image->mem_acc_crossover_vc) {
		ret = msm8998_osm_append_sequence_write(image,
			MSM8998_OSM_SEQUENCE_REGISTER(88),
			image->mem_acc_crossover_vc, MSM8998_OSM_ACCESS_SCM);
		if (ret)
			return ret;
	}
	for (i = 0; i < MSM8998_OSM_MEM_ACC_THRESHOLD_COUNT; i++) {
		ret = msm8998_osm_append_sequence_write(image,
			MSM8998_OSM_SEQUENCE_REGISTER(55 + i),
			image->mem_acc_level_vc[i], MSM8998_OSM_ACCESS_SCM);
		if (ret)
			return ret;
	}
	for (i = 0; i < config_count; i++) {
		u8 virtual_corner;

		if (!lut[i].config.virtual_corner)
			return -EINVAL;
		virtual_corner = lut[i].config.virtual_corner - 1;
		if (virtual_corner != image->mem_acc_level_vc[3])
			continue;
		ret = msm8998_osm_append_sequence_write(image,
			MSM8998_OSM_SEQUENCE_REGISTER(32),
			lut[i].config.frequency_data & MSM8998_OSM_L_VALUE_MASK,
			MSM8998_OSM_ACCESS_SCM);
		if (ret)
			return ret;
		l_value_count++;
	}
	if (l_value_count != 1)
		return -EINVAL;

	if (image->apm_threshold_vc) {
		ret = msm8998_osm_append_sequence_write(image,
			MSM8998_OSM_SEQUENCE_REGISTER1_ALIAS,
			image->apm_threshold_vc, MSM8998_OSM_ACCESS_MMIO);
		if (ret)
			return ret;
	}
	ret = msm8998_osm_append_sequence_write(image,
		MSM8998_OSM_SEQUENCE_REGISTER(72), image->apm_crossover_vc,
		MSM8998_OSM_ACCESS_SCM);
	if (ret)
		return ret;
	ret = msm8998_osm_append_sequence_write(image,
		MSM8998_OSM_SEQUENCE_REGISTER(15), image->apm_threshold_vc,
		MSM8998_OSM_ACCESS_SCM);
	if (ret)
		return ret;
	ret = msm8998_osm_append_sequence_write(image,
		MSM8998_OSM_SEQUENCE_REGISTER(31), image->apm_threshold_pre_vc,
		MSM8998_OSM_ACCESS_SCM);
	if (ret)
		return ret;
	ret = msm8998_osm_append_sequence_write(image,
		MSM8998_OSM_SEQUENCE_REGISTER(76),
		MSM8998_OSM_APM_SEQUENCE_VALUE |
			image->apm_threshold_vc << 6,
		MSM8998_OSM_ACCESS_SCM);

	return ret;
}

static int msm8998_osm_append_acd_operation(
			struct msm8998_osm_acd_image *image,
			enum msm8998_osm_acd_operation_type type,
			u16 offset, u32 value)
{
	struct msm8998_osm_acd_operation *operation;

	if (image->operation_count >= MSM8998_OSM_ACD_MAX_OPERATIONS)
		return -ENOSPC;

	operation = &image->operation[image->operation_count++];
	operation->type = type;
	operation->offset = offset;
	operation->value = value;

	return 0;
}

int msm8998_osm_build_acd_image(const struct msm8998_osm_acd_config *config,
				struct msm8998_osm_acd_image *image)
{
	u32 initial_mask;
	u32 final_mask;
	int ret;

	if (!config || !image)
		return -EINVAL;

	memset(image, 0, sizeof(*image));
	initial_mask = BIT(MSM8998_OSM_ACD_TUNABLE_DELAY / 4) |
		BIT(MSM8998_OSM_ACD_CONTROL / 4) |
		BIT(MSM8998_OSM_ACD_SOFT_START_CONTROL / 4) |
		BIT(MSM8998_OSM_ACD_EXTINT_CONFIG / 4);
	final_mask = initial_mask | BIT(MSM8998_OSM_ACD_GFMUX_CONFIG / 4);
	image->initial_auto_transfer_mask = initial_mask;
	image->power_collapse_auto_transfer_mask = final_mask;

	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_MASTER_WRITE, MSM8998_OSM_ACD_TUNABLE_DELAY,
		config->tunable_delay);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_MASTER_WRITE, MSM8998_OSM_ACD_CONTROL,
		config->control);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_MASTER_WRITE,
		MSM8998_OSM_ACD_SOFT_START_CONTROL,
		config->soft_start_control);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_MASTER_WRITE, MSM8998_OSM_ACD_EXTINT_CONFIG,
		config->initial_extint);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_MASTER_WRITE,
		MSM8998_OSM_ACD_AUTO_TRANSFER_CONTROL,
		config->auto_transfer_control);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_AUTO_TRANSFER,
		MSM8998_OSM_ACD_AUTO_TRANSFER_CONFIG, initial_mask);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_WRITE_THROUGH,
		MSM8998_OSM_ACD_GFMUX_CONFIG, 1);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_WRITE_THROUGH,
		MSM8998_OSM_ACD_DCVS_SWITCH, 1);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_WRITE_THROUGH,
		MSM8998_OSM_ACD_DCVS_SWITCH, 0);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_DELAY_US, 0, 1);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_WRITE_THROUGH, MSM8998_OSM_ACD_EXTINT_CONFIG,
		config->final_extint);
	if (ret)
		return ret;
	ret = msm8998_osm_append_acd_operation(image,
		MSM8998_OSM_ACD_MASTER_WRITE,
		MSM8998_OSM_ACD_AUTO_TRANSFER_CONFIG, final_mask);

	return ret;
}
