/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _QCOM_CPUFREQ_MSM8998_H
#define _QCOM_CPUFREQ_MSM8998_H

#include <linux/types.h>

#define MSM8998_CPR_CLUSTER_COUNT 2
#define MSM8998_CPR_FUSE_CORNER_COUNT 4
#define MSM8998_CPR_RING_OSC_COUNT 16
#define MSM8998_CPR_MAX_CORNER_COUNT 34
#define MSM8998_OSM_TABLE_SIZE 40
#define MSM8998_OSM_MAX_OPP_COUNT 32
#define MSM8998_OSM_TZ_SEQUENCE_MAX_WRITES 11
#define MSM8998_OSM_ACD_MAX_OPERATIONS 12
#define MSM8998_CPR_FUSE_ROW_COUNT 8

enum msm8998_cpr_cluster_id {
	MSM8998_CPR_POWER_CLUSTER,
	MSM8998_CPR_PERFORMANCE_CLUSTER,
};

struct msm8998_cpr_fuse_rows {
	u64 row38;
	u64 row39;
	u64 row67;
	u64 row68;
	u64 row69;
	u64 row70;
	u64 row71;
	u64 row100;
};

struct msm8998_cpr_cluster_fuses {
	u8 init_voltage[MSM8998_CPR_FUSE_CORNER_COUNT];
	u16 target_quot[MSM8998_CPR_FUSE_CORNER_COUNT];
	u8 ring_osc[MSM8998_CPR_FUSE_CORNER_COUNT];
	u8 quot_offset[MSM8998_CPR_FUSE_CORNER_COUNT];
};

struct msm8998_cpr_fuses {
	u8 speed_bin;
	u8 revision;
	u8 force_highest_corner;
	struct msm8998_cpr_cluster_fuses cluster[MSM8998_CPR_CLUSTER_COUNT];
};

struct msm8998_cpr_cluster_fuse_data {
	int fused_open_loop_uv[MSM8998_CPR_FUSE_CORNER_COUNT];
	int adjusted_fused_open_loop_uv[MSM8998_CPR_FUSE_CORNER_COUNT];
	u16 target_quot[MSM8998_CPR_FUSE_CORNER_COUNT];
	u16 quot_offset[MSM8998_CPR_FUSE_CORNER_COUNT];
	u8 ring_osc[MSM8998_CPR_FUSE_CORNER_COUNT];
};

struct msm8998_cpr_fuse_data {
	u8 speed_bin;
	u8 revision;
	u8 force_highest_corner;
	u8 fuse_combo;
	struct msm8998_cpr_cluster_fuse_data cluster[MSM8998_CPR_CLUSTER_COUNT];
};

struct msm8998_cpr_corner_config {
	u32 frequency_hz;
	int floor_uv;
	int ceiling_uv;
	int open_loop_adjust_uv;
	int closed_loop_adjust_uv;
	u32 max_floor_to_ceiling_uv;
};

struct msm8998_cpr_domain_config {
	const struct msm8998_cpr_corner_config *corners;
	const u32 *ro_scaling_factor;
	unsigned int corner_count;
	u8 fmax_corner[MSM8998_CPR_FUSE_CORNER_COUNT];
	int fuse_closed_loop_adjust_uv[MSM8998_CPR_FUSE_CORNER_COUNT];
	u32 step_uv;
	u32 apm_threshold_uv;
	u32 apm_hysteresis_uv;
	u32 apm_crossover_uv;
	u32 mem_acc_threshold_uv;
	u32 mem_acc_crossover_uv;
	bool scaled_open_loop_as_ceiling;
	bool allow_quotient_interpolation;
};

struct msm8998_cpr_corner_data {
	u32 frequency_hz;
	u32 target_quot;
	int floor_uv;
	int open_loop_uv;
	int ceiling_uv;
	u8 fuse_corner;
	u8 ring_osc;
	bool use_open_loop;
};

struct msm8998_cprh_controller_config {
	u32 clock_rate_hz;
	u32 sensor_time_ns;
	u32 loop_time_ns;
	u32 voltage_settling_time_ns;
	u32 corner_switch_delay_time_ns;
	u32 base_uv;
	u32 step_uv;
	u32 count_repeat;
	u16 sensor_count;
	u8 idle_clocks;
	u8 count_mode;
	u8 step_quot_init_min;
	u8 step_quot_init_max;
	u8 consecutive_up;
	u8 consecutive_down;
	u8 up_threshold;
	u8 down_threshold;
	u8 down_error_step_limit;
	u8 up_error_step_limit;
	bool saw_use_unit_mv;
	bool use_dynamic_step_quot;
};

struct msm8998_cprh_register_image {
	u32 corner[MSM8998_CPR_MAX_CORNER_COUNT];
	u32 target_quot[MSM8998_CPR_RING_OSC_COUNT];
	u32 cpr_ctl;
	u32 cpr_timer_auto_cont;
	u32 cpr_step_quot;
	u32 gcnt;
	u32 threshold;
	u32 margin_adjust_ctl;
	u32 saw_error_step_limit;
	u32 margin_temp_core_timers;
	u32 cprh_ctl;
	u16 gcnt_ro_mask;
	u16 ro_mask;
	u16 sensor_owner_count;
	u8 corner_count;
};

struct msm8998_osm_lut_config {
	u32 frequency_hz;
	u32 frequency_data;
	u32 override_data;
	u32 spare_data;
	u8 virtual_corner;
};

struct msm8998_osm_lut_data {
	struct msm8998_osm_lut_config config;
	int open_loop_uv;
};

struct msm8998_osm_opp {
	u32 frequency_hz;
	int open_loop_uv;
	u8 virtual_corner;
	u8 table_index;
};

struct msm8998_osm_hw_table_entry {
	u32 index;
	u32 frequency_data;
	u32 voltage_data;
	u32 override_data;
	u32 spare_data;
};

struct msm8998_osm_hw_table_image {
	struct msm8998_osm_hw_table_entry entry[MSM8998_OSM_TABLE_SIZE];
};

enum msm8998_osm_register_access {
	MSM8998_OSM_ACCESS_MMIO,
	MSM8998_OSM_ACCESS_SCM,
};

struct msm8998_osm_register_write {
	u32 offset;
	u32 value;
	enum msm8998_osm_register_access access;
};

struct msm8998_osm_sequence_config {
	u32 apm_threshold_uv;
	u32 mem_acc_threshold_uv;
	bool osm_no_tz;
};

struct msm8998_osm_sequence_image {
	struct msm8998_osm_register_write
		write[MSM8998_OSM_TZ_SEQUENCE_MAX_WRITES];
	u8 mem_acc_level_vc[4];
	u8 apm_crossover_vc;
	u8 apm_threshold_pre_vc;
	u8 apm_threshold_vc;
	u8 mem_acc_crossover_vc;
	u8 mem_acc_threshold_pre_vc;
	u8 mem_acc_threshold_vc;
	u8 write_count;
	bool sequencer_owned_by_tz;
};

struct msm8998_osm_acd_config {
	u32 tunable_delay;
	u32 control;
	u32 soft_start_control;
	u32 initial_extint;
	u32 final_extint;
	u32 auto_transfer_control;
};

enum msm8998_osm_acd_operation_type {
	MSM8998_OSM_ACD_MASTER_WRITE,
	MSM8998_OSM_ACD_AUTO_TRANSFER,
	MSM8998_OSM_ACD_WRITE_THROUGH,
	MSM8998_OSM_ACD_DELAY_US,
};

struct msm8998_osm_acd_operation {
	u32 value;
	u16 offset;
	enum msm8998_osm_acd_operation_type type;
};

struct msm8998_osm_acd_image {
	struct msm8998_osm_acd_operation
		operation[MSM8998_OSM_ACD_MAX_OPERATIONS];
	u32 initial_auto_transfer_mask;
	u32 power_collapse_auto_transfer_mask;
	u8 operation_count;
};

int msm8998_cpr_decode_fuses(const struct msm8998_cpr_fuse_rows *rows,
			     struct msm8998_cpr_fuses *fuses);
int msm8998_cpr_decode_voltage(int reference_uv, u8 fuse, int *voltage_uv);
int msm8998_cpr_calculate_fuse_data(const struct msm8998_cpr_fuses *fuses,
				    struct msm8998_cpr_fuse_data *data);
int msm8998_cpr_build_corners(const struct msm8998_cpr_domain_config *config,
			      const struct msm8998_cpr_cluster_fuse_data *fuse,
			      struct msm8998_cpr_corner_data *corners,
			      unsigned int corner_capacity);
int msm8998_cpr_apply_partial_binning(u8 force_highest_corner,
				      struct msm8998_cpr_corner_data *corners,
				      unsigned int corner_count);
int msm8998_cpr_append_crossover_corners(const struct msm8998_cpr_domain_config *config,
					 struct msm8998_cpr_corner_data *corners,
					 unsigned int corner_capacity,
					 unsigned int *corner_count);
int msm8998_cprh_build_register_image(const struct msm8998_cprh_controller_config *config,
				      const struct msm8998_cpr_corner_data *corners,
				      unsigned int corner_count, bool closed_loop,
				      struct msm8998_cprh_register_image *image);
int msm8998_osm_resolve_lut(const struct msm8998_osm_lut_config *config,
			    unsigned int config_count,
			    const struct msm8998_cpr_corner_data *corners,
			    unsigned int corner_count,
			    struct msm8998_osm_lut_data *lut,
			    unsigned int lut_count);
int msm8998_osm_build_opp_table(const struct msm8998_osm_lut_data *lut,
				unsigned int config_count,
				struct msm8998_osm_opp *opps,
				unsigned int opp_capacity,
				unsigned int *opp_count);
int msm8998_osm_build_hw_table_image(const struct msm8998_osm_lut_data *lut,
				     unsigned int config_count,
				     struct msm8998_osm_hw_table_image *image);
int msm8998_osm_build_tz_sequence_image(
			const struct msm8998_osm_sequence_config *config,
			const struct msm8998_osm_lut_data *lut,
			unsigned int config_count,
			const struct msm8998_cpr_corner_data *corners,
			unsigned int corner_count,
			struct msm8998_osm_sequence_image *image);
int msm8998_osm_build_acd_image(const struct msm8998_osm_acd_config *config,
				struct msm8998_osm_acd_image *image);

#endif
