// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017 Linaro Ltd.
 */
#include <linux/slab.h>
#include <linux/mutex.h>

#ifndef JOAN_VTRACE
/*
 * 调试跟踪默认写进内存环形缓冲（joan_tlog），不走控制台。
 * 原来这里用 pr_info，而 hfi_msgs.c 对**每一条** HFI 消息都会调一次 ——
 * 一次短压力测试就产生上千行内核日志，既是日志刷屏也是实打实的 CPU 开销。
 * 需要看时读 /sys/module/venus_core/parameters/joan_trace_ring。
 */
#define JOAN_VTRACE(fmt, ...) joan_tlog(fmt "\n", ##__VA_ARGS__)
#endif
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>

#include "core.h"
#include "hfi.h"
#include "hfi_cmds.h"
#include "hfi_venus.h"
#include "helpers.h"

#define TIMEOUT		msecs_to_jiffies(1000)

extern unsigned int joan_enc_prewarm;
extern unsigned int joan_no_rpm;

static int hfi_session_init_direct(struct venus_inst *inst, u32 pixfmt);

static void joan_warm_event(struct venus_inst *inst, u32 event,
			    struct hfi_event_data *data)
{
	inst->session_error = true;
	joan_tlog("[warm] event=%u id=%#x error=%#x\n", event, inst->id,
		  inst->error);
}

static const struct hfi_inst_ops joan_warm_ops = {
	.event_notify = joan_warm_event,
};

/* Called with joan_warm_lock held and with the core powered. */
static int joan_prewarm_encoder_locked(struct venus_core *core)
{
	struct venus_inst *warm, *other;
	int ret;

	if (core->joan_warm_enc)
		return 0;
	if (!joan_no_rpm || !IS_V3(core) || core->state != CORE_INIT)
		return -EINVAL;

	/* A replacement must also precede every live decoder session. */
	mutex_lock(&core->lock);
	list_for_each_entry(other, &core->instances, list) {
		if (other->session_type == VIDC_SESSION_TYPE_DEC &&
		    other->state != INST_UNINIT) {
			mutex_unlock(&core->lock);
			return -EBUSY;
		}
	}
	mutex_unlock(&core->lock);

	warm = kzalloc(sizeof(*warm), GFP_KERNEL);
	if (!warm)
		return -ENOMEM;
	warm->core = core;
	warm->session_type = VIDC_SESSION_TYPE_ENC;
	warm->joan_owns_warm = true;
	mutex_init(&warm->lock);
	INIT_LIST_HEAD(&warm->list);

	ret = hfi_session_create(warm, &joan_warm_ops);
	if (ret)
		goto free;
	ret = hfi_session_init_direct(warm, V4L2_PIX_FMT_H264);
	joan_tlog("[warm] create id=%#x state=%u core=%u ret=%d\n",
		  warm->id, warm->state, core->state, ret);
	if (ret) {
		hfi_session_destroy(warm);
		synchronize_irq(core->irq);
		goto free;
	}
	core->joan_warm_enc = warm;
	return 0;
free:
	mutex_destroy(&warm->lock);
	kfree(warm);
	return ret;
}

int hfi_joan_prewarm_encoder(struct venus_core *core)
{
	int ret;

	if (!joan_enc_prewarm)
		return 0;
	mutex_lock(&core->joan_warm_lock);
	ret = joan_prewarm_encoder_locked(core);
	mutex_unlock(&core->joan_warm_lock);
	return ret;
}

void hfi_joan_drop_warm_encoder(struct venus_core *core)
{
	struct venus_inst *warm;

	mutex_lock(&core->joan_warm_lock);
	warm = core->joan_warm_enc;
	if (warm) {
		core->joan_warm_enc = NULL;
		hfi_session_deinit(warm);
		hfi_session_destroy(warm);
		synchronize_irq(core->irq);
		mutex_destroy(&warm->lock);
		kfree(warm);
	}
	mutex_unlock(&core->joan_warm_lock);
}

static u32 to_codec_type(u32 pixfmt)
{
	switch (pixfmt) {
	case V4L2_PIX_FMT_H264:
	case V4L2_PIX_FMT_H264_NO_SC:
		return HFI_VIDEO_CODEC_H264;
	case V4L2_PIX_FMT_H263:
		return HFI_VIDEO_CODEC_H263;
	case V4L2_PIX_FMT_MPEG1:
		return HFI_VIDEO_CODEC_MPEG1;
	case V4L2_PIX_FMT_MPEG2:
		return HFI_VIDEO_CODEC_MPEG2;
	case V4L2_PIX_FMT_MPEG4:
		return HFI_VIDEO_CODEC_MPEG4;
	case V4L2_PIX_FMT_VC1_ANNEX_G:
	case V4L2_PIX_FMT_VC1_ANNEX_L:
		return HFI_VIDEO_CODEC_VC1;
	case V4L2_PIX_FMT_VP8:
		return HFI_VIDEO_CODEC_VP8;
	case V4L2_PIX_FMT_VP9:
		return HFI_VIDEO_CODEC_VP9;
	case V4L2_PIX_FMT_XVID:
		return HFI_VIDEO_CODEC_DIVX;
	case V4L2_PIX_FMT_HEVC:
		return HFI_VIDEO_CODEC_HEVC;
	default:
		return 0;
	}
}

int hfi_core_init(struct venus_core *core)
{
	int ret = 0;

	mutex_lock(&core->lock);

	if (core->state >= CORE_INIT)
		goto unlock;

	reinit_completion(&core->done);

	ret = core->ops->core_init(core);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&core->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;

	if (core->error != HFI_ERR_NONE) {
		ret = -EIO;
		goto unlock;
	}

	core->state = CORE_INIT;
unlock:
	mutex_unlock(&core->lock);
	return ret;
}

int hfi_core_deinit(struct venus_core *core, bool blocking)
{
	int ret = 0, empty;

	mutex_lock(&core->lock);

	if (core->state == CORE_UNINIT)
		goto unlock;

	empty = list_empty(&core->instances);

	if (!empty && !blocking) {
		ret = -EBUSY;
		goto unlock;
	}

	if (!empty) {
		mutex_unlock(&core->lock);
		wait_var_event(&core->insts_count,
			       !atomic_read(&core->insts_count));
		mutex_lock(&core->lock);
	}

	if (!core->ops)
		goto unlock;

	ret = core->ops->core_deinit(core);

	if (!ret)
		core->state = CORE_UNINIT;

unlock:
	mutex_unlock(&core->lock);
	return ret;
}

int hfi_core_suspend(struct venus_core *core)
{
	if (core->state != CORE_INIT)
		return 0;

	return core->ops->suspend(core);
}

int hfi_core_resume(struct venus_core *core, bool force)
{
	int ret;

	if (!force && core->state != CORE_INIT)
		return 0;

	ret = core->ops->resume(core);
	if (ret)
		return ret;

	/*
	 * 候选 49：resume 走 venus_power_on -> venus_run -> venus_boot_core，
	 * 其中 PAS auth_and_reset 会把 ARM9 **重新启动**，固件侧 HFI 状态清零。
	 * 原实现只做 power_on，不重发 HFI_CMD_SYS_INIT，于是空闲掉电再恢复后
	 * 第一次会话必然失败（实测：连续编码 600 帧无错，空闲 60 秒后下一次必失败）。
	 * 这里把状态标记为未初始化并重做 SYS_INIT 握手。
	 */
	core->state = CORE_UNINIT;
	reinit_completion(&core->done);
	ret = core->ops->core_init(core);
	if (ret)
		return ret;
	ret = wait_for_completion_timeout(&core->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;
	if (core->error != HFI_ERR_NONE)
		return -EIO;
	core->state = CORE_INIT;

	return 0;
}

int hfi_core_trigger_ssr(struct venus_core *core, u32 type)
{
	return core->ops->core_trigger_ssr(core, type);
}

atomic_t joan_sync_timeouts = ATOMIC_INIT(0);
atomic_t joan_sync_errors = ATOMIC_INIT(0);
EXPORT_SYMBOL_GPL(joan_sync_timeouts);
EXPORT_SYMBOL_GPL(joan_sync_errors);

static int wait_session_msg(struct venus_inst *inst)
{
	int ret;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		atomic_inc(&joan_sync_timeouts);
		return -ETIMEDOUT;
	}

	if (inst->error != HFI_ERR_NONE) {
		atomic_inc(&joan_sync_errors);
		return -EIO;
	}

	return 0;
}

int hfi_session_create(struct venus_inst *inst, const struct hfi_inst_ops *ops)
{
	struct venus_core *core = inst->core;
	bool max;
	int ret;

	if (!ops)
		return -EINVAL;

	inst->state = INST_UNINIT;
	init_completion(&inst->done);
	inst->ops = ops;

	mutex_lock(&core->lock);

	if (test_bit(0, &inst->core->sys_error)) {
		ret = -EIO;
		goto unlock;
	}

	max = atomic_add_unless(&core->insts_count, 1,
				core->max_sessions_supported);
	if (!max) {
		ret = -EAGAIN;
	} else {
		list_add_tail(&inst->list, &core->instances);
		ret = 0;
	}

unlock:
	mutex_unlock(&core->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(hfi_session_create);

static int hfi_session_init_direct(struct venus_inst *inst, u32 pixfmt)
{
	struct venus_core *core = inst->core;
	const struct hfi_ops *ops = core->ops;
	int ret;

	/*
	 * If core shutdown is in progress or if we are in system
	 * recovery, return an error as during system error recovery
	 * session_init() can't pass successfully
	 */
	mutex_lock(&core->lock);
	if (!core->ops || test_bit(0, &inst->core->sys_error)) {
		mutex_unlock(&core->lock);
		return -EIO;
	}
	mutex_unlock(&core->lock);

	if (inst->state != INST_UNINIT)
		return -EALREADY;

	if (!inst->id)
		inst->id = hash32_ptr(inst);

	inst->hfi_codec = to_codec_type(pixfmt);
	reinit_completion(&inst->done);

	ret = ops->session_init(inst, inst->session_type, inst->hfi_codec);
	JOAN_VTRACE("hfi_session_init: ops->session_init ret=%d type=%u codec=%u",
		    ret, inst->session_type, inst->hfi_codec);
	if (ret)
		return ret;

	ret = wait_session_msg(inst);
	joan_tlog("[init] id=%#x type=%u codec=%#x ret=%d\n",
		  inst->id, inst->session_type, pixfmt, ret);
	JOAN_VTRACE("hfi_session_init: wait_session_msg ret=%d", ret);
	if (ret)
		return ret;

	inst->state = INST_INIT;

	return 0;
}

int hfi_session_init(struct venus_inst *inst, u32 pixfmt)
{
	struct venus_core *core = inst->core;
	struct venus_inst *warm;
	int ret;

	if (!joan_enc_prewarm)
		return hfi_session_init_direct(inst, pixfmt);
	if (inst->state != INST_UNINIT)
		return -EALREADY;

	mutex_lock(&core->joan_warm_lock);
	if (inst->session_type == VIDC_SESSION_TYPE_DEC) {
		ret = joan_prewarm_encoder_locked(core);
		if (ret) {
			joan_tlog("[warm] replenish ret=%d\n", ret);
			goto unlock;
		}
	}

	warm = core->joan_warm_enc;
	if (inst->session_type == VIDC_SESSION_TYPE_ENC &&
	    pixfmt == V4L2_PIX_FMT_H264 && warm &&
	    warm->state == INST_INIT && !warm->session_error) {
		/* Change the reply lookup atomically; firmware retains the session. */
		mutex_lock(&core->lock);
		inst->id = warm->id;
		inst->hfi_codec = warm->hfi_codec;
		inst->state = INST_INIT;
		core->joan_warm_enc = NULL;
		list_del_init(&warm->list);
		if (atomic_dec_and_test(&core->insts_count))
			wake_up_var(&core->insts_count);
		mutex_unlock(&core->lock);
		synchronize_irq(core->irq);
		mutex_destroy(&warm->lock);
		kfree(warm);
		joan_tlog("[warm] adopt id=%#x state=%u\n", inst->id, inst->state);
		ret = 0;
	} else {
		ret = hfi_session_init_direct(inst, pixfmt);
	}
unlock:
	mutex_unlock(&core->joan_warm_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(hfi_session_init);

void hfi_session_destroy(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;

	mutex_lock(&core->lock);
	list_del_init(&inst->list);
	if (atomic_dec_and_test(&core->insts_count))
		wake_up_var(&core->insts_count);
	mutex_unlock(&core->lock);
}
EXPORT_SYMBOL_GPL(hfi_session_destroy);

int hfi_session_deinit(struct venus_inst *inst)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (inst->state == INST_UNINIT)
		return 0;

	if (inst->state < INST_INIT)
		return -EINVAL;

	if (test_bit(0, &inst->core->sys_error))
		goto done;

	reinit_completion(&inst->done);

	ret = ops->session_end(inst);
	if (ret)
		return ret;

	ret = wait_session_msg(inst);
	if (ret)
		return ret;

done:
	inst->state = INST_UNINIT;

	return 0;
}
EXPORT_SYMBOL_GPL(hfi_session_deinit);

int hfi_session_start(struct venus_inst *inst)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	if (inst->state != INST_LOAD_RESOURCES)
		return -EINVAL;

	reinit_completion(&inst->done);

	ret = ops->session_start(inst);
	if (ret)
		return ret;

	ret = wait_session_msg(inst);
	if (ret)
		return ret;

	inst->state = INST_START;

	return 0;
}
EXPORT_SYMBOL_GPL(hfi_session_start);

int hfi_session_stop(struct venus_inst *inst)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	if (inst->state != INST_START)
		return -EINVAL;

	reinit_completion(&inst->done);

	ret = ops->session_stop(inst);
	if (ret)
		return ret;

	ret = wait_session_msg(inst);
	if (ret)
		return ret;

	inst->state = INST_STOP;

	return 0;
}
EXPORT_SYMBOL_GPL(hfi_session_stop);

int hfi_session_continue(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	if (core->res->hfi_version == HFI_VERSION_1XX)
		return 0;

	return core->ops->session_continue(inst);
}
EXPORT_SYMBOL_GPL(hfi_session_continue);

int hfi_session_abort(struct venus_inst *inst)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	reinit_completion(&inst->done);

	ret = ops->session_abort(inst);
	if (ret)
		return ret;

	ret = wait_session_msg(inst);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(hfi_session_abort);

int hfi_session_load_res(struct venus_inst *inst)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	if (inst->state != INST_INIT)
		return -EINVAL;

	reinit_completion(&inst->done);

	ret = ops->session_load_res(inst);
	if (ret)
		return ret;

	ret = wait_session_msg(inst);
	if (ret)
		return ret;

	inst->state = INST_LOAD_RESOURCES;

	return 0;
}

int hfi_session_unload_res(struct venus_inst *inst)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	if (inst->state != INST_STOP)
		return -EINVAL;

	reinit_completion(&inst->done);

	ret = ops->session_release_res(inst);
	if (ret)
		return ret;

	ret = wait_session_msg(inst);
	if (ret)
		return ret;

	inst->state = INST_RELEASE_RESOURCES;

	return 0;
}
EXPORT_SYMBOL_GPL(hfi_session_unload_res);

int hfi_session_flush(struct venus_inst *inst, u32 type, bool block)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	reinit_completion(&inst->done);

	ret = ops->session_flush(inst, type);
	if (ret)
		return ret;

	if (block) {
		ret = wait_session_msg(inst);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(hfi_session_flush);

int hfi_session_set_buffers(struct venus_inst *inst, struct hfi_buffer_desc *bd)
{
	const struct hfi_ops *ops = inst->core->ops;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	return ops->session_set_buffers(inst, bd);
}

int hfi_session_unset_buffers(struct venus_inst *inst,
			      struct hfi_buffer_desc *bd)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	reinit_completion(&inst->done);

	ret = ops->session_unset_buffers(inst, bd);
	if (ret)
		return ret;

	if (!bd->response_required)
		return 0;

	ret = wait_session_msg(inst);
	if (ret)
		return ret;

	return 0;
}

int hfi_session_get_property(struct venus_inst *inst, u32 ptype,
			     union hfi_get_property *hprop)
{
	const struct hfi_ops *ops = inst->core->ops;
	int ret;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	if (inst->state < INST_INIT || inst->state >= INST_STOP)
		return -EINVAL;

	reinit_completion(&inst->done);

	ret = ops->session_get_property(inst, ptype);
	if (ret)
		return ret;

	ret = wait_session_msg(inst);
	if (ret)
		return ret;

	*hprop = inst->hprop;

	return 0;
}
EXPORT_SYMBOL_GPL(hfi_session_get_property);

int hfi_session_set_property(struct venus_inst *inst, u32 ptype, void *pdata)
{
	const struct hfi_ops *ops = inst->core->ops;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	if (inst->state < INST_INIT || inst->state >= INST_STOP) {
		JOAN_VTRACE("hfi_set_property: ptype=%#x EINVAL state=%d",
			    ptype, inst->state);
		return -EINVAL;
	}

	{
		int __ret = ops->session_set_property(inst, ptype, pdata);

		JOAN_VTRACE("hfi_set_property: ptype=%#x ret=%d state=%d",
			    ptype, __ret, inst->state);
		return __ret;
	}
}
EXPORT_SYMBOL_GPL(hfi_session_set_property);

int hfi_session_process_buf(struct venus_inst *inst, struct hfi_frame_data *fd)
{
	const struct hfi_ops *ops = inst->core->ops;

	if (test_bit(0, &inst->core->sys_error))
		return -EIO;

	{
		int __ret;

		if (fd->buffer_type == HFI_BUFFER_INPUT)
			__ret = ops->session_etb(inst, fd);
		else if (fd->buffer_type == HFI_BUFFER_OUTPUT ||
			 fd->buffer_type == HFI_BUFFER_OUTPUT2)
			__ret = ops->session_ftb(inst, fd);
		else
			return 0;
		JOAN_VTRACE("process_buf: type=%u bytes=%u ret=%d",
			    fd->buffer_type, fd->alloc_len, __ret);
		return __ret;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(hfi_session_process_buf);

irqreturn_t hfi_isr_thread(int irq, void *dev_id)
{
	struct venus_core *core = dev_id;

	return core->ops->isr_thread(core);
}

irqreturn_t hfi_isr(int irq, void *dev)
{
	struct venus_core *core = dev;

	return core->ops->isr(core);
}

int hfi_create(struct venus_core *core, const struct hfi_core_ops *ops)
{
	if (!ops)
		return -EINVAL;

	atomic_set(&core->insts_count, 0);
	core->core_ops = ops;
	core->state = CORE_UNINIT;
	init_completion(&core->done);
	pkt_set_version(core->res->hfi_version);

	return venus_hfi_create(core);
}

void hfi_destroy(struct venus_core *core)
{
	venus_hfi_destroy(core);
}

void hfi_reinit(struct venus_core *core)
{
	venus_hfi_queues_reinit(core);
}
