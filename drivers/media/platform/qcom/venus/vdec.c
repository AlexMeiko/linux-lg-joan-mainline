// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017 Linaro Ltd.
 */
#include <linux/clk.h>

extern atomic_t joan_sync_timeouts;
extern atomic_t joan_sync_errors;

#ifndef JOAN_VTRACE
/*
 * 调试跟踪默认写进内存环形缓冲（joan_tlog），不走控制台。
 * 原来这里用 pr_info，而 hfi_msgs.c 对**每一条** HFI 消息都会调一次 ——
 * 一次短压力测试就产生上千行内核日志，既是日志刷屏也是实打实的 CPU 开销。
 * 需要看时读 /sys/module/venus_core/parameters/joan_trace_ring。
 */
#define JOAN_VTRACE(fmt, ...) joan_tlog(fmt "\n", ##__VA_ARGS__)
#endif
#include <linux/module.h>
#include <linux/math64.h>

/* 候选 76：reconfig 期间是否丢弃空的捕获完成（默认 0=正常完成）*/
unsigned int joan_dec_drop_empty;

/* 候选 70：是否在 SOURCE_CHANGE 之前就登记捕获缓冲（默认 1，解码需要）*/
unsigned int joan_dec_early_cap_reg = 1;

/* 候选 62：解码器收到序列变化后由驱动内部完成重配（对用户态透明）*/
extern unsigned int joan_trace_quiet;

unsigned int joan_dec_transparent_reconfig = 1;

static bool joan_dec_ts_fallback = 1;

/* 候选 51：解码 G_FMT 在 INIT 状态是否放行（默认 0，保持原语义）*/
unsigned int joan_dec_gfmt_noblock = 1;
#include <linux/of.h>
/* 候选 28：固件不发 SOURCE_CHANGE 时，强制走 reconfigure（分配 DPB + SESSION_CONTINUE）*/
unsigned int joan_dec_force_reconfig;

/* 候选 33b：early reconfigure 跳过位（bit0=跳过 output_conf/set_num_bufs, bit1=跳过 DPB, bit2=跳过 CONTINUE）*/
unsigned int joan_early_skip;

#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "hfi_venus_io.h"
#include "hfi_parser.h"
#include "core.h"
#include "helpers.h"
#include "vdec.h"
#include "pm_helpers.h"

/*
 * Three resons to keep MPLANE formats (despite that the number of planes
 * currently is one):
 * - the MPLANE formats allow only one plane to be used
 * - the downstream driver use MPLANE formats too
 * - future firmware versions could add support for >1 planes
 */
static const struct venus_format vdec_formats[] = {
	[VENUS_FMT_NV12] = {
		.pixfmt = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	},
	[VENUS_FMT_QC08C] = {
		.pixfmt = V4L2_PIX_FMT_QC08C,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	},
	[VENUS_FMT_QC10C] = {
		.pixfmt = V4L2_PIX_FMT_QC10C,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	},
	[VENUS_FMT_P010] = {
		.pixfmt = V4L2_PIX_FMT_P010,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	},
	[VENUS_FMT_H264] = {
		.pixfmt = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_VP8] = {
		.pixfmt = V4L2_PIX_FMT_VP8,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_VP9] = {
		.pixfmt = V4L2_PIX_FMT_VP9,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_HEVC] = {
		.pixfmt = V4L2_PIX_FMT_HEVC,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_VC1_ANNEX_G] = {
		.pixfmt = V4L2_PIX_FMT_VC1_ANNEX_G,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_VC1_ANNEX_L] = {
		.pixfmt = V4L2_PIX_FMT_VC1_ANNEX_L,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_MPEG4] = {
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_MPEG2] = {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_H263] = {
		.pixfmt = V4L2_PIX_FMT_H263,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	[VENUS_FMT_XVID] = {
		.pixfmt = V4L2_PIX_FMT_XVID,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},

};

static const struct venus_format *
find_format(struct venus_inst *inst, u32 pixfmt, u32 type)
{
	const struct venus_format *fmt = vdec_formats;
	unsigned int size = ARRAY_SIZE(vdec_formats);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (fmt[i].pixfmt == pixfmt)
			break;
	}

	if (i == size || fmt[i].type != type)
		return NULL;

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    !venus_helper_check_codec(inst, fmt[i].pixfmt))
		return NULL;

	if (V4L2_TYPE_IS_CAPTURE(type) &&
	    !venus_helper_check_format(inst, fmt[i].pixfmt))
		return NULL;

	if (V4L2_TYPE_IS_CAPTURE(type) && fmt[i].pixfmt == V4L2_PIX_FMT_QC10C &&
	    !(inst->bit_depth == VIDC_BITDEPTH_10))
		return NULL;

	return &fmt[i];
}

static const struct venus_format *
find_format_by_index(struct venus_inst *inst, unsigned int index, u32 type)
{
	const struct venus_format *fmt = vdec_formats;
	unsigned int size = ARRAY_SIZE(vdec_formats);
	unsigned int i, k = 0;

	if (index > size)
		return NULL;

	for (i = 0; i < size; i++) {
		bool valid = false;

		if (fmt[i].type != type)
			continue;

		if (V4L2_TYPE_IS_OUTPUT(type)) {
			valid = venus_helper_check_codec(inst, fmt[i].pixfmt);
		} else {
			valid = venus_helper_check_format(inst, fmt[i].pixfmt);

			if (fmt[i].pixfmt == V4L2_PIX_FMT_QC10C &&
			    !(inst->bit_depth == VIDC_BITDEPTH_10))
				valid = false;
		}

		if (k == index && valid)
			break;
		if (valid)
			k++;
	}

	if (i == size)
		return NULL;

	return &fmt[i];
}

static const struct venus_format *
vdec_try_fmt_common(struct venus_inst *inst, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = pixmp->plane_fmt;
	const struct venus_format *fmt;
	u32 szimage;

	memset(pfmt[0].reserved, 0, sizeof(pfmt[0].reserved));
	memset(pixmp->reserved, 0, sizeof(pixmp->reserved));

	fmt = find_format(inst, pixmp->pixelformat, f->type);
	if (!fmt) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_NV12;
		else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_H264;
		else
			return NULL;
		fmt = find_format(inst, pixmp->pixelformat, f->type);
		if (!fmt)
			return NULL;
	}

	/*
	 * HEVC 的固件能力表里只有 BITRATE 项，没有 FRAME_WIDTH/HEIGHT，
	 * 于是 frame_width_max() 返回 0，clamp(w, 0, 0) 会把宽高清零，
	 * 固件拿到 0x0 的格式自然解不出东西（实测只消费 5 字节就停滞）。
	 * 缺能力项时退回合理范围。
	 */
	{
		u32 wmin = frame_width_min(inst), wmax = frame_width_max(inst);
		u32 hmin = frame_height_min(inst), hmax = frame_height_max(inst);

		if (!wmax) {
			wmin = 16;
			wmax = 4096;
		}
		if (!hmax) {
			hmin = 16;
			hmax = 4096;
		}
		pixmp->width = clamp(pixmp->width, wmin, wmax);
		pixmp->height = clamp(pixmp->height, hmin, hmax);
	}
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		pixmp->height = ALIGN(pixmp->height, 32);

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;
	pixmp->num_planes = fmt->num_planes;
	pixmp->flags = 0;

	szimage = venus_helper_get_framesz(pixmp->pixelformat, pixmp->width,
					   pixmp->height);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		unsigned int stride = pixmp->width;

		if (pixmp->pixelformat == V4L2_PIX_FMT_P010)
			stride *= 2;

		pfmt[0].sizeimage = szimage;
		pfmt[0].bytesperline = ALIGN(stride, 128);
	} else {
		pfmt[0].sizeimage = clamp_t(u32, pfmt[0].sizeimage, 0, SZ_8M);
		pfmt[0].sizeimage = max(pfmt[0].sizeimage, szimage);
		pfmt[0].bytesperline = 0;
	}

	return fmt;
}

static int vdec_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct venus_inst *inst = to_inst(file);

	vdec_try_fmt_common(inst, f);

	return 0;
}

static int vdec_check_src_change(struct venus_inst *inst)
{
	int ret;

	/*
	 * 候选 51：原来这里在 INIT 状态直接返回 -EINVAL，应用（ffmpeg 的 v4l2m2m）
	 * 会一直重试 G_FMT，永远不去启动捕获队列；而固件在没有捕获队列时又不消费
	 * 输入、不发 SOURCE_CHANGE —— 两边互为前提，形成死锁。
	 * 这里改为放行：先让应用拿到当前格式并把 capture 流起来，等固件真正报告
	 * 分辨率变化时再走 SOURCE_CHANGE/reconfig 流程。
	 */
	if (inst->subscriptions & V4L2_EVENT_SOURCE_CHANGE &&
	    inst->codec_state == VENUS_DEC_STATE_INIT &&
	    !inst->reconfig) {
		if (!joan_dec_gfmt_noblock)
			return -EINVAL;
		JOAN_VTRACE("vdec_g_fmt: INIT 状态放行（候选 51，开关=1）%ux%u",
			    inst->width, inst->height);
		return 0;
	}

	if (inst->subscriptions & V4L2_EVENT_SOURCE_CHANGE)
		return 0;

	/*
	 * The code snippet below is a workaround for backward compatibility
	 * with applications which doesn't support V4L2 events. It will be
	 * dropped in future once those applications are fixed.
	 */

	if (inst->codec_state != VENUS_DEC_STATE_INIT)
		goto done;

	ret = wait_event_timeout(inst->reconf_wait, inst->reconfig,
				 msecs_to_jiffies(100));
	if (!ret)
		return -EINVAL;

	if (!(inst->codec_state == VENUS_DEC_STATE_CAPTURE_SETUP) ||
	    !inst->reconfig)
		dev_dbg(inst->core->dev, VDBGH "wrong state\n");

done:
	return 0;
}

static int vdec_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct venus_inst *inst = to_inst(file);
	const struct venus_format *fmt = NULL;
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	int ret;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		fmt = inst->fmt_cap;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		fmt = inst->fmt_out;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vdec_check_src_change(inst);
		if (ret)
			return ret;
	}

	pixmp->pixelformat = fmt->pixfmt;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/*
		 * 候选 59：ffmpeg 用 G_FMT 的前后差异判断是否要 reinit
		 * （v4l2_resolution_changed）。如果事件前后都报对齐后的尺寸
		 * （320x256），它认为"没变化"，就不会重配捕获队列，导致一直拿不到帧。
		 * 这里在有真实裁剪尺寸（SOURCE_CHANGE 之后）时改报可见尺寸（320x240）。
		 */
		pixmp->width = inst->width;
		pixmp->height = inst->height;
		JOAN_VTRACE("vdec_g_fmt: pending=%d", v4l2_event_pending(&inst->fh));
		JOAN_VTRACE("vdec_g_fmt: cap %ux%u (reconfig=%d crop=%ux%u inst=%ux%u state=%d)",
			    pixmp->width, pixmp->height, inst->reconfig,
			    inst->crop.width, inst->crop.height,
			    inst->width, inst->height, inst->codec_state);
		pixmp->colorspace = inst->colorspace;
		pixmp->ycbcr_enc = inst->ycbcr_enc;
		pixmp->quantization = inst->quantization;
		pixmp->xfer_func = inst->xfer_func;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixmp->width = inst->out_width;
		pixmp->height = inst->out_height;
	}

	vdec_try_fmt_common(inst, f);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		joan_tlog("[gfmt] cap=%ux%u bpl=%u size=%u inst=%ux%u crop=%ux%u\n",
			  pixmp->width, pixmp->height,
			  pixmp->plane_fmt[0].bytesperline,
			  pixmp->plane_fmt[0].sizeimage, inst->width,
			  inst->height, inst->crop.width, inst->crop.height);

	return 0;
}

static int vdec_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	JOAN_VTRACE("vdec_s_fmt: type=%d %ux%u state=%d reconfig=%d",
		    f->type, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		    to_inst(file)->codec_state, to_inst(file)->reconfig);
	struct venus_inst *inst = to_inst(file);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_pix_format_mplane orig_pixmp;
	const struct venus_format *fmt;
	struct v4l2_format format;
	u32 pixfmt_out = 0, pixfmt_cap = 0;
	struct vb2_queue *q;

	q = v4l2_m2m_get_vq(inst->m2m_ctx, f->type);
	if (!q)
		return -EINVAL;

	if (vb2_is_busy(q))
		return -EBUSY;

	orig_pixmp = *pixmp;

	/*
	 * 4:2:0 需要偶数宽高，过小尺寸固件无法构成合法帧。
	 * 必须在 clamp/对齐之前用调用方原始值判断，否则 1920x1 这类
	 * 会被 clamp 到最小高度后放行，最终产出垃圾码流。
	 */
	if ((orig_pixmp.width & 1) || (orig_pixmp.height & 1) ||
	    orig_pixmp.width < 16 || orig_pixmp.height < 16)
		return -EINVAL;

	fmt = vdec_try_fmt_common(inst, f);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixfmt_out = pixmp->pixelformat;
		pixfmt_cap = inst->fmt_cap->pixfmt;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixfmt_cap = pixmp->pixelformat;
		pixfmt_out = inst->fmt_out->pixfmt;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_out;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(inst, &format);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		inst->out_width = format.fmt.pix_mp.width;
		inst->out_height = format.fmt.pix_mp.height;
		inst->colorspace = pixmp->colorspace;
		inst->ycbcr_enc = pixmp->ycbcr_enc;
		inst->quantization = pixmp->quantization;
		inst->xfer_func = pixmp->xfer_func;
		inst->input_buf_size = pixmp->plane_fmt[0].sizeimage;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_cap;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(inst, &format);

	inst->width = format.fmt.pix_mp.width;
	inst->height = format.fmt.pix_mp.height;
	inst->crop.top = 0;
	inst->crop.left = 0;
	inst->crop.width = inst->width;
	inst->crop.height = inst->height;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		inst->fmt_out = fmt;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		inst->fmt_cap = fmt;
		inst->output2_buf_size =
			venus_helper_get_framesz(pixfmt_cap, orig_pixmp.width, orig_pixmp.height);
	}

	return 0;
}

static int
vdec_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct venus_inst *inst = to_inst(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	s->r.top = 0;
	s->r.left = 0;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
			return -EINVAL;
		s->r.width = inst->out_width;
		s->r.height = inst->out_height;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		s->r.width = inst->width;
		s->r.height = inst->height;
		break;
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		s->r = inst->crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int
vdec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "qcom-venus", sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm Venus video decoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:qcom-venus", sizeof(cap->bus_info));

	return 0;
}

static int vdec_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct venus_inst *inst = to_inst(file);
	const struct venus_format *fmt;

	memset(f->reserved, 0, sizeof(f->reserved));

	fmt = find_format_by_index(inst, f->index, f->type);
	if (!fmt)
		return -EINVAL;

	f->pixelformat = fmt->pixfmt;
	f->flags = fmt->flags;

	return 0;
}

static int vdec_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct venus_inst *inst = to_inst(file);
	struct v4l2_captureparm *cap = &a->parm.capture;
	struct v4l2_fract *timeperframe = &cap->timeperframe;
	u64 us_per_frame, fps;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	memset(cap->reserved, 0, sizeof(cap->reserved));
	if (!timeperframe->denominator)
		timeperframe->denominator = inst->timeperframe.denominator;
	if (!timeperframe->numerator)
		timeperframe->numerator = inst->timeperframe.numerator;
	cap->readbuffers = 0;
	cap->extendedmode = 0;
	cap->capability = V4L2_CAP_TIMEPERFRAME;
	us_per_frame = timeperframe->numerator * (u64)USEC_PER_SEC;
	do_div(us_per_frame, timeperframe->denominator);

	us_per_frame = clamp(us_per_frame, 1, USEC_PER_SEC);
	fps = USEC_PER_SEC / (u32)us_per_frame;
	fps = min(VENUS_MAX_FPS, fps);

	inst->fps = fps;
	inst->timeperframe = *timeperframe;

	return 0;
}

static int vdec_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	struct venus_inst *inst = to_inst(file);
	const struct venus_format *fmt;

	fmt = find_format(inst, fsize->pixel_format,
			  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (!fmt) {
		fmt = find_format(inst, fsize->pixel_format,
				  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		if (!fmt)
			return -EINVAL;
	}

	if (fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;

	fsize->stepwise.min_width = frame_width_min(inst);
	fsize->stepwise.max_width = frame_width_max(inst);
	fsize->stepwise.step_width = frame_width_step(inst);
	fsize->stepwise.min_height = frame_height_min(inst);
	fsize->stepwise.max_height = frame_height_max(inst);
	fsize->stepwise.step_height = frame_height_step(inst);

	return 0;
}

static int vdec_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	struct venus_inst *inst = container_of(fh, struct venus_inst, fh);
	int ret;

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		ret = v4l2_src_change_event_subscribe(fh, sub);
		if (ret)
			return ret;
		inst->subscriptions |= V4L2_EVENT_SOURCE_CHANGE;
		return 0;
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static int
vdec_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *cmd)
{
	struct venus_inst *inst = to_inst(file);
	struct vb2_queue *dst_vq;
	struct hfi_frame_data fdata = {0};
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, cmd);
	if (ret)
		return ret;

	mutex_lock(&inst->lock);

	if (cmd->cmd == V4L2_DEC_CMD_STOP) {
		/*
		 * Implement V4L2_DEC_CMD_STOP by enqueue an empty buffer on
		 * decoder input to signal EOS.
		 */
		if (!(inst->streamon_out && inst->streamon_cap))
			goto unlock;

		fdata.buffer_type = HFI_BUFFER_INPUT;
		fdata.flags |= HFI_BUFFERFLAG_EOS;

		/* Send NULL EOS addr for only IRIS2 (SM8250),for firmware <= 1.0.87.
		 * SC7280 also reports "1.0.<hash>" parsed as 1.0.0; restricting to IRIS2
		 * avoids misapplying this quirk and breaking VP9 decode on SC7280.
		 */

		if (IS_IRIS2(inst->core) && is_fw_rev_or_older(inst->core, 1, 0, 87)) {
			fdata.device_addr = 0;
		} else {
			/*
			 * 候选 58：与编码侧（候选 44/48）同样的两个坑——
			 *   1) 0xdeadb000 这种伪地址会被 msm8998 固件以
			 *      HFI_ERR_SESSION_INVALID_PARAMETER 拒绝；
			 *   2) alloc_len=0 的 EOS 空缓冲同样被拒。
			 * 改为复用最近一次真实输入地址 + 真实缓冲大小。
			 */
			fdata.device_addr = inst->joan_last_in_addr;
			fdata.alloc_len = inst->input_buf_size;
			/*
			 * EOS 不是真正的 vb2 缓冲，完成时不应该匹配到任何缓冲。
			 * 原来用 clnt_data=0，而完成路径是按 index 匹配的 ——
			 * 一旦此刻 index0 的缓冲正躺在 ready 队列里（用户态重新入队
			 * 之后），EOS 的完成就会把它提前归还，固件还在读这块内存，
			 * 而它真正的完成则再也匹配不到、被静默丢弃。
			 * 用不可能等于 index 的哨兵值，完成时自然匹配失败、被忽略。
			 */
			fdata.clnt_data = VENUS_EOS_TAG;
			JOAN_VTRACE("vdec_dec_cmd: EOS addr=%#x alloc=%u",
				    (u32)fdata.device_addr, fdata.alloc_len);
		}

		ret = hfi_session_process_buf(inst, &fdata);

		if (!ret && inst->codec_state == VENUS_DEC_STATE_DECODING) {
			inst->codec_state = VENUS_DEC_STATE_DRAIN;
			inst->drain_active = true;
		}
	} else if (cmd->cmd == V4L2_DEC_CMD_START &&
		   inst->codec_state == VENUS_DEC_STATE_STOPPED) {
		dst_vq = v4l2_m2m_get_vq(inst->fh.m2m_ctx,
					 V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		vb2_clear_last_buffer_dequeued(dst_vq);

		inst->codec_state = VENUS_DEC_STATE_DECODING;
	}

unlock:
	mutex_unlock(&inst->lock);
	return ret;
}

/*
 * 解码捕获队列固定使用 non-coherent（缓存）分配，理由见队列初始化处。
 * 和编码侧一样，不能直接写 q->non_coherent_mem（会和 CREATE_BUFS 的
 * verify_coherency_flags 冲突），而是在 ioctl 入口补标志。
 */
static void vdec_force_non_coherent(struct v4l2_requestbuffers *rb)
{
	if (rb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    rb->memory == V4L2_MEMORY_MMAP)
		rb->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;
}

static int vdec_ioctl_reqbufs(struct file *file, void *priv,
			      struct v4l2_requestbuffers *rb)
{
	vdec_force_non_coherent(rb);
	return v4l2_m2m_ioctl_reqbufs(file, priv, rb);
}

static int vdec_ioctl_create_bufs(struct file *file, void *priv,
				  struct v4l2_create_buffers *cb)
{
	if (cb->format.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    cb->memory == V4L2_MEMORY_MMAP)
		cb->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;

	return v4l2_m2m_ioctl_create_bufs(file, priv, cb);
}

static void vdec_force_cache_sync(struct v4l2_buffer *b)
{
	b->flags &= ~(V4L2_BUF_FLAG_NO_CACHE_INVALIDATE |
		      V4L2_BUF_FLAG_NO_CACHE_CLEAN);
}

static int vdec_ioctl_prepare_buf(struct file *file, void *priv,
				  struct v4l2_buffer *b)
{
	vdec_force_cache_sync(b);
	return v4l2_m2m_ioctl_prepare_buf(file, priv, b);
}

static int vdec_ioctl_qbuf(struct file *file, void *priv, struct v4l2_buffer *b)
{
	vdec_force_cache_sync(b);
	return v4l2_m2m_ioctl_qbuf(file, priv, b);
}

static const struct v4l2_ioctl_ops vdec_ioctl_ops = {
	.vidioc_querycap = vdec_querycap,
	.vidioc_enum_fmt_vid_cap = vdec_enum_fmt,
	.vidioc_enum_fmt_vid_out = vdec_enum_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vdec_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vdec_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vdec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vdec_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vdec_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vdec_try_fmt,
	.vidioc_g_selection = vdec_g_selection,
	.vidioc_reqbufs = vdec_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_create_bufs = vdec_ioctl_create_bufs,
	.vidioc_prepare_buf = vdec_ioctl_prepare_buf,
	.vidioc_qbuf = vdec_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_s_parm = vdec_s_parm,
	.vidioc_enum_framesizes = vdec_enum_framesizes,
	.vidioc_subscribe_event = vdec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = vdec_decoder_cmd,
};

static int vdec_pm_get(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;
	struct device *dev = core->dev_dec;
	int ret;

	mutex_lock(&core->pm_lock);
	ret = pm_runtime_resume_and_get(dev);
	mutex_unlock(&core->pm_lock);

	return ret;
}

static int vdec_pm_put(struct venus_inst *inst, bool autosuspend)
{
	struct venus_core *core = inst->core;
	struct device *dev = core->dev_dec;
	int ret;

	mutex_lock(&core->pm_lock);

	if (autosuspend)
		ret = pm_runtime_put_autosuspend(dev);
	else
		ret = pm_runtime_put_sync(dev);

	mutex_unlock(&core->pm_lock);

	return ret < 0 ? ret : 0;
}

static int vdec_pm_get_put(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;
	struct device *dev = core->dev_dec;
	int ret = 0;

	mutex_lock(&core->pm_lock);

	if (pm_runtime_suspended(dev)) {
		ret = pm_runtime_resume_and_get(dev);
		if (ret < 0)
			goto error;

		ret = pm_runtime_put_autosuspend(dev);
	}

error:
	mutex_unlock(&core->pm_lock);

	return ret < 0 ? ret : 0;
}

static void vdec_pm_touch(struct venus_inst *inst)
{
	pm_runtime_mark_last_busy(inst->core->dev_dec);
}

static int vdec_set_properties(struct venus_inst *inst)
{
	struct vdec_controls *ctr = &inst->controls.dec;
	struct hfi_enable en = { .enable = 1 };
	u32 ptype, decode_order, conceal;
	int ret;

	if (ctr->post_loop_deb_mode) {
		ptype = HFI_PROPERTY_CONFIG_VDEC_POST_LOOP_DEBLOCKER;
		ret = hfi_session_set_property(inst, ptype, &en);
		if (ret)
			return ret;
	}

	if (ctr->display_delay_enable && ctr->display_delay == 0) {
		ptype = HFI_PROPERTY_PARAM_VDEC_OUTPUT_ORDER;
		decode_order = HFI_OUTPUT_ORDER_DECODE;
		ret = hfi_session_set_property(inst, ptype, &decode_order);
		if (ret)
			return ret;
	}

	/* Enabling sufficient sequence change support for VP9 */
	if (is_fw_rev_or_newer(inst->core, 5, 4, 51)) {
		ptype = HFI_PROPERTY_PARAM_VDEC_ENABLE_SUFFICIENT_SEQCHANGE_EVENT;
		ret = hfi_session_set_property(inst, ptype, &en);
		if (ret)
			return ret;
	}

	ptype = HFI_PROPERTY_PARAM_VDEC_CONCEAL_COLOR;
	conceal = ctr->conceal_color & 0xffff;
	conceal |= ((ctr->conceal_color >> 16) & 0xffff) << 10;
	conceal |= ((ctr->conceal_color >> 32) & 0xffff) << 20;

	ret = hfi_session_set_property(inst, ptype, &conceal);
	if (ret)
		return ret;

	return 0;
}

static int vdec_set_work_route(struct venus_inst *inst)
{
	u32 ptype = HFI_PROPERTY_PARAM_WORK_ROUTE;
	struct hfi_video_work_route wr;

	if (!(IS_IRIS2(inst->core) || IS_IRIS2_1(inst->core)))
		return 0;

	wr.video_work_route = inst->core->res->num_vpp_pipes;

	return hfi_session_set_property(inst, ptype, &wr);
}

#define is_ubwc_fmt(fmt) (!!((fmt) & HFI_COLOR_FORMAT_UBWC_BASE))
#define is_10bit_ubwc_fmt(fmt) (!!((fmt) & HFI_COLOR_FORMAT_10_BIT_BASE & \
				 HFI_COLOR_FORMAT_UBWC_BASE))


static int vdec_output_conf(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;
	struct hfi_enable en = { .enable = 1 };
	struct hfi_buffer_requirements bufreq;
	u32 width = inst->width;
	u32 height = inst->height;
	u32 out_fmt, out2_fmt;
	bool ubwc = false;
	u32 ptype;
	int ret;

	ret = venus_helper_set_work_mode(inst);
	if (ret)
		return ret;

	if (core->res->hfi_version == HFI_VERSION_1XX) {
		ptype = HFI_PROPERTY_PARAM_VDEC_CONTINUE_DATA_TRANSFER;
		ret = hfi_session_set_property(inst, ptype, &en);
		if (ret)
			return ret;
	}

	/* Force searching UBWC formats for bigger then HD resolutions */
	if (width > 1920 && height > ALIGN(1080, 32))
		ubwc = true;

	/* For Venus v4/v6 UBWC format is mandatory */
	if (IS_V4(core) || IS_V6(core))
		ubwc = true;

	ret = venus_helper_get_out_fmts(inst, inst->fmt_cap->pixfmt, &out_fmt,
					&out2_fmt, ubwc);
	if (ret)
		return ret;

	inst->output_buf_size =
			venus_helper_get_framesz_raw(out_fmt, width, height);
	inst->output2_buf_size =
			venus_helper_get_framesz_raw(out2_fmt, width, height);

	if (is_ubwc_fmt(out_fmt)) {
		inst->opb_buftype = HFI_BUFFER_OUTPUT2;
		inst->opb_fmt = out2_fmt;
		inst->dpb_buftype = HFI_BUFFER_OUTPUT;
		inst->dpb_fmt = out_fmt;
	} else if (is_ubwc_fmt(out2_fmt) || is_10bit_ubwc_fmt(out_fmt)) {
		inst->opb_buftype = HFI_BUFFER_OUTPUT;
		inst->opb_fmt = out_fmt;
		inst->dpb_buftype = HFI_BUFFER_OUTPUT2;
		inst->dpb_fmt = out2_fmt;
	} else {
		inst->opb_buftype = HFI_BUFFER_OUTPUT;
		inst->opb_fmt = out_fmt;
		inst->dpb_buftype = 0;
		inst->dpb_fmt = 0;
	}

	/*
	 * joan / msm8998（HFI 3XX）：若固件只把 NV12 列在 OUTPUT2 下，
	 * venus_helper_get_out_fmts() 会返回 out_fmt=0，上面就落到 else 分支，
	 * opb_fmt/dpb_fmt/output_buf_size 全为 0。之后
	 * UNCOMPRESSED_FORMAT_SELECT 会被发成 0，HFI_BUFFER_* 的 size 属性
	 * 一条也不会发（vdec.c 里 "if (inst->output_buf_size)" 的判断），
	 * 固件因此不解码。这里做兜底，保证格式与尺寸始终可算。
	 */
	if (!inst->opb_fmt) {
		inst->opb_fmt = (inst->bit_depth == 10) ?
			HFI_COLOR_FORMAT_P010 : HFI_COLOR_FORMAT_NV12;
		inst->opb_buftype = HFI_BUFFER_OUTPUT2;
		JOAN_VTRACE("output_conf: opb_fmt fallback %#x buftype=%u",
			    inst->opb_fmt, inst->opb_buftype);
	}
	if (!inst->output_buf_size)
		inst->output_buf_size = venus_helper_get_framesz_raw(inst->opb_fmt,
								     width, height);
	if (!inst->output2_buf_size && inst->opb_buftype == HFI_BUFFER_OUTPUT2)
		inst->output2_buf_size = venus_helper_get_framesz_raw(inst->opb_fmt,
								      width, height);
	JOAN_VTRACE("output_conf: out=%#x out2=%#x opb=%u/%#x dpb=%u/%#x osz=%u o2sz=%u",
		    out_fmt, out2_fmt, inst->opb_buftype, inst->opb_fmt,
		    inst->dpb_buftype, inst->dpb_fmt,
		    inst->output_buf_size, inst->output2_buf_size);

	ret = venus_helper_set_raw_format(inst, inst->opb_fmt,
					  inst->opb_buftype);
	if (ret)
		return ret;

	ret = venus_helper_set_format_constraints(inst);
	if (ret)
		return ret;

	if (inst->dpb_fmt) {
		ret = venus_helper_set_multistream(inst, false, true);
		if (ret)
			return ret;

		ret = venus_helper_set_raw_format(inst, inst->dpb_fmt,
						  inst->dpb_buftype);
		if (ret)
			return ret;

		ret = venus_helper_set_output_resolution(inst, width, height,
							 HFI_BUFFER_OUTPUT2);
		if (ret)
			return ret;
	}

	if (IS_V3(core) || IS_V4(core) || IS_V6(core)) {
		ret = venus_helper_get_bufreq(inst, HFI_BUFFER_OUTPUT, &bufreq);
		if (ret)
			return ret;

		if (bufreq.size > inst->output_buf_size)
			return -EINVAL;

		if (inst->dpb_fmt) {
			ret = venus_helper_get_bufreq(inst, HFI_BUFFER_OUTPUT2,
						      &bufreq);
			if (ret)
				return ret;

			if (bufreq.size > inst->output2_buf_size)
				return -EINVAL;
		}

		if (inst->output2_buf_size) {
			ret = venus_helper_set_bufsize(inst,
						       inst->output2_buf_size,
						       HFI_BUFFER_OUTPUT2);
			if (ret)
				return ret;
		}

		if (inst->output_buf_size) {
			ret = venus_helper_set_bufsize(inst,
						       inst->output_buf_size,
						       HFI_BUFFER_OUTPUT);
			if (ret)
				return ret;
		}
	}

	ret = venus_helper_set_dyn_bufmode(inst);
	if (ret)
		return ret;

	return 0;
}

static int vdec_session_init(struct venus_inst *inst)
{
	int ret;

	ret = venus_helper_session_init(inst);
	if (ret == -EALREADY)
		return 0;
	else if (ret)
		return ret;

	/*
	 * 候选 39：下游对 INPUT 帧尺寸发的是真实 S_FMT 尺寸
	 * （DS/msm_vdec.c:1340-1347），主线只发能力最小值；msm8998 固件
	 * 拿到 96x96 后按此预分配内部结构，遇到真实码流会崩（SYS_ERROR）。
	 * 有真实值时优先用它。
	 */
	JOAN_VTRACE("session_init: input res min=%ux%u actual=%ux%u",
		    frame_width_min(inst), frame_height_min(inst),
		    inst->out_width, inst->out_height);
	ret = venus_helper_set_input_resolution(inst,
			inst->out_width ? inst->out_width : frame_width_min(inst),
			inst->out_height ? inst->out_height : frame_height_min(inst));
	if (ret)
		goto deinit;

	return 0;
deinit:
	hfi_session_deinit(inst);
	return ret;
}

static int vdec_num_buffers(struct venus_inst *inst, unsigned int *in_num,
			    unsigned int *out_num)
{
	enum hfi_version ver = inst->core->res->hfi_version;
	struct hfi_buffer_requirements bufreq;
	int ret;

	*in_num = *out_num = 0;

	ret = venus_helper_get_bufreq(inst, HFI_BUFFER_INPUT, &bufreq);
	if (ret)
		return ret;

	*in_num = hfi_bufreq_get_count_min(&bufreq, ver);

	ret = venus_helper_get_bufreq(inst, HFI_BUFFER_OUTPUT, &bufreq);
	if (ret)
		return ret;

	*out_num = hfi_bufreq_get_count_min(&bufreq, ver);

	return 0;
}

static int vdec_queue_setup(struct vb2_queue *q,
			    unsigned int *num_buffers, unsigned int *num_planes,
			    unsigned int sizes[], struct device *alloc_devs[])
{
	/*
	 * 解码侧**不**启用 non_coherent_mem：
	 * - 解码路径没有 CPU 拷贝，实测 cached/uncached 都是 ~216 fps（1080p）；
	 * - 它会让 vb2_core_create_bufs() 的 verify_coherency_flags() 失败
	 *   （应用不传 V4L2_MEMORY_FLAG_NON_COHERENT，而我们置了 1），
	 *   于是 REQBUFS 之后再 CREATE_BUFS 的应用会拿到 -EINVAL。
	 * 这个代价只在确实需要缓存映射的编码侧重排上才值得付。
	 */

	struct venus_inst *inst = vb2_get_drv_priv(q);
	struct venus_core *core = inst->core;
	unsigned int in_num, out_num;
	int ret = 0;

	JOAN_VTRACE("vdec_queue_setup: type=%d count=%u planes=%u state=%d reconfig=%d",
		    q->type, *num_buffers, *num_planes,
		    inst->codec_state, inst->reconfig);
	if (*num_planes) {
		unsigned int output_buf_size = venus_helper_get_opb_size(inst);

		if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
		    *num_planes != inst->fmt_out->num_planes)
			return -EINVAL;

		if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
		    *num_planes != inst->fmt_cap->num_planes)
			return -EINVAL;

		if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
		    sizes[0] < inst->input_buf_size)
			return -EINVAL;

		if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
		    sizes[0] < output_buf_size)
			return -EINVAL;

		return 0;
	}

	if (test_bit(0, &core->sys_error)) {
		if (inst->nonblock)
			return -EAGAIN;

		ret = wait_event_interruptible(core->sys_err_done,
					       !test_bit(0, &core->sys_error));
		if (ret)
			return ret;
	}

	ret = vdec_pm_get(inst);
	if (ret)
		return ret;

	ret = vdec_session_init(inst);
	if (ret)
		goto put_power;

	ret = vdec_num_buffers(inst, &in_num, &out_num);
	if (ret)
		goto put_power;

	ret = vdec_pm_put(inst, false);
	if (ret)
		return ret;

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		*num_planes = inst->fmt_out->num_planes;
		sizes[0] = venus_helper_get_framesz(inst->fmt_out->pixfmt,
						    inst->out_width,
						    inst->out_height);
		sizes[0] = max(sizes[0], inst->input_buf_size);
		inst->input_buf_size = sizes[0];
		*num_buffers = max(*num_buffers, in_num);
		inst->num_input_bufs = *num_buffers;
		inst->num_output_bufs = out_num;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		*num_planes = inst->fmt_cap->num_planes;
		sizes[0] = venus_helper_get_framesz(inst->fmt_cap->pixfmt,
						    inst->width,
						    inst->height);
		inst->output_buf_size = sizes[0];
		*num_buffers = max(*num_buffers, out_num);
		inst->num_output_bufs = *num_buffers;

		mutex_lock(&inst->lock);
		if (inst->codec_state == VENUS_DEC_STATE_CAPTURE_SETUP)
			inst->codec_state = VENUS_DEC_STATE_STOPPED;
		mutex_unlock(&inst->lock);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;

put_power:
	vdec_pm_put(inst, false);
	return ret;
}

static int vdec_verify_conf(struct venus_inst *inst)
{
	enum hfi_version ver = inst->core->res->hfi_version;
	struct hfi_buffer_requirements bufreq;
	int ret;

	if (!inst->num_input_bufs || !inst->num_output_bufs)
		return -EINVAL;

	ret = venus_helper_get_bufreq(inst, HFI_BUFFER_OUTPUT, &bufreq);
	if (ret)
		return ret;

	if (inst->num_output_bufs < bufreq.count_actual ||
	    inst->num_output_bufs < hfi_bufreq_get_count_min(&bufreq, ver))
		return -EINVAL;

	ret = venus_helper_get_bufreq(inst, HFI_BUFFER_INPUT, &bufreq);
	if (ret)
		return ret;

	if (inst->num_input_bufs < hfi_bufreq_get_count_min(&bufreq, ver))
		return -EINVAL;

	return 0;
}

/*
 * 候选 68：把 reconfigure 主体抽出来，事件处理器（透明重配模式）也能复用，
 * 走与探针成功路径完全相同的一套步骤。
 */
static int vdec_reconfigure(struct venus_inst *inst)
{
	int ret;
	if (!(joan_early_skip & 1)) {
		ret = vdec_output_conf(inst);
		if (ret)
			return ret;

		/*
		 * 候选 39：声明实际缓冲数（下游用 count_actual/客户端实际值），
		 * 而不是固定 VB2_MAX_FRAME(32)。
		 */
		JOAN_VTRACE("start_capture: set_num_bufs in=%u out=%u",
			    inst->num_input_bufs, inst->num_output_bufs);
		ret = venus_helper_set_num_bufs(inst, inst->num_input_bufs,
						inst->num_output_bufs,
						inst->num_output_bufs);
		JOAN_VTRACE("early_reconfig: output_conf+num_bufs ret=%d", ret);
		if (ret)
			return ret;
	} else {
		JOAN_VTRACE("early_reconfig: skip output_conf/num_bufs");
	}

	ret = venus_helper_intbufs_realloc(inst);
	if (ret)
		goto err;

	venus_pm_load_scale(inst);

	inst->next_buf_last = false;

	if (!(joan_early_skip & 2)) {
		ret = venus_helper_alloc_dpb_bufs(inst);
		JOAN_VTRACE("vdec_start_capture: alloc_dpb_bufs ret=%d", ret);
		if (ret)
			goto err;
	} else {
		JOAN_VTRACE("early_reconfig: skip alloc_dpb_bufs");
	}

	if (!(joan_early_skip & 4)) {
		ret = hfi_session_continue(inst);
		JOAN_VTRACE("vdec_start_capture: hfi_session_continue ret=%d", ret);
		if (ret)
			goto free_dpb_bufs;
	} else {
		JOAN_VTRACE("early_reconfig: skip continue");
	}

	ret = venus_helper_queue_dpb_bufs(inst);
	if (ret)
		goto free_dpb_bufs;

	/* 候选 67：把 reconfig 期间暂缓的输入缓冲补提交 */
	ret = venus_helper_process_initial_out_bufs(inst);
	JOAN_VTRACE("vdec_start_capture: 补提交输入缓冲 ret=%d", ret);
	if (ret)
		goto free_dpb_bufs;

	ret = venus_helper_process_initial_cap_bufs(inst);
	if (ret)
		goto free_dpb_bufs;

	inst->codec_state = VENUS_DEC_STATE_DECODING;

	if (inst->drain_active)
		inst->codec_state = VENUS_DEC_STATE_DRAIN;

	inst->streamon_cap = 1;
	inst->sequence_cap = 0;
	inst->reconfig = false;
	inst->drain_active = false;

	return 0;

	return 0;

free_dpb_bufs:
	venus_helper_free_dpb_bufs(inst);
err:
	return ret;
}

static int vdec_start_capture(struct venus_inst *inst)
{
	int ret;

	if (!inst->streamon_out)
		return 0;

	if (inst->codec_state == VENUS_DEC_STATE_DECODING) {
		if (inst->reconfig)
			goto reconfigure;

		venus_helper_queue_dpb_bufs(inst);
		venus_helper_process_initial_cap_bufs(inst);
		inst->streamon_cap = 1;
		return 0;
	}

	if (inst->codec_state != VENUS_DEC_STATE_STOPPED) {
		/*
		 * joan / msm8998：固件在「内部 DPB 注册 + HFI_CMD_SESSION_CONTINUE」
		 * 之前不会发 SOURCE_CHANGE（实测整轮 0 个 EVENT_NOTIFY）。若在这里
		 * 直接返回，capture 侧 bring-up 永远不会发生。对该平台提前走
		 * reconfigure 路径；其它平台保持原行为。joan_dec_force_reconfig=1
		 * 可对所有平台强制开启（实验用）。
		 */
		/*
		 * 注意：实测 early reconfigure 会让 msm8998 固件报
		 * HFI_EVENT_SYS_ERROR(0x1)，随后内核卡死。默认必须关闭，
		 * 只保留为实验参数。
		 */
		if (!joan_dec_force_reconfig) {
			/*
			 * 候选 53：原来这里直接 return 0 —— 既不登记捕获缓冲，也不置
			 * streamon_cap。固件拿不到 HFI_BUFFER_OUTPUT 就没有地方写解码
			 * 结果，于是只回 SESSION_EMPTY_BUFFER、永远不产 FILL_BUFFER。
			 * 这里改成把已排队的捕获缓冲登记/提交给固件（不触发 reconfigure）。
			 */
			if (!joan_dec_early_cap_reg) {
				/* 候选 70：默认不提前登记捕获缓冲（交还给用户态重配流程） */
				JOAN_VTRACE("vdec_start_capture: 非 STOPPED 且未开启提前登记，直接返回");
				return 0;
			}
			{
				struct v4l2_m2m_buffer *b; unsigned int n = 0;
				v4l2_m2m_for_each_dst_buf(inst->m2m_ctx, b) { n++; }
				JOAN_VTRACE("vdec_start_capture: 候选53 进入时 dst 队列 n=%u", n);
			}
			venus_helper_process_initial_cap_bufs(inst);
			inst->streamon_cap = 1;
			return 0;
		}
		JOAN_VTRACE("vdec_start_capture: early reconfigure (state=%d, param=%u)",
			    inst->codec_state, joan_dec_force_reconfig);
		goto reconfigure;
	}

reconfigure:
	ret = vdec_reconfigure(inst);
	if (ret)
		return ret;
	return 0;

free_dpb_bufs:
	venus_helper_free_dpb_bufs(inst);
err:
	return ret;
}

static int vdec_start_output(struct venus_inst *inst)
{
	int ret;

	if (inst->codec_state == VENUS_DEC_STATE_SEEK) {
		ret = venus_helper_process_initial_out_bufs(inst);
		if (ret)
			return ret;

		if (inst->next_buf_last) {
			inst->codec_state = VENUS_DEC_STATE_DRC;
		} else {
			inst->codec_state = VENUS_DEC_STATE_DECODING;

			if (inst->streamon_cap) {
				ret = venus_helper_queue_dpb_bufs(inst);
				if (ret)
					return ret;
			}
		}
		goto done;
	}

	if (inst->codec_state == VENUS_DEC_STATE_INIT ||
	    inst->codec_state == VENUS_DEC_STATE_CAPTURE_SETUP) {
		ret = venus_helper_process_initial_out_bufs(inst);
		goto done;
	}

	if (inst->codec_state != VENUS_DEC_STATE_DEINIT)
		return -EINVAL;

	venus_helper_init_instance(inst);
	inst->sequence_out = 0;
	inst->reconfig = false;
	inst->next_buf_last = false;

	ret = vdec_set_properties(inst);
	if (ret)
		return ret;

	ret = vdec_set_work_route(inst);
	if (ret)
		return ret;

	ret = vdec_output_conf(inst);
	if (ret)
		return ret;

	ret = vdec_verify_conf(inst);
	if (ret)
		return ret;

	ret = venus_helper_set_num_bufs(inst, inst->num_input_bufs,
					VB2_MAX_FRAME, VB2_MAX_FRAME);
	if (ret)
		return ret;

	ret = venus_helper_vb2_start_streaming(inst);
	if (ret)
		return ret;

	ret = venus_helper_process_initial_out_bufs(inst);
	if (ret)
		return ret;

	inst->codec_state = VENUS_DEC_STATE_INIT;

done:
	inst->streamon_out = 1;
	return ret;
}

/*
 * 把 SOURCE_CHANGE 事件从消息线程转交给工作线程。
 *
 * 调用者（HFI 消息线程）只按值拷贝事件、调度 worker 就返回，
 * 绝不能在这里拿 inst->lock —— 那正是原来死锁的成因。
 * 每次入队都 schedule_work()：work 开始执行时 PENDING 已清除，
 * 运行期间再入队会重新排队，同一 workqueue 上同一 work 不会并发执行，
 * 因此不会漏唤醒。
 */
static void vdec_queue_seq_event(struct venus_inst *inst,
				 const struct hfi_event_data *data,
				 bool sufficient)
{
	unsigned long flags;
	unsigned int tail;

	spin_lock_irqsave(&inst->seq_lock, flags);

	if (inst->seq_closing || !inst->seq_accepting || inst->seq_fault)
		goto unlock;

	inst->seq_pending = true;

	if (inst->seq_count == VENUS_SEQ_DEPTH) {
		/*
		 * 队列满意味着我们跟不上固件。丢掉尺寸/状态转换之后继续用
		 * 现有缓冲配置没有正确性保证，按会话错误处理，不再入队。
		 */
		inst->seq_fault = true;
		inst->seq_overflows++;
		dev_err(inst->core->dev_dec,
			"joan: SOURCE_CHANGE 队列溢出（%llu），按会话错误处理\n",
			inst->seq_overflows);
	} else {
		tail = (inst->seq_head + inst->seq_count) % VENUS_SEQ_DEPTH;
		inst->seq_fifo[tail].ev = *data;
		inst->seq_fifo[tail].gen = inst->seq_gen;
		inst->seq_fifo[tail].sufficient = sufficient;
		inst->seq_count++;
	}

	/* 与 closing 检查同处一个临界区，避免 close 取消后生产者又排队 */
	schedule_work(&inst->seq_event_work);

unlock:
	spin_unlock_irqrestore(&inst->seq_lock, flags);
}

/* 作废当前代次并清空队列。调用者必须持 inst->lock。 */
static void vdec_seq_invalidate_locked(struct venus_inst *inst)
{
	unsigned long flags;

	lockdep_assert_held(&inst->lock);

	spin_lock_irqsave(&inst->seq_lock, flags);
	inst->seq_accepting = false;
	inst->seq_gen++;
	inst->seq_head = 0;
	inst->seq_count = 0;
	inst->seq_pending = false;
	spin_unlock_irqrestore(&inst->seq_lock, flags);
}

/* 打开入口并推进代次。调用者必须持 inst->lock。 */
static void vdec_seq_arm_locked(struct venus_inst *inst)
{
	unsigned long flags;

	lockdep_assert_held(&inst->lock);

	spin_lock_irqsave(&inst->seq_lock, flags);
	inst->seq_gen++;
	inst->seq_head = 0;
	inst->seq_count = 0;
	inst->seq_pending = false;
	inst->seq_fault = false;
	inst->seq_accepting = true;
	spin_unlock_irqrestore(&inst->seq_lock, flags);
}

static int vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct venus_inst *inst = vb2_get_drv_priv(q);
	int ret;

	mutex_lock(&inst->lock);

	/*
	 * 同步命令（LOAD_RES/OUTPUT_CONF/START/FLUSH...）共用 inst->done，
	 * 任何一次超时或错误都说明完成信号被串了。健康时保持零输出，
	 * 只在异常时打到 dmesg，避免又是一条每流一次的噪音。
	 */
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    (atomic_read(&joan_sync_timeouts) || atomic_read(&joan_sync_errors)))
		pr_err("joan-sync: 同步命令超时=%d 错误=%d\n",
		       atomic_read(&joan_sync_timeouts),
		       atomic_read(&joan_sync_errors));

	inst->joan_dec_stopping = false;
	/*
	 * 每次重新开始流都要清掉时间戳回退的状态。
	 * 同一个 fd 上先后解两个文件时，上一路流的 last_ts 会让新流的
	 * 第一帧被判成"时间戳回退"并被强行加大，时间轴就错了。
	 */
	/*
	 * 任意一个队列重新启动都要清掉时间戳回退状态。
	 * 只解绑输出队列（seek/换流）时捕获队列不会重新 streamon，
	 * 若把重置放在 CAPTURE 分支里，旧时间戳会残留。
	 */
	inst->joan_dec_last_ts = 0;
	inst->joan_dec_ts_seen = false;
	inst->joan_dec_ts_fixups = 0;
	inst->joan_dec_empty_recycled = 0;

	/*
	 * 输入侧重新开始：推进代次并打开 SOURCE_CHANGE 入口。
	 * 捕获队列单独重启不动代次（同一路输入流的尺寸事件仍然有效）。
	 */
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		vdec_seq_arm_locked(inst);

	JOAN_VTRACE("vdec_start_streaming: type=%d count=%u", q->type, count);
	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vdec_start_capture(inst);
		if (!ret)
			inst->seq_capture_started = true;
	} else {
		ret = vdec_pm_get(inst);
		if (ret)
			goto error;

		ret = venus_pm_acquire_core(inst);
		if (ret)
			goto put_power;

		ret = vdec_pm_put(inst, true);
		if (ret)
			goto error;

		inst->joan_first_in_sent = false;
		ret = vdec_start_output(inst);
	}

	JOAN_VTRACE("vdec_start_streaming: type=%d ret=%d", q->type, ret);
	if (ret)
		goto error;

	mutex_unlock(&inst->lock);
	return 0;

put_power:
	vdec_pm_put(inst, false);
error:
	venus_helper_buffers_done(inst, q->type, VB2_BUF_STATE_QUEUED);
	mutex_unlock(&inst->lock);
	return ret;
}

static void vdec_cancel_dst_buffers(struct venus_inst *inst)
{
	struct vb2_v4l2_buffer *buf;

	while ((buf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
}

static int vdec_stop_capture(struct venus_inst *inst)
{
	int ret = 0;

	switch (inst->codec_state) {
	case VENUS_DEC_STATE_DECODING:
		ret = hfi_session_flush(inst, HFI_FLUSH_ALL, true);
		fallthrough;
	case VENUS_DEC_STATE_DRAIN:
		inst->codec_state = VENUS_DEC_STATE_STOPPED;
		inst->drain_active = false;
		fallthrough;
	case VENUS_DEC_STATE_SEEK:
		vdec_cancel_dst_buffers(inst);
		break;
	case VENUS_DEC_STATE_DRC:
		ret = hfi_session_flush(inst, HFI_FLUSH_OUTPUT, true);
		inst->codec_state = VENUS_DEC_STATE_CAPTURE_SETUP;
		venus_helper_free_dpb_bufs(inst);
		break;
	case VENUS_DEC_STATE_CAPTURE_SETUP:
		/*
		 * 候选 74：SOURCE_CHANGE 之后用户态会 STREAMOFF(capture) 再重新
		 * REQBUFS/STREAMON。原来 CAPTURE_SETUP 落在 default 分支、状态不变，
		 * 于是下一次 STREAMON 走的不是 reconfigure 路径（不重设属性、
		 * 不发 SESSION_CONTINUE），固件永远不开始解码。
		 * 这里把状态推进到 STOPPED，让 vdec_start_capture 进入 reconfigure。
		 */
		JOAN_VTRACE("vdec_stop_capture: CAPTURE_SETUP -> STOPPED（候选 74）");
		inst->codec_state = VENUS_DEC_STATE_STOPPED;
		inst->drain_active = false;
		vdec_cancel_dst_buffers(inst);
		break;
	default:
		break;
	}

	return ret;
}

static int vdec_stop_output(struct venus_inst *inst)
{
	int ret = 0;

	switch (inst->codec_state) {
	case VENUS_DEC_STATE_DECODING:
	case VENUS_DEC_STATE_DRAIN:
	case VENUS_DEC_STATE_STOPPED:
	case VENUS_DEC_STATE_DRC:
		ret = hfi_session_flush(inst, HFI_FLUSH_ALL, true);
		inst->codec_state = VENUS_DEC_STATE_SEEK;
		break;
	case VENUS_DEC_STATE_INIT:
	case VENUS_DEC_STATE_CAPTURE_SETUP:
		ret = hfi_session_flush(inst, HFI_FLUSH_ALL, true);
		break;
	default:
		break;
	}

	return ret;
}

static void vdec_stop_streaming(struct vb2_queue *q)
{
	{
		struct venus_inst *i2 = vb2_get_drv_priv(q);
		JOAN_VTRACE("vdec_stop_streaming: type=%d state=%d reconfig=%d out=%d cap=%d",
			    q->type, i2->codec_state, i2->reconfig,
			    i2->streamon_out, i2->streamon_cap);
	}
	struct venus_inst *inst = vb2_get_drv_priv(q);
	int ret = -EINVAL;

	vdec_pm_get_put(inst);

	mutex_lock(&inst->lock);

	inst->joan_dec_stopping = true;
	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* 捕获侧停止：保留输入代次和事件数据，只是不再自动重配 */
		inst->seq_capture_started = false;
		ret = vdec_stop_capture(inst);
	} else {
		/* 输入侧停止：作废代次，旧事件不能再启动重配 */
		vdec_seq_invalidate_locked(inst);
		ret = vdec_stop_output(inst);
	}

	venus_helper_buffers_done(inst, q->type, VB2_BUF_STATE_ERROR);

	inst->session_error = 0;

	if (ret)
		goto unlock;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		inst->streamon_out = 0;
	else
		inst->streamon_cap = 0;

unlock:
	mutex_unlock(&inst->lock);
	JOAN_VTRACE("vdec_stop_streaming: type=%d 退出 ret=%d state=%d", q->type, ret, inst->codec_state);
}

static void vdec_session_release(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;
	int ret, abort = 0;

	vdec_pm_get(inst);

	mutex_lock(&inst->lock);
	inst->codec_state = VENUS_DEC_STATE_DEINIT;
	inst->joan_dec_stopping = true;
	if (joan_dec_ts_fallback || joan_dec_clean_stream)
		JOAN_VTRACE("decoder summary: id=%#x empty_recycled=%u ts_fixups=%u",
			    inst->id, inst->joan_dec_empty_recycled,
			    inst->joan_dec_ts_fixups);

	ret = hfi_session_stop(inst);
	abort = (ret && ret != -EINVAL) ? 1 : 0;
	ret = hfi_session_unload_res(inst);
	abort = (ret && ret != -EINVAL) ? 1 : 0;
	ret = venus_helper_unregister_bufs(inst);
	abort = (ret && ret != -EINVAL) ? 1 : 0;
	ret = venus_helper_intbufs_free(inst);
	abort = (ret && ret != -EINVAL) ? 1 : 0;
	ret = hfi_session_deinit(inst);
	abort = (ret && ret != -EINVAL) ? 1 : 0;

	if (inst->session_error || test_bit(0, &core->sys_error))
		abort = 1;

	if (abort)
		hfi_session_abort(inst);

	venus_helper_free_dpb_bufs(inst);
	venus_pm_load_scale(inst);
	INIT_LIST_HEAD(&inst->registeredbufs);
	mutex_unlock(&inst->lock);

	venus_pm_release_core(inst);
	vdec_pm_put(inst, false);
}

static int vdec_buf_init(struct vb2_buffer *vb)
{
	struct venus_inst *inst = vb2_get_drv_priv(vb->vb2_queue);

	inst->buf_count++;

	return venus_helper_vb2_buf_init(vb);
}

static void vdec_buf_cleanup(struct vb2_buffer *vb)
{
	struct venus_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct venus_buffer *buf = to_venus_buffer(vbuf);

	mutex_lock(&inst->lock);
	if (vb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		if (!list_empty(&inst->registeredbufs))
			list_del_init(&buf->reg_list);
	mutex_unlock(&inst->lock);

	inst->buf_count--;
	if (!inst->buf_count)
		vdec_session_release(inst);
}

static void vdec_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct venus_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	static const struct v4l2_event eos = { .type = V4L2_EVENT_EOS };

	vdec_pm_get_put(inst);

	mutex_lock(&inst->lock);

	if (inst->next_buf_last && V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) &&
	    inst->codec_state == VENUS_DEC_STATE_DRC) {
		vbuf->flags |= V4L2_BUF_FLAG_LAST;
		vbuf->sequence = inst->sequence_cap++;
		vbuf->field = V4L2_FIELD_NONE;
		vb2_set_plane_payload(vb, 0, 0);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);
		v4l2_event_queue_fh(&inst->fh, &eos);
		inst->next_buf_last = false;
		mutex_unlock(&inst->lock);
		return;
	}

	venus_helper_vb2_buf_queue(vb);
	mutex_unlock(&inst->lock);
}

static const struct vb2_ops vdec_vb2_ops = {
	.queue_setup = vdec_queue_setup,
	.buf_init = vdec_buf_init,
	.buf_cleanup = vdec_buf_cleanup,
	.buf_prepare = venus_helper_vb2_buf_prepare,
	.start_streaming = vdec_start_streaming,
	.stop_streaming = vdec_stop_streaming,
	.buf_queue = vdec_vb2_buf_queue,
};


/*
 * 解码输出布局转换：
 *   固件按 128 对齐步幅、32 行对齐高度写 NV12
 *     Y: ALIGN(w,128) x ALIGN(h,32)，UV 紧随其后；
 *   而用户态（ffmpeg v4l2m2m）按可见尺寸紧凑读写
 *     Y: w x h，UV 紧跟其后，每帧 w*h*3/2 字节。
 * 两者不一致时，ffmpeg 每帧都从错误偏移开始读，画面是纯绿 + 横纹
 * （实测 PSNR ≈ 5 dB）。这里在交还缓冲之前把固件布局搬成紧凑布局。
 * 顺序：先 Y（目标区间 0..w*h 不碰 UV 源），再 UV（逐行 src>dst，从后往前）。
 */

static void vdec_buf_done(struct venus_inst *inst, unsigned int buf_type,
			  u32 tag, u32 bytesused, u32 data_offset, u32 flags,
			  u32 hfi_flags, u64 timestamp_us)
{
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	struct vb2_v4l2_buffer *vbuf;
	struct vb2_buffer *vb;
	unsigned int type;

	vdec_pm_touch(inst);

	if (buf_type == HFI_BUFFER_INPUT)
		type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	else
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	vbuf = venus_helper_find_buf(inst, type, tag);
	if (!joan_trace_quiet && V4L2_TYPE_IS_CAPTURE(type)) {
		struct v4l2_m2m_buffer *b;
		unsigned int n = 0;

		v4l2_m2m_for_each_dst_buf(inst->m2m_ctx, b)
			n++;
		joan_tlog("[dcd] tag=%u bytes=%u found=%d q=%u\n", tag,
			  bytesused, vbuf ? 1 : 0, n);
	}
	JOAN_VTRACE("vdec_buf_done: type=%u tag=%u bytesused=%u found=%d",
		    type, tag, bytesused, vbuf ? 1 : 0);
	if (!vbuf && V4L2_TYPE_IS_CAPTURE(type)) {
		struct v4l2_m2m_buffer *b;
		char idxs[96] = "";
		unsigned int n = 0;

		v4l2_m2m_for_each_dst_buf(inst->m2m_ctx, b) {
			char tmp[8];
			snprintf(tmp, sizeof(tmp), "%u ", b->vb.vb2_buf.index);
			strlcat(idxs, tmp, sizeof(idxs));
			n++;
		}
		JOAN_VTRACE("vdec_buf_done: dst 队列 n=%u [%s]", n, idxs);
	}
	if (!vbuf) {
		venus_helper_change_dpb_owner(inst, vbuf, type, buf_type, tag);
		return;
	}

	if (joan_dec_clean_stream)
		to_venus_buffer(vbuf)->joan_submitted = false;

	/*
	 * A flush returns empty CAPTURE buffers; they are not decoded frames.
	 * Keep them in the driver while transparent reconfiguration is active.
	 * Reconfiguration submits the held buffers after FLUSH_DONE. For a late
	 * empty completion, resubmit only that buffer: submitting the whole dst
	 * queue here would send buffers which are still owned by firmware twice.
	 */
	if (joan_dec_clean_stream && joan_dec_transparent_reconfig &&
	    V4L2_TYPE_IS_CAPTURE(type) && !bytesused &&
	    !(hfi_flags & HFI_BUFFERFLAG_EOS) && !inst->joan_dec_stopping) {
		int ret = 0;

		inst->joan_dec_empty_recycled++;
		v4l2_m2m_buf_queue(inst->m2m_ctx, vbuf);
		if (!inst->reconfig)
			ret = venus_helper_process_one_buf(inst, vbuf);
		if (ret) {
			venus_helper_find_buf(inst, type, tag);
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		}
		return;
	}

	/*
	 * 候选 60：SOURCE_CHANGE 之后固件会把旧的捕获缓冲原样退回（bytesused=0）。
	 * 如果把这些空完成照常交给用户态，DQBUF 每次都“成功”，ffmpeg(v4l2m2m)
	 * 就永远不会进入 poll 分支去取 SOURCE_CHANGE 事件，也就不会重配捕获队列。
	 * 这里直接丢弃这类空完成（不放回 m2m 队列，避免重复完成），
	 * 让用户态阻塞在 DQBUF -> poll -> 收到事件 -> 走标准 reinit。
	 */
	/*
	 * 候选 158：空捕获完成不一定是"真的空帧"——重配/flush 之后固件会把
	 * 旧捕获缓冲原样退回（bytesused=0）。原来只有 inst->reconfig 为真时才
	 * 拦截，而重配完成后该标志已清，这些空完成就掉进下面
	 * "!bytesused → VB2_BUF_STATE_ERROR"，ffmpeg 于是把每一帧都当错误丢弃，
	 * 编码器一帧也拿不到。这里改成不看 reconfig 标志。
	 */
	if (V4L2_TYPE_IS_CAPTURE(type) && inst->reconfig && !bytesused &&
	    joan_dec_drop_empty) {
		if (joan_dec_transparent_reconfig) {
			/* 放回 m2m 队列，再由公共 helper 重新提交给固件 */
			v4l2_m2m_buf_queue(inst->m2m_ctx, vbuf);
			venus_helper_process_initial_cap_bufs(inst);
			JOAN_VTRACE("vdec_buf_done: 重新提交空捕获缓冲 tag=%u", tag);
		} else {
			JOAN_VTRACE("vdec_buf_done: 丢弃 reconfig 后的空捕获完成 tag=%u", tag);
		}
		return;
	}

	vbuf->flags = flags;
	vbuf->field = V4L2_FIELD_NONE;
	vb = &vbuf->vb2_buf;

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		vb2_set_plane_payload(vb, 0, bytesused);
		vb->planes[0].data_offset = data_offset;
		vb->timestamp = timestamp_us * NSEC_PER_USEC;
		vbuf->sequence = inst->sequence_cap++;

		if (vbuf->flags & V4L2_BUF_FLAG_LAST) {
			const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };

			v4l2_event_queue_fh(&inst->fh, &ev);

			if (inst->codec_state == VENUS_DEC_STATE_DRAIN) {
				inst->drain_active = false;
				inst->codec_state = VENUS_DEC_STATE_STOPPED;
			}
		} else {
			if (!bytesused)
				state = VB2_BUF_STATE_ERROR;
		}
	} else {
		vbuf->sequence = inst->sequence_out++;
	}

	venus_helper_get_ts_metadata(inst, timestamp_us, vbuf);

	if (hfi_flags & HFI_BUFFERFLAG_READONLY)
		venus_helper_acquire_buf_ref(vbuf);

	if (hfi_flags & HFI_BUFFERFLAG_DATACORRUPT)
		state = VB2_BUF_STATE_ERROR;

	if (hfi_flags & HFI_BUFFERFLAG_DROP_FRAME) {
		state = VB2_BUF_STATE_ERROR;
		vb2_set_plane_payload(vb, 0, 0);
		vb->timestamp = 0;
	}

	/*
	 * Raw H264 packets without PTS reach V4L2 with timestamp zero. FFmpeg
	 * copies that zero back to every decoded frame, and the msm8998 encoder
	 * rejects the next frame with HFI_ERR_SESSION_INVALID_PARAMETER:
	 * "Venus Encoder: invalid timestamp received!". Repair at CAPTURE so
	 * FFmpeg also sees progressing PTS before its frame synchronization.
	 * Keep increasing client timestamps and leave empty EOS untouched.
	 */
	if (joan_dec_ts_fallback && V4L2_TYPE_IS_CAPTURE(type) &&
	    !inst->joan_dec_stopping &&
	    (!(vbuf->flags & V4L2_BUF_FLAG_LAST) || bytesused)) {
		u64 original = vb->timestamp;
		u64 step = div_u64((u64)NSEC_PER_SEC *
				   inst->timeperframe.numerator,
				   inst->timeperframe.denominator ?: 30);

		step = max_t(u64, step, NSEC_PER_USEC);
		if (inst->joan_dec_ts_seen && original <= inst->joan_dec_last_ts) {
			vb->timestamp = inst->joan_dec_last_ts + step;
			if (inst->joan_dec_ts_fixups++ < 8)
				joan_tlog("[dec-ts] id=%#x in=%llu out=%llu step=%llu\n",
					  inst->id, original, vb->timestamp, step);
		}
		inst->joan_dec_last_ts = vb->timestamp;
		inst->joan_dec_ts_seen = true;
	}

	v4l2_m2m_buf_done(vbuf, state);
}

/*
 * 重配主体。调用者必须持 inst->lock（见下面的 seq_event_work）。
 * 原先是独立 work 的 joan_reconf_work_fn；改成 helper 后由事件 worker 直接
 * 调用，保证"事件 A 的参数更新 + 重配"整体完成之后才处理事件 B。
 */
static void joan_reconfigure_locked(struct venus_inst *inst)
{
	int r;

	lockdep_assert_held(&inst->lock);

	r = hfi_session_flush(inst, HFI_FLUSH_OUTPUT, true);
	if (!r) {
		r = vdec_reconfigure(inst);
		if (!r) {
			/*
			 * 候选 128：重配后把还在 m2m 源队列里的输入缓冲补提交一次，
			 * 否则解码在这次 SOURCE_CHANGE 之后就停住（转码时表现为
			 * 解码会话报 capture 错误、一帧都出不来）。
			 */
			int r2 = venus_helper_process_initial_out_bufs(inst);

			JOAN_VTRACE("joan_reconf: 重配完成 补提交输入=%d", r2);
		} else {
			JOAN_VTRACE("joan_reconf: vdec_reconfigure=%d", r);
		}
	} else {
		JOAN_VTRACE("joan_reconf: flush=%d", r);
	}
}

/*
 * 应用一次 SOURCE_CHANGE。调用者必须持 inst->lock。
 * 供 vdec_seq_event_work 调用。
 */
static void vdec_event_change_locked(struct venus_inst *inst,
				     const struct hfi_event_data *ev_data,
				     bool sufficient)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION };
	struct device *dev = inst->core->dev_dec;
	struct v4l2_format format = {};

	lockdep_assert_held(&inst->lock);

	/*
	 * 事件里 width/height 为 0 时（固件只发 24B 头、没有属性数据），
	 * 后续会把 inst->width/height、fw_min_cnt、bit_depth 全冲成 0，
	 * 还会把 fmt_cap 打回 NV12。这里只标记需要 reconfig，不动会话参数。
	 */
	if (!ev_data->width || !ev_data->height) {
		JOAN_VTRACE("event_change: zero-size event (w=%u h=%u), mark reconfig only",
			    ev_data->width, ev_data->height);
		inst->reconfig = true;
		return;
	}

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = inst->fmt_cap->pixfmt;
	format.fmt.pix_mp.width = ev_data->width;
	format.fmt.pix_mp.height = ev_data->height;

	vdec_try_fmt_common(inst, &format);

	inst->width = format.fmt.pix_mp.width;
	inst->height = format.fmt.pix_mp.height;
	/*
	 * Some versions of the firmware do not report crop information for
	 * all codecs. For these cases, set the crop to the coded resolution.
	 */
	if (ev_data->input_crop.width > 0 && ev_data->input_crop.height > 0) {
		inst->crop.left = ev_data->input_crop.left;
		inst->crop.top = ev_data->input_crop.top;
		inst->crop.width = ev_data->input_crop.width;
		inst->crop.height = ev_data->input_crop.height;
	} else {
		inst->crop.left = 0;
		inst->crop.top = 0;
		inst->crop.width = ev_data->width;
		inst->crop.height = ev_data->height;
	}

	inst->fw_min_cnt = ev_data->buf_count;
	/* overwriting this to 11 for vp9 due to fw bug */
	if (inst->hfi_codec == HFI_VIDEO_CODEC_VP9)
		inst->fw_min_cnt = 11;

	inst->out_width = ev_data->width;
	inst->out_height = ev_data->height;

	if (inst->bit_depth != ev_data->bit_depth) {
		inst->bit_depth = ev_data->bit_depth;
		if (inst->bit_depth == VIDC_BITDEPTH_10)
			inst->fmt_cap = &vdec_formats[VENUS_FMT_P010];
		else
			inst->fmt_cap = &vdec_formats[VENUS_FMT_NV12];
	}

	if (inst->pic_struct != ev_data->pic_struct)
		inst->pic_struct = ev_data->pic_struct;

	dev_dbg(dev, VDBGM "event %s sufficient resources (%ux%u)\n",
		sufficient ? "" : "not", ev_data->width, ev_data->height);

	switch (inst->codec_state) {
	case VENUS_DEC_STATE_INIT:
		inst->codec_state = VENUS_DEC_STATE_CAPTURE_SETUP;
		break;
	case VENUS_DEC_STATE_DECODING:
	case VENUS_DEC_STATE_DRAIN:
		inst->codec_state = VENUS_DEC_STATE_DRC;
		break;
	default:
		break;
	}

	/*
	 * The assumption is that the firmware have to return the last buffer
	 * before this event is received in the v4l2 driver. Also the firmware
	 * itself doesn't mark the last decoder output buffer with HFI EOS flag.
	 */

	if (inst->codec_state == VENUS_DEC_STATE_DRC) {
		int ret;

		inst->next_buf_last = true;

		ret = hfi_session_flush(inst, HFI_FLUSH_OUTPUT, false);
		if (ret)
			dev_dbg(dev, VDBGH "flush output error %d\n", ret);
	}

	inst->next_buf_last = true;
	inst->reconfig = true;

	if (joan_dec_transparent_reconfig) {
		/*
		 * 候选 81：不能在这个上下文里同步重配 —— 事件处理跑在 HFI 消息线程上，
		 * 而 vdec_reconfigure() 里的 flush/output_conf 都要等固件回包，同步调用
		 * 必然超时（实测 -110）。改成丢给工作队列。
		 * 候选 82：只在状态刚变成 CAPTURE_SETUP 时重配一次，否则固件会持续
		 * 推 SOURCE_CHANGE，形成事件风暴。
		 */
		if (inst->codec_state == VENUS_DEC_STATE_CAPTURE_SETUP)
			joan_reconfigure_locked(inst);
		wake_up(&inst->reconf_wait);
		return;
	}

	JOAN_VTRACE("event_change: 队列前 pending=%d subscribed=%#x",
		    v4l2_event_pending(&inst->fh), inst->subscriptions);
	v4l2_event_queue_fh(&inst->fh, &ev);
	JOAN_VTRACE("event_change: 队列后 pending=%d", v4l2_event_pending(&inst->fh));
	wake_up(&inst->reconf_wait);
}

/*
 * 消费 SOURCE_CHANGE 队列。
 *
 * 每取一个事件都先拿 inst->lock，再短暂拿 seq_lock，这样不会出现
 * "worker 已经私藏事件、stop 清空 FIFO 后它才拿到锁" 的生命周期漏洞。
 */
static void vdec_seq_event_work(struct work_struct *work)
{
	struct venus_inst *inst = container_of(work, struct venus_inst,
					      seq_event_work);

	for (;;) {
		struct venus_seq_event ev;
		unsigned long flags;
		bool have = false;

		mutex_lock(&inst->lock);

		spin_lock_irqsave(&inst->seq_lock, flags);
		if (!inst->seq_closing && !inst->seq_fault && inst->seq_count) {
			ev = inst->seq_fifo[inst->seq_head];
			inst->seq_head = (inst->seq_head + 1) % VENUS_SEQ_DEPTH;
			inst->seq_count--;
			have = (ev.gen == inst->seq_gen);
			if (!inst->seq_count)
				inst->seq_pending = false;
		}
		spin_unlock_irqrestore(&inst->seq_lock, flags);

		if (!have) {
			mutex_unlock(&inst->lock);
			break;
		}

		vdec_event_change_locked(inst, &ev.ev, ev.sufficient);
		mutex_unlock(&inst->lock);
	}
}

static void vdec_event_notify(struct venus_inst *inst, u32 event,
			      struct hfi_event_data *data)
{
	struct venus_core *core = inst->core;
	struct device *dev = core->dev_dec;

	vdec_pm_touch(inst);

	switch (event) {
	case EVT_SESSION_ERROR:
		inst->session_error = true;
		venus_helper_vb2_queue_error(inst);
		dev_err(dev, "dec: event session error %x\n", inst->error);
		break;
	case EVT_SYS_EVENT_CHANGE:
		switch (data->event_type) {
		case HFI_EVENT_DATA_SEQUENCE_CHANGED_SUFFICIENT_BUF_RESOURCES:
			vdec_queue_seq_event(inst, data, true);
			break;
		case HFI_EVENT_DATA_SEQUENCE_CHANGED_INSUFFICIENT_BUF_RESOURCES:
			vdec_queue_seq_event(inst, data, false);
			break;
		case HFI_EVENT_RELEASE_BUFFER_REFERENCE:
			venus_helper_release_buf_ref(inst, data->tag);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void vdec_flush_done(struct venus_inst *inst)
{
	dev_dbg(inst->core->dev_dec, VDBGH "flush done\n");
}

static const struct hfi_inst_ops vdec_hfi_ops = {
	.buf_done = vdec_buf_done,
	.event_notify = vdec_event_notify,
	.flush_done = vdec_flush_done,
};

static void vdec_inst_init(struct venus_inst *inst)
{
	inst->hfi_codec = HFI_VIDEO_CODEC_H264;
	inst->fmt_out = &vdec_formats[VENUS_FMT_H264];
	inst->fmt_cap = &vdec_formats[VENUS_FMT_NV12];
	inst->width = frame_width_min(inst);
	inst->height = ALIGN(frame_height_min(inst), 32);
	inst->crop.left = 0;
	inst->crop.top = 0;
	inst->crop.width = inst->width;
	inst->crop.height = inst->height;
	inst->fw_min_cnt = 8;
	inst->out_width = frame_width_min(inst);
	inst->out_height = frame_height_min(inst);
	inst->fps = 30;
	inst->timeperframe.numerator = 1;
	inst->timeperframe.denominator = 30;
	inst->opb_buftype = HFI_BUFFER_OUTPUT;
}

static void vdec_m2m_device_run(void *priv)
{
}

static const struct v4l2_m2m_ops vdec_m2m_ops = {
	.device_run = vdec_m2m_device_run,
	.job_abort = venus_helper_m2m_job_abort,
};

static int m2m_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct venus_inst *inst = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &vdec_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->drv_priv = inst;
	src_vq->buf_struct_size = sizeof(struct venus_buffer);
	src_vq->allow_zero_bytesused = 1;
	src_vq->min_queued_buffers = 0;
	src_vq->dev = inst->core->dev;
	src_vq->lock = &inst->ctx_q_lock;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	/*
	 * 解码输出（NV12 帧）是用户态要读的数据。非缓存内存下 CPU 读很慢：
	 * 4K 转码时解码缓冲 -> 编码缓冲的拷贝会把 4K 转码从 33 fps 拖到 19 fps
	 * （实测）。开 cache hints 让 vb2 走缓存映射 + 自动 dma_sync。
	 */
	dst_vq->allow_cache_hints = 1;
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->ops = &vdec_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->drv_priv = inst;
	dst_vq->buf_struct_size = sizeof(struct venus_buffer);
	dst_vq->allow_zero_bytesused = 1;
	dst_vq->min_queued_buffers = 0;
	dst_vq->dev = inst->core->dev;
	dst_vq->lock = &inst->ctx_q_lock;
	return vb2_queue_init(dst_vq);
}

static int vdec_open(struct file *file)
{
	struct venus_core *core = video_drvdata(file);
	struct venus_inst *inst;
	int ret;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	INIT_LIST_HEAD(&inst->dpbbufs);
	INIT_LIST_HEAD(&inst->registeredbufs);
	INIT_LIST_HEAD(&inst->internalbufs);
	INIT_LIST_HEAD(&inst->list);
	mutex_init(&inst->lock);
	mutex_init(&inst->ctx_q_lock);

	inst->core = core;
	inst->session_type = VIDC_SESSION_TYPE_DEC;
	inst->num_output_bufs = 1;
	inst->codec_state = VENUS_DEC_STATE_DEINIT;
	inst->buf_count = 0;
	inst->clk_data.core_id = VIDC_CORE_ID_DEFAULT;
	inst->core_acquired = false;
	inst->bit_depth = VIDC_BITDEPTH_8;
	inst->pic_struct = HFI_INTERLACE_FRAME_PROGRESSIVE;
	init_waitqueue_head(&inst->reconf_wait);
	spin_lock_init(&inst->seq_lock);
	INIT_WORK(&inst->seq_event_work, vdec_seq_event_work);
	inst->seq_accepting = false;
	inst->seq_closing = false;
	inst->seq_fault = false;
	inst->seq_pending = false;
	inst->seq_capture_started = false;
	inst->seq_head = 0;
	inst->seq_count = 0;
	inst->seq_gen = 0;
	inst->nonblock = file->f_flags & O_NONBLOCK;

	venus_helper_init_instance(inst);

	ret = vdec_ctrl_init(inst);
	if (ret)
		goto err_free;

	vdec_inst_init(inst);

	ida_init(&inst->dpb_ids);

	/*
	 * create m2m device for every instance, the m2m context scheduling
	 * is made by firmware side so we do not need to care about.
	 */
	inst->m2m_dev = v4l2_m2m_init(&vdec_m2m_ops);
	if (IS_ERR(inst->m2m_dev)) {
		ret = PTR_ERR(inst->m2m_dev);
		goto err_ctrl_deinit;
	}

	inst->m2m_ctx = v4l2_m2m_ctx_init(inst->m2m_dev, inst, m2m_queue_init);
	if (IS_ERR(inst->m2m_ctx)) {
		ret = PTR_ERR(inst->m2m_ctx);
		goto err_m2m_dev_release;
	}

	ret = hfi_session_create(inst, &vdec_hfi_ops);
	if (ret)
		goto err_m2m_ctx_release;

	v4l2_fh_init(&inst->fh, core->vdev_dec);

	inst->fh.ctrl_handler = &inst->ctrl_handler;
	v4l2_fh_add(&inst->fh, file);
	inst->fh.m2m_ctx = inst->m2m_ctx;

	return 0;

err_m2m_ctx_release:
	v4l2_m2m_ctx_release(inst->m2m_ctx);
err_m2m_dev_release:
	v4l2_m2m_release(inst->m2m_dev);
err_ctrl_deinit:
	v4l2_ctrl_handler_free(&inst->ctrl_handler);
err_free:
	kfree(inst);
	return ret;
}

static int vdec_close(struct file *file)
{
	struct venus_inst *inst = to_inst(file);

	vdec_pm_get(inst);

	/*
	 * 先把所有可能产生工作的入口封住，再取消工作。
	 * 顺序很重要：先设 closing，让还没进来的回调尽快返回；
	 * 然后在**不持** inst->lock 的前提下等 IRQ 排空、取消 work，
	 * 否则 worker 若正等 inst->lock 就会自己卡住自己。
	 */
	mutex_lock(&inst->lock);
	inst->joan_dec_stopping = true;
	inst->seq_capture_started = false;
	mutex_unlock(&inst->lock);

	{
		unsigned long flags;

		spin_lock_irqsave(&inst->seq_lock, flags);
		inst->seq_closing = true;
		inst->seq_accepting = false;
		inst->seq_gen++;
		inst->seq_head = 0;
		inst->seq_count = 0;
		inst->seq_pending = false;
		spin_unlock_irqrestore(&inst->seq_lock, flags);
	}

	/* 等已经越过 closing 检查的旧回调结束 */
	synchronize_irq(inst->core->irq);

	cancel_work_sync(&inst->seq_event_work);
	cancel_work_sync(&inst->delayed_process_work);

	venus_close_common(inst, file);

	/*
	 * common 已经把实例从 HFI 查找表摘除。再同步一次，等摘除之前
	 * 已经取到 inst 指针的消息处理者结束，之后才能释放。
	 */
	synchronize_irq(inst->core->irq);

	ida_destroy(&inst->dpb_ids);
	vdec_pm_put(inst, false);

	kfree(inst);
	return 0;
}

static const struct v4l2_file_operations vdec_fops = {
	.owner = THIS_MODULE,
	.open = vdec_open,
	.release = vdec_close,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

static int vdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	struct venus_core *core;
	int ret;

	if (!dev->parent)
		return -EPROBE_DEFER;

	core = dev_get_drvdata(dev->parent);
	if (!core)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, core);

	if (core->pm_ops->vdec_get) {
		ret = core->pm_ops->vdec_get(dev);
		if (ret)
			return ret;
	}

	vdev = video_device_alloc();
	if (!vdev)
		return -ENOMEM;

	strscpy(vdev->name, "qcom-venus-decoder", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &vdec_fops;
	vdev->ioctl_ops = &vdec_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &core->v4l2_dev;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_vdev_release;

	core->vdev_dec = vdev;
	core->dev_dec = dev;

	video_set_drvdata(vdev, core);
	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	return 0;

err_vdev_release:
	video_device_release(vdev);
	return ret;
}

static void vdec_remove(struct platform_device *pdev)
{
	struct venus_core *core = dev_get_drvdata(pdev->dev.parent);

	video_unregister_device(core->vdev_dec);
	pm_runtime_disable(core->dev_dec);

	if (core->pm_ops->vdec_put)
		core->pm_ops->vdec_put(core->dev_dec);
}

static __maybe_unused int vdec_runtime_suspend(struct device *dev)
{
	struct venus_core *core = dev_get_drvdata(dev);
	const struct venus_pm_ops *pm_ops = core->pm_ops;
	int ret = 0;

	if (pm_ops->vdec_power)
		ret = pm_ops->vdec_power(dev, POWER_OFF);

	return ret;
}

static __maybe_unused int vdec_runtime_resume(struct device *dev)
{
	struct venus_core *core = dev_get_drvdata(dev);
	const struct venus_pm_ops *pm_ops = core->pm_ops;
	int ret = 0;

	if (pm_ops->vdec_power)
		ret = pm_ops->vdec_power(dev, POWER_ON);

	return ret;
}

static const struct dev_pm_ops vdec_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(vdec_runtime_suspend, vdec_runtime_resume, NULL)
};

static const struct of_device_id vdec_dt_match[] = {
	{ .compatible = "venus-decoder" },
	{ }
};
MODULE_DEVICE_TABLE(of, vdec_dt_match);

static struct platform_driver qcom_venus_dec_driver = {
	.probe = vdec_probe,
	.remove = vdec_remove,
	.driver = {
		.name = "qcom-venus-decoder",
		.of_match_table = vdec_dt_match,
		.pm = &vdec_pm_ops,
	},
};
module_platform_driver(qcom_venus_dec_driver);

MODULE_ALIAS("platform:qcom-venus-decoder");
MODULE_DESCRIPTION("Qualcomm Venus video decoder driver");
MODULE_LICENSE("GPL v2");
