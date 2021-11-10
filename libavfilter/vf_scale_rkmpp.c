/*
* Copyright (c) 2021, RKMPP CORPORATION. All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
*/

#include <float.h>
#include <stdio.h>
#include <string.h>

#include "libavutil/avstring.h"
#include "libavutil/eval.h"
#include "libavutil/common.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"


#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "scale_eval.h"
#include "video.h"

#include "vf_scale_rkmpp.h"
#include "libavcodec/rkmppdec.h"


#define ZSPACE_SCALE_DEBUG 1
#if ZSPACE_SCALE_DEBUG
#define ZSPACE_SCALE_DEBUG_LEVEL AV_LOG_WARNING
#else
#define ZSPACE_SCALE_DEBUG_LEVEL AV_LOG_INFO
#endif

//mpp_common.h
#ifndef MPP_ALIGN
#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))
#define SZ_1K                   (1024)
#define SZ_2K                   (SZ_1K*2)
#define SZ_4K                   (SZ_1K*4)
#define SZ_8K                   (SZ_1K*8)
#define SZ_16K                  (SZ_1K*16)
#define SZ_32K                  (SZ_1K*32)
#define SZ_64K                  (SZ_1K*64)
#define SZ_128K                 (SZ_1K*128)
#define SZ_256K                 (SZ_1K*256)
#define SZ_512K                 (SZ_1K*512)
#define SZ_1M                   (SZ_1K*SZ_1K)
#define SZ_2M                   (SZ_1M*2)
#define SZ_4M                   (SZ_1M*4)
#define SZ_8M                   (SZ_1M*8)
#define SZ_16M                  (SZ_1M*16)
#define SZ_32M                  (SZ_1M*32)
#define SZ_64M                  (SZ_1M*64)
#define SZ_80M                  (SZ_1M*80)
#define SZ_128M                 (SZ_1M*128)
#endif


static const enum AVPixelFormat rkmpp_scale_supported_formats[] = {
    AV_PIX_FMT_DRM_PRIME,
    AV_PIX_FMT_DRM_PRIME_P010BE,
};

#define DIV_UP(a, b) ( ((a) + (b) - 1) / (b) )

static const char *const var_names[] = {
    "in_w",   "iw",
    "in_h",   "ih",
    "out_w",  "ow",
    "out_h",  "oh",
    "a",
    "sar",
    "dar",
    "hsub",
    "vsub",
    "ohsub",
    "ovsub",
    "n",
    "t",
    "pos",
    "main_w",
    "main_h",
    "main_a",
    "main_sar",
    "main_dar", "mdar",
    "main_hsub",
    "main_vsub",
    "main_n",
    "main_t",
    "main_pos",
    NULL
};

enum var_name {
    VAR_IN_W,   VAR_IW,
    VAR_IN_H,   VAR_IH,
    VAR_OUT_W,  VAR_OW,
    VAR_OUT_H,  VAR_OH,
    VAR_A,
    VAR_SAR,
    VAR_DAR,
    VAR_HSUB,
    VAR_VSUB,
    VAR_OHSUB,
    VAR_OVSUB,
    VAR_N,
    VAR_T,
    VAR_POS,
    VAR_S2R_MAIN_W,
    VAR_S2R_MAIN_H,
    VAR_S2R_MAIN_A,
    VAR_S2R_MAIN_SAR,
    VAR_S2R_MAIN_DAR, VAR_S2R_MDAR,
    VAR_S2R_MAIN_HSUB,
    VAR_S2R_MAIN_VSUB,
    VAR_S2R_MAIN_N,
    VAR_S2R_MAIN_T,
    VAR_S2R_MAIN_POS,
    VARS_NB
};


typedef struct RKMPPScaleContext {
    const AVClass *class;

    AVDRMDeviceContext *hwctx;

    enum AVPixelFormat in_fmt;
    enum AVPixelFormat out_fmt;

    AVBufferRef *frames_ctx;
    AVFrame     *frame;

    int passthrough;
    int src_w;
    int src_h;
    int dest_w;
    int dest_h;

    /**
     * Output sw format. AV_PIX_FMT_NONE for no conversion.
     */
    enum AVPixelFormat format;

    char *w_expr;               ///< width  expression string
    char *h_expr;               ///< height expression string
    AVExpr *w_pexpr;
    AVExpr *h_pexpr;
    double var_values[VARS_NB];

    int force_original_aspect_ratio;
    int force_divisible_by;

    MppBuffer frm_buf;
    MppBufferGroup buf_grp;
    int frm_buf_size;
} RKMPPScaleContext;

static int rkmppscale_config_props(AVFilterLink *outlink);

static RgaSURF_FORMAT rkmppscale_avpix_format_to_rga_format(int av_fmt) {
    switch (av_fmt) {
        case AV_PIX_FMT_DRM_PRIME:
            return RK_FORMAT_YCbCr_420_SP;
#ifdef DRM_FORMAT_NV12_10
        case AV_PIX_FMT_DRM_PRIME_P010BE:
            return RK_FORMAT_YCbCr_420_SP_10B;
#endif
        default:
            return -1;
    }
}

static int rkmppscale_check_exprs(AVFilterContext *ctx)
{
    RKMPPScaleContext *scale = ctx->priv;
    unsigned vars_w[VARS_NB] = { 0 }, vars_h[VARS_NB] = { 0 };

    if (!scale->w_pexpr && !scale->h_pexpr)
        return AVERROR(EINVAL);

    if (scale->w_pexpr)
        av_expr_count_vars(scale->w_pexpr, vars_w, VARS_NB);
    if (scale->h_pexpr)
        av_expr_count_vars(scale->h_pexpr, vars_h, VARS_NB);

    if (vars_w[VAR_OUT_W] || vars_w[VAR_OW]) {
        av_log(ctx, AV_LOG_ERROR, "Width expression cannot be self-referencing: '%s'.\n", scale->w_expr);
        return AVERROR(EINVAL);
    }

    if (vars_h[VAR_OUT_H] || vars_h[VAR_OH]) {
        av_log(ctx, AV_LOG_ERROR, "Height expression cannot be self-referencing: '%s'.\n", scale->h_expr);
        return AVERROR(EINVAL);
    }

    if ((vars_w[VAR_OUT_H] || vars_w[VAR_OH]) &&
        (vars_h[VAR_OUT_W] || vars_h[VAR_OW])) {
        av_log(ctx, AV_LOG_WARNING, "Circular references detected for width '%s' and height '%s' - possibly invalid.\n", scale->w_expr, scale->h_expr);
    }

    return 0;
}

static int rkmppscale_parse_expr(AVFilterContext *ctx, char *str_expr, AVExpr **pexpr_ptr, const char *var, const char *args)
{
    RKMPPScaleContext *scale = ctx->priv;
    int ret, is_inited = 0;
    char *old_str_expr = NULL;
    AVExpr *old_pexpr = NULL;

    if (str_expr) {
        old_str_expr = av_strdup(str_expr);
        if (!old_str_expr)
            return AVERROR(ENOMEM);
        av_opt_set(scale, var, args, 0);
    }

    if (*pexpr_ptr) {
        old_pexpr = *pexpr_ptr;
        *pexpr_ptr = NULL;
        is_inited = 1;
    }

    ret = av_expr_parse(pexpr_ptr, args, var_names,
                        NULL, NULL, NULL, NULL, 0, ctx);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Cannot parse expression for %s: '%s'\n", var, args);
        goto revert;
    }

    ret = rkmppscale_check_exprs(ctx);
    if (ret < 0)
        goto revert;

    if (is_inited && (ret = rkmppscale_config_props(ctx->outputs[0])) < 0)
        goto revert;

    av_expr_free(old_pexpr);
    old_pexpr = NULL;
    av_freep(&old_str_expr);

    return 0;

revert:
    av_expr_free(*pexpr_ptr);
    *pexpr_ptr = NULL;
    if (old_str_expr) {
        av_opt_set(scale, var, old_str_expr, 0);
        av_free(old_str_expr);
    }
    if (old_pexpr)
        *pexpr_ptr = old_pexpr;

    return ret;
}

static int rkmppscale_init(AVFilterContext *ctx)
{
    av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin!\n", __FUNCTION__, __LINE__);
    RKMPPScaleContext *s = ctx->priv;
    int ret = 0;

    s->format = AV_PIX_FMT_DRM_PRIME;
    s->frame = av_frame_alloc();
    if (!s->frame)
        return AVERROR(ENOMEM);

    ret = mpp_buffer_group_get_internal(&s->buf_grp, MPP_BUFFER_TYPE_DRM);
    if (ret) {
        av_log(ctx, AV_LOG_ERROR, "failed to get mpp buffer group ret %d\n", ret);
        ret = AVERROR_UNKNOWN;
        goto init_fail;
    }
    s->frm_buf = NULL;
    s->frm_buf_size = 0;


init_fail:
    return ret;
}

static void rkmppscale_uninit(AVFilterContext *ctx)
{
    RKMPPScaleContext *s = ctx->priv;

    if (s->hwctx) {
        
    }
    
    if (s->frm_buf) {
        mpp_buffer_put(s->frm_buf);
        s->frm_buf = NULL;
    }

    if (s->buf_grp) {
        mpp_buffer_group_put(s->buf_grp);
        s->buf_grp = NULL;
    }

    av_frame_free(&s->frame);
    av_buffer_unref(&s->frames_ctx);
}

static int rkmppscale_query_formats(AVFilterContext *ctx)
{
    int ret = 0;
    static const enum AVPixelFormat pixel_formats[] = {
        AV_PIX_FMT_DRM_PRIME, AV_PIX_FMT_DRM_PRIME_P010BE, AV_PIX_FMT_NONE,
    };

    AVFilterFormats *pix_fmts = ff_make_format_list(pixel_formats);
    if (!pix_fmts)
        return AVERROR(ENOMEM);

    ret = ff_set_common_formats(ctx, pix_fmts);
    av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] ret = %d.\n", __FUNCTION__, __LINE__, ret);
    return ret;
}


static int rkmpp_format_is_supported(enum AVPixelFormat fmt)
{
    int i;

    av_log(NULL, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] need fmt=%d.\n", __FUNCTION__, __LINE__, fmt);
    for (i = 0; i < FF_ARRAY_ELEMS(rkmpp_scale_supported_formats); i++)
        if (rkmpp_scale_supported_formats[i] == fmt)
            return 1;
    return 0;
}

static int rkmpp_init_processing_chain(AVFilterContext *ctx, int in_width, int in_height,
                                         int out_width, int out_height)
{
    RKMPPScaleContext *s = ctx->priv;

    AVHWFramesContext *in_frames_ctx;

    enum AVPixelFormat in_format;
    enum AVPixelFormat out_format;
    int ret;

    /* check that we have a hw context */
    if (!ctx->inputs[0]->hw_frames_ctx) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext*)ctx->inputs[0]->hw_frames_ctx->data;
    in_format     = in_frames_ctx->format;
    out_format    = (s->format == AV_PIX_FMT_NONE) ? in_format : s->format;
    av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] in_format=%d,out_format=%d,s->format=%d.\n", __FUNCTION__, __LINE__,
        in_format, out_format, s->format);

    if (!rkmpp_format_is_supported(in_format)) {
        av_log(ctx, AV_LOG_ERROR, "Unsupported input format: %s\n",
               av_get_pix_fmt_name(in_format));
        return AVERROR(ENOSYS);
    }
    if (!rkmpp_format_is_supported(out_format)) {
        av_log(ctx, AV_LOG_ERROR, "Unsupported output format: %s\n",
               av_get_pix_fmt_name(out_format));
        return AVERROR(ENOSYS);
    }

    s->in_fmt = in_format;
    s->out_fmt = out_format;

    av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] in_width=%d,in_height=%d,out_width=%d,out_height=%d\n", __FUNCTION__, __LINE__,
            in_width, in_height, out_width, out_height);
    if (s->passthrough && in_width == out_width && in_height == out_height && in_format == out_format) {
        s->frames_ctx = av_buffer_ref(ctx->inputs[0]->hw_frames_ctx);
        if (!s->frames_ctx)
            return AVERROR(ENOMEM);
    } else {
        s->passthrough = 0;
    }

    return 0;
}

static int rkmppscale_config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    RKMPPScaleContext *s  = ctx->priv;
    AVHWFramesContext     *frames_ctx = (AVHWFramesContext*)inlink->hw_frames_ctx->data;
    AVDRMDeviceContext *device_hwctx = frames_ctx->device_ctx->hwctx;
    char buf[64];
    int w = 0, h = 0;
    int ret = 0;
    char *scaler_ptx;
    const char *function_infix = "";

    av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin!\n", __FUNCTION__, __LINE__);
    s->hwctx = device_hwctx;
    s->passthrough = 1;


    if ((ret = ff_scale_eval_dimensions(s,
                                        s->w_expr, s->h_expr,
                                        inlink, outlink,
                                        &w, &h)) < 0)
        goto fail;

    ff_scale_adjust_dimensions(inlink, &w, &h,
                               s->force_original_aspect_ratio, s->force_divisible_by);

    if (((int64_t)h * inlink->w) > INT_MAX  ||
        ((int64_t)w * inlink->h) > INT_MAX)
        av_log(ctx, AV_LOG_ERROR, "Rescaled value for width or height is too big.\n");

    outlink->w = w;
    outlink->h = h;

    ret = rkmpp_init_processing_chain(ctx, inlink->w, inlink->h, w, h);
    if (ret < 0)
        return ret;

    av_log(ctx, AV_LOG_WARNING, "w:%d h:%d -> w:%d h:%d%s\n",
           inlink->w, inlink->h, outlink->w, outlink->h, s->passthrough ? " (passthrough)" : "nopass");

    if (inlink->sample_aspect_ratio.num) {
        av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] .\n", __FUNCTION__, __LINE__);
        outlink->sample_aspect_ratio = av_mul_q((AVRational){outlink->h*inlink->w,
                                                             outlink->w*inlink->h},
                                                inlink->sample_aspect_ratio);
    } else {
        av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] .\n", __FUNCTION__, __LINE__);
        outlink->sample_aspect_ratio = inlink->sample_aspect_ratio;
    }

    s->src_h = inlink->h;
    s->src_w = inlink->w;
    s->dest_h = outlink->h;
    s->dest_w = outlink->w;

    av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] End success!\n", __FUNCTION__, __LINE__);
    return 0;

fail:
    return ret;
}

static uint32_t rkmppscale_mpp_format_to_drm_format(MppFrameFormat mppformat)
{
    switch (mppformat) {
    case MPP_FMT_YUV420SP:          return DRM_FORMAT_NV12;
#ifdef DRM_FORMAT_NV12_10
    case MPP_FMT_YUV420SP_10BIT:    return DRM_FORMAT_NV12_10 | DRM_FORMAT_BIG_ENDIAN;
#endif
    default:                        return 0;
    }
}

static void rkmppscale_release_frame(void *opaque, uint8_t *data)
{
    AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)data;
    AVBufferRef *framecontextref = (AVBufferRef *)opaque;
    RKMPPFrameContext *framecontext = (RKMPPFrameContext *)framecontextref->data;
    MppBuffer buffer = NULL;

    //av_log(NULL, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] rkmppscale release mppframe(%p).\n", __FUNCTION__, __LINE__, (void *)(framecontext->frame));
    buffer = mpp_frame_get_buffer(framecontext->frame);
    //mpp_buffer_put(buffer);
    mpp_frame_deinit(&framecontext->frame);
    av_buffer_unref(&framecontext->decoder_ref);
    av_buffer_unref(&framecontextref);

    av_free(desc);
}

static int call_resize_kernel(AVFilterContext *ctx, AVFrame *out, AVFrame *in)
{
    //av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin!\n", __FUNCTION__, __LINE__);
    RKMPPScaleContext *s = ctx->priv;
    AVHWFramesContext *frames_ctx = NULL;
    AVDRMFrameDescriptor *desc = NULL;
    AVDRMLayerDescriptor *layer = NULL;
    MppFrame srcMppframe = NULL;
    MppBuffer srcBuffer = NULL;
    MppFrame destMppframe = NULL;
    MppBuffer destBuffer = NULL;
    rga_info_t src_info = {0};
    rga_info_t dst_info = {0};
    AVBufferRef *framecontextref = NULL;
    RKMPPFrameContext *framecontext = NULL;
    AVBufferRef *destFramecontextref = NULL;
    RKMPPFrameContext *destFramecontext = NULL;
    int srcHor_stride = 0, srcVer_stride = 0, destHor_stride = 0, destVer_stride = 0;
    int destSize = 0;
    int ret = 0;

    framecontextref = (AVBufferRef *) av_buffer_get_opaque(in->buf[0]);
    framecontext = (RKMPPFrameContext *)framecontextref->data;
    srcMppframe = framecontext->frame;
    srcBuffer = mpp_frame_get_buffer(srcMppframe);
    srcHor_stride = mpp_frame_get_hor_stride(srcMppframe);
    srcVer_stride = mpp_frame_get_ver_stride(srcMppframe);
    src_info.fd = mpp_buffer_get_fd(srcBuffer);
    src_info.mmuFlag = 1;
    rga_set_rect(&src_info.rect, 0, 0, s->src_w, s->src_h,
                 srcHor_stride, srcVer_stride, rkmppscale_avpix_format_to_rga_format(in->format));

    destHor_stride = MPP_ALIGN(s->dest_w, 16);
    destVer_stride = MPP_ALIGN(s->dest_h, 16);
    destSize = destHor_stride * destVer_stride * 3 / 2;
    if (s->frm_buf == NULL || destSize != s->frm_buf_size) {
        if (s->frm_buf) {
            mpp_buffer_put(s->frm_buf);
            s->frm_buf = NULL;
        }
        s->frm_buf_size = destSize;
        ret = mpp_buffer_get(s->buf_grp, &s->frm_buf, s->frm_buf_size);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "failed to get buffer for scale MppBuffer ret %d\n", ret);
            ret = AVERROR_UNKNOWN;
            goto resize_exit;
        }
    }
    destBuffer = s->frm_buf;
    //mpp_buffer_get(NULL, &destBuffer, destSize);
    dst_info.fd = mpp_buffer_get_fd(destBuffer);
    dst_info.mmuFlag = 1;
    rga_set_rect(&dst_info.rect, 0, 0, s->dest_w, s->dest_h,
                 destHor_stride, destVer_stride, RK_FORMAT_YCbCr_420_SP);

    if ((ret = c_RkRgaBlit(&src_info, &dst_info, NULL)) < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to do rga blit\n");
        goto resize_exit;
    }

    ret = mpp_frame_init(&destMppframe);
    if (ret) {
        av_log(NULL, AV_LOG_ERROR, "mpp_frame_init failed (%d)\n", ret);
        goto resize_exit;
    }

    mpp_frame_set_width(destMppframe, s->dest_w);
    mpp_frame_set_height(destMppframe, s->dest_h);
    mpp_frame_set_hor_stride(destMppframe, destHor_stride);
    mpp_frame_set_ver_stride(destMppframe, destVer_stride);
    mpp_frame_set_fmt(destMppframe, MPP_FMT_YUV420SP);
    mpp_frame_set_pts(destMppframe, in->pts);
    mpp_frame_set_buffer(destMppframe, destBuffer);

    out->crop_left        = 0;
    out->crop_right       = s->dest_w - mpp_frame_get_width(destMppframe);
    out->crop_top         = 0;
    out->crop_bottom      = s->dest_h - mpp_frame_get_height(destMppframe);
    out->width = s->dest_w;
    out->height = s->dest_h;
    

    desc = av_mallocz(sizeof(AVDRMFrameDescriptor));
    if (!desc) {
        ret = AVERROR(ENOMEM);
        goto resize_exit;
    }

    desc->nb_objects = 1;
    desc->objects[0].fd = mpp_buffer_get_fd(destBuffer);
    desc->objects[0].size = mpp_buffer_get_size(destBuffer);
    desc->objects[0].ptr = mpp_buffer_get_ptr(destBuffer);

    desc->nb_layers = 1;
    layer = &desc->layers[0];
    layer->format = rkmppscale_mpp_format_to_drm_format(mpp_frame_get_fmt(srcMppframe));
    layer->nb_planes = 2;

    layer->planes[0].object_index = 0;
    layer->planes[0].offset = 0;
    layer->planes[0].pitch = destHor_stride;

    layer->planes[1].object_index = 0;
    layer->planes[1].offset = layer->planes[0].pitch * destVer_stride;
    layer->planes[1].pitch = layer->planes[0].pitch;

    // we also allocate a struct in buf[0] that will allow to hold additionnal information
    // for releasing properly MPP frames and decoder
    destFramecontextref = av_buffer_allocz(sizeof(*destFramecontext));
    if (!destFramecontextref) {
        ret = AVERROR(ENOMEM);
        goto resize_exit;
    }

    // MPP decoder needs to be closed only when all frames have been released.
    destFramecontext = (RKMPPFrameContext *)destFramecontextref->data;
    destFramecontext->decoder_ref = av_buffer_ref(framecontext->decoder_ref);
    destFramecontext->frame = destMppframe;
    //av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] rkmppscale mppframe(%p), frame(%p).\n", __FUNCTION__, __LINE__, (void *)destMppframe, (void *)out);

    out->data[0]  = (uint8_t *)desc;
    out->buf[0]   = av_buffer_create((uint8_t *)desc, sizeof(*desc), rkmppscale_release_frame,
                                       destFramecontextref, AV_BUFFER_FLAG_READONLY);

    if (!out->buf[0]) {
        ret = AVERROR(ENOMEM);
        goto resize_exit;
    }

    out->hw_frames_ctx = av_buffer_ref(in->hw_frames_ctx);
    if (!out->hw_frames_ctx) {
        ret = AVERROR(ENOMEM);
        goto resize_exit;
    }

    return 0;
resize_exit:

    if (destMppframe)
        mpp_frame_deinit(&destMppframe);

    if (destFramecontext)
        av_buffer_unref(&destFramecontext->decoder_ref);

    if (destFramecontextref)
        av_buffer_unref(&destFramecontextref);

    if (desc)
        av_free(desc);
    return ret;
}

static void rkmppscale_setinfo_avframe(AVFrame *outframe, AVFrame *inframe) {

    outframe->pts              = inframe->pts;
    outframe->pkt_pts          = inframe->pkt_pts;

    outframe->reordered_opaque = inframe->reordered_opaque;
    outframe->color_range      = inframe->color_range;
    outframe->color_primaries  = inframe->color_primaries;
    outframe->color_trc        = inframe->color_trc;
    outframe->colorspace       = inframe->colorspace;

    outframe->interlaced_frame = inframe->interlaced_frame;
    outframe->top_field_first  = inframe->top_field_first;
}

static int rkmppscale_resize(AVFilterContext *ctx,
                            AVFrame *out, AVFrame *in)
{
    //av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin!\n", __FUNCTION__, __LINE__);
    RKMPPScaleContext *s = ctx->priv;
    int ret = 0;

    switch (s->out_fmt) {
        case AV_PIX_FMT_DRM_PRIME:
            out->format = s->out_fmt;
            ret = call_resize_kernel(ctx, out, in);
            if (ret < 0) {
                av_log(ctx, AV_LOG_ERROR, "[zspace] [%s:%d] call_resize_kernel(%d) failed!\n", __FUNCTION__, __LINE__, ret);
                break;
            }
            rkmppscale_setinfo_avframe(out, in);
            break;
        case AV_PIX_FMT_NV12:
        default:
            return AVERROR_BUG;
    }

    return ret;
}

static int rkmppscale_scale(AVFilterContext *ctx, AVFrame *out, AVFrame *in)
{
    //av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin!\n", __FUNCTION__, __LINE__);
    RKMPPScaleContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    int ret;

    ret = rkmppscale_resize(ctx, out, in);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "[zspace] [%s:%d] rkmppscale_resize(%d) failed!\n", __FUNCTION__, __LINE__, ret);
        return ret;
    }


    ret = av_frame_copy_props(out, in);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "[zspace] [%s:%d] av_frame_copy_props(%d) failed!\n", __FUNCTION__, __LINE__, ret);
        return ret;
    }

    return 0;
}

static int rkmppscale_filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext       *ctx = link->dst;
    RKMPPScaleContext        *s = ctx->priv;
    AVFilterLink      *outlink = ctx->outputs[0];
    AVFrame *out = NULL;
    int ret = 0;

    //av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin!\n", __FUNCTION__, __LINE__);
    if (s->passthrough)
        return ff_filter_frame(outlink, in);

    out = av_frame_alloc();
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    ret = rkmppscale_scale(ctx, out, in);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "rkmpp scale failed!\n");
        goto fail;
    }

    av_reduce(&out->sample_aspect_ratio.num, &out->sample_aspect_ratio.den,
              (int64_t)in->sample_aspect_ratio.num * outlink->h * link->w,
              (int64_t)in->sample_aspect_ratio.den * outlink->w * link->h,
              INT_MAX);

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
fail:
    av_frame_free(&in);
    av_frame_free(&out);
    return ret;
}

static AVFrame *rkmppscale_get_video_buffer(AVFilterLink *inlink, int w, int h)
{
    RKMPPScaleContext *s = inlink->dst->priv;
    AVFrame * ret_frame = NULL;
    av_log(NULL, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin!\n", __FUNCTION__, __LINE__);


    av_log(NULL, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] End ret_frame=%p!\n", __FUNCTION__, __LINE__, ret_frame);
    return ret_frame;
}

static int rkmppscale_process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    RKMPPScaleContext *scale = ctx->priv;
    char *str_expr;
    AVExpr **pexpr_ptr;
    int ret, w, h;

    w = !strcmp(cmd, "width")  || !strcmp(cmd, "w");
    h = !strcmp(cmd, "height")  || !strcmp(cmd, "h");

    if (w || h) {
        str_expr = w ? scale->w_expr : scale->h_expr;
        pexpr_ptr = w ? &scale->w_pexpr : &scale->h_pexpr;

        ret = rkmppscale_parse_expr(ctx, str_expr, pexpr_ptr, cmd, args);
    } else
        ret = AVERROR(ENOSYS);

    if (ret < 0)
        av_log(ctx, AV_LOG_ERROR, "Failed to process command. Continuing with existing parameters.\n");

    return ret;
}

#define OFFSET(x) offsetof(RKMPPScaleContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption options[] = {
    { "w", "Output video width",  OFFSET(w_expr), AV_OPT_TYPE_STRING, { .str = "iw" }, .flags = FLAGS },
    { "h", "Output video height", OFFSET(h_expr), AV_OPT_TYPE_STRING, { .str = "ih" }, .flags = FLAGS },
    { "passthrough", "Do not process frames at all if parameters match", OFFSET(passthrough), AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, .flags = FLAGS },
    { "force_original_aspect_ratio", "decrease or increase w/h if necessary to keep the original AR", OFFSET(force_original_aspect_ratio), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 2, FLAGS, "force_oar" },
    { "force_divisible_by", "enforce that the output resolution is divisible by a defined integer when force_original_aspect_ratio is used", OFFSET(force_divisible_by), AV_OPT_TYPE_INT, { .i64 = 1 }, 1, 256, FLAGS },
    { NULL },
};

static const AVClass rkmppscale_class = {
    .class_name = "rkmppscale",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const AVFilterPad rkmppscale_inputs[] = {
    {
        .name        = "default",
        .type        = AVMEDIA_TYPE_VIDEO,
        .filter_frame = rkmppscale_filter_frame,
        .get_video_buffer = rkmppscale_get_video_buffer,
    },
    { NULL }
};

static const AVFilterPad rkmppscale_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = rkmppscale_config_props,
    },
    { NULL }
};

AVFilter ff_vf_scale_rkmpp = {
    .name      = "scale_rkmpp",
    .description = NULL_IF_CONFIG_SMALL("RGA accelerated video resizer"),

    .init          = rkmppscale_init,
    .uninit        = rkmppscale_uninit,
    .query_formats = rkmppscale_query_formats,

    .priv_size = sizeof(RKMPPScaleContext),
    .priv_class = &rkmppscale_class,

    .inputs    = rkmppscale_inputs,
    .outputs   = rkmppscale_outputs,

    .process_command = rkmppscale_process_command,
};
