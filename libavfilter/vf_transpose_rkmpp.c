/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include <string.h>

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"

#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "transpose.h"
#include "libavcodec/rkmppdec.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"

#define ZSPACE_TRANSPOSE_DEBUG 1
#if ZSPACE_TRANSPOSE_DEBUG
#define ZSPACE_TRANSPOSE_DEBUG_LEVEL AV_LOG_WARNING
#else
#define ZSPACE_TRANSPOSE_DEBUG_LEVEL AV_LOG_INFO
#endif

static const enum AVPixelFormat transpose_rkmpp_supported_formats[] = {
    AV_PIX_FMT_DRM_PRIME,
    AV_PIX_FMT_DRM_PRIME_P010BE,
};

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

typedef struct TransposeRKMPPContext {
    AVDRMDeviceContext *hwctx; // must be the first field
    int passthrough;         // PassthroughType, landscape passthrough mode enabled
    int dir;                 // TransposeDir

    int rotation_state;

    int output_width;   // computed width
    int output_height;  // computed height
    int src_w;
    int src_h;

    /**
     * Output  format. AV_PIX_FMT_NONE for no conversion.
     */
    enum AVPixelFormat output_format;
    enum AVPixelFormat input_format;

    AVBufferRef *frames_ctx;
    AVFrame     *frame;

    MppBuffer frm_buf;
    MppBufferGroup buf_grp;
    int frm_buf_size;
} TransposeRKMPPContext;

static int transpose_rkmpp_build_filter_params(AVFilterContext *avctx)
{
    TransposeRKMPPContext *ctx = avctx->priv;
    av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin dir=%d.\n", __FUNCTION__, __LINE__, ctx->dir);


    switch (ctx->dir) {
    case TRANSPOSE_CCLOCK_FLIP:
        ctx->rotation_state = 0;
        break;
    case TRANSPOSE_CLOCK:
        ctx->rotation_state = 0;
        break;
    case TRANSPOSE_CCLOCK:
        ctx->rotation_state = 0;
        break;
    case TRANSPOSE_CLOCK_FLIP:
        ctx->rotation_state = 0;
        break;
    case TRANSPOSE_REVERSAL:
        ctx->rotation_state = 0;
        break;
    case TRANSPOSE_HFLIP:
        ctx->rotation_state = 0;
        break;
    case TRANSPOSE_VFLIP:
        ctx->rotation_state = 0;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Failed to set direction to %d\n", ctx->dir);
        return AVERROR(EINVAL);
    }


    return 0;
}

static int transpose_rkmpp_format_is_supported(enum AVPixelFormat fmt)
{
    int i;

    av_log(NULL, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] Checking fmt=%d.\n", __FUNCTION__, __LINE__, fmt);
    for (i = 0; i < FF_ARRAY_ELEMS(transpose_rkmpp_supported_formats); i++)
        if (transpose_rkmpp_supported_formats[i] == fmt)
            return 1;
    return 0;
}

static RgaSURF_FORMAT transpose_rkmpp_avpix_format_to_rga_format(int av_fmt) {
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

static uint32_t transpose_rkmpp_mppFormat_to_drmFormat(MppFrameFormat mppformat)
{
    switch (mppformat) {
    case MPP_FMT_YUV420SP:          return DRM_FORMAT_NV12;
#ifdef DRM_FORMAT_NV12_10
    case MPP_FMT_YUV420SP_10BIT:    return DRM_FORMAT_NV12_10 | DRM_FORMAT_BIG_ENDIAN;
#endif
    default:                        return 0;
    }
}

static void transpose_rkmpp_release_frame(void *opaque, uint8_t *data)
{
    AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)data;
    AVBufferRef *framecontextref = (AVBufferRef *)opaque;
    RKMPPFrameContext *framecontext = (RKMPPFrameContext *)framecontextref->data;

    //av_log(NULL, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] rkmppscale release mppframe(%p).\n", __FUNCTION__, __LINE__, (void *)(framecontext->frame));
    mpp_frame_deinit(&framecontext->frame);
    av_buffer_unref(&framecontext->decoder_ref);
    av_buffer_unref(&framecontextref);

    av_free(desc);
}


static void transpose_rkmpp_setinfo_avframe(AVFrame *outframe, AVFrame *inframe) {

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

static int transpose_rkmpp_transpose(AVFilterContext *avctx, AVFrame *out, AVFrame *in)
{
    //av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin!\n", __FUNCTION__, __LINE__);
    TransposeRKMPPContext *ctx = (TransposeRKMPPContext *)avctx->priv;
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
    //src_info.mmuFlag = 1;
    if (ctx->dir == TRANSPOSE_CCLOCK)
        src_info.rotation = HAL_TRANSFORM_ROT_180;
    else if (ctx->dir == TRANSPOSE_CLOCK_FLIP)
        src_info.rotation = HAL_TRANSFORM_ROT_270;
    else
        src_info.rotation = HAL_TRANSFORM_ROT_90; //1 or default
    rga_set_rect(&src_info.rect, 0, 0, ctx->src_w, ctx->src_h,
                 srcHor_stride, srcVer_stride, transpose_rkmpp_avpix_format_to_rga_format(in->format));

    destHor_stride = MPP_ALIGN(ctx->output_width, 16);
    destVer_stride = MPP_ALIGN(ctx->output_height, 16);
    destSize = destHor_stride * destVer_stride * 3 / 2;
    if (ctx->frm_buf == NULL || destSize != ctx->frm_buf_size) {
        if (ctx->frm_buf) {
            mpp_buffer_put(ctx->frm_buf);
            ctx->frm_buf = NULL;
        }
        ctx->frm_buf_size = destSize;
        ret = mpp_buffer_get(ctx->buf_grp, &ctx->frm_buf, ctx->frm_buf_size);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "failed to get buffer for scale MppBuffer ret %d\n", ret);
            ret = AVERROR_UNKNOWN;
            goto transpose_exit;
        }
    }
    destBuffer = ctx->frm_buf;
    //mpp_buffer_get(NULL, &destBuffer, destSize);
    dst_info.fd = mpp_buffer_get_fd(destBuffer);
    //dst_info.mmuFlag = 1;
    rga_set_rect(&dst_info.rect, 0, 0, ctx->output_width, ctx->output_height,
                 destHor_stride, destVer_stride, transpose_rkmpp_avpix_format_to_rga_format(out->format));

    if ((ret = c_RkRgaBlit(&src_info, &dst_info, NULL)) < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to do rga blit\n");
        goto transpose_exit;
    }

    ret = mpp_frame_init(&destMppframe);
    if (ret) {
        av_log(NULL, AV_LOG_ERROR, "mpp_frame_init failed (%d)\n", ret);
        goto transpose_exit;
    }

    mpp_frame_set_width(destMppframe, ctx->output_width);
    mpp_frame_set_height(destMppframe, ctx->output_height);
    mpp_frame_set_hor_stride(destMppframe, destHor_stride);
    mpp_frame_set_ver_stride(destMppframe, destVer_stride);
    mpp_frame_set_fmt(destMppframe, MPP_FMT_YUV420SP);
    mpp_frame_set_pts(destMppframe, in->pts);
    mpp_frame_set_buffer(destMppframe, destBuffer);

    out->crop_left        = 0;
    out->crop_right       = ctx->output_width - mpp_frame_get_width(destMppframe);
    out->crop_top         = 0;
    out->crop_bottom      = ctx->output_height - mpp_frame_get_height(destMppframe);
    out->width = ctx->output_width;
    out->height = ctx->output_height;
    

    desc = av_mallocz(sizeof(AVDRMFrameDescriptor));
    if (!desc) {
        ret = AVERROR(ENOMEM);
        goto transpose_exit;
    }

    desc->nb_objects = 1;
    desc->objects[0].fd = mpp_buffer_get_fd(destBuffer);
    desc->objects[0].size = mpp_buffer_get_size(destBuffer);
    desc->objects[0].ptr = mpp_buffer_get_ptr(destBuffer);

    desc->nb_layers = 1;
    layer = &desc->layers[0];
    layer->format = transpose_rkmpp_mppFormat_to_drmFormat(mpp_frame_get_fmt(srcMppframe));
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
        goto transpose_exit;
    }

    // MPP decoder needs to be closed only when all frames have been released.
    destFramecontext = (RKMPPFrameContext *)destFramecontextref->data;
    destFramecontext->decoder_ref = av_buffer_ref(framecontext->decoder_ref);
    destFramecontext->frame = destMppframe;
    //av_log(ctx, ZSPACE_SCALE_DEBUG_LEVEL, "[zspace] [%s:%d] rkmppscale mppframe(%p), frame(%p).\n", __FUNCTION__, __LINE__, (void *)destMppframe, (void *)out);

    out->data[0]  = (uint8_t *)desc;
    out->buf[0]   = av_buffer_create((uint8_t *)desc, sizeof(*desc), transpose_rkmpp_release_frame,
                                       destFramecontextref, AV_BUFFER_FLAG_READONLY);

    if (!out->buf[0]) {
        ret = AVERROR(ENOMEM);
        goto transpose_exit;
    }

    out->hw_frames_ctx = av_buffer_ref(in->hw_frames_ctx);
    if (!out->hw_frames_ctx) {
        ret = AVERROR(ENOMEM);
        goto transpose_exit;
    }

    return 0;
transpose_exit:

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


static int transpose_rkmpp_filter_frame(AVFilterLink *inlink, AVFrame *input_frame)
{
    AVFilterContext *avctx     = inlink->dst;
    AVFilterLink *outlink      = avctx->outputs[0];
    TransposeRKMPPContext *ctx = avctx->priv;
    AVFrame *output_frame      = NULL;
    int err = 0;
    //av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin.\n", __FUNCTION__, __LINE__);

    if (ctx->passthrough)
        return ff_filter_frame(outlink, input_frame);

    /*av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "Filter input: %s, %ux%u (%"PRId64").\n",
           av_get_pix_fmt_name(input_frame->format),
           input_frame->width, input_frame->height, input_frame->pts);*/


    output_frame = av_frame_alloc();
    if (!output_frame) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    output_frame->format = ctx->output_format;
    transpose_rkmpp_setinfo_avframe(output_frame, input_frame);

    err = transpose_rkmpp_transpose(avctx, output_frame, input_frame);
    if (err < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to transpose %d\n", err);
        goto fail;

    err = av_frame_copy_props(output_frame, input_frame);
    if (err < 0)
        goto fail;
    }

    av_frame_free(&input_frame);
    /*av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "Filter output: %s, %ux%u (%"PRId64").\n",
           av_get_pix_fmt_name(output_frame->format),
           output_frame->width, output_frame->height, output_frame->pts);*/

    return ff_filter_frame(outlink, output_frame);

fail:
    av_frame_free(&input_frame);
    av_frame_free(&output_frame);
    return err;
}

static av_cold int transpose_rkmpp_init(AVFilterContext *avctx)
{
    av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin.\n", __FUNCTION__, __LINE__);
    TransposeRKMPPContext *ctx = avctx->priv;
    int ret = 0;

    ctx->frame = av_frame_alloc();
    if (!ctx->frame)
        return AVERROR(ENOMEM);

    ret = mpp_buffer_group_get_internal(&ctx->buf_grp, MPP_BUFFER_TYPE_DRM);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "failed to get mpp buffer group ret %d\n", ret);
        ret = AVERROR_UNKNOWN;
        goto init_fail;
    }
    ctx->frm_buf = NULL;
    ctx->frm_buf_size = 0;

init_fail:
    return ret;
}

static av_cold void transpose_rkmpp_uninit(AVFilterContext *avctx)
{
    av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin.\n", __FUNCTION__, __LINE__);
    TransposeRKMPPContext *ctx = avctx->priv;

    if (ctx->frm_buf) {
        mpp_buffer_put(ctx->frm_buf);
        ctx->frm_buf = NULL;
    }

    if (ctx->buf_grp) {
        mpp_buffer_group_put(ctx->buf_grp);
        ctx->buf_grp = NULL;
    }

    av_frame_free(&ctx->frame);
    av_buffer_unref(&ctx->frames_ctx);
    return ;
}

static int transpose_rkmpp_config_output(AVFilterLink *outlink)
{
    AVFilterContext *avctx     = outlink->src;
    TransposeRKMPPContext *ctx = avctx->priv;
    AVFilterLink *inlink       = avctx->inputs[0];
    enum AVPixelFormat in_format = AV_PIX_FMT_NONE;
    enum AVPixelFormat out_format = AV_PIX_FMT_NONE;

    /* check that we have a hw context */
    if (!avctx->inputs[0]->hw_frames_ctx) {
        av_log(avctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    AVHWFramesContext     *frames_ctx = (AVHWFramesContext*)inlink->hw_frames_ctx->data;
    AVDRMDeviceContext *device_hwctx = frames_ctx->device_ctx->hwctx;

    if ((inlink->w >= inlink->h && ctx->passthrough == TRANSPOSE_PT_TYPE_LANDSCAPE) ||
        (inlink->w <= inlink->h && ctx->passthrough == TRANSPOSE_PT_TYPE_PORTRAIT)) {
        outlink->hw_frames_ctx = av_buffer_ref(inlink->hw_frames_ctx);
        if (!outlink->hw_frames_ctx)
            return AVERROR(ENOMEM);
        av_log(avctx, AV_LOG_VERBOSE,
               "w:%d h:%d -> w:%d h:%d (passthrough mode)\n",
               inlink->w, inlink->h, inlink->w, inlink->h);
        return 0;
    }

    ctx->passthrough = TRANSPOSE_PT_TYPE_NONE;
    ctx->hwctx = device_hwctx;

    switch (ctx->dir) {
    case TRANSPOSE_CCLOCK_FLIP:
    case TRANSPOSE_CCLOCK:
    case TRANSPOSE_CLOCK:
    case TRANSPOSE_CLOCK_FLIP:
        ctx->output_width  = ctx->src_h = avctx->inputs[0]->h;
        ctx->output_height = ctx->src_w = avctx->inputs[0]->w;
        av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "swap width and height [%d,%d] for clock/cclock rotation\n", ctx->output_width, ctx->output_height);
        break;
    default:
        break;
    }

    in_format     = frames_ctx->format;
    out_format    = AV_PIX_FMT_DRM_PRIME;
    av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] in_format=%d,out_format=%d.\n", __FUNCTION__, __LINE__, in_format, out_format);
    if (!transpose_rkmpp_format_is_supported(in_format)) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported input format: %s\n",
               av_get_pix_fmt_name(in_format));
        return AVERROR(ENOSYS);
    }
    if (!transpose_rkmpp_format_is_supported(out_format)) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported output format: %s\n",
               av_get_pix_fmt_name(out_format));
        return AVERROR(ENOSYS);
    }

    ctx->input_format = in_format;
    ctx->output_format = out_format;

    ctx->src_w = inlink->w;
    ctx->src_h = inlink->h;

    ctx->output_width = MPP_ALIGN(ctx->output_width, 16);
    ctx->output_height = MPP_ALIGN(ctx->output_height, 16);
    outlink->w = ctx->output_width;
    outlink->h = ctx->output_height;

    av_log(avctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] End src[%d, %d] out[%d, %d] dir=%d.\n", __FUNCTION__, __LINE__,
        ctx->src_w, ctx->src_h, ctx->output_width, ctx->output_height, ctx->dir);
    return 0;
}

static AVFrame *get_video_buffer(AVFilterLink *inlink, int w, int h)
{
    TransposeRKMPPContext *ctx = inlink->dst->priv;
    av_log(inlink->src, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] Begin.\n", __FUNCTION__, __LINE__);

    return ctx->passthrough ?
        ff_null_get_video_buffer(inlink, w, h) :
        ff_default_get_video_buffer(inlink, w, h);
}


static int transpose_rkmpp_query_formats(AVFilterContext *ctx)
{
    int ret = 0;
    static const enum AVPixelFormat pixel_formats[] = {
        AV_PIX_FMT_DRM_PRIME, AV_PIX_FMT_DRM_PRIME_P010BE, AV_PIX_FMT_NONE,
    };

    AVFilterFormats *pix_fmts = ff_make_format_list(pixel_formats);
    if (!pix_fmts)
        return AVERROR(ENOMEM);

    ret = ff_set_common_formats(ctx, pix_fmts);
    av_log(ctx, ZSPACE_TRANSPOSE_DEBUG_LEVEL, "[zspace] [%s:%d] ret = %d.\n", __FUNCTION__, __LINE__, ret);
    return ret;
}

#define OFFSET(x) offsetof(TransposeRKMPPContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)
static const AVOption transpose_rkmpp_options[] = {
    { "dir", "set transpose direction", OFFSET(dir), AV_OPT_TYPE_INT, { .i64 = TRANSPOSE_CCLOCK_FLIP }, 0, 6, FLAGS, "dir" },
        { "cclock_flip",   "rotate counter-clockwise with vertical flip", 0, AV_OPT_TYPE_CONST, { .i64 = TRANSPOSE_CCLOCK_FLIP }, .flags=FLAGS, .unit = "dir" },
        { "clock",         "rotate clockwise",                            0, AV_OPT_TYPE_CONST, { .i64 = TRANSPOSE_CLOCK       }, .flags=FLAGS, .unit = "dir" },
        { "cclock",        "rotate counter-clockwise",                    0, AV_OPT_TYPE_CONST, { .i64 = TRANSPOSE_CCLOCK      }, .flags=FLAGS, .unit = "dir" },
        { "clock_flip",    "rotate clockwise with vertical flip",         0, AV_OPT_TYPE_CONST, { .i64 = TRANSPOSE_CLOCK_FLIP  }, .flags=FLAGS, .unit = "dir" },
        { "reversal",      "rotate by half-turn",                         0, AV_OPT_TYPE_CONST, { .i64 = TRANSPOSE_REVERSAL    }, .flags=FLAGS, .unit = "dir" },
        { "hflip",         "flip horizontally",                           0, AV_OPT_TYPE_CONST, { .i64 = TRANSPOSE_HFLIP       }, .flags=FLAGS, .unit = "dir" },
        { "vflip",         "flip vertically",                             0, AV_OPT_TYPE_CONST, { .i64 = TRANSPOSE_VFLIP       }, .flags=FLAGS, .unit = "dir" },

    { "passthrough", "do not apply transposition if the input matches the specified geometry",
      OFFSET(passthrough), AV_OPT_TYPE_INT, {.i64=TRANSPOSE_PT_TYPE_NONE},  0, INT_MAX, FLAGS, "passthrough" },
        { "none",      "always apply transposition",   0, AV_OPT_TYPE_CONST, {.i64=TRANSPOSE_PT_TYPE_NONE},      INT_MIN, INT_MAX, FLAGS, "passthrough" },
        { "portrait",  "preserve portrait geometry",   0, AV_OPT_TYPE_CONST, {.i64=TRANSPOSE_PT_TYPE_PORTRAIT},  INT_MIN, INT_MAX, FLAGS, "passthrough" },
        { "landscape", "preserve landscape geometry",  0, AV_OPT_TYPE_CONST, {.i64=TRANSPOSE_PT_TYPE_LANDSCAPE}, INT_MIN, INT_MAX, FLAGS, "passthrough" },

    { NULL }
};


AVFILTER_DEFINE_CLASS(transpose_rkmpp);

static const AVFilterPad transpose_rkmpp_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = &transpose_rkmpp_filter_frame,
        .get_video_buffer = get_video_buffer,
        //.config_props = &ff_rkmpp_vpp_config_input,
    },
    { NULL }
};

static const AVFilterPad transpose_rkmpp_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .config_props = &transpose_rkmpp_config_output,
    },
    { NULL }
};

AVFilter ff_vf_transpose_rkmpp = {
    .name           = "transpose_rkmpp",
    .description    = NULL_IF_CONFIG_SMALL("RKMPP RGA for transpose"),
    .priv_size      = sizeof(TransposeRKMPPContext),
    .init           = &transpose_rkmpp_init,
    .uninit         = &transpose_rkmpp_uninit,
    .query_formats  = &transpose_rkmpp_query_formats,
    .inputs         = transpose_rkmpp_inputs,
    .outputs        = transpose_rkmpp_outputs,
    .priv_class     = &transpose_rkmpp_class,
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
};
