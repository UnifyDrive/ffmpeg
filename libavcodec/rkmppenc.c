/*
 * RockChip MPP Video Encoder
 * Copyright (c) 2017 Lionel CHAZALLON
 *
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

#include <drm_fourcc.h>
#include <pthread.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/rk_mpi.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>

#include "avcodec.h"
#include "decode.h"
#include "hwconfig.h"
#include "internal.h"
#include "libavutil/buffer.h"
#include "libavutil/common.h"
#include "libavutil/frame.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "libavutil/pixfmt.h"
#include "libavutil/time.h"
#include "libavutil/buffer_internal.h"


#define RECEIVE_PACKET_TIMEOUT   100
#define ZSPACE_ENCODER_DEBUG 1

#if ZSPACE_ENCODER_DEBUG
#define ZSPACE_ENCODER_DEBUG_LEVEL AV_LOG_WARNING
#else
#define ZSPACE_ENCODER_DEBUG_LEVEL AV_LOG_INFO
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

static const uint8_t rkmpp_start_code[] = { 0, 0, 0, 1 };

typedef struct {
    // global flow control flag
    RK_U32 frm_eos;
    RK_U32 pkt_eos;
    RK_U32 frm_pkt_cnt;
    RK_S32 frame_count;
    RK_U64 stream_size;

    // base flow context
    MppCtx ctx;
    MppApi *mpi;
    MppEncCfg cfg;
    MppEncPrepCfg prep_cfg;
    MppEncRcCfg rc_cfg;
    MppEncCodecCfg codec_cfg;
    MppEncSliceSplit split_cfg;
    MppEncOSDPltCfg osd_plt_cfg;
    MppEncOSDPlt    osd_plt;
    MppEncOSDData   osd_data;
    MppEncROIRegion roi_region[3];
    MppEncROICfg    roi_cfg;

    //MppBufferGroup frame_group;
    // input / output
    MppBufferGroup buf_grp;
    MppBuffer frm_buf;
    MppBuffer pkt_buf;
    MppEncSeiMode sei_mode;
    MppEncHeaderMode header_mode;

    // paramter for resource malloc
    RK_U32 width;
    RK_U32 height;
    RK_U32 hor_stride;
    RK_U32 ver_stride;
    MppFrameFormat fmt;
    MppCodingType type;
    RK_S32 num_frames;
    RK_S32 loop_times;

    // resources
    size_t header_size;
    size_t frame_size;
    /* NOTE: packet buffer may overflow */
    size_t packet_size;

    RK_U32 osd_enable;
    RK_U32 osd_mode;
    RK_U32 split_mode;
    RK_U32 split_arg;

    RK_U32 user_data_enable;
    RK_U32 roi_enable;

    // rate control runtime parameter
    RK_S32 fps_in_flex;
    RK_S32 fps_in_den;
    RK_S32 fps_in_num;
    RK_S32 fps_out_flex;
    RK_S32 fps_out_den;
    RK_S32 fps_out_num;
    RK_S32 bps;
    RK_S32 bps_max;
    RK_S32 bps_min;
    RK_S32 rc_mode;
    RK_S32 gop_mode;
    RK_S32 gop_len;
    RK_S32 vi_len;

    RK_S32 profile;
    RK_S32 level;

    char first_packet;
    char eos_reached;
    AVBufferRef *frames_ref;
    AVBufferRef *device_ref;
    int64_t timeNow;
    int64_t timeLast;
} RKMPPEncoder;

typedef struct {
    AVClass *av_class;
    AVBufferRef *encoder_ref;

    int64_t profile;
    int64_t level;
    int64_t entropy;
    int a53_cc;
    int64_t realtime;
    int64_t frames_before;
    int64_t frames_after;

    int64_t allow_sw;
    int64_t require_sw;
} RKMPPEncodeContext;

typedef struct {
    MppFrame frame;
    AVBufferRef *decoder_ref;
} RKMPPFrameContext;


static MppCodingType rkmpp_get_codingtype(AVCodecContext *avctx)
{
    switch (avctx->codec_id) {
    case AV_CODEC_ID_H264:          return MPP_VIDEO_CodingAVC;
    case AV_CODEC_ID_HEVC:          return MPP_VIDEO_CodingHEVC;
    default:                        return MPP_VIDEO_CodingUnused;
    }
}

static MppFrameFormat rkmpp_get_frameformat(enum AVPixelFormat format)
{
    switch (format) {
        case AV_PIX_FMT_YUV420P:            return MPP_FMT_YUV420P;
        case AV_PIX_FMT_NV12:               return MPP_FMT_YUV420SP;
        case AV_PIX_FMT_DRM_PRIME:          return MPP_FMT_YUV420SP;
#ifdef DRM_FORMAT_NV12_10
        //case AV_PIX_FMT_DRM_PRIME:    return MPP_FMT_YUV420SP_10BIT;//DRM_FORMAT_NV12_10;
#endif
        default:                        return MPP_FMT_BUTT;
    }
}

static int rkmpp_close_encoder(AVCodecContext *avctx)
{
    RKMPPEncodeContext *rk_context = avctx->priv_data;
    av_log(NULL, AV_LOG_ERROR, "[zspace] [%s:%d] Begin unref encoder_ref.\n", __FUNCTION__, __LINE__);
    av_buffer_unref(&rk_context->encoder_ref);
    rk_context->encoder_ref = NULL;
    av_log(NULL, AV_LOG_ERROR, "[zspace] [%s:%d] End unref encoder_ref and set to NULL.\n", __FUNCTION__, __LINE__);
    return 0;
}

static void rkmpp_release_encoder(void *opaque, uint8_t *data)
{
    RKMPPEncoder *encoder = (RKMPPEncoder *)data;

    if (!encoder) {
        return;
    }

    if (encoder->mpi) {
        encoder->mpi->reset(encoder->ctx);
        mpp_destroy(encoder->ctx);
        encoder->ctx = NULL;
    }

    if (encoder->cfg) {
        mpp_enc_cfg_deinit(encoder->cfg);
        encoder->cfg = NULL;
    }

    if (encoder->frm_buf) {
        mpp_buffer_put(encoder->frm_buf);
        encoder->frm_buf = NULL;
    }

    if (encoder->pkt_buf) {
        mpp_buffer_put(encoder->pkt_buf);
        encoder->pkt_buf = NULL;
    }

    if (encoder->osd_data.buf) {
        mpp_buffer_put(encoder->osd_data.buf);
        encoder->osd_data.buf = NULL;
    }

    if (encoder->buf_grp) {
        mpp_buffer_group_put(encoder->buf_grp);
        encoder->buf_grp = NULL;
    }

    av_buffer_unref(&encoder->frames_ref);
    av_buffer_unref(&encoder->device_ref);

    av_free(encoder);
}

static int rkmpp_get_encode_parameters(
    AVCodecContext *avctx,
    RKMPPEncodeContext *rk_context,
    RKMPPEncoder *encoder)
{
    int ret = 0;

    encoder->width = avctx->width;
    encoder->height = avctx->height;
    encoder->hor_stride = (MPP_ALIGN(avctx->width, 16));
    encoder->ver_stride = (MPP_ALIGN(avctx->height, 16));
    encoder->fmt = rkmpp_get_frameformat(avctx->pix_fmt);

    encoder->rc_mode = (RK_S32)(avctx->bit_rate_tolerance != avctx->bit_rate?MPP_ENC_RC_MODE_VBR:MPP_ENC_RC_MODE_CBR);
    encoder->num_frames = 1;
    if (encoder->type == MPP_VIDEO_CodingMJPEG && encoder->num_frames == 0) {
        encoder->num_frames = 1;
    }
    encoder->bps = (RK_S32) avctx->bit_rate;
    encoder->bps_max = (RK_S32) avctx->rc_max_rate;
    encoder->bps_min = (RK_S32) avctx->rc_min_rate;

    encoder->fps_in_flex = 0;
    encoder->fps_in_den   = 1;
    encoder->fps_in_num   = avctx->framerate.den>0?(avctx->framerate.num/avctx->framerate.den):25;
    encoder->fps_out_flex = 0;
    encoder->fps_out_den  = 1;
    encoder->fps_out_num  = avctx->framerate.den>0?(avctx->framerate.num/avctx->framerate.den):25;

    encoder->gop_mode = 0;//???
    encoder->gop_len = avctx->gop_size>encoder->fps_out_num?avctx->gop_size:0;
    encoder->vi_len = 1;//???

    encoder->profile = (RK_S32)rk_context->profile;
    if(encoder->profile != 66 && encoder->profile != 77 && encoder->profile != 100) {
        encoder->profile = 100;
    }

    encoder->level = (RK_S32)rk_context->level;
    if(encoder->level < 10 || encoder->level > 52) {
        encoder->level = 40;
    }

    // update resource parameter
    switch (encoder->fmt & MPP_FRAME_FMT_MASK) {
        case MPP_FMT_YUV420SP:
        case MPP_FMT_YUV420P: {
            encoder->frame_size = MPP_ALIGN(encoder->hor_stride, 64) * MPP_ALIGN(encoder->ver_stride, 64) * 3 / 2;
        } break;

        case MPP_FMT_YUV422_YUYV :
        case MPP_FMT_YUV422_YVYU :
        case MPP_FMT_YUV422_UYVY :
        case MPP_FMT_YUV422_VYUY :
        case MPP_FMT_YUV422P :
        case MPP_FMT_YUV422SP :
        case MPP_FMT_RGB444 :
        case MPP_FMT_BGR444 :
        case MPP_FMT_RGB555 :
        case MPP_FMT_BGR555 :
        case MPP_FMT_RGB565 :
        case MPP_FMT_BGR565 : {
            encoder->frame_size = MPP_ALIGN(encoder->hor_stride, 64) * MPP_ALIGN(encoder->ver_stride, 64) * 2;
        } break;

        default: {
            encoder->frame_size = MPP_ALIGN(encoder->hor_stride, 64) * MPP_ALIGN(encoder->ver_stride, 64) * 4;
        } break;
    }
    if (MPP_FRAME_FMT_IS_FBC(encoder->fmt))
        encoder->header_size = MPP_ALIGN(MPP_ALIGN(encoder->width, 16) * MPP_ALIGN(encoder->height, 16) / 16, SZ_4K);
    else
        encoder->header_size = 0;

    av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] timebase.num=%d,timebase.den=%d\n", avctx->time_base.num, avctx->time_base.den);
    av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] avctx->framerate.num=%d,avctx->framerate.den=%d\n", avctx->framerate.num, avctx->framerate.den);
    av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] encoder->fmt=%x,avctx->pix_fmt=%d, AV_PIX_FMT_DRM_PRIME=%d, AV_PIX_FMT_YUV420P=%d,AV_PIX_FMT_NV12=%d\n", encoder->fmt, avctx->pix_fmt, AV_PIX_FMT_DRM_PRIME,
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_NV12);
    av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace]Get parameters encoder->width=%d,encoder->height=%d, encoder->hor_stride=%d, encoder->ver_stride=%d, encoder->rc_mode=%d, encoder->bps=%d, encoder->gop_len=%d, encoder->profile=%d, encoder->level=%d, encoder->fps_in_num=%d\n", 
        encoder->width, encoder->height, encoder->hor_stride, encoder->ver_stride, encoder->rc_mode, encoder->bps, encoder->gop_len, encoder->profile, encoder->level, encoder->fps_in_num);

    return ret;
}

static int rkmpp_setup_encode_parameters(
    AVCodecContext *avctx,
    RKMPPEncoder *encoder)
{
    int ret = 0;
    MppApi *mpi;
    MppCtx ctx;
    MppEncCfg cfg;

    mpi = encoder->mpi;
    ctx = encoder->ctx;
    cfg = encoder->cfg;

    /* setup default parameter */
    if (encoder->fps_in_den == 0)
        encoder->fps_in_den = 1;
    if (encoder->fps_in_num == 0)
        encoder->fps_in_num = 30;
    if (encoder->fps_out_den == 0)
        encoder->fps_out_den = 1;
    if (encoder->fps_out_num == 0)
        encoder->fps_out_num = 30;
    if (!encoder->bps)
        encoder->bps = encoder->width * encoder->height / 8 * (encoder->fps_out_num / encoder->fps_out_den);

    mpp_enc_cfg_set_s32(cfg, "prep:width", encoder->width);
    mpp_enc_cfg_set_s32(cfg, "prep:height", encoder->height);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", encoder->hor_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", encoder->ver_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:format", encoder->fmt);
    mpp_enc_cfg_set_s32(cfg, "rc:mode", encoder->rc_mode);

    /* fix input / output frame rate */
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_flex", encoder->fps_in_flex);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", encoder->fps_in_num);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", encoder->fps_in_den);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_flex", encoder->fps_out_flex);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", encoder->fps_out_num);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm", encoder->fps_out_den);
    mpp_enc_cfg_set_s32(cfg, "rc:gop", encoder->gop_len ? encoder->gop_len : encoder->fps_out_num * 2);

    /* drop frame or not when bitrate overflow */
    mpp_enc_cfg_set_u32(cfg, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
    mpp_enc_cfg_set_u32(cfg, "rc:drop_thd", 20);        /* 20% of max bps */
    mpp_enc_cfg_set_u32(cfg, "rc:drop_gap", 1);         /* Do not continuous drop frame */

    /* setup bitrate for different rc_mode */
    mpp_enc_cfg_set_s32(cfg, "rc:bps_target", encoder->bps);
    switch (encoder->rc_mode) {
        case MPP_ENC_RC_MODE_FIXQP : {
            /* do not setup bitrate on FIXQP mode */
        } break;
        case MPP_ENC_RC_MODE_CBR : {
            /* CBR mode has narrow bound */
            mpp_enc_cfg_set_s32(cfg, "rc:bps_max", encoder->bps_max ? encoder->bps_max : encoder->bps * 17 / 16);
            mpp_enc_cfg_set_s32(cfg, "rc:bps_min", encoder->bps_min ? encoder->bps_min : encoder->bps * 15 / 16);
        } break;
        case MPP_ENC_RC_MODE_VBR :
        case MPP_ENC_RC_MODE_AVBR : {
            /* VBR mode has wide bound */
            mpp_enc_cfg_set_s32(cfg, "rc:bps_max", encoder->bps_max ? encoder->bps_max : encoder->bps * 17 / 16);
            mpp_enc_cfg_set_s32(cfg, "rc:bps_min", encoder->bps_min ? encoder->bps_min : encoder->bps * 1 / 16);
        } break;
        default : {
            /* default use CBR mode */
            mpp_enc_cfg_set_s32(cfg, "rc:bps_max", encoder->bps_max ? encoder->bps_max : encoder->bps * 17 / 16);
            mpp_enc_cfg_set_s32(cfg, "rc:bps_min", encoder->bps_min ? encoder->bps_min : encoder->bps * 15 / 16);
        } break;
    }

    /* setup qp for different codec and rc_mode */
    switch (encoder->type) {
        case MPP_VIDEO_CodingAVC :
        case MPP_VIDEO_CodingHEVC : {
            switch (encoder->rc_mode) {
                case MPP_ENC_RC_MODE_FIXQP : {
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 20);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 20);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 20);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 20);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 20);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);
                } break;
                case MPP_ENC_RC_MODE_CBR :
                case MPP_ENC_RC_MODE_VBR :
                case MPP_ENC_RC_MODE_AVBR : {
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 26);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 51);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 10);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 51);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 10);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);
                } break;
                default : {
                    av_log(avctx, AV_LOG_ERROR, "unsupport encoder rc mode %d\n", encoder->rc_mode);
                } break;
            }
        } break;
        case MPP_VIDEO_CodingVP8 : {
            /* vp8 only setup base qp range */
            mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 40);
            mpp_enc_cfg_set_s32(cfg, "rc:qp_max",  127);
            mpp_enc_cfg_set_s32(cfg, "rc:qp_min",  0);
            mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 127);
            mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 0);
            mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 6);
        } break;
        case MPP_VIDEO_CodingMJPEG : {
            /* jpeg use special codec config to control qtable */
            mpp_enc_cfg_set_s32(cfg, "jpeg:q_factor", 80);
            mpp_enc_cfg_set_s32(cfg, "jpeg:qf_max", 99);
            mpp_enc_cfg_set_s32(cfg, "jpeg:qf_min", 1);
        } break;
        default : {
        } break;
    }

    /* setup codec  */
    mpp_enc_cfg_set_s32(cfg, "codec:type", encoder->type);
    switch (encoder->type) {
        case MPP_VIDEO_CodingAVC : {
            /*
             * H.264 profile_idc parameter
             * 66  - Baseline profile
             * 77  - Main profile
             * 100 - High profile
             */
            mpp_enc_cfg_set_s32(cfg, "h264:profile", encoder->profile);
            /*
             * H.264 level_idc parameter
             * 10 / 11 / 12 / 13    - qcif@15fps / cif@7.5fps / cif@15fps / cif@30fps
             * 20 / 21 / 22         - cif@30fps / half-D1@@25fps / D1@12.5fps
             * 30 / 31 / 32         - D1@25fps / 720p@30fps / 720p@60fps
             * 40 / 41 / 42         - 1080p@25fps / 1080p@30fps / 1080p@60fps
             * 50 / 51 / 52         - 4K@30fps
             */
            mpp_enc_cfg_set_s32(cfg, "h264:level", encoder->level);
            mpp_enc_cfg_set_s32(cfg, "h264:cabac_en", 1);
            mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc", 0);
            if (encoder->profile == 100) {
                mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 1);
            }else {
                mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 0);
            }
        } break;
        case MPP_VIDEO_CodingHEVC : {
            
        } break;
        case MPP_VIDEO_CodingMJPEG :
        case MPP_VIDEO_CodingVP8 : {
        } break;
        default : {
            av_log(avctx, AV_LOG_ERROR, "unsupport encoder coding type %d\n", encoder->type);
        } break;
    }

    encoder->split_mode = 0;
    encoder->split_arg = 0;

    //mpp_env_get_u32("split_mode", &encoder->split_mode, (RK_U32)MPP_ENC_SPLIT_NONE);
    //mpp_env_get_u32("split_arg", &encoder->split_arg, (RK_U32)0);
    if (encoder->split_mode) {
        av_log(avctx, AV_LOG_ERROR, "%p split_mode %d split_arg %d\n", ctx, encoder->split_mode, encoder->split_arg);
        mpp_enc_cfg_set_s32(cfg, "split:mode", encoder->split_mode);
        mpp_enc_cfg_set_s32(cfg, "split:arg", encoder->split_arg);
    }

    ret = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "mpi control enc set cfg failed ret %d\n", ret);
        goto RET;
    }

    
    /* optional */
    encoder->sei_mode = MPP_ENC_SEI_MODE_ONE_FRAME;
    ret = mpi->control(ctx, MPP_ENC_SET_SEI_CFG, &encoder->sei_mode);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "mpi control enc set sei cfg failed ret %d\n", ret);
        goto RET;
    }

    if (encoder->type == MPP_VIDEO_CodingAVC || encoder->type == MPP_VIDEO_CodingHEVC) {
        encoder->header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
        ret = mpi->control(ctx, MPP_ENC_SET_HEADER_MODE, &encoder->header_mode);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "mpi control enc set header mode failed ret %d\n", ret);
            goto RET;
        }
    }

    //RK_U32 gop_mode = encoder->gop_mode;

    //mpp_env_get_u32("gop_mode", &gop_mode, gop_mode);
    /*
    if (gop_mode) {
        MppEncRefCfg ref;

        mpp_enc_ref_cfg_init(&ref);

        if (encoder->gop_mode < 4)
            mpi_enc_gen_ref_cfg(ref, gop_mode);
        else
            mpi_enc_gen_smart_gop_ref_cfg(ref, encoder->gop_len, encoder->vi_len);

        ret = mpi->control(ctx, MPP_ENC_SET_REF_CFG, ref);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "mpi control enc set ref cfg failed ret %d\n", ret);
            goto RET;
        }
        mpp_enc_ref_cfg_deinit(&ref);
    }*/

    /* setup test mode by env */
    //mpp_env_get_u32("osd_enable", &encoder->osd_enable, 0);
    //mpp_env_get_u32("osd_mode", &encoder->osd_mode, MPP_ENC_OSD_PLT_TYPE_DEFAULT);
    //mpp_env_get_u32("roi_enable", &encoder->roi_enable, 0);
    //mpp_env_get_u32("user_data_enable", &encoder->user_data_enable, 0);


RET:
    return ret;
}


static int rkmpp_get_encoded_extradata(
    AVCodecContext *avctx,
    RKMPPEncoder *encoder)
{
    int status = 0;
    int ret = 0;
    MppApi *mpi;
    MppCtx ctx;
    MppPacket packet = NULL;

    mpi = encoder->mpi;
    ctx = encoder->ctx;

    switch (encoder->type) {
        case MPP_VIDEO_CodingAVC:
        case MPP_VIDEO_CodingHEVC:
            mpp_packet_init_with_buffer(&packet, encoder->pkt_buf);
            /* NOTE: It is important to clear output packet length!! */
            mpp_packet_set_length(packet, 0);
            ret = mpi->control(ctx, MPP_ENC_GET_HDR_SYNC, packet);
            if (ret) {
                av_log(avctx, AV_LOG_ERROR, "mpi control enc get extra info failed\n");
                status = AVERROR_UNKNOWN;
                goto end_noextradata;
            } else {
                /* get and write sps/pps for H.264 */

                void *ptr   = mpp_packet_get_pos(packet);
                size_t len  = mpp_packet_get_length(packet);
                //memcpy
                if (!avctx->extradata && (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER)) {
                    avctx->extradata = av_mallocz(len + AV_INPUT_BUFFER_PADDING_SIZE);
                    if (!avctx->extradata) {
                        status = AVERROR(ENOMEM);
                        goto end_noextradata;
                    }
                    memcpy(avctx->extradata, (uint8_t *)ptr, len);
                    avctx->extradata_size = len;
                    av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] sps/pps/vps size:%d\n", avctx->extradata_size);
                    for(int i = 0; i < avctx->extradata_size; i++){
                        av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] avctx->extradata[%d] %02x\n", i, avctx->extradata[i]);
                    }
                }
            }
            mpp_packet_deinit(&packet);
            packet = NULL;
            break;
        default :
            break;
    }

end_noextradata:
    if (packet) {
        mpp_packet_deinit(&packet);
    }

    return status;
}

static int rkmpp_init_encoder(AVCodecContext *avctx)
{
    RKMPPEncodeContext *rk_context = avctx->priv_data;
    RKMPPEncoder *encoder = NULL;
    MppCodingType codectype = MPP_VIDEO_CodingUnused;
    MppPollType timeout = MPP_POLL_BLOCK;
    int ret = 0;

    // create a decoder and a ref to it
    encoder = av_mallocz(sizeof(RKMPPEncoder));
    if (!encoder) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    rk_context->encoder_ref = av_buffer_create((uint8_t *)encoder, sizeof(*encoder), rkmpp_release_encoder,
                                                   NULL, AV_BUFFER_FLAG_READONLY);
    if (!rk_context->encoder_ref) {
        av_log(avctx, AV_LOG_ERROR, "av_buffer_create() for encoder_ref failed!\n");
        av_free(encoder);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    av_log(avctx, AV_LOG_DEBUG, "Initializing RKMPP encoder.\n");
    codectype = rkmpp_get_codingtype(avctx);
    if (codectype == MPP_VIDEO_CodingUnused) {
        av_log(avctx, AV_LOG_ERROR, "Unknown codec type (%d).\n", avctx->codec_id);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }
    encoder->type = codectype;

    ret = mpp_check_support_format(MPP_CTX_ENC, codectype);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Codec type (%d) unsupported by MPP encoder!\n", avctx->codec_id);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = rkmpp_get_encode_parameters(avctx, rk_context, encoder);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set parameters for encoder(code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    
    ret = mpp_buffer_group_get_internal(&encoder->buf_grp, MPP_BUFFER_TYPE_DRM);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "failed to get mpp buffer group ret %d\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_buffer_get(encoder->buf_grp, &encoder->frm_buf, encoder->frame_size + encoder->header_size);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "failed to get buffer for input frame ret %d\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_buffer_get(encoder->buf_grp, &encoder->pkt_buf, encoder->frame_size);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "failed to get buffer for output packet ret %d\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // Create the MPP context
    ret = mpp_create(&encoder->ctx, &encoder->mpi);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to create MPP context for encoder(code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = encoder->mpi->control(encoder->ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
    if (MPP_OK != ret) {
        av_log(avctx, AV_LOG_ERROR, "mpi control set output timeout %d ret %d\n", timeout, ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // initialize mpp
    ret = mpp_init(encoder->ctx, MPP_CTX_ENC, codectype);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize MPP context for encoder(code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_enc_cfg_init(&encoder->cfg);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Fmpp_enc_cfg_init failed ret %d.\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = rkmpp_setup_encode_parameters(avctx, encoder);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "rkmpp_set_encode_parameters failed ret %d.\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = rkmpp_get_encoded_extradata(avctx, encoder);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "rkmpp_get_encoded_extradata failed ret %d.\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    av_log(avctx, AV_LOG_DEBUG, "RKMPP decoder initialized successfully.\n");

    encoder->device_ref = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
    if (!encoder->device_ref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ret = av_hwdevice_ctx_init(encoder->device_ref);
    if (ret < 0)
        goto fail;

    encoder->first_packet = 1;
    return 0;
fail:
    av_log(avctx, AV_LOG_ERROR, "Failed to initialize RKMPP encoder.\n");
    rkmpp_close_encoder(avctx);
    return ret;
}

static int read_image_data(RK_U8 *buf, const AVFrame  *inframe, RK_U32 width, RK_U32 height,
                   RK_U32 hor_stride, RK_U32 ver_stride, MppFrameFormat fmt)
{
    MPP_RET ret = MPP_OK;
    RK_U8 *buf_y = buf;
    RK_U8 *buf_u = buf_y + hor_stride * ver_stride; // NOTE: diff from gen_yuv_image
    RK_U8 *buf_v = buf_u + hor_stride * ver_stride / 4; // NOTE: diff from gen_yuv_image

    switch (fmt & MPP_FRAME_FMT_MASK) {
        case MPP_FMT_YUV420SP : {/* YYYY... UV... (NV12)     */
            memcpy(buf_y, inframe->data[0], hor_stride * ver_stride);
            memcpy(buf_u, inframe->data[1], hor_stride * ver_stride / 2);
        } break;
        case MPP_FMT_YUV420P : {/* YYYY... U...V...  (I420) */
            memcpy(buf_y, inframe->data[0], hor_stride * ver_stride);
            memcpy(buf_u, inframe->data[1], hor_stride * ver_stride / 4);
            memcpy(buf_v, inframe->data[2], hor_stride * ver_stride / 4);
            
        } break;
        case MPP_FMT_ARGB8888 :
        case MPP_FMT_ABGR8888 :
        case MPP_FMT_BGRA8888 :
        case MPP_FMT_RGBA8888 :
        case MPP_FMT_RGB101010 :
        case MPP_FMT_BGR101010 : {
            //ret = read_with_pixel_width(buf_y, width, height, hor_stride, 4, fp);
        } break;
        case MPP_FMT_YUV422P :
        case MPP_FMT_YUV422SP :
        case MPP_FMT_BGR444 :
        case MPP_FMT_RGB444 :
        case MPP_FMT_RGB555 :
        case MPP_FMT_BGR555 :
        case MPP_FMT_RGB565 :
        case MPP_FMT_BGR565 :
        case MPP_FMT_YUV422_YUYV :
        case MPP_FMT_YUV422_YVYU :
        case MPP_FMT_YUV422_UYVY :
        case MPP_FMT_YUV422_VYUY : {
            //ret = read_with_pixel_width(buf_y, width, height, hor_stride, 2, fp);
        } break;
        case MPP_FMT_RGB888 :
        case MPP_FMT_BGR888 : {
            //ret = read_with_pixel_width(buf_y, width, height, hor_stride, 3, fp);
        } break;
        default : {
            av_log(NULL, AV_LOG_ERROR, "read image do not support fmt %d\n", fmt);
            ret = MPP_ERR_VALUE;
        } break;
    }

    return ret;
}


static int rkmpp_send_frame(AVCodecContext *avctx,
                            RKMPPEncoder *encoder,
                            const AVFrame  *inframe,
                            MppPacket *packet)
{
    int status = 0;
    MPP_RET ret = MPP_OK;
    MppApi *mpi;
    MppCtx ctx;
    MppMeta meta = NULL;
    MppFrame rkmppframe = NULL;
    void *buf = NULL;
    AVBufferRef *framecontextref = NULL;
    RKMPPFrameContext *framecontext = NULL;

    //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] [%s:%d] begin .\n", __FUNCTION__, __LINE__);
    mpi = encoder->mpi;
    ctx = encoder->ctx;

    if (!inframe) {
        encoder->frm_eos = 1;
        encoder->eos_reached = 1;
        av_log(avctx, AV_LOG_ERROR, "[zspace] Meet NUll inframe, set frm_eof = 1\n");
    }else {
        encoder->timeNow = av_gettime_relative();
        if (avctx->pix_fmt == AV_PIX_FMT_DRM_PRIME && inframe) {
            framecontextref = (AVBufferRef *) av_buffer_get_opaque(inframe->buf[0]);
            framecontext = (RKMPPFrameContext *)framecontextref->data;
            rkmppframe = framecontext->frame;
            //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] [%s:%d] AV_PIX_FMT_DRM_PRIME(%p).\n", __FUNCTION__, __LINE__, (void *)rkmppframe);
        }else {
            buf = mpp_buffer_get_ptr(encoder->frm_buf);
            status = read_image_data(buf, inframe, encoder->width, encoder->height, encoder->hor_stride, encoder->ver_stride, encoder->fmt);
            if (status) {
                av_log(avctx, AV_LOG_ERROR, "read_image_data failed (%d)\n", status);
                status = AVERROR_UNKNOWN;
                return status;
            }
            //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] read_image_data costTime=%ld\n", av_gettime_relative()-encoder->timeNow);
            
        }
    }

    if (rkmppframe == NULL) {
        ret = mpp_frame_init(&rkmppframe);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "mpp_frame_init failed (%d)\n", ret);
            status = AVERROR_UNKNOWN;
            return status;
        }
    }

    if (framecontextref == NULL) {
        mpp_frame_set_width(rkmppframe, encoder->width);
        mpp_frame_set_height(rkmppframe, encoder->height);
        mpp_frame_set_hor_stride(rkmppframe, encoder->hor_stride);
        mpp_frame_set_ver_stride(rkmppframe, encoder->ver_stride);
        mpp_frame_set_fmt(rkmppframe, encoder->fmt);
        if (encoder->eos_reached) {
            av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] [%s:%d] eos_reached.\n", __FUNCTION__, __LINE__);
            mpp_frame_set_buffer(rkmppframe, NULL);
        } else {
            mpp_frame_set_buffer(rkmppframe, encoder->frm_buf);
        }
    }

    mpp_frame_set_eos(rkmppframe, encoder->frm_eos);
    if (inframe) {
        mpp_frame_set_pts(rkmppframe, inframe->pts);
        //mpp_frame_set_dts(rkmppframe, inframe->pkt_dts);
        //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] inframe->pts=%lld, pkt_dts=%lld\n", inframe->pts, inframe->pkt_dts);
    }

    meta = mpp_frame_get_meta(rkmppframe);
    if (*packet == NULL) {
        mpp_packet_init_with_buffer(packet, encoder->pkt_buf);
        /* NOTE: It is important to clear output packet length!! */
        mpp_packet_set_length(*packet, 0);
    }
    mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, *packet);

    /*
     * NOTE: in non-block mode the frame can be resent.
     * The default input timeout mode is block.
     *
     * User should release the input frame to meet the requirements of
     * resource creator must be the resource destroyer.
     */
    ret = mpi->encode_put_frame(ctx, rkmppframe);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "mpp encode put frame failed (%d)\n", ret);
        status = AVERROR_UNKNOWN;
    }

    if (framecontextref == NULL) {
        mpp_frame_deinit(&rkmppframe);
    }else {
        //av_buffer_unref(&framecontextref);
    }

    return status;
}

static int rkmpp_get_packet(
    AVCodecContext *avctx,
    RKMPPEncoder   *encoder,
    AVPacket       *pkt,
    MppPacket      *packet)
{
    int status = 0;
    MppApi *mpi;
    MppCtx ctx;
    RK_U32 eoi = 1;
    MPP_RET ret = MPP_OK;
    MppMeta meta = NULL;
    int64_t timeCost = 0;

    mpi = encoder->mpi;
    ctx = encoder->ctx;

    do {
        ret = mpi->encode_get_packet(ctx, packet);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "mpp encode get packet failed (%d)\n", ret);
            status = AVERROR_UNKNOWN;
            return status;
        }

        if (*packet) {
            // write packet to file here
            void *ptr   = mpp_packet_get_pos(*packet);
            size_t len  = mpp_packet_get_length(*packet);
            char log_buf[256];
            RK_S32 log_size = sizeof(log_buf) - 1;
            RK_S32 log_len = 0;

            encoder->pkt_eos = mpp_packet_get_eos(*packet);

            //memcpy
            status = ff_alloc_packet2(avctx, pkt, len, len);
            if (status < 0) {
                av_log(avctx, AV_LOG_ERROR, "mpp encode get packet failed (%d)\n", ret);
                return status;
            }
            if (encoder->pkt_eos) {
                memcpy(pkt->data, ((uint8_t *)ptr), len);
                pkt->size = len;
                pkt->pts = mpp_packet_get_pts(*packet);
                pkt->dts = mpp_packet_get_dts(*packet);
            }else {
                memcpy(pkt->data, ((uint8_t *)ptr), len);
                pkt->size = len;
                pkt->pts = mpp_packet_get_pts(*packet);
                pkt->dts = pkt->pts;//mpp_packet_get_dts(*packet);
            }
            encoder->timeNow = av_gettime_relative();
            timeCost = encoder->timeNow - encoder->timeLast;
            av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] get avpacket costTime=%ld, duration=%ld, size:%d, pts=%ld dts=%ld %02x %02x %02x %02x %02x\n", timeCost, pkt->duration, pkt->size, pkt->pts, pkt->dts,
                    pkt->data[0], pkt->data[1], pkt->data[2], pkt->data[3], pkt->data[4]);
            encoder->timeLast = encoder->timeNow;

            log_len += snprintf(log_buf + log_len, log_size - log_len,
                                "encoded frame %-4d", encoder->frame_count);

            /* for low delay partition encoding */
            if (mpp_packet_is_partition(*packet)) {
                eoi = mpp_packet_is_eoi(*packet);

                log_len += snprintf(log_buf + log_len, log_size - log_len,
                                    " pkt %d", encoder->frm_pkt_cnt);
                encoder->frm_pkt_cnt = (eoi) ? (0) : (encoder->frm_pkt_cnt + 1);
            }

            log_len += snprintf(log_buf + log_len, log_size - log_len,
                                " size %-7zu", len);

            if (mpp_packet_has_meta(*packet)) {
                RK_S32 temporal_id = 0;
                RK_S32 lt_idx = -1;
                RK_S32 avg_qp = -1;

                meta = mpp_packet_get_meta(*packet);
                if (MPP_OK == mpp_meta_get_s32(meta, KEY_TEMPORAL_ID, &temporal_id))
                    log_len += snprintf(log_buf + log_len, log_size - log_len,
                                        " tid %d", temporal_id);

                if (MPP_OK == mpp_meta_get_s32(meta, KEY_LONG_REF_IDX, &lt_idx))
                    log_len += snprintf(log_buf + log_len, log_size - log_len,
                                        " lt %d", lt_idx);

                if (MPP_OK == mpp_meta_get_s32(meta, KEY_ENC_AVERAGE_QP, &avg_qp))
                    log_len += snprintf(log_buf + log_len, log_size - log_len,
                                        " qp %d", avg_qp);
            }

            //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "%p %s\n", ctx, log_buf);


            encoder->stream_size += len;
            encoder->frame_count += eoi;

            if (encoder->frm_eos && encoder->pkt_eos) {
                av_log(avctx, AV_LOG_ERROR, "%p found last packet,encoded %d frames, total size:%lld\n", ctx, encoder->frame_count, encoder->stream_size);
            }
        }
    } while (!eoi);

    
    return status;
}

static int rkmpp_encode_frame(
    AVCodecContext *avctx,
    AVPacket       *pkt,
    const AVFrame  *frame,
    int            *got_packet)
{
    int status = 0;
    RKMPPEncodeContext *rk_context = avctx->priv_data;
    RKMPPEncoder *encoder = (RKMPPEncoder *)rk_context->encoder_ref->data;
    MppPacket packet = NULL;
    int64_t time_now = 0;

    //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] [%s:%d] begin .\n", __FUNCTION__, __LINE__);
    *got_packet = 0;
    if (encoder->first_packet) {
        //do some need work!!!
        encoder->first_packet = 0;
    }

    time_now = av_gettime_relative();
    status = rkmpp_send_frame(avctx, encoder, frame, &packet);
    if (status) {
        goto end_nopkt;
    }
    //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] rkmpp_send_frame costTime=%ld, skip_time=%ld\n", av_gettime_relative()-time_now, time_now-encoder->timeLast);

    time_now = av_gettime_relative();
    status = rkmpp_get_packet(avctx, encoder, pkt, &packet);
    if (status || encoder->pkt_eos) {
        goto end_nopkt;
    }
    //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] rkmpp_get_packet costTime=%ld\n", av_gettime_relative()-time_now);

    *got_packet = 1;
    //av_log(avctx, ZSPACE_ENCODER_DEBUG_LEVEL, "[zspace] [%s:%d] Out get one packet .\n", __FUNCTION__, __LINE__);
    return 0;

end_nopkt:
    if (packet) {
        mpp_packet_deinit(&packet);
    }
    av_packet_unref(pkt);
    return status;
}

typedef enum RKMPP_H264Profile {
    RKMPP_H264_PROF_AUTO,
    RKMPP_H264_PROF_BASELINE = 66,
    RKMPP_H264_PROF_MAIN = 77,
    RKMPP_H264_PROF_HIGH = 100,
    RKMPP_H264_PROF_EXTENDED,
    RKMPP_H264_PROF_COUNT
} RKMPP_H264Profile;

typedef enum RKMPPH264Entropy{
    RKMPP_ENTROPY_NOT_SET,
    RKMPP_CAVLC,
    RKMPP_CABAC
} RKMPPH264Entropy;


#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
#define OFFSET(x) offsetof(RKMPPEncodeContext, x)

#define COMMON_OPTIONS \
    { "allow_sw", "Allow software encoding", OFFSET(allow_sw), AV_OPT_TYPE_BOOL, \
        { .i64 = 0 }, 0, 1, VE }, \
    { "require_sw", "Require software encoding", OFFSET(require_sw), AV_OPT_TYPE_BOOL, \
        { .i64 = 0 }, 0, 1, VE }, \
    { "realtime", "Hint that encoding should happen in real-time if not faster (e.g. capturing from camera).", \
        OFFSET(realtime), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE }, \
    { "frames_before", "Other frames will come before the frames in this session. This helps smooth concatenation issues.", \
        OFFSET(frames_before), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE }, \
    { "frames_after", "Other frames will come after the frames in this session. This helps smooth concatenation issues.", \
        OFFSET(frames_after), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },

static const AVOption rkmpp_h264_options[] = {
    { "profile", "Profile", OFFSET(profile), AV_OPT_TYPE_INT, { .i64 = RKMPP_H264_PROF_AUTO }, RKMPP_H264_PROF_AUTO, RKMPP_H264_PROF_COUNT, VE, "profile" },
    { "baseline", "Baseline Profile", 0, AV_OPT_TYPE_CONST, { .i64 = RKMPP_H264_PROF_BASELINE }, INT_MIN, INT_MAX, VE, "profile" },
    { "main",     "Main Profile",     0, AV_OPT_TYPE_CONST, { .i64 = RKMPP_H264_PROF_MAIN     }, INT_MIN, INT_MAX, VE, "profile" },
    { "high",     "High Profile",     0, AV_OPT_TYPE_CONST, { .i64 = RKMPP_H264_PROF_HIGH     }, INT_MIN, INT_MAX, VE, "profile" },
    { "extended", "Extend Profile",   0, AV_OPT_TYPE_CONST, { .i64 = RKMPP_H264_PROF_EXTENDED }, INT_MIN, INT_MAX, VE, "profile" },

    { "level", "Level", OFFSET(level), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 52, VE, "level" },
    { "1.3", "Level 1.3, only available with Baseline Profile", 0, AV_OPT_TYPE_CONST, { .i64 = 13 }, INT_MIN, INT_MAX, VE, "level" },
    { "3.0", "Level 3.0", 0, AV_OPT_TYPE_CONST, { .i64 = 30 }, INT_MIN, INT_MAX, VE, "level" },
    { "3.1", "Level 3.1", 0, AV_OPT_TYPE_CONST, { .i64 = 31 }, INT_MIN, INT_MAX, VE, "level" },
    { "3.2", "Level 3.2", 0, AV_OPT_TYPE_CONST, { .i64 = 32 }, INT_MIN, INT_MAX, VE, "level" },
    { "4.0", "Level 4.0", 0, AV_OPT_TYPE_CONST, { .i64 = 40 }, INT_MIN, INT_MAX, VE, "level" },
    { "4.1", "Level 4.1", 0, AV_OPT_TYPE_CONST, { .i64 = 41 }, INT_MIN, INT_MAX, VE, "level" },
    { "4.2", "Level 4.2", 0, AV_OPT_TYPE_CONST, { .i64 = 42 }, INT_MIN, INT_MAX, VE, "level" },
    { "5.0", "Level 5.0", 0, AV_OPT_TYPE_CONST, { .i64 = 50 }, INT_MIN, INT_MAX, VE, "level" },
    { "5.1", "Level 5.1", 0, AV_OPT_TYPE_CONST, { .i64 = 51 }, INT_MIN, INT_MAX, VE, "level" },
    { "5.2", "Level 5.2", 0, AV_OPT_TYPE_CONST, { .i64 = 52 }, INT_MIN, INT_MAX, VE, "level" },

    { "coder", "Entropy coding", OFFSET(entropy), AV_OPT_TYPE_INT, { .i64 = RKMPP_ENTROPY_NOT_SET }, RKMPP_ENTROPY_NOT_SET, RKMPP_CABAC, VE, "coder" },
    { "cavlc", "CAVLC entropy coding", 0, AV_OPT_TYPE_CONST, { .i64 = RKMPP_CAVLC }, INT_MIN, INT_MAX, VE, "coder" },
    { "vlc",   "CAVLC entropy coding", 0, AV_OPT_TYPE_CONST, { .i64 = RKMPP_CAVLC }, INT_MIN, INT_MAX, VE, "coder" },
    { "cabac", "CABAC entropy coding", 0, AV_OPT_TYPE_CONST, { .i64 = RKMPP_CABAC }, INT_MIN, INT_MAX, VE, "coder" },
    { "ac",    "CABAC entropy coding", 0, AV_OPT_TYPE_CONST, { .i64 = RKMPP_CABAC }, INT_MIN, INT_MAX, VE, "coder" },

    { "a53cc", "Use A53 Closed Captions (if available)", OFFSET(a53_cc), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, VE },

    COMMON_OPTIONS
    { NULL },
};

    
static const enum AVPixelFormat rkmpp_enc_avc_pix_fmts[] = {
    AV_PIX_FMT_DRM_PRIME,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_NONE
};


#define RKMPP_ENC_CLASS(NAME) \
    static const AVClass rkmpp_##NAME##_enc_class = { \
        .class_name = "rkmpp_" #NAME "_enc", \
        .item_name  = av_default_item_name, \
        .option     = rkmpp_h264_options, \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define RKMPP_ENC(NAME, ID, BSFS) \
    RKMPP_ENC_CLASS(NAME) \
    AVCodec ff_##NAME##_rkmpp_encoder = { \
        .name           = #NAME "_rkmppenc", \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " (rkmppenc)"), \
        .type           = AVMEDIA_TYPE_VIDEO, \
        .id             = ID, \
        .priv_data_size = sizeof(RKMPPEncodeContext), \
        .init           = rkmpp_init_encoder, \
        .close          = rkmpp_close_encoder, \
        .encode2        = rkmpp_encode_frame, \
        .priv_class     = &rkmpp_##NAME##_enc_class, \
        .capabilities   = AV_CODEC_CAP_DELAY, \
        .pix_fmts       = rkmpp_enc_avc_pix_fmts, \
        .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE | FF_CODEC_CAP_INIT_CLEANUP, \
    };

RKMPP_ENC(h264,  AV_CODEC_ID_H264,          NULL)
RKMPP_ENC(hevc,  AV_CODEC_ID_HEVC,          NULL)
