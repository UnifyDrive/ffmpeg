/*
 * HW decode acceleration through rkmpp
 *
 * Copyright (c) 2021 Anton ZSpace
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

#ifndef AVCODEC_RKMPPDEC_H
#define AVCODEC_RKMPPDEC_H
#include <drm_fourcc.h>
#include <rga/RgaApi.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/rk_mpi.h>

#include "libavutil/buffer.h"
#include "libavutil/common.h"
#include "libavutil/frame.h"


typedef struct {
    MppCtx ctx;
    MppApi *mpi;
    MppBufferGroup frame_group;

    char first_packet;
    char eos_reached;

    AVBufferRef *frames_ref;
    AVBufferRef *device_ref;
} RKMPPDecoder;

typedef struct {
    AVClass *av_class;
    AVBufferRef *decoder_ref;
} RKMPPDecodeContext;

typedef struct {
    MppFrame frame;
    AVBufferRef *decoder_ref;
} RKMPPFrameContext;



#endif /* AVCODEC_RKMPPDEC_H */
