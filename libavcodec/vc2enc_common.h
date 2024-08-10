/*
 * Copyright (C) 2016 Open Broadcast Systems Ltd.
 * Author        2016 Rostislav Pehlivanov <atomnuker@gmail.com>
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

#ifndef AVCODEC_VC2ENC_COMMON_H
#define AVCODEC_VC2ENC_COMMON_H

#include "avcodec.h"
#include "dirac.h"
#include "put_bits.h"

#include "vc2enc_dwt.h"
#include "diractab.h"
#include "vulkan.h"

/* The limited size resolution of each slice forces us to do this */
#define SSIZE_ROUND(b) (FFALIGN((b), s->size_scaler) + 4 + s->prefix_bytes)

/* Decides the cutoff point in # of slices to distribute the leftover bytes */
#define SLICE_REDIST_TOTAL 150

typedef struct VC2BaseVideoFormat {
    enum AVPixelFormat pix_fmt;
    AVRational time_base;
    int width, height, interlaced, level;
    const char *name;
} VC2BaseVideoFormat;

static const VC2BaseVideoFormat base_video_fmts[] = {
    { 0 }, /* Custom format, here just to make indexing equal to base_vf */
    { AV_PIX_FMT_YUV420P,   { 1001, 15000 },  176,  120, 0, 1,     "QSIF525" },
    { AV_PIX_FMT_YUV420P,   {    2,    25 },  176,  144, 0, 1,     "QCIF"    },
    { AV_PIX_FMT_YUV420P,   { 1001, 15000 },  352,  240, 0, 1,     "SIF525"  },
    { AV_PIX_FMT_YUV420P,   {    2,    25 },  352,  288, 0, 1,     "CIF"     },
    { AV_PIX_FMT_YUV420P,   { 1001, 15000 },  704,  480, 0, 1,     "4SIF525" },
    { AV_PIX_FMT_YUV420P,   {    2,    25 },  704,  576, 0, 1,     "4CIF"    },

    { AV_PIX_FMT_YUV422P10, { 1001, 30000 },  720,  480, 1, 2,   "SD480I-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    25 },  720,  576, 1, 2,   "SD576I-50" },

    { AV_PIX_FMT_YUV422P10, { 1001, 60000 }, 1280,  720, 0, 3,  "HD720P-60"  },
    { AV_PIX_FMT_YUV422P10, {    1,    50 }, 1280,  720, 0, 3,  "HD720P-50"  },
    { AV_PIX_FMT_YUV422P10, { 1001, 30000 }, 1920, 1080, 1, 3,  "HD1080I-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    25 }, 1920, 1080, 1, 3,  "HD1080I-50" },
    { AV_PIX_FMT_YUV422P10, { 1001, 60000 }, 1920, 1080, 0, 3,  "HD1080P-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    50 }, 1920, 1080, 0, 3,  "HD1080P-50" },

    { AV_PIX_FMT_YUV444P12, {    1,    24 }, 2048, 1080, 0, 4,        "DC2K" },
    { AV_PIX_FMT_YUV444P12, {    1,    24 }, 4096, 2160, 0, 5,        "DC4K" },

    { AV_PIX_FMT_YUV422P10, { 1001, 60000 }, 3840, 2160, 0, 6, "UHDTV 4K-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    50 }, 3840, 2160, 0, 6, "UHDTV 4K-50" },

    { AV_PIX_FMT_YUV422P10, { 1001, 60000 }, 7680, 4320, 0, 7, "UHDTV 8K-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    50 }, 7680, 4320, 0, 7, "UHDTV 8K-50" },

    { AV_PIX_FMT_YUV422P10, { 1001, 24000 }, 1920, 1080, 0, 3,  "HD1080P-24" },
    { AV_PIX_FMT_YUV422P10, { 1001, 30000 },  720,  486, 1, 2,  "SD Pro486"  },
};
static const int base_video_fmts_len = FF_ARRAY_ELEMS(base_video_fmts);

enum VC2_QM {
    VC2_QM_DEF = 0,
    VC2_QM_COL,
    VC2_QM_FLAT,

    VC2_QM_NB
};

typedef struct SubBand {
    dwtcoef *buf;
    ptrdiff_t stride;
    int width;
    int height;
    int shift;
} SubBand;

typedef struct Plane {
    SubBand band[MAX_DWT_LEVELS][4];
    dwtcoef *coef_buf;
    int width;
    int height;
    int dwt_width;
    int dwt_height;
    ptrdiff_t coef_stride;
} Plane;

typedef struct SliceArgs {
    PutBitContext pb;
    int cache[DIRAC_MAX_QUANT_INDEX];
    void *ctx;
    int x;
    int y;
    int quant_idx;
    int bits_ceil;
    int bits_floor;
    int bytes;
} SliceArgs;

typedef struct TransformArgs {
    void *ctx;
    Plane *plane;
    const void *idata;
    ptrdiff_t istride;
    int field;
    VC2TransformContext t;
} TransformArgs;

typedef struct VC2DwtPlane {
    int width;
    int height;
    int coef_stride;
    int pad;
} VC2DwtPlane;

typedef struct VC2DwtPushData {
    int s;
    int diff_offset;
    int level;
    int pad;
    VC2DwtPlane planes[3];
    VkDeviceAddress src_buf[3];
    VkDeviceAddress dst_buf[3];
} VC2DwtPushData;

typedef struct VC2EncAuxData {
    uint32_t quant[MAX_DWT_LEVELS][4];
    int ff_dirac_qscale_tab[116];
} VC2EncAuxData;

typedef struct VC2EncPushData {
    VkDeviceAddress p[4];
    VkDeviceAddress pb;
    VkDeviceAddress luts;
    VkDeviceAddress slice;
    int num_x;
    int num_y;
    int slice_x;
    int slice_y;
    int plane_x;
    int plane_y;
    int wavelet_depth;
    int quant_idx;
    int size_scaler;
    int prefix_bytes;
    int num_frame;
} VC2EncPushData;

typedef struct VC2EncSliceArgs {
    int quant_idx;
    int bytes;
    int pb_start;
    int pad;
} VC2EncSliceArgs;

typedef struct VC2EncSliceCalcPushData {
    VkDeviceAddress p[4];
    VkDeviceAddress luts;
    VkDeviceAddress slice;
    int num_x;
    int num_y;
    int slice_dim_x;
    int slice_dim_y;
    int plane_x;
    int plane_y;
    int wavelet_depth;
    int quant_idx;
    int size_scaler;
    int prefix_bytes;
    int bits_ceil;
    int bits_floor;
} VC2EncSliceCalcPushData;

typedef struct VC2EncContext {
    AVClass *av_class;
    PutBitContext pb;
    Plane plane[3];
    AVCodecContext *avctx;
    DiracVersionInfo ver;

    SliceArgs *slice_args;
    TransformArgs transform_args[3];

    /* For conversion from unsigned pixel values to signed */
    int diff_offset;
    int bpp;
    int bpp_idx;

    /* Picture number */
    uint32_t picture_number;

    /* Base video format */
    int base_vf;
    int level;
    int profile;

    /* Quantization matrix */
    uint8_t quant[MAX_DWT_LEVELS][4];
    int custom_quant_matrix;

    /* Division LUT */
    uint32_t qmagic_lut[116][2];

    int num_x; /* #slices horizontally */
    int num_y; /* #slices vertically */
    int group_x;
    int group_y;
    int prefix_bytes;
    int size_scaler;
    int chroma_x_shift;
    int chroma_y_shift;

    /* Rate control stuff */
    int frame_max_bytes;
    int slice_max_bytes;
    int slice_min_bytes;
    int q_ceil;
    int q_avg;

    /* Options */
    double tolerance;
    int wavelet_idx;
    int wavelet_depth;
    int strict_compliance;
    int slice_height;
    int slice_width;
    int interlaced;
    int num_frame;
    enum VC2_QM quant_matrix;

    /* Parse code state */
    uint32_t next_parse_offset;
    enum DiracParseCodes last_parse_code;

    /* Vulkan state */
    FFVulkanContext vkctx;
    FFVkQueueFamilyCtx qf;
    FFVkExecPool e;

    FFVulkanPipeline dwt_upload_pl;
    FFVulkanPipeline dwt_hor_pl, dwt_ver_pl, dwt_de_pl;
    FFVulkanPipeline slice_pl;
    FFVulkanPipeline enc_pl;
    FFVkSPIRVShader shd;
    FFVkSPIRVShader enc_shd;
    AVBufferPool* dwt_buf_pool;

    VkBuffer src_buf, dst_buf;
    VkBuffer slice_buf;
    uint32_t buf_plane_size;
    VC2EncPushData enc_consts;
    VC2DwtPushData dwt_consts;
    VC2EncSliceCalcPushData calc_consts;
} VC2EncContext;

void encode_parse_info(VC2EncContext *s, enum DiracParseCodes pcode);

void encode_seq_header(VC2EncContext *s);

void encode_picture_start(VC2EncContext *s);

int calc_slice_sizes(VC2EncContext *s);

int encode_slices(VC2EncContext *s);

int encode_hq_slice(AVCodecContext *avctx, void *arg);

#endif
