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
#include "libavutil/vulkan.h"

/* The limited size resolution of each slice forces us to do this */
#define SSIZE_ROUND(b) (FFALIGN((b), s->size_scaler) + 4 + s->prefix_bytes)

/* Decides the cutoff point in # of slices to distribute the leftover bytes */
#define SLICE_REDIST_TOTAL 150

typedef struct VC2BaseVideoFormat {
    enum AVPixelFormat pix_fmt;
    AVRational time_base;
    int width, height;
    uint8_t interlaced, level;
    char name[13];
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
    const struct VC2EncContext *ctx;
    union {
        int cache[DIRAC_MAX_QUANT_INDEX];
        uint8_t *buf;
    };
    int x;
    int y;
    int quant_idx;
    int bits_ceil;
    int bits_floor;
    int bytes;
} SliceArgs;

typedef struct TransformArgs {
    struct VC2EncContext *ctx;
    Plane *plane;
    const void *idata;
    ptrdiff_t istride;
    int field;
    VC2TransformContext t;
} TransformArgs;

typedef struct VC2DwtPlane {
    int width;
    int height;
    int dwt_width;
    int dwt_height;
} VC2DwtPlane;

typedef struct VC2DwtPushData {
    int s;
    union {
        int diff_offset;
        int plane_idx;
    };
    int level;
    VC2DwtPlane planes[3];
    VkDeviceAddress pbuf[3];
} VC2DwtPushData;

typedef struct VC2EncAuxData {
    uint32_t quant[MAX_DWT_LEVELS][4];
    int ff_dirac_qscale_tab[116];
} VC2EncAuxData;

typedef struct VC2EncPushData {
    VkDeviceAddress p[3];
    VkDeviceAddress pb;
    VkDeviceAddress luts;
    VkDeviceAddress slice;
    int num_x;
    int num_y;
    VC2DwtPlane planes[3];
    int wavelet_depth;
    int size_scaler;
    int prefix_bytes;
} VC2EncPushData;

typedef struct VC2EncSliceArgs {
    int quant_idx;
    int bytes;
    int pb_start;
    int pad;
} VC2EncSliceArgs;

typedef struct VC2EncSliceCalcPushData {
    VkDeviceAddress p[3];
    VkDeviceAddress luts;
    VkDeviceAddress slice;
    int num_x;
    int num_y;
    VC2DwtPlane planes[3];
    int wavelet_depth;
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
    VC2EncSliceArgs* vk_slice_args;
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
    enum VC2_QM quant_matrix;

    /* Parse code state */
    uint32_t next_parse_offset;
    enum DiracParseCodes last_parse_code;

    /* Vulkan state */
    FFVulkanContext vkctx;
    AVVulkanDeviceQueueFamily *qf;
    FFVkExecPool e;

    FFVulkanShader dwt_haar_shd;
    FFVulkanShader dwt_upload_shd;
    FFVulkanShader dwt_hor_shd, dwt_ver_shd;
    FFVulkanShader slice_shd;
    FFVulkanShader enc_shd;
    AVBufferPool* dwt_buf_pool;
    int haar_subgroup;

    VkBuffer plane_buf, slice_buf;
    uint32_t buf_plane_size;
    VC2EncPushData enc_consts;
    VC2DwtPushData dwt_consts;
    VC2EncSliceCalcPushData calc_consts;

    /* Intermediate frame pool */
    AVBufferRef *intermediate_frames_ref;
    AVFrame *intermediate_frame;
    VkImageView intermediate_views[AV_NUM_DATA_POINTERS];
} VC2EncContext;

static inline void put_vc2_ue_uint(PutBitContext *pb, uint32_t val)
{
    int i;
    int bits = 0;
    unsigned topbit = 1, maxval = 1;
    uint64_t pbits = 0;

    if (!val++) {
        put_bits(pb, 1, 1);
        return;
    }

    while (val > maxval) {
        topbit <<= 1;
        maxval <<= 1;
        maxval |=  1;
    }

    bits = ff_log2(topbit);

    for (i = 0; i < bits; i++) {
        topbit >>= 1;
        av_assert2(pbits <= UINT64_MAX>>3);
        pbits <<= 2;
        if (val & topbit)
            pbits |= 0x1;
    }

    put_bits64(pb, bits*2 + 1, (pbits << 1) | 1);
}

static inline int count_vc2_ue_uint(uint32_t val)
{
    int topbit = 1, maxval = 1;

    if (!val++)
        return 1;

    while (val > maxval) {
        topbit <<= 1;
        maxval <<= 1;
        maxval |=  1;
    }

    return ff_log2(topbit)*2 + 1;
}

void ff_vc2_init_quant_matrix(VC2EncContext *s);

void ff_vc2_encode_parse_info(VC2EncContext *s, enum DiracParseCodes pcode);

void ff_vc2_encode_seq_header(VC2EncContext *s);

void ff_vc2_encode_picture_start(VC2EncContext *s);

#endif
