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

extern const VC2BaseVideoFormat base_video_fmts[];
extern const int base_video_fmts_len;

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
    enum VC2_QM quant_matrix;

    /* Parse code state */
    uint32_t next_parse_offset;
    enum DiracParseCodes last_parse_code;
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
