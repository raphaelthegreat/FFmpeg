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
#include "libavutil/attributes_internal.h"

#include "vc2enc_dwt.h"
#include "diractab.h"

/* The limited size resolution of each slice forces us to do this */
#define SSIZE_ROUND(b) (FFALIGN((b), s->size_scaler) + 4 + s->prefix_bytes)

FF_VISIBILITY_PUSH_HIDDEN

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

typedef struct TransformArgs {
    const struct VC2EncContext *ctx;
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

    struct SliceArgs *slice_args;
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
    int quant[MAX_DWT_LEVELS][4];
    int custom_quant_matrix;

    /* Division LUT */
    uint32_t qmagic_lut[116][2];

    int num_x; /* #slices horizontally */
    int num_y; /* #slices vertically */
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

extern uint16_t interleaved_ue_golomb_tab[256];
extern uint16_t top_interleaved_ue_golomb_tab[256];
extern uint8_t golomb_len_tab[256];
extern uint8_t qscale_len_tab[FF_ARRAY_ELEMS(ff_dirac_qscale_tab)];

static inline void ff_put_vc2_ue_uint_inline(PutBitContext *pb, uint32_t val)
{
    uint64_t pbits = 1;
    int bits = 1;

    ++val;

    while (val >> 8) {
        pbits |= (uint64_t)interleaved_ue_golomb_tab[val & 0xff] << bits;
        val  >>= 8;
        bits  += 16;
    }
    pbits |= (uint64_t)top_interleaved_ue_golomb_tab[val] << bits;
    bits  += golomb_len_tab[val];

    put_bits63(pb, bits, pbits);
}

int ff_vc2_encode_init(AVCodecContext *avctx, int depth);

int ff_vc2_frame_init(AVCodecContext *avctx, VC2EncContext *s);

void ff_vc2_write_frame_header(VC2EncContext *s);

void ff_vc2_write_sequence_end(VC2EncContext *s);

void ff_vc2_init_quant_matrix(VC2EncContext *s, int quant[MAX_DWT_LEVELS][4]);

void ff_vc2_encode_frame(VC2EncContext *s, void(*encode_slices)(VC2EncContext*));

FF_VISIBILITY_POP_HIDDEN

#endif
