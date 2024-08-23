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

#include "vc2enc_common.h"

static av_always_inline void put_vc2_ue_uint(PutBitContext *pb, uint32_t val)
{
    int i;
    int pbits = 0, bits = 0, topbit = 1, maxval = 1;

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
        pbits <<= 2;
        if (val & topbit)
            pbits |= 0x1;
    }

    put_bits(pb, bits*2 + 1, (pbits << 1) | 1);
}

static av_always_inline int count_vc2_ue_uint(uint32_t val)
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

/* VC-2 10.4 - parse_info() */
void encode_parse_info(VC2EncContext *s, enum DiracParseCodes pcode)
{
    uint32_t cur_pos, dist;

    align_put_bits(&s->pb);

    cur_pos = put_bytes_count(&s->pb, 0);

    /* Magic string */
    ff_put_string(&s->pb, "BBCD", 0);

    /* Parse code */
    put_bits(&s->pb, 8, pcode);

    /* Next parse offset */
    dist = cur_pos - s->next_parse_offset;
    AV_WB32(s->pb.buf + s->next_parse_offset + 5, dist);
    s->next_parse_offset = cur_pos;
    put_bits32(&s->pb, pcode == DIRAC_PCODE_END_SEQ ? 13 : 0);

    cur_pos = put_bytes_count(&s->pb, 0);

    /* Last parse offset */
    put_bits32(&s->pb, s->last_parse_code == DIRAC_PCODE_END_SEQ ? 13 : dist);

    s->last_parse_code = pcode;
}

/* VC-2 11.1 - parse_parameters()
 * The level dictates what the decoder should expect in terms of resolution
 * and allows it to quickly reject whatever it can't support. Remember,
 * this codec kinda targets cheapo FPGAs without much memory. Unfortunately
 * it also limits us greatly in our choice of formats, hence the flag to disable
 * strict_compliance */
static void encode_parse_params(VC2EncContext *s)
{
    put_vc2_ue_uint(&s->pb, s->ver.major); /* VC-2 demands this to be 2 */
    put_vc2_ue_uint(&s->pb, s->ver.minor); /* ^^ and this to be 0       */
    put_vc2_ue_uint(&s->pb, s->profile);   /* 3 to signal HQ profile    */
    put_vc2_ue_uint(&s->pb, s->level);     /* 3 - 1080/720, 6 - 4K      */
}

/* VC-2 11.3 - frame_size() */
static void encode_frame_size(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        AVCodecContext *avctx = s->avctx;
        put_vc2_ue_uint(&s->pb, avctx->width);
        put_vc2_ue_uint(&s->pb, avctx->height);
    }
}

/* VC-2 11.3.3 - color_diff_sampling_format() */
static void encode_sample_fmt(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        int idx;
        if (s->chroma_x_shift == 1 && s->chroma_y_shift == 0)
            idx = 1; /* 422 */
        else if (s->chroma_x_shift == 1 && s->chroma_y_shift == 1)
            idx = 2; /* 420 */
        else
            idx = 0; /* 444 */
        put_vc2_ue_uint(&s->pb, idx);
    }
}

/* VC-2 11.3.4 - scan_format() */
static void encode_scan_format(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance)
        put_vc2_ue_uint(&s->pb, s->interlaced);
}

/* VC-2 11.3.5 - frame_rate() */
static void encode_frame_rate(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        AVCodecContext *avctx = s->avctx;
        put_vc2_ue_uint(&s->pb, 0);
        put_vc2_ue_uint(&s->pb, avctx->time_base.den);
        put_vc2_ue_uint(&s->pb, avctx->time_base.num);
    }
}

/* VC-2 11.3.6 - aspect_ratio() */
static void encode_aspect_ratio(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        AVCodecContext *avctx = s->avctx;
        put_vc2_ue_uint(&s->pb, 0);
        put_vc2_ue_uint(&s->pb, avctx->sample_aspect_ratio.num);
        put_vc2_ue_uint(&s->pb, avctx->sample_aspect_ratio.den);
    }
}

/* VC-2 11.3.7 - clean_area() */
static void encode_clean_area(VC2EncContext *s)
{
    put_bits(&s->pb, 1, 0);
}

/* VC-2 11.3.8 - signal_range() */
static void encode_signal_range(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    uint32_t num = put_bits_count(&s->pb);
    if (!s->strict_compliance)
        put_vc2_ue_uint(&s->pb, s->bpp_idx);
}

/* VC-2 11.3.9 - color_spec() */
static void encode_color_spec(VC2EncContext *s)
{
    AVCodecContext *avctx = s->avctx;
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        int val;
        put_vc2_ue_uint(&s->pb, 0);

        /* primaries */
        put_bits(&s->pb, 1, 1);
        if (avctx->color_primaries == AVCOL_PRI_BT470BG)
            val = 2;
        else if (avctx->color_primaries == AVCOL_PRI_SMPTE170M)
            val = 1;
        else if (avctx->color_primaries == AVCOL_PRI_SMPTE240M)
            val = 1;
        else
            val = 0;
        put_vc2_ue_uint(&s->pb, val);

        /* color matrix */
        put_bits(&s->pb, 1, 1);
        if (avctx->colorspace == AVCOL_SPC_RGB)
            val = 3;
        else if (avctx->colorspace == AVCOL_SPC_YCOCG)
            val = 2;
        else if (avctx->colorspace == AVCOL_SPC_BT470BG)
            val = 1;
        else
            val = 0;
        put_vc2_ue_uint(&s->pb, val);

        /* transfer function */
        put_bits(&s->pb, 1, 1);
        if (avctx->color_trc == AVCOL_TRC_LINEAR)
            val = 2;
        else if (avctx->color_trc == AVCOL_TRC_BT1361_ECG)
            val = 1;
        else
            val = 0;
        put_vc2_ue_uint(&s->pb, val);
    }
}

/* VC-2 11.3 - source_parameters() */
static void encode_source_params(VC2EncContext *s)
{
    encode_frame_size(s);
    encode_sample_fmt(s);
    encode_scan_format(s);
    encode_frame_rate(s);
    encode_aspect_ratio(s);
    encode_clean_area(s);
    encode_signal_range(s);
    encode_color_spec(s);
}

/* VC-2 11 - sequence_header() */
void encode_seq_header(VC2EncContext *s)
{
    align_put_bits(&s->pb);
    encode_parse_params(s);
    put_vc2_ue_uint(&s->pb, s->base_vf);
    encode_source_params(s);
    put_vc2_ue_uint(&s->pb, s->interlaced); /* Frames or fields coding */
}

/* VC-2 12.1 - picture_header() */
static void encode_picture_header(VC2EncContext *s)
{
    align_put_bits(&s->pb);
    put_bits32(&s->pb, s->picture_number++);
}

/* VC-2 12.3.4.1 - slice_parameters() */
static void encode_slice_params(VC2EncContext *s)
{
    put_vc2_ue_uint(&s->pb, s->num_x);
    put_vc2_ue_uint(&s->pb, s->num_y);
    put_vc2_ue_uint(&s->pb, s->prefix_bytes);
    put_vc2_ue_uint(&s->pb, s->size_scaler);
}

/* 1st idx = LL, second - vertical, third - horizontal, fourth - total */
static const uint8_t vc2_qm_col_tab[][4] = {
    {20,  9, 15,  4},
    { 0,  6,  6,  4},
    { 0,  3,  3,  5},
    { 0,  3,  5,  1},
    { 0, 11, 10, 11}
};

static const uint8_t vc2_qm_flat_tab[][4] = {
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0}
};

static void init_quant_matrix(VC2EncContext *s)
{
    int level, orientation;

    if (s->wavelet_depth <= 4 && s->quant_matrix == VC2_QM_DEF) {
        s->custom_quant_matrix = 0;
        for (level = 0; level < s->wavelet_depth; level++) {
            s->quant[level][0] = ff_dirac_default_qmat[s->wavelet_idx][level][0];
            s->quant[level][1] = ff_dirac_default_qmat[s->wavelet_idx][level][1];
            s->quant[level][2] = ff_dirac_default_qmat[s->wavelet_idx][level][2];
            s->quant[level][3] = ff_dirac_default_qmat[s->wavelet_idx][level][3];
        }
        return;
    }

    s->custom_quant_matrix = 1;

    if (s->quant_matrix == VC2_QM_DEF) {
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = 0; orientation < 4; orientation++) {
                if (level <= 3)
                    s->quant[level][orientation] = ff_dirac_default_qmat[s->wavelet_idx][level][orientation];
                else
                    s->quant[level][orientation] = vc2_qm_col_tab[level][orientation];
            }
        }
    } else if (s->quant_matrix == VC2_QM_COL) {
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = 0; orientation < 4; orientation++) {
                s->quant[level][orientation] = vc2_qm_col_tab[level][orientation];
            }
        }
    } else {
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = 0; orientation < 4; orientation++) {
                s->quant[level][orientation] = vc2_qm_flat_tab[level][orientation];
            }
        }
    }
}

/* VC-2 12.3.4.2 - quant_matrix() */
static void encode_quant_matrix(VC2EncContext *s)
{
    int level;
    put_bits(&s->pb, 1, s->custom_quant_matrix);
    if (s->custom_quant_matrix) {
        put_vc2_ue_uint(&s->pb, s->quant[0][0]);
        for (level = 0; level < s->wavelet_depth; level++) {
            put_vc2_ue_uint(&s->pb, s->quant[level][1]);
            put_vc2_ue_uint(&s->pb, s->quant[level][2]);
            put_vc2_ue_uint(&s->pb, s->quant[level][3]);
        }
    }
}

/* VC-2 12.3 - transform_parameters() */
static void encode_transform_params(VC2EncContext *s)
{
    put_vc2_ue_uint(&s->pb, s->wavelet_idx);
    put_vc2_ue_uint(&s->pb, s->wavelet_depth);

    encode_slice_params(s);
    encode_quant_matrix(s);
}

/* VC-2 12.2 - wavelet_transform() */
static void encode_wavelet_transform(VC2EncContext *s)
{
    encode_transform_params(s);
    align_put_bits(&s->pb);
}

/* VC-2 12 - picture_parse() */
void encode_picture_start(VC2EncContext *s)
{
    align_put_bits(&s->pb);
    encode_picture_header(s);
    align_put_bits(&s->pb);
    encode_wavelet_transform(s);
}

#define QUANT(c, mul, add, shift) (((mul) * (c) + (add)) >> (shift))

/* VC-2 13.5.5.2 - slice_band() */
static void encode_subband(VC2EncContext *s, PutBitContext *pb, int sx, int sy,
                           SubBand *b, int quant)
{
    int x, y;

    const int left   = b->width  * (sx+0) / s->num_x;
    const int right  = b->width  * (sx+1) / s->num_x;
    const int top    = b->height * (sy+0) / s->num_y;
    const int bottom = b->height * (sy+1) / s->num_y;

    dwtcoef *coeff = b->buf + top * b->stride;
    const uint64_t q_m = ((uint64_t)(s->qmagic_lut[quant][0])) << 2;
    const uint64_t q_a = s->qmagic_lut[quant][1];
    //const int q_s = av_log2(ff_dirac_qscale_tab[quant]) + 32;
    const int qfactor = ff_dirac_qscale_tab[quant];

    for (y = top; y < bottom; y++) {
        for (x = left; x < right; x++) {
            //uint32_t c_abs = QUANT(FFABS(coeff[x]), q_m, q_a, q_s);
            uint32_t c_abs = (FFABS(coeff[x]));
            c_abs = (c_abs << 2) / qfactor;
            put_vc2_ue_uint(pb, c_abs);
            if (c_abs)
                put_bits(pb, 1, coeff[x] < 0);
        }
        coeff += b->stride;
    }
}

static int count_hq_slice(SliceArgs *slice, int quant_idx)
{
    int x, y;
    uint8_t quants[MAX_DWT_LEVELS][4];
    int bits = 0, p, level, orientation;
    VC2EncContext *s = slice->ctx;

    if (slice->cache[quant_idx])
        return slice->cache[quant_idx];

    bits += 8*s->prefix_bytes;
    bits += 8; /* quant_idx */

    // slice_quantizers
    for (level = 0; level < s->wavelet_depth; level++)
        for (orientation = !!level; orientation < 4; orientation++)
            quants[level][orientation] = FFMAX(quant_idx - s->quant[level][orientation], 0);

    for (p = 0; p < 3; p++) {
        int bytes_start, bytes_len, pad_s, pad_c;
        bytes_start = bits >> 3;
        bits += 8;
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = !!level; orientation < 4; orientation++) {
                SubBand *b = &s->plane[p].band[level][orientation];

                const int q_idx = quants[level][orientation];
                //const uint64_t q_m = ((uint64_t)s->qmagic_lut[q_idx][0]) << 2;
                //const uint64_t q_a = s->qmagic_lut[q_idx][1];
                //const int q_s = av_log2(ff_dirac_qscale_tab[q_idx]) + 32;
                const int qfactor = ff_dirac_qscale_tab[q_idx];

                const int left   = b->width  * slice->x    / s->num_x;
                const int right  = b->width  *(slice->x+1) / s->num_x;
                const int top    = b->height * slice->y    / s->num_y;
                const int bottom = b->height *(slice->y+1) / s->num_y;

                dwtcoef *buf = b->buf + top * b->stride;

                for (y = top; y < bottom; y++) {
                    for (x = left; x < right; x++) {
                        //uint32_t c_abs = QUANT(FFABS(buf[x]), q_m, q_a, q_s);
                        uint32_t c_abs = (FFABS(buf[x]));
                        c_abs = (c_abs << 2) / qfactor;
                        bits += count_vc2_ue_uint(c_abs);
                        bits += !!c_abs;
                    }
                    buf += b->stride;
                }
            }
        }
        bits += FFALIGN(bits, 8) - bits;
        bytes_len = (bits >> 3) - bytes_start - 1;
        pad_s = FFALIGN(bytes_len, s->size_scaler)/s->size_scaler;
        pad_c = (pad_s*s->size_scaler) - bytes_len;
        bits += pad_c*8;
    }

    slice->cache[quant_idx] = bits;

    return bits;
}

/* Approaches the best possible quantizer asymptotically, its kinda exaustive
 * but we have a LUT to get the coefficient size in bits. Guaranteed to never
 * overshoot, which is apparently very important when streaming */
static int rate_control(AVCodecContext *avctx, void *arg)
{
    SliceArgs *slice_dat = arg;
    VC2EncContext *s = slice_dat->ctx;
    const int top = slice_dat->bits_ceil;
    const int bottom = slice_dat->bits_floor;
    int quant_buf[2] = {-1, -1};
    int quant = slice_dat->quant_idx, step = 1;
    int bits_last, bits = count_hq_slice(slice_dat, quant);
    while ((bits > top) || (bits < bottom)) {
        const int signed_step = bits > top ? +step : -step;
        quant  = av_clip(quant + signed_step, 0, s->q_ceil-1);
        bits   = count_hq_slice(slice_dat, quant);
        if (quant_buf[1] == quant) {
            quant = FFMAX(quant_buf[0], quant);
            bits  = quant == quant_buf[0] ? bits_last : bits;
            break;
        }
        step         = av_clip(step/2, 1, (s->q_ceil-1)/2);
        quant_buf[1] = quant_buf[0];
        quant_buf[0] = quant;
        bits_last    = bits;
    }
    slice_dat->quant_idx = av_clip(quant, 0, s->q_ceil-1);
    slice_dat->bytes = SSIZE_ROUND(bits >> 3);
    return 0;
}

int calc_slice_sizes(VC2EncContext *s)
{
    int i, j, slice_x, slice_y, bytes_left = 0;
    int bytes_top[SLICE_REDIST_TOTAL] = {0};
    int64_t total_bytes_needed = 0;
    int slice_redist_range = FFMIN(SLICE_REDIST_TOTAL, s->num_x*s->num_y);
    SliceArgs *enc_args = s->slice_args;
    SliceArgs *top_loc[SLICE_REDIST_TOTAL] = {NULL};

    init_quant_matrix(s);

    for (slice_y = 0; slice_y < s->num_y; slice_y++) {
        for (slice_x = 0; slice_x < s->num_x; slice_x++) {
            SliceArgs *args = &enc_args[s->num_x*slice_y + slice_x];
            args->ctx = s;
            args->x   = slice_x;
            args->y   = slice_y;
            args->bits_ceil  = s->slice_max_bytes << 3;
            args->bits_floor = s->slice_min_bytes << 3;
            memset(args->cache, 0, s->q_ceil*sizeof(*args->cache));
        }
    }

    /* First pass - determine baseline slice sizes w.r.t. max_slice_size */
    s->avctx->execute(s->avctx, rate_control, enc_args, NULL, s->num_x*s->num_y,
                      sizeof(SliceArgs));

    /*for (i = 0; i < s->num_x*s->num_y; i++) {
        SliceArgs *args = &enc_args[i];
        bytes_left += args->bytes;
        for (j = 0; j < slice_redist_range; j++) {
            if (args->bytes > bytes_top[j]) {
                bytes_top[j] = args->bytes;
                top_loc[j]   = args;
                break;
            }
        }
    }

    bytes_left = s->frame_max_bytes - bytes_left;

    while (bytes_left > 0) {
        int distributed = 0;
        for (i = 0; i < slice_redist_range; i++) {
            SliceArgs *args;
            int bits, bytes, diff, prev_bytes, new_idx;
            if (bytes_left <= 0)
                break;
            if (!top_loc[i] || !top_loc[i]->quant_idx)
                break;
            args = top_loc[i];
            prev_bytes = args->bytes;
            new_idx = FFMAX(args->quant_idx - 1, 0);
            bits  = count_hq_slice(args, new_idx);
            bytes = SSIZE_ROUND(bits >> 3);
            diff  = bytes - prev_bytes;
            if ((bytes_left - diff) > 0) {
                args->quant_idx = new_idx;
                args->bytes = bytes;
                bytes_left -= diff;
                distributed++;
            }
        }
        if (!distributed)
            break;
    }*/

    for (i = 0; i < s->num_x*s->num_y; i++) {
        SliceArgs *args = &enc_args[i];
        total_bytes_needed += args->bytes;
        s->q_avg = (s->q_avg + args->quant_idx)/2;
    }

    return total_bytes_needed;
}

/* VC-2 13.5.3 - hq_slice */
int encode_hq_slice(AVCodecContext *avctx, void *arg)
{
    SliceArgs *slice_dat = arg;
    VC2EncContext *s = slice_dat->ctx;
    PutBitContext *pb = &slice_dat->pb;
    const int slice_x = slice_dat->x;
    const int slice_y = slice_dat->y;
    const int quant_idx = slice_dat->quant_idx;
    const int slice_bytes_max = slice_dat->bytes;
    uint8_t quants[MAX_DWT_LEVELS][4];
    int p, level, orientation;

    /* The reference decoder ignores it, and its typical length is 0 */
    uint8_t* start_ptr = put_bits_ptr(pb);
    memset(put_bits_ptr(pb), 0, s->prefix_bytes);
    skip_put_bytes(pb, s->prefix_bytes);

    put_bits(pb, 8, quant_idx);

    /* Slice quantization (slice_quantizers() in the specs) */
    for (level = 0; level < s->wavelet_depth; level++)
        for (orientation = !!level; orientation < 4; orientation++)
            quants[level][orientation] = FFMAX(quant_idx - s->quant[level][orientation], 0);

    /* Luma + 2 Chroma planes */
    for (p = 0; p < 3; p++) {
        int bytes_start, bytes_len, pad_s, pad_c;
        bytes_start = put_bytes_count(pb, 0);
        put_bits(pb, 8, 0);
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = !!level; orientation < 4; orientation++) {
                encode_subband(s, pb, slice_x, slice_y,
                               &s->plane[p].band[level][orientation],
                               quants[level][orientation]);
            }
        }
        flush_put_bits(pb);
        bytes_len = put_bytes_output(pb) - bytes_start - 1;
        if (p == 2) {
            int len_diff = slice_bytes_max - put_bytes_output(pb);
            pad_s = FFALIGN((bytes_len + len_diff), s->size_scaler)/s->size_scaler;
            pad_c = (pad_s*s->size_scaler) - bytes_len;
        } else {
            pad_s = FFALIGN(bytes_len, s->size_scaler)/s->size_scaler;
            pad_c = (pad_s*s->size_scaler) - bytes_len;
        }
        if (slice_dat->x == 0 && slice_dat->y == 0) {
            printf("Cpu bytes_len %d pad_s %d pad_c %d\n", bytes_len, pad_s, pad_c);
        }
        pb->buf[bytes_start] = pad_s;
        /* vc2-reference uses that padding that decodes to '0' coeffs */
        memset(put_bits_ptr(pb), 0xFF, pad_c);
        skip_put_bytes(pb, pad_c);
    }
    uint8_t* end_ptr = put_bits_ptr(pb);
    int written = end_ptr - start_ptr;
    //printf("(%d, %d) written %d slice_bytes_max %d\n", slice_x, slice_y, written, slice_bytes_max);
    return 0;
}

/* VC-2 13.5.1 - low_delay_transform_data() */
int encode_slices(VC2EncContext *s)
{
    uint8_t *buf;
    int slice_x, slice_y, skip = 0;
    SliceArgs *enc_args = s->slice_args;

    flush_put_bits(&s->pb);
    buf = put_bits_ptr(&s->pb);

    for (slice_y = 0; slice_y < s->num_y; slice_y++) {
        for (slice_x = 0; slice_x < s->num_x; slice_x++) {
            SliceArgs *args = &enc_args[s->num_x*slice_y + slice_x];
            init_put_bits(&args->pb, buf + skip, args->bytes+s->prefix_bytes);
            skip += args->bytes;
        }
    }

    s->avctx->execute(s->avctx, encode_hq_slice, enc_args, NULL, s->num_x*s->num_y,
                      sizeof(SliceArgs));

    skip_put_bytes(&s->pb, skip);

    return 0;
}
