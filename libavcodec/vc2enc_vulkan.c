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

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavutil/version.h"
#include "libavfilter/vulkan_spirv.h"
#include "libavutil/hwcontext_vulkan.h"
#include "libavutil/vulkan_loader.h"
#include "codec_internal.h"
#include "internal.h"
#include "encode.h"
#include "version.h"
#include "vc2enc_common.h"
#include "vulkan.h"
#include "hwconfig.h"

extern const char *ff_source_dwt_comp;
extern const char *ff_source_encode_comp;

static void init_vulkan(AVCodecContext *avctx) {
    VC2EncContext *s = avctx->priv_data;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVkSPIRVShader *shd;
    FFVkSPIRVCompiler *spv;
    FFVulkanDescriptorSetBinding *desc;
    uint8_t *spv_data;
    size_t spv_len;
    void *spv_opaque = NULL;
    AVBufferRef* dwt_buf = NULL;
    AVBufferRef* coef_buf[3];
    FFVkBuffer* vk_buf = NULL;
    VC2EncAuxData* ad = NULL;
    Plane *p;
    int i, level, err, ret;

    vkctx->frames_ref = av_buffer_ref(avctx->hw_frames_ctx);
    vkctx->frames = (AVHWFramesContext *)vkctx->frames_ref->data;
    vkctx->hwfc = vkctx->frames->hwctx;
    vkctx->device = (AVHWDeviceContext *)vkctx->frames->device_ref->data;
    vkctx->hwctx = vkctx->device->hwctx;
    vkctx->extensions = ff_vk_extensions_to_mask(vkctx->hwctx->enabled_dev_extensions,
                                                 vkctx->hwctx->nb_enabled_dev_extensions);
    ff_vk_load_functions(vkctx->device, &vkctx->vkfn, vkctx->extensions, 1, 1);
    ff_vk_load_props(vkctx);

    /* Initialize spirv compiler */
    spv = ff_vk_spirv_init();
    if (!spv) {
        av_log(avctx, AV_LOG_ERROR, "Unable to initialize SPIR-V compiler!\n");
        return;
    }

    ff_vk_qf_init(vkctx, &s->qf, VK_QUEUE_COMPUTE_BIT);

    for (i = 0; i < 3; i++) {
        p = &s->plane[i];
        ret = ff_vk_get_pooled_buffer(vkctx, &s->dwt_buf_pool, &coef_buf[i],
                                      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, NULL,
                                      p->coef_stride*p->dwt_height*sizeof(dwtcoef),
                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
        vk_buf = (FFVkBuffer*)coef_buf[i]->data;
        s->dwt_consts.p[i] = vk_buf->address;
        s->enc_consts.p[i] = vk_buf->address;
    }

    /* Slices */
    s->enc_consts.num_x = s->num_x;
    s->enc_consts.num_y = s->num_y;

    /* Create uniform buffer for encoder auxilary data. */
    ret = ff_vk_get_pooled_buffer(vkctx, &s->dwt_buf_pool, &dwt_buf,
                                  VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, NULL,
                                  sizeof(VC2EncAuxData),
                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    vk_buf = (FFVkBuffer*)dwt_buf->data;
    s->enc_consts.luts = vk_buf->address;
    ad = (VC2EncAuxData*)vk_buf->mapped_mem;
    if (s->wavelet_depth <= 4 && s->quant_matrix == VC2_QM_DEF) {
        s->custom_quant_matrix = 0;
        for (level = 0; level < s->wavelet_depth; level++) {
            ad->quant[level][0] = ff_dirac_default_qmat[s->wavelet_idx][level][0];
            ad->quant[level][1] = ff_dirac_default_qmat[s->wavelet_idx][level][1];
            ad->quant[level][2] = ff_dirac_default_qmat[s->wavelet_idx][level][2];
            ad->quant[level][3] = ff_dirac_default_qmat[s->wavelet_idx][level][3];
        }
    }
    memcpy(ad->ff_dirac_qscale_tab, ff_dirac_qscale_tab, sizeof(ff_dirac_qscale_tab));

    /* Initialize encoder push data */
    s->enc_consts.wavelet_depth = s->wavelet_depth;
    s->enc_consts.slice_x = s->slice_width;
    s->enc_consts.slice_y = s->slice_height;
    s->dwt_consts.wavelet_depth = s->wavelet_depth;

    ff_vk_exec_pool_init(vkctx, &s->qf, &s->e, s->qf.nb_queues * 4, 0, 0, 0, NULL);
    ff_vk_shader_init(&s->dwt_pl, &s->shd, "haar_dwt", VK_SHADER_STAGE_COMPUTE_BIT, 0);
    shd = &s->shd;

    /* Build DWT descriptor set. */
    desc = (FFVulkanDescriptorSetBinding[])
    {
        {
            .name = "textures",
            .type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .mem_layout = "r32f",
            .dimensions = 2,
            .elems = 3,
            .stages = VK_SHADER_STAGE_COMPUTE_BIT,
        },
    };
    RET(ff_vk_pipeline_descriptor_set_add(vkctx, &s->dwt_pl, shd, desc, 1, 0, 0));
    ff_vk_add_push_constant(&s->dwt_pl, 0, sizeof(VC2DwtPushData), VK_SHADER_STAGE_COMPUTE_BIT);

    GLSLD(ff_source_dwt_comp);

    /* Compile Haar shader */
    RET(spv->compile_shader(spv, vkctx, shd, &spv_data, &spv_len, "main", &spv_opaque));
    RET(ff_vk_shader_create(vkctx, shd, spv_data, spv_len, "main"));

    /* Initialize Haar compute pipeline */
    RET(ff_vk_init_compute_pipeline(vkctx, &s->dwt_pl, &s->shd));
    RET(ff_vk_exec_pipeline_register(vkctx, &s->e, &s->dwt_pl));

    ff_vk_shader_init(&s->enc_pl, &s->enc_shd, "encode", VK_SHADER_STAGE_COMPUTE_BIT, 0);
    shd = &s->enc_shd;

    /* Build encoder descriptor set. */
    ff_vk_add_push_constant(&s->enc_pl, 0, sizeof(VC2EncPushData), VK_SHADER_STAGE_COMPUTE_BIT);

    /* Compile encoding shader */
    GLSLD(ff_source_encode_comp);
    RET(spv->compile_shader(spv, vkctx, shd, &spv_data, &spv_len, "main", &spv_opaque));
    RET(ff_vk_shader_create(vkctx, shd, spv_data, spv_len, "main"));

    /* Initialize encoding compute pipeline */
    RET(ff_vk_init_compute_pipeline(vkctx, &s->enc_pl, shd));
    RET(ff_vk_exec_pipeline_register(vkctx, &s->e, &s->enc_pl));

fail:
}

/*
 * Transform basics for a 3 level transform
 * |---------------------------------------------------------------------|
 * |  LL-0  | HL-0  |                 |                                  |
 * |--------|-------|      HL-1       |                                  |
 * |  LH-0  | HH-0  |                 |                                  |
 * |----------------|-----------------|              HL-2                |
 * |                |                 |                                  |
 * |     LH-1       |      HH-1       |                                  |
 * |                |                 |                                  |
 * |----------------------------------|----------------------------------|
 * |                                  |                                  |
 * |                                  |                                  |
 * |                                  |                                  |
 * |              LH-2                |              HH-2                |
 * |                                  |                                  |
 * |                                  |                                  |
 * |                                  |                                  |
 * |---------------------------------------------------------------------|
 *
 * DWT transforms are generally applied by splitting the image in two vertically
 * and applying a low pass transform on the left part and a corresponding high
 * pass transform on the right hand side. This is known as the horizontal filter
 * stage.
 * After that, the same operation is performed except the image is divided
 * horizontally, with the high pass on the lower and the low pass on the higher
 * side.
 * Therefore, you're left with 4 subdivisions - known as  low-low, low-high,
 * high-low and high-high. They're referred to as orientations in the decoder
 * and encoder.
 *
 * The LL (low-low) area contains the original image downsampled by the amount
 * of levels. The rest of the areas can be thought as the details needed
 * to restore the image perfectly to its original size.
 */
static void dwt_plane(VC2EncContext *s, FFVkExecContext *exec, const AVFrame *frame)
{
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    VkImageView views[AV_NUM_DATA_POINTERS];

    /* Haar DWT pipeline */
    ff_vk_exec_bind_pipeline(vkctx, exec, &s->dwt_pl);

    /* Bind plane images */
    ff_vk_create_imageviews(vkctx, exec, views, frame);
    ff_vk_update_descriptor_img_array(vkctx, &s->dwt_pl, exec, frame, views, 0, 0,
                                      VK_IMAGE_LAYOUT_GENERAL, VK_NULL_HANDLE);

    /* Push data */
    ff_vk_update_push_exec(vkctx, exec, &s->dwt_pl, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(VC2DwtPushData), &s->dwt_consts);

    /* End of Haar DWT pass */
    vk->CmdDispatch(exec->buf, s->num_x, s->num_y, 1);
}

static void vulkan_encode_slices(VC2EncContext *s, FFVkExecContext *exec)
{
    uint8_t *buf;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;

    flush_put_bits(&s->pb);
    s->enc_consts.pb += put_bytes_count(&s->pb, 0);

    /* Encoder pipeline */
    ff_vk_exec_bind_pipeline(vkctx, exec, &s->enc_pl);

    /* Push data */
    s->enc_consts.quant_idx = 0; // TODO: Part of slice size estimation.
    s->enc_consts.prefix_bytes = 0;
    s->enc_consts.size_scaler = s->size_scaler;
    ff_vk_update_push_exec(vkctx, exec, &s->enc_pl, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(VC2EncPushData), &s->enc_consts);

    /* End of encoding pass pass */
    vk->CmdDispatch(exec->buf, s->num_x, s->num_y, 1);

    /* Submit command buffer and wait for completion */
    ff_vk_exec_submit(vkctx, exec);
    ff_vk_exec_wait(vkctx, exec);
}

static int encode_frame(VC2EncContext *s, AVPacket *avpkt, const AVFrame *frame,
                        const char *aux_data, const int header_size, int field)
{
    int i, ret;
    int64_t max_frame_bytes;
    AVBufferRef *avpkt_buf = NULL;
    FFVkBuffer* buf_vk = NULL;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    FFVkExecContext *exec = ff_vk_exec_get(&s->e);

    ff_vk_exec_start(vkctx, exec);
    ret = ff_vk_exec_add_dep_frame(vkctx, exec, frame,
                                   VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                                   VK_PIPELINE_STAGE_2_TRANSFER_BIT);

    /* Perform Haar DWT pass on the inpute frame. */
    dwt_plane(s, exec, frame);

    /* Calculate per-slice quantizers and sizes */
    /* TODO: Properly implement this */
    max_frame_bytes = header_size + 1280 * 720 * 3 / 2;

    /* Get a pooled device local host visible buffer for writing output data */
    if (field < 2) {
        /*ret = ff_get_encode_buffer(s->avctx, avpkt,
                                   max_frame_bytes << s->interlaced, 0);

        VkExternalMemoryBufferCreateInfo create_desc = {
            .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO,
            .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_HOST_ALLOCATION_BIT_EXT,
        };

        VkImportMemoryHostPointerInfoEXT import_desc = {
            .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_HOST_POINTER_INFO_EXT,
            .handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_HOST_ALLOCATION_BIT_EXT,
            .pHostPointer = avpkt->data,
        };

        VkMemoryHostPointerPropertiesEXT p_props = {
            .sType = VK_STRUCTURE_TYPE_MEMORY_HOST_POINTER_PROPERTIES_EXT,
        };

        ret = vk->GetMemoryHostPointerPropertiesEXT(vkctx->hwctx->act_dev,
                                                    import_desc.handleType,
                                                    import_desc.pHostPointer,
                                                    &p_props);
        ret = ff_vk_create_avbuf(vkctx, &avpkt_buf, max_frame_bytes << s->interlaced,
                                 &create_desc, &import_desc,
                                 VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        buf_vk = (FFVkBuffer *)avpkt_buf->data;*/

        ret = ff_vk_get_pooled_buffer(vkctx, &s->dwt_buf_pool, &avpkt_buf,
                                      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, NULL,
                                      max_frame_bytes << s->interlaced,
                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
        avpkt->buf = avpkt_buf;
        buf_vk = (FFVkBuffer *)avpkt_buf->data;
        avpkt->data = buf_vk->mapped_mem;
        avpkt->size = max_frame_bytes << s->interlaced;
        s->enc_consts.pb = buf_vk->address;

        if (ret) {
            av_log(s->avctx, AV_LOG_ERROR, "Error getting output packet.\n");
            return ret;
        }
        init_put_bits(&s->pb, avpkt->data, avpkt->size);
    }

    /* Sequence header */
    encode_parse_info(s, DIRAC_PCODE_SEQ_HEADER);
    encode_seq_header(s);

    /* Encoder version */
    if (aux_data) {
        encode_parse_info(s, DIRAC_PCODE_AUX);
        ff_put_string(&s->pb, aux_data, 1);
    }

    /* Picture header */
    encode_parse_info(s, DIRAC_PCODE_PICTURE_HQ);
    encode_picture_start(s);

    /* Encode slices */
    vulkan_encode_slices(s, exec);

    /* End sequence */
    encode_parse_info(s, DIRAC_PCODE_END_SEQ);

    return 0;
}

static av_cold int vc2_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                                    const AVFrame *frame, int *got_packet)
{
    int ret = 0;
    int slice_ceil, sig_size = 256;
    VC2EncContext *s = avctx->priv_data;
    const int bitexact = avctx->flags & AV_CODEC_FLAG_BITEXACT;
    const char *aux_data = bitexact ? "Lavc" : LIBAVCODEC_IDENT;
    const int aux_data_size = bitexact ? sizeof("Lavc") : sizeof(LIBAVCODEC_IDENT);
    const int header_size = 100 + aux_data_size;
    int64_t r_bitrate = avctx->bit_rate >> (s->interlaced);
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    FFVkExecContext *exec;

    /* Create an image view for each plane of source YUV420 frame */
    VkImageView views[AV_NUM_DATA_POINTERS];
    exec = ff_vk_exec_get(&s->e);
    ff_vk_create_imageviews(&s->vkctx, exec, views, frame);

    s->avctx = avctx;
    s->size_scaler = 2;
    s->prefix_bytes = 0;
    s->last_parse_code = 0;
    s->next_parse_offset = 0;

    /* Rate control */
    s->frame_max_bytes = (av_rescale(r_bitrate, s->avctx->time_base.num,
                                     s->avctx->time_base.den) >> 3) - header_size;
    s->slice_max_bytes = slice_ceil = av_rescale(s->frame_max_bytes, 1, s->num_x*s->num_y);

    /* Find an appropriate size scaler */
    while (sig_size > 255) {
        int r_size = SSIZE_ROUND(s->slice_max_bytes);
        if (r_size > slice_ceil) {
            s->slice_max_bytes -= r_size - slice_ceil;
            r_size = SSIZE_ROUND(s->slice_max_bytes);
        }
        sig_size = r_size/s->size_scaler; /* Signalled slize size */
        s->size_scaler <<= 1;
    }

    s->slice_min_bytes = s->slice_max_bytes - s->slice_max_bytes*(s->tolerance/100.0f);
    if (s->slice_min_bytes < 0)
        return AVERROR(EINVAL);

    ret = encode_frame(s, avpkt, frame, aux_data, header_size, s->interlaced);
    if (ret)
        return ret;
    if (s->interlaced) {
        ret = encode_frame(s, avpkt, frame, aux_data, header_size, 2);
        if (ret)
            return ret;
    }

    *got_packet = 1;

    return 0;
}

static av_cold int vc2_encode_end(AVCodecContext *avctx)
{
    int i;
    VC2EncContext *s = avctx->priv_data;

    av_log(avctx, AV_LOG_INFO, "Qavg: %i\n", s->q_avg);

    for (i = 0; i < 3; i++) {
        ff_vc2enc_free_transforms(&s->transform_args[i].t);
        av_freep(&s->plane[i].coef_buf);
    }

    av_freep(&s->slice_args);

    return 0;
}

static av_cold int vc2_encode_init(AVCodecContext *avctx)
{
    Plane *p;
    SubBand *b;
    int i, level, o, shift, ret, err = 0;
    const AVPixFmtDescriptor *fmt = av_pix_fmt_desc_get(avctx->pix_fmt);
    const int depth = fmt->comp[0].depth;
    VC2EncContext *s = avctx->priv_data;

    s->picture_number = 0;

    /* Total allowed quantization range */
    s->q_ceil    = DIRAC_MAX_QUANT_INDEX;

    s->ver.major = 2;
    s->ver.minor = 0;
    s->profile   = 3;
    s->level     = 3;

    s->base_vf   = -1;
    s->strict_compliance = 1;

    s->q_avg = 0;
    s->slice_max_bytes = 0;
    s->slice_min_bytes = 0;

    /* Mark unknown as progressive */
    s->interlaced = !((avctx->field_order == AV_FIELD_UNKNOWN) ||
                      (avctx->field_order == AV_FIELD_PROGRESSIVE));

    for (i = 0; i < base_video_fmts_len; i++) {
        const VC2BaseVideoFormat *fmt = &base_video_fmts[i];
        if (avctx->pix_fmt != fmt->pix_fmt)
            continue;
        if (avctx->time_base.num != fmt->time_base.num)
            continue;
        if (avctx->time_base.den != fmt->time_base.den)
            continue;
        if (avctx->width != fmt->width)
            continue;
        if (avctx->height != fmt->height)
            continue;
        if (s->interlaced != fmt->interlaced)
            continue;
        s->base_vf = i;
        s->level   = base_video_fmts[i].level;
        break;
    }

    if (s->interlaced)
        av_log(avctx, AV_LOG_WARNING, "Interlacing enabled!\n");

    if ((s->slice_width  & (s->slice_width  - 1)) ||
        (s->slice_height & (s->slice_height - 1))) {
        av_log(avctx, AV_LOG_ERROR, "Slice size is not a power of two!\n");
        return AVERROR_UNKNOWN;
    }

    if ((s->slice_width > avctx->width) ||
        (s->slice_height > avctx->height)) {
        av_log(avctx, AV_LOG_ERROR, "Slice size is bigger than the image!\n");
        return AVERROR_UNKNOWN;
    }

    if (s->base_vf <= 0) {
        if (avctx->strict_std_compliance < FF_COMPLIANCE_STRICT) {
            s->strict_compliance = s->base_vf = 0;
            av_log(avctx, AV_LOG_WARNING, "Format does not strictly comply with VC2 specs\n");
        } else {
            av_log(avctx, AV_LOG_WARNING, "Given format does not strictly comply with "
                   "the specifications, decrease strictness to use it.\n");
            return AVERROR_UNKNOWN;
        }
    } else {
        av_log(avctx, AV_LOG_INFO, "Selected base video format = %i (%s)\n",
               s->base_vf, base_video_fmts[s->base_vf].name);
    }

    /* Chroma subsampling */
    ret = av_pix_fmt_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_x_shift, &s->chroma_y_shift);
    if (ret)
        return ret;

    /* Bit depth and color range index */
    if (depth == 8 && avctx->color_range == AVCOL_RANGE_JPEG) {
        s->bpp = 1;
        s->bpp_idx = 1;
        s->diff_offset = 128;
    } else if (depth == 8 && (avctx->color_range == AVCOL_RANGE_MPEG ||
               avctx->color_range == AVCOL_RANGE_UNSPECIFIED)) {
        s->bpp = 1;
        s->bpp_idx = 2;
        s->diff_offset = 128;
    } else if (depth == 10) {
        s->bpp = 2;
        s->bpp_idx = 3;
        s->diff_offset = 512;
    } else {
        s->bpp = 2;
        s->bpp_idx = 4;
        s->diff_offset = 2048;
    }

    /* Planes initialization */
    AVHWFramesContext* hwfc = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
    for (i = 0; i < 3; i++) {
        int w, h;
        p = &s->plane[i];
        p->width      = avctx->width  >> (i ? s->chroma_x_shift : 0);
        p->height     = avctx->height >> (i ? s->chroma_y_shift : 0);
        if (s->interlaced)
            p->height >>= 1;
        p->dwt_width  = w = FFALIGN(p->width,  (1 << s->wavelet_depth));
        p->dwt_height = h = FFALIGN(p->height, (1 << s->wavelet_depth));
        p->coef_stride = FFALIGN(p->dwt_width, 32);
        p->coef_buf = av_mallocz(p->coef_stride*p->dwt_height*sizeof(dwtcoef));
        if (!p->coef_buf)
            return AVERROR(ENOMEM);
        for (level = s->wavelet_depth-1; level >= 0; level--) {
            w = w >> 1;
            h = h >> 1;
            for (o = 0; o < 4; o++) {
                b = &p->band[level][o];
                b->width  = w;
                b->height = h;
                b->stride = p->coef_stride;
                shift = (o > 1)*b->height*b->stride + (o & 1)*b->width;
                b->buf = p->coef_buf + shift;
            }
        }

        /* DWT init */
        if (ff_vc2enc_init_transforms(&s->transform_args[i].t,
                                      s->plane[i].coef_stride,
                                      s->plane[i].dwt_height,
                                      s->slice_width, s->slice_height))
            return AVERROR(ENOMEM);
    }

    /* Slices */
    s->num_x = s->plane[0].dwt_width/s->slice_width;
    s->num_y = s->plane[0].dwt_height/s->slice_height;

    s->slice_args = av_calloc(s->num_x*s->num_y, sizeof(SliceArgs));
    if (!s->slice_args)
        return AVERROR(ENOMEM);

    init_vulkan(avctx);

    return 0;
}

#define VC2ENC_FLAGS (AV_OPT_FLAG_ENCODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption vc2enc_options[] = {
    {"tolerance",     "Max undershoot in percent", offsetof(VC2EncContext, tolerance), AV_OPT_TYPE_DOUBLE, {.dbl = 5.0f}, 0.0f, 45.0f, VC2ENC_FLAGS, .unit = "tolerance"},
    {"slice_width",   "Slice width",  offsetof(VC2EncContext, slice_width), AV_OPT_TYPE_INT, {.i64 = 32}, 32, 1024, VC2ENC_FLAGS, .unit = "slice_width"},
    {"slice_height",  "Slice height", offsetof(VC2EncContext, slice_height), AV_OPT_TYPE_INT, {.i64 = 16}, 8, 1024, VC2ENC_FLAGS, .unit = "slice_height"},
    {"wavelet_depth", "Transform depth", offsetof(VC2EncContext, wavelet_depth), AV_OPT_TYPE_INT, {.i64 = 4}, 1, 5, VC2ENC_FLAGS, .unit = "wavelet_depth"},
    {"wavelet_type",  "Transform type",  offsetof(VC2EncContext, wavelet_idx), AV_OPT_TYPE_INT, {.i64 = VC2_TRANSFORM_9_7}, 0, VC2_TRANSFORMS_NB, VC2ENC_FLAGS, .unit = "wavelet_idx"},
        {"9_7",          "Deslauriers-Dubuc (9,7)", 0, AV_OPT_TYPE_CONST, {.i64 = VC2_TRANSFORM_9_7},    INT_MIN, INT_MAX, VC2ENC_FLAGS, .unit = "wavelet_idx"},
        {"5_3",          "LeGall (5,3)",            0, AV_OPT_TYPE_CONST, {.i64 = VC2_TRANSFORM_5_3},    INT_MIN, INT_MAX, VC2ENC_FLAGS, .unit = "wavelet_idx"},
        {"haar",         "Haar (with shift)",       0, AV_OPT_TYPE_CONST, {.i64 = VC2_TRANSFORM_HAAR_S}, INT_MIN, INT_MAX, VC2ENC_FLAGS, .unit = "wavelet_idx"},
        {"haar_noshift", "Haar (without shift)",    0, AV_OPT_TYPE_CONST, {.i64 = VC2_TRANSFORM_HAAR},   INT_MIN, INT_MAX, VC2ENC_FLAGS, .unit = "wavelet_idx"},
    {"qm", "Custom quantization matrix", offsetof(VC2EncContext, quant_matrix), AV_OPT_TYPE_INT, {.i64 = VC2_QM_DEF}, 0, VC2_QM_NB, VC2ENC_FLAGS, .unit = "quant_matrix"},
        {"default",   "Default from the specifications", 0, AV_OPT_TYPE_CONST, {.i64 = VC2_QM_DEF}, INT_MIN, INT_MAX, VC2ENC_FLAGS, .unit = "quant_matrix"},
        {"color",     "Prevents low bitrate discoloration", 0, AV_OPT_TYPE_CONST, {.i64 = VC2_QM_COL}, INT_MIN, INT_MAX, VC2ENC_FLAGS, .unit = "quant_matrix"},
        {"flat",      "Optimize for PSNR", 0, AV_OPT_TYPE_CONST, {.i64 = VC2_QM_FLAT}, INT_MIN, INT_MAX, VC2ENC_FLAGS, .unit = "quant_matrix"},
    {NULL}
};

static const AVClass vc2enc_class = {
    .class_name = "vc2_vulkan_encoder",
    .category = AV_CLASS_CATEGORY_ENCODER,
    .option = vc2enc_options,
    .item_name = av_default_item_name,
    .version = LIBAVUTIL_VERSION_INT
};

static const FFCodecDefault vc2enc_defaults[] = {
    { "b",              "600000000"   },
    { NULL },
};

const AVCodecHWConfigInternal *const ff_vc2_hw_configs[] = {
    HW_CONFIG_ENCODER_FRAMES(VULKAN, VULKAN),
    HW_CONFIG_ENCODER_DEVICE(NONE,  VULKAN),
    NULL,
};

const FFCodec ff_vc2_vulkan_encoder = {
    .p.name         = "vc2_vulkan",
    CODEC_LONG_NAME("SMPTE VC-2"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_DIRAC,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_HARDWARE,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .priv_data_size = sizeof(VC2EncContext),
    .init           = vc2_encode_init,
    .close          = vc2_encode_end,
    FF_CODEC_ENCODE_CB(vc2_encode_frame),
    .p.priv_class   = &vc2enc_class,
    .defaults       = vc2enc_defaults,
    .p.pix_fmts = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_VULKAN,
        AV_PIX_FMT_NONE,
    },
    .hw_configs     = ff_vc2_hw_configs,
};
