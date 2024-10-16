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
#include "libavutil/vulkan_spirv.h"
#include "libavutil/hwcontext_vulkan.h"
#include "libavutil/vulkan_loader.h"
#include "libavutil/vulkan.h"
#include "codec_internal.h"
#include "internal.h"
#include "encode.h"
#include "version.h"
#include "vc2enc_common.h"
#include "hwconfig.h"

#define LEGALL_WORKGROUP_X 64
#define SLICE_WORKGROUP_X 128

extern const char *ff_source_encode_comp;
extern const char *ff_source_dwt_hor_legall_comp;
extern const char *ff_source_dwt_ver_legall_comp;
extern const char *ff_source_slice_sizes_comp;
extern const char *ff_source_dwt_upload_comp;
extern const char *ff_source_dwt_haar_comp;
extern const char *ff_source_dwt_haar_subgroup_comp;

static int init_vulkan_pipeline(VC2EncContext* s, FFVkSPIRVCompiler *spv,
                                FFVulkanShader* shd, int push_size,
                                int lg_x, int lg_y, int lg_z,
                                const char* pl_name, const char* pl_source,
                                int plane_img)
{
    uint8_t *spv_data;
    size_t spv_len;
    void *spv_opaque = NULL;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanDescriptorSetBinding *desc;
    int err = 0;

    ff_vk_shader_init(vkctx, shd, pl_name, VK_SHADER_STAGE_COMPUTE_BIT,
                      NULL, 0, lg_x, lg_y, lg_z, 0);

    if (plane_img) {
        desc = (FFVulkanDescriptorSetBinding []) {
            {
                .name       = "plane_imgs",
                .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                .mem_layout = ff_vk_shader_rep_fmt(vkctx->frames->sw_format,
                                                   s->vkctx.hwfc->format, FF_VK_REP_NATIVE),
                .dimensions = 2,
                .elems      = 3,
                .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
            },
        };
        RET(ff_vk_shader_add_descriptor_set(vkctx, shd, desc, 1, 0, 0));
    }

    ff_vk_shader_add_push_const(shd, 0, push_size, VK_SHADER_STAGE_COMPUTE_BIT);
    GLSLD(pl_source);

    /* Compile Haar shader */
    RET(spv->compile_shader(vkctx, spv, shd, &spv_data, &spv_len, "main", &spv_opaque));
    RET(ff_vk_shader_link(vkctx, shd, spv_data, spv_len, "main"));
    RET(ff_vk_shader_register_exec(vkctx, &s->e, shd));

fail:
    return err;
}

static int init_vulkan(AVCodecContext *avctx)
{
    VC2EncContext *s = avctx->priv_data;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVkSPIRVCompiler *spv;
    AVBufferRef* dwt_buf = NULL;
    AVBufferRef* coef_buf = NULL;
    FFVkBuffer* vk_buf = NULL;
    VC2EncAuxData* ad = NULL;
    Plane *p;
    VC2DwtPlane vk_plane;
    int i, level, ret;
    uint32_t subgroup_size = vkctx->subgroup_props.maxSubgroupSize;

    /* Initialize spirv compiler */
    spv = ff_vk_spirv_init();
    if (!spv) {
        av_log(avctx, AV_LOG_ERROR, "Unable to initialize SPIR-V compiler!\n");
        return -1;
    }

    ff_vk_qf_init(vkctx, &s->qf, VK_QUEUE_COMPUTE_BIT);
    ff_vk_exec_pool_init(vkctx, &s->qf, &s->e, 1, 0, 0, 0, NULL);

    /* Allocate coefficient buffer for each plane */
    p = &s->plane[0];
    s->buf_plane_size = p->coef_stride*p->dwt_height*sizeof(dwtcoef);
    ret = ff_vk_get_pooled_buffer(vkctx, &s->dwt_buf_pool, &coef_buf,
                                  VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                  VK_BUFFER_USAGE_TRANSFER_DST_BIT, NULL,
                                  s->buf_plane_size * 3,
                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vk_buf = (FFVkBuffer*)coef_buf->data;
    s->plane_buf = vk_buf->buf;

    for (i = 0; i < 3; i++) {
        p = &s->plane[i];
        vk_plane.dwt_width = p->dwt_width;
        vk_plane.dwt_height = p->dwt_height;
        vk_plane.width = p->width;
        vk_plane.height = p->height;
        memcpy(&s->calc_consts.planes[i], &vk_plane, sizeof(vk_plane));
        memcpy(&s->dwt_consts.planes[i], &vk_plane, sizeof(vk_plane));
        memcpy(&s->enc_consts.planes[i], &vk_plane, sizeof(vk_plane));
        s->enc_consts.p[i] = vk_buf->address + s->buf_plane_size * i;
        s->calc_consts.p[i] = vk_buf->address + s->buf_plane_size * i;
        s->dwt_consts.pbuf[i] = vk_buf->address + s->buf_plane_size * i;
    }

    /* Initialize Haar push data */
    s->dwt_consts.diff_offset = s->diff_offset;
    s->dwt_consts.s = s->wavelet_idx == VC2_TRANSFORM_HAAR_S ? 1 : 0;
    s->dwt_consts.level = 0;

    /* Initializer slice calc push data */
    s->calc_consts.num_x = s->num_x;
    s->calc_consts.num_y = s->num_y;
    s->calc_consts.wavelet_depth = s->wavelet_depth;
    s->calc_consts.prefix_bytes = s->prefix_bytes;

    /* Initialize encoder push data */
    s->enc_consts.wavelet_depth = s->wavelet_depth;
    s->enc_consts.num_x = s->num_x;
    s->enc_consts.num_y = s->num_y;

    /* Create buffer for encoder auxilary data. */
    ret = ff_vk_get_pooled_buffer(vkctx, &s->dwt_buf_pool, &dwt_buf,
                                  VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, NULL,
                                  sizeof(VC2EncAuxData),
                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    vk_buf = (FFVkBuffer*)dwt_buf->data;
    s->calc_consts.luts = vk_buf->address;
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

    /* Create buffer for slice arguments */
    ret = ff_vk_get_pooled_buffer(vkctx, &s->dwt_buf_pool, &dwt_buf,
                                  VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, NULL,
                                  sizeof(VC2EncSliceArgs) * s->num_x * s->num_y,
                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    vk_buf = (FFVkBuffer*)dwt_buf->data;
    s->slice_buf = vk_buf->buf;
    s->vk_slice_args = (VC2EncSliceArgs*)vk_buf->mapped_mem;
    s->calc_consts.slice = vk_buf->address;
    s->enc_consts.slice = vk_buf->address;

    s->haar_subgroup = 0;

    /* Initialize encoding pipelines */
    init_vulkan_pipeline(s, spv, &s->dwt_upload_shd, sizeof(VC2DwtPushData),
                         8, 8, 1, "dwt_upload_pl", ff_source_dwt_upload_comp, 1);
    init_vulkan_pipeline(s, spv, &s->slice_shd, sizeof(VC2EncPushData),
                         128, 1, 1, "slice_pl", ff_source_slice_sizes_comp, 0);
    init_vulkan_pipeline(s, spv, &s->enc_shd, sizeof(VC2EncPushData),
                         128, 1, 1, "enc_pl", ff_source_encode_comp, 0);

    if (s->wavelet_idx == VC2_TRANSFORM_HAAR || s->wavelet_idx == VC2_TRANSFORM_HAAR_S) {
        if (subgroup_size == 32 && s->wavelet_depth < 3) {
            init_vulkan_pipeline(s, spv, &s->dwt_haar_shd, sizeof(VC2DwtPushData),
                                 64, 1, 1, "dwt_haar_pl", ff_source_dwt_haar_subgroup_comp, 0);
            s->haar_subgroup = 1;
        } else if (subgroup_size == 64 && s->wavelet_depth < 4) {
            init_vulkan_pipeline(s, spv, &s->dwt_haar_shd, sizeof(VC2DwtPushData),
                                 64, 1, 1, "dwt_haar_pl", ff_source_dwt_haar_subgroup_comp, 0);
            s->haar_subgroup = 1;
        } else {
            init_vulkan_pipeline(s, spv, &s->dwt_haar_shd, sizeof(VC2DwtPushData),
                                 16, 16, 1, "dwt_haar_pl", ff_source_dwt_haar_comp, 0);
        }
    } else if (s->wavelet_idx == VC2_TRANSFORM_5_3) {
        init_vulkan_pipeline(s, spv, &s->dwt_hor_shd, sizeof(VC2DwtPushData),
                             64, 1, 1, "dwt_hor_pl", ff_source_dwt_hor_legall_comp, 0);
        init_vulkan_pipeline(s, spv, &s->dwt_ver_shd, sizeof(VC2DwtPushData),
                             64, 1, 1, "dwt_ver_pl", ff_source_dwt_ver_legall_comp, 0);
    }

    s->group_x = s->plane[0].dwt_width >> 3;
    s->group_y = s->plane[0].dwt_height >> 3;
    return ret;
}

static void dwt_plane_haar(VC2EncContext *s, FFVkExecContext *exec, VkBufferMemoryBarrier2* buf_bar)
{
    int p, group_x, group_y;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;

    s->dwt_consts.level = s->wavelet_depth;
    ff_vk_exec_bind_shader(vkctx, exec, &s->dwt_haar_shd);

    /* Haar pass */
    for (p = 0; p < 3; p++) {
        s->dwt_consts.plane_idx = p;
        if (s->haar_subgroup) {
            group_x = FFALIGN(s->plane[p].dwt_width, 8) >> 3;
            group_y = FFALIGN(s->plane[p].dwt_height, 8) >> 3;
        } else {
            group_x = FFALIGN(s->plane[p].dwt_width, 16) >> 4;
            group_y = FFALIGN(s->plane[p].dwt_height, 16) >> 4;
        }

        ff_vk_shader_update_push_const(vkctx, exec, &s->dwt_haar_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(VC2DwtPushData), &s->dwt_consts);
        vk->CmdDispatch(exec->buf, group_x, group_y, 1);
    }

    /* Wait for Haar dispatches to complete */
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                           .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                           .pBufferMemoryBarriers = buf_bar,
                                           .bufferMemoryBarrierCount = 1U,
                                       });
}

static void dwt_plane_legall(VC2EncContext *s, FFVkExecContext *exec, VkBufferMemoryBarrier2* buf_bar)
{
    int i;
    int legall_group_x = (s->plane[0].dwt_height + LEGALL_WORKGROUP_X - 1) >> 6;
    int legall_group_y = (s->plane[0].dwt_width + LEGALL_WORKGROUP_X - 1) >> 6;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;

    /* Perform Haar wavelet trasform */
    for (i = 0; i < s->wavelet_depth; i++) {
        s->dwt_consts.level = i;

        /* Horizontal Haar pass */
        ff_vk_exec_bind_shader(vkctx, exec, &s->dwt_hor_shd);
        ff_vk_shader_update_push_const(vkctx, exec, &s->dwt_hor_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(VC2DwtPushData), &s->dwt_consts);
        vk->CmdDispatch(exec->buf, legall_group_x, 1, 3);
        vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                               .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                               .pBufferMemoryBarriers = buf_bar,
                                               .bufferMemoryBarrierCount = 1U,
                                           });

        /* Vertical Haar pass */
        ff_vk_exec_bind_shader(vkctx, exec, &s->dwt_ver_shd);
        ff_vk_shader_update_push_const(vkctx, exec, &s->dwt_ver_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(VC2DwtPushData), &s->dwt_consts);
        vk->CmdDispatch(exec->buf, legall_group_y, 1, 3);
        vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                               .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                               .pBufferMemoryBarriers = buf_bar,
                                               .bufferMemoryBarrierCount = 1U,
                                           });
    }
}

static void dwt_plane(VC2EncContext *s, FFVkExecContext *exec, AVFrame *frame)
{
    int i, group_x = s->group_x, group_y = s->group_y;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    uint32_t num_slice_groups = (s->num_x*s->num_y + SLICE_WORKGROUP_X - 1) >> 7;
    VkBufferMemoryBarrier2 buf_bar;
    VkBufferMemoryBarrier2 slice_buf_bar;
    VkImageView views[AV_NUM_DATA_POINTERS];
    VkImageMemoryBarrier2 img_bar[AV_NUM_DATA_POINTERS];
    int nb_img_bar = 0;

    buf_bar = (VkBufferMemoryBarrier2) {
        .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2,
        .srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
        .dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
        .srcAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT,
        .dstAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT,
        .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .buffer = s->plane_buf,
        .size = s->buf_plane_size * 3,
        .offset = 0,
    };

    ff_vk_exec_add_dep_frame(vkctx, exec, frame,
                             VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                             VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT);
    ff_vk_create_imageviews(vkctx, exec, views, frame, FF_VK_REP_UINT);
    ff_vk_frame_barrier(vkctx, exec, frame, img_bar, &nb_img_bar,
                        VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                        VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                        VK_ACCESS_SHADER_READ_BIT,
                        VK_IMAGE_LAYOUT_GENERAL,
                        VK_QUEUE_FAMILY_IGNORED);
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                           .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                           .pImageMemoryBarriers = img_bar,
                                           .imageMemoryBarrierCount = nb_img_bar,
                                       });

    ff_vk_shader_update_img_array(vkctx, exec, &s->dwt_upload_shd, frame, views, 0, 0,
                                      VK_IMAGE_LAYOUT_GENERAL,
                                      VK_NULL_HANDLE);

    /* Upload coefficients from planes to the buffer. */
    s->dwt_consts.diff_offset = s->diff_offset;
    ff_vk_exec_bind_shader(vkctx, exec, &s->dwt_upload_shd);
    ff_vk_shader_update_push_const(vkctx, exec, &s->dwt_upload_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(VC2DwtPushData), &s->dwt_consts);
    vk->CmdDispatch(exec->buf, group_x, group_y, 3);
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                           .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                           .pBufferMemoryBarriers = &buf_bar,
                                           .bufferMemoryBarrierCount = 1U,
                                       });

    /* Perform Haar wavelet trasform */
    if (s->wavelet_idx == VC2_TRANSFORM_HAAR || s->wavelet_idx == VC2_TRANSFORM_HAAR_S) {
        dwt_plane_haar(s, exec, &buf_bar);
    } else if (s->wavelet_idx == VC2_TRANSFORM_5_3) {
        dwt_plane_legall(s, exec, &buf_bar);
    }

    /* Calculate slice sizes. */
    ff_vk_exec_bind_shader(vkctx, exec, &s->slice_shd);
    ff_vk_shader_update_push_const(vkctx, exec, &s->slice_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(VC2EncSliceCalcPushData), &s->calc_consts);
    vk->CmdDispatch(exec->buf, num_slice_groups, 1, 1);

    slice_buf_bar = (VkBufferMemoryBarrier2) {
        .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2,
        .srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
        .dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
        .srcAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT,
        .dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT,
        .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .buffer = s->slice_buf,
        .size = sizeof(VC2EncSliceArgs) * s->num_x * s->num_y,
        .offset = 0,
    };
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                           .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                           .pBufferMemoryBarriers = &slice_buf_bar,
                                           .bufferMemoryBarrierCount = 1U,
                                       });
}

static void vulkan_encode_slices(VC2EncContext *s, FFVkExecContext *exec)
{
    uint32_t num_slice_groups = (s->num_x*s->num_y + SLICE_WORKGROUP_X - 1) >> 7;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    int skip = 0;

    flush_put_bits(&s->pb);
    s->enc_consts.pb += put_bytes_output(&s->pb);

    ff_vk_exec_bind_shader(vkctx, exec, &s->enc_shd);
    ff_vk_shader_update_push_const(vkctx, exec, &s->enc_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(VC2EncPushData), &s->enc_consts);

    vk->CmdDispatch(exec->buf, num_slice_groups, 1, 1);

    ff_vk_exec_submit(vkctx, exec);
    ff_vk_exec_wait(vkctx, exec);

    for (int slice_y = 0; slice_y < s->num_y; slice_y++) {
        for (int slice_x = 0; slice_x < s->num_x; slice_x++) {
            VC2EncSliceArgs *args = &s->vk_slice_args[s->num_x*slice_y + slice_x];
            skip += args->bytes;
        }
    }

    /* Skip forward to write end header */
    skip_put_bytes(&s->pb, skip);
}

static int encode_frame(VC2EncContext *s, AVPacket *avpkt, const AVFrame *frame,
                        const char *aux_data, const int header_size, int field)
{
    int ret;
    int64_t max_frame_bytes;
    AVBufferRef *avpkt_buf = NULL;
    FFVkBuffer* buf_vk = NULL;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVkExecContext *exec = ff_vk_exec_get(vkctx, &s->e);

    ff_vk_exec_start(vkctx, exec);

    /* Perform Haar DWT pass on the inpute frame. */
    dwt_plane(s, exec, (AVFrame*)frame);

    /* Allocate a buffer that can fit at all all 3 planes of data */
    max_frame_bytes = header_size + s->avctx->width * s->avctx->height * sizeof(dwtcoef);
    s->custom_quant_matrix = 0;

    /* Get a pooled device local host visible buffer for writing output data */
    if (field < 2) {
        ret = ff_vk_get_pooled_buffer(vkctx, &s->dwt_buf_pool, &avpkt_buf,
                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                          VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, NULL,
                                      max_frame_bytes << s->interlaced,
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT |
                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        avpkt->buf = avpkt_buf;
        buf_vk = (FFVkBuffer *)avpkt_buf->data;
        avpkt->data = buf_vk->mapped_mem;
        avpkt->size = max_frame_bytes << s->interlaced;
        s->enc_consts.pb = buf_vk->address;

        if (ret < 0) {
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

    /* Update slice calc push data */
    s->calc_consts.size_scaler = s->size_scaler;
    s->calc_consts.bits_ceil  = s->slice_max_bytes << 3;
    s->calc_consts.bits_floor = s->slice_min_bytes << 3;
    s->enc_consts.prefix_bytes = 0;
    s->enc_consts.size_scaler = s->size_scaler;

    ret = encode_frame(s, avpkt, frame, aux_data, header_size, s->interlaced);
    if (ret)
        return ret;
    if (s->interlaced) {
        ret = encode_frame(s, avpkt, frame, aux_data, header_size, 2);
        if (ret)
            return ret;
    }

    flush_put_bits(&s->pb);
    av_shrink_packet(avpkt, put_bytes_output(&s->pb));
    avpkt->flags |= AV_PKT_FLAG_KEY;
    *got_packet = 1;

    return 0;
}

static av_cold int vc2_encode_end(AVCodecContext *avctx)
{
    int i;
    VC2EncContext *s = avctx->priv_data;

    av_log(avctx, AV_LOG_INFO, "Qavg: %i\n", s->q_avg);

    return 0;
}

static av_cold int vc2_encode_init(AVCodecContext *avctx)
{
    Plane *p;
    SubBand *b;
    int i, level, o, ret, depth;
    const AVPixFmtDescriptor *fmt;
    VC2EncContext *s = avctx->priv_data;
    FFVulkanContext *vkctx = &s->vkctx;

    vkctx->frames_ref = av_buffer_ref(avctx->hw_frames_ctx);
    vkctx->frames = (AVHWFramesContext *)vkctx->frames_ref->data;
    vkctx->hwfc = vkctx->frames->hwctx;
    vkctx->device = (AVHWDeviceContext *)vkctx->frames->device_ref->data;
    vkctx->hwctx = vkctx->device->hwctx;
    vkctx->extensions = ff_vk_extensions_to_mask(vkctx->hwctx->enabled_dev_extensions,
                                                 vkctx->hwctx->nb_enabled_dev_extensions);
    ff_vk_load_functions(vkctx->device, &vkctx->vkfn, vkctx->extensions, 1, 1);
    ff_vk_load_props(vkctx);

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
    ret = av_pix_fmt_get_chroma_sub_sample(vkctx->frames->sw_format, &s->chroma_x_shift, &s->chroma_y_shift);
    if (ret)
        return ret;

    /* Bit depth and color range index */
    fmt = av_pix_fmt_desc_get(vkctx->frames->sw_format);
    depth = fmt->comp[0].depth;
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
    for (i = 0; i < 3; i++) {
        int w, h;
        p = &s->plane[i];
        p->width      = avctx->width  >> (i ? s->chroma_x_shift : 0);
        p->height     = avctx->height >> (i ? s->chroma_y_shift : 0);
        if (s->interlaced)
            p->height >>= 1;
        p->dwt_width  = w = FFALIGN(p->width,  1 << s->wavelet_depth);
        p->dwt_height = h = FFALIGN(p->height, 1 << s->wavelet_depth);
        p->coef_stride = FFALIGN(p->dwt_width, 32);
        for (level = s->wavelet_depth-1; level >= 0; level--) {
            w = w >> 1;
            h = h >> 1;
            for (o = 0; o < 4; o++) {
                b = &p->band[level][o];
                b->width  = w;
                b->height = h;
                b->stride = p->coef_stride;
                b->shift = (o > 1)*b->height*b->stride + (o & 1)*b->width;
            }
        }
    }

    /* Slices */
    s->num_x = s->plane[0].dwt_width/s->slice_width;
    s->num_y = s->plane[0].dwt_height/s->slice_height;

    s->slice_args = av_calloc(s->num_x*s->num_y, sizeof(SliceArgs));
    if (!s->slice_args)
        return AVERROR(ENOMEM);

    for (i = 0; i < 116; i++) {
        const uint64_t qf = ff_dirac_qscale_tab[i];
        const uint32_t m = av_log2(qf);
        const uint32_t t = (1ULL << (m + 32)) / qf;
        const uint32_t r = (t*qf + qf) & UINT32_MAX;
        if (!(qf & (qf - 1))) {
            s->qmagic_lut[i][0] = 0xFFFFFFFF;
            s->qmagic_lut[i][1] = 0xFFFFFFFF;
        } else if (r <= 1 << m) {
            s->qmagic_lut[i][0] = t + 1;
            s->qmagic_lut[i][1] = 0;
        } else {
            s->qmagic_lut[i][0] = t;
            s->qmagic_lut[i][1] = t;
        }
    }
    init_vulkan(avctx);

    return 0;
}

#define VC2ENC_FLAGS (AV_OPT_FLAG_ENCODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption vc2enc_options[] = {
    {"tolerance",     "Max undershoot in percent", offsetof(VC2EncContext, tolerance), AV_OPT_TYPE_DOUBLE, {.dbl = 5.0f}, 0.0f, 45.0f, VC2ENC_FLAGS, .unit = "tolerance"},
    {"slice_width",   "Slice width",  offsetof(VC2EncContext, slice_width), AV_OPT_TYPE_INT, {.i64 = 32}, 32, 1024, VC2ENC_FLAGS, .unit = "slice_width"},
    {"slice_height",  "Slice height", offsetof(VC2EncContext, slice_height), AV_OPT_TYPE_INT, {.i64 = 16}, 8, 1024, VC2ENC_FLAGS, .unit = "slice_height"},
    {"wavelet_depth", "Transform depth", offsetof(VC2EncContext, wavelet_depth), AV_OPT_TYPE_INT, {.i64 = 4}, 1, 5, VC2ENC_FLAGS, .unit = "wavelet_depth"},
    {"wavelet_type",  "Transform type",  offsetof(VC2EncContext, wavelet_idx), AV_OPT_TYPE_INT, {.i64 = VC2_TRANSFORM_HAAR_S}, 0, VC2_TRANSFORMS_NB, VC2ENC_FLAGS, .unit = "wavelet_idx"},
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
