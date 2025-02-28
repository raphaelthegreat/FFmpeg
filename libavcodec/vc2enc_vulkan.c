/*
 * Copyright (C) 2025 raphaelthegreat <geoster3d@gmail.com>
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
#include "libavutil/thread.h"
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

#define LEGALL_TILE_DIM 16
#define LEGALL_WORKGROUP_X 64
#define SLICE_WORKGROUP_X 128
#define MAX_NUM_PLANES 3

extern const char *ff_source_common_comp;
extern const char *ff_source_vc2_encode_comp;
extern const char *ff_source_vc2_dwt_hor_legall_comp;
extern const char *ff_source_vc2_dwt_ver_legall_comp;
extern const char *ff_source_vc2_slice_sizes_comp;
extern const char *ff_source_vc2_dwt_upload_comp;
extern const char *ff_source_vc2_dwt_haar_comp;
extern const char *ff_source_vc2_dwt_haar_subgroup_comp;

typedef struct VC2DwtPushData {
    int s;
    union {
        int diff_offset;
        int plane_idx;
    };
    int level;
} VC2DwtPushData;

typedef struct VC2EncAuxData {
    int quant[MAX_DWT_LEVELS][4];
    int ff_dirac_qscale_tab[116];
    uint16_t interleaved_ue_golomb_tab[256];
    uint16_t top_interleaved_ue_golomb_tab[256];
    uint8_t golomb_len_tab[256];
} VC2EncAuxData;

typedef struct VC2EncPushData {
    VkDeviceAddress pb;
    int num_x;
    int num_y;
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
    int num_x;
    int num_y;
    int wavelet_depth;
    int size_scaler;
    int prefix_bytes;
    int bits_ceil;
    int bits_floor;
} VC2EncSliceCalcPushData;

typedef struct VC2EncVulkanContext {
    VC2EncContext base;
    FFVkBuffer lut_buf;
    FFVkBuffer slice_buf;
    VC2EncSliceArgs *slice_args;

    /* Vulkan state */
    FFVulkanContext vkctx;
    AVVulkanDeviceQueueFamily *qf;
    FFVkExecPool e;
    FFVkExecContext *exec;

    FFVulkanShader dwt_haar_shd;
    FFVulkanShader dwt_upload_shd;
    FFVulkanShader dwt_hor_shd, dwt_ver_shd;
    FFVulkanShader slice_shd;
    FFVulkanShader enc_shd;
    AVBufferPool* dwt_buf_pool;
    int haar_subgroup;

    VkBuffer plane_buf;
    VC2EncPushData enc_consts;
    VC2DwtPushData dwt_consts;
    VC2EncSliceCalcPushData calc_consts;

    /* Intermediate frame pool */
    AVBufferRef *intermediate_frames_ref[3];
    AVFrame *intermediate_frame[AV_NUM_DATA_POINTERS];
    VkImageView intermediate_views[AV_NUM_DATA_POINTERS];
} VC2EncVulkanContext;

static int init_vulkan_pipeline(VC2EncVulkanContext* s, FFVkSPIRVCompiler *spv,
                                FFVulkanShader* shd, int push_size,
                                int lg_x, int lg_y, int lg_z,
                                const char* pl_name, const char* pl_source,
                                int start_desc, int num_desc)
{
    int err = 0;
    uint8_t *spv_data;
    size_t spv_len;
    void *spv_opaque = NULL;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanDescriptorSetBinding *desc;

    ff_vk_shader_init(vkctx, shd, pl_name, VK_SHADER_STAGE_COMPUTE_BIT,
                      NULL, 0, lg_x, lg_y, lg_z, 0);

    av_bprintf(&shd->src, "struct SliceArgs {int quant_idx;int bytes;int pb_start;int pad;};\n");

    desc = (FFVulkanDescriptorSetBinding []) {
        {
            .name       = "src_planes",
            .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .mem_layout = ff_vk_shader_rep_fmt(vkctx->frames->sw_format, FF_VK_REP_UINT),
            .dimensions = 2,
            .elems      = av_pix_fmt_count_planes(vkctx->frames->sw_format),
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
        },
        {
            .name       = "coef_buf",
            .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .mem_layout = "r32i",
            .dimensions = 2,
            .elems      = 3,
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
        },
        {
            .name        = "AuxData",
            .type        = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            .stages      = VK_SHADER_STAGE_COMPUTE_BIT,
            .mem_layout  = "scalar",
            .buf_content = "int lut_quant[5][4]; int ff_dirac_qscale_tab[116]; "
                           "uint16_t interleaved_ue_golomb_tab[256]; "
                           "uint16_t top_interleaved_ue_golomb_tab[256]; "
                           "uint8_t golomb_len_tab[256];",
        },
        {
            .name        = "SliceBuffer",
            .type        = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            .stages      = VK_SHADER_STAGE_COMPUTE_BIT,
            .mem_layout  = "scalar",
            .buf_content = "SliceArgs slice_args[];",
        },
    };
    RET(ff_vk_shader_add_descriptor_set(vkctx, shd, desc + start_desc, num_desc, 0, 0));

    ff_vk_shader_add_push_const(shd, 0, push_size, VK_SHADER_STAGE_COMPUTE_BIT);
    av_bprintf(&shd->src, "#define PB_UNALIGNED\n");
    av_bprintf(&shd->src, "#define PLANE_FMT %d\n", vkctx->frames->sw_format);
    GLSLD(ff_source_common_comp);
    GLSLD(pl_source);

    /* Compile Haar shader */
    RET(spv->compile_shader(vkctx, spv, shd, &spv_data, &spv_len, "main", &spv_opaque));
    RET(ff_vk_shader_link(vkctx, shd, spv_data, spv_len, "main"));
    RET(ff_vk_shader_register_exec(vkctx, &s->e, shd));

fail:
    return err;
}

static int init_frame_pools(AVCodecContext *avctx)
{
    int i, err = 0;
    VC2EncVulkanContext *sv = avctx->priv_data;
    AVHWFramesContext *frames_ctx;
    AVVulkanFramesContext *vk_frames;
    enum AVPixelFormat sw_format = AV_PIX_FMT_GRAY32;

    for (i = 0; i < 3; i++) {
        sv->intermediate_frames_ref[i] = av_hwframe_ctx_alloc(sv->vkctx.device_ref);
        if (!sv->intermediate_frames_ref[i])
            return AVERROR(ENOMEM);

        frames_ctx = (AVHWFramesContext *)sv->intermediate_frames_ref[i]->data;
        frames_ctx->format    = AV_PIX_FMT_VULKAN;
        frames_ctx->sw_format = sw_format;
        frames_ctx->width     = sv->base.plane[i].dwt_width;
        frames_ctx->height    = sv->base.plane[i].dwt_height;

        vk_frames = frames_ctx->hwctx;
        vk_frames->tiling    = VK_IMAGE_TILING_OPTIMAL;
        vk_frames->usage     = VK_IMAGE_USAGE_STORAGE_BIT;
        vk_frames->img_flags = VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT;

        err = av_hwframe_ctx_init(sv->intermediate_frames_ref[i]);
        if (err < 0) {
            av_log(avctx, AV_LOG_ERROR, "Unable to initialize frame pool with format %s: %s\n",
                av_get_pix_fmt_name(sw_format), av_err2str(err));
            av_buffer_unref(&sv->intermediate_frames_ref[i]);
            return err;
        }
    }

    return err;
}

static void vulkan_bind_img_planes(FFVulkanContext *s, FFVkExecContext *e,
                                   FFVulkanShader *shd, VkImageView *views,
                                   int set, int binding)
{
    for (int i = 0; i < 3; i++)
        ff_vk_shader_update_img(s, e, shd, set, binding, i,
                                views[i], VK_IMAGE_LAYOUT_GENERAL,
                                VK_NULL_HANDLE);
}

static void dwt_plane_haar(VC2EncVulkanContext *s, FFVkExecContext *exec,
                          VkImageMemoryBarrier2* img_bar, int nb_img_bar)
{
    int p, group_x, group_y;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    Plane* plane;

    s->dwt_consts.level = s->base.wavelet_depth;
    vulkan_bind_img_planes(vkctx, exec, &s->dwt_haar_shd, s->intermediate_views, 0, 0);
    ff_vk_exec_bind_shader(vkctx, exec, &s->dwt_haar_shd);

    /* Haar pass */
    for (p = 0; p < 3; p++) {
        plane = &s->base.plane[p];
        s->dwt_consts.plane_idx = p;
        if (s->haar_subgroup) {
            group_x = FFALIGN(plane->dwt_width, 8) >> 3;
            group_y = FFALIGN(plane->dwt_height, 8) >> 3;
        } else {
            group_x = FFALIGN(plane->dwt_width, 32) >> 5;
            group_y = FFALIGN(plane->dwt_height, 32) >> 5;
        }

        ff_vk_shader_update_push_const(vkctx, exec, &s->dwt_haar_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(VC2DwtPushData), &s->dwt_consts);
        vk->CmdDispatch(exec->buf, group_x, group_y, 1);
    }

    /* Wait for haar dispatches to complete */
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                           .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                           .pImageMemoryBarriers = img_bar,
                                           .imageMemoryBarrierCount = nb_img_bar,
                                       });
}

static void dwt_plane_legall(VC2EncVulkanContext *s, FFVkExecContext *exec,
                             VkImageMemoryBarrier2* img_bar, int nb_img_bar)
{
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    int legall_group_x = (s->base.plane[0].dwt_height + LEGALL_WORKGROUP_X - 1) >> 6;
    int legall_group_y = (s->base.plane[0].dwt_width + LEGALL_WORKGROUP_X - 1) >> 6;
    int i;

    /* Perform legall wavelet trasform */
    for (i = 0; i < s->base.wavelet_depth; i++) {
        s->dwt_consts.level = i;

        /* Horizontal legall pass */
        vulkan_bind_img_planes(vkctx, exec, &s->dwt_hor_shd, s->intermediate_views, 0, 0);
        ff_vk_exec_bind_shader(vkctx, exec, &s->dwt_hor_shd);
        ff_vk_shader_update_push_const(vkctx, exec, &s->dwt_hor_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(VC2DwtPushData), &s->dwt_consts);
        vk->CmdDispatch(exec->buf, legall_group_x, 1, 3);
        vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                               .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                               .pImageMemoryBarriers = img_bar,
                                               .imageMemoryBarrierCount = nb_img_bar,
                                           });

        /* Vertical legall pass */
        vulkan_bind_img_planes(vkctx, exec, &s->dwt_ver_shd, s->intermediate_views, 0, 0);
        ff_vk_exec_bind_shader(vkctx, exec, &s->dwt_ver_shd);
        ff_vk_shader_update_push_const(vkctx, exec, &s->dwt_ver_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(VC2DwtPushData), &s->dwt_consts);
        vk->CmdDispatch(exec->buf, legall_group_y, 1, 3);
        vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                               .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                               .pImageMemoryBarriers = img_bar,
                                               .imageMemoryBarrierCount = nb_img_bar,
                                           });
    }
}

static int dwt_planes(VC2EncVulkanContext *s, AVFrame *frame)
{
    int i, err = 0, nb_img_bar = 0;
    int wavelet_idx = s->base.wavelet_idx;
    int group_x = s->base.plane[0].dwt_width >> 3;
    int group_y = s->base.plane[0].dwt_height >> 3;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    FFVkExecContext *exec = s->exec;
    VkImageView views[AV_NUM_DATA_POINTERS];
    VkImageMemoryBarrier2 img_bar[AV_NUM_DATA_POINTERS];

    /* Generate barriers and image views for frame images. */
    RET(ff_vk_exec_add_dep_frame(vkctx, exec, frame,
                                 VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                                 VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT));
    RET(ff_vk_create_imageviews(vkctx, exec, views, frame, FF_VK_REP_UINT));
    ff_vk_frame_barrier(vkctx, exec, frame, img_bar, &nb_img_bar,
                        VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                        VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                        VK_ACCESS_SHADER_READ_BIT,
                        VK_IMAGE_LAYOUT_GENERAL,
                        VK_QUEUE_FAMILY_IGNORED);

    /* Submit the image barriers. */
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                           .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                           .pImageMemoryBarriers = img_bar,
                                           .imageMemoryBarrierCount = nb_img_bar,
                                       });

    /* Create a temporaty frames */
    nb_img_bar = 0;
    for (i = 0; i < 3; i++) {
        s->intermediate_frame[i] = av_frame_alloc();
        if (!s->intermediate_frame[i])
            return AVERROR(ENOMEM);

        RET(av_hwframe_get_buffer(s->intermediate_frames_ref[i],
                                  s->intermediate_frame[i], 0));
        RET(ff_vk_exec_add_dep_frame(vkctx, exec, s->intermediate_frame[i],
                                     VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                                     VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT));
        RET(ff_vk_create_imageviews(vkctx, exec, &s->intermediate_views[i],
                                    s->intermediate_frame[i], FF_VK_REP_INT));
        ff_vk_frame_barrier(vkctx, exec, s->intermediate_frame[i], img_bar, &nb_img_bar,
                            VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                            VK_ACCESS_SHADER_READ_BIT,
                            VK_IMAGE_LAYOUT_GENERAL,
                            VK_QUEUE_FAMILY_IGNORED);
    }

    /* Submit the image barriers. */
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                           .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                           .pImageMemoryBarriers = img_bar,
                                           .imageMemoryBarrierCount = nb_img_bar,
                                       });

    /* Bind input images to the shader. */
    ff_vk_shader_update_img_array(vkctx, exec, &s->dwt_upload_shd, frame, views, 0, 0,
                                  VK_IMAGE_LAYOUT_GENERAL, VK_NULL_HANDLE);
    vulkan_bind_img_planes(vkctx, exec, &s->dwt_upload_shd, s->intermediate_views, 0, 1);

    /* Upload coefficients from planes to the buffer. */
    s->dwt_consts.diff_offset = s->base.diff_offset;
    ff_vk_exec_bind_shader(vkctx, exec, &s->dwt_upload_shd);
    ff_vk_shader_update_push_const(vkctx, exec, &s->dwt_upload_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(VC2DwtPushData), &s->dwt_consts);
    vk->CmdDispatch(exec->buf, group_x, group_y, 1);
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
                                            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
                                            .pImageMemoryBarriers = img_bar,
                                            .imageMemoryBarrierCount = nb_img_bar,
                                        });

    /* Perform wavelet trasform. */
    if (wavelet_idx == VC2_TRANSFORM_HAAR || wavelet_idx == VC2_TRANSFORM_HAAR_S)
        dwt_plane_haar(s, exec, img_bar, nb_img_bar);
    else if (wavelet_idx == VC2_TRANSFORM_5_3)
        dwt_plane_legall(s, exec, img_bar, nb_img_bar);

fail:
    return err;
}

static void encode_slices(VC2EncVulkanContext *sv)
{
    VC2EncContext *s = &sv->base;
    FFVkExecContext *exec = sv->exec;
    int num_slices = s->num_x * s->num_y;
    int num_slice_groups = (num_slices + SLICE_WORKGROUP_X - 1) >> 7;
    int i, skip = 0;
    FFVulkanContext *vkctx = &sv->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;

    /* Calculate slice sizes. */
    vulkan_bind_img_planes(vkctx, exec, &sv->slice_shd, sv->intermediate_views, 0, 0);
    ff_vk_shader_update_desc_buffer(vkctx, exec, &sv->slice_shd,
                                    0, 1, 0, &sv->lut_buf, 0,
                                    sizeof(VC2EncAuxData),
                                    VK_FORMAT_UNDEFINED);
    ff_vk_shader_update_desc_buffer(vkctx, exec, &sv->slice_shd,
                                    0, 2, 0, &sv->slice_buf, 0,
                                    sv->slice_buf.size,
                                    VK_FORMAT_UNDEFINED);
    ff_vk_exec_bind_shader(vkctx, exec, &sv->slice_shd);
    ff_vk_shader_update_push_const(vkctx, exec, &sv->slice_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(VC2EncSliceCalcPushData), &sv->calc_consts);
    vk->CmdDispatch(exec->buf, num_slice_groups, 1, 1);

    flush_put_bits(&s->pb);
    sv->enc_consts.pb += put_bytes_output(&s->pb);

    /* Wait for slice sizes to be written. */
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
        .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
        .pBufferMemoryBarriers = &(VkBufferMemoryBarrier2) {
            .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2,
            .srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
            .dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
            .srcAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT,
            .dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .buffer = sv->slice_buf.buf,
            .size = sizeof(VC2EncSliceArgs) * num_slices,
            .offset = 0,
        },
        .bufferMemoryBarrierCount = 1U,
    });

    /* Perform the encoding. */
    vulkan_bind_img_planes(vkctx, exec, &sv->enc_shd, sv->intermediate_views, 0, 0);
    ff_vk_shader_update_desc_buffer(vkctx, exec, &sv->enc_shd,
                                    0, 1, 0, &sv->lut_buf, 0,
                                    sizeof(VC2EncAuxData),
                                    VK_FORMAT_UNDEFINED);
    ff_vk_shader_update_desc_buffer(vkctx, exec, &sv->enc_shd,
                                    0, 2, 0, &sv->slice_buf, 0,
                                    sv->slice_buf.size,
                                    VK_FORMAT_UNDEFINED);
    ff_vk_exec_bind_shader(vkctx, exec, &sv->enc_shd);
    ff_vk_shader_update_push_const(vkctx, exec, &sv->enc_shd, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(VC2EncPushData), &sv->enc_consts);

    vk->CmdDispatch(exec->buf, num_slice_groups, 1, 1);

    ff_vk_exec_submit(vkctx, exec);
    ff_vk_exec_wait(vkctx, exec);

    for (int slice_y = 0; slice_y < s->num_y; slice_y++) {
        for (int slice_x = 0; slice_x < s->num_x; slice_x++) {
            VC2EncSliceArgs *args = &sv->slice_args[s->num_x * slice_y + slice_x];
            skip += args->bytes;
        }
    }

    /* Skip forward to write end header */
    skip_put_bytes(&s->pb, skip);

    /* Free allocated intermediate frames */
    for (i = 0; i < 3; i++)
        av_frame_free(&sv->intermediate_frame[i]);
}

static int encode_frame(VC2EncVulkanContext *sv, AVPacket *avpkt,
                        const AVFrame *frame, const int header_size)
{
    int ret;
    int64_t max_frame_bytes;
    AVBufferRef *avpkt_buf = NULL;
    FFVkBuffer* buf_vk = NULL;
    VC2EncContext* s = &sv->base;
    FFVulkanContext *vkctx = &sv->vkctx;

    /* Perform wavelet pass on the input data. */
    ret = dwt_planes(sv, (AVFrame*)frame);
    if (ret)
        return ret;

    /* Allocate a buffer that can fit at all all 3 planes of data */
    max_frame_bytes = header_size + MAX_NUM_PLANES * s->avctx->width
                                                   * s->avctx->height
                                                   * sizeof(dwtcoef);

    /* Get a pooled device local host visible buffer for writing output data */
    ret = ff_vk_get_pooled_buffer(vkctx, &sv->dwt_buf_pool, &avpkt_buf,
                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                  VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, NULL,
                                  max_frame_bytes,
                                  VK_MEMORY_PROPERTY_HOST_CACHED_BIT |
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                  VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (ret < 0)
        return ret;

    ff_vk_exec_add_dep_buf(vkctx, sv->exec, &avpkt_buf, 1, 1);
    buf_vk = (FFVkBuffer *)avpkt_buf->data;
    sv->enc_consts.pb = buf_vk->address;

    /* Initialize packet. */
    avpkt->buf = avpkt_buf;
    avpkt->data = buf_vk->mapped_mem;
    avpkt->size = max_frame_bytes;
    init_put_bits(&s->pb, avpkt->data, avpkt->size);

    /* Encode frame */
    ff_vc2_write_frame_header(s);
    encode_slices(sv);
    ff_vc2_write_sequence_end(s);

    return 0;
}

static av_cold int vc2_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                                    const AVFrame *frame, int *got_packet)
{
    int ret = 0;
    VC2EncVulkanContext *sv = avctx->priv_data;
    VC2EncContext *s = &sv->base;
    const int bitexact = avctx->flags & AV_CODEC_FLAG_BITEXACT;
    const int aux_data_size = bitexact ? sizeof("Lavc") : sizeof(LIBAVCODEC_IDENT);
    const int header_size = 100 + aux_data_size;

    ret = ff_vc2_frame_init(avctx, s);
    if (ret)
        return ret;

    sv->calc_consts.size_scaler = s->size_scaler;
    sv->calc_consts.bits_ceil  = s->slice_max_bytes << 3;
    sv->calc_consts.bits_floor = s->slice_min_bytes << 3;
    sv->enc_consts.prefix_bytes = 0;
    sv->enc_consts.size_scaler = s->size_scaler;

    sv->exec = ff_vk_exec_get(&sv->vkctx, &sv->e);
    ff_vk_exec_start(&sv->vkctx, sv->exec);

    ret = encode_frame(sv, avpkt, frame, header_size);
    if (ret)
        return ret;

    flush_put_bits(&s->pb);
    av_shrink_packet(avpkt, put_bytes_output(&s->pb));
    avpkt->flags |= AV_PKT_FLAG_KEY;
    *got_packet = 1;

    return 0;
}

static av_cold int vc2_encode_end(AVCodecContext *avctx)
{
    VC2EncVulkanContext *sv = avctx->priv_data;
    FFVulkanContext *vkctx = &sv->vkctx;
    int i;

    ff_vk_exec_pool_free(vkctx, &sv->e);

    ff_vk_shader_free(vkctx, &sv->dwt_upload_shd);
    ff_vk_shader_free(vkctx, &sv->dwt_haar_shd);
    ff_vk_shader_free(vkctx, &sv->dwt_hor_shd);
    ff_vk_shader_free(vkctx, &sv->dwt_ver_shd);
    ff_vk_shader_free(vkctx, &sv->slice_shd);
    ff_vk_shader_free(vkctx, &sv->enc_shd);

    ff_vk_free_buf(vkctx, &sv->slice_buf);
    ff_vk_free_buf(vkctx, &sv->lut_buf);

    for (i = 0; i < 3; i++) {
        ff_vc2enc_free_transforms(&sv->base.transform_args[i].t);
        av_buffer_unref(&sv->intermediate_frames_ref[i]);
    }

    av_buffer_pool_uninit(&sv->dwt_buf_pool);
    ff_vk_uninit(vkctx);

    return 0;
}

static av_cold int vc2_encode_init(AVCodecContext *avctx)
{
    int err = 0, depth;
    const AVPixFmtDescriptor *fmt;
    VC2EncVulkanContext *sv = avctx->priv_data;
    VC2EncContext *s = &sv->base;
    FFVulkanContext *vkctx = &sv->vkctx;
    FFVkSPIRVCompiler *spv;
    VC2EncAuxData *ad = NULL;
    unsigned int subgroup_size = vkctx->subgroup_props.maxSubgroupSize;

    /* Init vulkan */
    err = ff_vk_init(&sv->vkctx, avctx, NULL, avctx->hw_frames_ctx);
    if (err < 0)
        return err;

    sv->qf = ff_vk_qf_find(vkctx, VK_QUEUE_COMPUTE_BIT, 0);
    if (!sv->qf) {
        av_log(avctx, AV_LOG_ERROR, "Device has no compute queues!\n");
        return AVERROR(ENOTSUP);
    }

    spv = ff_vk_spirv_init();
    if (!spv) {
        av_log(avctx, AV_LOG_ERROR, "Unable to initialize SPIR-V compiler!\n");
        return AVERROR_EXTERNAL;
    }

    ff_vk_exec_pool_init(vkctx, sv->qf, &sv->e, 1, 0, 0, 0, NULL);

    /* Chroma subsampling */
    err = av_pix_fmt_get_chroma_sub_sample(vkctx->frames->sw_format, &s->chroma_x_shift,
                                                                     &s->chroma_y_shift);
    if (err < 0)
        return err;

    /* Bit depth and color range index */
    fmt = av_pix_fmt_desc_get(vkctx->frames->sw_format);
    depth = fmt->comp[0].depth;

    /* 16-bit depth is unsupported by this encoder */
    if (depth == 16) {
        av_log(avctx, AV_LOG_ERROR, "16-bit pixel format depth is unsupported by this encoder\n");
        return AVERROR(ENOTSUP);
    }

    /* Perform common initialization. */
    err = ff_vc2_encode_init(avctx, depth);
    if (err < 0)
        return err;

    /* Initialize Haar push data */
    sv->dwt_consts.diff_offset = s->diff_offset;
    sv->dwt_consts.s = s->wavelet_idx == VC2_TRANSFORM_HAAR_S ? 1 : 0;
    sv->dwt_consts.level = 0;

    /* Initializer slice calculation push data */
    sv->calc_consts.num_x = s->num_x;
    sv->calc_consts.num_y = s->num_y;
    sv->calc_consts.wavelet_depth = s->wavelet_depth;
    sv->calc_consts.prefix_bytes = s->prefix_bytes;

    /* Initialize encoder push data */
    sv->enc_consts.wavelet_depth = s->wavelet_depth;
    sv->enc_consts.num_x = s->num_x;
    sv->enc_consts.num_y = s->num_y;

    /* Create buffer for encoder auxilary data. */
    RET(ff_vk_create_buf(vkctx, &sv->lut_buf, sizeof(VC2EncAuxData), NULL, NULL,
                         VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT |
                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT));
    RET(ff_vk_map_buffer(vkctx, &sv->lut_buf, (void *)&ad, 0));
    ff_vc2_init_quant_matrix(s, ad->quant);
    memcpy(ad->ff_dirac_qscale_tab, ff_dirac_qscale_tab, sizeof(ff_dirac_qscale_tab));
    memcpy(ad->interleaved_ue_golomb_tab, interleaved_ue_golomb_tab, sizeof(interleaved_ue_golomb_tab));
    memcpy(ad->top_interleaved_ue_golomb_tab, top_interleaved_ue_golomb_tab, sizeof(top_interleaved_ue_golomb_tab));
    memcpy(ad->golomb_len_tab, golomb_len_tab, sizeof(golomb_len_tab));
    RET(ff_vk_unmap_buffer(vkctx, &sv->lut_buf, 1));

    /* Create buffer for encoder auxilary data. */
    RET(ff_vk_create_buf(vkctx, &sv->slice_buf,
                         sizeof(VC2EncSliceArgs) * s->num_x * s->num_y,
                         NULL, NULL,
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT));
    RET(ff_vk_map_buffer(vkctx, &sv->slice_buf, (void *)&sv->slice_args, 0));
    memset(sv->slice_args, 0, sv->slice_buf.size);

    /* Initialize intermediate frame pool. */
    RET(init_frame_pools(avctx));

    /* Initialize encoding pipelines */
    init_vulkan_pipeline(sv, spv, &sv->dwt_upload_shd, sizeof(VC2DwtPushData),
                         8, 8, 1, "dwt_upload_pl", ff_source_vc2_dwt_upload_comp, 0, 2);
    init_vulkan_pipeline(sv, spv, &sv->slice_shd, sizeof(VC2EncPushData),
                         SLICE_WORKGROUP_X, 1, 1, "slice_pl", ff_source_vc2_slice_sizes_comp, 1, 3);
    init_vulkan_pipeline(sv, spv, &sv->enc_shd, sizeof(VC2EncPushData),
                         SLICE_WORKGROUP_X, 1, 1, "enc_pl", ff_source_vc2_encode_comp, 1, 3);
    sv->haar_subgroup = 0;

    if (s->wavelet_idx == VC2_TRANSFORM_HAAR || s->wavelet_idx == VC2_TRANSFORM_HAAR_S) {
        if (subgroup_size == 32 && s->wavelet_depth < 3) {
            init_vulkan_pipeline(sv, spv, &sv->dwt_haar_shd, sizeof(VC2DwtPushData),
                                 64, 1, 1, "dwt_haar_pl", ff_source_vc2_dwt_haar_subgroup_comp, 1, 1);
            sv->haar_subgroup = 1;
        } else if (subgroup_size == 64 && s->wavelet_depth < 4) {
            init_vulkan_pipeline(sv, spv, &sv->dwt_haar_shd, sizeof(VC2DwtPushData),
                                 64, 1, 1, "dwt_haar_pl", ff_source_vc2_dwt_haar_subgroup_comp, 1, 1);
            sv->haar_subgroup = 1;
        } else {
            init_vulkan_pipeline(sv, spv, &sv->dwt_haar_shd, sizeof(VC2DwtPushData),
                                 32, 32, 1, "dwt_haar_pl", ff_source_vc2_dwt_haar_comp, 1, 1);
        }
    } else if (s->wavelet_idx == VC2_TRANSFORM_5_3) {
        init_vulkan_pipeline(sv, spv, &sv->dwt_hor_shd, sizeof(VC2DwtPushData),
                             LEGALL_WORKGROUP_X, 1, 1, "dwt_hor_pl", ff_source_vc2_dwt_hor_legall_comp, 1, 1);
        init_vulkan_pipeline(sv, spv, &sv->dwt_ver_shd, sizeof(VC2DwtPushData),
                             LEGALL_WORKGROUP_X, 1, 1, "dwt_ver_pl", ff_source_vc2_dwt_ver_legall_comp, 1, 1);
    }

fail:
    return err;
}

#define VC2ENC_FLAGS (AV_OPT_FLAG_ENCODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption vc2enc_options[] = {
    {"tolerance",     "Max undershoot in percent", offsetof(VC2EncContext, tolerance), AV_OPT_TYPE_DOUBLE, {.dbl = 5.0f}, 0.0f, 45.0f, VC2ENC_FLAGS, .unit = "tolerance"},
    {"slice_width",   "Slice width",  offsetof(VC2EncContext, slice_width), AV_OPT_TYPE_INT, {.i64 = 32}, 32, 1024, VC2ENC_FLAGS, .unit = "slice_width"},
    {"slice_height",  "Slice height", offsetof(VC2EncContext, slice_height), AV_OPT_TYPE_INT, {.i64 = 16}, 8, 1024, VC2ENC_FLAGS, .unit = "slice_height"},
    {"wavelet_depth", "Transform depth", offsetof(VC2EncContext, wavelet_depth), AV_OPT_TYPE_INT, {.i64 = 4}, 1, 5, VC2ENC_FLAGS, .unit = "wavelet_depth"},
    {"wavelet_type",  "Transform type",  offsetof(VC2EncContext, wavelet_idx), AV_OPT_TYPE_INT, {.i64 = VC2_TRANSFORM_HAAR}, 1, VC2_TRANSFORMS_NB, VC2ENC_FLAGS, .unit = "wavelet_idx"},
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

static const AVCodecHWConfigInternal *const vc2_hw_configs[] = {
    HW_CONFIG_ENCODER_FRAMES(VULKAN, VULKAN),
    HW_CONFIG_ENCODER_DEVICE(NONE,  VULKAN),
    NULL,
};

const FFCodec ff_vc2_vulkan_encoder = {
    .p.name         = "vc2_vulkan",
    CODEC_LONG_NAME("SMPTE VC-2"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_DIRAC,
    .p.capabilities = AV_CODEC_CAP_HARDWARE,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .priv_data_size = sizeof(VC2EncVulkanContext),
    .init           = vc2_encode_init,
    .close          = vc2_encode_end,
    FF_CODEC_ENCODE_CB(vc2_encode_frame),
    .p.priv_class   = &vc2enc_class,
    .defaults       = vc2enc_defaults,
    CODEC_PIXFMTS(AV_PIX_FMT_VULKAN),
    .hw_configs     = vc2_hw_configs,
};
