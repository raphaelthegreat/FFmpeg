// Generated from libavcodec/vulkan/dwt_ver.comp
const char *ff_source_dwt_ver_comp =
"#extension GL_EXT_scalar_block_layout : require\n"
"#extension GL_EXT_debug_printf : require\n"
"\n"
"layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;\n"
"\n"
"layout(scalar, buffer_reference, buffer_reference_align = 4) buffer DwtCoef {\n"
"    int coef_buf[];\n"
"};\n"
"\n"
"struct Plane {\n"
"    ivec2 dim;\n"
"    int coef_stride;\n"
"    int pad;\n"
"};\n"
"\n"
"#define VC2_TRANSFORM_HAAR (3)\n"
"#define VC2_TRANSFORM_HAAR_S (4)\n"
"#define VC2_TRANSFORM_5_3 (1)\n"
"\n"
"layout(push_constant, scalar) uniform ComputeInfo {\n"
"    int s;\n"
"    int diff_offset;\n"
"    int level;\n"
"    int wavelet_type;\n"
"    Plane planes[3];\n"
"    DwtCoef dst_buf[3];\n"
"    DwtCoef src_buf[3];\n"
"};\n"
"\n"
"void main() {\n"
"    ivec2 coord = ivec2(gl_GlobalInvocationID.xy);\n"
"\n"
"    /* For vertical synth pass, each invocation handles a vertical pair of pixels */\n"
"    uint plane_idx = gl_GlobalInvocationID.z;\n"
"    ivec2 coord_y = coord << ivec2(0, 1);\n"
"    ivec2 work_area = planes[plane_idx].dim >> level;\n"
"    ivec2 dwt_area = work_area >> 1;\n"
"    DwtCoef buf = dst_buf[plane_idx];\n"
"    if (wavelet_type == VC2_TRANSFORM_HAAR || wavelet_type == VC2_TRANSFORM_HAAR_S) {\n"
"        if (any(greaterThanEqual(coord_y, work_area))) {\n"
"            return;\n"
"        }\n"
"        int stride = planes[plane_idx].coef_stride;\n"
"        int a = dst_buf[plane_idx].coef_buf[coord_y.y * stride + coord_y.x];\n"
"        int b = dst_buf[plane_idx].coef_buf[(coord_y.y + 1) * stride + coord_y.x];\n"
"        int dst_b = b - a;\n"
"        int dst_a = a + ((dst_b + 1) >> 1);\n"
"        dst_buf[plane_idx].coef_buf[coord_y.y * stride + coord_y.x] = dst_a;\n"
"        dst_buf[plane_idx].coef_buf[(coord_y.y + 1) * stride + coord_y.x] = dst_b;\n"
"    } else {\n"
"        if (any(greaterThanEqual(coord, dwt_area))) {\n"
"            return;\n"
"        }\n"
"        // Lifting stage 2\n"
"        int stride = planes[plane_idx].coef_stride;\n"
"        if (coord.y == 0) {\n"
"            buf.coef_buf[stride + coord.x] -= (buf.coef_buf[coord.x] + buf.coef_buf[2 * stride + coord.x] + 1) >> 1;\n"
"        } else if (coord.y < dwt_area.y - 1) {\n"
"            int start = coord.y * stride * 2;\n"
"            buf.coef_buf[start + stride + coord.x] -= (buf.coef_buf[start + coord.x] + buf.coef_buf[start + coord.x + stride * 2] + 1) >> 1;\n"
"        } else {\n"
"            int start = (work_area.y - 1) * stride;\n"
"            buf.coef_buf[start + coord.x] -= (2 * buf.coef_buf[start + coord.x - stride] + 1) >> 1;\n"
"        }\n"
"\n"
"        // Lifting stage 1\n"
"        if (coord.y == 0) {\n"
"            buf.coef_buf[coord.x] += (2 * buf.coef_buf[stride + coord.x] + 2) >> 2;\n"
"        } else if (coord.y < dwt_area.y - 1) {\n"
"            int start = coord.y * stride * 2;\n"
"            buf.coef_buf[start + coord.x] += (buf.coef_buf[start + coord.x + stride] + buf.coef_buf[start + coord.x - stride] + 2) >> 2;\n"
"        } else {\n"
"            int start = (work_area.y - 2) * stride;\n"
"            buf.coef_buf[start + coord.x] += (buf.coef_buf[start + coord.x - stride] + buf.coef_buf[start + coord.x + stride] + 2) >> 2;\n"
"        }\n"
"\n"
"        /*if (any(greaterThanEqual(coord, work_area))) {\n"
"            return;\n"
"        }\n"
"        int stride = planes[plane_idx].coef_stride;\n"
"        int index = stride * coord.y + coord.x;\n"
"        // Lifting stage 2\n"
"        if (coord.y > 0 && coord.y < work_area.y - 1 && (((coord.y - 1) & 1) == 0)) {\n"
"            int a = dst_buf[plane_idx].coef_buf[index];\n"
"            int b = dst_buf[plane_idx].coef_buf[stride * (coord.y - 1) + coord.x];\n"
"            int c = dst_buf[plane_idx].coef_buf[stride * (coord.y + 1) + coord.x];\n"
"            a -= (b + c + 1) >> 1;\n"
"            dst_buf[plane_idx].coef_buf[index] = a;\n"
"        } else if (coord.y == work_area.y - 1) {\n"
"            int a = dst_buf[plane_idx].coef_buf[index];\n"
"            int b = dst_buf[plane_idx].coef_buf[stride * (coord.y - 1) + coord.x];\n"
"            a -= (2 * b + 1) >> 1;\n"
"            dst_buf[plane_idx].coef_buf[index] = a;\n"
"        }\n"
"\n"
"        // Lifting stage 1\n"
"        if (coord.y == 0) {\n"
"            int a = dst_buf[plane_idx].coef_buf[index];\n"
"            int b = dst_buf[plane_idx].coef_buf[stride * (coord.y + 1) + coord.x];\n"
"            a += (2 * b + 2) >> 2;\n"
"            dst_buf[plane_idx].coef_buf[index] = a;\n"
"        } else if ((coord.y & 1) == 0) {\n"
"            int a = dst_buf[plane_idx].coef_buf[index];\n"
"            int b = dst_buf[plane_idx].coef_buf[stride * (coord.y - 1) + coord.x];\n"
"            int c = dst_buf[plane_idx].coef_buf[stride * (coord.y + 1) + coord.x];\n"
"            a += (c + b + 2) >> 2;\n"
"            dst_buf[plane_idx].coef_buf[index] = a;\n"
"        }*/\n"
"    }\n"
"}\n"
;
