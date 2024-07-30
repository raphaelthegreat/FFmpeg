// Generated from libavcodec/vulkan/dwt_hor.comp
const char *ff_source_dwt_hor_comp =
"#extension GL_EXT_scalar_block_layout : require\n"
"#extension GL_EXT_debug_printf : require\n"
"\n"
"layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;\n"
"\n"
"layout(scalar, buffer_reference, buffer_reference_align = 4) buffer DwtCoef {\n"
"    uint coef_buf[];\n"
"};\n"
"\n"
"layout(push_constant, scalar) uniform ComputeInfo {\n"
"    int wavelet_depth;\n"
"    int s;\n"
"    int stride;\n"
"    int diff_offset;\n"
"    ivec2 work_area;\n"
"    DwtCoef dst_buf[3];\n"
"    DwtCoef src_buf[3];\n"
"};\n"
"\n"
"void main() {\n"
"    ivec2 coord = ivec2(gl_GlobalInvocationID.xy);\n"
"\n"
"    /* For horizontal synth pass, each invocation handles a horizontal pair of pixels */\n"
"    uint plane_idx = gl_GlobalInvocationID.z;\n"
"    ivec2 coord_x = coord << ivec2(1, 0);\n"
"    if (any(greaterThanEqual(coord_x, work_area))) {\n"
"        return;\n"
"    }\n"
"    uint a = src_buf[plane_idx].coef_buf[coord_x.y * stride + coord_x.x];\n"
"    uint b = src_buf[plane_idx].coef_buf[coord_x.y * stride + coord_x.x + 1];\n"
"    uint dst_b = (b - a) * (1 << s);\n"
"    uint dst_a = a * (1 << s) + ((dst_b + 1) >> 1);\n"
"    dst_buf[plane_idx].coef_buf[coord_x.y * stride + coord_x.x] = dst_a;\n"
"    dst_buf[plane_idx].coef_buf[coord_x.y * stride + coord_x.x + 1] = dst_b;\n"
"}\n"
;
