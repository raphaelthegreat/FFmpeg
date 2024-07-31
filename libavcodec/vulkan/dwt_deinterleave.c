// Generated from libavcodec/vulkan/dwt_deinterleave.comp
const char *ff_source_dwt_deinterleave_comp =
"#extension GL_EXT_scalar_block_layout : require\n"
"#extension GL_EXT_debug_printf : require\n"
"\n"
"layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;\n"
"\n"
"layout(scalar, buffer_reference, buffer_reference_align = 4) buffer DwtCoef {\n"
"    uint coef_buf[];\n"
"};\n"
"\n"
"struct Plane {\n"
"    ivec2 dim;\n"
"    int coef_stride;\n"
"    int pad;\n"
"};\n"
"\n"
"layout(push_constant, scalar) uniform ComputeInfo {\n"
"    int s;\n"
"    int diff_offset;\n"
"    int level;\n"
"    int pad;\n"
"    Plane planes[3];\n"
"    DwtCoef src_buf[3];\n"
"    DwtCoef dst_buf[3];\n"
"};\n"
"\n"
"void main() {\n"
"    ivec2 coord = ivec2(gl_GlobalInvocationID.xy);\n"
"    uint plane_idx = gl_GlobalInvocationID.z;\n"
"    ivec2 work_area = planes[plane_idx].dim >> level;\n"
"    if (any(greaterThanEqual(coord, work_area))) {\n"
"        return;\n"
"    }\n"
"    int stride = planes[plane_idx].coef_stride;\n"
"    ivec2 synth_size = work_area >> 1;\n"
"    ivec2 subband = coord & ivec2(1);\n"
"    ivec2 new_coord = subband * synth_size + (coord >> 1);\n"
"    uint texel = src_buf[plane_idx].coef_buf[coord.y * stride + coord.x];\n"
"    dst_buf[plane_idx].coef_buf[new_coord.y * stride + new_coord.x] = texel;\n"
"}\n"
;
