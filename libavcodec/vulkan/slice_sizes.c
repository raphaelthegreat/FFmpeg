// Generated from libavcodec/vulkan/slice_sizes.comp
const char *ff_source_slice_sizes_comp =
"#extension GL_EXT_shader_explicit_arithmetic_types : require\n"
"#extension GL_EXT_scalar_block_layout : require\n"
"#extension GL_EXT_debug_printf : require\n"
"\n"
"// TODO make parallel\n"
"layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;\n"
"\n"
"#define DIRAC_MAX_QUANT_INDEX 116\n"
"#define MAX_DWT_LEVELS 5\n"
"\n"
"struct SliceArgs {\n"
"    int quant_idx;\n"
"    int bits_ceil;\n"
"    int bits_floor;\n"
"    int bytes;\n"
"};\n"
"\n"
"layout(scalar, buffer_reference, buffer_reference_align = 4) buffer DwtCoef {\n"
"    int coef_buf[];\n"
"};\n"
"\n"
"layout(scalar, buffer_reference, buffer_reference_align = 16) buffer SliceArgBuf {\n"
"    SliceArgs args[];\n"
"};\n"
"\n"
"layout(scalar, buffer_reference, buffer_reference_align = 4) buffer QuantLuts {\n"
"    int quant[5][4];\n"
"    int ff_dirac_qscale_tab[116];\n"
"};\n"
"\n"
"layout(push_constant, scalar) uniform ComputeInfo {\n"
"    DwtCoef planes[4];\n"
"    QuantLuts luts;\n"
"    SliceArgBuf slice;\n"
"    ivec2 num_slices;\n"
"    ivec2 slice_dim;\n"
"    int wavelet_depth;\n"
"    int quant_idx;\n"
"    int size_scaler;\n"
"    int prefix_bytes;\n"
"    int bits_ceil;\n"
"    int bits_floor;\n"
"};\n"
"\n"
"int count_vc2_ue_uint(uint val) {\n"
"    uint topbit = 1, maxval = 1;\n"
"    int bits = 0;\n"
"    if (val == 0) {\n"
"        return 1;\n"
"    }\n"
"    val++;\n"
"    while (val > maxval) {\n"
"        bits++;\n"
"        topbit <<= 1;\n"
"        maxval <<= 1;\n"
"        maxval |=  1;\n"
"    }\n"
"    return bits * 2 + 1;\n"
"}\n"
"\n"
"int ffalign(int x, int a) {\n"
"    return (x + a - 1) & ~(a - 1);\n"
"}\n"
"\n"
"int cache[DIRAC_MAX_QUANT_INDEX];\n"
"\n"
"int count_hq_slice(int quant_index) {\n"
"    int bits = 0;\n"
"    if (cache[quant_index] != 0) {\n"
"        return cache[quant_index];\n"
"    }\n"
"\n"
"    bits += 8*prefix_bytes;\n"
"    bits += 8; /* quant_idx */\n"
"\n"
"    ivec2 slice_coord = ivec2(gl_GlobalInvocationID.xy);\n"
"    for (int p = 0; p < 3; p++) {\n"
"        int bytes_start = bits >> 3;\n"
"        bits += 8;\n"
"        for (int level = 0; level < wavelet_depth; level++) {\n"
"            for (int o = int(level > 0); o < 4; o++) {\n"
"                int chroma_shift = (p == 0 ? 0 : 1);\n"
"                ivec2 plane_dim = (slice_dim * num_slices) >> chroma_shift;\n"
"                ivec2 band_dim = plane_dim >> (wavelet_depth - level);\n"
"                const int left = band_dim.x * slice_coord.x / num_slices.x;\n"
"                const int right = band_dim.x * (slice_coord.x+1) / num_slices.x;\n"
"                const int top = band_dim.y * slice_coord.y / num_slices.y;\n"
"                const int bottom = band_dim.y * (slice_coord.y+1) / num_slices.y;\n"
"\n"
"                const int stride = plane_dim.x;\n"
"                const int band_ptr = int(o > 1) * band_dim.y * stride + (o & 1) * band_dim.x;\n"
"                int start = band_ptr + top * stride;\n"
"                for (int y = top; y < bottom; y++) {\n"
"                    for (int x = left; x < right; x++) {\n"
"                        int coef = planes[p].coef_buf[start + x];\n"
"                        uint c_abs = uint(abs(coef));\n"
"                        bits += count_vc2_ue_uint(c_abs);\n"
"                        bits += int(c_abs > 0);\n"
"                    }\n"
"                    start += stride;\n"
"                }\n"
"            }\n"
"        }\n"
"        bits += ffalign(bits, 8) - bits;\n"
"        int bytes_len = (bits >> 3) - bytes_start - 1;\n"
"        int pad_s = ffalign(bytes_len, size_scaler) / size_scaler;\n"
"        int pad_c = (pad_s * size_scaler) - bytes_len;\n"
"        bits += pad_c * 8;\n"
"    }\n"
"\n"
"    cache[quant_index] = bits;\n"
"    return bits;\n"
"}\n"
"\n"
"int av_rescale(int a, int b, int c) {\n"
"    // TODO: This won't overflow, right?\n"
"    const int r = c / 2;\n"
"    return (a * b + r) / c;\n"
"}\n"
"\n"
"int ssize_round(int b) {\n"
"    return ffalign(b, size_scaler) + 4 + prefix_bytes;\n"
"}\n"
"\n"
"void main() {\n"
"    for (int i = 0; i < DIRAC_MAX_QUANT_INDEX; i++) {\n"
"        cache[i] = 0;\n"
"    }\n"
"    const int q_ceil = DIRAC_MAX_QUANT_INDEX;\n"
"    const int top = bits_ceil;\n"
"    const int bottom = bits_floor;\n"
"    int quant_buf[2] = int[2](-1, -1);\n"
"    int quant = quant_idx;\n"
"    int step = 1;\n"
"    int bits_last = 0;\n"
"    int bits = count_hq_slice(quant);\n"
"    ivec2 slice_coord = ivec2(gl_GlobalInvocationID.xy);\n"
"    while ((bits > top) || (bits < bottom)) {\n"
"        const int signed_step = bits > top ? +step : -step;\n"
"        quant = clamp(quant + signed_step, 0, q_ceil-1);\n"
"        bits = count_hq_slice(quant);\n"
"        if (quant_buf[1] == quant) {\n"
"            quant = max(quant_buf[0], quant);\n"
"            bits = quant == quant_buf[0] ? bits_last : bits;\n"
"            break;\n"
"        }\n"
"        step = clamp(step / 2, 1, (q_ceil - 1) / 2);\n"
"        quant_buf[1] = quant_buf[0];\n"
"        quant_buf[0] = quant;\n"
"        bits_last = bits;\n"
"    }\n"
"    uint slice_index = gl_WorkGroupID.y * gl_NumWorkGroups.x + gl_WorkGroupID.x;\n"
"    debugPrintfEXT(\"slice_index %d\", slice_index);\n"
"    slice.args[slice_index].quant_idx = clamp(quant, 0, q_ceil-1);\n"
"    slice.args[slice_index].bytes = ssize_round(bits >> 3);\n"
"}\n";
