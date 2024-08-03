// Generated from libavcodec/vulkan/encode.comp
const char *ff_source_encode_comp =
"#extension GL_EXT_shader_explicit_arithmetic_types : require\n"
"#extension GL_EXT_scalar_block_layout : require\n"
"#extension GL_EXT_debug_printf : require\n"
"\n"
"// TODO: Process pixels in parallel\n"
"#define WORKGROUP_X 1024\n"
"\n"
"layout(local_size_x = WORKGROUP_X, local_size_y = 1, local_size_z = 1) in;\n"
"\n"
"#define MAX_DWT_LEVELS (5)\n"
"\n"
"struct SliceArgs {\n"
"    int quant_idx;\n"
"    int bytes;\n"
"    int pb_start;\n"
"    int pad;\n"
"};\n"
"layout(std430, buffer_reference, buffer_reference_align = 16) buffer SliceArgBuf {\n"
"    SliceArgs args[];\n"
"};\n"
"layout(scalar, buffer_reference, buffer_reference_align = 4) buffer DwtCoef {\n"
"    int coef_buf[];\n"
"};\n"
"layout(scalar, buffer_reference, buffer_reference_align = 4) buffer BitBuf {\n"
"    uint data[];\n"
"};\n"
"layout(scalar, buffer_reference, buffer_reference_align = 4) buffer QuantLuts {\n"
"    int quant[5][4];\n"
"    int ff_dirac_qscale_tab[116];\n"
"};\n"
"\n"
"layout(push_constant, scalar) uniform ComputeInfo {\n"
"    DwtCoef planes[4];\n"
"    BitBuf pb;\n"
"    QuantLuts luts;\n"
"    SliceArgBuf slice;\n"
"    ivec2 num_slices;\n"
"    ivec2 slice_dim;\n"
"    int wavelet_depth;\n"
"    int quant_idx;\n"
"    int size_scaler;\n"
"    int prefix_bytes;\n"
"    int num_frame;\n"
"};\n"
"\n"
"#define BUF_BITS 32\n"
"#define BUF_BYTES 4\n"
"\n"
"int bit_buf = 0;\n"
"int bit_left = 32;\n"
"int write_start = 0;\n"
"int write_ptr = 0;\n"
"\n"
"void put_bits(int n, int value) {\n"
"    if (n < bit_left) {\n"
"        bit_buf = (bit_buf << n) | value;\n"
"        bit_left -= n;\n"
"    } else {\n"
"        bit_buf <<= bit_left;\n"
"        bit_buf |= (value >> (n - bit_left));\n"
"        uint data = pack32(unpack8(bit_buf).wzyx);\n"
"        pb.data[write_ptr++] = data;\n"
"        bit_left += BUF_BITS - n;\n"
"        bit_buf = value;\n"
"    }\n"
"}\n"
"\n"
"void flush_put_bits() {\n"
"    int bit_left_lo = (bit_left >> 3) << 3;\n"
"    bit_buf <<= (bit_left - bit_left_lo);\n"
"    if (bit_left_lo == 0) {\n"
"        uint data = pack32(unpack8(bit_buf).wzyx);\n"
"        pb.data[write_ptr++] = data;\n"
"        bit_left = 32;\n"
"        bit_buf = 0;\n"
"    } else {\n"
"        bit_left = bit_left_lo;\n"
"    }\n"
"}\n"
"\n"
"int put_bytes_count() {\n"
"    return (write_ptr - write_start) * BUF_BYTES + ((BUF_BITS - bit_left) >> 3);\n"
"}\n"
"\n"
"/* Same as skip_put_bytes in put_bits.h but fills in 0xFF */\n"
"void skip_put_bytes(int n) {\n"
"    int bytes_left = bit_left >> 3;\n"
"    if (n < bytes_left) {\n"
"        int n_bits = n << 3;\n"
"        int mask = (1 << n_bits) - 1;\n"
"        bit_buf <<= n_bits;\n"
"        bit_buf |= mask;\n"
"        bit_left -= n_bits;\n"
"        return;\n"
"    }\n"
"    int mask = (1 << bit_left) - 1;\n"
"    bit_buf <<= bit_left;\n"
"    bit_buf |= mask;\n"
"    pb.data[write_ptr++] = pack32(unpack8(bit_buf).wzyx);\n"
"    n -= bit_left >> 3;\n"
"    int skip_dwords = n >> 2;\n"
"    while (skip_dwords > 0) {\n"
"        pb.data[write_ptr++] = 0xFFFFFFFF;\n"
"        skip_dwords--;\n"
"    }\n"
"    int skip_bits = (n & 3) << 3;\n"
"    bit_buf = (1 << skip_bits) - 1;\n"
"    bit_left = BUF_BITS - skip_bits;\n"
"}\n"
"\n"
"void put_vc2_ue_uint(uint val) {\n"
"    int pbits = 0, topbit = 1, maxval = 1, bits = 0;\n"
"    if (val == 0) {\n"
"        put_bits(1, 1);\n"
"        return;\n"
"    }\n"
"    val++;\n"
"\n"
"    while (val > maxval) {\n"
"        topbit <<= 1;\n"
"        bits++;\n"
"        maxval <<= 1;\n"
"        maxval |=  1;\n"
"    }\n"
"\n"
"    for (int i = 0; i < bits; i++) {\n"
"        topbit >>= 1;\n"
"        pbits <<= 2;\n"
"        if ((val & topbit) != 0) {\n"
"            pbits |= 1;\n"
"        }\n"
"    }\n"
"\n"
"    put_bits(bits * 2 + 1, (pbits << 1) | 1);\n"
"}\n"
"\n"
"int align(int x, int a) {\n"
"    return (x + a - 1) & ~(a - 1);\n"
"}\n"
"\n"
"void main() {\n"
"    int slice_index = int(gl_GlobalInvocationID.x);\n"
"    int max_index = num_slices.x * num_slices.y;\n"
"    if (slice_index >= max_index) {\n"
"        return;\n"
"    }\n"
"\n"
"    /* Step 2. Quantize and encode */\n"
"    int pb_start = slice.args[slice_index].pb_start;\n"
"    for (int i = 0, index = WORKGROUP_X - 1; i < gl_WorkGroupID.x; i++) {\n"
"        pb_start += slice.args[index].pb_start + slice.args[index].bytes;\n"
"        index += WORKGROUP_X;\n"
"    }\n"
"    ivec2 slice_coord = ivec2(slice_index % num_slices.x, slice_index / num_slices.x);\n"
"    write_ptr = (pb_start >> 2);\n"
"    write_start = write_ptr;\n"
"\n"
"    int slice_bytes_max = slice.args[slice_index].bytes;\n"
"\n"
"    /* Write quant index for this slice */\n"
"    put_bits(8, slice.args[slice_index].quant_idx);\n"
"\n"
"    /* Luma + 2 Chroma planes */\n"
"    for (int p = 0; p < 3; p++) {\n"
"        int pad_s, pad_c;\n"
"        int bytes_start = put_bytes_count();\n"
"\n"
"        /* Save current location and write a zero value */\n"
"        int write_start = write_ptr;\n"
"        int bit_left_start = bit_left;\n"
"        int chroma_shift = (p == 0 ? 0 : 1);\n"
"        ivec2 plane_dim = (slice_dim * num_slices) >> chroma_shift;\n"
"        put_bits(8, 0);\n"
"        for (int level = 0; level < wavelet_depth; level++) {\n"
"            ivec2 band_size = plane_dim >> (wavelet_depth - level);\n"
"            for (int o = int(level > 0); o < 4; o++) {\n"
"                /* Encode subband */\n"
"                int left = band_size.x * (slice_coord.x) / num_slices.x;\n"
"                int right = band_size.x * (slice_coord.x+1) / num_slices.x;\n"
"                int top = band_size.y * (slice_coord.y) / num_slices.y;\n"
"                int bottom = band_size.y * (slice_coord.y+1) / num_slices.y;\n"
"\n"
"                int stride = plane_dim.x;\n"
"                int band_ptr = int(o > 1) * band_size.y * stride + (o & 1) * band_size.x;\n"
"                int start = band_ptr + top * stride;\n"
"                for (int y = top; y < bottom; y++) {\n"
"                    for (int x = left; x < right; x++) {\n"
"                        int coef = planes[p].coef_buf[start + x];\n"
"                        uint c_abs = uint(abs(coef));\n"
"                        put_vc2_ue_uint(c_abs);\n"
"                        if (c_abs != 0) {\n"
"                            put_bits(1, int(coef < 0));\n"
"                        }\n"
"                    }\n"
"                    start += stride;\n"
"                }\n"
"            }\n"
"        }\n"
"        flush_put_bits();\n"
"        int bytes_len = put_bytes_count() - bytes_start - 1;\n"
"        if (p == 2) {\n"
"            int len_diff = slice_bytes_max - put_bytes_count();\n"
"            pad_s = align((bytes_len + len_diff), size_scaler)/size_scaler;\n"
"            pad_c = (pad_s*size_scaler) - bytes_len;\n"
"        } else {\n"
"            pad_s = align(bytes_len, size_scaler)/size_scaler;\n"
"            pad_c = (pad_s*size_scaler) - bytes_len;\n"
"        }\n"
"        pb.data[write_start] |= (pad_s & 0xFF) << (BUF_BITS - bit_left_start);\n"
"        /* vc2-reference uses that padding that decodes to '0' coeffs */\n"
"        skip_put_bytes(pad_c);\n"
"    }\n"
"\n"
"    if ((write_ptr - write_start) != (slice_bytes_max >> 2)) {\n"
"        debugPrintfEXT(\"(%d) MISMATCH (%d, %d) written %d slice_bytes_max %d\\\\n\",\n"
"                        num_frame,\n"
"                       slice_coord.x, slice_coord.y,\n"
"                      (write_ptr - write_start) * 4, slice_bytes_max);\n"
"    }\n"
"}\n";
