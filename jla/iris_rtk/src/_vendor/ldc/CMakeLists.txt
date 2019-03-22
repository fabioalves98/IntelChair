
include_directories(zstd zstd/common)

set(ldc_SRC STATIC
    ## lz4 compressor
    lz4/lz4.c
    quicklz/quicklz.c
    pithy/pithy.cpp

    zstd/common/entropy_common.c
    zstd/common/fse_decompress.c
    zstd/common/threading.c
    zstd/common/pool.c
    zstd/common/zstd_common.c
    zstd/common/error_private.c
    zstd/common/xxhash.c
    zstd/common/debug.c
    zstd/compress/fse_compress.c
    zstd/compress/huf_compress.c
    zstd/compress/zstd_compress.c
    zstd/compress/zstdmt_compress.c
    zstd/compress/zstd_fast.c
    zstd/compress/zstd_double_fast.c
    zstd/compress/zstd_lazy.c
    zstd/compress/zstd_opt.c
    zstd/compress/zstd_ldm.c
    zstd/compress/hist.c
    zstd/decompress/huf_decompress.c
    zstd/decompress/zstd_decompress.c
    zstd/decompress/zstd_ddict.c
    zstd/dictBuilder/cover.c
    zstd/dictBuilder/divsufsort.c
    zstd/dictBuilder/zdict.c
    zstd/deprecated/zbuff_common.c
    zstd/deprecated/zbuff_compress.c
    zstd/deprecated/zbuff_decompress.c
)

add_library(ldc ${ldc_SRC})
set_property(TARGET ldc PROPERTY POSITION_INDEPENDENT_CODE ON)

