#include "rknn_api.h"
#include <stdio.h>
#include <string.h>

int open_model(const char *path, rknn_context *ctx) {
    int ret = rknn_init(ctx, (void *)path, 0, 0, NULL);
    if (ret) return ret;
    rknn_sdk_version version = {0};
    if (rknn_query(*ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(version)) == 0)
        fprintf(stderr, "RKNN runtime %s, driver %s\n", version.api_version, version.drv_version);
    rknn_input_output_num n = {0};
    ret = rknn_query(*ctx, RKNN_QUERY_IN_OUT_NUM, &n, sizeof(n));
    if (ret || n.n_input != 1 || n.n_output != 1) return -101;
    rknn_tensor_attr out = {.index=0};
    ret = rknn_query(*ctx, RKNN_QUERY_OUTPUT_ATTR, &out, sizeof(out));
    if (ret || out.n_dims != 3 || out.dims[0] != 1 || out.dims[1] != 5 || out.dims[2] != 14490)
        return -102;
    return 0;
}
int run_frame(rknn_context ctx, unsigned char *rgb, float *result) {
    rknn_input input = {.index=0, .buf=rgb, .size=736*960*3,
                        .type=RKNN_TENSOR_UINT8, .fmt=RKNN_TENSOR_NHWC};
    int ret = rknn_inputs_set(ctx, 1, &input);
    if (ret) return ret;
    ret = rknn_run(ctx, NULL);
    if (ret) return ret;
    rknn_output out = {.index=0, .want_float=1, .is_prealloc=0};
    ret = rknn_outputs_get(ctx, 1, &out, NULL);
    if (ret) return ret;
    if (out.size == 5*14490*sizeof(float)) memcpy(result, out.buf, out.size);
    else ret = -103;
    rknn_outputs_release(ctx, 1, &out);
    return ret;
}
