#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 0: 성공, 음수: 실패
int init_dl_csi_onnx(const char *model_path, int64_t T, int64_t D);

// 0: 성공, 음수: 실패
int run_dl_csi_onnx(const float *input, float *output);

// (옵션) 종료 시 호출
void fini_dl_csi_onnx(void);

#ifdef __cplusplus
}
#endif

