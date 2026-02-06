#include "dl_csi_onnx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "common/utils/LOG/log.h"

// ONNX Runtime
#ifdef OAI_USE_ONNXRUNTIME
#include <onnxruntime_c_api.h>
#endif

static int64_t gT = 0;
static int64_t gD = 0;

static int g_inited = 0;   // 1 ok, 0 not, -1 failed
static int g_use_test_mode = 0;
static char g_test_mode[32] = {0};

static int get_env_int(const char *k, int defv) {
  const char *v = getenv(k);
  if (!v || !*v) return defv;
  return atoi(v);
}

static void get_env_str(const char *k, char *buf, size_t buflen) {
  const char *v = getenv(k);
  if (!v || !*v) { buf[0] = 0; return; }
  snprintf(buf, buflen, "%s", v);
}

#ifdef OAI_USE_ONNXRUNTIME
static const OrtApi *ort = NULL;
static OrtEnv *env = NULL;
static OrtSessionOptions *sess_opts = NULL;
static OrtSession *session = NULL;
static OrtMemoryInfo *meminfo = NULL;
static OrtAllocator *allocator = NULL;

static char *in_name = NULL;
static char *out_name = NULL;

static OrtStatus *check_status(OrtStatus *st, const char *where) {
  if (st) {
    const char *msg = ort->GetErrorMessage(st);
    LOG_E(NR_PHY, "[DL_CSI_ONNX] %s failed: %s\n", where, msg ? msg : "(null)");
    ort->ReleaseStatus(st);
    return (OrtStatus*)1; // non-null marker
  }
  return NULL;
}
#endif

int init_dl_csi_onnx(const char *model_path, int64_t T, int64_t D)
{
  gT = T;
  gD = D;

  // test mode
  get_env_str("OAI_DL_CSI_ONNX_TEST_MODE", g_test_mode, sizeof(g_test_mode));
  if (g_test_mode[0]) {
    g_use_test_mode = 1;
    g_inited = 1;
    LOG_I(NR_PHY, "[DL_CSI_ONNX] TEST_MODE=%s (no onnxruntime needed)\n", g_test_mode);
    return 0;
  }

#ifndef OAI_USE_ONNXRUNTIME
  LOG_E(NR_PHY, "[DL_CSI_ONNX] onnxruntime not enabled. Define OAI_USE_ONNXRUNTIME and link libonnxruntime.\n");
  g_inited = -1;
  return -1;
#else
  if (!model_path || !*model_path) {
    LOG_E(NR_PHY, "[DL_CSI_ONNX] model_path is empty\n");
    g_inited = -1;
    return -2;
  }

  ort = OrtGetApiBase()->GetApi(ORT_API_VERSION);
  if (!ort) {
    LOG_E(NR_PHY, "[DL_CSI_ONNX] OrtGetApi failed\n");
    g_inited = -1;
    return -3;
  }

  if (check_status(ort->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "oai-dl-csi", &env), "CreateEnv")) { g_inited = -1; return -4; }
  if (check_status(ort->CreateSessionOptions(&sess_opts), "CreateSessionOptions")) { g_inited = -1; return -5; }

  // threads: verification 목적이면 1로 고정 (재현성/부하 최소화)
  ort->SetIntraOpNumThreads(sess_opts, 1);
  ort->SetInterOpNumThreads(sess_opts, 1);
  ort->SetSessionGraphOptimizationLevel(sess_opts, ORT_ENABLE_ALL);

  if (check_status(ort->CreateSession(env, model_path, sess_opts, &session), "CreateSession")) { g_inited = -1; return -6; }
  if (check_status(ort->GetAllocatorWithDefaultOptions(&allocator), "GetAllocatorWithDefaultOptions")) { g_inited = -1; return -7; }

  size_t in_count = 0, out_count = 0;
  ort->SessionGetInputCount(session, &in_count);
  ort->SessionGetOutputCount(session, &out_count);
  if (in_count < 1 || out_count < 1) {
    LOG_E(NR_PHY, "[DL_CSI_ONNX] invalid io count: in=%zu out=%zu\n", in_count, out_count);
    g_inited = -1;
    return -8;
  }

  if (check_status(ort->SessionGetInputName(session, 0, allocator, &in_name), "SessionGetInputName")) { g_inited = -1; return -9; }
  if (check_status(ort->SessionGetOutputName(session, 0, allocator, &out_name), "SessionGetOutputName")) { g_inited = -1; return -10; }

  if (check_status(ort->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &meminfo), "CreateCpuMemoryInfo")) { g_inited = -1; return -11; }

  LOG_I(NR_PHY, "[DL_CSI_ONNX] INIT OK model=%s, expect input shape [1,%"PRId64",%"PRId64"]\n", model_path, gT, gD);
  LOG_I(NR_PHY, "[DL_CSI_ONNX] IO names: input=%s output=%s\n", in_name, out_name);

  g_inited = 1;
  return 0;
#endif
}


int run_dl_csi_onnx(const float *input, float *output)
{
  if (g_inited != 1) {
  LOG_E(NR_PHY, "[ONNX] run called but g_inited=%d (init not done/failed)\n", g_inited);
  return -1;
}

  if (!input || !output) return -2;

  const int64_t N = gT * gD;      // 입력 크기
  const int64_t N_out = gD;       // 출력 크기 (sequence-to-one 모델)

  // test mode (identity/zero)
  LOG_I(NR_PHY, "[ONNX] gT=%" PRId64 " gD=%" PRId64 "\n", gT, gD);
  LOG_I(NR_PHY, "[ONNX] input_name=%s\n", in_name);
  LOG_I(NR_PHY, "[ONNX] output_name=%s\n", out_name);

  if (g_use_test_mode) {
    if (!strcmp(g_test_mode, "identity")) {
      // 입력의 마지막 타임스텝만 복사
      memcpy(output, input + (gT - 1) * gD, sizeof(float) * (size_t)N_out);
      return 0;
    }
    if (!strcmp(g_test_mode, "zero")) {
      memset(output, 0, sizeof(float) * (size_t)N_out);
      return 0;
    }
    // unknown -> identity
    memcpy(output, input + (gT - 1) * gD, sizeof(float) * (size_t)N_out);
    return 0;
  }

#ifndef OAI_USE_ONNXRUNTIME
  return -3;
#else
  OrtValue *in_tensor = NULL;
  OrtValue *out_tensor = NULL;

  int64_t in_shape[3]  = {1, gT, gD};      // 입력: [batch, seq_len, features]
  int64_t out_shape[2] = {1, gD};         // 출력: [batch, features] (sequence-to-one)

  size_t in_bytes = (size_t)N * sizeof(float);
  size_t out_bytes = (size_t)N_out * sizeof(float);

  if (check_status(ort->CreateTensorWithDataAsOrtValue(meminfo, (void*)input, in_bytes,
                                                       in_shape, 3, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
                                                       &in_tensor), "CreateTensorWithDataAsOrtValue(input)")) {
    return -4;
  }

  if (check_status(ort->CreateTensorWithDataAsOrtValue(meminfo, (void*)output, out_bytes,
                                                       out_shape, 2, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
                                                       &out_tensor), "CreateTensorWithDataAsOrtValue(output)")) {
    ort->ReleaseValue(in_tensor);
    return -5;
  }

  const char *in_names[] = { in_name };
  const char *out_names[] = { out_name };
  OrtValue *in_vals[] = { in_tensor };
  OrtValue *out_vals[] = { out_tensor };

  OrtStatus *st = ort->Run(session, NULL,
                           in_names, (const OrtValue* const*)in_vals, 1,
                           out_names, 1, out_vals);
  if (check_status(st, "Run")) {
    ort->ReleaseValue(in_tensor);
    ort->ReleaseValue(out_tensor);
    return -6;
  }

  ort->ReleaseValue(in_tensor);
  ort->ReleaseValue(out_tensor);
  return 0;
#endif
}

void fini_dl_csi_onnx(void)
{
#ifdef OAI_USE_ONNXRUNTIME
  if (out_name && allocator) allocator->Free(allocator, out_name);
  if (in_name && allocator) allocator->Free(allocator, in_name);
  out_name = NULL; in_name = NULL;

  if (session) ort->ReleaseSession(session);
  if (sess_opts) ort->ReleaseSessionOptions(sess_opts);
  if (meminfo) ort->ReleaseMemoryInfo(meminfo);
  if (env) ort->ReleaseEnv(env);

  session = NULL; sess_opts = NULL; meminfo = NULL; env = NULL;
#endif
  g_inited = 0;
}

