# CSI-RS 관련 코드 근거 정리 [1]–[5]

---

## [1] freq_density의 의미 및 세팅

### (증거)
- File: `nfapi/open-nFAPI/nfapi/public_inc/fapi_nr_ue_interface.h`
- Function: (구조체 정의)
- Lines: 458-466
- Snippet:
```c
  uint8_t freq_density;             // The density field, p and comb offset (for dot5). [3GPP TS 38.211, sec 7.4.1.5.3 and table 7.4.1.5.3-1], Value: 0: dot5 (even RB); 1: dot5 (odd RB); 2: one; 3: three
  uint16_t scramb_id;
  ...
} fapi_nr_dl_config_csirs_pdu_rel15_t;
```
- What this proves: `freq_density`는 `uint8_t`. 0=dot5 even RB, 1=dot5 odd RB, 2=one, 3=three.

### (증거)
- File: `openair2/LAYER2/NR_MAC_UE/nr_ue_scheduler.c`
- Function: `configure_csi_resource_mapping`
- Lines: 1964-1969
- Snippet:
```c
  csirs_config_pdu->freq_density = resourceMapping->density.present;
  if ((resourceMapping->density.present == NR_CSI_RS_ResourceMapping__density_PR_dot5)
      && (resourceMapping->density.choice.dot5 == NR_CSI_RS_ResourceMapping__density__dot5_evenPRBs))
    csirs_config_pdu->freq_density--;
```
- What this proves: RRC `density.present`로 먼저 세팅. dot5+evenPRBs면 `freq_density--` → dot5 even→0, dot5 odd→1, one→2, three→3.

### (증거)
- File: `openair1/PHY/nr_phy_common/src/nr_phy_common_csirs.c`
- Function: `csi_rs_resource_mapping`
- Lines: 39-42
- Snippet:
```c
  for (int n = start_rb; n < (start_rb + nb_rbs); n++) {
    if ((freq_density > 1) || (freq_density == (n % 2))) {  // for freq density 0.5 checks if even or odd RB
      ...
    }
  }
```
- What this proves: `freq_density<=1`이면 `n%2`와 비교. 0이면 짝수 RB만, 1이면 홀수 RB만; `>1`이면 모든 RB.

### (정리)
- `freq_density` 타입: `uint8_t`, `fapi_nr_dl_config_csirs_pdu_rel15_t` 내 필드.
- 설정 경로: RRC `NR_CSI_RS_ResourceMapping.density` → `configure_csi_resource_mapping`에서 `freq_density` 설정. dot5+evenPRBs일 때만 `--` 적용.
- 값 체계: 0=dot5 even, 1=dot5 odd, 2=one, 3=three.
- `<=1`일 때 `rb%2` 비교: 0→even RB만, 1→odd RB만. `nr_phy_common_csirs.c` 및 `csi_rx.c` 전반에서 동일 조건 사용.

---

## [2] 24-tap 주파수 필터의 동작

### (증거)
- File: `openair1/PHY/NR_UE_ESTIMATION/filt16a_32.h`
- Function: (정적 배열)
- Lines: 231-239
- Snippet:
```c
static const short filt24_start[24] = {12288, 11605, 10923, 10240, 9557, 8875, 8192, 7509, 6827, 6144, 5461, 4779,
                                      0,     0,     0,     0,     0,    0,    0,    0,    0,    0,    0,    0};
static const short filt24_end[24] = {4096,  4779,  5461,  6144,  6827,  7509,  8192,  8875,  9557,  10240, 10923, 11605,
                                    16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384};
static const short filt24_middle[24] = {4096,  4779,  5461,  6144,  6827, 7509, 8192, 8875, 9557, 10240, 10923, 11605,
                                       12288, 11605, 10923, 10240, 9557, 8875, 8192, 7509, 6827, 6144,  5461,  4779};
```
- What this proves: 24-tap. start: 앞 12개만 비영, 뒤 12개 0. end: 12+12. middle: 대칭. 16384 등으로 1.0 스케일(필터 헤더/다른 filt 계수와 동일 패턴).

### (증거)
- File: `openair1/PHY/TOOLS/tools_defs.h`
- Function: `multadd_real_vector_complex_scalar`
- Lines: 285-311
- Snippet:
```c
/*!\fn ...
The function implemented is : \f$\mathbf{y} = y + \alpha\mathbf{x}\f$
*/
  static inline void multadd_real_vector_complex_scalar(const int16_t *x, const c16_t alpha, c16_t *y, const uint32_t N)
  {
    simd_q15_t *x_128 = (simd_q15_t *)x, *y_128 = (simd_q15_t *)y;
    const simd_q15_t alpha_r_128 = set1_int16(alpha.r);
    const simd_q15_t alpha_i_128 = set1_int16(alpha.i);
    for (uint32_t i = 0; i < N >> 3; i++) {
      const simd_q15_t yr = mulhi_s1_int16(alpha_r_128, x_128[i]);
      const simd_q15_t yi = mulhi_s1_int16(alpha_i_128, x_128[i]);
      const simd_q15_t tmp = simde_mm_loadu_si128(y_128);
      simde_mm_storeu_si128(y_128++, simde_mm_adds_epi16(tmp, simde_mm_unpacklo_epi16(yr, yi)));
      const simd_q15_t tmp2 = simde_mm_loadu_si128(y_128);
      simde_mm_storeu_si128(y_128++, simde_mm_adds_epi16(tmp2, simde_mm_unpackhi_epi16(yr, yi)));
    }
  }
```
- What this proves: `y += alpha * x`. x는 실수 벡터, alpha는 복소; 연속 N개(y)에 누적. N=24면 `y[0..23]` 갱신.

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 823-836
- Snippet:
```c
      uint16_t k = (fp->first_carrier_offset + rb * NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;
      for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
        c16_t csi_rs_ls_estimated_channel16 = csi_rs_ls_estimated_channel[ant_rx][port_tx][k];
        c16_t *csi_rs_estimated_channel16 = &csi_rs_estimated_channel_freq[ant_rx][port_tx][k_offset];
        if( (k == 0) || (k == fp->first_carrier_offset) ) {
          multadd_real_vector_complex_scalar(filt24_start, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16, 24);
        } else if(((k + NR_NB_SC_PER_RB) >= fp->ofdm_symbol_size) ||
                   (rb == (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs-1))) {
          multadd_real_vector_complex_scalar(filt24_end, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16 - 12, 24);
        } else {
          multadd_real_vector_complex_scalar(filt24_middle, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16 - 12, 24);
        }
      }
```
- What this proves: start는 `out_ptr = &...[k_offset]`로 `[k_offset..k_offset+23]` 갱신(실제 non-zero는 앞 12탭). middle/end는 `out_ptr = &...[k_offset-12]`로 `[k_offset-12..k_offset+11]` 갱신. 즉 RB 기준 k 주변 ±12 sc 구간이 필터로 채워짐.

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: (상단), `nr_ue_csi_rs_procedures`
- Lines: 44-45, 1991-1997
- Snippet:
```c
#define FILTER_MARGIN 32
// ...
  c16_t csi_rs_estimated_channel_freq[frame_parms->nb_antennas_rx][mapping_parms.ports]
                                     [frame_parms->ofdm_symbol_size + FILTER_MARGIN];
  // (long)&csi_rs_estimated_channel_freq[0][0][frame_parms->first_carrier_offset] & 0x1F ...
  uint8_t mem_offset = (((32 - ((long)&csi_rs_estimated_channel_freq[0][0][frame_parms->first_carrier_offset])) & 0x1F) >> 2);
```
- What this proves: `ofdm_symbol_size + FILTER_MARGIN`은 필터/정렬용 여유. `mem_offset`은 `first_carrier_offset`을 32B 정렬하기 위한 인덱스 오프셋(주석대로 `>>2` ⇔ `/sizeof(int32_t)`).

### (정리)
- filt24_*: 24-tap, `filt16a_32.h` 정의. 16384=1.0 스케일 등 동일 계열.
- `multadd_real_vector_complex_scalar(x, alpha, y, N)`: `y += alpha*x`, 연속 N 복소 샘플 갱신.
- 갱신 범위: start → `[k_offset..k_offset+11]`(뒤 12탭 0). middle/end → `[k_offset-12..k_offset+11]`.
- `-12` 이유: RB 첫 subcarrier k 기준, 왼쪽 12 sc까지 확장하기 위함.
- 연속 RB: RB i는 `[ki-12..ki+11]`, RB i+1은 `[ki+12-12..ki+12+11]` → `[ki..ki+23]` 중 `[ki..ki+11]` 구간 겹침. 동일 y에 **누적**되므로 한 subcarrier가 여러 RB seed의 합으로 채워질 수 있음.
- `FILTER_MARGIN`=32: 보간/정렬 여유. `mem_offset`: 32B 정렬용 인덱스 보정.

---

## [3] 보간 후 “모든 subcarrier” 채움 여부

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 808-812
- Snippet:
```c
    /// Channel interpolation
    for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
      memset(csi_rs_estimated_channel_freq[ant_rx][port_tx], 0, (fp->ofdm_symbol_size + FILTER_MARGIN) * sizeof(c16_t));
    }
```
- What this proves: 보간 전 전체 `ofdm_symbol_size + FILTER_MARGIN`을 0으로 초기화.

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 814-819
- Snippet:
```c
    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {
      // for freq density 0.5 checks if even or odd RB
      if(csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      ...
    }
```
- What this proves: 보간 루프는 `start_rb`~`start_rb+nr_of_rbs-1` RB만 순회. freq_density 0.5일 때 `rb%2` 불일치 RB는 스킵.

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 823-824
- Snippet:
```c
      uint16_t k = (fp->first_carrier_offset + rb * NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;
```
- What this proves: 갱신되는 k는 `first_carrier_offset + rb*12`에 대응. RB 루프 밖의 k는 루프에 없으므로 갱신되지 않음.

### (정리)
- memset(0): 보간 전 `[ant_rx][port_tx][0..ofdm_symbol_size+FILTER_MARGIN-1]` 전부 0.
- 보간 루프: `start_rb`~`start_rb+nr_of_rbs-1` RB만, density 조건 통과한 RB에 대해 `k = first_carrier_offset + rb*12` 주변으로 filt24 적용.
- **채워지는 k 구간**: CSI-RS RB 대역 내, 각 RB의 k 주변 `k_offset±12` (또는 start 시 `k_offset..k_offset+11`)가 **누적**되어 채워짐. 연속 RB가 12 subcarrier 간격이므로, RB i의 24-tap은 `[ki-12..ki+11]`, RB i+2는 `[ki+12..ki+35]` 등. **density=0.5로 스킵된 RB의 k**(예: RB i+1의 `ki+12..ki+23`)는 **이웃 RB(i, i+2)의 24-tap이 겹치는 구간**이므로, 그 구간이 채워짐.
- **RB 범위 밖 k**: 루프에 없음 → 갱신 없음 → **0 유지**.
- **전체 FFT bin이 항상 유효**인 것은 아님. `first_carrier_offset`~`start_rb*12` 미만, 및 `(start_rb+nr_of_rbs)*12` 초과 등 RB 밖 구간은 0.

---

## [4] subcarrier당 16개 채널(4×4) 여부

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_ue_csi_rs_procedures`
- Lines: 1989-1991
- Snippet:
```c
  c16_t csi_rs_ls_estimated_channel[frame_parms->nb_antennas_rx][mapping_parms.ports][frame_parms->ofdm_symbol_size];
  c16_t csi_rs_estimated_channel_freq[frame_parms->nb_antennas_rx][mapping_parms.ports]
                                     [frame_parms->ofdm_symbol_size + FILTER_MARGIN];
```
- What this proves: `csi_rs_estimated_channel_freq` 차원 `[nb_antennas_rx][ports][ofdm_symbol_size + FILTER_MARGIN]`. subcarrier k마다 `(nb_antennas_rx × ports)`개 복소값.

### (증거)
- File: `openair1/PHY/defs_nr_common.h` / `openair1/PHY/defs_nr_UE.h`
- Function: (구조체)
- Lines: 251, 214
- Snippet:
```c
  uint8_t nb_antennas_rx;  // NR_DL_FRAME_PARMS
  unsigned char  nb_antennas_rx;  // PHY_VARS_NR_UE
```
- What this proves: `nb_antennas_rx`는 프레임 파라미터/UE 설정에서 옴.

### (증거)
- File: `openair1/PHY/nr_phy_common/src/nr_phy_common_csirs.c`
- Function: `get_csi_mapping_parms`
- Lines: 274-291
- Snippet:
```c
    case 4:
      csi_parms.ports = 4;
      csi_parms.kprime = 1;
      csi_parms.lprime = 0;
      csi_parms.size = 2;
      ...
      break;
```
- What this proves: row=4 → `ports=4`. (row 3→2, 5→4 등 동일 테이블.)

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_pmi_estimation`
- Lines: 1269-1276
- Snippet:
```c
      // Get channel matrix H for this subcarrier: H is nb_antennas_rx x N_ports
      for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
        const c16_t h0 = csi_rs_estimated_channel_freq[ant_rx][0][k_offset]; // H[ant_rx][port0]
        const c16_t h1 = csi_rs_estimated_channel_freq[ant_rx][1][k_offset];
        const c16_t h2 = csi_rs_estimated_channel_freq[ant_rx][2][k_offset];
        const c16_t h3 = csi_rs_estimated_channel_freq[ant_rx][3][k_offset];
        ...
      }
```
- What this proves: PMI에서 `[ant_rx][port_tx][k_offset]`로 접근. 4×4이면 16개 복소 채널.

### (정리)
- `csi_rs_estimated_channel_freq[ant_rx][port_tx][k]`: `ant_rx`=0..nb_antennas_rx-1, `port_tx`=0..ports-1, `k`=0..ofdm_symbol_size+FILTER_MARGIN-1.
- subcarrier k마다 `nb_antennas_rx × ports`개 복소 채널 존재.
- `nb_antennas_rx`: `NR_DL_FRAME_PARMS`/UE 설정. `ports`: `get_csi_mapping_parms(row,...)`에서 row로 결정(예: row=4 → 4).
- 4×4 조건: `nb_antennas_rx==4` 이고 row→ports=4. PMI/RI에서 `[ant_rx][0..3][k_offset]` 접근으로 16개 사용.

---

## [5] CSI prediction(ONNX) 적용 방식

### (증거)
- File: `openair1/PHY/NR_UE_ESTIMATION/dl_csi_onnx.c`
- Function: `init_dl_csi_onnx`, `run_dl_csi_onnx`
- Lines: 55-58, 120-126, 148-151
- Snippet:
```c
int init_dl_csi_onnx(const char *model_path, int64_t T, int64_t D)
{
  gT = T;
  gD = D;
  ...
  LOG_I(NR_PHY, "[DL_CSI_ONNX] INIT OK model=%s, expect input shape [1,%"PRId64",%"PRId64"]\n", model_path, gT, gD);
```
```c
int run_dl_csi_onnx(const float *input, float *output)
{
  ...
  const int64_t N = gT * gD;      // 입력 크기
  const int64_t N_out = gD;       // 출력 크기 (sequence-to-one 모델)
  ...
  int64_t in_shape[3]  = {1, gT, gD};      // 입력: [batch, seq_len, features]
  int64_t out_shape[2] = {1, gD};         // 출력: [batch, features] (sequence-to-one)
```
- What this proves: 입력 `[1, T, D]`, 출력 `[1, D]`. sequence-to-one. `(ant_rx, port)` 인자 없음.

### (증거)
- File: `openair1/PHY/NR_UE_ESTIMATION/dl_csi_onnx.c`, `dl_csi_onnx.h`
- Function: N/A (검색 결과)
- Lines: 선언/정의부만 존재
- Snippet: `init_dl_csi_onnx`, `run_dl_csi_onnx`, `fini_dl_csi_onnx`, `#include "dl_csi_onnx.h"` 검색 시 **dl_csi_onnx.c / .h 내부에만 등장**. `csi_rx.c`, `nr_dl_channel_estimation.c`, `nr_init_ue.c` 등 PHY 경로 어디서도 참조 없음.
- What this proves: **CSI-RS 채널 추정 경로에서 ONNX predictor 호출 없음.** `init`/`run`/`fini` 사용처 없음.

### (증거)
- File: `openair1/PHY/NR_UE_ESTIMATION/dl_csi_onnx.c`
- Function: `run_dl_csi_onnx`
- Lines: 128-141
- Snippet:
```c
  if (g_use_test_mode) {
    if (!strcmp(g_test_mode, "identity")) {
      memcpy(output, input + (gT - 1) * gD, sizeof(float) * (size_t)N_out);
      return 0;
    }
    if (!strcmp(g_test_mode, "zero")) {
      memset(output, 0, sizeof(float) * (size_t)N_out);
      return 0;
    }
    memcpy(output, input + (gT - 1) * gD, sizeof(float) * (size_t)N_out);
    return 0;
  }
```
- What this proves: 테스트 모드에서도 input/output은 `float` 연속 버퍼. `D`차원 단일 벡터 in/out. `(ant_rx,port)` 루프나 16채널 배치 없음.

### (정리)
- predictor 위치: `dl_csi_onnx.c` / `dl_csi_onnx.h`. `OAI_USE_ONNXRUNTIME` 정의 시 빌드, `OAI_DL_CSI_ONNX_TEST_MODE` 등 env 사용.
- **현재 OAI 코드상 ONNX CSI predictor는 CSI-RS 채널 추정/PMI/RI 경로에 연결되어 있지 않음.** `init_dl_csi_onnx`/`run_dl_csi_onnx`/`fini_dl_csi_onnx`를 호출하는 코드 없음.
- API: `run_dl_csi_onnx(input, output)` — input `[1,T,D]`, output `[1,D]`. 단일 시퀀스→단일 벡터. `(ant_rx, port)` 선택 인자 없음.
- 입력/출력이 `csi_rs_estimated_channel_freq`의 어떤 slice로 채워지거나 덮어쓰이는 **연결 코드 없음**.
- **16개 전부 예측**하려면, 호출부에서 `(ant_rx, port)` 루프를 도는 등 별도 연동이 필요함. 현재 구현만으로는 **적용 자체가 안 됨**이며, 적용되더라도 API는 1회 호출당 `D`차원 1벡터이므로, 16채널 전체를 쓰려면 16회 호출 또는 입력/출력 layout 확장이 필요함.
