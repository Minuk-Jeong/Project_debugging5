# OAI CSI-RS 기반 채널 추정 경로 분석

## [1] CSI-RS RE 위치 매핑

### (증거)
- File: `openair1/PHY/nr_phy_common/src/nr_phy_common_csirs.c`
- Function: `get_csi_mapping_parms`
- Lines: 207-562
- Snippet:
```c
csi_mapping_parms_t get_csi_mapping_parms(int row, int b, int l0, int l1)
{
  csi_mapping_parms_t csi_parms = {0};
  // row에 따라 ports, kprime, lprime, koverline[], loverline[] 결정
  // 예: row=4 -> ports=4, kprime=1, size=2
  // koverline[0] = k_n[0], koverline[1] = k_n[0] + 2
  // loverline[0] = l0, loverline[1] = l0
}
```
- What this proves: CSI-RS 설정(row, freq_domain bitmap, l0, l1)으로부터 매핑 파라미터 생성

### (증거)
- File: `openair1/PHY/nr_phy_common/src/nr_phy_common_csirs.c`
- Function: `csi_rs_resource_mapping`
- Lines: 24-86
- Snippet:
```c
static void csi_rs_resource_mapping(...)
{
  for (int n = start_rb; n < (start_rb + nb_rbs); n++) {
    if ((freq_density > 1) || (freq_density == (n % 2))) {  // freq density 0.5 체크
      for (int ji = 0; ji < mapping_parms->size; ji++) { // CDM groups
        for (int s = 0 ; s < gs; s++)  { // CDM group size
          int p = s + mapping_parms->j[ji] * gs; // port index
          for (int kp = 0; kp <= mapping_parms->kprime; kp++) {
            // frequency index: k = (start_sc + (n * NR_NB_SC_PER_RB) + mapping_parms->koverline[ji] + kp) % ofdm_symbol_size
            int k = (start_sc + (n * NR_NB_SC_PER_RB) + mapping_parms->koverline[ji] + kp) % (ofdm_symbol_size);
            for (int lp = 0; lp <= mapping_parms->lprime; lp++) {
              int l = lp + mapping_parms->loverline[ji];
              int index = (l * ofdm_symbol_size + k) + dataF_offset;
              dataF[p][index].r = ...; // CSI-RS 심볼 배치
            }
          }
        }
      }
    }
  }
}
```
- What this proves: CSI-RS가 RE 좌표(l, k)로 매핑됨. k는 FFT 기준 subcarrier index (0..ofdm_symbol_size-1)

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_get_csi_rs_signal`
- Lines: 594-699
- Snippet:
```c
for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {
  if(csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
    continue; // freq density 0.5인 경우 짝수/홀수 RB 체크
  }
  for (int cdm_id = 0; cdm_id < csi_mapping->size; cdm_id++) {
    for (int s = 0; s < CDM_group_size; s++)  {
      for (int kp = 0; kp <= csi_mapping->kprime; kp++) {
        uint16_t k = (fp->first_carrier_offset + (rb * NR_NB_SC_PER_RB) + 
                     csi_mapping->koverline[cdm_id] + kp) % fp->ofdm_symbol_size;
        for (int lp = 0; lp <= csi_mapping->lprime; lp++) {
          uint16_t symb = lp + csi_mapping->loverline[cdm_id];
          uint64_t symbol_offset = symb * fp->ofdm_symbol_size;
          const c16_t *rx_signal = &rxdataF[ant_rx][symbol_offset];
          rx_csi_rs_signal[k].r = rx_signal[k].r;
          rx_csi_rs_signal[k].i = rx_signal[k].i;
        }
      }
    }
  }
}
```
- What this proves: UE가 CSI-RS를 읽을 때도 동일한 (l, k) 좌표계 사용. k는 FFT 기준 subcarrier index

### (정리)
- CSI-RS 설정 파라미터: `fapi_nr_dl_config_csirs_pdu_rel15_t` 구조체에 저장 (`row`, `freq_domain`, `symb_l0`, `symb_l1`, `start_rb`, `nr_of_rbs`, `freq_density` 등)
- 설정 → 매핑 파라미터 변환: `get_csi_mapping_parms(row, freq_domain, l0, l1)` 호출로 `csi_mapping_parms_t` 생성
- RE 좌표 계산: `k = (first_carrier_offset + rb*12 + koverline[cdm_id] + kp) % ofdm_symbol_size`, `l = loverline[cdm_id] + lp`
- 인덱싱 방식: **FFT 기준 절대 subcarrier index** (0..ofdm_symbol_size-1). RB-상대 인덱스가 아님
- freq_density 처리: 0.5인 경우 `(rb % 2) == freq_density` 체크로 짝수/홀수 RB 필터링
- 포트별 RE: CDM group과 CDM group size로 포트 분리 (`port = s + j[cdm_id] * CDM_group_size`)

---

## [2] 채널 추정 범위 및 보간

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 717-894
- Snippet (LS 추정 단계):
```c
// LS channel estimation
for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {
  if(csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
    continue;
  }
  for (int cdm_id = 0; cdm_id < csi_mapping->size; cdm_id++) {
    for (int s = 0; s < CDM_group_size; s++)  {
      uint16_t port_tx = s + csi_mapping->j[cdm_id] * CDM_group_size;
      for (int kp = 0; kp <= csi_mapping->kprime; kp++) {
        uint16_t kinit = (fp->first_carrier_offset + rb*NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
        uint16_t k = kinit + csi_mapping->koverline[cdm_id] + kp;
        for (int lp = 0; lp <= csi_mapping->lprime; lp++) {
          uint16_t symb = lp + csi_mapping->loverline[cdm_id];
          uint64_t symbol_offset = symb * fp->ofdm_symbol_size;
          const c16_t *tx_csi_rs_signal = &csi_rs_generated_signal[port_tx][symbol_offset+dataF_offset];
          const c16_t *rx_csi_rs_signal = &csi_rs_received_signal[ant_rx][symbol_offset];
          c16_t tmp = c16MulConjShift(tx_csi_rs_signal[k], rx_csi_rs_signal[k], ...);
          // RB당 CSI-RS RE들을 합산 (최적화 목적)
          csi_rs_ls_estimated_channel[ant_rx][port_tx][kinit].r += tmp.r;
          csi_rs_ls_estimated_channel[ant_rx][port_tx][kinit].i += tmp.i;
        }
      }
    }
  }
}
```
- What this proves: LS 추정은 **CSI-RS RE 위치에서만** 수행. `csi_rs_ls_estimated_channel`은 RB당 하나의 값만 저장 (인덱스 kinit = RB의 첫 subcarrier)

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 808-837 (보간 단계)
- Snippet:
```c
/// Channel interpolation
for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
  memset(csi_rs_estimated_channel_freq[ant_rx][port_tx], 0, (fp->ofdm_symbol_size + FILTER_MARGIN) * sizeof(c16_t));
}

for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {
  if(csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
    continue;
  }
  uint16_t k = (fp->first_carrier_offset + rb * NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
  uint16_t k_offset = k + mem_offset;
  for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
    c16_t csi_rs_ls_estimated_channel16 = csi_rs_ls_estimated_channel[ant_rx][port_tx][k];
    c16_t *csi_rs_estimated_channel16 = &csi_rs_estimated_channel_freq[ant_rx][port_tx][k_offset];
    if( (k == 0) || (k == fp->first_carrier_offset) ) { // Start case
      multadd_real_vector_complex_scalar(filt24_start, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16, 24);
    } else if(((k + NR_NB_SC_PER_RB) >= fp->ofdm_symbol_size) ||
               (rb == (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs-1))) { // End case
      multadd_real_vector_complex_scalar(filt24_end, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16 - 12, 24);
    } else { // Middle case
      multadd_real_vector_complex_scalar(filt24_middle, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16 - 12, 24);
    }
  }
}
```
- What this proves: **freq interpolation으로 전체 subcarrier로 확장**. filt24 계수로 24-tap 보간. 출력은 `csi_rs_estimated_channel_freq[ant_rx][port][ofdm_symbol_size + FILTER_MARGIN]`

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_ue_csi_rs_procedures`
- Lines: 1989-1997
- Snippet:
```c
c16_t csi_rs_ls_estimated_channel[frame_parms->nb_antennas_rx][mapping_parms.ports][frame_parms->ofdm_symbol_size];
c16_t csi_rs_estimated_channel_freq[frame_parms->nb_antennas_rx][mapping_parms.ports]
                                   [frame_parms->ofdm_symbol_size + FILTER_MARGIN];
```
- What this proves: LS 추정 결과는 `ofdm_symbol_size` 크기, 보간 후 결과는 `ofdm_symbol_size + FILTER_MARGIN` 크기

### (정리)
- LS 추정 범위: **CSI-RS RE 위치에서만** 수행. RB당 하나의 LS 추정값 (RB의 첫 subcarrier 인덱스에 저장)
- 보간 방식: **freq interpolation (A형태)**. filt24 계수 사용 (24-tap filter). RB 단위로 보간하여 전체 RB의 12개 subcarrier에 확장
- 최종 채널 추정 범위: **전체 ofdm_symbol_size**로 확장됨 (CSI-RS가 배치된 RB 범위 내)
- Time averaging: 코드상 명시적인 time averaging 없음. 각 슬롯/심볼에서 독립적으로 추정
- 루프 범위: 보간 단계는 **CSI-RS RB 개수만큼** 루프. 각 RB에서 filt24로 12개 subcarrier로 확장

---

## [3] 채널 추정 알고리즘

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 770-775
- Snippet:
```c
const c16_t *tx_csi_rs_signal = &csi_rs_generated_signal[port_tx][symbol_offset+dataF_offset];
const c16_t *rx_csi_rs_signal = &csi_rs_received_signal[ant_rx][symbol_offset];
c16_t tmp = c16MulConjShift(tx_csi_rs_signal[k], rx_csi_rs_signal[k], nr_csi_info->csi_rs_generated_signal_bits);
// This is not just the LS estimation for each (k,l), but also the sum of the different contributions
// for the sake of optimizing the memory used.
csi_rs_ls_estimated_channel[ant_rx][port_tx][kinit].r += tmp.r;
csi_rs_ls_estimated_channel[ant_rx][port_tx][kinit].i += tmp.i;
```
- What this proves: **LS 추정 (Least Squares)**. H_est = conj(tx) * rx / |tx|^2. c16MulConjShift가 conj(tx)*rx 연산 수행

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 838-859 (노이즈 파워 추정)
- Snippet:
```c
/// Power noise estimation
uint16_t noise_real[fp->nb_antennas_rx][csi_mapping->ports][csirs_config_pdu->nr_of_rbs];
uint16_t noise_imag[fp->nb_antennas_rx][csi_mapping->ports][csirs_config_pdu->nr_of_rbs];
for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {
  if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
    continue;
  }
  uint16_t k = (fp->first_carrier_offset + rb*NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
  uint16_t k_offset = k + mem_offset;
  for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
    c16_t *csi_rs_ls_estimated_channel16 = &csi_rs_ls_estimated_channel[ant_rx][port_tx][k];
    c16_t *csi_rs_estimated_channel16 = &csi_rs_estimated_channel_freq[ant_rx][port_tx][k_offset];
    noise_real[ant_rx][port_tx][rb-csirs_config_pdu->start_rb] = abs(csi_rs_ls_estimated_channel16->r-csi_rs_estimated_channel16->r);
    noise_imag[ant_rx][port_tx][rb-csirs_config_pdu->start_rb] = abs(csi_rs_ls_estimated_channel16->i-csi_rs_estimated_channel16->i);
  }
}
*noise_power /= (fp->nb_antennas_rx * csi_mapping->ports);
```
- What this proves: 노이즈 파워는 LS 추정값과 보간값의 차이로 계산. **MMSE/LMMSE 사용 안 함**

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 738-744
- Snippet:
```c
for (int ant_rx = 0; ant_rx < fp->nb_antennas_rx; ant_rx++) {
  /// LS channel estimation
  for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
    memset(csi_rs_ls_estimated_channel[ant_rx][port_tx], 0, fp->ofdm_symbol_size * sizeof(c16_t));
  }
  // ... LS 추정 루프 ...
}
```
- What this proves: **Rx antenna와 Tx port에 대한 이중 루프**로 채널 행렬 구성

### (정리)
- 추정 알고리즘: **LS (Least Squares)**. H_est = conj(tx) * rx / normalization
- MMSE/LMMSE: 사용 안 함. noise variance는 노이즈 추정에만 사용
- 포트/Rx 처리: 이중 루프 `for(ant_rx) for(port_tx)`. 채널 행렬 H[ant_rx][port_tx]
- CDM 처리: CDM group 내 여러 RE를 합산하여 최적화 (메모리 절약 목적)

---

## [4] 채널 추정 출력 차원/개수

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_ue_csi_rs_procedures`
- Lines: 1989-1990
- Snippet:
```c
c16_t csi_rs_ls_estimated_channel[frame_parms->nb_antennas_rx][mapping_parms.ports][frame_parms->ofdm_symbol_size];
c16_t csi_rs_estimated_channel_freq[frame_parms->nb_antennas_rx][mapping_parms.ports]
                                   [frame_parms->ofdm_symbol_size + FILTER_MARGIN];
```
- What this proves: 최종 출력은 `csi_rs_estimated_channel_freq[nb_antennas_rx][ports][ofdm_symbol_size + FILTER_MARGIN]`

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 827-834 (보간 출력 위치)
- Snippet:
```c
uint16_t k = (fp->first_carrier_offset + rb * NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
uint16_t k_offset = k + mem_offset;
c16_t *csi_rs_estimated_channel16 = &csi_rs_estimated_channel_freq[ant_rx][port_tx][k_offset];
```
- What this proves: 인덱스는 `[ant_rx][port_tx][k_offset]`. k_offset은 k + mem_offset (메모리 정렬용)

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_ue_csi_rs_procedures`
- Lines: 2023-2037
- Snippet:
```c
if (csirs_config_pdu->measurement_bitmap > 1)
  nr_csi_rs_channel_estimation(frame_parms, proc, csirs_config_pdu, csi_info,
                               (const c16_t **)csi_info->csi_rs_generated_signal,
                               csi_rs_received_signal, &mapping_parms, CDM_group_size,
                               mem_offset, csi_rs_ls_estimated_channel,
                               csi_rs_estimated_channel_freq, &log2_re, &log2_maxh, &noise_power);
```
- What this proves: 채널 추정은 **슬롯당 한 번** 호출됨 (CSI-RS가 있는 슬롯에서)

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_pmi_estimation`, `nr_csi_rs_ri_estimation`
- Lines: 1042, 1086, 1140-1141, 1272-1275 등
- Snippet:
```c
// 2-port 예시
const c16_t p0 = csi_rs_estimated_channel_freq[ant_rx][0][k_offset];
const c16_t p1 = csi_rs_estimated_channel_freq[ant_rx][1][k_offset];

// 4-port 예시
const c16_t h0 = csi_rs_estimated_channel_freq[ant_rx][0][k_offset]; // H[ant_rx][port0]
const c16_t h1 = csi_rs_estimated_channel_freq[ant_rx][1][k_offset]; // H[ant_rx][port1]
const c16_t h2 = csi_rs_estimated_channel_freq[ant_rx][2][k_offset]; // H[ant_rx][port2]
const c16_t h3 = csi_rs_estimated_channel_freq[ant_rx][3][k_offset]; // H[ant_rx][port3]
```
- What this proves: 채널 추정 결과는 `H[ant_rx][port_tx][subcarrier]` 형태로 접근됨

### (정리)
- 출력 변수명: `csi_rs_estimated_channel_freq`
- 타입: `c16_t` (complex16, Q15 고정소수점)
- 할당 크기: `[nb_antennas_rx][ports][ofdm_symbol_size + FILTER_MARGIN]`
- 차원 명시: **H[ant_rx][port_tx][k]** 형태
  - ant_rx: Rx antenna 인덱스 (0..nb_antennas_rx-1)
  - port_tx: CSI-RS Tx port 인덱스 (0..ports-1)
  - k: subcarrier 인덱스 (0..ofdm_symbol_size-1, 메모리 정렬로 인해 k_offset = k + mem_offset 사용)
- 호출 빈도: **슬롯당 한 번** (CSI-RS가 있는 슬롯에서)
- 4x4 시스템 예시: `H[4][4][ofdm_symbol_size]` = 16개 채널 경로 (4 Rx × 4 Tx ports) × 전체 subcarrier

---

---

## [체크 1] c16MulConjShift() 구현 및 정규화

### (증거)
- File: `openair1/PHY/TOOLS/tools_defs.h`
- Function: `c16MulConjShift` (inline 함수)
- Lines: 190-196
- Snippet:
```c
__attribute__((always_inline)) inline c16_t c16MulConjShift(const c16_t a, const c16_t b, const int Shift)
{
  return (c16_t) {
    .r = (int16_t)((a.r * b.r + a.i * b.i) >> Shift),
    .i = (int16_t)((a.r * b.i - a.i * b.r) >> Shift)
  };
}
```
- What this proves: **`c16MulConjShift(a, b, Shift)` = `a * conj(b)` >> Shift**. a가 tx, b가 rx인 경우: `tx * conj(rx)` >> Shift (즉, `conj(tx) * rx`와 동일한 결과). |tx|^2로 나누지 않고 Shift만 적용.

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_ue_csi_rs_procedures`
- Lines: 1987
- Snippet:
```c
csi_info->csi_rs_generated_signal_bits = log2_approx(AMP);
```
- What this proves: `csi_rs_generated_signal_bits`는 전송 CSI-RS 신호의 진폭(AMP)의 log2 근사값. Shift 인자로 사용됨.

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 770
- Snippet:
```c
c16_t tmp = c16MulConjShift(tx_csi_rs_signal[k], rx_csi_rs_signal[k], nr_csi_info->csi_rs_generated_signal_bits);
```
- What this proves: `c16MulConjShift(tx, rx, shift)`는 `tx * conj(rx)` >> shift 계산. CSI-RS의 경우 tx 진폭에 맞춰 정규화됨.

---

## [체크 2] RB당 합산값의 평균화/정규화

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 771-774
- Snippet:
```c
c16_t tmp = c16MulConjShift(tx_csi_rs_signal[k], rx_csi_rs_signal[k], nr_csi_info->csi_rs_generated_signal_bits);
// This is not just the LS estimation for each (k,l), but also the sum of the different contributions
// for the sake of optimizing the memory used.
csi_rs_ls_estimated_channel[ant_rx][port_tx][kinit].r += tmp.r;
csi_rs_ls_estimated_channel[ant_rx][port_tx][kinit].i += tmp.i;
```
- What this proves: **RB당 여러 CSI-RS RE 값을 누적(sum)만 함. 나누기/평균화 없음.**

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 887-888
- Snippet:
```c
*log2_maxh = log2_approx(maxh - 1);
*log2_re = log2_approx(count - 1);
```
- What this proves: `log2_re`는 유효 RB 개수의 log2, `log2_maxh`는 채널 추정 최대값의 log2. PMI/RI 계산에서 정규화에 사용 (예: 473-474 라인에서 `>> log2_maxh` 적용).

---

## [체크 3] filt24 필터의 의미와 적용 범위

### (증거)
- File: `openair1/PHY/NR_UE_ESTIMATION/filt16a_32.h`
- Function: (정적 배열 정의)
- Lines: 232-239
- Snippet:
```c
static const short filt24_start[24] = {12288, 11605, 10923, 10240, 9557, 8875, 8192, 7509, 6827, 6144, 5461, 4779,
                                      0,     0,     0,     0,     0,    0,    0,    0,    0,    0,    0,    0};

static const short filt24_end[24] = {4096,  4779,  5461,  6144,  6827,  7509,  8192,  8875,  9557,  10240, 10923, 11605,
                                    16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384};

static const short filt24_middle[24] = {4096,  4779,  5461,  6144,  6827, 7509, 8192, 8875, 9557, 10240, 10923, 11605,
                                       12288, 11605, 10923, 10240, 9557, 8875, 8192, 7509, 6827, 6144,  5461,  4779};
```
- What this proves: filt24는 24-tap 필터. start는 앞 12개만 유효(뒤 12개는 0), end는 전체 24개 유효, middle은 대칭 형태. 계수 값은 Q14 형식 (16384 = 1.0, 8192 = 0.5).

### (증거)
- File: `openair1/PHY/TOOLS/tools_defs.h`
- Function: `multadd_real_vector_complex_scalar`
- Lines: 295-309
- Snippet:
```c
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
- What this proves: `multadd_real_vector_complex_scalar(x, alpha, y, N)`는 `y += alpha * x` (벡터 스칼라 곱셈-누적). N=24이면 y의 연속 24개 샘플에 누적.

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 824-835
- Snippet:
```c
uint16_t k = (fp->first_carrier_offset + rb * NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
uint16_t k_offset = k + mem_offset;
for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
  c16_t csi_rs_ls_estimated_channel16 = csi_rs_ls_estimated_channel[ant_rx][port_tx][k];
  c16_t *csi_rs_estimated_channel16 = &csi_rs_estimated_channel_freq[ant_rx][port_tx][k_offset];
  if( (k == 0) || (k == fp->first_carrier_offset) ) { // Start case
    multadd_real_vector_complex_scalar(filt24_start, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16, 24);
  } else if(((k + NR_NB_SC_PER_RB) >= fp->ofdm_symbol_size) ||
             (rb == (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs-1))) { // End case
    multadd_real_vector_complex_scalar(filt24_end, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16 - 12, 24);
  } else { // Middle case
    multadd_real_vector_complex_scalar(filt24_middle, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16 - 12, 24);
  }
}
```
- What this proves: **k_offset - 12 시작점으로 24-tap 필터 적용 → RB 내부 12개 subcarrier 채움** (k_offset-12부터 k_offset+11까지). start는 k_offset부터 시작, middle/end는 k_offset-12부터 시작.

### (증거)
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_ue_csi_rs_procedures`
- Lines: 45, 1993-1997
- Snippet:
```c
#define FILTER_MARGIN 32
// ...
// (long)&csi_rs_estimated_channel_freq[0][0][frame_parms->first_carrier_offset] & 0x1F
// gives us the remainder of the integer division by 32 of the memory address
// By subtracting the previous value of 32, we know how much is left to have a multiple of 32.
// Doing >> 2 <=> /sizeof(int32_t), we know what is the index offset of the array.
uint8_t mem_offset = (((32 - ((long)&csi_rs_estimated_channel_freq[0][0][frame_parms->first_carrier_offset])) & 0x1F) >> 2);
```
- What this proves: **FILTER_MARGIN=32는 보간 필터 적용을 위한 여유 공간**. mem_offset은 메모리 32바이트 정렬을 위한 인덱스 오프셋 (first_carrier_offset 위치를 32바이트 경계에 맞춤).

---

## 요약

1. **CSI-RS RE 위치**: FFT 기준 절대 subcarrier index (0..ofdm_symbol_size-1)로 표현
2. **채널 추정 범위**: LS 추정은 CSI-RS RE 위치에서만 → freq interpolation으로 전체 subcarrier로 확장
3. **추정 알고리즘**: LS (Least Squares). MMSE 미사용
4. **출력 차원**: `H[ant_rx][port_tx][k]` (c16_t 타입, Q15)
5. **c16MulConjShift**: `tx * conj(rx)` >> shift (|tx|^2 정규화 없음)
6. **RB당 합산**: 평균화 없이 누적만 수행. log2_re/log2_maxh는 후처리 정규화용
7. **filt24 보간**: 24-tap 필터로 RB 내부 12개 subcarrier 채움. FILTER_MARGIN=32는 경계 처리용
