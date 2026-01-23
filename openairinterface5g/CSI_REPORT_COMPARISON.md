# OAI CSI Report 코드 비교 분석

## 개요
본 문서는 두 OAI 코드베이스의 CSI report 관련 코드를 비교 분석한 결과입니다.
- **기준 코드베이스**: `/home/lab/바탕화면/Project_CSIplusMatlab/openairinterface5g` (표준 OAI)
- **비교 코드베이스**: `/home/lab/바탕화면/Project_잘됨_RTD/openairinterface5g` (RTD 패치 적용)

---

## [1] CSI Report 생성 경로 비교

### (증거) - 표준 OAI
- File: `openair1/PHY/NR_UE_TRANSPORT/csi_rx.c`
- Function: `nr_csi_rs_channel_estimation`
- Lines: 2110-2131
- Snippet:
```c
// Send CSI measurements to MAC
if (!ue->if_inst || !ue->if_inst->dl_indication)
  return;

fapi_nr_l1_measurements_t l1_measurements = {
  .gNB_index = proc->gNB_id,
  .meas_type = NFAPI_NR_CSI_MEAS,
  .Nid_cell = frame_parms->Nid_cell,
  .is_neighboring_cell = false,
  .rsrp_dBm = rsrp_dBm,
  .rank_indicator = rank_indicator,
  .i1 = *i1,
  .i2 = *i2,
  .cqi = cqi,
  .radiolink_monitoring = RLM_no_monitoring,
};
nr_downlink_indication_t dl_indication;
fapi_nr_rx_indication_t rx_ind = {0};
nr_fill_dl_indication(&dl_indication, NULL, &rx_ind, proc, ue, NULL);
nr_fill_rx_indication(&rx_ind, FAPI_NR_MEAS_IND, ue, NULL, NULL, 1, proc, (void *)&l1_measurements, NULL);
ue->if_inst->dl_indication(&dl_indication);
```
- What this proves: PHY에서 MAC으로 CSI 측정값(RI, PMI i1/i2, CQI) 전달

### (증거) - 표준 OAI
- File: `openair2/LAYER2/NR_MAC_UE/nr_ue_procedures.c`
- Function: `nr_ue_process_l1_measurements`
- Lines: 1509-1530
- Snippet:
```c
void nr_ue_process_l1_measurements(NR_UE_MAC_INST_t *mac, frame_t frame, int slot, fapi_nr_l1_measurements_t *l1_measurements)
{
  LOG_D(NR_MAC, "(%d.%d) Received measurements from L1\n", frame, slot);
  bool csi_meas = l1_measurements->meas_type == NFAPI_NR_CSI_MEAS;
  if (!csi_meas && !l1_measurements->is_neighboring_cell) {
    // SSB 측정 처리
  } else if (csi_meas) {
    mac->csirs_measurements.rsrp_dBm = l1_measurements->rsrp_dBm;
    mac->csirs_measurements.i1 = l1_measurements->i1;
    mac->csirs_measurements.i2 = l1_measurements->i2;
    mac->csirs_measurements.cqi = l1_measurements->cqi;
    mac->csirs_measurements.ri = l1_measurements->rank_indicator;
  }
  // ...
}
```
- What this proves: MAC에서 PHY로부터 받은 CSI 측정값을 `mac->csirs_measurements`에 저장

### (증거) - 표준 OAI
- File: `openair2/LAYER2/NR_MAC_UE/nr_ue_procedures.c`
- Function: `nr_get_csi_measurements`
- Lines: 2680-2743
- Snippet:
```c
if (mac->sc_info.csi_MeasConfig) {
  NR_CSI_MeasConfig_t *csi_measconfig = mac->sc_info.csi_MeasConfig;
  // ...
  if(csirep->reportConfigType.present == NR_CSI_ReportConfig__reportConfigType_PR_periodic) {
    // periodic CSI report 스케줄링
    csi_payload_t csi = nr_get_csi_payload(mac, csi_report_id, WIDEBAND_ON_PUCCH, csi_measconfig);
    pucch->n_csi = csi.p1_bits;
    pucch->csi_part1_payload = csi.part1_payload;
    pucch->pucch_resource = csi_pucch;
  }
}
```
- What this proves: PUCCH 스케줄러에서 `nr_get_csi_payload()` 호출하여 CSI payload 생성

### (정리)
- **PHY → MAC 전달**: `fapi_nr_l1_measurements_t` 구조체로 RI, i1, i2, CQI 전달
- **MAC 저장**: `mac->csirs_measurements` 구조체에 저장
- **Payload 생성**: `nr_get_csi_payload()` 함수에서 `mac->csirs_measurements` 값을 읽어 payload 생성
- **RTD 패치**: 동일한 경로 사용, `get_csirs_RI_PMI_CQI_payload()` 함수 내부에 RTD 모드 분기 추가

---

## [2] Payload Bit Length 계산 방식 비교

### (증거) - 표준 OAI
- File: `openair2/LAYER2/NR_MAC_COMMON/nr_mac_common.c`
- Function: `nr_get_csi_bitlen`
- Lines: 4794-4828
- Snippet:
```c
uint16_t nr_get_csi_bitlen(nr_csi_report_t *csi_report)
{
  csi_meas_bitlen = &(csi_report->csi_meas_bitlen);
  uint16_t temp_bitlen;
  for (int i = 0; i < 8; i++) {  // 모든 RI 값(1~8)에 대해
    temp_bitlen = (csi_meas_bitlen->cri_bitlen+
                   csi_meas_bitlen->ri_bitlen+
                   csi_meas_bitlen->li_bitlen[i]+
                   csi_meas_bitlen->cqi_bitlen[i]+
                   csi_meas_bitlen->pmi_x1_bitlen[i]+
                   csi_meas_bitlen->pmi_x2_bitlen[i]);
    if(temp_bitlen > max_bitlen)
      max_bitlen = temp_bitlen;
  }
  csi_bitlen += max_bitlen;
  return csi_bitlen;
}
```
- What this proves: **max_over_all_RI** 방식으로 모든 RI에 대해 bitlen 계산 후 최대값 사용

### (증거) - RTD 패치
- File: `openair2/LAYER2/NR_MAC_COMMON/nr_mac_common.c`
- Function: `nr_get_csi_bitlen`
- Lines: 4822-4825 (RTD 패치)
- Snippet:
```c
#ifdef FORCE_RTD_CSI_BITMAP
  // RTD Test Mode: Force fixed 13-bit format
  return 13;
#else
  // 기존 로직 (max_over_all_RI)
#endif
```
- What this proves: RTD 모드에서는 **고정 13bit** 반환

### (증거) - RTD 패치
- File: `openair2/LAYER2/NR_MAC_COMMON/nr_mac_common.c`
- Function: `compute_csi_bitlen`
- Lines: 4791-4816 (RTD 패치)
- Snippet:
```c
#ifdef FORCE_RTD_CSI_BITMAP
  // RTD Test Mode: Force fixed bitlen values
  if (csi_report->reportQuantity_type == NR_CSI_ReportConfig__reportQuantity_PR_cri_RI_PMI_CQI
      || csi_report->reportQuantity_type == NR_CSI_ReportConfig__reportQuantity_PR_cri_RI_CQI) {
    csi_report->csi_meas_bitlen.cri_bitlen = 0;
    csi_report->csi_meas_bitlen.ri_bitlen = 2;
    // PMI, CQI bitlen 강제 설정
  }
#endif
```
- What this proves: RTD 모드에서는 bitlen 계산 시 **고정값 강제 설정**

### (정리)
- **표준 OAI**: 동적 계산 (RI, 안테나 구성, codebook mode에 따라 가변)
  - `max_over_all_RI` 방식: 모든 RI에 대해 계산 후 최대값 사용
  - 예: RI=1일 때 16bit, RI=2일 때 17bit → 최종 17bit 사용
- **RTD 패치**: 고정 13bit 강제
  - `FORCE_RTD_CSI_BITMAP` 플래그로 분기
  - bitlen 계산 함수에서 고정값 반환
  - 필드별 bitlen도 강제 설정 (CRI=0, RI=2, PMI=6, CQI=4, PAD=1)

---

## [3] Payload 인코딩 방식 비교

### (증거) - 표준 OAI
- File: `openair2/LAYER2/NR_MAC_UE/nr_ue_procedures.c`
- Function: `get_csirs_RI_PMI_CQI_payload`
- Lines: 3039-3049
- Snippet:
```c
else {
  p1_bits = nr_get_csi_bitlen(csi_report);
  padding_bitlen = p1_bits - (cri_bitlen + ri_bitlen + pmi_x1_bitlen + pmi_x2_bitlen + cqi_bitlen);
  temp_payload_1 = (0/*mac->csi_measurements.cri*/ << (cqi_bitlen + pmi_x2_bitlen + pmi_x1_bitlen + padding_bitlen + ri_bitlen)) |
                   (mac->csirs_measurements.ri << (cqi_bitlen + pmi_x2_bitlen + pmi_x1_bitlen + padding_bitlen)) |
                   (mac->csirs_measurements.i1 << (cqi_bitlen + pmi_x2_bitlen)) |
                   (mac->csirs_measurements.i2 << (cqi_bitlen)) |
                   (mac->csirs_measurements.cqi);
}
temp_payload_1 = reverse_bits(temp_payload_1, p1_bits);
```
- What this proves: 동적 bitlen 기반 인코딩, **reverse_bits 항상 적용**

### (증거) - RTD 패치
- File: `openair2/LAYER2/NR_MAC_UE/nr_ue_procedures.c`
- Function: `get_csirs_RI_PMI_CQI_payload`
- Lines: 3660-3727 (RTD 패치)
- Snippet:
```c
#ifdef FORCE_RTD_CSI_BITMAP
  else {
    // PUCCH mode: RTD fixed format
    p1_bits = 13;  // Fixed: RI(2) + PMI_i11(4) + PMI_i2(2) + CQI(4) + PAD(1)
    p2_bits = 0;
    
    // RI Encoding
    uint8_t ri_raw = mac->csirs_measurements.ri;  // 0-based
    uint8_t ri_val;
#ifdef FORCE_RTD_RI_ENC_RANK
    ri_val = (ri_raw + 1) & 0x3;  // rank value (1-4)
    if (ri_raw >= 3) ri_val = 3;
#else
    ri_val = ri_raw & 0x3;  // rank-1 (0-3), 기본값
#endif
    
    // PMI Encoding
    uint8_t pmi_i11_val;
    uint8_t pmi_i2_val;
#ifdef FORCE_RTD_PMI_ZERO
    pmi_i11_val = 0;
    pmi_i2_val = 0;
#else
    pmi_i11_val = mac->csirs_measurements.i1 & 0xF;  // 하위 4 bits
    pmi_i2_val = mac->csirs_measurements.i2 & 0x3;    // 하위 2 bits
#endif
    
    // Payload Packing (MSB → LSB): RI(2) | PMI_i11(4) | PMI_i2(2) | CQI(4) | PAD(1)
    temp_payload_1 = ((uint64_t)ri_val << 11) |
                     ((uint64_t)pmi_i11_val << 7) |
                     ((uint64_t)pmi_i2_val << 5) |
                     ((uint64_t)cqi_val << 1) |
                     ((uint64_t)pad_val);
    
#ifdef FORCE_RTD_WITH_REVERSE
    temp_payload_1 = reverse_bits(temp_payload_1, p1_bits);
#else
    // Default: NO reverse_bits (RTD expects direct bit order)
#endif
  }
#endif
```
- What this proves: RTD 모드에서는 **고정 13bit 포맷**, **reverse_bits 옵션화**

### (정리)
- **표준 OAI**:
  - 동적 bitlen 기반 인코딩
  - `pmi_x1_bitlen[ri]`, `pmi_x2_bitlen[ri]` 사용 (RI에 따라 가변)
  - `reverse_bits` **항상 적용**
  - 비트 순서: LSB → MSB (reverse_bits 적용 전) → MSB → LSB (적용 후)
- **RTD 패치**:
  - 고정 13bit 포맷 강제
  - RI 인코딩 옵션: `FORCE_RTD_RI_ENC_RANK` (rank 값 vs rank-1)
  - PMI 축소 매핑: `FORCE_RTD_PMI_ZERO` (0 강제) vs 마스킹 (기본값)
  - `reverse_bits` 옵션화: `FORCE_RTD_WITH_REVERSE` (적용) vs 미적용 (기본값)
  - 비트 순서: 옵션에 따라 결정

---

## [4] 필드별 Bit Length 계산 비교

### (증거) - 표준 OAI
- File: `openair2/LAYER2/NR_MAC_COMMON/nr_mac_common.c`
- Function: `set_bitlen_size_singlepanel`
- Lines: 4462-4593
- Snippet:
```c
// RI=1 케이스
case 1:
  if (codebook_mode == 1) {
    csi_bitlen->pmi_i11_bitlen[i] = ceil(log2(n1 * o1));  // 예: n1=2, o1=4 → 3bit
    csi_bitlen->pmi_i12_bitlen[i] = ceil(log2(n2 * o2));  // 예: n2=1, o2=1 → 0bit
    csi_bitlen->pmi_x2_bitlen[i] = 2;                     // 2bit
  }
  csi_bitlen->pmi_i13_bitlen[i] = 0;  // RI=1에서는 i13 없음

// RI=2 케이스
case 2:
  if(n1 * n2 == 2) {
    if (codebook_mode == 1) {
      csi_bitlen->pmi_i11_bitlen[i] = ceil(log2(n1 * o1));  // 3bit
      csi_bitlen->pmi_i12_bitlen[i] = ceil(log2(n2 * o2));  // 0bit
      csi_bitlen->pmi_x2_bitlen[i] = 1;                     // RI=2에서는 1bit
    }
    csi_bitlen->pmi_i13_bitlen[i] = 1;  // RI=2에서 i13 추가 (1bit)
  }
  
csi_bitlen->pmi_x1_bitlen[i] = csi_bitlen->pmi_i11_bitlen[i] + 
                                csi_bitlen->pmi_i12_bitlen[i] + 
                                csi_bitlen->pmi_i13_bitlen[i];
```
- What this proves: RI, 안테나 구성, codebook mode에 따라 **PMI bitlen 가변**

### (증거) - 표준 OAI
- File: `openair2/LAYER2/NR_MAC_COMMON/nr_mac_common.c`
- Function: `compute_ri_bitlen`
- Lines: 4329-4333
- Snippet:
```c
nb_allowed_ri = number_of_bits_set(ri_restriction);  // 비트마스크에서 1인 비트 개수
ri_bitlen = ceil(log2(nb_allowed_ri));               // 허용된 RI 개수의 log2
ri_bitlen = ri_bitlen < 2 ? ri_bitlen : 2;          // 상한 2bit cap
csi_report->csi_meas_bitlen.ri_bitlen = ri_bitlen;
```
- What this proves: RI bitlen은 **ri_Restriction 비트마스크**에 따라 계산 (최대 2bit)

### (정리)
- **표준 OAI**:
  - **RI bitlen**: `ri_Restriction` 비트마스크 기반 계산 (최대 2bit)
  - **PMI bitlen**: RI, n1/n2, o1/o2, codebook mode에 따라 가변
    - RI=1: i11 + i12 + i2 (예: 3+0+2=5bit 또는 4+0+2=6bit)
    - RI=2: i11 + i12 + i13 + i2 (예: 3+0+1+1=5bit 또는 3+3+2+1=9bit)
  - **CQI bitlen**: 4bit 고정
  - **Padding**: `p1_bits - (cri + ri + pmi_x1 + pmi_x2 + cqi)` 가변
- **RTD 패치**:
  - **RI bitlen**: 고정 2bit
  - **PMI bitlen**: 고정 6bit (i11=4, i2=2)
  - **CQI bitlen**: 4bit 고정
  - **Padding**: 고정 1bit (값 0)

---

## [5] PUCCH Payload 합성 비교

### (증거) - 표준 OAI
- File: `openair2/LAYER2/NR_MAC_UE/nr_ue_procedures.c`
- Function: `nr_ue_configure_pucch`
- Lines: 1743 (Format 2)
- Snippet:
```c
pucch_pdu->payload = (pucch->csi_part1_payload << (pucch->n_harq + pucch->n_sr)) | 
                     (pucch->sr_payload << pucch->n_harq) | 
                     pucch->ack_payload;
```
- What this proves: ACK, SR, CSI를 순차적으로 합성 (LSB → MSB)

### (증거) - RTD 패치
- File: `openair2/LAYER2/NR_MAC_UE/nr_ue_procedures.c`
- Function: `nr_ue_configure_pucch`
- Lines: 1808-1827 (Format 2, RTD 패치)
- Snippet:
```c
#ifdef FORCE_RTD_CSI_BITMAP
  int csi_offset = pucch->n_harq + pucch->n_sr;  // CSI starts after ACK and SR
  int total_bits = pucch->n_csi + pucch->n_harq + pucch->n_sr;
  int csi_msb = total_bits - 1;
  int csi_lsb = total_bits - pucch->n_csi;
  
  LOG_I(NR_MAC, "[RTD_TEST_MODE]   CSI position in final payload: bits [%d:%d] (offset=%d)\n", 
        csi_msb, csi_lsb, csi_offset);
  LOG_I(NR_MAC, "[RTD_TEST_MODE]   CSI-only hex (extracted): 0x%lx\n", 
        (pucch_pdu->payload >> csi_offset) & ((1UL << pucch->n_csi) - 1));
#endif
```
- What this proves: RTD 모드에서는 **CSI 위치 명시 로깅** 추가

### (정리)
- **표준 OAI**:
  - ACK, SR, CSI 순차 합성
  - 로깅: 기본 디버그 로그만
- **RTD 패치**:
  - 동일한 합성 방식
  - **상세 로깅 추가**: CSI 위치 (bit 범위), CSI-only hex 값 출력
  - RTD 디코더 검증을 위한 정보 제공

---

## [6] 주요 차이점 요약

### 6.1 Bit Length 계산
| 항목 | 표준 OAI | RTD 패치 |
|------|----------|----------|
| 계산 방식 | 동적 (RI, 안테나 구성, codebook mode) | 고정 13bit |
| RI bitlen | 가변 (ri_Restriction 기반, 최대 2bit) | 고정 2bit |
| PMI bitlen | 가변 (RI에 따라 5~9bit) | 고정 6bit (i11=4, i2=2) |
| CQI bitlen | 4bit 고정 | 4bit 고정 |
| Padding | 가변 | 고정 1bit |

### 6.2 Payload 인코딩
| 항목 | 표준 OAI | RTD 패치 |
|------|----------|----------|
| 포맷 | 동적 (bitlen에 따라) | 고정 13bit |
| RI 인코딩 | 0-based (rank-1) | 옵션 (rank-1 또는 rank) |
| PMI 인코딩 | 전체 i1, i2 사용 | 축소 매핑 (i11 하위 4bit, i2 하위 2bit) |
| reverse_bits | 항상 적용 | 옵션 (기본값: 미적용) |
| 비트 순서 | MSB → LSB (reverse_bits 적용 후) | 옵션에 따라 결정 |

### 6.3 로깅 및 디버깅
| 항목 | 표준 OAI | RTD 패치 |
|------|----------|----------|
| 로깅 레벨 | DEBUG (LOG_D) | INFO (LOG_I) |
| 로깅 내용 | 기본 필드값 | 상세 위치 정보, CSI-only hex |
| Sanity Check | 없음 | 설정 기반 검증 추가 |

### 6.4 컴파일 플래그
| 플래그 | 목적 | 기본값 |
|--------|------|--------|
| `FORCE_RTD_CSI_BITMAP` | RTD 모드 활성화 | 미정의 (표준 모드) |
| `FORCE_RTD_RI_ENC_RANK` | RI를 rank 값으로 인코딩 | 미정의 (rank-1) |
| `FORCE_RTD_WITH_REVERSE` | reverse_bits 적용 | 미정의 (미적용) |
| `FORCE_RTD_PMI_ZERO` | PMI를 0으로 강제 | 미정의 (마스킹) |

---

## [7] 코드 위치 참조

### 표준 OAI
| 기능 | 파일 | 함수 | 라인 |
|------|------|------|------|
| CSI 측정값 전달 | `csi_rx.c` | `nr_csi_rs_channel_estimation` | 2110-2131 |
| CSI 측정값 수신 | `nr_ue_procedures.c` | `nr_ue_process_l1_measurements` | 1509-1530 |
| CSI payload 생성 | `nr_ue_procedures.c` | `get_csirs_RI_PMI_CQI_payload` | 2994-3068 |
| Bitlen 계산 | `nr_mac_common.c` | `nr_get_csi_bitlen` | 4794-4828 |
| PMI bitlen 계산 | `nr_mac_common.c` | `set_bitlen_size_singlepanel` | 4462-4593 |
| PUCCH 합성 | `nr_ue_procedures.c` | `nr_ue_configure_pucch` | 1743 |

### RTD 패치
| 기능 | 파일 | 함수 | 라인 |
|------|------|------|------|
| Sanity Check | `nr_ue_procedures.c` | `get_csirs_RI_PMI_CQI_payload` | 3550-3620 |
| RTD Payload 생성 | `nr_ue_procedures.c` | `get_csirs_RI_PMI_CQI_payload` | 3646-3750 |
| Bitlen 강제 | `nr_mac_common.c` | `compute_csi_bitlen` | 4791-4816 |
| Bitlen 고정 | `nr_mac_common.c` | `nr_get_csi_bitlen` | 4822-4825 |
| 상세 로깅 | `nr_ue_procedures.c` | `nr_ue_configure_pucch` | 1808-1827 |

---

## [8] 결론

### 표준 OAI의 특징
1. **3GPP 표준 준수**: 동적 bitlen 계산, reverse_bits 적용
2. **유연성**: RI, 안테나 구성, codebook mode에 따라 자동 조정
3. **표준 호환**: 다양한 설정에서 동작

### RTD 패치의 특징
1. **RTD 디코더 호환**: 고정 13bit 포맷으로 RTD 가정에 맞춤
2. **옵션 제공**: RI 인코딩, reverse_bits, PMI 매핑 방식 선택 가능
3. **디버깅 지원**: 상세 로깅 및 sanity check 제공
4. **표준 비준수**: 테스트 전용, 표준과 다를 수 있음

### 사용 권장사항
- **표준 환경**: 표준 OAI 사용
- **RTD 테스트**: RTD 패치 + `FORCE_RTD_CSI_BITMAP` 플래그 사용
- **호환성 확인**: RTD 디코더와 OAI 인코더 간 비트 순서, bitlen 일치 여부 확인 필요
