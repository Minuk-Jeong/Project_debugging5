/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/***********************************************************************
*
* FILENAME    :  csi_rx.c
*
* MODULE      :
*
* DESCRIPTION :  function to receive the channel state information
*
************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "executables/nr-softmodem-common.h"
#include "nr_transport_proto_ue.h"
#include "PHY/phy_extern_nr_ue.h"
#include "PHY/NR_REFSIG/nr_refsig.h"
#include "common/utils/nr/nr_common.h"
#include "PHY/NR_UE_ESTIMATION/filt16a_32.h"

// Additional memory allocation, because of applying the filter and the memory offset to ensure memory alignment
#define FILTER_MARGIN 32

#define NR_CSIRS_DEBUG
//#define NR_CSIIM_DEBUG

// CSI-RS debugging: Enable comprehensive logging for RI/PMI/SINR calculation
// Set to 1 to enable, 0 to disable (compile-time control)
// ============================================================================
// CSI Logging Configuration Flags (Purpose-specific separation)
// ============================================================================
// CSI_LOG_INPUT_SUMMARY: Enable 1-line input summary log for RI/PMI/SINR/CQI
//   Default: ON (1) - Outputs key input values in compact format
//   To disable: #define CSI_LOG_INPUT_SUMMARY 0
#ifndef CSI_LOG_INPUT_SUMMARY
#define CSI_LOG_INPUT_SUMMARY 1
#endif

// CSI_LOG_CSIRS_VERBOSE: Enable verbose CSI-RS logs (periodicity, events, RE dumps)
//   Default: OFF (0) - Disables verbose CSI-RS related logs
//   To enable: #define CSI_LOG_CSIRS_VERBOSE 1
#ifndef CSI_LOG_CSIRS_VERBOSE
#define CSI_LOG_CSIRS_VERBOSE 0
#endif

// CSI_LOG_EVERY_N: Rate limit for input summary logs (output every N events)
//   Default: 20 - Outputs 1 log per 20 CSI-RS events
//   To increase frequency: #define CSI_LOG_EVERY_N 5 (more logs)
//   To decrease frequency: #define CSI_LOG_EVERY_N 50 (fewer logs)
#ifndef CSI_LOG_EVERY_N
#define CSI_LOG_EVERY_N 20
#endif

// CSI_LOG_ON_ANOMALY: Enable anomaly-triggered logs (bypasses rate limit)
//   Default: ON (1) - Outputs logs when anomalies detected
//   To disable: #define CSI_LOG_ON_ANOMALY 0
#ifndef CSI_LOG_ON_ANOMALY
#define CSI_LOG_ON_ANOMALY 1
#endif

// Anomaly detection thresholds
#define CSI_ANOMALY_INTF_THRESHOLD (UINT32_MAX / 2)  // interference_plus_noise_power > threshold
#define CSI_ANOMALY_ZERO_SINR_COUNT 10  // Consecutive zero SINR count
#define CSI_ANOMALY_ZERO_H_THRESHOLD_Q30 (1ULL << 20)  // |H|^2 < threshold (Q30 format, ~1e-6)
#define CSI_ANOMALY_ZERO_RX_THRESHOLD_Q30 (1ULL << 20)  // |rx|^2 < threshold (Q30 format)

#if CSI_LOG_INPUT_SUMMARY || CSI_LOG_CSIRS_VERBOSE
// Helper function to calculate absolute slot number
static inline uint32_t calc_abs_slot(int frame, int slot, int slots_per_frame) {
  return (uint32_t)(frame * slots_per_frame + slot);
}
#endif

#if CSI_LOG_INPUT_SUMMARY
// ============================================================================
// Input Summary Log State Tracker
// ============================================================================
typedef struct {
  int event_count;
  int zero_sinr_count;
  uint8_t last_rank;
  uint8_t last_i2;
  uint32_t last_intfN;
} csi_input_log_state_t;

static csi_input_log_state_t csi_input_log_state = {0};

// ============================================================================
// Helper function to calculate average |rx|^2 from CSI-RS received signal
// ============================================================================
static uint32_t calc_avg_rx_power(const PHY_VARS_NR_UE *ue,
                                   const c16_t csi_rs_received_signal[][ue->frame_parms.samples_per_slot_wCP],
                                   const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
                                   const csi_mapping_parms_t *csi_mapping,
                                   const NR_DL_FRAME_PARMS *frame_parms,
                                   int CDM_group_size) {
  uint64_t rx_power_sum = 0;
  uint32_t rx_count = 0;
  
  for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      
      for (int cdm_id = 0; cdm_id < csi_mapping->size; cdm_id++) {
        for (int s = 0; s < CDM_group_size; s++) {
          for (int kp = 0; kp <= csi_mapping->kprime; kp++) {
            uint16_t k = (frame_parms->first_carrier_offset + (rb * NR_NB_SC_PER_RB) + 
                         csi_mapping->koverline[cdm_id] + kp) % frame_parms->ofdm_symbol_size;
            
            for (int lp = 0; lp <= csi_mapping->lprime; lp++) {
              uint16_t symb = lp + csi_mapping->loverline[cdm_id];
              uint64_t symbol_offset = symb * frame_parms->ofdm_symbol_size;
              const c16_t *rx_signal = &csi_rs_received_signal[ant_rx][symbol_offset];
              
              int32_t rx_power = ((int32_t)rx_signal[k].r * rx_signal[k].r) + 
                                 ((int32_t)rx_signal[k].i * rx_signal[k].i);
              rx_power_sum += rx_power;
              rx_count++;
            }
          }
        }
      }
    }
  }
  
  return (rx_count > 0) ? (uint32_t)(rx_power_sum / rx_count) : 0;
}

// ============================================================================
// Helper function to calculate average |H|^2, max |H|^2, and zero H percentage
// ============================================================================
static void calc_h_statistics(const PHY_VARS_NR_UE *ue,
                              const c16_t csi_rs_estimated_channel_freq[][4][ue->frame_parms.ofdm_symbol_size + FILTER_MARGIN],
                              const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
                              const NR_DL_FRAME_PARMS *frame_parms,
                              uint8_t mem_offset,
                              uint8_t N_ports,
                              uint32_t *avg_h_power,
                              uint32_t *max_h_power,
                              uint32_t *zero_h_percent) {
  uint64_t h_power_sum = 0;
  uint32_t h_count = 0;
  uint32_t h_max = 0;
  uint32_t zero_h_count = 0;
  
  for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
    for (int port = 0; port < N_ports; port++) {
      for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
        if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
          continue;
        }
        
        // Calculate k (subcarrier index) for this RB - same as other functions
        uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
        // Calculate k_offset - same as other functions (nr_csi_rs_ri_estimation, nr_csi_rs_pmi_estimation)
        uint16_t k_offset = k + mem_offset;
        
        // Check channel estimate at this subcarrier (CSI-RS location)
        // Note: We only check the CSI-RS subcarrier, not all subcarriers in the RB
        // This matches how other functions access the channel estimate
        const c16_t *h = &csi_rs_estimated_channel_freq[ant_rx][port][k_offset];
        // Convert Q15 to Q30 for power calculation
        int64_t h_power_q30 = ((int64_t)h->r * h->r) + ((int64_t)h->i * h->i);
        
        h_power_sum += h_power_q30;
        h_count++;
        
        if (h_power_q30 > h_max) {
          h_max = h_power_q30;
        }
        
        if (h_power_q30 < CSI_ANOMALY_ZERO_H_THRESHOLD_Q30) {
          zero_h_count++;
        }
      }
    }
  }
  
  *avg_h_power = (h_count > 0) ? (uint32_t)(h_power_sum / h_count) : 0;
  *max_h_power = h_max;
  *zero_h_percent = (h_count > 0) ? (zero_h_count * 100 / h_count) : 0;
}

// ============================================================================
// Input Summary Log Function
// ============================================================================
static void log_csi_input_summary(const PHY_VARS_NR_UE *ue,
                                  const UE_nr_rxtx_proc_t *proc,
                                  const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
                                  const csi_mapping_parms_t *mapping_parms,
                                  const c16_t csi_rs_received_signal[][ue->frame_parms.samples_per_slot_wCP],
                                  const c16_t csi_rs_estimated_channel_freq[][4][ue->frame_parms.ofdm_symbol_size + FILTER_MARGIN],
                                  uint8_t mem_offset,
                                  int CDM_group_size,
                                  uint32_t rsrp,
                                  uint32_t noise_power,
                                  uint32_t interference_plus_noise_power,
                                  uint8_t rank_indicator,
                                  uint8_t i2,
                                  uint32_t sinr_dB,
                                  uint8_t cqi) {
  csi_input_log_state_t *state = &csi_input_log_state;
  state->event_count++;
  
  // Calculate input statistics
  uint32_t rxP = calc_avg_rx_power(ue, csi_rs_received_signal, csirs_config_pdu, mapping_parms,
                                    &ue->frame_parms, CDM_group_size);
  
  uint32_t hP = 0, hMax = 0, zeroH_percent = 0;
  calc_h_statistics(ue, csi_rs_estimated_channel_freq, csirs_config_pdu, &ue->frame_parms,
                    mem_offset, mapping_parms->ports, &hP, &hMax, &zeroH_percent);
  
  // Anomaly detection
  bool anomaly = false;
  if (CSI_LOG_ON_ANOMALY) {
    if (interference_plus_noise_power > CSI_ANOMALY_INTF_THRESHOLD) {
      anomaly = true;
    }
    if (sinr_dB == 0) {
      state->zero_sinr_count++;
      if (state->zero_sinr_count >= CSI_ANOMALY_ZERO_SINR_COUNT) {
        anomaly = true;
      }
    } else {
      state->zero_sinr_count = 0;
    }
    if (hP < CSI_ANOMALY_ZERO_H_THRESHOLD_Q30 || rxP < CSI_ANOMALY_ZERO_RX_THRESHOLD_Q30) {
      anomaly = true;
    }
  }
  
  // Rate limit check (unless anomaly)
  bool should_log = false;
  if (anomaly) {
    should_log = true;
  } else if (state->event_count % CSI_LOG_EVERY_N == 0) {
    should_log = true;
  }
  
  if (should_log) {
    uint32_t abs_slot = calc_abs_slot(proc->frame_rx, proc->nr_slot_rx, ue->frame_parms.slots_per_frame);
    int pmi_value = (i2 >= 0 && i2 < 16) ? (int)i2 : -1;  // -1 for "not implemented"
    
    LOG_I(NR_PHY, "[CSI-IN] f=%d s=%d abs=%u ports=%d row=%d dens=%d rb=%d@%d l0=%d rxP=%u hP=%u hMax=%u zeroH=%u%% rsrp=%u noise=%u intfN=%u RI=%d rankInd=%d PMI=%d sinr=%udB CQI=%d\n",
          proc->frame_rx, proc->nr_slot_rx, abs_slot,
          mapping_parms->ports, csirs_config_pdu->row, csirs_config_pdu->freq_density,
          csirs_config_pdu->nr_of_rbs, csirs_config_pdu->start_rb, csirs_config_pdu->symb_l0,
          rxP, hP, hMax, zeroH_percent,
          rsrp, noise_power, interference_plus_noise_power,
          rank_indicator + 1, rank_indicator, pmi_value, sinr_dB, cqi);
    
    // Update state
    state->last_rank = rank_indicator;
    state->last_i2 = i2;
    state->last_intfN = interference_plus_noise_power;
  }
}
#endif // CSI_LOG_INPUT_SUMMARY

#if CSI_LOG_CSIRS_VERBOSE
// CSI-RS periodicity tracking structure (only when verbose logging enabled)
#define CSI_PERIOD_STATS_WINDOW 50
#define CSI_RS_EVENT_LOG_EVERY_N 100

typedef struct {
  uint32_t prev_abs_slot;
  int prev_frame;
  int prev_slot;
  uint32_t delta_slots[CSI_PERIOD_STATS_WINDOW];
  int stats_count;
  int event_count;
  uint32_t mode_delta;
} csi_rs_period_tracker_t;

static csi_rs_period_tracker_t csi_rs_tracker = {0};

static inline double calc_slot_duration_ms(int numerology_index) {
  return 1.0 / (1 << numerology_index);
}

static void log_csi_rs_period_stats(const NR_DL_FRAME_PARMS *frame_parms, 
                                     int frame, int slot,
                                     uint32_t delta_slot, double delta_time_ms,
                                     int n_ports, int row, int freq_density) {
  csi_rs_period_tracker_t *t = &csi_rs_tracker;
  t->delta_slots[t->stats_count % CSI_PERIOD_STATS_WINDOW] = delta_slot;
  t->stats_count++;
  
  if (t->stats_count >= CSI_PERIOD_STATS_WINDOW) {
    uint32_t min_delta = UINT32_MAX, max_delta = 0, sum_delta = 0;
    uint32_t mode_delta = 0;
    int mode_count = 0;
    int freq[256] = {0};
    
    for (int i = 0; i < CSI_PERIOD_STATS_WINDOW; i++) {
      uint32_t d = t->delta_slots[i];
      if (d < 256) {
        freq[d]++;
        if (freq[d] > mode_count) {
          mode_count = freq[d];
          mode_delta = d;
        }
      }
      if (d < min_delta) min_delta = d;
      if (d > max_delta) max_delta = d;
      sum_delta += d;
    }
    
    double avg_delta = (double)sum_delta / CSI_PERIOD_STATS_WINDOW;
    double avg_time_ms = avg_delta * calc_slot_duration_ms(frame_parms->numerology_index);
    t->mode_delta = mode_delta;
    
    if ((t->stats_count % CSI_PERIOD_STATS_WINDOW) == 0) {
      LOG_I(NR_PHY, "[CSI-RS Period] Stats (last %d events): min=%u, max=%u, avg=%.1f slots (%.2f ms), mode=%u (freq=%d)\n",
            CSI_PERIOD_STATS_WINDOW, min_delta, max_delta, avg_delta, avg_time_ms, mode_delta, mode_count);
    }
  }
  
  if (t->event_count % CSI_RS_EVENT_LOG_EVERY_N == 0) {
    LOG_I(NR_PHY, "[CSI-RS Event] abs_slot=%u (f=%d s=%d) delta_slot=%u (%.2f ms) ports=%d row=%d density=%d\n",
          calc_abs_slot(frame, slot, frame_parms->slots_per_frame),
          frame, slot, delta_slot, delta_time_ms, n_ports, row, freq_density);
  }
  t->event_count++;
}
#endif // CSI_LOG_CSIRS_VERBOSE

void nr_det_A_MF_2x2(int32_t *a_mf_00,
                     int32_t *a_mf_01,
                     int32_t *a_mf_10,
                     int32_t *a_mf_11,
                     int32_t *det_fin,
                     const unsigned short nb_rb) {

  simde__m128i ad_re_128, bc_re_128, det_re_128;

  simde__m128i *a_mf_00_128 = (simde__m128i *)a_mf_00;
  simde__m128i *a_mf_01_128 = (simde__m128i *)a_mf_01;
  simde__m128i *a_mf_10_128 = (simde__m128i *)a_mf_10;
  simde__m128i *a_mf_11_128 = (simde__m128i *)a_mf_11;
  simde__m128i *det_fin_128 = (simde__m128i *)det_fin;

  for (int rb = 0; rb<3*nb_rb; rb++) {

    //complex multiplication (I_a+jQ_a)(I_d+jQ_d) = (I_aI_d - Q_aQ_d) + j(Q_aI_d + I_aQ_d)
    //The imag part is often zero, we compute only the real part
    ad_re_128 = simde_mm_madd_epi16(oai_mm_conj(a_mf_00_128[0]), a_mf_11_128[0]); //Re: I_a0*I_d0 - Q_a1*Q_d1

    //complex multiplication (I_b+jQ_b)(I_c+jQ_c) = (I_bI_c - Q_bQ_c) + j(Q_bI_c + I_bQ_c)
    //The imag part is often zero, we compute only the real part
    bc_re_128 = simde_mm_madd_epi16(oai_mm_conj(a_mf_01_128[0]), a_mf_10_128[0]); //Re: I_b0*I_c0 - Q_b1*Q_c1

    det_re_128 = simde_mm_sub_epi32(ad_re_128, bc_re_128);

    //det in Q30 format
    det_fin_128[0] = simde_mm_abs_epi32(det_re_128);

    det_fin_128+=1;
    a_mf_00_128+=1;
    a_mf_01_128+=1;
    a_mf_10_128+=1;
    a_mf_11_128+=1;
  }
}

void nr_squared_matrix_element(int32_t *a,
                               int32_t *a_sq,
                               const unsigned short nb_rb) {
  simde__m128i *a_128 = (simde__m128i *)a;
  simde__m128i *a_sq_128 = (simde__m128i *)a_sq;
  for (int rb=0; rb<3*nb_rb; rb++) {
    a_sq_128[0] = simde_mm_madd_epi16(a_128[0], a_128[0]);
    a_sq_128+=1;
    a_128+=1;
  }
}

void nr_numer_2x2(int32_t *a_00_sq,
                  int32_t *a_01_sq,
                  int32_t *a_10_sq,
                  int32_t *a_11_sq,
                  int32_t *num_fin,
                  const unsigned short nb_rb) {
  simde__m128i *a_00_sq_128 = (simde__m128i *)a_00_sq;
  simde__m128i *a_01_sq_128 = (simde__m128i *)a_01_sq;
  simde__m128i *a_10_sq_128 = (simde__m128i *)a_10_sq;
  simde__m128i *a_11_sq_128 = (simde__m128i *)a_11_sq;
  simde__m128i *num_fin_128 = (simde__m128i *)num_fin;
  for (int rb=0; rb<3*nb_rb; rb++) {
    simde__m128i sq_a_plus_sq_d_128 = simde_mm_add_epi32(a_00_sq_128[0], a_11_sq_128[0]);
    simde__m128i sq_b_plus_sq_c_128 = simde_mm_add_epi32(a_01_sq_128[0], a_10_sq_128[0]);
    num_fin_128[0] = simde_mm_add_epi32(sq_a_plus_sq_d_128, sq_b_plus_sq_c_128);
    num_fin_128+=1;
    a_00_sq_128+=1;
    a_01_sq_128+=1;
    a_10_sq_128+=1;
    a_11_sq_128+=1;
  }
}

// 4x4 system RI estimation using condition number based on 2x2 submatrices
static int nr_csi_rs_ri_estimation_4x4(const PHY_VARS_NR_UE *ue,
                                       const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
                                       const nr_csi_info_t *nr_csi_info,
                                       const uint8_t N_ports,
                                       uint8_t mem_offset,
                                       c16_t csi_rs_estimated_channel_freq[][N_ports][ue->frame_parms.ofdm_symbol_size + FILTER_MARGIN],
                                       const int16_t log2_maxh,
                                       uint8_t *rank_indicator)
{
  const NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
  const int16_t cond_dB_threshold = 5;
  int count = 0;
  *rank_indicator = 0;

  // For 4x4 system, compute condition number using 2x2 submatrices
  // We evaluate multiple 2x2 submatrices and use the average condition number
  c16_t csi_rs_estimated_A_MF[4][4][frame_parms->ofdm_symbol_size + FILTER_MARGIN] __attribute__((aligned(32)));
  memset(csi_rs_estimated_A_MF, 0, sizeof(csi_rs_estimated_A_MF));

  for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
    if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
      continue;
    }
    uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
    uint16_t k_offset = k + mem_offset;

    // For each subcarrier in the RB, compute H^H x H matrix and condition number
    for (int sc_idx = 0; sc_idx < NR_NB_SC_PER_RB; sc_idx++) {
      uint16_t sc_k_offset = k_offset + sc_idx;
      
      // Initialize A_MF matrix for this subcarrier
      c16_t A_MF[4][4];
      memset(A_MF, 0, sizeof(A_MF));

      // Compute H^H x H matrix for 4x4 system: (H^H x H)[i][j] = sum_k(conj(H[k][i]) * H[k][j])
      for (int port_tx_i = 0; port_tx_i < 4; port_tx_i++) {
        for (int port_tx_j = 0; port_tx_j < 4; port_tx_j++) {
          c16_t sum = {0, 0};
          for (int ant_rx = 0; ant_rx < 4; ant_rx++) {
            c16_t conj_h, h, prod;
            // Get H[ant_rx][port_tx_i] and compute its conjugate
            conj_h.r = csi_rs_estimated_channel_freq[ant_rx][port_tx_i][sc_k_offset].r;
            conj_h.i = -csi_rs_estimated_channel_freq[ant_rx][port_tx_i][sc_k_offset].i;
            // Get H[ant_rx][port_tx_j]
            h = csi_rs_estimated_channel_freq[ant_rx][port_tx_j][sc_k_offset];
            // Compute product: conj(H) * H
            prod.r = ((conj_h.r * h.r) - (conj_h.i * h.i)) >> log2_maxh;
            prod.i = ((conj_h.r * h.i) + (conj_h.i * h.r)) >> log2_maxh;
            sum.r += prod.r;
            sum.i += prod.i;
          }
          A_MF[port_tx_i][port_tx_j] = sum;
        }
      }

      // Compute condition number using 2x2 submatrices
      int32_t avg_cond_num = 0;
      int num_submatrices = 0;

      // Evaluate 2x2 submatrices: upper-left (0,1)x(0,1) and lower-right (2,3)x(2,3)
      for (int base_i = 0; base_i < 3; base_i += 2) {
        int i = base_i;
        int j = base_i + 1;
        
        // Compute determinant of 2x2 submatrix [i,j]x[i,j]
        // det = A[i][i] * A[j][j] - A[i][j] * conj(A[j][i])
        // Note: A is Hermitian, so A[j][i] = conj(A[i][j])
        c16_t a_ii = A_MF[i][i];
        c16_t a_jj = A_MF[j][j];
        c16_t a_ij = A_MF[i][j];
        c16_t a_ji = A_MF[j][i]; // Should be conj(a_ij) for Hermitian matrix

        // det = real(a_ii * a_jj - a_ij * conj(a_ij))
        int32_t a_ii_a_jj_re = ((int32_t)a_ii.r * (int32_t)a_jj.r) - ((int32_t)a_ii.i * (int32_t)a_jj.i);
        int32_t a_ij_conj_re = ((int32_t)a_ij.r * (int32_t)a_ij.r) + ((int32_t)a_ij.i * (int32_t)a_ij.i);
        int32_t det = abs(a_ii_a_jj_re - a_ij_conj_re);

        // Compute numerator: sum of squares of diagonal elements
        int32_t numer = ((int32_t)a_ii.r * (int32_t)a_ii.r) + ((int32_t)a_ii.i * (int32_t)a_ii.i) +
                        ((int32_t)a_jj.r * (int32_t)a_jj.r) + ((int32_t)a_jj.i * (int32_t)a_jj.i);

        // Compute condition number in dB
        if (det > 0) {
          int32_t numer_db = dB_fixed(numer);
          int32_t det_db = dB_fixed(det);
          int32_t cond_db = numer_db - det_db;
          avg_cond_num += cond_db;
          num_submatrices++;
        }
      }

      if (num_submatrices > 0) {
        avg_cond_num = avg_cond_num / num_submatrices;
        if (avg_cond_num < cond_dB_threshold) {
          count++;
        } else {
          count--;
        }
      }
    }
  }

  // If condition number is lower than threshold in half or more REs, rank > 1
  if (count > 0) {
    *rank_indicator = 1; // Rank 2 or higher
  } else {
    *rank_indicator = 0; // Rank 1
  }

#ifdef NR_CSIRS_DEBUG
#if CSI_LOG_CSIRS_VERBOSE
  // Verbose RI logging (only when CSI_LOG_CSIRS_VERBOSE enabled)
  LOG_I(NR_PHY, "[4x4 RI] count = %i, rank = %i\n", count, (*rank_indicator) + 1);
#endif
#endif

  return 0;
}

bool is_csi_rs_in_symbol(const fapi_nr_dl_config_csirs_pdu_rel15_t csirs_config_pdu, const int symbol) {

  bool ret = false;

  // 38.211-Table 7.4.1.5.3-1: CSI-RS locations within a slot
  switch(csirs_config_pdu.row){
    case 1:
    case 2:
    case 3:
    case 4:
    case 6:
    case 9:
      if(symbol == csirs_config_pdu.symb_l0) {
        ret = true;
      }
      break;
    case 5:
    case 7:
    case 8:
    case 10:
    case 11:
    case 12:
      if(symbol == csirs_config_pdu.symb_l0 || symbol == (csirs_config_pdu.symb_l0+1) ) {
        ret = true;
      }
      break;
    case 13:
    case 14:
    case 16:
    case 17:
      if(symbol == csirs_config_pdu.symb_l0 || symbol == (csirs_config_pdu.symb_l0+1) ||
          symbol == csirs_config_pdu.symb_l1 || symbol == (csirs_config_pdu.symb_l1+1)) {
        ret = true;
      }
      break;
    case 15:
    case 18:
      if(symbol == csirs_config_pdu.symb_l0 || symbol == (csirs_config_pdu.symb_l0+1) || symbol == (csirs_config_pdu.symb_l0+2) ) {
        ret = true;
      }
      break;
    default:
      AssertFatal(0==1, "Row %d is not valid for CSI Table 7.4.1.5.3-1\n", csirs_config_pdu.row);
  }

  return ret;
}

static int nr_get_csi_rs_signal(const PHY_VARS_NR_UE *ue,
                                const UE_nr_rxtx_proc_t *proc,
                                const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
                                const nr_csi_info_t *nr_csi_info,
                                const csi_mapping_parms_t *csi_mapping,
                                const int CDM_group_size,
                                c16_t csi_rs_received_signal[][ue->frame_parms.samples_per_slot_wCP],
                                uint32_t *rsrp,
                                int *rsrp_dBm,
                                const c16_t rxdataF[][ue->frame_parms.samples_per_slot_wCP])
{
  const NR_DL_FRAME_PARMS *fp = &ue->frame_parms;
  uint16_t meas_count = 0;
  uint32_t rsrp_sum = 0;

  for (int ant_rx = 0; ant_rx < fp->nb_antennas_rx; ant_rx++) {

    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {

      // for freq density 0.5 checks if even or odd RB
      if(csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }

      for (int cdm_id = 0; cdm_id < csi_mapping->size; cdm_id++) {
        for (int s = 0; s < CDM_group_size; s++)  {

          // loop over frequency resource elements within a group
          for (int kp = 0; kp <= csi_mapping->kprime; kp++) {

            uint16_t k = (fp->first_carrier_offset + (rb * NR_NB_SC_PER_RB) + csi_mapping->koverline[cdm_id] + kp) % fp->ofdm_symbol_size;

            // loop over time resource elements within a group
            for (int lp = 0; lp <= csi_mapping->lprime; lp++) {
              uint16_t symb = lp + csi_mapping->loverline[cdm_id];
              uint64_t symbol_offset = symb * fp->ofdm_symbol_size;
              const c16_t *rx_signal = &rxdataF[ant_rx][symbol_offset];
              c16_t *rx_csi_rs_signal = &csi_rs_received_signal[ant_rx][symbol_offset];
              rx_csi_rs_signal[k].r = rx_signal[k].r;
              rx_csi_rs_signal[k].i = rx_signal[k].i;

              rsrp_sum += (((int32_t)(rx_csi_rs_signal[k].r)*rx_csi_rs_signal[k].r) +
                           ((int32_t)(rx_csi_rs_signal[k].i)*rx_csi_rs_signal[k].i));

              meas_count++;

              // RE dump logging: only when NR_CSIRS_DUMP_RE=1, and only for k stride=64, ant_rx=0, port=0, l=symb_l0
              static int dump_re_enabled = -1; // -1: not checked yet, 0: disabled, 1: enabled
              if (dump_re_enabled == -1) {
                char *env_val = getenv("NR_CSIRS_DUMP_RE");
                dump_re_enabled = (env_val != NULL && strcmp(env_val, "1") == 0) ? 1 : 0;
              }
              
              if (dump_re_enabled == 1) {
                uint16_t port_tx = s + csi_mapping->j[cdm_id] * CDM_group_size;
                // Only dump: k stride=64, ant_rx=0, port=0, l=symb_l0
                if (ant_rx == 0 && port_tx == 0 && symb == csirs_config_pdu->symb_l0 && (k % 64 == 0)) {
                  int dataF_offset = proc->nr_slot_rx * fp->samples_per_slot_wCP;
                  c16_t *tx_csi_rs_signal = &nr_csi_info->csi_rs_generated_signal[port_tx][symbol_offset + dataF_offset];
                  LOG_I(NR_PHY,
                        "l,k (%2d,%4d) |\tport_tx %d (%4d,%4d)\tant_rx %d (%4d,%4d)\n",
                        symb,
                        k,
                        port_tx+3000,
                        tx_csi_rs_signal[k].r,
                        tx_csi_rs_signal[k].i,
                        ant_rx,
                        rx_csi_rs_signal[k].r,
                        rx_csi_rs_signal[k].i);
                }
              }
            }
          }
        }
      }
    }
  }


  *rsrp = rsrp_sum/meas_count;
  *rsrp_dBm = dB_fixed(*rsrp) + 30 - SQ15_SQUARED_NORM_FACTOR_DB
      - ((int)ue->openair0_cfg[0].rx_gain[0] - (int)ue->openair0_cfg[0].rx_gain_offset[0]) - dB_fixed(ue->frame_parms.ofdm_symbol_size);

#ifdef NR_CSIRS_DEBUG
  LOG_I(NR_PHY, "RSRP = %i (%i dBm)\n", *rsrp, *rsrp_dBm);
#endif

  return 0;
}

uint32_t calc_power_csirs(const uint16_t *x, const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu)
{
  uint64_t sum_x = 0;
  uint64_t sum_x2 = 0;
  uint16_t size = 0;
  for (int rb = 0; rb < csirs_config_pdu->nr_of_rbs; rb++) {
    if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != ((rb + csirs_config_pdu->start_rb) % 2)) {
      continue;
    }
    sum_x = sum_x + x[rb];
    sum_x2 = sum_x2 + x[rb] * x[rb];
    size++;
  }
  return sum_x2 / size - (sum_x / size) * (sum_x / size);
}

static int nr_csi_rs_channel_estimation(
    const NR_DL_FRAME_PARMS *fp,
    const UE_nr_rxtx_proc_t *proc,
    const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
    const nr_csi_info_t *nr_csi_info,
    const c16_t **csi_rs_generated_signal,
    const c16_t csi_rs_received_signal[][fp->samples_per_slot_wCP],
    const csi_mapping_parms_t *csi_mapping,
    const int CDM_group_size,
    uint8_t mem_offset,
    c16_t csi_rs_ls_estimated_channel[][csi_mapping->ports][fp->ofdm_symbol_size],
    c16_t csi_rs_estimated_channel_freq[][csi_mapping->ports][fp->ofdm_symbol_size + FILTER_MARGIN],
    int16_t *log2_re,
    int16_t *log2_maxh,
    uint32_t *noise_power)
{
  const int dataF_offset = proc->nr_slot_rx * fp->samples_per_slot_wCP;
  *noise_power = 0;
  int maxh = 0;
  int count = 0;

  for (int ant_rx = 0; ant_rx < fp->nb_antennas_rx; ant_rx++) {

    /// LS channel estimation

    for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
      memset(csi_rs_ls_estimated_channel[ant_rx][port_tx], 0, fp->ofdm_symbol_size * sizeof(c16_t));
    }

    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {

      // for freq density 0.5 checks if even or odd RB
      if(csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }

      for (int cdm_id = 0; cdm_id < csi_mapping->size; cdm_id++) {
        for (int s = 0; s < CDM_group_size; s++)  {

          uint16_t port_tx = s + csi_mapping->j[cdm_id] * CDM_group_size;

          // loop over frequency resource elements within a group
          for (int kp = 0; kp <= csi_mapping->kprime; kp++) {

            uint16_t kinit = (fp->first_carrier_offset + rb*NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
            uint16_t k = kinit + csi_mapping->koverline[cdm_id] + kp;

            // loop over time resource elements within a group
            for (int lp = 0; lp <= csi_mapping->lprime; lp++) {
              uint16_t symb = lp + csi_mapping->loverline[cdm_id];
              uint64_t symbol_offset = symb * fp->ofdm_symbol_size;
              const c16_t *tx_csi_rs_signal = &csi_rs_generated_signal[port_tx][symbol_offset+dataF_offset];
              const c16_t *rx_csi_rs_signal = &csi_rs_received_signal[ant_rx][symbol_offset];
              c16_t tmp = c16MulConjShift(tx_csi_rs_signal[k], rx_csi_rs_signal[k], nr_csi_info->csi_rs_generated_signal_bits);
              // This is not just the LS estimation for each (k,l), but also the sum of the different contributions
              // for the sake of optimizing the memory used.
              csi_rs_ls_estimated_channel[ant_rx][port_tx][kinit].r += tmp.r;
              csi_rs_ls_estimated_channel[ant_rx][port_tx][kinit].i += tmp.i;
            }
          }
        }
      }
    }

#ifdef NR_CSIRS_DEBUG
#if 0
    // RE-level logging disabled to reduce log explosion
    // All for(k) loop logs removed per requirements
    for(int symb = 0; symb < NR_SYMBOLS_PER_SLOT; symb++) {
      if(!is_csi_rs_in_symbol(*csirs_config_pdu,symb)) {
        continue;
      }
      for(int k = 0; k < fp->ofdm_symbol_size; k++) {
        LOG_I(NR_PHY, "l,k (%2d,%4d) | ", symb, k);
        for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
          uint64_t symbol_offset = symb * fp->ofdm_symbol_size;
          c16_t *tx_csi_rs_signal = (c16_t*)&csi_rs_generated_signal[port_tx][symbol_offset+dataF_offset];
          c16_t *rx_csi_rs_signal = (c16_t*)&csi_rs_received_signal[ant_rx][symbol_offset];
          c16_t *csi_rs_ls_estimated_channel16 = csi_rs_ls_estimated_channel[ant_rx][port_tx];
          printf("port_tx %d --> ant_rx %d, tx (%4d,%4d), rx (%4d,%4d), ls (%4d,%4d) | ",
                 port_tx+3000, ant_rx,
                 tx_csi_rs_signal[k].r, tx_csi_rs_signal[k].i,
                 rx_csi_rs_signal[k].r, rx_csi_rs_signal[k].i,
                 csi_rs_ls_estimated_channel16[k].r, csi_rs_ls_estimated_channel16[k].i);
        }
        printf("\n");
      }
    }
#endif
#endif

    /// Channel interpolation

    for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
      memset(csi_rs_estimated_channel_freq[ant_rx][port_tx], 0, (fp->ofdm_symbol_size + FILTER_MARGIN) * sizeof(c16_t));
    }

    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {

      // for freq density 0.5 checks if even or odd RB
      if(csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }

      count++;

      uint16_t k = (fp->first_carrier_offset + rb * NR_NB_SC_PER_RB) % fp->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;
      for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
        c16_t csi_rs_ls_estimated_channel16 = csi_rs_ls_estimated_channel[ant_rx][port_tx][k];
        c16_t *csi_rs_estimated_channel16 = &csi_rs_estimated_channel_freq[ant_rx][port_tx][k_offset];
        if( (k == 0) || (k == fp->first_carrier_offset) ) { // Start of OFDM symbol case or first occupied subcarrier case
          multadd_real_vector_complex_scalar(filt24_start, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16, 24);
        } else if(((k + NR_NB_SC_PER_RB) >= fp->ofdm_symbol_size) ||
                   (rb == (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs-1))) { // End of OFDM symbol case or Last occupied subcarrier case
          multadd_real_vector_complex_scalar(filt24_end, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16 - 12, 24);
        } else { // Middle case
          multadd_real_vector_complex_scalar(filt24_middle, csi_rs_ls_estimated_channel16, csi_rs_estimated_channel16 - 12, 24);
        }
      }
    }

    /// Power noise estimation
    AssertFatal(csirs_config_pdu->nr_of_rbs > 0, " nr_of_rbs needs to be greater than 0\n");
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
        maxh = cmax3(maxh, abs(csi_rs_estimated_channel16->r), abs(csi_rs_estimated_channel16->i));
      }
    }
    for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
      *noise_power += (calc_power_csirs(noise_real[ant_rx][port_tx], csirs_config_pdu) + calc_power_csirs(noise_imag[ant_rx][port_tx],csirs_config_pdu));
    }

#ifdef NR_CSIRS_DEBUG
#if 0
    // RE-level (k loop) logging disabled to reduce log explosion
    for(int k = 0; k < fp->ofdm_symbol_size; k++) {
      int rb = k >= fp->first_carrier_offset ?
               (k - fp->first_carrier_offset)/NR_NB_SC_PER_RB :
               (k + fp->ofdm_symbol_size - fp->first_carrier_offset)/NR_NB_SC_PER_RB;
      // RE-level logging disabled: LOG_I(NR_PHY, "(k = %4d) |\t", k);
      for(uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
        c16_t *csi_rs_ls_estimated_channel16 = &csi_rs_ls_estimated_channel[ant_rx][port_tx][0];
        c16_t *csi_rs_estimated_channel16 = &csi_rs_estimated_channel_freq[ant_rx][port_tx][mem_offset];
        printf("Channel port_tx %d --> ant_rx %d : ls (%4d,%4d), int (%4d,%4d), noise (%4d,%4d) | ",
               port_tx+3000, ant_rx,
               csi_rs_ls_estimated_channel16[k].r, csi_rs_ls_estimated_channel16[k].i,
               csi_rs_estimated_channel16[k].r, csi_rs_estimated_channel16[k].i,
               rb >= csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs ? 0 : noise_real[ant_rx][port_tx][rb-csirs_config_pdu->start_rb],
               rb >= csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs ? 0 : noise_imag[ant_rx][port_tx][rb-csirs_config_pdu->start_rb]);
      }
      printf("\n");
    }
#endif
#endif

  }

  *noise_power /= (fp->nb_antennas_rx * csi_mapping->ports);
  *log2_maxh = log2_approx(maxh - 1);
  *log2_re = log2_approx(count - 1);

#ifdef NR_CSIRS_DEBUG
  LOG_I(NR_PHY, "Noise power estimation based on CSI-RS: %i\n", *noise_power);
#endif
  return 0;
}

int nr_csi_rs_ri_estimation(const PHY_VARS_NR_UE *ue,
                            const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
                            const nr_csi_info_t *nr_csi_info,
                            const uint8_t N_ports,
                            uint8_t mem_offset,
                            c16_t csi_rs_estimated_channel_freq[][N_ports][ue->frame_parms.ofdm_symbol_size + FILTER_MARGIN],
                            const int16_t log2_maxh,
                            uint8_t *rank_indicator)
{
  const NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
  const int16_t cond_dB_threshold = 5;
  int count = 0;
  *rank_indicator = 0;

  if (ue->frame_parms.nb_antennas_rx == 1 || N_ports == 1) {
    return 0;
  } else if( !(ue->frame_parms.nb_antennas_rx == 2 && N_ports == 2) && 
             !(ue->frame_parms.nb_antennas_rx == 4 && N_ports == 4) ) {
    LOG_W(NR_PHY, "Rank indicator computation is not implemented for %i x %i system\n",
          ue->frame_parms.nb_antennas_rx, N_ports);
    return -1;
  }

  // Handle 4x4 system
  if (ue->frame_parms.nb_antennas_rx == 4 && N_ports == 4) {
    return nr_csi_rs_ri_estimation_4x4(ue, csirs_config_pdu, nr_csi_info, N_ports, mem_offset,
                                       csi_rs_estimated_channel_freq, log2_maxh, rank_indicator);
  }

  /* Example 2x2: Hh x H =
  *            | conjch00 conjch10 | x | ch00 ch01 | = | conjch00*ch00+conjch10*ch10 conjch00*ch01+conjch10*ch11 |
  *            | conjch01 conjch11 |   | ch10 ch11 |   | conjch01*ch00+conjch11*ch10 conjch01*ch01+conjch11*ch11 |
  */

  c16_t csi_rs_estimated_conjch_ch[frame_parms->nb_antennas_rx][N_ports][frame_parms->nb_antennas_rx][N_ports]
                                  [frame_parms->ofdm_symbol_size + FILTER_MARGIN] __attribute__((aligned(32)));
  int32_t csi_rs_estimated_A_MF[N_ports][N_ports][frame_parms->ofdm_symbol_size + FILTER_MARGIN] __attribute__((aligned(32)));
  int32_t csi_rs_estimated_A_MF_sq[N_ports][N_ports][frame_parms->ofdm_symbol_size + FILTER_MARGIN] __attribute__((aligned(32)));
  int32_t csi_rs_estimated_determ_fin[frame_parms->ofdm_symbol_size + FILTER_MARGIN] __attribute__((aligned(32)));
  int32_t csi_rs_estimated_numer_fin[frame_parms->ofdm_symbol_size + FILTER_MARGIN] __attribute__((aligned(32)));
  const uint8_t sum_shift = 1; // log2(2x2) = 2, which is a shift of 1 bit
  
  for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb+csirs_config_pdu->nr_of_rbs); rb++) {

    if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
      continue;
    }
    uint16_t k = (frame_parms->first_carrier_offset + rb*NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
    uint16_t k_offset = k + mem_offset;

    for (int ant_rx_conjch = 0; ant_rx_conjch < frame_parms->nb_antennas_rx; ant_rx_conjch++) {
      for(uint16_t port_tx_conjch = 0; port_tx_conjch < N_ports; port_tx_conjch++) {
        for (int ant_rx_ch = 0; ant_rx_ch < frame_parms->nb_antennas_rx; ant_rx_ch++) {
          for(uint16_t port_tx_ch = 0; port_tx_ch < N_ports; port_tx_ch++) {

            // conjch x ch computation
            nr_conjch0_mult_ch1(&csi_rs_estimated_channel_freq[ant_rx_conjch][port_tx_conjch][k_offset],
                                &csi_rs_estimated_channel_freq[ant_rx_ch][port_tx_ch][k_offset],
                                &csi_rs_estimated_conjch_ch[ant_rx_conjch][port_tx_conjch][ant_rx_ch][port_tx_ch][k_offset],
                                1,
                                log2_maxh);

            // construct Hh x H elements
            if(ant_rx_conjch == ant_rx_ch) {
              nr_a_sum_b(
                  (c16_t *)&csi_rs_estimated_A_MF[port_tx_conjch][port_tx_ch][k_offset], (c16_t *)&csi_rs_estimated_conjch_ch[ant_rx_conjch][port_tx_conjch][ant_rx_ch][port_tx_ch][k_offset], 1);
            }
          }
        }
      }
    }

    // compute the determinant of A_MF (denominator)
    nr_det_A_MF_2x2(&csi_rs_estimated_A_MF[0][0][k_offset],
                    &csi_rs_estimated_A_MF[0][1][k_offset],
                    &csi_rs_estimated_A_MF[1][0][k_offset],
                    &csi_rs_estimated_A_MF[1][1][k_offset],
                    &csi_rs_estimated_determ_fin[k_offset],
                    1);

    // compute the square of A_MF (numerator)
    nr_squared_matrix_element(&csi_rs_estimated_A_MF[0][0][k_offset], &csi_rs_estimated_A_MF_sq[0][0][k_offset], 1);
    nr_squared_matrix_element(&csi_rs_estimated_A_MF[0][1][k_offset], &csi_rs_estimated_A_MF_sq[0][1][k_offset], 1);
    nr_squared_matrix_element(&csi_rs_estimated_A_MF[1][0][k_offset], &csi_rs_estimated_A_MF_sq[1][0][k_offset], 1);
    nr_squared_matrix_element(&csi_rs_estimated_A_MF[1][1][k_offset], &csi_rs_estimated_A_MF_sq[1][1][k_offset], 1);
    nr_numer_2x2(&csi_rs_estimated_A_MF_sq[0][0][k_offset],
                 &csi_rs_estimated_A_MF_sq[0][1][k_offset],
                 &csi_rs_estimated_A_MF_sq[1][0][k_offset],
                 &csi_rs_estimated_A_MF_sq[1][1][k_offset],
                 &csi_rs_estimated_numer_fin[k_offset],
                 1);

#ifdef NR_CSIRS_DEBUG
#if 0
    // RE-level (k loop) logging disabled to reduce log explosion
    for(uint16_t port_tx_conjch = 0; port_tx_conjch < N_ports; port_tx_conjch++) {
      for(uint16_t port_tx_ch = 0; port_tx_ch < N_ports; port_tx_ch++) {
        c16_t *csi_rs_estimated_A_MF_k = (c16_t *) &csi_rs_estimated_A_MF[port_tx_conjch][port_tx_ch][k_offset];
        // RE-level logging disabled to reduce log explosion
        // LOG_I(NR_PHY, "(%i) csi_rs_estimated_A_MF[%i][%i] = (%i, %i)\n", ...);
        // LOG_I(NR_PHY, "(%i) csi_rs_estimated_A_MF_sq[%i][%i] = (%i, %i)\n", ...);
      }
    }
    // RE-level logging disabled: LOG_I(NR_PHY, "(%i) csi_rs_estimated_determ_fin = %i\n", ...);
    // RE-level logging disabled: LOG_I(NR_PHY, "(%i) csi_rs_estimated_numer_fin = %i\n", ...);
#endif
#endif

    // compute the conditional number
    for (int sc_idx=0; sc_idx < NR_NB_SC_PER_RB; sc_idx++) {
      int8_t csi_rs_estimated_denum_db = dB_fixed(csi_rs_estimated_determ_fin[k_offset + sc_idx]);
      int8_t csi_rs_estimated_numer_db = dB_fixed(csi_rs_estimated_numer_fin[k_offset + sc_idx]>>sum_shift);
      int8_t cond_db = csi_rs_estimated_numer_db - csi_rs_estimated_denum_db;

#ifdef NR_CSIRS_DEBUG
      LOG_I(NR_PHY, "csi_rs_estimated_denum_db = %i\n", csi_rs_estimated_denum_db);
      LOG_I(NR_PHY, "csi_rs_estimated_numer_db = %i\n", csi_rs_estimated_numer_db);
      LOG_I(NR_PHY, "cond_db = %i\n", cond_db);
#endif

      if (cond_db < cond_dB_threshold) {
        count++;
      } else {
        count--;
      }
    }
  }

  // conditional number is lower than cond_dB_threshold in half on more REs
  if (count > 0) {
    *rank_indicator = 1;
  }

#ifdef NR_CSIRS_DEBUG
  LOG_I(NR_PHY, "count = %i\n", count);
  LOG_I(NR_PHY, "rank = %i\n", (*rank_indicator)+1);
#endif

  return 0;
}

int nr_csi_rs_pmi_estimation(const PHY_VARS_NR_UE *ue,
                             const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
                             const nr_csi_info_t *nr_csi_info,
                             const uint8_t N_ports,
                             uint8_t mem_offset,
                             const c16_t csi_rs_estimated_channel_freq[][N_ports][ue->frame_parms.ofdm_symbol_size + FILTER_MARGIN],
                             const uint32_t interference_plus_noise_power,
                             const uint8_t rank_indicator,
                             const int16_t log2_re,
                             uint8_t *i1,
                             uint8_t *i2,
                             uint32_t *precoded_sinr_dB)
{
  const NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;

  // i1 is a three-element vector in the form of [i11 i12 i13], when CodebookType is specified as 'Type1SinglePanel'.
  // Note that i13 is not applicable when the number of transmission layers is one of {1, 5, 6, 7, 8}.
  // i2, for 'Type1SinglePanel' codebook type, it is a scalar when PMIMode is specified as 'wideband', and when PMIMode
  // is specified as 'subband' or when PRGSize, the length of the i2 vector equals to the number of subbands or PRGs.
  // Note that when the number of CSI-RS ports is 2, the applicable codebook type is 'Type1SinglePanel'. In this case,
  // the precoding matrix is obtained by a single index (i2 field here) based on TS 38.214 Table 5.2.2.2.1-1.
  // The first column is applicable if the UE is reporting a Rank = 1, whereas the second column is applicable if the
  // UE is reporting a Rank = 2.

#if CSI_LOG_CSIRS_VERBOSE
  // Note: PMI input logging removed - will be included in input summary log
  // All RE-level logs have been removed to reduce log volume
#endif // CSI_LOG_CSIRS_VERBOSE

  if (interference_plus_noise_power == 0) {
#if CSI_LOG_CSIRS_VERBOSE
    // Verbose PMI warning (only when CSI_LOG_CSIRS_VERBOSE enabled)
    LOG_W(NR_PHY, "[PMI] interference_plus_noise_power=0, skipping PMI estimation\n");
#endif
    return 0;
  }

  if (N_ports == 1) {
    // SISO case: SINR = E[|h|^2] / noise_power. No PMI to estimate.
    int64_t signal_power = 0;
    int count = 0;

    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;

      const c16_t h = csi_rs_estimated_channel_freq[0][0][k_offset];
      signal_power += (int64_t)h.r * h.r + (int64_t)h.i * h.i;
      count++;
    }

        if (count > 0) {
      const int64_t avg_signal_power = signal_power / count; // Q30 (Q15^2) 가정

      // SINR(dB) = 10log10(signal) - 10log10(interference+noise)
      // (정수 나눗셈으로 sinr_linear가 0이 되는 문제를 피하기 위해 dB 영역에서 계산)
      int32_t sinr_dB = dB_fixed((uint32_t)(avg_signal_power > 0 ? avg_signal_power : 1))
                        - dB_fixed(interference_plus_noise_power);

      if (sinr_dB < 0)
        sinr_dB = 0;

      *precoded_sinr_dB = (uint32_t)sinr_dB;

#ifdef NR_CSIRS_DEBUG
      // 디버그용: Q20 고정소수점 비율도 같이 출력(값 스케일 확인용)
      const int32_t Q = 20;
      const uint32_t ratio_q20 =
          (avg_signal_power > 0)
              ? (uint32_t)(((uint64_t)avg_signal_power << Q) / interference_plus_noise_power)
              : 0;

      LOG_I(NR_PHY,
            "[SISO SINR] count=%d avg_sig=%ld intf=%u ratio_q20=%u sinr_dB=%d\n",
            count, avg_signal_power, interference_plus_noise_power, ratio_q20, sinr_dB);
#endif
    }


    return 0;
  }

  // Handle 2-port case (existing implementation)
  if(N_ports == 2 && (rank_indicator == 0 || rank_indicator == 1)) {
        // PMI 후보별 E[|h_eff|^2] 누적 (샘플 단위 제곱 누적)
    int64_t sumsq_re[4] = {0};
    int64_t sumsq_im[4] = {0};
    int64_t tested_precoded_sinr[4] = {0}; // Q20 선형 SINR 저장
    int count = 0;

    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {

      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }

      uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;

      for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
        const c16_t p0 = csi_rs_estimated_channel_freq[ant_rx][0][k_offset];
        const c16_t p1 = csi_rs_estimated_channel_freq[ant_rx][1][k_offset];

        // 4개 PMI 후보(2-port Type1SinglePanel의 대표 조합)용 effective channel
        // cand0: H_p0 + 1*H_p1
        const int32_t t0_re = (int32_t)p0.r + (int32_t)p1.r;
        const int32_t t0_im = (int32_t)p0.i + (int32_t)p1.i;

        // cand1: H_p0 + 1j*H_p1
        const int32_t t1_re = (int32_t)p0.r - (int32_t)p1.i;
        const int32_t t1_im = (int32_t)p0.i + (int32_t)p1.r;

        // cand2: H_p0 - 1*H_p1
        const int32_t t2_re = (int32_t)p0.r - (int32_t)p1.r;
        const int32_t t2_im = (int32_t)p0.i - (int32_t)p1.i;

        // cand3: H_p0 - 1j*H_p1
        const int32_t t3_re = (int32_t)p0.r + (int32_t)p1.i;
        const int32_t t3_im = (int32_t)p0.i - (int32_t)p1.r;

        // 샘플 단위 제곱을 64비트로 누적(오버플로우 방지)
        sumsq_re[0] += (int64_t)t0_re * t0_re;
        sumsq_im[0] += (int64_t)t0_im * t0_im;

        sumsq_re[1] += (int64_t)t1_re * t1_re;
        sumsq_im[1] += (int64_t)t1_im * t1_im;

        sumsq_re[2] += (int64_t)t2_re * t2_re;
        sumsq_im[2] += (int64_t)t2_im * t2_im;

        sumsq_re[3] += (int64_t)t3_re * t3_re;
        sumsq_im[3] += (int64_t)t3_im * t3_im;

        count++;
      }
    }

    if (count == 0) {
#ifdef NR_CSIRS_DEBUG
#if CSI_LOG_CSIRS_VERBOSE
      // Verbose PMI logging (only when CSI_LOG_CSIRS_VERBOSE enabled)
      LOG_I(NR_PHY, "[PMI SINR] count==0 (no valid RE)\n");
#endif
#endif
      return 0;
    }

    // Q20 비율로 SINR 선형값 계산: sinr_q20 = (avg_pow << 20) / (interf+noise)
  const int32_t Q = 20;

for (int p = 0; p < 4; p++) {
  const int64_t avg_pow_q30 = (sumsq_re[p] + sumsq_im[p]) / count; // Q30

  if (count <= 0 || avg_pow_q30 <= 0 || interference_plus_noise_power <= 0) {
    tested_precoded_sinr[p] = 0;
#if 0
    // RE-level (for(p) loop) logging disabled to reduce log explosion
    LOG_I(PHY, "[PMI-SINR-ZERO] p=%d count=%d avg_pow_q30=%ld intN=%ld\n",
          p, count, avg_pow_q30, interference_plus_noise_power);
#endif
    continue;
  }

  tested_precoded_sinr[p] =
      (uint32_t)(((uint64_t)avg_pow_q30 << Q) / (uint64_t)interference_plus_noise_power);
  
#ifdef NR_CSIRS_DEBUG
#if 0
  // RE-level (for(p) loop) logging disabled to reduce log explosion
  LOG_I(NR_PHY, "[PMI SINR] p=%d count=%d avg_pow_q30=%ld intf=%u sinr_q20=%u (sinr_dB=%d)\n",
        p, count, avg_pow_q30, interference_plus_noise_power, tested_precoded_sinr[p],
        (tested_precoded_sinr[p] > 0) ? dB_fixed(tested_precoded_sinr[p]) : 0);
#endif
#endif
}


    if(rank_indicator == 0) {
      i2[0] = 0;
      for(int tested_i2 = 0; tested_i2 < 4; tested_i2++) {
        if(tested_precoded_sinr[tested_i2] > tested_precoded_sinr[i2[0]]) {
          i2[0] = tested_i2;
        }
      }
      *precoded_sinr_dB = dB_fixed(tested_precoded_sinr[i2[0]]);
    } else {
      i2[0] = tested_precoded_sinr[0]+tested_precoded_sinr[2] > tested_precoded_sinr[1]+tested_precoded_sinr[3] ? 0 : 1;
      *precoded_sinr_dB = dB_fixed((tested_precoded_sinr[i2[0]] + tested_precoded_sinr[i2[0]+2])>>1);
    }

  } 
  // Handle 4-port, rank=1 case (RI=1, 1-layer)
  else if(N_ports == 4 && rank_indicator == 0) {
    // ========================================================================
    // 4-port, Rank=1 Type-I Single-Panel codebook implementation (approximate)
    // Based on TS 38.214 Table 5.2.2.2.1-3 (reusing rank=2 codebook structure)
    // ========================================================================
    // 
    // Implementation approach:
    //   - Reuse the rank=2 codebook structure (W = W1 * W2) with i1=[0,0,0]
    //   - For RI=1, we evaluate both columns (w0 and w1) of the rank=2 precoding matrix
    //   - For each i2 candidate (0-15), compute power for both w0 and w1
    //   - Select the stronger column (w0 or w1) for each i2 candidate
    //   - Choose the i2 candidate with maximum power
    // 
    // Note: This is an approximate implementation. The standard TS 38.214 Type-I
    // Single-Panel 4-port RI=1 codebook has a different structure, but this approach
    // provides a practical solution that reuses existing rank=2 codebook logic.
    // ========================================================================
    
    // Initialize i1 to default values (beam pair 0,1)
    i1[0] = 0; // i11
    i1[1] = 0; // i12
    i1[2] = 0; // i13 (not used for rank=1)
    
    // 16 i2 candidates for 4-port rank=1 codebook (reusing rank=2 structure)
    // For each i2, we test both w0 and w1 columns and select the stronger one
    int64_t tested_precoded_power[16] = {0}; // Accumulated power for each i2 candidate (best column)
    int count_per_candidate[16] = {0}; // Count of samples for each i2 candidate
    
    // For each RB, compute effective channel for all 16 i2 candidates
    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      
      uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;
      
      // Get channel matrix H for this subcarrier: H is nb_antennas_rx x N_ports
      // For 4-port case: H[ant_rx][port] where port = 0,1,2,3
      for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
        const c16_t h0 = csi_rs_estimated_channel_freq[ant_rx][0][k_offset]; // H[ant_rx][port0]
        const c16_t h1 = csi_rs_estimated_channel_freq[ant_rx][1][k_offset]; // H[ant_rx][port1]
        const c16_t h2 = csi_rs_estimated_channel_freq[ant_rx][2][k_offset]; // H[ant_rx][port2]
        const c16_t h3 = csi_rs_estimated_channel_freq[ant_rx][3][k_offset]; // H[ant_rx][port3]
        
        // For each i2 candidate (0-15), compute W2 and then W = W1 * W2
        for (int i2_cand = 0; i2_cand < 16; i2_cand++) {
          // Extract co-phasing bits from i2 (same as rank=2)
          int b0 = (i2_cand >> 0) & 0x1;
          int b1 = (i2_cand >> 1) & 0x1;
          int b2 = (i2_cand >> 2) & 0x1;
          int b3 = (i2_cand >> 3) & 0x1;
          
          // Co-phasing factors: phi = exp(j * pi * b / 2)
          // phi = 1 if b=0, phi = j if b=1
          // In fixed-point Q15: 1 = (32767, 0), j = (0, 32767)
          int32_t phi0_re = (b0 == 0) ? 32767 : 0;
          int32_t phi0_im = (b0 == 0) ? 0 : 32767;
          int32_t phi1_re = (b1 == 0) ? 32767 : 0;
          int32_t phi1_im = (b1 == 0) ? 0 : 32767;
          int32_t phi2_re = (b2 == 0) ? 32767 : 0;
          int32_t phi2_im = (b2 == 0) ? 0 : 32767;
          int32_t phi3_re = (b3 == 0) ? 32767 : 0;
          int32_t phi3_im = (b3 == 0) ? 0 : 32767;
          
          // Compute precoding matrix columns w0 and w1 (same as rank=2)
          int32_t w0_0_re = (phi0_re + phi2_re) >> 1; // (phi0 + phi2) / 2
          int32_t w0_0_im = (phi0_im + phi2_im) >> 1;
          int32_t w0_1_re = (phi0_re - phi2_re) >> 1; // (phi0 - phi2) / 2
          int32_t w0_1_im = (phi0_im - phi2_im) >> 1;
          int32_t w0_2_re = (phi0_re + phi2_re) >> 1;
          int32_t w0_2_im = (phi0_im + phi2_im) >> 1;
          int32_t w0_3_re = (phi0_re - phi2_re) >> 1;
          int32_t w0_3_im = (phi0_im - phi2_im) >> 1;
          
          int32_t w1_0_re = (phi1_re + phi3_re) >> 1; // (phi1 + phi3) / 2
          int32_t w1_0_im = (phi1_im + phi3_im) >> 1;
          int32_t w1_1_re = (phi1_re - phi3_re) >> 1; // (phi1 - phi3) / 2
          int32_t w1_1_im = (phi1_im - phi3_im) >> 1;
          int32_t w1_2_re = (phi1_re + phi3_re) >> 1;
          int32_t w1_2_im = (phi1_im + phi3_im) >> 1;
          int32_t w1_3_re = (phi1_re - phi3_re) >> 1;
          int32_t w1_3_im = (phi1_im - phi3_im) >> 1;
          
          // Compute effective channel for column w0: Heff0 = H * w0
          int32_t heff0_re = ((int32_t)h0.r * w0_0_re - (int32_t)h0.i * w0_0_im) >> 15;
          heff0_re += ((int32_t)h1.r * w0_1_re - (int32_t)h1.i * w0_1_im) >> 15;
          heff0_re += ((int32_t)h2.r * w0_2_re - (int32_t)h2.i * w0_2_im) >> 15;
          heff0_re += ((int32_t)h3.r * w0_3_re - (int32_t)h3.i * w0_3_im) >> 15;
          
          int32_t heff0_im = ((int32_t)h0.r * w0_0_im + (int32_t)h0.i * w0_0_re) >> 15;
          heff0_im += ((int32_t)h1.r * w0_1_im + (int32_t)h1.i * w0_1_re) >> 15;
          heff0_im += ((int32_t)h2.r * w0_2_im + (int32_t)h2.i * w0_2_re) >> 15;
          heff0_im += ((int32_t)h3.r * w0_3_im + (int32_t)h3.i * w0_3_re) >> 15;
          
          // Compute effective channel for column w1: Heff1 = H * w1
          int32_t heff1_re = ((int32_t)h0.r * w1_0_re - (int32_t)h0.i * w1_0_im) >> 15;
          heff1_re += ((int32_t)h1.r * w1_1_re - (int32_t)h1.i * w1_1_im) >> 15;
          heff1_re += ((int32_t)h2.r * w1_2_re - (int32_t)h2.i * w1_2_im) >> 15;
          heff1_re += ((int32_t)h3.r * w1_3_re - (int32_t)h3.i * w1_3_im) >> 15;
          
          int32_t heff1_im = ((int32_t)h0.r * w1_0_im + (int32_t)h0.i * w1_0_re) >> 15;
          heff1_im += ((int32_t)h1.r * w1_1_im + (int32_t)h1.i * w1_1_re) >> 15;
          heff1_im += ((int32_t)h2.r * w1_2_im + (int32_t)h2.i * w1_2_re) >> 15;
          heff1_im += ((int32_t)h3.r * w1_3_im + (int32_t)h3.i * w1_3_re) >> 15;
          
          // Compute |Heff0|^2 and |Heff1|^2
          int64_t layer0_power = (int64_t)heff0_re * heff0_re + (int64_t)heff0_im * heff0_im;
          int64_t layer1_power = (int64_t)heff1_re * heff1_re + (int64_t)heff1_im * heff1_im;
          
          // For RI=1, select the stronger column (w0 or w1) for this i2 candidate
          int64_t best_power = (layer0_power > layer1_power) ? layer0_power : layer1_power;
          
          // Accumulate power for this i2 candidate (using the stronger column)
          tested_precoded_power[i2_cand] += best_power;
          count_per_candidate[i2_cand]++; // Count RB x antenna combinations for this i2_cand
        }
      }
    }
    
    // Check if we have valid samples (defensive logic)
    int total_count = 0;
    for (int i = 0; i < 16; i++) {
      total_count += count_per_candidate[i];
    }
    
    if (total_count == 0) {
      // No valid samples - return without warning (defensive logic to prevent log explosion)
      return 0;
    }
    
    // Compute average power and SINR for each i2 candidate
    const int32_t Q = 20;
    uint32_t tested_precoded_sinr[16] = {0}; // Q20 linear SINR storage
    for (int i2_cand = 0; i2_cand < 16; i2_cand++) {
      if (count_per_candidate[i2_cand] == 0) {
        tested_precoded_sinr[i2_cand] = 0;
        continue;
      }
      
      const int64_t avg_pow_q30 = tested_precoded_power[i2_cand] / count_per_candidate[i2_cand]; // Q30
      
      if (avg_pow_q30 <= 0 || interference_plus_noise_power <= 0) {
        tested_precoded_sinr[i2_cand] = 0;
        continue;
      }
      
      // SINR = avg_power / interference_plus_noise_power (single layer, no division by 2)
      // Convert to Q20: SINR_q20 = (avg_pow_q30 * 2^20) / interference_plus_noise_power
      uint64_t numerator = (uint64_t)avg_pow_q30 << Q; // Q30 * 2^20 = Q50
      if (numerator < (uint64_t)interference_plus_noise_power) {
        // If numerator is too small, result will be 0 after division
        tested_precoded_sinr[i2_cand] = 0;
        continue;
      }
      
      tested_precoded_sinr[i2_cand] = (uint32_t)(numerator / (uint64_t)interference_plus_noise_power);
    }
    
    // Select i2 with maximum SINR
    i2[0] = 0;
    for (int i2_cand = 1; i2_cand < 16; i2_cand++) {
      if (tested_precoded_sinr[i2_cand] > tested_precoded_sinr[i2[0]]) {
        i2[0] = i2_cand;
      }
    }
    
    *precoded_sinr_dB = dB_fixed(tested_precoded_sinr[i2[0]]);
    
  }
  // Handle 4-port, rank=2 case (new implementation)
  else if(N_ports == 4 && rank_indicator == 1) {
    // ========================================================================
    // 4-port, Rank=2 Type-I Single-Panel codebook implementation
    // Based on TS 38.214 Table 5.2.2.2.1-3
    // ========================================================================
    // 
    // Codebook structure: W = W1 * W2
    //   - W1: beam selection matrix (4x2) - determined by i1 = [i11, i12, i13]
    //   - W2: co-phasing matrix (2x2) - determined by i2 (0-15)
    //
    // For beam pair (0,1) with i1=[0,0,0]:
    //   W1 = [v0, v1] where:
    //     v0 = [1, 1, 1, 1]^T / 2  (first beam)
    //     v1 = [1, -1, 1, -1]^T / 2  (second beam)
    //
    // W2 co-phasing matrix (2x2) for rank=2:
    //   W2 = [[phi0, phi1], [phi2, phi3]] / sqrt(2)
    //   where phi_i = exp(j * pi * b_i / 2), b_i is i-th bit of i2
    //
    // Final precoding matrix W = W1 * W2 (4x2):
    //   W = [w0, w1] where:
    //     w0 = [phi0+phi2, phi0-phi2, phi0+phi2, phi0-phi2]^T / 2
    //     w1 = [phi1+phi3, phi1-phi3, phi1+phi3, phi1-phi3]^T / 2
    //
    // Implementation note:
    //   - i1 is fixed to [0,0,0] to select beam pair (0,1) (simplified)
    //   - All 16 i2 candidates (0-15) are tested
    //   - Each i2 corresponds to a different co-phasing combination
    //   - The candidate with maximum average SINR across 2 layers is selected
    // ========================================================================
    
    // Initialize i1 to default values (beam pair 0,1)
    i1[0] = 0; // i11
    i1[1] = 0; // i12
    i1[2] = 0; // i13 (not used for rank=2)
    
    // 16 i2 candidates for 4-port rank=2 codebook
    // Each i2 corresponds to a different co-phasing combination
    int64_t tested_precoded_power[16] = {0}; // Accumulated power for each i2 candidate
    int count_per_candidate[16] = {0}; // Count of samples for each i2 candidate
    
#if CSI_LOG_CSIRS_VERBOSE
    // Channel matrix H statistics for debugging (only when verbose logging enabled)
    int64_t h_power_sum[4] = {0}; // Sum of |H|^2 per port
    int64_t h_power_max[4] = {0}; // Max |H|^2 per port
    int64_t h_power_min[4] = {INT64_MAX, INT64_MAX, INT64_MAX, INT64_MAX}; // Min |H|^2 per port
    int h_sample_count = 0;
    uint16_t max_h_re_indices[4] = {0}; // RE indices with max |H| per port
    int64_t max_h_power_global = 0;
    uint16_t max_h_re_global = 0;
#endif // CSI_LOG_CSIRS_VERBOSE
    
    // Helper function to compute 4-port rank=2 precoding matrix W for given i2
    // Based on TS 38.214 Table 5.2.2.2.1-3
    // W is a 4x2 matrix: W = [w0, w1] where w0 and w1 are 4x1 column vectors
    // For beam pair (0,1) with i1=[0,0], W1 is:
    //   W1 = [v0, v1] where v0 = [1, 1, 1, 1]^T / 2, v1 = [1, -1, 1, -1]^T / 2
    // W2 co-phasing matrix depends on i2:
    //   i2 bits: [b0, b1, b2, b3]
    //   phi0 = exp(j * pi * b0 / 2)
    //   phi1 = exp(j * pi * b1 / 2)
    //   phi2 = exp(j * pi * b2 / 2)
    //   phi3 = exp(j * pi * b3 / 2)
    
    // For each RB, compute effective channel for all 16 i2 candidates
    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      
      uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;
      
      // Get channel matrix H for this subcarrier: H is nb_antennas_rx x N_ports
      // For 4-port case: H[ant_rx][port] where port = 0,1,2,3
      for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
        const c16_t h0 = csi_rs_estimated_channel_freq[ant_rx][0][k_offset]; // H[ant_rx][port0]
        const c16_t h1 = csi_rs_estimated_channel_freq[ant_rx][1][k_offset]; // H[ant_rx][port1]
        const c16_t h2 = csi_rs_estimated_channel_freq[ant_rx][2][k_offset]; // H[ant_rx][port2]
        const c16_t h3 = csi_rs_estimated_channel_freq[ant_rx][3][k_offset]; // H[ant_rx][port3]
        
#if CSI_LOG_CSIRS_VERBOSE
        // Collect H statistics (rate-limited: only first antenna, first few REs)
        if (ant_rx == 0 && h_sample_count < 4) {
          int64_t h0_pow = (int64_t)h0.r * h0.r + (int64_t)h0.i * h0.i;
          int64_t h1_pow = (int64_t)h1.r * h1.r + (int64_t)h1.i * h1.i;
          int64_t h2_pow = (int64_t)h2.r * h2.r + (int64_t)h2.i * h2.i;
          int64_t h3_pow = (int64_t)h3.r * h3.r + (int64_t)h3.i * h3.i;
          
          // RE-level H sample logging removed - will be included in DETAIL log when triggered
          // Accumulate statistics
          h_power_sum[0] += h0_pow;
          h_power_sum[1] += h1_pow;
          h_power_sum[2] += h2_pow;
          h_power_sum[3] += h3_pow;
          
          if (h0_pow > h_power_max[0]) { h_power_max[0] = h0_pow; max_h_re_indices[0] = k_offset; }
          if (h1_pow > h_power_max[1]) { h_power_max[1] = h1_pow; max_h_re_indices[1] = k_offset; }
          if (h2_pow > h_power_max[2]) { h_power_max[2] = h2_pow; max_h_re_indices[2] = k_offset; }
          if (h3_pow > h_power_max[3]) { h_power_max[3] = h3_pow; max_h_re_indices[3] = k_offset; }
          
          if (h0_pow < h_power_min[0]) h_power_min[0] = h0_pow;
          if (h1_pow < h_power_min[1]) h_power_min[1] = h1_pow;
          if (h2_pow < h_power_min[2]) h_power_min[2] = h2_pow;
          if (h3_pow < h_power_min[3]) h_power_min[3] = h3_pow;
          
          int64_t total_pow = h0_pow + h1_pow + h2_pow + h3_pow;
          if (total_pow > max_h_power_global) {
            max_h_power_global = total_pow;
            max_h_re_global = k_offset;
          }
          
          h_sample_count++;
        }
#endif // CSI_LOG_CSIRS_VERBOSE
        
        // For beam pair (0,1) with i1=[0,0], W1 vectors are:
        // v0 = [1, 1, 1, 1]^T / 2
        // v1 = [1, -1, 1, -1]^T / 2
        // So W1 = [v0, v1] = [[1,1,1,1]^T, [1,-1,1,-1]^T] / 2
        
        // For each i2 candidate (0-15), compute W2 and then W = W1 * W2
        for (int i2_cand = 0; i2_cand < 16; i2_cand++) {
          // Extract co-phasing bits from i2
          // i2 encoding: 4 bits [b0, b1, b2, b3]
          // phi0 = exp(j * pi * b0 / 2), phi1 = exp(j * pi * b1 / 2)
          // phi2 = exp(j * pi * b2 / 2), phi3 = exp(j * pi * b3 / 2)
          int b0 = (i2_cand >> 0) & 0x1;
          int b1 = (i2_cand >> 1) & 0x1;
          int b2 = (i2_cand >> 2) & 0x1;
          int b3 = (i2_cand >> 3) & 0x1;
          
          // Co-phasing factors: phi = exp(j * pi * b / 2)
          // phi = 1 if b=0, phi = j if b=1
          // In fixed-point Q15: 1 = (32767, 0), j = (0, 32767)
          int32_t phi0_re = (b0 == 0) ? 32767 : 0;
          int32_t phi0_im = (b0 == 0) ? 0 : 32767;
          int32_t phi1_re = (b1 == 0) ? 32767 : 0;
          int32_t phi1_im = (b1 == 0) ? 0 : 32767;
          int32_t phi2_re = (b2 == 0) ? 32767 : 0;
          int32_t phi2_im = (b2 == 0) ? 0 : 32767;
          int32_t phi3_re = (b3 == 0) ? 32767 : 0;
          int32_t phi3_im = (b3 == 0) ? 0 : 32767;
          
          // W2 matrix for rank=2 (2x2):
          // W2 = [[phi0, phi1], [phi2, phi3]] / sqrt(2)
          // But for simplicity, we incorporate 1/sqrt(2) into the final normalization
          
          // Compute W = W1 * W2
          // W1 = [[1,1,1,1]^T, [1,-1,1,-1]^T] / 2
          // W2 = [[phi0, phi1], [phi2, phi3]]
          // W = W1 * W2 = [[w0, w1]] where:
          // w0 = [phi0+phi2, phi0-phi2, phi0+phi2, phi0-phi2]^T / 2
          // w1 = [phi1+phi3, phi1-phi3, phi1+phi3, phi1-phi3]^T / 2
          
          // Compute precoding matrix columns w0 and w1
          // w0[port] = (phi0 + phi2) / 2 for port in {0,2}, (phi0 - phi2) / 2 for port in {1,3}
          // w1[port] = (phi1 + phi3) / 2 for port in {0,2}, (phi1 - phi3) / 2 for port in {1,3}
          int32_t w0_0_re = (phi0_re + phi2_re) >> 1; // (phi0 + phi2) / 2
          int32_t w0_0_im = (phi0_im + phi2_im) >> 1;
          int32_t w0_1_re = (phi0_re - phi2_re) >> 1; // (phi0 - phi2) / 2
          int32_t w0_1_im = (phi0_im - phi2_im) >> 1;
          int32_t w0_2_re = (phi0_re + phi2_re) >> 1;
          int32_t w0_2_im = (phi0_im + phi2_im) >> 1;
          int32_t w0_3_re = (phi0_re - phi2_re) >> 1;
          int32_t w0_3_im = (phi0_im - phi2_im) >> 1;
          
          int32_t w1_0_re = (phi1_re + phi3_re) >> 1; // (phi1 + phi3) / 2
          int32_t w1_0_im = (phi1_im + phi3_im) >> 1;
          int32_t w1_1_re = (phi1_re - phi3_re) >> 1; // (phi1 - phi3) / 2
          int32_t w1_1_im = (phi1_im - phi3_im) >> 1;
          int32_t w1_2_re = (phi1_re + phi3_re) >> 1;
          int32_t w1_2_im = (phi1_im + phi3_im) >> 1;
          int32_t w1_3_re = (phi1_re - phi3_re) >> 1;
          int32_t w1_3_im = (phi1_im - phi3_im) >> 1;
          
          // Compute effective channel: Heff = H * W
          // Heff is nb_antennas_rx x 2 (2 layers)
          // Heff[ant_rx][layer] = sum(port=0 to 3) H[ant_rx][port] * W[port][layer]
          // Complex multiplication: (a+bi) * (c+di) = (ac-bd) + (ad+bc)i
          
          // Layer 0: Heff[ant_rx][0] = H[ant_rx][0]*w0[0] + H[ant_rx][1]*w0[1] + H[ant_rx][2]*w0[2] + H[ant_rx][3]*w0[3]
          // H[ant_rx][port] * w0[port]: (h.r + j*h.i) * (w0_re + j*w0_im) = (h.r*w0_re - h.i*w0_im) + j*(h.r*w0_im + h.i*w0_re)
          int32_t heff0_re = ((int32_t)h0.r * w0_0_re - (int32_t)h0.i * w0_0_im) >> 15;
          heff0_re += ((int32_t)h1.r * w0_1_re - (int32_t)h1.i * w0_1_im) >> 15;
          heff0_re += ((int32_t)h2.r * w0_2_re - (int32_t)h2.i * w0_2_im) >> 15;
          heff0_re += ((int32_t)h3.r * w0_3_re - (int32_t)h3.i * w0_3_im) >> 15;
          
          int32_t heff0_im = ((int32_t)h0.r * w0_0_im + (int32_t)h0.i * w0_0_re) >> 15;
          heff0_im += ((int32_t)h1.r * w0_1_im + (int32_t)h1.i * w0_1_re) >> 15;
          heff0_im += ((int32_t)h2.r * w0_2_im + (int32_t)h2.i * w0_2_re) >> 15;
          heff0_im += ((int32_t)h3.r * w0_3_im + (int32_t)h3.i * w0_3_re) >> 15;
          
          // Layer 1: Heff[ant_rx][1] = H[ant_rx][0]*w1[0] + H[ant_rx][1]*w1[1] + H[ant_rx][2]*w1[2] + H[ant_rx][3]*w1[3]
          int32_t heff1_re = ((int32_t)h0.r * w1_0_re - (int32_t)h0.i * w1_0_im) >> 15;
          heff1_re += ((int32_t)h1.r * w1_1_re - (int32_t)h1.i * w1_1_im) >> 15;
          heff1_re += ((int32_t)h2.r * w1_2_re - (int32_t)h2.i * w1_2_im) >> 15;
          heff1_re += ((int32_t)h3.r * w1_3_re - (int32_t)h3.i * w1_3_im) >> 15;
          
          int32_t heff1_im = ((int32_t)h0.r * w1_0_im + (int32_t)h0.i * w1_0_re) >> 15;
          heff1_im += ((int32_t)h1.r * w1_1_im + (int32_t)h1.i * w1_1_re) >> 15;
          heff1_im += ((int32_t)h2.r * w1_2_im + (int32_t)h2.i * w1_2_re) >> 15;
          heff1_im += ((int32_t)h3.r * w1_3_im + (int32_t)h3.i * w1_3_re) >> 15;
          
          // Compute |Heff[0]|^2 + |Heff[1]|^2 for this antenna
          int64_t layer0_power = (int64_t)heff0_re * heff0_re + (int64_t)heff0_im * heff0_im;
          int64_t layer1_power = (int64_t)heff1_re * heff1_re + (int64_t)heff1_im * heff1_im;
          int64_t total_power = layer0_power + layer1_power;
          
          // Accumulate power for this i2 candidate
          tested_precoded_power[i2_cand] += total_power;
          count_per_candidate[i2_cand]++; // Count RB x antenna combinations for this i2_cand
        }
      }
    }
    
    // Compute average power and SINR for each i2 candidate
    const int32_t Q = 20;
    uint32_t tested_precoded_sinr[16] = {0}; // Q20 선형 SINR 저장
    for (int i2_cand = 0; i2_cand < 16; i2_cand++) {
      if (count_per_candidate[i2_cand] == 0) {
        tested_precoded_sinr[i2_cand] = 0;
        continue;
      }
      
      const int64_t avg_pow_q30 = tested_precoded_power[i2_cand] / count_per_candidate[i2_cand]; // Q30
      
      if (avg_pow_q30 <= 0 || interference_plus_noise_power <= 0) {
        tested_precoded_sinr[i2_cand] = 0;
#ifdef NR_CSIRS_DEBUG
        // Detailed RE-level logging disabled - included in DETAIL log when triggered
        // LOG_I(NR_PHY, "[PMI 4x4 Rank2] i2_cand=%d: avg_pow_q30=%ld, intf_noise=%u (zero SINR)\n", ...);
#endif
        continue;
      }
      
      // SINR = (avg_power / 2) / interference_plus_noise_power
      // Divide by 2 because we have 2 layers (average per layer)
      // Convert to Q20: SINR_q20 = (avg_pow_q30 / 2) * 2^20 / interference_plus_noise_power
      //                = (avg_pow_q30 * 2^19) / interference_plus_noise_power
      uint64_t numerator = (uint64_t)avg_pow_q30 << (Q - 1); // Q30 * 2^19 = Q49
      if (numerator < (uint64_t)interference_plus_noise_power) {
        // If numerator is too small, result will be 0 after division
        tested_precoded_sinr[i2_cand] = 0;
#ifdef NR_CSIRS_DEBUG
        // Detailed RE-level logging disabled - included in DETAIL log when triggered
        // LOG_I(NR_PHY, "[PMI 4x4 Rank2] i2_cand=%d: numerator=%lu < denominator=%u (zero SINR)\n", ...);
#endif
        continue;
      }
      
      tested_precoded_sinr[i2_cand] = (uint32_t)(numerator / (uint64_t)interference_plus_noise_power);
    }
    
    // Select i2 with maximum SINR
    i2[0] = 0;
    for (int i2_cand = 1; i2_cand < 16; i2_cand++) {
      if (tested_precoded_sinr[i2_cand] > tested_precoded_sinr[i2[0]]) {
        i2[0] = i2_cand;
      }
    }
    
    *precoded_sinr_dB = dB_fixed(tested_precoded_sinr[i2[0]]);
    
#if CSI_LOG_CSIRS_VERBOSE
    // Find top-3 candidates for detailed logging (only when verbose logging enabled)
    struct {
      int idx;
      uint32_t sinr;
    } top3[3] = {{-1, 0}, {-1, 0}, {-1, 0}};
    
    for (int i = 0; i < 16; i++) {
      if (tested_precoded_sinr[i] > top3[2].sinr) {
        if (tested_precoded_sinr[i] > top3[0].sinr) {
          top3[2] = top3[1];
          top3[1] = top3[0];
          top3[0].idx = i;
          top3[0].sinr = tested_precoded_sinr[i];
        } else if (tested_precoded_sinr[i] > top3[1].sinr) {
          top3[2] = top3[1];
          top3[1].idx = i;
          top3[1].sinr = tested_precoded_sinr[i];
        } else {
          top3[2].idx = i;
          top3[2].sinr = tested_precoded_sinr[i];
        }
      }
    }
    
    // DETAIL log: Check trigger conditions (only when verbose logging enabled)
    // DETAIL log: Check trigger conditions (only when verbose logging enabled)
    bool detail_trigger = false;
    static uint8_t last_rank_for_detail = 255;
    static uint8_t last_i2_for_detail = 255;
    
    // Trigger conditions for DETAIL log
    if (rank_indicator != last_rank_for_detail) {
      detail_trigger = true;
      last_rank_for_detail = rank_indicator;
    }
    if (i2[0] != last_i2_for_detail) {
      detail_trigger = true;
      last_i2_for_detail = i2[0];
    }
    if (*precoded_sinr_dB == 0 || interference_plus_noise_power == 0) {
      detail_trigger = true;
    }
    
    // DETAIL log output (only when triggered)
    if (detail_trigger) {
      // Log H statistics (4-port rank=2 case)
      if (h_sample_count > 0) {
        LOG_I(NR_PHY, "[PMI DETAIL H Stats] ports=%d samples=%d: port0(avg=%ld max=%ld min=%ld RE@max=%d) port1(avg=%ld max=%ld min=%ld RE@max=%d) port2(avg=%ld max=%ld min=%ld RE@max=%d) port3(avg=%ld max=%ld min=%ld RE@max=%d) max_total_pow=%ld@RE=%d\n",
              N_ports, h_sample_count,
              h_power_sum[0] / h_sample_count, h_power_max[0], h_power_min[0], max_h_re_indices[0],
              h_power_sum[1] / h_sample_count, h_power_max[1], h_power_min[1], max_h_re_indices[1],
              h_power_sum[2] / h_sample_count, h_power_max[2], h_power_min[2], max_h_re_indices[2],
              h_power_sum[3] / h_sample_count, h_power_max[3], h_power_min[3], max_h_re_indices[3],
              max_h_power_global, max_h_re_global);
      }
      
      // Log best PMI with detailed info
      int b0 = (i2[0] >> 0) & 0x1;
      int b1 = (i2[0] >> 1) & 0x1;
      int b2 = (i2[0] >> 2) & 0x1;
      int b3 = (i2[0] >> 3) & 0x1;
      int64_t avg_pow_best = count_per_candidate[i2[0]] > 0 ? 
        tested_precoded_power[i2[0]] / count_per_candidate[i2[0]] : 0;
      LOG_I(NR_PHY, "[PMI DETAIL Best] i2=%d (bits=%d%d%d%d, phase=[%s,%s,%s,%s]) sinr_q20=%u sinr_dB=%u avg_pow_q30=%ld count=%d intf_noise=%u\n",
            i2[0], b3, b2, b1, b0,
            (b0 == 0) ? "1" : "j", (b1 == 0) ? "1" : "j", (b2 == 0) ? "1" : "j", (b3 == 0) ? "1" : "j",
            tested_precoded_sinr[i2[0]], *precoded_sinr_dB, avg_pow_best, 
            count_per_candidate[i2[0]], interference_plus_noise_power);
      
      // Log top-3 candidates
      LOG_I(NR_PHY, "[PMI DETAIL Top3] #1: i2=%d sinr_q20=%u sinr_dB=%u | #2: i2=%d sinr_q20=%u sinr_dB=%u | #3: i2=%d sinr_q20=%u sinr_dB=%u\n",
            top3[0].idx, top3[0].sinr, dB_fixed(top3[0].sinr),
            top3[1].idx >= 0 ? top3[1].idx : -1, top3[1].idx >= 0 ? top3[1].sinr : 0, top3[1].idx >= 0 ? dB_fixed(top3[1].sinr) : 0,
            top3[2].idx >= 0 ? top3[2].idx : -1, top3[2].idx >= 0 ? top3[2].sinr : 0, top3[2].idx >= 0 ? dB_fixed(top3[2].sinr) : 0);
      
      if (tested_precoded_sinr[i2[0]] == 0) {
        LOG_W(NR_PHY, "[PMI DETAIL WARNING] Selected i2=%d has zero SINR! avg_pow=%ld, count=%d\n",
              i2[0], avg_pow_best, count_per_candidate[i2[0]]);
      }
    }
#endif

#ifdef NR_CSIRS_DEBUG
#if 0
    // This log removed - information included in SUMMARY/DETAIL logs
    LOG_I(NR_PHY, "[PMI 4x4 Rank2] selected i2=%d, sinr_q20=%u, sinr_dB=%u, count=%d, intf_noise=%u\n",
          i2[0], tested_precoded_sinr[i2[0]], *precoded_sinr_dB, 
          count_per_candidate[i2[0]], interference_plus_noise_power);
    if (tested_precoded_sinr[i2[0]] == 0) {
      LOG_W(NR_PHY, "[PMI 4x4 Rank2] WARNING: Selected i2=%d has zero SINR! avg_pow=%ld, count=%d\n",
            i2[0], tested_precoded_power[i2[0]] / (count_per_candidate[i2[0]] > 0 ? count_per_candidate[i2[0]] : 1),
            count_per_candidate[i2[0]]);
    }
#endif
#endif
  } 
  else {
    LOG_W(NR_PHY, "PMI computation is not implemented for N_ports=%d, rank indicator %i\n", N_ports, rank_indicator+1);
    return -1;
  }

  return 0;
}

int nr_csi_rs_cqi_estimation(const uint32_t precoded_sinr,
                             uint8_t *cqi) {

  *cqi = 0;

  // Default SINR table for an AWGN channel for SISO scenario, considering 0.1 BLER condition and TS 38.214 Table 5.2.2.1-2
  if(precoded_sinr>0 && precoded_sinr<=2) {
    *cqi = 4;
  } else if(precoded_sinr==3) {
    *cqi = 5;
  } else if(precoded_sinr>3 && precoded_sinr<=5) {
    *cqi = 6;
  } else if(precoded_sinr>5 && precoded_sinr<=7) {
    *cqi = 7;
  } else if(precoded_sinr>7 && precoded_sinr<=9) {
    *cqi = 8;
  } else if(precoded_sinr==10) {
    *cqi = 9;
  } else if(precoded_sinr>10 && precoded_sinr<=12) {
    *cqi = 10;
  } else if(precoded_sinr>12 && precoded_sinr<=15) {
    *cqi = 11;
  } else if(precoded_sinr==16) {
    *cqi = 12;
  } else if(precoded_sinr>16 && precoded_sinr<=18) {
    *cqi = 13;
  } else if(precoded_sinr==19) {
    *cqi = 14;
  } else if(precoded_sinr>19) {
    *cqi = 15;
  }

  return 0;
}

static void nr_csi_im_power_estimation(const PHY_VARS_NR_UE *ue,
                                       const UE_nr_rxtx_proc_t *proc,
                                       const fapi_nr_dl_config_csiim_pdu_rel15_t *csiim_config_pdu,
                                       uint32_t *interference_plus_noise_power,
                                       const c16_t rxdataF[][ue->frame_parms.samples_per_slot_wCP])
{
  const NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;

  const uint16_t end_rb = csiim_config_pdu->start_rb + csiim_config_pdu->nr_of_rbs > csiim_config_pdu->bwp_size ?
                          csiim_config_pdu->bwp_size : csiim_config_pdu->start_rb + csiim_config_pdu->nr_of_rbs;

  int32_t count = 0;
  int32_t sum_re = 0;
  int32_t sum_im = 0;
  int32_t sum2_re = 0;
  int32_t sum2_im = 0;

  int l_csiim[4] = {-1, -1, -1, -1};

  for(int symb_idx = 0; symb_idx < 4; symb_idx++) {

    uint8_t symb = csiim_config_pdu->l_csiim[symb_idx];
    bool done = false;
    for (int symb_idx2 = 0; symb_idx2 < symb_idx; symb_idx2++) {
      if (l_csiim[symb_idx2] == symb) {
        done = true;
      }
    }

    if (done) {
      continue;
    }

    l_csiim[symb_idx] = symb;
    uint64_t symbol_offset = symb*frame_parms->ofdm_symbol_size;

    for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {

      const c16_t *rx_signal = &rxdataF[ant_rx][symbol_offset];

      for (int rb = csiim_config_pdu->start_rb; rb < end_rb; rb++) {

        uint16_t sc0_offset = (frame_parms->first_carrier_offset + rb*NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;

        for (int sc_idx = 0; sc_idx < 4; sc_idx++) {

          uint16_t sc = sc0_offset + csiim_config_pdu->k_csiim[sc_idx];
          if (sc >= frame_parms->ofdm_symbol_size) {
            sc -= frame_parms->ofdm_symbol_size;
          }

#ifdef NR_CSIIM_DEBUG
#if 0
          // RE-level (sc loop) logging disabled to reduce log explosion
          LOG_I(NR_PHY, "(ant_rx %i, sc %i) real %i, imag %i\n", ant_rx, sc, rx_signal[sc].r, rx_signal[sc].i);
#endif
#endif

          if (sc == 0) // skip DC for noise power estimation
            continue;
          sum_re += rx_signal[sc].r;
          sum_im += rx_signal[sc].i;
          sum2_re += rx_signal[sc].r * rx_signal[sc].r;
          sum2_im += rx_signal[sc].i * rx_signal[sc].i;
          count++;
        }
      }
    }
  }

  int32_t power_re = sum2_re / count - (sum_re / count) * (sum_re / count);
  int32_t power_im = sum2_im / count - (sum_im / count) * (sum_im / count);

  *interference_plus_noise_power = power_re + power_im;

#ifdef NR_CSIIM_DEBUG
  LOG_I(NR_PHY, "interference_plus_noise_power based on CSI-IM = %i\n", *interference_plus_noise_power);
#endif
}

void nr_ue_csi_im_procedures(PHY_VARS_NR_UE *ue,
                             const UE_nr_rxtx_proc_t *proc,
                             const c16_t rxdataF[][ue->frame_parms.samples_per_slot_wCP],
                             const fapi_nr_dl_config_csiim_pdu_rel15_t *csiim_config_pdu)
{

#ifdef NR_CSIIM_DEBUG
  LOG_I(NR_PHY, "csiim_config_pdu->bwp_size = %i\n", csiim_config_pdu->bwp_size);
  LOG_I(NR_PHY, "csiim_config_pdu->bwp_start = %i\n", csiim_config_pdu->bwp_start);
  LOG_I(NR_PHY, "csiim_config_pdu->subcarrier_spacing = %i\n", csiim_config_pdu->subcarrier_spacing);
  LOG_I(NR_PHY, "csiim_config_pdu->start_rb = %i\n", csiim_config_pdu->start_rb);
  LOG_I(NR_PHY, "csiim_config_pdu->nr_of_rbs = %i\n", csiim_config_pdu->nr_of_rbs);
  LOG_I(NR_PHY, "csiim_config_pdu->k_csiim = %i.%i.%i.%i\n", csiim_config_pdu->k_csiim[0], csiim_config_pdu->k_csiim[1], csiim_config_pdu->k_csiim[2], csiim_config_pdu->k_csiim[3]);
  LOG_I(NR_PHY, "csiim_config_pdu->l_csiim = %i.%i.%i.%i\n", csiim_config_pdu->l_csiim[0], csiim_config_pdu->l_csiim[1], csiim_config_pdu->l_csiim[2], csiim_config_pdu->l_csiim[3]);
#endif

  nr_csi_im_power_estimation(ue, proc, csiim_config_pdu, &ue->nr_csi_info->interference_plus_noise_power, rxdataF);
  ue->nr_csi_info->csi_im_meas_computed = true;
}

void nr_ue_csi_rs_procedures(PHY_VARS_NR_UE *ue,
                             const UE_nr_rxtx_proc_t *proc,
                             const c16_t rxdataF[][ue->frame_parms.samples_per_slot_wCP],
                             fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu)
{
  const NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;

#if CSI_LOG_CSIRS_VERBOSE
  // CSI-RS periodicity tracking (only when verbose logging enabled)
  int frame = proc->frame_rx;
  int slot = proc->nr_slot_rx;
  uint32_t abs_slot = calc_abs_slot(frame, slot, frame_parms->slots_per_frame);
  uint32_t delta_slot = 0;
  double delta_time_ms = 0.0;
  
  csi_rs_period_tracker_t *t = &csi_rs_tracker;
  if (t->prev_abs_slot > 0) {
    delta_slot = abs_slot - t->prev_abs_slot;
    delta_time_ms = delta_slot * calc_slot_duration_ms(frame_parms->numerology_index);
  }
  t->prev_abs_slot = abs_slot;
  t->prev_frame = frame;
  t->prev_slot = slot;
#endif

#ifdef NR_CSIRS_DEBUG
#if CSI_LOG_CSIRS_VERBOSE
  // Verbose CSI-RS config dump (only when CSI_LOG_CSIRS_VERBOSE enabled)
  LOG_I(NR_PHY, "csirs_config_pdu->subcarrier_spacing = %i\n", csirs_config_pdu->subcarrier_spacing);
  LOG_I(NR_PHY, "csirs_config_pdu->cyclic_prefix = %i\n", csirs_config_pdu->cyclic_prefix);
  LOG_I(NR_PHY, "csirs_config_pdu->start_rb = %i\n", csirs_config_pdu->start_rb);
  LOG_I(NR_PHY, "csirs_config_pdu->nr_of_rbs = %i\n", csirs_config_pdu->nr_of_rbs);
  LOG_I(NR_PHY, "csirs_config_pdu->csi_type = %i (0:TRS, 1:CSI-RS NZP, 2:CSI-RS ZP)\n", csirs_config_pdu->csi_type);
  LOG_I(NR_PHY, "csirs_config_pdu->row = %i\n", csirs_config_pdu->row);
  LOG_I(NR_PHY, "csirs_config_pdu->freq_domain = %i\n", csirs_config_pdu->freq_domain);
  LOG_I(NR_PHY, "csirs_config_pdu->symb_l0 = %i\n", csirs_config_pdu->symb_l0);
  LOG_I(NR_PHY, "csirs_config_pdu->symb_l1 = %i\n", csirs_config_pdu->symb_l1);
  LOG_I(NR_PHY, "csirs_config_pdu->cdm_type = %i\n", csirs_config_pdu->cdm_type);
  LOG_I(NR_PHY, "csirs_config_pdu->freq_density = %i (0: dot5 (even RB), 1: dot5 (odd RB), 2: one, 3: three)\n", csirs_config_pdu->freq_density);
  LOG_I(NR_PHY, "csirs_config_pdu->scramb_id = %i\n", csirs_config_pdu->scramb_id);
  LOG_I(NR_PHY, "csirs_config_pdu->power_control_offset = %i\n", csirs_config_pdu->power_control_offset);
  LOG_I(NR_PHY, "csirs_config_pdu->power_control_offset_ss = %i\n", csirs_config_pdu->power_control_offset_ss);
#endif
#endif

  if(csirs_config_pdu->csi_type == 0) {
    LOG_E(NR_PHY, "Handling of CSI-RS for tracking not handled yet at PHY\n");
    return;
  }

  if(csirs_config_pdu->csi_type == 2) {
    LOG_E(NR_PHY, "Handling of ZP CSI-RS not handled yet at PHY\n");
    return;
  }

  csi_mapping_parms_t mapping_parms = get_csi_mapping_parms(csirs_config_pdu->row,
                                                            csirs_config_pdu->freq_domain,
                                                            csirs_config_pdu->symb_l0,
                                                            csirs_config_pdu->symb_l1);
#if CSI_LOG_CSIRS_VERBOSE
  // Log periodicity with actual ports information (only when verbose logging enabled)
  log_csi_rs_period_stats(frame_parms, frame, slot, delta_slot, delta_time_ms,
                          mapping_parms.ports, csirs_config_pdu->row, csirs_config_pdu->freq_density);
#endif
  nr_csi_info_t *csi_info = ue->nr_csi_info;
  nr_generate_csi_rs(frame_parms,
                     &mapping_parms,
                     AMP,
                     proc->nr_slot_rx,
                     csirs_config_pdu->freq_density,
                     csirs_config_pdu->start_rb,
                     csirs_config_pdu->nr_of_rbs,
                     csirs_config_pdu->symb_l0,
                     csirs_config_pdu->symb_l1,
                     csirs_config_pdu->row,
                     csirs_config_pdu->scramb_id,
                     csirs_config_pdu->power_control_offset_ss,
                     csirs_config_pdu->cdm_type,
                     csi_info->csi_rs_generated_signal);

  csi_info->csi_rs_generated_signal_bits = log2_approx(AMP);

  c16_t csi_rs_ls_estimated_channel[frame_parms->nb_antennas_rx][mapping_parms.ports][frame_parms->ofdm_symbol_size];
  c16_t csi_rs_estimated_channel_freq[frame_parms->nb_antennas_rx][mapping_parms.ports]
                                     [frame_parms->ofdm_symbol_size + FILTER_MARGIN];

  // (long)&csi_rs_estimated_channel_freq[0][0][frame_parms->first_carrier_offset] & 0x1F
  // gives us the remainder of the integer division by 32 of the memory address
  // By subtracting the previous value of 32, we know how much is left to have a multiple of 32.
  // Doing >> 2 <=> /sizeof(int32_t), we know what is the index offset of the array.
  uint8_t mem_offset = (((32 - ((long)&csi_rs_estimated_channel_freq[0][0][frame_parms->first_carrier_offset])) & 0x1F) >> 2);
  int CDM_group_size = get_cdm_group_size(csirs_config_pdu->cdm_type);
  c16_t csi_rs_received_signal[frame_parms->nb_antennas_rx][frame_parms->samples_per_slot_wCP];
  uint32_t rsrp = 0;
  int rsrp_dBm = 0;
  nr_get_csi_rs_signal(ue,
                       proc,
                       csirs_config_pdu,
                       csi_info,
                       &mapping_parms,
                       CDM_group_size,
                       csi_rs_received_signal,
                       &rsrp,
                       &rsrp_dBm,
                       rxdataF);


  if (csirs_config_pdu->measurement_bitmap == 0) {
    LOG_D(NR_PHY, "No CSI-RS measurements configured\n");
    return;
  }

  uint32_t noise_power = 0;
  int16_t log2_re = 0;
  int16_t log2_maxh = 0;
  // if we need to measure only RSRP no need to do channel estimation
  if (csirs_config_pdu->measurement_bitmap > 1)
    nr_csi_rs_channel_estimation(frame_parms,
                                 proc,
                                 csirs_config_pdu,
                                 csi_info,
                                 (const c16_t **)csi_info->csi_rs_generated_signal,
                                 csi_rs_received_signal,
                                 &mapping_parms,
                                 CDM_group_size,
                                 mem_offset,
                                 csi_rs_ls_estimated_channel,
                                 csi_rs_estimated_channel_freq,
                                 &log2_re,
                                 &log2_maxh,
                                 &noise_power);

  uint8_t rank_indicator = 0;
  // bit 1 in bitmap to indicate RI measurment
  if (csirs_config_pdu->measurement_bitmap & 2) {
    nr_csi_rs_ri_estimation(ue,
                            csirs_config_pdu,
                            csi_info,
                            mapping_parms.ports,
                            mem_offset,
                            csi_rs_estimated_channel_freq,
                            log2_maxh,
                            &rank_indicator);
  }

  uint8_t i1[3] = {0};
  uint8_t i2[1] = {0};
  uint8_t cqi = 0;
  uint32_t precoded_sinr_dB = 0;
  // bit 3 in bitmap to indicate RI measurment
  if (csirs_config_pdu->measurement_bitmap & 8) {
    nr_csi_rs_pmi_estimation(ue,
                             csirs_config_pdu,
                             csi_info,
                             mapping_parms.ports,
                             mem_offset,
                             csi_rs_estimated_channel_freq,
                             csi_info->csi_im_meas_computed ? csi_info->interference_plus_noise_power : noise_power,
                             rank_indicator,
                             log2_re,
                             i1,
                             i2,
                             &precoded_sinr_dB);

    // bit 4 in bitmap to indicate RI measurment
    if(csirs_config_pdu->measurement_bitmap & 16)
      nr_csi_rs_cqi_estimation(precoded_sinr_dB, &cqi);
  }

  // ============================================================================
  // Input Summary Log Output (RI/PMI/SINR/CQI calculation inputs)
  // ============================================================================
#if CSI_LOG_INPUT_SUMMARY
  uint32_t interference_plus_noise = csi_info->csi_im_meas_computed ? 
                                     csi_info->interference_plus_noise_power : noise_power;
  
  // Output input summary log (rate-limited + anomaly-triggered)
  log_csi_input_summary(ue, proc, csirs_config_pdu, &mapping_parms,
                        csi_rs_received_signal, csi_rs_estimated_channel_freq,
                        mem_offset, CDM_group_size,
                        rsrp, noise_power, interference_plus_noise,
                        rank_indicator, i2[0], precoded_sinr_dB, cqi);
#endif

  switch (csirs_config_pdu->measurement_bitmap) {
    case 1 :
      LOG_I(NR_PHY, "[UE %d] RSRP = %i dBm\n", ue->Mod_id, rsrp_dBm);
      break;
    case 26 :
      // Test log: verify code is running (use LOG_I instead of printf to ensure output)
      LOG_I(NR_PHY, "[TEST-LOG] RI=%d N_ports=%d rank_ind=%d i2=%d\n", 
            rank_indicator + 1, mapping_parms.ports, rank_indicator, i2[0]);
      LOG_I(NR_PHY, "RI = %i i1 = %i.%i.%i, i2 = %i, SINR = %i dB, CQI = %i\n",
            rank_indicator + 1, i1[0], i1[1], i1[2], i2[0], precoded_sinr_dB, cqi);
      break;
    case 27 :
      LOG_I(NR_PHY, "RSRP = %i dBm, RI = %i i1 = %i.%i.%i, i2 = %i, SINR = %i dB, CQI = %i\n",
            rsrp_dBm, rank_indicator + 1, i1[0], i1[1], i1[2], i2[0], precoded_sinr_dB, cqi);
      break;
    default :
      AssertFatal(false, "Not supported measurement configuration\n");
  }

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
    .radiolink_monitoring = RLM_no_monitoring, // TODO do be activated in case of RLM based on CSI-RS
  };
  nr_downlink_indication_t dl_indication;
  fapi_nr_rx_indication_t rx_ind = {0};
  nr_fill_dl_indication(&dl_indication, NULL, &rx_ind, proc, ue, NULL);
  nr_fill_rx_indication(&rx_ind, FAPI_NR_MEAS_IND, ue, NULL, NULL, 1, proc, (void *)&l1_measurements, NULL);
  ue->if_inst->dl_indication(&dl_indication);
}
