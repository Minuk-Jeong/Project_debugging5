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
#include <time.h>

#include "executables/nr-softmodem-common.h"
#include "nr_transport_proto_ue.h"
#include "PHY/phy_extern_nr_ue.h"
#include "PHY/NR_REFSIG/nr_refsig.h"
#include "common/utils/nr/nr_common.h"
#include "PHY/NR_UE_ESTIMATION/filt16a_32.h"
#include "executables/nr-uesoftmodem.h"
#include "PHY/NR_UE_ESTIMATION/dl_csi_onnx.h"
#include <math.h>

// Forward declaration for CSI statistics (defined in nr_ue_procedures.c)
#define CSI_STAT_WINDOW_SIZE 1000
#define CSI_STAT_ONNX_TIMES_MAX 1000

typedef struct {
  uint32_t sent;
  uint32_t skip_period;
  uint32_t skip_not_ready;
  uint32_t deadline_miss;
  uint64_t onnx_times[CSI_STAT_ONNX_TIMES_MAX];
  uint32_t onnx_count;
  uint32_t window_start_abs_slot;
  time_t last_print_time;
} csi_stat_t;

extern csi_stat_t g_csi_stat;

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

// ONNX mode: Store H_onnx[4][4] and A_MF_onnx[4][4] for use in RI/PMI/SINR calculations
static c16_t g_H_onnx[4][4] = {0};
static c16_t g_A_MF_onnx[4][4] = {0};
static int g_onnx_H_ready = 0;
static int g_onnx_A_MF_ready = 0;

// Average channel matrix across all RBs: [4][4] = average of csi_rs_ls_per_rb[51][4][4]
static c16_t g_H_avg_rb[4][4] = {0};
static int g_H_avg_rb_ready = 0;

// ONNX output replicated to all RBs: [51][4][4] = g_H_onnx[4][4] replicated to all RBs
static c16_t g_H_onnx_per_rb[51][4][4] = {{{0}}};
static int g_H_onnx_per_rb_ready = 0;

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
    
    // [CSI-IN] log removed - not needed for RTD matching
    // LOG_I(NR_PHY, "[CSI-IN] f=%d s=%d abs=%u ports=%d row=%d dens=%d rb=%d@%d l0=%d rxP=%u hP=%u hMax=%u zeroH=%u%% rsrp=%u noise=%u intfN=%u RI=%d rankInd=%d PMI=%d sinr=%udB CQI=%d\n",
    //       proc->frame_rx, proc->nr_slot_rx, abs_slot,
    //       mapping_parms->ports, csirs_config_pdu->row, csirs_config_pdu->freq_density,
    //       csirs_config_pdu->nr_of_rbs, csirs_config_pdu->start_rb, csirs_config_pdu->symb_l0,
    //       rxP, hP, hMax, zeroH_percent,
    //       rsrp, noise_power, interference_plus_noise_power,
    //       rank_indicator + 1, rank_indicator, pmi_value, sinr_dB, cqi);
    
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
  
  // Check if ONNX is enabled
  nrUE_params_t *nrUE_params = get_nrUE_params();
  int use_onnx = (nrUE_params && nrUE_params->onnx == 1 && g_onnx_H_ready && g_onnx_A_MF_ready);
  
  if (use_onnx) {
    // Replicate H_onnx[4][4] to all subcarriers in csi_rs_estimated_channel_freq[4][4][1056]
    for (int k = 0; k < (frame_parms->ofdm_symbol_size + FILTER_MARGIN); k++) {
      for (int rx = 0; rx < 4; rx++) {
        for (int tx = 0; tx < 4; tx++) {
          csi_rs_estimated_channel_freq[rx][tx][k] = g_H_onnx[rx][tx];
        }
      }
    }
    
    // Replicate A_MF_onnx[4][4] to all subcarriers [4][4][1056]
    for (int k = 0; k < (frame_parms->ofdm_symbol_size + FILTER_MARGIN); k++) {
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          csi_rs_estimated_A_MF[i][j][k] = g_A_MF_onnx[i][j];
        }
      }
    }
    
    // Compare ONNX H_onnx scale with legacy H scale for validation
    // Calculate statistics for H_onnx (from first valid RB's first subcarrier, same as ONNX input)
    uint32_t onnx_avg_h_power = 0;
    uint32_t onnx_max_h_power = 0;
    uint32_t onnx_zero_h_percent = 0;
    {
      int first_valid_rb = csirs_config_pdu->start_rb;
      while (first_valid_rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs)) {
        if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (first_valid_rb % 2)) {
          first_valid_rb++;
          continue;
        }
        break;
      }
      if (first_valid_rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs)) {
        uint16_t k = (frame_parms->first_carrier_offset + first_valid_rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
        uint16_t k_offset = k + mem_offset;
        
        // Calculate |H_onnx|^2 statistics (Q30 format)
        uint64_t h_power_sum = 0;
        uint32_t h_max = 0;
        uint32_t zero_h_count = 0;
        uint32_t h_count = 0;
        
        for (int rx = 0; rx < 4; rx++) {
          for (int tx = 0; tx < 4; tx++) {
            const c16_t *h = &g_H_onnx[rx][tx];
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
        onnx_avg_h_power = (h_count > 0) ? (uint32_t)(h_power_sum / h_count) : 0;
        onnx_max_h_power = h_max;
        onnx_zero_h_percent = (h_count > 0) ? (zero_h_count * 100 / h_count) : 0;
      }
    }
    // --- Legacy H statistics (before ONNX replacement) ---
uint32_t legacy_avg_h_power_before_onnx = 0;
uint32_t legacy_max_h_power_before_onnx = 0;
uint32_t legacy_zero_h_percent_before_onnx = 0;

calc_h_statistics(ue,
                  csi_rs_estimated_channel_freq,
                  csirs_config_pdu,
                  frame_parms,
                  mem_offset,
                  N_ports,
                  &legacy_avg_h_power_before_onnx,
                  &legacy_max_h_power_before_onnx,
                  &legacy_zero_h_percent_before_onnx);

  } else {
    if (nrUE_params && nrUE_params->onnx == 1) {
      LOG_I(NR_PHY, "[4][4][1056] 배열 생성 이후 onnx=1 (legacy mode - H_onnx/A_MF not ready)\n");
    }
  }

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
      
      if (use_onnx) {
        // Use pre-computed A_MF_onnx from ONNX model (already replicated to all subcarriers)
        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            A_MF[i][j] = csi_rs_estimated_A_MF[i][j][sc_k_offset];
          }
        }
      } else {
        // Legacy: Compute H^H x H matrix for 4x4 system
        memset(A_MF, 0, sizeof(A_MF));
        // Compute H^H x H matrix: (H^H x H)[i][j] = sum_k(conj(H[k][i]) * H[k][j])
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
  
  // Calculate intermediate values for debugging
  int32_t rsrp_dB = dB_fixed(*rsrp);
  int32_t rx_gain_diff = (int)ue->openair0_cfg[0].rx_gain[0] - (int)ue->openair0_cfg[0].rx_gain_offset[0];
  int32_t ofdm_symbol_size_dB = dB_fixed(ue->frame_parms.ofdm_symbol_size);
  
  // RX gain 보정 조정: 실제 하드웨어 gain과 설정값의 차이를 보정
  // 로그 분석 결과 (SINR 46 dB 기준):
  //   - 계산된 RSRP: -111 dBm
  //   - 예상 실제 RSRP: -80 ~ -90 dBm (SINR 기반 역산)
  //   - 차이: 약 20-30 dB
  //   - 따라서 총 보정값: 55 + 25 = 80 dB
  // TODO: 실제 하드웨어 gain을 측정하여 이 값을 정확히 조정해야 함
  int32_t rx_gain_correction = 80; // 임시 보정값 (dB) - 실제 gain 측정 후 조정 필요
  int32_t adjusted_rx_gain_diff = rx_gain_diff > rx_gain_correction ? 
                                   (rx_gain_diff - rx_gain_correction) : 0;
  
  *rsrp_dBm = rsrp_dB + 30 - SQ15_SQUARED_NORM_FACTOR_DB - adjusted_rx_gain_diff - ofdm_symbol_size_dB;

#ifdef NR_CSIRS_DEBUG
  LOG_I(NR_PHY, "RSRP = %i (%i dBm) [rsrp_dB=%d, rx_gain_diff=%d (adjusted=%d), ofdm_sz_dB=%d]\n", 
        *rsrp, *rsrp_dBm, rsrp_dB, rx_gain_diff, adjusted_rx_gain_diff, ofdm_symbol_size_dB);
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
    c16_t csi_rs_ls_per_rb[][csi_mapping->ports][fp->nb_antennas_rx],
    uint32_t nvar_per_rb[][csi_mapping->ports],  // RB별 노이즈 분산 저장
    int16_t *log2_re,
    int16_t *log2_maxh,
    uint32_t *noise_power)
{
  const int dataF_offset = proc->nr_slot_rx * fp->samples_per_slot_wCP;
  *noise_power = 0;
  int maxh = 0;
  int count = 0;

  // Initialize RB-specific LS estimation array
  for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
    for (uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
      for (int ant_rx = 0; ant_rx < fp->nb_antennas_rx; ant_rx++) {
        csi_rs_ls_per_rb[rb_idx][port_tx][ant_rx].r = 0;
        csi_rs_ls_per_rb[rb_idx][port_tx][ant_rx].i = 0;
      }
      nvar_per_rb[rb_idx][port_tx] = 0;  // Initialize nVar per RB
    }
  }

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

      // Calculate RB index for per-RB storage (0-based index within CSI-RS resource)
      int rb_idx = rb - csirs_config_pdu->start_rb;

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
              
              // Store LS estimation result per RB: [rb_idx][port_tx][ant_rx]
              csi_rs_ls_per_rb[rb_idx][port_tx][ant_rx].r += tmp.r;
              csi_rs_ls_per_rb[rb_idx][port_tx][ant_rx].i += tmp.i;
              
              // Calculate noise: noise = y - h_LS * x
              // Q-scale analysis:
              //   - rx, tx: Q15 (y 도메인)
              //   - tmp = c16MulConjShift(tx, rx, shift) = (tx * conj(rx)) >> shift
              //     * tx * conj(rx): Q30
              //     * tmp: Q(30-shift)
              //   - tx_power = |tx|²: Q30
              //   - tmp_tx = tmp * tx: Q(30-shift) * Q15 = Q(45-shift)
              //   - y_est = (tmp_tx << 15) / tx_power: Q(60-shift) / Q30 = Q(30-shift)
              //   - noise = rx - y_est: Q15 - Q(30-shift) → 스케일 불일치!
              //   - 해결: y_est를 Q15로 맞춰야 함
              //     * y_est_q15 = y_est >> (30-shift-15) = y_est >> (15-shift)
              //   - noise_power = |noise|²: Q15 * Q15 = Q30 (목표)
              uint32_t tx_power = (uint32_t)tx_csi_rs_signal[k].r * tx_csi_rs_signal[k].r + 
                                  (uint32_t)tx_csi_rs_signal[k].i * tx_csi_rs_signal[k].i;
              if (tx_power > 0) {
                const int shift = nr_csi_info->csi_rs_generated_signal_bits;
                
                // Compute tmp * tx (complex multiplication)
                // tmp: Q(30-shift), tx: Q15 → tmp_tx: Q(45-shift)
                int32_t tmp_tx_re = (int32_t)tmp.r * tx_csi_rs_signal[k].r - (int32_t)tmp.i * tx_csi_rs_signal[k].i;
                int32_t tmp_tx_im = (int32_t)tmp.r * tx_csi_rs_signal[k].i + (int32_t)tmp.i * tx_csi_rs_signal[k].r;
                
                // Compute y_estimated = (tmp_tx << 15) / tx_power
                // tmp_tx << 15: Q(60-shift), tx_power: Q30 → y_est: Q(30-shift)
                int64_t y_est_re_q30_shift = ((int64_t)tmp_tx_re << 15) / tx_power;
                int64_t y_est_im_q30_shift = ((int64_t)tmp_tx_im << 15) / tx_power;
                
                // Convert y_est from Q(30-shift) to Q15 for proper subtraction with rx (Q15)
                // y_est_q15 = y_est_q30_shift >> (30-shift-15) = y_est_q30_shift >> (15-shift)
                // Safety: if shift >= 15, then (15-shift) <= 0, so we need to shift left instead
                int scale_shift = 15 - shift;
                int32_t y_est_re, y_est_im;
                if (scale_shift > 0) {
                  y_est_re = (int32_t)(y_est_re_q30_shift >> scale_shift);
                  y_est_im = (int32_t)(y_est_im_q30_shift >> scale_shift);
                } else if (scale_shift < 0) {
                  // shift > 15: need to shift left
                  y_est_re = (int32_t)(y_est_re_q30_shift << (-scale_shift));
                  y_est_im = (int32_t)(y_est_im_q30_shift << (-scale_shift));
                } else {
                  // shift == 15: already Q15
                  y_est_re = (int32_t)y_est_re_q30_shift;
                  y_est_im = (int32_t)y_est_im_q30_shift;
                }
                
                // Compute noise: noise = rx - y_estimated (both Q15 now)
                // rx: Q15, y_est: Q15 → noise: Q15
                int32_t noise_re = (int32_t)rx_csi_rs_signal[k].r - y_est_re;
                int32_t noise_im = (int32_t)rx_csi_rs_signal[k].i - y_est_im;
                
                // Accumulate noise power for this RB: |noise|²
                // noise: Q15 → noise_power: Q30 (correct scale for nVar)
                uint32_t noise_power_re = (uint32_t)(noise_re * noise_re);
                uint32_t noise_power_im = (uint32_t)(noise_im * noise_im);
                nvar_per_rb[rb_idx][port_tx] += (noise_power_re + noise_power_im);
              }
            }
          }
        }
      }
    }
    
    // Average nVar per RB over all REs and antennas for each port
    for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
      for (uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
        // Count number of REs for this RB and port
        int re_count = 0;
        int rb = csirs_config_pdu->start_rb + rb_idx;
        if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
          continue;
        }
        for (int cdm_id = 0; cdm_id < csi_mapping->size; cdm_id++) {
          for (int s = 0; s < CDM_group_size; s++) {
            if (s + csi_mapping->j[cdm_id] * CDM_group_size == port_tx) {
              for (int kp = 0; kp <= csi_mapping->kprime; kp++) {
                for (int lp = 0; lp <= csi_mapping->lprime; lp++) {
                  re_count++;
                }
              }
            }
          }
        }
        if (re_count > 0) {
          nvar_per_rb[rb_idx][port_tx] /= (re_count * fp->nb_antennas_rx);
        }
      }
    }
    
    // Calculate overall average nVar across all RBs and ports
    uint64_t total_nvar_sum = 0;
    int total_rb_port_count = 0;
    for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
      int rb = csirs_config_pdu->start_rb + rb_idx;
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      for (uint16_t port_tx = 0; port_tx < csi_mapping->ports; port_tx++) {
        total_nvar_sum += nvar_per_rb[rb_idx][port_tx];
        total_rb_port_count++;
      }
    }
    
    // Store overall average nVar in noise_power (if valid RBs exist)
    if (total_rb_port_count > 0) {
      *noise_power = (uint32_t)(total_nvar_sum / total_rb_port_count);
    }
    
    // Calculate average channel matrix across all RBs: [4][4] = average of csi_rs_ls_per_rb[51][4][4]
    // Only calculate if ONNX is enabled to avoid affecting onnx=0 execution path
    // Memory layout: csi_rs_ls_per_rb[rb_idx][port][ant_rx] -> g_H_avg_rb[ant_rx][port]
    nrUE_params_t *nrUE_params_check = get_nrUE_params();
    if (nrUE_params_check && nrUE_params_check->onnx == 1) {
      memset(g_H_avg_rb, 0, sizeof(g_H_avg_rb));
      int valid_rb_count = 0;
      
      for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
        int rb = csirs_config_pdu->start_rb + rb_idx;
        if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
          continue;
        }
        
        // Accumulate channel values for this RB
        // Note: csi_rs_ls_per_rb[rb_idx][port][ant_rx] -> g_H_avg_rb[ant_rx][port]
        for (uint16_t port = 0; port < csi_mapping->ports; port++) {
          for (int ant_rx = 0; ant_rx < fp->nb_antennas_rx; ant_rx++) {
            g_H_avg_rb[ant_rx][port].r += csi_rs_ls_per_rb[rb_idx][port][ant_rx].r;
            g_H_avg_rb[ant_rx][port].i += csi_rs_ls_per_rb[rb_idx][port][ant_rx].i;
          }
        }
        valid_rb_count++;
      }
      
      // Average: divide by number of valid RBs
      if (valid_rb_count > 0) {
        for (int ant_rx = 0; ant_rx < fp->nb_antennas_rx; ant_rx++) {
          for (uint16_t port = 0; port < csi_mapping->ports; port++) {
            g_H_avg_rb[ant_rx][port].r /= valid_rb_count;
            g_H_avg_rb[ant_rx][port].i /= valid_rb_count;
          }
        }
        g_H_avg_rb_ready = 1;
      } else {
        g_H_avg_rb_ready = 0;
      }
    } else {
      // ONNX disabled: don't calculate g_H_avg_rb to keep execution path identical to Project_debugging7
      g_H_avg_rb_ready = 0;
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

  // Note: noise_power is now calculated from nvar_per_rb average (see above in the nVar calculation section)
  // The old calculation below is replaced by the nVar-based calculation
  // *noise_power /= (fp->nb_antennas_rx * csi_mapping->ports);
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

// Forward declarations for MMSE SINR functions
static inline uint64_t mmse_sinr_metric_L1_L2(
    const c16_t H[4][4], int Nr, int Nt,
    const c16_t W[4][2], int L,
    uint32_t nVar,
    uint64_t out_gamma[2]);
static inline void build_codebook_4port_rank1(int i11, int i2, c16_t W[4][2]);
static inline void build_codebook_4port_rank2(int i11, int i13, int i2, c16_t W[4][2]);
static inline void build_H_rb(
    const c16_t *ls_rb_port_rx, int Nt, int Nr, c16_t H[4][4]);

int nr_csi_rs_pmi_estimation(const PHY_VARS_NR_UE *ue,
                             const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
                             const nr_csi_info_t *nr_csi_info,
                             const uint8_t N_ports,
                             uint8_t mem_offset,
                             const c16_t csi_rs_estimated_channel_freq[][N_ports][ue->frame_parms.ofdm_symbol_size + FILTER_MARGIN],
                             const uint32_t interference_plus_noise_power,
                             const uint8_t rank_indicator,
                             const int16_t log2_re,
                             const c16_t csi_rs_ls_per_rb[][N_ports][ue->frame_parms.nb_antennas_rx],  // RB별 LS 추정 채널
                             const uint32_t nvar_per_rb[][N_ports],  // RB별 노이즈 분산
                             uint64_t sinr_per_rb[][2][32],  // SINRperRB[RB][Layer][PMI] 출력 배열 (uint64_t로 변경하여 포화 제거)
                             uint8_t *i1,
                             uint8_t *i2,
                             uint32_t *precoded_sinr_dB)
{
  const NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
  
  // Use g_H_onnx_per_rb[51][4][4] if available (ONNX output), otherwise use csi_rs_ls_per_rb
  // g_H_onnx_per_rb is the ONNX output from g_H_avg_rb (average of csi_rs_ls_per_rb)
  // Memory layout difference:
  //   - csi_rs_ls_per_rb[rb_idx][port][ant_rx]
  //   - g_H_onnx_per_rb[rb_idx][ant_rx][port] (ONNX output format)
  int use_onnx_per_rb = (g_H_onnx_per_rb_ready && N_ports == 4 && frame_parms->nb_antennas_rx == 4);

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

      // SINR(dB) = 10log10(signal) - 10log10(nVar)
      // nVar = E[|y - h_LS * x|²] (noise variance from LS estimation)
      // (정수 나눗셈으로 sinr_linear가 0이 되는 문제를 피하기 위해 dB 영역에서 계산)
      int32_t sinr_dB = dB_fixed((uint32_t)(avg_signal_power > 0 ? avg_signal_power : 1))
                        - dB_fixed(interference_plus_noise_power);  // interference_plus_noise_power contains nVar

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
            "[SISO SINR] count=%d avg_sig=%ld nVar=%u ratio_q20=%u sinr_dB=%d\n",
            count, avg_signal_power, interference_plus_noise_power, ratio_q20, sinr_dB);
#endif
    }


    return 0;
  }

  // Initialize SINRperRB array
  const int max_pmi_candidates = 32;  // Maximum PMI candidates (4-port rank2)
  for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
    for (int layer = 0; layer < 2; layer++) {
      for (int pmi = 0; pmi < max_pmi_candidates; pmi++) {
        sinr_per_rb[rb_idx][layer][pmi] = 0;
      }
    }
  }

  // Handle 2-port case (existing implementation)
  if(N_ports == 2 && (rank_indicator == 0 || rank_indicator == 1)) {
    // PMI 후보별 E[|h_eff|^2] 누적 (샘플 단위 제곱 누적) - wideband
    int64_t sumsq_re[4] = {0};
    int64_t sumsq_im[4] = {0};
    int64_t tested_precoded_sinr[4] = {0}; // Q20 선형 SINR 저장
    int count = 0;
    const int32_t Q = 20;
    const int num_pmi_candidates = (rank_indicator == 0) ? 4 : 2;  // rank1: 4, rank2: 2

    // RB별 SINR 계산 및 저장
    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }

      int rb_idx = rb - csirs_config_pdu->start_rb;
      uint32_t nvar_rb = nvar_per_rb[rb_idx][0];  // Use nVar for port 0 (2-port uses same nVar)
      if (nvar_rb == 0) continue;

      // Calculate signal power per RB using csi_rs_ls_per_rb
      int64_t rb_sumsq_re[4] = {0};
      int64_t rb_sumsq_im[4] = {0};
      int rb_count = 0;

      for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
        // Use RB-specific channel estimation: g_H_onnx_per_rb or csi_rs_ls_per_rb
        // Memory layout: csi_rs_ls_per_rb[rb_idx][port][ant_rx] vs g_H_onnx_per_rb[rb_idx][ant_rx][port]
        const c16_t p0 = use_onnx_per_rb ? g_H_onnx_per_rb[rb_idx][ant_rx][0] : csi_rs_ls_per_rb[rb_idx][0][ant_rx];
        const c16_t p1 = use_onnx_per_rb ? g_H_onnx_per_rb[rb_idx][ant_rx][1] : csi_rs_ls_per_rb[rb_idx][1][ant_rx];

        // 4개 PMI 후보용 effective channel
        const int32_t t0_re = (int32_t)p0.r + (int32_t)p1.r;
        const int32_t t0_im = (int32_t)p0.i + (int32_t)p1.i;

        const int32_t t1_re = (int32_t)p0.r - (int32_t)p1.i;
        const int32_t t1_im = (int32_t)p0.i + (int32_t)p1.r;

        const int32_t t2_re = (int32_t)p0.r - (int32_t)p1.r;
        const int32_t t2_im = (int32_t)p0.i - (int32_t)p1.i;

        const int32_t t3_re = (int32_t)p0.r + (int32_t)p1.i;
        const int32_t t3_im = (int32_t)p0.i - (int32_t)p1.r;

        rb_sumsq_re[0] += (int64_t)t0_re * t0_re;
        rb_sumsq_im[0] += (int64_t)t0_im * t0_im;

        rb_sumsq_re[1] += (int64_t)t1_re * t1_re;
        rb_sumsq_im[1] += (int64_t)t1_im * t1_im;

        rb_sumsq_re[2] += (int64_t)t2_re * t2_re;
        rb_sumsq_im[2] += (int64_t)t2_im * t2_im;

        rb_sumsq_re[3] += (int64_t)t3_re * t3_re;
        rb_sumsq_im[3] += (int64_t)t3_im * t3_im;

        rb_count++;
      }

      if (rb_count > 0) {
        // Calculate SINR per RB for each PMI candidate
        for (int p = 0; p < num_pmi_candidates; p++) {
          const int64_t avg_pow_q30 = (rb_sumsq_re[p] + rb_sumsq_im[p]) / rb_count;
          if (avg_pow_q30 > 0 && nvar_rb > 0) {
            sinr_per_rb[rb_idx][0][p] = ((uint64_t)avg_pow_q30 << Q) / (uint64_t)nvar_rb;
          }
        }
      }

      // Wideband accumulation (for final PMI selection)
      uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;

      for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
        const c16_t p0 = csi_rs_estimated_channel_freq[ant_rx][0][k_offset];
        const c16_t p1 = csi_rs_estimated_channel_freq[ant_rx][1][k_offset];

        const int32_t t0_re = (int32_t)p0.r + (int32_t)p1.r;
        const int32_t t0_im = (int32_t)p0.i + (int32_t)p1.i;

        const int32_t t1_re = (int32_t)p0.r - (int32_t)p1.i;
        const int32_t t1_im = (int32_t)p0.i + (int32_t)p1.r;

        const int32_t t2_re = (int32_t)p0.r - (int32_t)p1.r;
        const int32_t t2_im = (int32_t)p0.i - (int32_t)p1.i;

        const int32_t t3_re = (int32_t)p0.r + (int32_t)p1.i;
        const int32_t t3_im = (int32_t)p0.i - (int32_t)p1.r;

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
      LOG_I(NR_PHY, "[PMI SINR] count==0 (no valid RE)\n");
#endif
#endif
      return 0;
    }

    // Q20 비율로 SINR 선형값 계산: sinr_q20 = (avg_pow << 20) / nVar
    for (int p = 0; p < 4; p++) {
      const int64_t avg_pow_q30 = (sumsq_re[p] + sumsq_im[p]) / count; // Q30

      if (count <= 0 || avg_pow_q30 <= 0 || interference_plus_noise_power <= 0) {
        tested_precoded_sinr[p] = 0;
        continue;
      }

      tested_precoded_sinr[p] =
          (uint32_t)(((uint64_t)avg_pow_q30 << Q) / (uint64_t)interference_plus_noise_power);
    }


    // Rank별 최적 PMI 선택: PMI[r] = argmax(Σ(모든 RE) Σ(모든 Layer) SINR[RE, Layer, PMI, rank=r])
    // sinr_per_rb[RB][Layer][PMI]를 사용하여 모든 RB와 모든 Layer에 대해 합산
    const int num_pmi_candidates_2port = (rank_indicator == 0) ? 4 : 2;
    uint64_t total_sinr_per_pmi[4] = {0};  // 모든 RE와 Layer에 대한 SINR 합산
    
    for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
      int rb = csirs_config_pdu->start_rb + rb_idx;
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      
      // 각 RB는 12개의 RE를 포함 (NR_NB_SC_PER_RB = 12)
      const int re_per_rb = 12;
      
      for (int layer = 0; layer < (rank_indicator == 0 ? 1 : 2); layer++) {
        for (int pmi = 0; pmi < num_pmi_candidates_2port; pmi++) {
          // RB별 SINR에 RE 개수를 곱하여 모든 RE에 대한 합산
          total_sinr_per_pmi[pmi] += (uint64_t)sinr_per_rb[rb_idx][layer][pmi] * re_per_rb;
        }
      }
    }
    
    // 최대값을 가진 PMI 선택
    if(rank_indicator == 0) {
      i2[0] = 0;
      for(int tested_i2 = 0; tested_i2 < 4; tested_i2++) {
        if(total_sinr_per_pmi[tested_i2] > total_sinr_per_pmi[i2[0]]) {
          i2[0] = tested_i2;
        }
      }
      // 평균 SINR 계산 (모든 RE와 Layer의 합 / RE 개수)
      uint64_t total_re_count = 0;
      for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
        int rb = csirs_config_pdu->start_rb + rb_idx;
        if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
          continue;
        }
        total_re_count += 12;  // 각 RB당 12 RE
      }
      if (total_re_count > 0) {
        uint32_t avg_sinr = (uint32_t)(total_sinr_per_pmi[i2[0]] / total_re_count);
        *precoded_sinr_dB = dB_fixed(avg_sinr);
      } else {
        *precoded_sinr_dB = dB_fixed(tested_precoded_sinr[i2[0]]);
      }
    } else {
      // Rank 2: 두 PMI 조합 비교
      uint64_t sum_pmi0 = total_sinr_per_pmi[0] + total_sinr_per_pmi[2];
      uint64_t sum_pmi1 = total_sinr_per_pmi[1] + total_sinr_per_pmi[3];
      i2[0] = (sum_pmi0 > sum_pmi1) ? 0 : 1;
      
      // 평균 SINR 계산
      uint64_t total_re_count = 0;
      for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
        int rb = csirs_config_pdu->start_rb + rb_idx;
        if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
          continue;
        }
        total_re_count += 12 * 2;  // 각 RB당 12 RE, 2 layers
      }
      if (total_re_count > 0) {
        uint64_t selected_sum = total_sinr_per_pmi[i2[0]] + total_sinr_per_pmi[i2[0] + 2];
        uint32_t avg_sinr = (uint32_t)(selected_sum / total_re_count);
        *precoded_sinr_dB = dB_fixed(avg_sinr);
      } else {
        *precoded_sinr_dB = dB_fixed((tested_precoded_sinr[i2[0]] + tested_precoded_sinr[i2[0]+2])>>1);
      }
    }

  } 
  // Handle 4-port, rank=1 case (RI=1, 1-layer)
  else if(N_ports == 4 && rank_indicator == 0) {
    // ========================================================================
    // 4-port, Rank=1 Type-I Single-Panel codebook implementation
    // Based on TS 38.214 Table 5.2.2.2.1-3
    // ========================================================================
    // 
    // PMI structure: i11 (3bit, 0-7) × i2 (1bit, 0-1) = 16 combinations
    //   - i11: beam pair selection (0-7)
    //   - i13: NOT USED for rank=1 (always 0)
    //   - i2: co-phasing selection (0-1)
    //   - Total: 8 × 2 = 16 candidates
    // ========================================================================
    
    // Test all 16 combinations: i11 (0-7) × i2 (0-1)
    // Index mapping: candidate_idx = i11 * 2 + i2
    int64_t tested_precoded_power[16] = {0}; // Accumulated gamma_q20 for each candidate (wideband, Q20)
    int count_per_candidate[16] = {0}; // Count of samples for each candidate
    const int num_pmi_candidates = 16;  // rank1: 16 candidates
    
    // For each RB, compute effective channel for all 16 candidates
    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      
      int rb_idx = rb - csirs_config_pdu->start_rb;
      // Use average nVar across ports for this RB
      uint64_t nvar_rb_sum = 0;
      for (uint8_t p = 0; p < N_ports; p++) {
        nvar_rb_sum += nvar_per_rb[rb_idx][p];
      }
      uint32_t nvar_rb = (uint32_t)(nvar_rb_sum / N_ports); // average
      
      // nvar_rb=1 발생 경로 확정 로그 1: nvar_rb 계산 직후(평균 직후)
      static int nvar_path_log_count_r1 = 0;
      if (nvar_path_log_count_r1 < 10 && rb_idx == 0) {
        LOG_I(NR_PHY, "[NVAR_PATH_R1_STEP1] count=%d rb_idx=0: nvar_rb=%u(Q30) interference_plus_noise_power=%u(Q30) | nvar_per_rb[0][0]=%u nvar_per_rb[0][1]=%u nvar_per_rb[0][2]=%u nvar_per_rb[0][3]=%u (Q30)\n",
              nvar_path_log_count_r1, nvar_rb, interference_plus_noise_power,
              nvar_per_rb[0][0], nvar_per_rb[0][1], nvar_per_rb[0][2], nvar_per_rb[0][3]);
        nvar_path_log_count_r1++;
      }
      
      // If nvar_rb is 0 or too small, use interference_plus_noise_power (overall average) to maintain proper scale
      // Both nvar_rb and interference_plus_noise_power are in Q30 scale
      // This prevents SINR/CQI saturation due to abnormally small noise variance
      uint32_t nvar_rb_before_fallback = nvar_rb;
      if (nvar_rb == 0 || (interference_plus_noise_power > 0 && nvar_rb < (interference_plus_noise_power >> 10))) {
        nvar_rb = interference_plus_noise_power;
      }
      
      // nvar_rb=1 발생 경로 확정 로그 2: 보정(fallback) 적용 직후
      static int nvar_path_log_count_r1_step2 = 0;
      if (nvar_path_log_count_r1_step2 < 10 && rb_idx == 0) {
        LOG_I(NR_PHY, "[NVAR_PATH_R1_STEP2] count=%d rb_idx=0: nvar_rb_before=%u nvar_rb_after=%u(Q30) interference_plus_noise_power=%u(Q30) | fallback_applied=%d\n",
              nvar_path_log_count_r1_step2, nvar_rb_before_fallback, nvar_rb, interference_plus_noise_power,
              (nvar_rb != nvar_rb_before_fallback) ? 1 : 0);
        nvar_path_log_count_r1_step2++;
      }
      
      uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;
      
      // Build H matrix for this RB: H[Nr][Nt] from g_H_onnx_per_rb or csi_rs_ls_per_rb
      // Memory layout:
      //   - csi_rs_ls_per_rb[rb_idx][port][ant_rx] -> H[ant_rx][port] (via build_H_rb)
      //   - g_H_onnx_per_rb[rb_idx][ant_rx][port] -> H[ant_rx][port] (direct copy)
      c16_t H[4][4] = {{{0}}};
      if (use_onnx_per_rb) {
        // Direct copy from g_H_onnx_per_rb[rb_idx][ant_rx][port] to H[ant_rx][port]
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            H[ant_rx][port] = g_H_onnx_per_rb[rb_idx][ant_rx][port];
          }
        }
      } else {
        // Use build_H_rb to convert csi_rs_ls_per_rb[rb_idx][port][ant_rx] to H[ant_rx][port]
        const c16_t *p = (const c16_t *)&csi_rs_ls_per_rb[rb_idx][0][0];
        build_H_rb(p, N_ports, frame_parms->nb_antennas_rx, H);
      }
      
#ifdef NR_CSIRS_DEBUG
      // 검증 1: H 매핑 검증 - H[ant][port] vs source array 동일성 확인 (rank=1)
      if (rb_idx == 0) {  // 첫 번째 RB만 로그 (로그 폭주 방지)
        int mapping_ok = 1;
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx && mapping_ok; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            const c16_t h_from_array = use_onnx_per_rb ? 
              g_H_onnx_per_rb[rb_idx][ant_rx][port] : 
              csi_rs_ls_per_rb[rb_idx][port][ant_rx];
            const c16_t h_from_matrix = H[ant_rx][port];
            if (h_from_array.r != h_from_matrix.r || h_from_array.i != h_from_matrix.i) {
              mapping_ok = 0;
              LOG_W(NR_PHY, "[H_MAPPING_CHECK_R1] rb=%d ant=%d port=%d: source=(%d,%d) != H[%d][%d]=(%d,%d) (using_onnx=%d)\n",
                    rb_idx, ant_rx, port, h_from_array.r, h_from_array.i,
                    ant_rx, port, h_from_matrix.r, h_from_matrix.i, use_onnx_per_rb);
            }
          }
        }
        if (mapping_ok) {
          LOG_I(NR_PHY, "[H_MAPPING_CHECK_R1] rb=%d: H mapping OK (using_onnx=%d)\n", rb_idx, use_onnx_per_rb);
        }
      }
      
      // 검증 2: 노이즈/채널 스케일 비교 - nvar_rb, interference_plus_noise_power, Σ|H|² (rank=1)
      if (rb_idx == 0) {  // 첫 번째 RB만 로그
        uint64_t h_power_sum = 0;
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            int64_t h_pow = (int64_t)H[ant_rx][port].r * H[ant_rx][port].r + 
                           (int64_t)H[ant_rx][port].i * H[ant_rx][port].i;
            h_power_sum += (uint64_t)h_pow;
          }
        }
        LOG_I(NR_PHY, "[SCALE_CHECK_R1] rb=%d: nvar_rb=%u interference_plus_noise_power=%u Σ|H|²=%llu (Q30)\n",
              rb_idx, nvar_rb, interference_plus_noise_power, (unsigned long long)h_power_sum);
      }
#endif
      
      // Also get interpolated channel for wideband calculation
      // Use ONNX H if available (onnx=1), otherwise use interpolated channel from csi_rs_estimated_channel_freq
      c16_t H_int[4][4] = {{{0}}};
      if (use_onnx_per_rb) {
        // Use ONNX H for wideband accumulation (same as RB-specific calculation)
        // Memory layout: g_H_onnx_per_rb[rb_idx][ant_rx][port] -> H_int[ant_rx][port]
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            H_int[ant_rx][port] = g_H_onnx_per_rb[rb_idx][ant_rx][port];
          }
        }
      } else {
        // Use interpolated channel from csi_rs_estimated_channel_freq (original method)
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            H_int[ant_rx][port] = csi_rs_estimated_channel_freq[ant_rx][port][k_offset];
          }
        }
      }
      
      // Test all 16 combinations: i11 (0-7) × i2 (0-1)
      // For rank=1, i13 is NOT USED (always 0)
      // 각 RB에 대해 각 candidate의 gamma를 직접 계산하여 sinr_per_rb에 저장
      for (int i11_cand = 0; i11_cand < 8; i11_cand++) {
        for (int i2_cand = 0; i2_cand < 2; i2_cand++) {
          int candidate_idx = i11_cand * 2 + i2_cand; // 0-15
          
          // Build precoding matrix W using codebook builder
          c16_t W[4][2];
          build_codebook_4port_rank1(i11_cand, i2_cand, W);
          
          // nvar_rb=1 발생 경로 확정 로그 3: MMSE 호출 직전
          static int nvar_path_log_count_r1_step3 = 0;
          if (nvar_path_log_count_r1_step3 < 10 && rb_idx == 0 && i11_cand == 0 && i2_cand == 0) {
            LOG_I(NR_PHY, "[NVAR_PATH_R1_STEP3] count=%d rb_idx=0 candidate=(%d,%d): nvar_rb=%u(Q30) interference_plus_noise_power=%u(Q30) | before_mmse_call\n",
                  nvar_path_log_count_r1_step3, i11_cand, i2_cand, nvar_rb, interference_plus_noise_power);
            nvar_path_log_count_r1_step3++;
          }
          
          // Compute MMSE SINR metric for this RB and candidate
          // MMSE formula: γ_q20 = (||H*W||^2 / nVar) * 2^20 (for rank=1)
          // Returns gamma_q20 directly (Q20 fixed-point)
          uint64_t gamma_layer_q20[2] = {0, 0};
          (void)mmse_sinr_metric_L1_L2(
              H, frame_parms->nb_antennas_rx, N_ports, W, 1, nvar_rb, gamma_layer_q20);
          
          // Store MMSE SINR for this RB directly (already in Q20)
          // RB 하나에 대해 candidate 하나의 gamma를 그대로 저장
          // 평균을 내고 싶으면 나중에 RB 개수로 나누면 됨 (현재는 RB별 저장이 목적)
          // uint64_t로 변경하여 포화 제거
          if (gamma_layer_q20[0] > 0) {
            sinr_per_rb[rb_idx][0][candidate_idx] = gamma_layer_q20[0];
          }
          
          // Wideband accumulation (for final PMI selection)
          // Use ONNX H if available (onnx=1), otherwise use interpolated channel
          uint64_t gamma_int_layer_q20[2] = {0, 0};
          uint64_t metric_int_q20 = mmse_sinr_metric_L1_L2(
              H_int, frame_parms->nb_antennas_rx, N_ports, W, 1, interference_plus_noise_power, gamma_int_layer_q20);
          
          // Accumulate wideband metric (already in Q20)
          tested_precoded_power[candidate_idx] += metric_int_q20;
          count_per_candidate[candidate_idx]++;
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
    
    // Compute average MMSE SINR for each candidate (i11 × i2)
    // tested_precoded_power now contains MMSE metric (sum of gamma_q20 across RBs)
    // Q is already defined above (line 1596)
    uint32_t tested_precoded_sinr[16] = {0}; // Q20 fixed-point SINR storage
    for (int candidate_idx = 0; candidate_idx < 16; candidate_idx++) {
      if (count_per_candidate[candidate_idx] == 0) {
        tested_precoded_sinr[candidate_idx] = 0;
        continue;
      }
      
      // Average MMSE metric (gamma_q20) over all samples
      // tested_precoded_power contains sum of MMSE metrics (gamma_q20 values, already Q20)
      uint64_t avg_gamma_q20 = tested_precoded_power[candidate_idx] / count_per_candidate[candidate_idx];
      
      // gamma is already in Q20 format, just store it (clamp to uint32_t max)
      tested_precoded_sinr[candidate_idx] = (avg_gamma_q20 > UINT32_MAX) 
          ? UINT32_MAX 
          : (uint32_t)avg_gamma_q20;
    }
    
    // Rank별 최적 PMI 선택: PMI[r] = argmax(Σ(모든 RE) Σ(모든 Layer) SINR[RE, Layer, PMI, rank=r])
    // sinr_per_rb[RB][Layer][PMI]를 사용하여 모든 RB와 모든 Layer에 대해 합산
    uint64_t total_sinr_per_pmi[16] = {0};  // 모든 RE와 Layer에 대한 SINR 합산
    
    for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
      int rb = csirs_config_pdu->start_rb + rb_idx;
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      
      // 각 RB는 12개의 RE를 포함 (NR_NB_SC_PER_RB = 12)
      const int re_per_rb = 12;
      
      // Rank 1: Layer 0만 사용
      for (int pmi = 0; pmi < 16; pmi++) {
        // RB별 SINR에 RE 개수를 곱하여 모든 RE에 대한 합산
        total_sinr_per_pmi[pmi] += (uint64_t)sinr_per_rb[rb_idx][0][pmi] * re_per_rb;
      }
    }
    
    // 최대값을 가진 PMI 선택
    int best_candidate_idx = 0;
    for (int candidate_idx = 1; candidate_idx < 16; candidate_idx++) {
      if (total_sinr_per_pmi[candidate_idx] > total_sinr_per_pmi[best_candidate_idx]) {
        best_candidate_idx = candidate_idx;
      }
    }
    
    // Extract i11 and i2 from best candidate (i13 is NOT USED for rank=1)
    i1[0] = best_candidate_idx / 2; // i11 (0-7)
    i1[1] = 0; // i12 (not used, set to 0)
    i1[2] = 0; // i13 (NOT USED for rank=1, always 0)
    i2[0] = best_candidate_idx % 2; // i2 (0-1)
    
    // 평균 SINR 계산 (모든 RE와 Layer의 합 / RE 개수)
    // total_sinr_per_pmi는 Q20 format이므로, 평균도 Q20
    uint64_t total_re_count = 0;
    for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
      int rb = csirs_config_pdu->start_rb + rb_idx;
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      total_re_count += 12;  // 각 RB당 12 RE
    }
    if (total_re_count > 0) {
      // avg_sinr_q20: Q20 format
      uint64_t avg_sinr_q20 = total_sinr_per_pmi[best_candidate_idx] / total_re_count;
      // Convert Q20 to linear for dB_fixed: gamma_linear = gamma_q20 >> 20
      uint32_t avg_sinr_linear = (uint32_t)(avg_sinr_q20 >> 20);
      *precoded_sinr_dB = dB_fixed(avg_sinr_linear > 0 ? avg_sinr_linear : 1);
    } else {
      // tested_precoded_sinr is Q20, convert to linear
      uint32_t sinr_linear = tested_precoded_sinr[best_candidate_idx] >> 20;
      *precoded_sinr_dB = dB_fixed(sinr_linear > 0 ? sinr_linear : 1);
    }
    
  }
  // Handle 4-port, rank=2 case (new implementation)
  else if(N_ports == 4 && rank_indicator == 1) {
    // ========================================================================
    // 4-port, Rank=2 Type-I Single-Panel codebook implementation
    // Based on TS 38.214 Table 5.2.2.2.1-3
    // ========================================================================
    // 
    // PMI structure: i11 (3bit, 0-7) × i13 (1bit, 0-1) × i2 (1bit, 0-1) = 32 combinations
    //   - i11: beam pair selection (0-7)
    //   - i13: beam pair selection (0-1)
    //   - i2: co-phasing selection (0-1)
    //   - Total: 8 × 2 × 2 = 32 candidates
    // ========================================================================
    
    // Test all 32 combinations: i11 (0-7) × i13 (0-1) × i2 (0-1)
    // Index mapping: candidate_idx = i11 * 4 + i13 * 2 + i2
    int64_t tested_precoded_power[32] = {0}; // Accumulated gamma_q20 for each candidate (wideband, Q20)
    int count_per_candidate[32] = {0}; // Count of samples for each candidate
    const int num_pmi_candidates = 32;  // rank2: 32 candidates
    
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
    
    // For each RB, compute effective channel for all 32 candidates
    for (int rb = csirs_config_pdu->start_rb; rb < (csirs_config_pdu->start_rb + csirs_config_pdu->nr_of_rbs); rb++) {
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      
      int rb_idx = rb - csirs_config_pdu->start_rb;
      // Use average nVar across ports for this RB
      uint64_t nvar_rb_sum = 0;
      for (uint8_t p = 0; p < N_ports; p++) {
        nvar_rb_sum += nvar_per_rb[rb_idx][p];
      }
      uint32_t nvar_rb = (uint32_t)(nvar_rb_sum / N_ports); // average
      
      // nvar_rb=1 발생 경로 확정 로그 1: nvar_rb 계산 직후(평균 직후)
      static int nvar_path_log_count_r2 = 0;
      if (nvar_path_log_count_r2 < 10 && rb_idx == 0) {
        LOG_I(NR_PHY, "[NVAR_PATH_R2_STEP1] count=%d rb_idx=0: nvar_rb=%u(Q30) interference_plus_noise_power=%u(Q30) | nvar_per_rb[0][0]=%u nvar_per_rb[0][1]=%u nvar_per_rb[0][2]=%u nvar_per_rb[0][3]=%u (Q30)\n",
              nvar_path_log_count_r2, nvar_rb, interference_plus_noise_power,
              nvar_per_rb[0][0], nvar_per_rb[0][1], nvar_per_rb[0][2], nvar_per_rb[0][3]);
        nvar_path_log_count_r2++;
      }
      
      // If nvar_rb is 0 or too small, use interference_plus_noise_power (overall average) to maintain proper scale
      // Both nvar_rb and interference_plus_noise_power are in Q30 scale
      // This prevents SINR/CQI saturation due to abnormally small noise variance
      uint32_t nvar_rb_before_fallback = nvar_rb;
      if (nvar_rb == 0 || (interference_plus_noise_power > 0 && nvar_rb < (interference_plus_noise_power >> 10))) {
        nvar_rb = interference_plus_noise_power;
      }
      
      // nvar_rb=1 발생 경로 확정 로그 2: 보정(fallback) 적용 직후
      static int nvar_path_log_count_r2_step2 = 0;
      if (nvar_path_log_count_r2_step2 < 10 && rb_idx == 0) {
        LOG_I(NR_PHY, "[NVAR_PATH_R2_STEP2] count=%d rb_idx=0: nvar_rb_before=%u nvar_rb_after=%u(Q30) interference_plus_noise_power=%u(Q30) | fallback_applied=%d\n",
              nvar_path_log_count_r2_step2, nvar_rb_before_fallback, nvar_rb, interference_plus_noise_power,
              (nvar_rb != nvar_rb_before_fallback) ? 1 : 0);
        nvar_path_log_count_r2_step2++;
      }
      
      uint16_t k = (frame_parms->first_carrier_offset + rb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
      uint16_t k_offset = k + mem_offset;
      
      // Build H matrix for this RB: H[Nr][Nt] from g_H_onnx_per_rb or csi_rs_ls_per_rb
      // Memory layout:
      //   - csi_rs_ls_per_rb[rb_idx][port][ant_rx] -> H[ant_rx][port] (via build_H_rb)
      //   - g_H_onnx_per_rb[rb_idx][ant_rx][port] -> H[ant_rx][port] (direct copy)
      c16_t H[4][4] = {{{0}}};
      if (use_onnx_per_rb) {
        // Direct copy from g_H_onnx_per_rb[rb_idx][ant_rx][port] to H[ant_rx][port]
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            H[ant_rx][port] = g_H_onnx_per_rb[rb_idx][ant_rx][port];
          }
        }
      } else {
        // Use build_H_rb to convert csi_rs_ls_per_rb[rb_idx][port][ant_rx] to H[ant_rx][port]
        const c16_t *p = (const c16_t *)&csi_rs_ls_per_rb[rb_idx][0][0];
        build_H_rb(p, N_ports, frame_parms->nb_antennas_rx, H);
      }
      
#ifdef NR_CSIRS_DEBUG
      // 검증 1: H 매핑 검증 - H[ant][port] vs source array 동일성 확인 (rank=2)
      if (rb_idx == 0) {  // 첫 번째 RB만 로그 (로그 폭주 방지)
        int mapping_ok = 1;
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx && mapping_ok; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            const c16_t h_from_array = use_onnx_per_rb ? 
              g_H_onnx_per_rb[rb_idx][ant_rx][port] : 
              csi_rs_ls_per_rb[rb_idx][port][ant_rx];
            const c16_t h_from_matrix = H[ant_rx][port];
            if (h_from_array.r != h_from_matrix.r || h_from_array.i != h_from_matrix.i) {
              mapping_ok = 0;
              LOG_W(NR_PHY, "[H_MAPPING_CHECK_R2] rb=%d ant=%d port=%d: source=(%d,%d) != H[%d][%d]=(%d,%d) (using_onnx=%d)\n",
                    rb_idx, ant_rx, port, h_from_array.r, h_from_array.i,
                    ant_rx, port, h_from_matrix.r, h_from_matrix.i, use_onnx_per_rb);
            }
          }
        }
        if (mapping_ok) {
          LOG_I(NR_PHY, "[H_MAPPING_CHECK_R2] rb=%d: H mapping OK (using_onnx=%d)\n", rb_idx, use_onnx_per_rb);
        }
      }
      
      // 검증 2: 노이즈/채널 스케일 비교 - nvar_rb, interference_plus_noise_power, Σ|H|² (rank=2)
      if (rb_idx == 0) {  // 첫 번째 RB만 로그
        uint64_t h_power_sum = 0;
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            int64_t h_pow = (int64_t)H[ant_rx][port].r * H[ant_rx][port].r + 
                           (int64_t)H[ant_rx][port].i * H[ant_rx][port].i;
            h_power_sum += (uint64_t)h_pow;
          }
        }
        LOG_I(NR_PHY, "[SCALE_CHECK_R2] rb=%d: nvar_rb=%u interference_plus_noise_power=%u Σ|H|²=%llu (Q30)\n",
              rb_idx, nvar_rb, interference_plus_noise_power, (unsigned long long)h_power_sum);
      }
#endif
      
      // Also get interpolated channel for wideband calculation
      // Use ONNX H if available (onnx=1), otherwise use interpolated channel from csi_rs_estimated_channel_freq
      c16_t H_int[4][4] = {{{0}}};
      if (use_onnx_per_rb) {
        // Use ONNX H for wideband accumulation (same as RB-specific calculation)
        // Memory layout: g_H_onnx_per_rb[rb_idx][ant_rx][port] -> H_int[ant_rx][port]
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            H_int[ant_rx][port] = g_H_onnx_per_rb[rb_idx][ant_rx][port];
          }
        }
      } else {
        // Use interpolated channel from csi_rs_estimated_channel_freq (original method)
        for (int ant_rx = 0; ant_rx < frame_parms->nb_antennas_rx; ant_rx++) {
          for (uint8_t port = 0; port < N_ports; port++) {
            H_int[ant_rx][port] = csi_rs_estimated_channel_freq[ant_rx][port][k_offset];
          }
        }
      }
      
      // Test all 32 combinations: i11 (0-7) × i13 (0-1) × i2 (0-1)
      // 각 RB에 대해 각 candidate의 gamma를 직접 계산하여 sinr_per_rb에 저장
      for (int i11_cand = 0; i11_cand < 8; i11_cand++) {
        for (int i13_cand = 0; i13_cand < 2; i13_cand++) {
          for (int i2_cand = 0; i2_cand < 2; i2_cand++) {
            int candidate_idx = i11_cand * 4 + i13_cand * 2 + i2_cand; // 0-31
            
            // Build precoding matrix W using codebook builder
            c16_t W[4][2];
            build_codebook_4port_rank2(i11_cand, i13_cand, i2_cand, W);
            
            // nvar_rb=1 발생 경로 확정 로그 3: MMSE 호출 직전
            static int nvar_path_log_count_r2_step3 = 0;
            if (nvar_path_log_count_r2_step3 < 10 && rb_idx == 0 && i11_cand == 0 && i13_cand == 0 && i2_cand == 0) {
              LOG_I(NR_PHY, "[NVAR_PATH_R2_STEP3] count=%d rb_idx=0 candidate=(%d,%d,%d): nvar_rb=%u(Q30) interference_plus_noise_power=%u(Q30) | before_mmse_call\n",
                    nvar_path_log_count_r2_step3, i11_cand, i13_cand, i2_cand, nvar_rb, interference_plus_noise_power);
              nvar_path_log_count_r2_step3++;
            }
            
            // Compute MMSE SINR metric for this RB and candidate
            // MMSE formula: γ_l_q20 = ((1 / (nVar * [(W^H H^H H W + nVar I)^-1]_l,l)) - 1) * 2^20
            // Returns gamma_q20 directly (Q20 fixed-point)
            uint64_t gamma_layer_q20[2] = {0, 0};
            (void)mmse_sinr_metric_L1_L2(
                H, frame_parms->nb_antennas_rx, N_ports, W, 2, nvar_rb, gamma_layer_q20);
            
            // Store MMSE SINR for this RB directly (already in Q20)
            // RB 하나에 대해 candidate 하나의 gamma를 그대로 저장 (layer별로)
            // 평균을 내고 싶으면 나중에 RB 개수로 나누면 됨 (현재는 RB별 저장이 목적)
            // uint64_t로 변경하여 포화 제거
            if (gamma_layer_q20[0] > 0) {
              sinr_per_rb[rb_idx][0][candidate_idx] = gamma_layer_q20[0];
            }
            
            if (gamma_layer_q20[1] > 0) {
              sinr_per_rb[rb_idx][1][candidate_idx] = gamma_layer_q20[1];
            }
            
            // Wideband accumulation (for final PMI selection)
            // Use ONNX H if available (onnx=1), otherwise use interpolated channel
            uint64_t gamma_int_layer_q20[2] = {0, 0};
            uint64_t metric_int_q20 = mmse_sinr_metric_L1_L2(
                H_int, frame_parms->nb_antennas_rx, N_ports, W, 2, interference_plus_noise_power, gamma_int_layer_q20);
            
            // Accumulate wideband metric (sum of gamma_q20 across layers, already in Q20)
            tested_precoded_power[candidate_idx] += metric_int_q20;
            count_per_candidate[candidate_idx]++;
          }
        }
      }
    }
    
    // Compute average MMSE SINR for each candidate (i11 × i13 × i2)
    // tested_precoded_power now contains MMSE metric (sum of gamma_q20 across RBs and layers)
    // Q is already defined above (line 1867)
    uint32_t tested_precoded_sinr[32] = {0}; // Q20 fixed-point SINR 저장
    for (int candidate_idx = 0; candidate_idx < 32; candidate_idx++) {
      if (count_per_candidate[candidate_idx] == 0) {
        tested_precoded_sinr[candidate_idx] = 0;
        continue;
      }
      
      // Average MMSE metric (gamma_sum_q20 = gamma0_q20 + gamma1_q20) over all samples
      // tested_precoded_power contains sum of MMSE metrics (gamma_sum_q20 values, already Q20)
      uint64_t avg_gamma_sum_q20 = tested_precoded_power[candidate_idx] / count_per_candidate[candidate_idx];
      
      // gamma is already in Q20 format, just store it (clamp to uint32_t max)
      tested_precoded_sinr[candidate_idx] = (avg_gamma_sum_q20 > UINT32_MAX) 
          ? UINT32_MAX 
          : (uint32_t)avg_gamma_sum_q20;
    }
    
    // Rank별 최적 PMI 선택: PMI[r] = argmax(Σ(모든 RE) Σ(모든 Layer) SINR[RE, Layer, PMI, rank=r])
    // sinr_per_rb[RB][Layer][PMI]를 사용하여 모든 RB와 모든 Layer에 대해 합산
    uint64_t total_sinr_per_pmi[32] = {0};  // 모든 RE와 Layer에 대한 SINR 합산
    
    for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
      int rb = csirs_config_pdu->start_rb + rb_idx;
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      
      // 각 RB는 12개의 RE를 포함 (NR_NB_SC_PER_RB = 12)
      const int re_per_rb = 12;
      
      // Rank 2: Layer 0과 Layer 1 모두 사용
      for (int layer = 0; layer < 2; layer++) {
        for (int pmi = 0; pmi < 32; pmi++) {
          // RB별 SINR에 RE 개수를 곱하여 모든 RE에 대한 합산
          total_sinr_per_pmi[pmi] += (uint64_t)sinr_per_rb[rb_idx][layer][pmi] * re_per_rb;
        }
      }
    }
    
    // 최대값을 가진 PMI 선택
    int best_candidate_idx = 0;
    for (int candidate_idx = 1; candidate_idx < 32; candidate_idx++) {
      if (total_sinr_per_pmi[candidate_idx] > total_sinr_per_pmi[best_candidate_idx]) {
        best_candidate_idx = candidate_idx;
      }
    }
    
    // Extract i11, i13, and i2 from best candidate
    i1[0] = best_candidate_idx / 4; // i11 (0-7)
    i1[1] = 0; // i12 (not used, set to 0)
    i1[2] = (best_candidate_idx / 2) % 2; // i13 (0-1)
    i2[0] = best_candidate_idx % 2; // i2 (0-1)
    
    // 평균 SINR 계산 (모든 RE와 Layer의 합 / RE 개수)
    // total_sinr_per_pmi는 Q20 format이므로, 평균도 Q20
    uint64_t total_re_count = 0;
    for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
      int rb = csirs_config_pdu->start_rb + rb_idx;
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
        continue;
      }
      total_re_count += 12 * 2;  // 각 RB당 12 RE, 2 layers
    }
    if (total_re_count > 0) {
      // avg_sinr_q20: Q20 format
      uint64_t avg_sinr_q20 = total_sinr_per_pmi[best_candidate_idx] / total_re_count;
      // Convert Q20 to linear for dB_fixed: gamma_linear = gamma_q20 >> 20
      uint32_t avg_sinr_linear = (uint32_t)(avg_sinr_q20 >> 20);
      *precoded_sinr_dB = dB_fixed(avg_sinr_linear > 0 ? avg_sinr_linear : 1);
    } else {
      // tested_precoded_sinr is Q20, convert to linear
      uint32_t sinr_linear = tested_precoded_sinr[best_candidate_idx] >> 20;
      *precoded_sinr_dB = dB_fixed(sinr_linear > 0 ? sinr_linear : 1);
    }
    
#if CSI_LOG_CSIRS_VERBOSE
    // Find top-3 candidates for detailed logging (only when verbose logging enabled)
    struct {
      int idx;
      uint32_t sinr;
    } top3[3] = {{-1, 0}, {-1, 0}, {-1, 0}};
    
    for (int i = 0; i < 32; i++) {
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
      int best_candidate_idx = i1[0] * 2 + i2[0];
      int64_t avg_pow_best = count_per_candidate[best_candidate_idx] > 0 ? 
        tested_precoded_power[best_candidate_idx] / count_per_candidate[best_candidate_idx] : 0;
      // Convert Q20 to linear for dB conversion: sinr_linear = sinr_q20 >> 20
      uint32_t sinr_linear_best = tested_precoded_sinr[best_candidate_idx] > 0 ? 
        (tested_precoded_sinr[best_candidate_idx] >> 20) : 1;
      uint32_t sinr_dB_best = dB_fixed(sinr_linear_best);
      LOG_I(NR_PHY, "[PMI DETAIL Best] i11=%d (3bit) i2=%d (1bit) candidate_idx=%d sinr_q20=%u sinr_linear=%u sinr_dB=%u avg_pow_q30=%ld count=%d intf_noise=%u\n",
            i1[0], i2[0], best_candidate_idx,
            tested_precoded_sinr[best_candidate_idx], sinr_linear_best, sinr_dB_best, avg_pow_best, 
            count_per_candidate[best_candidate_idx], interference_plus_noise_power);
      
      // Log top-3 candidates
      // Helper function to convert Q20 to dB
      uint32_t sinr_linear_0 = top3[0].sinr > 0 ? (top3[0].sinr >> 20) : 1;
      uint32_t sinr_linear_1 = top3[1].idx >= 0 && top3[1].sinr > 0 ? (top3[1].sinr >> 20) : 1;
      uint32_t sinr_linear_2 = top3[2].idx >= 0 && top3[2].sinr > 0 ? (top3[2].sinr >> 20) : 1;
      LOG_I(NR_PHY, "[PMI DETAIL Top3] #1: i11=%d i2=%d idx=%d sinr_q20=%u sinr_linear=%u sinr_dB=%u | #2: i11=%d i2=%d idx=%d sinr_q20=%u sinr_linear=%u sinr_dB=%u | #3: i11=%d i2=%d idx=%d sinr_q20=%u sinr_linear=%u sinr_dB=%u\n",
            top3[0].idx >= 0 ? (top3[0].idx / 2) : -1, top3[0].idx >= 0 ? (top3[0].idx % 2) : -1, top3[0].idx, top3[0].sinr, sinr_linear_0, dB_fixed(sinr_linear_0),
            top3[1].idx >= 0 ? (top3[1].idx / 2) : -1, top3[1].idx >= 0 ? (top3[1].idx % 2) : -1, top3[1].idx >= 0 ? top3[1].idx : -1, top3[1].idx >= 0 ? top3[1].sinr : 0, sinr_linear_1, top3[1].idx >= 0 ? dB_fixed(sinr_linear_1) : 0,
            top3[2].idx >= 0 ? (top3[2].idx / 2) : -1, top3[2].idx >= 0 ? (top3[2].idx % 2) : -1, top3[2].idx >= 0 ? top3[2].idx : -1, top3[2].idx >= 0 ? top3[2].sinr : 0, sinr_linear_2, top3[2].idx >= 0 ? dB_fixed(sinr_linear_2) : 0);
      
      if (tested_precoded_sinr[best_candidate_idx] == 0) {
        LOG_W(NR_PHY, "[PMI DETAIL WARNING] Selected i11=%d i2=%d (idx=%d) has zero SINR! avg_pow=%ld, count=%d\n",
              i1[0], i2[0], best_candidate_idx, avg_pow_best, count_per_candidate[best_candidate_idx]);
      }
    }
#endif

#ifdef NR_CSIRS_DEBUG
#if 0
    // This log removed - information included in SUMMARY/DETAIL logs
    // Convert Q20 to linear for dB conversion
    uint32_t sinr_linear_rank2 = tested_precoded_sinr[i2[0]] > 0 ? (tested_precoded_sinr[i2[0]] >> 20) : 1;
    uint32_t sinr_dB_rank2 = dB_fixed(sinr_linear_rank2);
    LOG_I(NR_PHY, "[PMI 4x4 Rank2] selected i2=%d, sinr_q20=%u, sinr_linear=%u, sinr_dB=%u, count=%d, intf_noise=%u\n",
          i2[0], tested_precoded_sinr[i2[0]], sinr_linear_rank2, sinr_dB_rank2, 
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

// -----------------------------
// Q15 complex helpers
// Note: c16_t is already defined in common/platform_types.h
// -----------------------------

static inline int32_t cmul_re_q15(int16_t ar, int16_t ai, int16_t br, int16_t bi) {
  // (a * b).re = ar*br - ai*bi
  return (int32_t)ar * br - (int32_t)ai * bi;
}
static inline int32_t cmul_im_q15(int16_t ar, int16_t ai, int16_t br, int16_t bi) {
  // (a * b).im = ar*bi + ai*br
  return (int32_t)ar * bi + (int32_t)ai * br;
}
static inline int64_t cabs2_i32(int32_t xr, int32_t xi) {
  return (int64_t)xr * xr + (int64_t)xi * xi;
}

// Convert linear SINR (Q0 or any positive integer scale) to dB (existing OAI helper assumed)
// extern uint32_t dB_fixed(uint32_t x);

// -----------------------------
// Build a (Nr x Nt) channel matrix H from per-RB LS results
// H[r][t] = csi_rs_ls_per_rb[rb_idx][t][r]
// -----------------------------
static inline void build_H_rb(
    const c16_t *ls_rb_port_rx, // pointer to [Nt][Nr] in memory order [port][rx]
    int Nt, int Nr,
    c16_t H[4][4])              // max 4x4 for this use-case
{
  for (int r = 0; r < Nr; r++) {
    for (int t = 0; t < Nt; t++) {
      const c16_t v = ls_rb_port_rx[t * Nr + r];
      H[r][t] = v;
    }
  }
}

// -----------------------------
// Compute R = H * W
// H: Nr x Nt (c16 Q15-ish)
// W: Nt x L  (c16 Q15), normalized
// R: Nr x L  (int32, Q15-ish after >>15)
// -----------------------------
static inline void matmul_H_W(
    const c16_t H[4][4], int Nr, int Nt,
    const c16_t W[4][2], int L,
    int32_t R_re[4][2], int32_t R_im[4][2])
{
  for (int r = 0; r < Nr; r++) {
    for (int l = 0; l < L; l++) {
      int64_t acc_re = 0;
      int64_t acc_im = 0;
      for (int t = 0; t < Nt; t++) {
        const int16_t hr = H[r][t].r;
        const int16_t hi = H[r][t].i;
        const int16_t wr = W[t][l].r;
        const int16_t wi = W[t][l].i;
        acc_re += (int64_t)cmul_re_q15(hr, hi, wr, wi);
        acc_im += (int64_t)cmul_im_q15(hr, hi, wr, wi);
      }
      // scale back to ~Q15
      R_re[r][l] = (int32_t)(acc_re >> 15);
      R_im[r][l] = (int32_t)(acc_im >> 15);
    }
  }
}

// -----------------------------
// MMSE SINR metric for L=1 or L=2
// MATLAB equivalent:
//   R = H*W
//   G = R^H R + nVar I
//   gamma_l = 1/(nVar * inv(G)_{l,l}) - 1
// Returns: gamma_sum_q20 = sum_l gamma_l_q20  (Q20 fixed-point)
// Also optionally returns per-layer gamma_q20 in out_gamma[2] (Q20 fixed-point)
// 
// Q-scale notes:
//   - H: Q15, W: Q15
//   - R = H*W: Q15 (after >>15 scaling)
//   - R^H R: Q30 (Q15 * Q15)
//   - nVar: Q30 (noise_re^2 where noise_re is Q15)
//   - a = r0^H r0 + nVar: Q30 + Q30 = Q30
//   - det = a*d - |b|^2: Q60 - Q60 = Q60
//   - gamma = det/(nVar*d) - 1: Q60/Q60 - 1 = Q0 (linear)
//   - gamma_q20 = gamma * 2^20: Q20
// -----------------------------
static inline uint64_t mmse_sinr_metric_L1_L2(
    const c16_t H[4][4], int Nr, int Nt,
    const c16_t W[4][2], int L,
    uint32_t nVar,  // Q30 scale (noise variance)
    uint64_t out_gamma[2])  // Output: gamma_q20[2] (Q20 fixed-point)
{
  int32_t R_re[4][2] = {{0}};
  int32_t R_im[4][2] = {{0}};
  matmul_H_W(H, Nr, Nt, W, L, R_re, R_im);

  if (L == 1) {
    // s = ||r||^2 (Q30: R is Q15, |R|^2 is Q30)
    uint64_t s = 0;
    for (int r = 0; r < Nr; r++) {
      s += (uint64_t)cabs2_i32(R_re[r][0], R_im[r][0]);
    }
    // gamma_q20 = (s / nVar) * 2^20 = (s << 20) / nVar
    // s: Q30, nVar: Q30, gamma_q20: Q20
    uint64_t gamma_q20 = 0;
    if (nVar > 0) {
      // Scale s to Q50, then divide by Q30 nVar to get Q20
      uint64_t s_q50 = s << 20;  // Q30 -> Q50
      if (s_q50 >= (uint64_t)nVar) {  // Avoid underflow
        gamma_q20 = s_q50 / (uint64_t)nVar;
      }
    }
    
    // MMSE 내부 자기일관성 로그: gamma 계산 직후, return 직전
    // L==1: s(Q30), nVar(Q30), gamma_q20(Q20), sinr_dB 출력
    // 기대식: gamma_q20 ≈ (s << 20) / nVar
    static int mmse_consistency_log_count_r1 = 0;
    if (mmse_consistency_log_count_r1 < 5) {
      // Convert gamma_q20 (Q20) to linear scale for dB conversion
      // 계산은 64비트로 유지, dB 변환만 32비트 범위로 안전하게 clamp
      uint64_t gamma_linear64 = (gamma_q20 > 0) ? (gamma_q20 >> 20) : 0;
      uint32_t gamma_linear32 = (gamma_linear64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)gamma_linear64;
      int8_t sinr_dB = (gamma_linear32 > 0) ? dB_fixed(gamma_linear32) : 0;
      
      // Calculate expected gamma_q20 for verification
      uint64_t expected_gamma_q20 = (nVar > 0 && s > 0) ? ((s << 20) / nVar) : 0;
      
      LOG_I(NR_PHY, "[MMSE_CONSISTENCY_L1] count=%d: s=%llu(Q30) nVar=%u(Q30) gamma_q20=%llu(Q20) expected_gamma_q20=%llu(Q20) sinr_dB=%d | check: gamma_q20≈(s<<20)/nVar\n",
            mmse_consistency_log_count_r1, (unsigned long long)s, nVar, 
            (unsigned long long)gamma_q20, (unsigned long long)expected_gamma_q20, sinr_dB);
      mmse_consistency_log_count_r1++;
    }
    
#ifdef NR_CSIRS_DEBUG
    // 검증 3: MMSE 내부 중간값 (rank=1)
    static int mmse_debug_count_r1 = 0;
    if (mmse_debug_count_r1 < 5) {
      LOG_I(NR_PHY, "[MMSE_INTERNAL_R1] count=%d: ||R||²=s=%llu(Q30) nVar=%u(Q30) gamma_q20=(s<<20)/nVar=%llu(Q20)\n",
            mmse_debug_count_r1, (unsigned long long)s, nVar, (unsigned long long)gamma_q20);
      mmse_debug_count_r1++;
    }
#endif
    
    out_gamma[0] = gamma_q20;
    out_gamma[1] = 0;
    return gamma_q20;
  }

  // L == 2
  // a = r0^H r0 + nVar (both Q30)
  // d = r1^H r1 + nVar (both Q30)
  // b = r0^H r1 (complex, Q30)
  // Note: R is Q15, so |R|^2 is Q30. nVar is also Q30, so addition is correct.
  uint64_t a = (uint64_t)nVar;  // Q30
  uint64_t d = (uint64_t)nVar;  // Q30
  int64_t  b_re = 0;  // Q30
  int64_t  b_im = 0;  // Q30

  for (int r = 0; r < Nr; r++) {
    const int32_t r0_re = R_re[r][0], r0_im = R_im[r][0];  // Q15
    const int32_t r1_re = R_re[r][1], r1_im = R_im[r][1];  // Q15

    // |r0|^2: Q15 * Q15 = Q30
    a += (uint64_t)cabs2_i32(r0_re, r0_im);  // Q30
    // |r1|^2: Q15 * Q15 = Q30
    d += (uint64_t)cabs2_i32(r1_re, r1_im);  // Q30

    // b += conj(r0) * r1
    // conj(r0) * r1 = (r0_re - j r0_im)*(r1_re + j r1_im)
    // re = r0_re*r1_re + r0_im*r1_im (Q15 * Q15 = Q30)
    // im = -r0_im*r1_re + r0_re*r1_im (Q15 * Q15 = Q30)
    b_re += (int64_t)r0_re * r1_re + (int64_t)r0_im * r1_im;  // Q30
    b_im += -(int64_t)r0_im * r1_re + (int64_t)r0_re * r1_im;  // Q30
  }

  // det = a*d - |b|^2  (Q30 * Q30 = Q60, |b|^2 is Q60)
  __int128 ad = (__int128)a * (__int128)d;  // Q60
  __int128 bb = (__int128)b_re * (__int128)b_re + (__int128)b_im * (__int128)b_im;  // Q60
  __int128 det = ad - bb;  // Q60
  
#ifdef NR_CSIRS_DEBUG
  // 검증 3: MMSE 내부 중간값 - A 대각(a, d), det, denom0, denom1, 최종 gamma 계산 직전 값
  static int mmse_debug_count = 0;
  if (mmse_debug_count < 5) {  // 처음 5번만 로그
    LOG_I(NR_PHY, "[MMSE_INTERNAL] count=%d: A_diag[0]=a=%llu(Q30) A_diag[1]=d=%llu(Q30) |b|²=%llu(Q60) det=%lld(Q60) nVar=%u(Q30)\n",
          mmse_debug_count, (unsigned long long)a, (unsigned long long)d, 
          (unsigned long long)bb, (long long)det, nVar);
    mmse_debug_count++;
  }
#endif
  
  if (det <= 0 || nVar == 0) {
#ifdef NR_CSIRS_DEBUG
    static int mmse_debug_warn_count = 0;
    if (mmse_debug_warn_count < 5) {
      LOG_W(NR_PHY, "[MMSE_INTERNAL] det=%lld <= 0 or nVar=%u == 0, returning gamma_q20=0\n", 
            (long long)det, nVar);
      mmse_debug_warn_count++;
    }
#endif
    out_gamma[0] = 0;
    out_gamma[1] = 0;
    return 0;
  }

  // gamma0 = det/(nVar*d) - 1
  // gamma1 = det/(nVar*a) - 1
  // det: Q60, nVar: Q30, d: Q30, a: Q30
  // denom0 = nVar*d: Q30 * Q30 = Q60
  // denom1 = nVar*a: Q30 * Q30 = Q60
  __int128 denom0 = (__int128)nVar * (__int128)d;  // Q60
  __int128 denom1 = (__int128)nVar * (__int128)a;  // Q60

#ifdef NR_CSIRS_DEBUG
  // 검증 3 계속: denom0, denom1, t0_q20, t1_q20 값
  static int mmse_debug_count2 = 0;
  if (mmse_debug_count2 < 5) {
    __int128 t0_q20_before = (det << 20) / denom0;  // Q80 / Q60 = Q20
    __int128 t1_q20_before = (det << 20) / denom1;  // Q80 / Q60 = Q20
    LOG_I(NR_PHY, "[MMSE_INTERNAL] count=%d: denom0=%lld(Q60) denom1=%lld(Q60) t0_q20=%lld(Q20) t1_q20=%lld(Q20)\n",
          mmse_debug_count2, (long long)denom0, (long long)denom1, 
          (long long)t0_q20_before, (long long)t1_q20_before);
    mmse_debug_count2++;
  }
#endif

  // gamma0 = det/(nVar*d) - 1, gamma1 = det/(nVar*a) - 1
  // det: Q60, denom0/denom1: Q60
  // Compute directly in Q20: t0_q20 = (det << 20) / denom0, t1_q20 = (det << 20) / denom1
  // det: Q60, denom0/denom1: Q60
  // Compute directly in Q20: t0_q20 = (det << 20) / denom0, t1_q20 = (det << 20) / denom1
  // This gives: t0_q20 = (det/denom0) * 2^20, t1_q20 = (det/denom1) * 2^20
  __int128 t0_q20 = (det << 20) / denom0;  // Q80 / Q60 = Q20
  __int128 t1_q20 = (det << 20) / denom1;  // Q80 / Q60 = Q20

  // gamma0_q20 = t0_q20 - (1 << 20), gamma1_q20 = t1_q20 - (1 << 20)
  // Both are in Q20 format
  uint64_t gamma0_q20 = 0;
  uint64_t gamma1_q20 = 0;
  
  __int128 one_q20 = (__int128)1 << 20;  // 1 in Q20 format
  
  if (t0_q20 > one_q20) {
    gamma0_q20 = (uint64_t)(t0_q20 - one_q20);
    // Q20를 uint64_t로 끝까지 유지 (clamp 제거)
  }
  
  if (t1_q20 > one_q20) {
    gamma1_q20 = (uint64_t)(t1_q20 - one_q20);
    // Q20를 uint64_t로 끝까지 유지 (clamp 제거)
  }

  // MMSE 내부 자기일관성 로그: gamma 계산 직후, return 직전
  // L==2: s(Q30) = ||r0||^2 + ||r1||^2 = (a - nVar) + (d - nVar) = a + d - 2*nVar
  // nVar(Q30), gamma0_q20/gamma1_q20(Q20), sinr_dB 출력
  // 기대식: gamma0_q20 ≈ (det << 20) / denom0 - (1 << 20), gamma1_q20 ≈ (det << 20) / denom1 - (1 << 20)
  static int mmse_consistency_log_count_r2 = 0;
  if (mmse_consistency_log_count_r2 < 5) {
    // Calculate s = ||r0||^2 + ||r1||^2 = a + d - 2*nVar (Q30)
    uint64_t s = (a > 2 * (uint64_t)nVar && d > 2 * (uint64_t)nVar) ? (a + d - 2 * (uint64_t)nVar) : 0;
    
    // Convert gamma to linear scale for dB conversion
    // 계산은 64비트로 유지, dB 변환만 32비트 범위로 안전하게 clamp
    uint64_t gamma0_linear64 = (gamma0_q20 > 0) ? (gamma0_q20 >> 20) : 0;
    uint64_t gamma1_linear64 = (gamma1_q20 > 0) ? (gamma1_q20 >> 20) : 0;
    uint64_t gamma_sum_linear64 = gamma0_linear64 + gamma1_linear64;
    uint32_t gamma_sum_linear32 = (gamma_sum_linear64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)gamma_sum_linear64;
    int8_t sinr_dB = (gamma_sum_linear32 > 0) ? dB_fixed(gamma_sum_linear32) : 0;
    
    // Calculate expected gamma for verification
    __int128 expected_t0_q20 = (denom0 > 0) ? ((det << 20) / denom0) : 0;
    __int128 expected_t1_q20 = (denom1 > 0) ? ((det << 20) / denom1) : 0;
    uint64_t expected_gamma0_q20 = (expected_t0_q20 > one_q20) ? (uint64_t)(expected_t0_q20 - one_q20) : 0;
    uint64_t expected_gamma1_q20 = (expected_t1_q20 > one_q20) ? (uint64_t)(expected_t1_q20 - one_q20) : 0;
    
    LOG_I(NR_PHY, "[MMSE_CONSISTENCY_L2] count=%d: s=%llu(Q30) nVar=%u(Q30) | gamma0_q20=%llu(Q20) expected_gamma0_q20=%llu(Q20) gamma1_q20=%llu(Q20) expected_gamma1_q20=%llu(Q20) sinr_dB=%d | check: gamma0≈det/denom0-1, gamma1≈det/denom1-1\n",
          mmse_consistency_log_count_r2, (unsigned long long)s, nVar,
          (unsigned long long)gamma0_q20, (unsigned long long)expected_gamma0_q20,
          (unsigned long long)gamma1_q20, (unsigned long long)expected_gamma1_q20, sinr_dB);
    mmse_consistency_log_count_r2++;
  }

#ifdef NR_CSIRS_DEBUG
  // 검증 3 계속: 최종 gamma_q20 값
  static int mmse_debug_count3 = 0;
  if (mmse_debug_count3 < 5) {
    LOG_I(NR_PHY, "[MMSE_INTERNAL] count=%d: gamma0_q20=%llu(Q20) gamma1_q20=%llu(Q20) gamma_sum_q20=%llu(Q20)\n",
          mmse_debug_count3, (unsigned long long)gamma0_q20, (unsigned long long)gamma1_q20,
          (unsigned long long)(gamma0_q20 + gamma1_q20));
    mmse_debug_count3++;
  }
#endif

  out_gamma[0] = gamma0_q20;
  out_gamma[1] = gamma1_q20;
  return gamma0_q20 + gamma1_q20;
}

// -----------------------------
// Simple codebook builders (2-port fixed, 4-port "Type1-like" with oversampled beam index)
// NOTE:
// - Nt=2: exact 3GPP-style common 4 vectors and 2 rank2 matrices (orthogonal pairs)
// - Nt=4: this is a practical "Type1SinglePanel-like" construction using:
//   * i11: beam index 0..7 (oversampling factor 4 for N1=2 -> 8 beams)
//   * i2 : co-phasing between pol groups (0..1 -> {1, j})
//   * i13: layer-2 co-phasing selector (0..1)
// Port grouping assumption: [0,1]=pol0, [2,3]=pol1, each pol has 2 elements.
// W normalized to approx 1/sqrt(Nt) (Q15 scaling).
// -----------------------------
static inline c16_t q15_from_phase_quarter(int b) {
  // exp(j*pi/2 * b) for b in {0,1,2,3} -> {1, j, -1, -j}
  // Q15: 1 -> 32767, -1 -> -32767
  switch (b & 3) {
    case 0: return (c16_t){ 32767,     0};
    case 1: return (c16_t){     0, 32767};
    case 2: return (c16_t){-32767,     0};
    default:return (c16_t){     0,-32767};
  }
}

static inline void build_codebook_2port_rank1(int cand, c16_t W[4][2]) {
  // cand 0..3: [1, 1], [1, j], [1,-1], [1,-j] all / sqrt(2)
  // Q15 scale for 1/sqrt(2) ~ 23170
  const int16_t s = 23170;
  memset(W, 0, sizeof(c16_t)*4*2);
  W[0][0] = (c16_t){s,0};
  if (cand == 0) W[1][0] = (c16_t){ s, 0};
  if (cand == 1) W[1][0] = (c16_t){ 0, s};
  if (cand == 2) W[1][0] = (c16_t){-s, 0};
  if (cand == 3) W[1][0] = (c16_t){ 0,-s};
}

static inline void build_codebook_2port_rank2(int cand, c16_t W[4][2]) {
  // Two orthogonal 2x2 matrices (common choices):
  // cand0: (1/sqrt(2))*[ [1,  1],
  //                     [1, -1] ]
  // cand1: (1/sqrt(2))*[ [1,   1],
  //                     [j,  -j] ]
  const int16_t s = 23170;
  memset(W, 0, sizeof(c16_t)*4*2);
  if (cand == 0) {
    W[0][0]=(c16_t){ s,0}; W[0][1]=(c16_t){ s,0};
    W[1][0]=(c16_t){ s,0}; W[1][1]=(c16_t){-s,0};
  } else {
    W[0][0]=(c16_t){ s,0}; W[0][1]=(c16_t){ s,0};
    W[1][0]=(c16_t){ 0,s}; W[1][1]=(c16_t){ 0,-s};
  }
}

static inline void build_codebook_4port_rank1(int i11, int i2, c16_t W[4][2]) {
  // beam vector v(theta) = [1, exp(j*theta)] / sqrt(2), theta = 2*pi*i11/8
  // pol co-phase = exp(j*pi/2 * i2) where i2 in {0,1} -> {1, j}
  // overall normalization ~ 1/sqrt(4) = 1/2
  // => each nonzero entry magnitude about 1/2 * 1/sqrt(2) ~ 0.3535
  // Q15: 0.3535 ~ 11585
  const int16_t a = 11585;

  // phase for second element in each pol group:
  // theta = 2*pi*i11/8 = (pi/4)*i11  => quarter-phase steps
  // exp(j*theta) can be represented via b in {0..7} using quarter lookup twice:
  // pi/4 step -> not exactly pi/2. Here we approximate with 8-point unit circle Q15.
  // For correctness you may replace this with a cosine/sine LUT of 8 points.
  static const c16_t e8[8] = {
    { 32767,     0}, { 23170, 23170}, {     0, 32767}, {-23170, 23170},
    {-32767,     0}, {-23170,-23170}, {     0,-32767}, { 23170,-23170}
  };
  c16_t ejt = e8[i11 & 7];
  c16_t pol = q15_from_phase_quarter(i2); // 0->1, 1->j

  memset(W, 0, sizeof(c16_t)*4*2);
  // Layer0 only
  // pol0: [1, ejt]
  W[0][0] = (c16_t){a, 0};
  W[1][0] = (c16_t){ (int16_t)(( (int32_t)a * ejt.r) >> 15),
                     (int16_t)(( (int32_t)a * ejt.i) >> 15) };

  // pol1: pol * [1, ejt]
  // W[2]=pol*1, W[3]=pol*ejt
  W[2][0] = (c16_t){ (int16_t)(((int32_t)a * pol.r) >> 15),
                     (int16_t)(((int32_t)a * pol.i) >> 15) };
  // pol*ejt
  int32_t re = cmul_re_q15(pol.r, pol.i, ejt.r, ejt.i);
  int32_t im = cmul_im_q15(pol.r, pol.i, ejt.r, ejt.i);
  W[3][0] = (c16_t){ (int16_t)(( (int32_t)a * re) >> 15),
                     (int16_t)(( (int32_t)a * im) >> 15) };
}

static inline void build_codebook_4port_rank2(int i11, int i13, int i2, c16_t W[4][2]) {
  // Build two orthogonal layers using v and v_perp:
  // v      = [1,  ejt] / sqrt(2)
  // v_perp = [1, -ejt] / sqrt(2)  (orthogonal for 2 elements)
  // pol0 uses v/v_perp
  // pol1 uses co-phase:
  //   layer0: pol0_phase = exp(j*pi/2*i2)
  //   layer1: pol1_phase = exp(j*pi/2*(i2 + 2*i13))  (toggle by i13 -> multiply by -1)
  // overall normalization ~ 1/2
  const int16_t a = 11585;

  static const c16_t e8[8] = {
    { 32767,     0}, { 23170, 23170}, {     0, 32767}, {-23170, 23170},
    {-32767,     0}, {-23170,-23170}, {     0,-32767}, { 23170,-23170}
  };
  c16_t ejt = e8[i11 & 7];

  c16_t pol0 = q15_from_phase_quarter(i2);                 // 0->1,1->j
  c16_t pol1 = q15_from_phase_quarter(i2 + 2*(i13 & 1));   // i13 toggles sign (adds pi)

  memset(W, 0, sizeof(c16_t)*4*2);

  // layer0 (v)
  W[0][0] = (c16_t){a,0};
  W[1][0] = (c16_t){ (int16_t)(((int32_t)a * ejt.r) >> 15),
                     (int16_t)(((int32_t)a * ejt.i) >> 15) };
  // pol0 * v
  W[2][0] = (c16_t){ (int16_t)(((int32_t)a * pol0.r) >> 15),
                     (int16_t)(((int32_t)a * pol0.i) >> 15) };
  int32_t re0 = cmul_re_q15(pol0.r, pol0.i, ejt.r, ejt.i);
  int32_t im0 = cmul_im_q15(pol0.r, pol0.i, ejt.r, ejt.i);
  W[3][0] = (c16_t){ (int16_t)(((int32_t)a * re0) >> 15),
                     (int16_t)(((int32_t)a * im0) >> 15) };

  // layer1 (v_perp = [1, -ejt])
  W[0][1] = (c16_t){a,0};
  W[1][1] = (c16_t){ (int16_t)(-(((int32_t)a * ejt.r) >> 15)),
                     (int16_t)(-(((int32_t)a * ejt.i) >> 15)) };
  // pol1 * v_perp
  W[2][1] = (c16_t){ (int16_t)(((int32_t)a * pol1.r) >> 15),
                     (int16_t)(((int32_t)a * pol1.i) >> 15) };
  int32_t re1 = cmul_re_q15(pol1.r, pol1.i, (int16_t)(-ejt.r), (int16_t)(-ejt.i));
  int32_t im1 = cmul_im_q15(pol1.r, pol1.i, (int16_t)(-ejt.r), (int16_t)(-ejt.i));
  W[3][1] = (c16_t){ (int16_t)(((int32_t)a * re1) >> 15),
                     (int16_t)(((int32_t)a * im1) >> 15) };
}

// -----------------------------
// Main function: MATLAB-like wideband PMI selection using MMSE SINR metric
// -----------------------------
int nr_csi_rs_pmi_selection_per_rb(
    const PHY_VARS_NR_UE *ue,
    const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu,
    const uint8_t N_ports,
    const c16_t csi_rs_ls_per_rb[][N_ports][ue->frame_parms.nb_antennas_rx],
    const uint32_t interference_plus_noise_power,
    const uint8_t rank_indicator, // 0->rank1, 1->rank2
    uint8_t *i1,
    uint8_t *i2,
    uint32_t *precoded_sinr_dB)
{
  const NR_DL_FRAME_PARMS *fp = &ue->frame_parms;
  const int Nr = fp->nb_antennas_rx;
  const int Nt = N_ports;
  const int L  = (rank_indicator == 0) ? 1 : 2;

  // Basic guards
  // Note: interference_plus_noise_power parameter now contains nVar (noise variance)
  // calculated from LS estimation: nVar = E[|y - h_LS * x|²]
  if (interference_plus_noise_power == 0 || Nr <= 0 || Nt <= 0) {
    LOG_W(NR_PHY, "[PMI-per-RB] invalid nVar or dims (nVar=%u Nr=%d Nt=%d)\n",
          interference_plus_noise_power, Nr, Nt);
    return 0;
  }
  if ((Nt != 2 && Nt != 4) || (L != 1 && L != 2)) {
    LOG_W(NR_PHY, "[PMI-per-RB] unsupported Nt=%d rank=%d\n", Nt, L);
    return 0;
  }
  if (Nr > 4) {
    // This implementation uses fixed max 4 for simplicity
    LOG_W(NR_PHY, "[PMI-per-RB] Nr=%d > 4 not supported by this implementation\n", Nr);
    return 0;
  }

  // Candidate enumeration sizes
  int numCand = 0;
  // For outputs
  uint8_t best_i11 = 0, best_i13 = 0, best_i2 = 0;
  uint64_t best_metric = 0;
  uint64_t best_gamma_sum = 0;
  int used_rb = 0;

  // For 2-port:
  //   rank1: 4 candidates (cand=0..3)
  //   rank2: 2 candidates (cand=0..1)
  // For 4-port:
  //   rank1: i11=0..7, i2=0..1 => 16
  //   rank2: i11=0..7, i13=0..1, i2=0..1 => 32
  if (Nt == 2) numCand = (L == 1) ? 4 : 2;
  if (Nt == 4) numCand = (L == 1) ? (8 * 2) : (8 * 2 * 2);

  // Iterate candidates, compute wideband metric = sum over RBs of sum over layers gamma
  for (int cand = 0; cand < numCand; cand++) {
    uint64_t metric_sum = 0;
    uint64_t gamma_sum_all = 0;
    int rb_count = 0;

    // decode candidate indices and build W
    c16_t W[4][2];
    uint8_t cand_i11 = 0, cand_i13 = 0, cand_i2 = 0;

    if (Nt == 2) {
      if (L == 1) {
        cand_i2 = (uint8_t)cand;
        build_codebook_2port_rank1(cand, W);
      } else {
        cand_i2 = (uint8_t)cand; // 0..1
        build_codebook_2port_rank2(cand, W);
      }
      cand_i11 = 0; cand_i13 = 0;
    } else { // Nt==4
      if (L == 1) {
        cand_i11 = (uint8_t)(cand / 2); // 0..7
        cand_i2  = (uint8_t)(cand % 2); // 0..1
        cand_i13 = 0;
        build_codebook_4port_rank1(cand_i11, cand_i2, W);
      } else {
        cand_i11 = (uint8_t)(cand / 4);        // 0..7
        cand_i13 = (uint8_t)((cand / 2) % 2);  // 0..1
        cand_i2  = (uint8_t)(cand % 2);        // 0..1
        build_codebook_4port_rank2(cand_i11, cand_i13, cand_i2, W);
      }
    }

    // RB loop (wideband sum)
    for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
      int rb = csirs_config_pdu->start_rb + rb_idx;

      // Keep your existing density rule if needed (this is from your original code)
      if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2))
        continue;

      // Build H (Nr x Nt) from g_H_onnx_per_rb or csi_rs_ls_per_rb
      // Memory layout:
      //   - csi_rs_ls_per_rb[rb_idx][port][ant_rx] -> H[ant_rx][port] (via build_H_rb)
      //   - g_H_onnx_per_rb[rb_idx][ant_rx][port] -> H[ant_rx][port] (direct copy)
      c16_t H[4][4] = {{{0}}};
      int use_onnx_per_rb_pmi = (g_H_onnx_per_rb_ready && Nt == 4 && Nr == 4);
      if (use_onnx_per_rb_pmi) {
        // Direct copy from g_H_onnx_per_rb[rb_idx][ant_rx][port] to H[ant_rx][port]
        for (int ant_rx = 0; ant_rx < Nr; ant_rx++) {
          for (uint8_t port = 0; port < Nt; port++) {
            H[ant_rx][port] = g_H_onnx_per_rb[rb_idx][ant_rx][port];
          }
        }
      } else {
        // Use build_H_rb to convert csi_rs_ls_per_rb[rb_idx][port][ant_rx] to H[ant_rx][port]
        const c16_t *p = (const c16_t *)&csi_rs_ls_per_rb[rb_idx][0][0];
        build_H_rb(p, Nt, Nr, H);
      }

      uint64_t gamma_layer[2] = {0,0};
      uint64_t metric = mmse_sinr_metric_L1_L2(
          H, Nr, Nt, W, L, interference_plus_noise_power, gamma_layer);

      metric_sum += metric;
      gamma_sum_all += (gamma_layer[0] + gamma_layer[1]);
      rb_count++;
    }

    if (rb_count == 0) continue;

    // Select max metric
    if (metric_sum > best_metric) {
      best_metric = metric_sum;
      best_gamma_sum = gamma_sum_all;
      best_i11 = cand_i11;
      best_i13 = cand_i13;
      best_i2  = cand_i2;
      used_rb  = rb_count;
    }
  }

  if (best_metric == 0 || used_rb == 0) {
    LOG_W(NR_PHY, "[PMI-per-RB] no valid RBs or metric=0\n");
    return 0;
  }

  // Output mapping
  // For Type1SinglePanel-like:
  //   i1 = [i11, i12, i13], here i12 unused -> 0
  //   i2 = best_i2
  i1[0] = best_i11;
  i1[1] = 0;
  i1[2] = best_i13;
  i2[0] = best_i2;

  // Report a representative SINR:
  // Use average gamma over RBs and layers (linear), then dB_fixed
  // avg_gamma = best_gamma_sum / (used_rb * L)
  uint64_t denom = (uint64_t)used_rb * (uint64_t)L;
  uint64_t avg_gamma = (denom > 0) ? (best_gamma_sum / denom) : 0;

  // dB_fixed expects uint32
  uint32_t avg_gamma_u32 = (avg_gamma > 0xFFFFFFFFULL) ? 0xFFFFFFFFu : (uint32_t)avg_gamma;
  *precoded_sinr_dB = dB_fixed(avg_gamma_u32);

  LOG_I(NR_PHY,
        "[PMI-per-RB][MMSE] Nt=%d rank=%d used_rb=%d metric=%llu -> i11=%u i13=%u i2=%u avg_sinr_dB=%u\n",
        Nt, L, used_rb, (unsigned long long)best_metric,
        i1[0], i1[2], i2[0], *precoded_sinr_dB);

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
  
  // Record CSI processing start time for deadline check (only when ONNX is enabled)
  struct timespec ts_start, ts_end;
  uint64_t start_us = 0;
  bool deadline_check_enabled = false;
  {
    nrUE_params_t *nrUE_params = get_nrUE_params();
    if (nrUE_params && nrUE_params->onnx == 1) {
      deadline_check_enabled = true;
      if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts_start) == 0) {
        start_us = (uint64_t)ts_start.tv_sec * 1000000ULL + (uint64_t)ts_start.tv_nsec / 1000ULL;
      }
    }
  }

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
  // Per-RB LS estimation results: [rb_index][port_tx][ant_rx]
  // Maximum 51 RBs as specified, with 4 ports and 4 RX antennas
  c16_t csi_rs_ls_per_rb[51][mapping_parms.ports][frame_parms->nb_antennas_rx];
  // Per-RB noise variance: [rb_index][port_tx]
  uint32_t nvar_per_rb[51][mapping_parms.ports];

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
  
  // Store legacy H statistics before ONNX replacement (for scale comparison)
  uint32_t legacy_avg_h_power_before_onnx = 0;
  uint32_t legacy_max_h_power_before_onnx = 0;
  uint32_t legacy_zero_h_percent_before_onnx = 0;
  
  // if we need to measure only RSRP no need to do channel estimation
  if (csirs_config_pdu->measurement_bitmap > 1) {
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
                                 csi_rs_ls_per_rb,
                                 nvar_per_rb,
                                 &log2_re,
                                 &log2_maxh,
                                 &noise_power);
    
    // Calculate legacy H statistics before ONNX replacement
    calc_h_statistics(ue, csi_rs_estimated_channel_freq, csirs_config_pdu, frame_parms,
                      mem_offset, mapping_parms.ports,
                      &legacy_avg_h_power_before_onnx, &legacy_max_h_power_before_onnx,
                      &legacy_zero_h_percent_before_onnx);
  }

  // ONNX processing: Use g_H_avg_rb[4][4] (average channel matrix across all RBs) as input
  nrUE_params_t *nrUE_params = get_nrUE_params();
  if (nrUE_params && nrUE_params->onnx == 1) {
    // Check if g_H_avg_rb is ready (calculated from csi_rs_ls_per_rb[51][4][4])
    if (g_H_avg_rb_ready) {
      // Use g_H_avg_rb[4][4] directly (already averaged across all RBs)
      // Memory layout: g_H_avg_rb[ant_rx][port] -> feature[32] (row-major, Re/Im interleaved)
      
      // Calculate input power P_in (unnormalized) from g_H_avg_rb
      // This is the power before RMS normalization
      float P_in = 0.0f;
      for (int rx = 0; rx < 4; rx++) {
        for (int tx = 0; tx < 4; tx++) {
          // Convert Q15 to float and calculate power
          float re = (float)g_H_avg_rb[rx][tx].r / 32768.0f;
          float im = (float)g_H_avg_rb[rx][tx].i / 32768.0f;
          P_in += re * re + im * im;
        }
      }
      P_in /= 16.0f;  // Average power per element (4x4 = 16 elements)
      
      // Convert to row-major, (Re,Im) interleaved 32-dim float feature
      float feature[32];
      int idx = 0;
      for (int rx = 0; rx < 4; rx++) {
        for (int tx = 0; tx < 4; tx++) {
          // Convert Q15 fixed-point to float (divide by 32768.0)
          feature[idx++] = (float)g_H_avg_rb[rx][tx].r / 32768.0f;
          feature[idx++] = (float)g_H_avg_rb[rx][tx].i / 32768.0f;
        }
      }
      
      // RMS normalization
      float rms = 0.0f;
      for (int i = 0; i < 32; i++) {
        rms += feature[i] * feature[i];
      }
      rms = sqrtf(rms / 32.0f);
      
      if (rms > 1e-6f) {
        for (int i = 0; i < 32; i++) {
          feature[i] /= rms;
        }
      }
      
      // Replicate 10 times to form X[10][32] = [1,10,32] shape
      float onnx_input[1 * 10 * 32];
      for (int t = 0; t < 10; t++) {
        memcpy(&onnx_input[t * 32], feature, sizeof(float) * 32);
      }
      
      // Log CSI input ready timing
      {
        struct timespec ts;
        if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts) == 0) {
          int frame = proc->frame_rx;
          int slot = proc->nr_slot_rx;
          uint32_t abs_slot = (uint32_t)(frame * frame_parms->slots_per_frame + slot);
          uint64_t t_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
          LOG_I(NR_PHY, "[CSI-TIMING] abs=%u f=%d s=%d stage=CSI_INPUT_READY t_us=%llu (using g_H_avg_rb)\n",
                abs_slot, frame, slot, (unsigned long long)t_us);
        }
      }
      
      // --- ONNX init (once) + run ---
      static int onnx_ready = 0;

      if (!onnx_ready) {
        const char *model_path = getenv("OAI_DL_CSI_ONNX_MODEL");
        // If environment variable is not set, use default model path
        const char *default_model_path = "models/channel_predictor.onnx";
        if (!model_path || !*model_path) {
          model_path = default_model_path;
          LOG_I(NR_PHY, "[ONNX] OAI_DL_CSI_ONNX_MODEL not set, using default: %s\n", model_path);
        }
        int ir = init_dl_csi_onnx(model_path, 10, 32);
        LOG_I(NR_PHY, "[ONNX] init ret=%d (model=%s)\n", ir, model_path);
        if (ir == 0) onnx_ready = 1;
      }

      if (onnx_ready) {
        // Log ONNX inference start timing
        struct timespec ts_start, ts_end;
        int frame = proc->frame_rx;
        int slot = proc->nr_slot_rx;
        uint32_t abs_slot = (uint32_t)(frame * frame_parms->slots_per_frame + slot);
        uint64_t start_us = 0;
        
        if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts_start) == 0) {
          start_us = (uint64_t)ts_start.tv_sec * 1000000ULL + (uint64_t)ts_start.tv_nsec / 1000ULL;
          LOG_I(NR_PHY, "[ONNX-TIMING] abs=%u f=%d s=%d start_us=%llu\n",
                abs_slot, frame, slot, (unsigned long long)start_us);
        }
        
        float onnx_output[32] = {0};
        int ret = run_dl_csi_onnx(onnx_input, onnx_output);
        
        // Log ONNX inference end timing and duration
        if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts_end) == 0) {
          uint64_t end_us = (uint64_t)ts_end.tv_sec * 1000000ULL + (uint64_t)ts_end.tv_nsec / 1000ULL;
          uint64_t infer_us = (start_us > 0) ? (end_us - start_us) : 0;
          LOG_I(NR_PHY, "[ONNX-TIMING] abs=%u f=%d s=%d end_us=%llu infer_us=%llu\n",
                abs_slot, frame, slot, (unsigned long long)end_us, (unsigned long long)infer_us);
          
          // Update statistics: ONNX inference time
          if (g_csi_stat.onnx_count < CSI_STAT_ONNX_TIMES_MAX) {
            g_csi_stat.onnx_times[g_csi_stat.onnx_count++] = infer_us;
          }
        }
        
        if (ret == 0) {
          // Convert ONNX output[32] back to H_onnx[4][4] matrix
          // ONNX output is (Re,Im) interleaved in row-major order
          // Note: ONNX output has its own model-defined scale (not legacy csi_rs_ls_per_rb scale)
          // DO NOT apply scale_shift to match legacy scale - this causes excessive clipping
          // ONNX output maintains its model-defined scale in Q15 format
          c16_t H_onnx[4][4];
          int idx = 0;
          
          // Calculate output power P_out (denormalize 전) from ONNX output
          // This is the power of the normalized ONNX output
          float P_out = 0.0f;
          for (int i = 0; i < 32; i++) {
            P_out += onnx_output[i] * onnx_output[i];
          }
          P_out /= 32.0f;  // Average power per element (32 elements: 16 complex = 32 real)
          
          // Calculate scale factor alpha = sqrt(P_in / P_out) to match input power
          // This ensures the denormalized ONNX output has the same average power as the input
          float alpha = 1.0f;
          if (P_out > 1e-10f && P_in > 1e-10f) {
            alpha = sqrtf(P_in / P_out);
          }
          
          // Calculate RMS of ONNX output for logging
          float onnx_output_rms = sqrtf(P_out);
          
          // Find max absolute value in ONNX output
          float onnx_max_abs = 0.0f;
          for (int i = 0; i < 32; i++) {
            float abs_val = fabsf(onnx_output[i]);
            if (abs_val > onnx_max_abs) onnx_max_abs = abs_val;
          }
          
          // Log ONNX output scale statistics (first few times only)
          static int onnx_scale_log_count = 0;
          if (onnx_scale_log_count < 10) {
            LOG_I(NR_PHY, "[ONNX-POWER-MATCH] count=%d abs=%u f=%d s=%d: P_in=%.6f P_out=%.6f alpha=%.6f RMS=%.6f max_abs=%.6f\n",
                  onnx_scale_log_count, abs_slot, frame, slot, P_in, P_out, alpha, onnx_output_rms, onnx_max_abs);
            onnx_scale_log_count++;
          }
          
          // Power matching scaling: alpha = sqrt(P_in / P_out)
          // This ensures the denormalized ONNX output has the same average power as the input
          // This is more stable than using RMS alone, as it matches the actual channel power
          const float fixed_scale = alpha;
          
          // Track clipping statistics
          int clip_count = 0;
          
          for (int rx = 0; rx < 4; rx++) {
            for (int tx = 0; tx < 4; tx++) {
              // Use fixed scale (no max_abs based normalization)
              float re_float = onnx_output[idx] * fixed_scale;
              float im_float = onnx_output[idx + 1] * fixed_scale;
              
              // Convert to Q15 fixed-point (multiply by 32768.0)
              // Use lrintf() for rounding instead of truncation to avoid bias
              int32_t re_q15 = (int32_t)lrintf(re_float * 32768.0f);
              int32_t im_q15 = (int32_t)lrintf(im_float * 32768.0f);
              
              // DO NOT apply scale_shift - ONNX output maintains its model-defined scale
              // Legacy scale matching (Q15 → Q(30-shift)) is disabled to prevent excessive clipping
              // ONNX output stays in Q15 format
              
              // Clip to int16_t range (same as csi_rs_ls_per_rb storage)
              if (re_q15 > 32767) {
                re_q15 = 32767;
                clip_count++;
              }
              if (re_q15 < -32768) {
                re_q15 = -32768;
                clip_count++;
              }
              if (im_q15 > 32767) {
                im_q15 = 32767;
                clip_count++;
              }
              if (im_q15 < -32768) {
                im_q15 = -32768;
                clip_count++;
              }
              
              H_onnx[rx][tx].r = (int16_t)re_q15;
              H_onnx[rx][tx].i = (int16_t)im_q15;
              
              idx += 2;
            }
          }
          
          // Log clipping statistics if clipping occurred
          if (clip_count > 0 && onnx_scale_log_count < 10) {
            LOG_W(NR_PHY, "[ONNX-OUTPUT-CLIP] abs=%u f=%d s=%d: clip_count=%d/32 (RMS=%.6f max_abs=%.6f)\n",
                  abs_slot, frame, slot, clip_count, onnx_output_rms, onnx_max_abs);
          }
          
          // Compute H_onnx^H × H_onnx matrix
          // Use int32 accumulation and proper scaling: Q15 × Q15 → Q30, then >>15 to Q15
          // This matches the legacy code format (legacy uses >>log2_maxh, but >>15 is standard for Q30→Q15)
          c16_t A_MF_onnx[4][4];
          memset(A_MF_onnx, 0, sizeof(A_MF_onnx));
          
          // A_MF = H^H × H: (H^H × H)[i][j] = sum_k(conj(H[k][i]) * H[k][j])
          // Accumulate in Q30 format for better precision, then scale once at the end
          for (int port_tx_i = 0; port_tx_i < 4; port_tx_i++) {
            for (int port_tx_j = 0; port_tx_j < 4; port_tx_j++) {
              int32_t sum_r = 0;  // Q30 accumulator
              int32_t sum_i = 0;  // Q30 accumulator
              for (int ant_rx = 0; ant_rx < 4; ant_rx++) {
                // Get H_onnx[ant_rx][port_tx_i] and compute its conjugate
                int32_t conj_h_r = (int32_t)H_onnx[ant_rx][port_tx_i].r;
                int32_t conj_h_i = -(int32_t)H_onnx[ant_rx][port_tx_i].i;
                // Get H_onnx[ant_rx][port_tx_j]
                int32_t h_r = (int32_t)H_onnx[ant_rx][port_tx_j].r;
                int32_t h_i = (int32_t)H_onnx[ant_rx][port_tx_j].i;
                
                // Compute product: conj(H) * H
                // Q15 × Q15 → Q30, accumulate in Q30 format
                sum_r += (conj_h_r * h_r) - (conj_h_i * h_i);
                sum_i += (conj_h_r * h_i) + (conj_h_i * h_r);
              }
              
              // Scale from Q30 to Q15: right shift by 15 with rounding
              // Add 1<<14 for rounding (rounds to nearest)
              sum_r = (sum_r + (1 << 14)) >> 15;
              sum_i = (sum_i + (1 << 14)) >> 15;
              
              // Clip to int16_t range and store
              if (sum_r > 32767) sum_r = 32767;
              if (sum_r < -32768) sum_r = -32768;
              if (sum_i > 32767) sum_i = 32767;
              if (sum_i < -32768) sum_i = -32768;
              
              A_MF_onnx[port_tx_i][port_tx_j].r = (int16_t)sum_r;
              A_MF_onnx[port_tx_i][port_tx_j].i = (int16_t)sum_i;
            }
          }
          
          // Store H_onnx and A_MF_onnx globally for use in RI/PMI/SINR calculations
          memcpy(g_H_onnx, H_onnx, sizeof(H_onnx));
          memcpy(g_A_MF_onnx, A_MF_onnx, sizeof(A_MF_onnx));
          g_onnx_H_ready = 1;
          g_onnx_A_MF_ready = 1;
          
          // Replicate g_H_onnx[4][4] to all RBs: g_H_onnx_per_rb[51][4][4]
          // g_H_onnx[4][4] is the ONNX output from g_H_avg_rb[4][4] input
          // Copy the same H_onnx[4][4] to all RB indices
          int max_rbs = (csirs_config_pdu->nr_of_rbs > 51) ? 51 : csirs_config_pdu->nr_of_rbs;
          for (int rb_idx = 0; rb_idx < max_rbs; rb_idx++) {
            for (int rx = 0; rx < 4; rx++) {
              for (int tx = 0; tx < 4; tx++) {
                g_H_onnx_per_rb[rb_idx][rx][tx] = g_H_onnx[rx][tx];
              }
            }
          }
          g_H_onnx_per_rb_ready = 1;
          
        } else {
          LOG_W(NR_PHY, "[ONNX] Model execution failed with return code %d\n", ret);
        }
      } else {
        // Log skip reason: ONNX init not ready
        int frame = proc->frame_rx;
        int slot = proc->nr_slot_rx;
        uint32_t abs_slot = (uint32_t)(frame * frame_parms->slots_per_frame + slot);
        LOG_I(NR_PHY, "[UCI-CSI] abs=%u f=%d s=%d SKIP reason=ONNX_NOT_READY\n", abs_slot, frame, slot);
        LOG_W(NR_PHY, "[ONNX] skip run: init not ready\n");
        // Update statistics: ONNX not ready
        g_csi_stat.skip_not_ready++;
      }
    } else {
      // Log skip reason: No valid RB found
      int frame = proc->frame_rx;
      int slot = proc->nr_slot_rx;
      uint32_t abs_slot = (uint32_t)(frame * frame_parms->slots_per_frame + slot);
      LOG_I(NR_PHY, "[UCI-CSI] abs=%u f=%d s=%d SKIP reason=NO_VALID_RB\n", abs_slot, frame, slot);
      LOG_W(NR_PHY, "[ONNX] No valid RB found for H_seed extraction\n");
    }
  }

  // Log CSI source (ONNX or LEGACY) before RI/PMI calculations
  {
    nrUE_params_t *nrUE_params = get_nrUE_params();
    int use_onnx = (nrUE_params && nrUE_params->onnx == 1 && g_onnx_H_ready && g_onnx_A_MF_ready);
    int frame = proc->frame_rx;
    int slot = proc->nr_slot_rx;
    uint32_t abs_slot = (uint32_t)(frame * frame_parms->slots_per_frame + slot);
    const char *src = use_onnx ? "ONNX" : "LEGACY";
    LOG_I(NR_PHY, "[CSI-SRC] abs=%u f=%d s=%d src=%s\n", abs_slot, frame, slot, src);
    
    // If ONNX is enabled but not ready, log skip reason
    if (nrUE_params && nrUE_params->onnx == 1 && !use_onnx) {
      LOG_I(NR_PHY, "[UCI-CSI] abs=%u f=%d s=%d SKIP reason=ONNX_NOT_READY\n", abs_slot, frame, slot);
      // Update statistics: ONNX not ready
      g_csi_stat.skip_not_ready++;
    }
  }

  // RI와 PMI를 함께 선택: 각 rank에 대해 PMI 선택 후 totalSINR 비교
  uint8_t rank_indicator = 0;
  uint8_t i1[3] = {0};
  uint8_t i2[1] = {0};
  uint8_t cqi = 0;
  uint32_t precoded_sinr_dB = 0;
  
  // bit 3 in bitmap to indicate PMI measurement
  if (csirs_config_pdu->measurement_bitmap & 8) {
    // Use nVar (noise_power) for PMI calculation instead of interference_plus_noise_power
    // noise_power is calculated from nvar_per_rb average (y - h_LS * x)
    uint32_t nvar_for_pmi = noise_power;  // nVar from LS estimation
    // SINRperRB[RB][Layer][PMI] 배열 선언: 최대 51 RB, 2 layers, 32 PMI candidates
    // uint64_t로 변경하여 포화 제거 (사용되지 않는 변수이지만 타입 일관성을 위해 변경)
    uint64_t sinr_per_rb[51][2][32] = {{{0}}};
    
    // 각 rank에 대해 PMI 선택 및 totalSINR 계산
    uint8_t i1_rank0[3] = {0};
    uint8_t i2_rank0[1] = {0};
    uint32_t total_sinr_rank0 = 0;  // Project_debugging7과 동일하게 uint32_t 사용
    uint32_t precoded_sinr_dB_rank0 = 0;
    
    uint8_t i1_rank1[3] = {0};
    uint8_t i2_rank1[1] = {0};
    uint32_t total_sinr_rank1 = 0;  // Project_debugging7과 동일하게 uint32_t 사용
    uint32_t precoded_sinr_dB_rank1 = 0;
    
    // Rank 0 (rank 1)에 대해 PMI 선택
    {
      uint64_t sinr_per_rb_rank0[51][2][32] = {{{0}}};
      nr_csi_rs_pmi_estimation(ue,
                               csirs_config_pdu,
                               csi_info,
                               mapping_parms.ports,
                               mem_offset,
                               csi_rs_estimated_channel_freq,
                               nvar_for_pmi,
                               0,  // rank_indicator = 0 (rank 1)
                               log2_re,
                               csi_rs_ls_per_rb,
                               nvar_per_rb,
                               sinr_per_rb_rank0,
                               i1_rank0,
                               i2_rank0,
                               &precoded_sinr_dB_rank0);
      
      // Rank 0의 totalSINR 계산: 선택된 PMI의 모든 RE와 Layer에 대한 SINR 합산
      // totalSINR[r] = 합산(모든 RE, 모든 Layer) SINR[RE, Layer, 선택된 PMI[r], rank=r]
      // rank=1일 때는 layer 0만 사용
      uint64_t total_sinr_sum_rank0 = 0;
      int selected_pmi_rank0 = 0;
      if (mapping_parms.ports == 2) {
        selected_pmi_rank0 = i2_rank0[0];  // 2-port rank1: i2가 PMI 인덱스
      } else if (mapping_parms.ports == 4) {
        selected_pmi_rank0 = i1_rank0[0] * 2 + i2_rank0[0];  // 4-port rank1: i11*2 + i2
      }
      
      for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
        int rb = csirs_config_pdu->start_rb + rb_idx;
        if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
          continue;
        }
        const int re_per_rb = 12;
        // Rank 0: Layer 0만 사용
        total_sinr_sum_rank0 += (uint64_t)sinr_per_rb_rank0[rb_idx][0][selected_pmi_rank0] * re_per_rb;
      }
      // rank=1의 totalSINR: Q20에서 일반 값으로 변환 (Project_debugging7과 동일)
      // Project_debugging7에서는 평균을 내지 않고 단순히 스케일만 변환
      total_sinr_rank0 = (uint32_t)(total_sinr_sum_rank0 >> 20);  // Q20에서 일반 값으로 변환 (대략)
    }
    
    // Rank 1 (rank 2)에 대해 PMI 선택 (2-port 또는 4-port인 경우만)
    if (mapping_parms.ports == 2 || mapping_parms.ports == 4) {
      uint64_t sinr_per_rb_rank1[51][2][32] = {{{0}}};
      nr_csi_rs_pmi_estimation(ue,
                               csirs_config_pdu,
                               csi_info,
                               mapping_parms.ports,
                               mem_offset,
                               csi_rs_estimated_channel_freq,
                               nvar_for_pmi,
                               1,  // rank_indicator = 1 (rank 2)
                               log2_re,
                               csi_rs_ls_per_rb,
                               nvar_per_rb,
                               sinr_per_rb_rank1,
                               i1_rank1,
                               i2_rank1,
                               &precoded_sinr_dB_rank1);
      
      // Rank 1의 totalSINR 계산: 선택된 PMI의 모든 RE와 Layer에 대한 SINR 합산
      // totalSINR[r] = 합산(모든 RE, 모든 Layer) SINR[RE, Layer, 선택된 PMI[r], rank=r]
      // rank=2일 때는 두 layer를 합산 (Project_debugging7과 동일)
      uint64_t total_sinr_sum_rank1 = 0;
      int selected_pmi_rank1 = 0;
      if (mapping_parms.ports == 2) {
        selected_pmi_rank1 = i2_rank1[0];  // 2-port rank2: i2가 PMI 인덱스
      } else if (mapping_parms.ports == 4) {
        selected_pmi_rank1 = i1_rank1[0] * 4 + i1_rank1[2] * 2 + i2_rank1[0];  // 4-port rank2: i11*4 + i13*2 + i2
      }
      
      for (int rb_idx = 0; rb_idx < csirs_config_pdu->nr_of_rbs; rb_idx++) {
        int rb = csirs_config_pdu->start_rb + rb_idx;
        if (csirs_config_pdu->freq_density <= 1 && csirs_config_pdu->freq_density != (rb % 2)) {
          continue;
        }
        const int re_per_rb = 12;
        // Rank 1: Layer 0과 Layer 1 모두 사용 (평균 내지 않고 합산)
        // Project_debugging7과 동일하게 합산만 수행
        total_sinr_sum_rank1 += (uint64_t)sinr_per_rb_rank1[rb_idx][0][selected_pmi_rank1] * re_per_rb;
        total_sinr_sum_rank1 += (uint64_t)sinr_per_rb_rank1[rb_idx][1][selected_pmi_rank1] * re_per_rb;
      }
      // rank=2의 totalSINR: Q20에서 일반 값으로 변환 (Project_debugging7과 동일)
      // Project_debugging7에서는 평균을 내지 않고 단순히 스케일만 변환
      total_sinr_rank1 = (uint32_t)(total_sinr_sum_rank1 >> 20);  // Q20에서 일반 값으로 변환 (대략)
    }
    
    // RI 선택: totalSINR이 최대인 rank 선택
    // totalSINR[r] = f(SINR[선택된 PMI[r], rank=r])
    // RI = argmax(totalSINR[r])
    if (mapping_parms.ports == 2 || mapping_parms.ports == 4) {
      if (total_sinr_rank1 > total_sinr_rank0) {
        rank_indicator = 1;  // Rank 2 선택
        i1[0] = i1_rank1[0];
        i1[1] = i1_rank1[1];
        i1[2] = i1_rank1[2];
        i2[0] = i2_rank1[0];
        precoded_sinr_dB = precoded_sinr_dB_rank1;
      } else {
        rank_indicator = 0;  // Rank 1 선택
        i1[0] = i1_rank0[0];
        i1[1] = i1_rank0[1];
        i1[2] = i1_rank0[2];
        i2[0] = i2_rank0[0];
        precoded_sinr_dB = precoded_sinr_dB_rank0;
      }
    } else {
      // 1-port인 경우 rank 0만 가능
      rank_indicator = 0;
      i1[0] = i1_rank0[0];
      i1[1] = i1_rank0[1];
      i1[2] = i1_rank0[2];
      i2[0] = i2_rank0[0];
      precoded_sinr_dB = precoded_sinr_dB_rank0;
    }
    
    // Log i13 calculation result with frame/slot info (after PMI estimation)
    {
      int frame = proc->frame_rx;
      int slot = proc->nr_slot_rx;
      uint32_t abs_slot = (uint32_t)(frame * frame_parms->slots_per_frame + slot);
      LOG_I(NR_PHY, "[PMI-i13] abs=%u f=%d s=%d rank=%d i11=%d i13=%d i2=%d sinr_dB=%u totalSINR_rank0=%u totalSINR_rank1=%u\n",
            abs_slot, frame, slot, rank_indicator + 1, i1[0], i1[2], i2[0], precoded_sinr_dB,
            total_sinr_rank0, total_sinr_rank1);
    }

    // bit 4 in bitmap to indicate CQI measurement
    // 선택된 최종 SINR을 CQI 매핑 테이블에 연결
    if(csirs_config_pdu->measurement_bitmap & 16) {
      // precoded_sinr_dB는 이미 최종 선택된 rank의 SINR (dB 단위)
      // CQI estimation 함수에 전달하여 CQI 매핑 테이블에서 CQI 값 결정
      nr_csi_rs_cqi_estimation(precoded_sinr_dB, &cqi);
      
      // Log CQI selection with final SINR
      {
        int frame = proc->frame_rx;
        int slot = proc->nr_slot_rx;
        uint32_t abs_slot = (uint32_t)(frame * frame_parms->slots_per_frame + slot);
        LOG_I(NR_PHY, "[CQI] abs=%u f=%d s=%d rank=%d final_sinr_dB=%u cqi=%d\n",
              abs_slot, frame, slot, rank_indicator + 1, precoded_sinr_dB, cqi);
      }
    }
  } else {
    // PMI measurement가 없으면 기존 RI estimation 사용
    // bit 1 in bitmap to indicate RI measurement
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

  // Encode i1: i11 (3 bits, 0-7) in lower 3 bits, i13 (1 bit, 0-1) in bit 3
  // i1_encoded = i11 | (i13 << 3)
  uint16_t i1_encoded = i1[0] | (i1[2] << 3);
  
  fapi_nr_l1_measurements_t l1_measurements = {
    .gNB_index = proc->gNB_id,
    .meas_type = NFAPI_NR_CSI_MEAS,
    .Nid_cell = frame_parms->Nid_cell,
    .is_neighboring_cell = false,
    .rsrp_dBm = rsrp_dBm,
    .rank_indicator = rank_indicator,
    .i1 = i1_encoded,
    .i2 = *i2,
    .cqi = cqi,
    .radiolink_monitoring = RLM_no_monitoring, // TODO do be activated in case of RLM based on CSI-RS
  };
  nr_downlink_indication_t dl_indication;
  fapi_nr_rx_indication_t rx_ind = {0};
  nr_fill_dl_indication(&dl_indication, NULL, &rx_ind, proc, ue, NULL);
  nr_fill_rx_indication(&rx_ind, FAPI_NR_MEAS_IND, ue, NULL, NULL, 1, proc, (void *)&l1_measurements, NULL);
  ue->if_inst->dl_indication(&dl_indication);
  
  // Check deadline miss (only when ONNX is enabled)
  if (deadline_check_enabled && start_us > 0) {
    if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts_end) == 0) {
      uint64_t end_us = (uint64_t)ts_end.tv_sec * 1000000ULL + (uint64_t)ts_end.tv_nsec / 1000ULL;
      uint64_t total_us = end_us - start_us;
      
      // Calculate slot budget in microseconds: 1000us / (2^numerology_index)
      // Slot duration = 1.0 / (2^numerology_index) ms = 1000.0 / (2^numerology_index) us
      uint64_t budget_us = (uint64_t)(1000.0 / (1 << frame_parms->numerology_index));
      
      int frame = proc->frame_rx;
      int slot = proc->nr_slot_rx;
      uint32_t abs_slot = (uint32_t)(frame * frame_parms->slots_per_frame + slot);
      
      if (total_us > budget_us) {
        LOG_W(NR_PHY, "[CSI-DEADLINE] abs=%u total_us=%llu WARN budget_us=%llu\n",
              abs_slot, (unsigned long long)total_us, (unsigned long long)budget_us);
        // Update statistics: deadline miss
        g_csi_stat.deadline_miss++;
      }
    }
  }
}
