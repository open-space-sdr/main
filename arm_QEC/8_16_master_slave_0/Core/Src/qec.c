/*
 * qec.c
 *
 */

#include "qec.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "max2850.h"
#include "max2851.h"
#include "app_log.h"
#include "main.h" // For HAL_Delay
#include <math.h> // For roundf, fabsf, sqrtf, atan2, sinf, cosf

// Forward declarations for local helper functions
void correlator_measurement(uint16_t rxqec0_txqec1, uint16_t qec_pow2_num_samples, int32_t * corr_ii, int32_t * corr_iq, int32_t * corr_qi, int32_t * corr_qq);
void qec_calc(float X11, float X12, float X22, float * gain_correction, float * phase_correction);
void apply_rxqec_correction(float gain, float phase);
void apply_txqec_correction(uint16_t ant_idx, float gain, float phase, float delay, float dc_i, float dc_q);


/**
 * @brief Sets the frequency of the CW tone generator in the FPGA.
 * @param freq_mhz The desired frequency in MHz, relative to the LO.
 * The FPGA expects a scaled 16-bit integer.
 * @return void.
 */
void set_tx_tone_freq(float freq_mhz) {
    reg_write(0x24, (int16_t)roundf(freq_mhz / 80.0f * 65536.0f));
    uint16_t ctrl_val = reg_read(REG_CTRL);
    reg_write(REG_CTRL, ctrl_val | CTRL_FORCE_CW_TONE);
    HAL_Delay(1);
}

void QEC(void){

  // changed skip mask from 0x10 to 0x00 to enable antenna 4.
  const int16_t channel_skip_mask = 0x00; // 1 bit indicates channel should be skipped.

  const uint16_t qec_pow2_num_samples = 20;
  const float rssi_target = 85.0;
  const uint16_t transmit_level = 0x15;

  reg_write(0x65, 0);
  reg_write(REG_SPI_MAX2850, (15 << 10) | 0x246);
  reg_write(REG_SPI_MAX2850, (16 << 10) | 0x00C);
  reg_write(REG_SPI_MAX2850, (17 << 10) | 0x333);
  HAL_Delay(50);

  app_log("- Receiver Quadrature Error Correction (Rx-QEC) -\r\n");
  float freq_measure;
  int16_t best_gain_per_ch[5] = {0};
  for(int16_t ant_idx = 1; ant_idx<=4; ant_idx++){

    if( channel_skip_mask & (1 << ant_idx) )
      continue;

    app_log("\r\nAntenna %d", ant_idx);

    set_Rx(0);
    reg_write(REG_RX_MUX, ant_idx);
    set_Tx(transmit_level, 1 << (ant_idx-1));

    apply_rxqec_correction(1.0, 0.0);
    apply_txqec_correction(ant_idx, 1.0, 0.0, 0.0, 0.0, 0.0);
    HAL_Delay(50);

    freq_measure = +17.501f;
    set_tx_tone_freq(freq_measure);
    reg_write(REG_TLM_MUX, TELEM_SEL_RSSI_D_IN);

    int16_t rx_gain_best = 0;
    float rssi_best = -1000.0f;

    for(int16_t gain_try=1;gain_try<20; gain_try++){
        set_Rx_gain(gain_try);
        HAL_Delay(1);
        float loopback_rssi = reg_read(REG_TLM_MUX);

        if(fabsf(loopback_rssi-rssi_target) < fabsf(rssi_best-rssi_target)){
          rx_gain_best = gain_try;
          rssi_best = loopback_rssi;
        }
    }
    best_gain_per_ch[ant_idx] = rx_gain_best;
    set_Rx_gain(rx_gain_best);
    app_log(" best gain was %d @ RSSI=%.1f\r\n", rx_gain_best, rssi_best);

    freq_measure = 17.501f;
    set_tx_tone_freq(+freq_measure - 1.0f);
    int32_t corr_pos_ii, corr_pos_iq, corr_pos_qi, corr_pos_qq;
    correlator_measurement(0, qec_pow2_num_samples, &corr_pos_ii, &corr_pos_iq, &corr_pos_qi, &corr_pos_qq);
    set_tx_tone_freq(-freq_measure - 1.0f);
    int32_t corr_neg_ii, corr_neg_iq, corr_neg_qi, corr_neg_qq;
    correlator_measurement(0, qec_pow2_num_samples, &corr_neg_ii, &corr_neg_iq, &corr_neg_qi, &corr_neg_qq);

    float gain_correction, phase_correction;
    qec_calc((float)(corr_pos_ii + corr_neg_ii), (float)(corr_pos_iq + corr_neg_iq), (float)(corr_pos_qq + corr_neg_qq), &gain_correction, &phase_correction);
    app_log("original  phase:%.3f gain:%.3f", phase_correction, gain_correction);

    apply_rxqec_correction(gain_correction, phase_correction);

    freq_measure = 17.501f;
    set_tx_tone_freq(+freq_measure - 1.0f);
    correlator_measurement(0, qec_pow2_num_samples, &corr_pos_ii, &corr_pos_iq, &corr_pos_qi, &corr_pos_qq);
    set_tx_tone_freq(-freq_measure - 1.0f);
    correlator_measurement(0, qec_pow2_num_samples, &corr_neg_ii, &corr_neg_iq, &corr_neg_qi, &corr_neg_qq);

    float phase_correction_pos, phase_correction_neg;
    qec_calc((float)(corr_pos_ii), (float)(corr_pos_iq), (float)(corr_pos_qq), &gain_correction, &phase_correction_pos);
    qec_calc((float)(corr_neg_ii), (float)(corr_neg_iq), (float)(corr_neg_qq), &gain_correction, &phase_correction_neg);

    float delay_est = ((phase_correction_pos - phase_correction_neg)/2.0f) / (6.2831853f*fabsf(freq_measure)/80.0f);
    app_log(" delay:%.3f", delay_est);

    qec_calc((float)(corr_pos_ii + corr_neg_ii), (float)(corr_pos_iq + corr_neg_iq), (float)(corr_pos_qq + corr_neg_qq), &gain_correction, &phase_correction);

    app_log("\r\ncorrected phase:%.3f gain:%.3f", phase_correction, gain_correction);

  }

  reg_write(REG_SPI_MAX2850, (15 << 10) | 0x246);
  reg_write(REG_SPI_MAX2850, (16 << 10) | 0x000);
  reg_write(REG_SPI_MAX2850, (17 << 10) | 0x000);
  HAL_Delay(50);

  app_log("\r\n\r\n- Transmit Quadrature Error Correction (Tx-QEC) -\r\n");
  for(int16_t ant_idx = 1; ant_idx<=4; ant_idx++){

    if( channel_skip_mask & (1 << ant_idx) )
      continue;

    app_log("\r\nAntenna %d", ant_idx);

    set_Rx(0);
    reg_write(REG_RX_MUX, ant_idx);
    set_Tx(transmit_level, 1 << (ant_idx-1));
    apply_txqec_correction(ant_idx, 1.0, 0.0, 0.0, 0.0, 0.0);
    set_Rx_gain(best_gain_per_ch[ant_idx]);

    float tx_gain_correction = 1.0f;
    float tx_phase_correction = 0.0f;
    float tx_delay_correction = 0.0f;

    // no significant change after 6 iterations
    for (int k=0; k<6; k++){
      freq_measure = 17.42321f;
      set_tx_tone_freq(+freq_measure);
      int32_t corr_pos_ii, corr_pos_iq, corr_pos_qi, corr_pos_qq;
      correlator_measurement(1, qec_pow2_num_samples, &corr_pos_ii, &corr_pos_iq, &corr_pos_qi, &corr_pos_qq);
      set_tx_tone_freq(-freq_measure);
      int32_t corr_neg_ii, corr_neg_iq, corr_neg_qi, corr_neg_qq;
      correlator_measurement(1, qec_pow2_num_samples, &corr_neg_ii, &corr_neg_iq, &corr_neg_qi, &corr_neg_qq);

      double B = corr_pos_ii + corr_neg_ii;
      double D = corr_pos_iq + corr_neg_iq;
      double A = corr_pos_qi + corr_neg_qi;
      double C = corr_pos_qq + corr_neg_qq;
      double Bp = corr_pos_ii - corr_neg_ii;
      double Dp = corr_pos_iq - corr_neg_iq;
      double Ap = corr_pos_qi - corr_neg_qi;
      double Cp = corr_pos_qq - corr_neg_qq;
      double G = 2.0*sqrt(A*A + Dp*Dp + B*B + Cp*Cp);
      double gtx = 2.0*sqrt(D*D + Ap*Ap + C*C + Bp*Bp) / G;
      double phase_tx = (atan2(-A-Dp, +B-Cp) + atan2(-A+Dp, +B+Cp))/2.0 - (atan2(D+Ap, C-Bp) + atan2(D-Ap, C+Bp))/2.0;
      double delay_tx = (atan2(-A-Dp, +B-Cp) - atan2(-A+Dp, +B+Cp))/2.0 - (atan2(D+Ap, C-Bp) - atan2(D-Ap, C+Bp))/2.0;
      delay_tx = delay_tx / (6.2831853*fabs(freq_measure)) * 80.0;
      app_log("\r\n G:%.2f phase_tx:%.4f gain_tx:%.4f delay_tx:%.4f", G, phase_tx, gtx, delay_tx);
      tx_gain_correction = tx_gain_correction * gtx;
      tx_phase_correction = tx_phase_correction + phase_tx;
      tx_delay_correction = tx_delay_correction + delay_tx;
      apply_txqec_correction(ant_idx, tx_gain_correction, tx_phase_correction, 0.0, 0.0, 0.0);
      HAL_Delay(50);
    }
  }

  reg_write(REG_CTRL, reg_read(REG_CTRL) & ~CTRL_FORCE_CW_TONE);
  app_log("\r\nQEC complete.\r\n");
  // removed infinite loop so program can continue
  return;
}

void qec_calc(float X11, float X12, float X22, float * gain_correction, float * phase_correction){
  float q_scale_prime = 1.0f / sqrtf(X22*X11 - X12*X12);
  *phase_correction = -X12*q_scale_prime;
  *gain_correction  = +X11*q_scale_prime;
  return;
}

void correlator_measurement(uint16_t rxqec0_txqec1, uint16_t qec_pow2_num_samples, int32_t * corr_ii, int32_t * corr_iq, int32_t * corr_qi, int32_t * corr_qq)
{
  uint16_t temp;
   reg_write(REG_QEC_EN, (qec_pow2_num_samples<<2) | (rxqec0_txqec1 << 1) | 1);
   reg_write(REG_CTRL, reg_read(REG_CTRL) & ~CTRL_FORCE_CW_TONE);
   reg_write(REG_CTRL, reg_read(REG_CTRL) | CTRL_FORCE_CW_TONE);
   reg_write(REG_QEC_EN, 0x00);
   HAL_Delay(20);
   reg_write(REG_QEC_EN, 0 << 7);
   temp = reg_read(REG_QEC_EN);
   reg_write(REG_QEC_EN, 1 << 7);
   *corr_ii = (int32_t)((((uint32_t)reg_read(REG_QEC_EN)) << 16) | temp);
   reg_write(REG_QEC_EN, 2 << 7);
   temp = reg_read(REG_QEC_EN);
   reg_write(REG_QEC_EN, 3 << 7);
   *corr_iq = (int32_t)((((uint32_t)reg_read(REG_QEC_EN)) << 16) | temp);
   reg_write(REG_QEC_EN, 4 << 7);
   temp = reg_read(REG_QEC_EN);
   reg_write(REG_QEC_EN, 5 << 7);
   *corr_qi = (int32_t)((((uint32_t)reg_read(REG_QEC_EN)) << 16) | temp);
   reg_write(REG_QEC_EN, 6 << 7);
   temp = reg_read(REG_QEC_EN);
   reg_write(REG_QEC_EN, 7 << 7);
   *corr_qq = (int32_t)((((uint32_t)reg_read(REG_QEC_EN)) << 16) | temp);
    if ((rxqec0_txqec1==0) && (*corr_qi != *corr_iq) ) {
      app_log("\r\n *** RxQEC CORRELATOR GLITCH ASSERTION FAILED ***\r\n");
      return;
  }
  return;
}

void apply_rxqec_correction(float gain, float phase){
  int8_t phase_int8, gain_int8;
  if (fabsf(phase) > 0.24f || fabsf(gain - 1.0f) > 0.24f ) {
      app_log("\r\n *** RxQEC Correction out of Bounds: phase=%.3f, gain=%.3f ***\r\n", phase, gain);
      return;
  }
  phase_int8 = roundf(phase*256.0f);
  gain_int8  = roundf((gain - 1.0f)*256.0f);
  uint16_t phase_unsigned = (phase_int8 >= 0) ? phase_int8 : ((int16_t)phase_int8 + (int16_t)256);
  uint16_t gain_unsigned =  (gain_int8  >= 0) ? gain_int8  : ((int16_t)gain_int8  + (int16_t)256);
  uint16_t write_val = (phase_unsigned *256) | gain_unsigned;
  reg_write(0x63, write_val);
  HAL_Delay(10);
  return;
}

void apply_txqec_correction(uint16_t ant_idx, float gain, float phase, float delay, float dc_i, float dc_q)
{
  gain = gain/cosf(phase) - 1.0f;
  phase = sinf(phase);
  delay = (delay*2.0f)/2.0f;
  uint16_t write_value;
  write_value = ((int16_t)roundf(dc_q * 256.0f));
  reg_write(0x27, (ant_idx<<8) | (write_value & 0xff));
  write_value = ((int16_t)roundf(dc_i * 256.0f));
  reg_write(0x27, (ant_idx<<8) | (write_value & 0xff));
  write_value = ((int16_t)roundf(delay * 1024.0f));
  reg_write(0x27, (ant_idx<<8) | (write_value & 0xff));
  write_value = ((int16_t)roundf(phase * 512.0f));
  reg_write(0x27, (ant_idx<<8) | (write_value & 0xff));
  write_value = ((int16_t)roundf(gain * 512.0f));
  reg_write(0x27, (ant_idx<<8) | (write_value & 0xff));
  return;
}
