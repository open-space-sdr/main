#define MAX2850_REG_ADDR 0x42

// Quadrature Error Correction
// Perform Rx-QEC on all channels first before Tx-QEC because moving offsetting main Tx mixer PLL changes Tx side impairments.
void QEC(Stream &FPGA){ // channel can be skipped 
 
  int16_t channel_skip_mask = 0x00; // 1 bit indicates channel should be skipped. 0x10 is ch4

  // Settings
    uint16_t qec_pow2_num_samples = 20; 
    float rssi_target = 85.0; // empirical best RMS digital signal level in receiver
    uint16_t transmit_level = 0x15; // may want to let caller set this
  

  // TODO! Each channel stores corrections! Swap with mux or have dedicated correction blocks!

  // ensure digital loopback disabled
  reg_write(FPGA, 0x65, 0); 
  // Tune Tx-PLL MAX2850 to +1 Mhz offset to eliminate correlation effect of Tx quadrature errors on Rx measurements.
  // Note, when we do this, it may invalidate all Tx-QEC as impairments can change with PLL reset
  // So need to do Tx-QEC on all all-channels _after_ we're done with Rx-QEC
  reg_write(FPGA, MAX2850_REG_ADDR, (15 << 10) | 0x246);
  reg_write(FPGA, MAX2850_REG_ADDR, (16 << 10) | 0x00C); 
  reg_write(FPGA, MAX2850_REG_ADDR, (17 << 10) | 0x333); 
  delay(50);

  // TODO should just force tone here instead of using as trigger
 

  // Perform Rx-QEC on all channels first before Tx-QEC because moving offsetting main Tx mixer PLL changes Tx side impairments. 
  Serial.print("--Receiver Quadrature Error Correction (Rx-QEC)--\n");
  float freq_measure; int16_t best_gain_per_ch[5] = {0};
  for(int16_t ant_idx = 1; ant_idx<=4; ant_idx++){ 

    if( channel_skip_mask & (1 << ant_idx) )
      continue; // skip this channel if present in skip mask

    Serial.print("\nAntenna "); Serial.print(ant_idx, DEC);

    set_Rx(FPGA, 0); // put into Rx first, zero gain
    if(ant_idx!=4)
        reg_write(FPGA, 0x22, ant_idx);  // set Rx MUX on FPGA to this antenna
     else
      reg_write(FPGA, 0x22, 5); // use rx 5 for 4th antenna

    set_Tx(FPGA, transmit_level, 1 << (ant_idx-1)); // want relatively high gain to emulate real tx, this also switches RFSW to Tx

    // clear all existing Rx-QEC & Tx-QEC corrections
    apply_rxqec_correction(FPGA, 1.0, 0.0);
    apply_txqec_correction(FPGA, ant_idx, 1.0, 0.0, 0.0,   0.0,0.0); // eliminate initial tx gain/delay corrections
    delay(50);

    // find the best (small) gain to hear own loopback
    freq_measure = +17.501;
    set_tx_tone_freq(FPGA, freq_measure); // gain search tone freq
    reg_write(FPGA, 0x4F, 2);  // set telemetry mux to demod rssi

    
    int16_t rx_gain_best = 0; float rssi_best = 0.0;
    for(int16_t gain_try=1;gain_try<20; gain_try++){
        set_Rx_gain(FPGA, gain_try); // set a gain to try
        delay(1);
        float loopback_rssi = reg_read(FPGA, 0x4F); // check current rssi

        if(fabs(loopback_rssi-rssi_target) < fabs(rssi_best-rssi_target)){
          // found a better gain
          rx_gain_best = gain_try; //save
          rssi_best = loopback_rssi;
        }
    }
    best_gain_per_ch[ant_idx] = rx_gain_best;
    set_Rx_gain(FPGA, rx_gain_best); // set to best gain found
    Serial.print(" best gain was "); Serial.print(rx_gain_best, DEC); Serial.print(" @ RSSI="); Serial.print(rssi_best, 1); Serial.print("\n");

    //// Begin Rx-QEC ////
    // Take measurements
    freq_measure = 17.501;
    set_tx_tone_freq(FPGA, +freq_measure - 1.0); // Subtract the Tx RF PLL offset
    int32_t corr_pos_ii, corr_pos_iq, corr_pos_qi, corr_pos_qq;
    correlator_measurement(FPGA, 0, qec_pow2_num_samples, &corr_pos_ii, &corr_pos_iq, &corr_pos_qi, &corr_pos_qq);
    set_tx_tone_freq(FPGA, -freq_measure - 1.0); // Subtract the Tx RF PLL offset
    int32_t corr_neg_ii, corr_neg_iq, corr_neg_qi, corr_neg_qq;
    correlator_measurement(FPGA, 0, qec_pow2_num_samples, &corr_neg_ii, &corr_neg_iq, &corr_neg_qi, &corr_neg_qq); 

    float gain_correction, phase_correction;
    qec_calc((float)(corr_pos_ii + corr_neg_ii), (float)(corr_pos_iq + corr_neg_iq), (float)(corr_pos_qq + corr_neg_qq), &gain_correction, &phase_correction);
    Serial.print("original  phase:"); Serial.print(phase_correction, 3);
    Serial.print(" gain:"); Serial.print(gain_correction, 3);

    // correct rxqec based on estimate
    apply_rxqec_correction(FPGA, gain_correction, phase_correction);

    // Measure again, also estimating delay now
    freq_measure = 17.501;
    set_tx_tone_freq(FPGA, +freq_measure - 1.0); // Subtract the Tx RF PLL offset
    correlator_measurement(FPGA, 0, qec_pow2_num_samples, &corr_pos_ii, &corr_pos_iq, &corr_pos_qi, &corr_pos_qq);
    set_tx_tone_freq(FPGA, -freq_measure - 1.0); // Subtract the Tx RF PLL offset
    correlator_measurement(FPGA, 0, qec_pow2_num_samples, &corr_neg_ii, &corr_neg_iq, &corr_neg_qi, &corr_neg_qq); 

    float phase_correction_pos, phase_correction_neg;
    qec_calc((float)(corr_pos_ii), (float)(corr_pos_iq), (float)(corr_pos_qq), &gain_correction, &phase_correction_pos);
    qec_calc((float)(corr_neg_ii), (float)(corr_neg_iq), (float)(corr_neg_qq), &gain_correction, &phase_correction_neg);

    float delay_est = ((phase_correction_pos - phase_correction_neg)/2.0) / (6.2831853*fabs(freq_measure)/80.0);
    Serial.print(" delay:"); Serial.print(delay_est, 3);

    /// don't correct delay for now ///
    qec_calc((float)(corr_pos_ii + corr_neg_ii), (float)(corr_pos_iq + corr_neg_iq), (float)(corr_pos_qq + corr_neg_qq), &gain_correction, &phase_correction);

    Serial.print("\ncorrected phase:"); Serial.print(phase_correction, 3);
    Serial.print(" gain:"); Serial.print(gain_correction, 3);

  } // done with Rx-QEC

  // Tune MAX2850 back to equal RF PLL frequencies
  reg_write(FPGA, MAX2850_REG_ADDR, (15 << 10) | 0x246);
  reg_write(FPGA, MAX2850_REG_ADDR, (16 << 10) | 0x000); 
  reg_write(FPGA, MAX2850_REG_ADDR, (17 << 10) | 0x000); 
  delay(50);
  //reg_write(FPGA, 0x23, 0x00);  // Force CW off

// Perform Tx-QEC on all channels
  Serial.print("\n\n--Transmit Quadrature Error Correction (Tx-QEC)--\n");
  for(int16_t ant_idx = 1; ant_idx<=4; ant_idx++){ 

    if( channel_skip_mask & (1 << ant_idx) )
      continue; // skip this channel if present in skip mask

    Serial.print("\nAntenna "); Serial.print(ant_idx, DEC);

  set_Rx(FPGA, 0); // put into Rx first, zero gain
  reg_write(FPGA, 0x22, ant_idx);  // set Rx MUX on FPGA to this antenna
  set_Tx(FPGA, transmit_level, 1 << (ant_idx-1)); // want relatively high gain to emulate real tx, this also switches RFSW to Tx
  // eliminate initial tx gain/delay corrections
  apply_txqec_correction(FPGA, ant_idx, 1.0, 0.0, 0.0,   0.0,0.0); 
  // use best gain previously found
  set_Rx_gain(FPGA, best_gain_per_ch[ant_idx]);


  float tx_gain_correction = 1.0;
  float tx_phase_correction = 0.0;
  float tx_delay_correction = 0.0;

  for (int k=0; k<6; k++){
  freq_measure = 17.42321;
  set_tx_tone_freq(FPGA, +freq_measure);
  int32_t corr_pos_ii, corr_pos_iq, corr_pos_qi, corr_pos_qq;
  correlator_measurement(FPGA, 1, qec_pow2_num_samples, &corr_pos_ii, &corr_pos_iq, &corr_pos_qi, &corr_pos_qq);
  set_tx_tone_freq(FPGA, -freq_measure);
  int32_t corr_neg_ii, corr_neg_iq, corr_neg_qi, corr_neg_qq;
  correlator_measurement(FPGA, 1, qec_pow2_num_samples, &corr_neg_ii, &corr_neg_iq, &corr_neg_qi, &corr_neg_qq); 

  // Calculate Tx quadrature error
  double B = corr_pos_ii + corr_neg_ii;
  double D = corr_pos_iq + corr_neg_iq;
  double A = corr_pos_qi + corr_neg_qi;
  double C = corr_pos_qq + corr_neg_qq;

  double Bp = corr_pos_ii - corr_neg_ii;
  double Dp = corr_pos_iq - corr_neg_iq;
  double Ap = corr_pos_qi - corr_neg_qi;
  double Cp = corr_pos_qq - corr_neg_qq;

  ///// ??? TxQEC still seems loopback delay sensitive?

  double G = 2.0*sqrt(A*A + Dp*Dp + B*B + Cp*Cp);
  double gtx = 2.0*sqrt(D*D + Ap*Ap + C*C + Bp*Bp) / G;
  double phase_tx = (atan2(-A-Dp, +B-Cp) + atan2(-A+Dp, +B+Cp))/2.0 - (atan2(D+Ap, C-Bp) + atan2(D-Ap, C+Bp))/2.0;
  double delay_tx = (atan2(-A-Dp, +B-Cp) - atan2(-A+Dp, +B+Cp))/2.0 - (atan2(D+Ap, C-Bp) - atan2(D-Ap, C+Bp))/2.0;

  double rf_loopback_value = (atan2(-A-Dp, +B-Cp) + atan2(-A+Dp, +B+Cp))/2.0;
  double bbi_loopback_value = (atan2(-A-Dp, +B-Cp) - atan2(-A+Dp, +B+Cp))/2.0; // includes I channel txbb delay and DRX and DRF*baseband freq
  double bbq_loopback_value = (atan2(D+Ap, C-Bp) - atan2(D-Ap, C+Bp))/2.0; // includes I channel txbb delay and DRX and DRF*baseband freq
  double sanity_delay_tx = bbi_loopback_value - bbq_loopback_value;

  delay_tx = delay_tx / (6.2831853*fabs(freq_measure)) * 80.0;
  Serial.print("\n");
  Serial.print(" G:"); Serial.print(G, 2);
  Serial.print(" phase tx:"); Serial.print(phase_tx, 4);
  Serial.print(" gain tx:"); Serial.print(gtx, 4);
  Serial.print(" delay tx:"); Serial.print(delay_tx, 4);

  Serial.print(" RF Loopback:"); Serial.print(rf_loopback_value, 4);
  Serial.print(" BBi Loopback:"); Serial.print(bbi_loopback_value, 4);
  Serial.print(" BBq Loopback:"); Serial.print(bbq_loopback_value, 4);
  Serial.print(" sanity_delay_tx:"); Serial.print(sanity_delay_tx, 4);
  // Delay looks like whole sigma delta sample off!

  // strongly looks like sign error in measurement because DTXI and DTXQ are opposite sign and very similar magnitude.
  // ? Due to negatives of I and Q in circuit? No, that doesn't affect.

  //if(k>=2){
  // try programming a correction
    tx_gain_correction = tx_gain_correction * gtx;
    tx_phase_correction = tx_phase_correction + phase_tx;
    tx_delay_correction = tx_delay_correction + delay_tx; // disabled delay correction for now

    apply_txqec_correction(FPGA, ant_idx, tx_gain_correction, tx_phase_correction, tx_delay_correction, 0.0, 0.0);
    delay(50);
  //} 

  } // 6 iterations
  
  //reg_write(FPGA, 0x23, 0x08); // tone enabled again
  //delay(10);
  //sdm_capture(FPGA);
  //waitForKeyPress();
    /* // demod capture
  reg_write(FPGA, 0x4D, 0);  // reset the capture
  delay(10);
  reg_write(FPGA, 0x4D, 1);  // arm
  delay(10);
  reg_write(FPGA, 0x41, 1); reg_write(FPGA, 0x41, 0); // force trigger
  delay(100);
  Serial.print("\n");
    for (int k = 0; k < 8191; k++) {
      reg_write(FPGA, 0x40, k);
      Serial.print(reg_read(FPGA, 0x4A), HEX);
      Serial.print(" ");
    }
  Serial.print("\n");*/
  } // per antenna
 
 
  reg_write(FPGA, 0x23, 0x00);  // Force CW off

  waitForKeyPress();
  return;
}

void qec_calc(float X11, float X12, float X22, float * gain_correction, float * phase_correction){

  float q_scale_prime = 1.0 / sqrtf(X22*X11 - X12*X12);

 *phase_correction = -X12*q_scale_prime;
 *gain_correction  = +X11*q_scale_prime;
  return;
}

void set_tx_tone_freq(Stream &FPGA, float freq_mhz){
  reg_write(FPGA, 0x24, (int16_t)round(freq_mhz/80*65536)); // MHz
  reg_write(FPGA, 0x23, 0x08); // enable force tone
  delay(1);
  return;
}

// Perform Rx QEC Correlation measurement
void correlator_measurement(Stream &FPGA, uint16_t rxqec0_txqec1, uint16_t qec_pow2_num_samples, int32_t * corr_ii, int32_t * corr_iq, int32_t * corr_qi, int32_t * corr_qq)
{
  uint16_t temp;
  
   reg_write(FPGA, 0x25, (qec_pow2_num_samples<<2) | (rxqec0_txqec1 << 1) | 1);  // rxqec or txqec
   reg_write(FPGA, 0x23, 0x00);  // Force CW off (needed to retrigger)
   reg_write(FPGA, 0x23, 0x08);  // Force CW tone is the trigger, Note: QEC corr engine waits to flush loopback so tone is present at Rx
   reg_write(FPGA, 0x25, 0x00); // disable correlation from getting retriggered

  //do while(!reg_read(FPGA, 0x25)){ // readback only nonzero when correlation completed TODO: this has chance of hang if actual value is 0
    delay(20); // 1 million samples is 1/80 of a second
  //}

  reg_write(FPGA, 0x25, 0 << 7); // qec_read_idx
  temp = reg_read(FPGA, 0x25);
  reg_write(FPGA, 0x25, 1 << 7); // qec_read_idx
  *corr_ii = (int32_t)((((uint32_t)reg_read(FPGA, 0x25)) << 16) | temp);

  reg_write(FPGA, 0x25, 2 << 7); // qec_read_idx
  temp = reg_read(FPGA, 0x25);
  reg_write(FPGA, 0x25, 3 << 7); // qec_read_idx
  *corr_iq = (int32_t)((((uint32_t)reg_read(FPGA, 0x25)) << 16) | temp);

  reg_write(FPGA, 0x25, 4 << 7); // qec_read_idx
  temp = reg_read(FPGA, 0x25);
  reg_write(FPGA, 0x25, 5 << 7); // qec_read_idx
  *corr_qi = (int32_t)((((uint32_t)reg_read(FPGA, 0x25)) << 16) | temp);

  reg_write(FPGA, 0x25, 6 << 7); // qec_read_idx
  temp = reg_read(FPGA, 0x25);
  reg_write(FPGA, 0x25, 7 << 7);  // qec_read_idx
  *corr_qq = (int32_t)((((uint32_t)reg_read(FPGA, 0x25)) << 16) | temp);
  
    if ((rxqec0_txqec1==0) && (*corr_qi != *corr_iq) ) {
      Serial.print("\n *** RxQEC CORRELATOR GLITCH ASSERTION FAILED ***\n");
      return;
  }
  return;
}

void apply_rxqec_correction(Stream &FPGA, float gain, float phase){

  int8_t phase_int8, gain_int8;

  if (fabs(phase) > 0.24 || fabs(gain - 1.0) > 0.24 ) {
      Serial.print("\n *** RxQEC Correction out of Bounds ***\n");
      Serial.print(phase); Serial.print(" "); Serial.print(gain); Serial.print("\n");
      return;
  }
  phase_int8 = roundf(phase*256);
  gain_int8  = roundf((gain - 1.0)*256);

  // Arduino does not wrap signed integers
  uint16_t phase_unsigned = (phase_int8 >= 0) ? phase_int8 : ((int16_t)phase_int8 + (int16_t)256);
  uint16_t gain_unsigned =  (gain_int8  >= 0) ? gain_int8  : ((int16_t)gain_int8  + (int16_t)256);

  uint16_t write_val = (phase_unsigned *256) | gain_unsigned;

  reg_write(FPGA, 0x63, write_val);
  delay(10);
  return;
}


void apply_txqec_correction(Stream &FPGA, uint16_t ant_idx, float gain, float phase, float delay, float dc_i, float dc_q)
{  
  gain = gain/cosf(phase) - 1.0;  // gain applied is relative scale added to original signal
  phase = sinf(phase);
  delay = (delay*2.0)/2.0; // delay relative to 0.5 sample, 2x scale because applied to 2x-rate (160 MSPS) sample, half scaled because applied both to I and Q,

    uint16_t write_value;
    
    //Serial.print("TXQEC WRITE: ");

    write_value = ((int16_t)roundf(dc_q * 256.0));
    //Serial.print(write_value, HEX); Serial.print(" ");
    reg_write(FPGA, 0x27, (ant_idx<<8) | (write_value & 0xff));

    write_value = ((int16_t)roundf(dc_i * 256.0));
    //Serial.print(write_value, HEX); Serial.print(" ");
    reg_write(FPGA, 0x27, (ant_idx<<8) | (write_value & 0xff));

    write_value = ((int16_t)roundf(delay * 1024.0));
    //Serial.print(write_value, HEX); Serial.print(" ");
    reg_write(FPGA, 0x27, (ant_idx<<8) | (write_value & 0xff));

    write_value = ((int16_t)roundf(phase * 512.0));
    //Serial.print(write_value, HEX); Serial.print(" ");
    reg_write(FPGA, 0x27, (ant_idx<<8) | (write_value & 0xff));

    write_value = ((int16_t)roundf(gain * 512.0));
    //Serial.print(write_value, HEX); Serial.print(" ");
    reg_write(FPGA, 0x27, (ant_idx<<8) | (write_value & 0xff));

  return;
}
