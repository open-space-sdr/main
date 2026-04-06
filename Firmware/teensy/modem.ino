
void tx_telem(Stream &FPGA)
{


}

void rx_telem(Stream &FPGA)
{
/////// Telemetry indicies //////////
//1: {dc_i[7:0], dc_q[7:0]}
//2: rssi_d_in[8:0]
//3: rssi_resampled[8:0]
//4: leaky_avg_evm[8:0]
//5: rx_period[31:16]
//6: adjust_needed_count[4:0]

reg_write(FPGA,0x4F, 1);
uint16_t dc_iq = reg_read(FPGA,0x4F);
float dc_i = (float)(dc_iq>>8);
float dc_q = (float)(dc_iq % 256);
dc_i = dc_i > 127 ? dc_i - 256.0 : dc_i;
dc_q = dc_q > 127 ? dc_q - 256.0 : dc_q;
Serial.print("dc_i:"); Serial.print(dc_i, 1); Serial.print(",");
Serial.print("dc_q:"); Serial.print(dc_q, 1); Serial.print(",");
float dc_mag = sqrtf(dc_i*dc_i + dc_q*dc_q);
Serial.print("dc:"); Serial.print(dc_mag, 1); Serial.print(",");
  
reg_write(FPGA,0x4F, 2);
uint16_t rssi_in = reg_read(FPGA,0x4F);
Serial.print("rssin:"); Serial.print(rssi_in, 1); Serial.print(",");

reg_write(FPGA,0x4F, 3);
uint16_t rssi_res = reg_read(FPGA,0x4F);
Serial.print("rssres:"); Serial.print(rssi_res, 1); Serial.print(",");

reg_write(FPGA,0x4F, 4);
uint16_t evm = reg_read(FPGA,0x4F);
Serial.print("evm:"); Serial.print(evm, 1); Serial.print(",");

reg_write(FPGA,0x4F, 5);
uint16_t rxp = reg_read(FPGA,0x4F);
Serial.print("rxp:"); Serial.print(rxp, 1); Serial.print(",");

reg_write(FPGA,0x4F, 6);
uint16_t adj = reg_read(FPGA,0x4F);
Serial.print("adj:"); Serial.print(adj, 1); //Serial.print("\n");

return;
}


void loopback_timing_test(Stream &FPGA, int num_iters)
{

set_bb_loopback(FPGA, 0x05 /*loopback gain*/); 
//reg_write(FPGA, 0x65, 0x02); // 0x01 is loopback on PCM data (0x02 for sigma delta loopback)

delay(100); get_modem_metric(FPGA, 1); get_modem_metric(FPGA, 1);  // throw away
 get_modem_metric(FPGA, 0);
 delay(100); get_modem_metric(FPGA, 1); get_modem_metric(FPGA, 1);  // throw away
 get_modem_metric(FPGA, 0);
 delay(100); get_modem_metric(FPGA, 1); get_modem_metric(FPGA, 1);  // throw away
 get_modem_metric(FPGA, 0);


for(int iters=0;iters<num_iters;iters++){
  reg_write(FPGA, 0x50, 1);  // Arm Timing Capture
  // wait until finished capture
  while(reg_read(FPGA,0x50))
  {Serial.print("");delay(10);}; //wait
  // read back timing registers
  uint32_t tx_time_out_capture = reg_read(FPGA,0x54) << 16 | reg_read(FPGA,0x53);
  uint16_t rx_symbol_idx_capture = reg_read(FPGA,0x55);
  uint16_t rx_frac_out_capture = reg_read(FPGA,0x56);
  uint32_t rx_period_out_capture = reg_read(FPGA,0x58) << 16 | reg_read(FPGA,0x57);
  uint32_t phase_out_capture = reg_read(FPGA,0x59);

  int32_t TX_ERR_EST_FPGA = reg_read(FPGA,0x5B) << 16 | reg_read(FPGA,0x5A);
  uint32_t tx_period_est_capture = reg_read(FPGA,0x5D) << 16 | reg_read(FPGA,0x5C);

  // calculate tx_period_est
  //int64_t tx_period_est_int64 =  (int64_t)3758096384 - ((((int64_t)rx_period_out_capture - (int64_t)613566757) * (int64_t)3288334336) >> 32);
  //int64_t tx_period_est_int64 =  (int64_t)4227858432 -  (((int64_t)rx_period_out_capture * (int64_t)3288334336 /* 49/64*2^32 */) >> 32);

  uint32_t tx_period_est_int64 =  (int64_t)-67108864 -  (((rx_period_out_capture)>>1) + ((rx_period_out_capture)>>2) + ((rx_period_out_capture)>>6));

  float tx_period_est = (tx_period_est_int64/65536) / 65536.0; // two step divide (integer then float) to get better precision
  //float ppm_err = (tx_period_est/4294967296.0 - 0.875037500)/(0.875037500) * 1000000.0;
  float ppm_err = (tx_period_est_int64 - 3758257445)/3758.0; // assuming true Tx QPSK is default 0.875037500

  /*
  Serial.print(tx_time_out_capture);Serial.print("\n");
  Serial.print(rx_symbol_idx_capture);Serial.print("\n");
  Serial.print(rx_frac_out_capture);Serial.print("\n");
  Serial.print(rx_period_out_capture);Serial.print("\n");
  Serial.print(phase_out_capture);Serial.print("\n");
  Serial.print("\n");*/

  int64_t TOTAL_DELAY_SAMP = (int64_t)((32.0  +  11  /* + 0.0300 + 3.215 REMOVE FOR MATCHING */) * 65536.0); // + 0.0300 for sigma delta,  + 3.215 for analog loopback
  int64_t symbol_time_code_based_int64 =  (TOTAL_DELAY_SAMP - rx_frac_out_capture) * tx_period_est_int64 / (int64_t)4294967296;
  float symbol_time_code_based = symbol_time_code_based_int64/65536.0 - 10.0 - 2.0; //adjustment for 2038 vs 2048 and the -2.0 fudge


  float tx_time_mod2048 = (tx_time_out_capture % (2048*65536)) / 65536.0;
  //Serial.print(tx_time_mod2048, 6);Serial.print("\n");
  //Serial.print(symbol_time_code_based, 6);Serial.print("\n");


  float SIGMA_DELTA_TX_DELAY_SAMPLES = 0.02607;

  float ardunio_value = (tx_time_mod2048 - symbol_time_code_based);
  float FPGA_value = TX_ERR_EST_FPGA / 65536.0;
  Serial.print("\n ardunio_value:");
  Serial.print(ardunio_value/ tx_period_est - SIGMA_DELTA_TX_DELAY_SAMPLES, 5);
  Serial.print("\n FPGA_value:");
  Serial.print(FPGA_value/ tx_period_est  - SIGMA_DELTA_TX_DELAY_SAMPLES, 5);
  Serial.print("\n");

  float sample_delay_measured = (symbol_time_code_based-tx_time_mod2048)/ tx_period_est;
  Serial.print("samperror:");Serial.print(sample_delay_measured, 5);Serial.print(",ppm:");
  Serial.print(ppm_err, 3); Serial.print("\n");
}
  return;
}


void calibrate_spectral_tilt(Stream &FPGA)
{

  int8_t adjust_try;

  while(1){


    float pow_low_cw = 0.0;
    float pow_high_cw = 0.0;

    reg_write(FPGA,0x28, adjust_try);

    adjust_try++;

    for(int k=0;k<1000;k++){ // take a bunch of captures
        reg_write(FPGA, 0x50, 1);  // Arm Timing Capture
      // wait until finished capture
      while(reg_read(FPGA,0x50))
      {Serial.print("");delay(10);}; //wait

      int16_t cw_sum_low_i = (int16_t)reg_read(FPGA,0x51);
      int16_t cw_sum_low_q = (int16_t)reg_read(FPGA,0x52);
  
      int16_t cw_sum_high_i = (int16_t)reg_read(FPGA,0x5E);
      int16_t cw_sum_high_q = (int16_t)reg_read(FPGA,0x5F);

      /*Serial.print("\n");
      Serial.print((float)(cw_sum_low_i/1000.0)); Serial.print(" ");
      Serial.print((float)(cw_sum_low_q/1000.0)); Serial.print(" ");
      Serial.print((float)(cw_sum_high_i/1000.0)); Serial.print(" ");
      Serial.print((float)(cw_sum_high_q/1000.0)); Serial.print(" "); */

      pow_low_cw += ((float)cw_sum_low_i * (float)cw_sum_low_i    +  (float)cw_sum_low_q * (float)cw_sum_low_q);
      pow_high_cw += ((float)cw_sum_high_i * (float)cw_sum_high_i  +  (float)cw_sum_high_q * (float)cw_sum_high_q);
    }
 
      Serial.print("\n");
      Serial.print((float)(pow_high_cw / pow_low_cw)); Serial.print(" ");
      //Serial.print((float)(pow_high_cw/1000.0)); Serial.print(" ");

  }


  return;
}
