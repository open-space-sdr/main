#include <SPI.h>

#define FPGA_LEFT Serial1
#define FPGA_RIGHT Serial2

//ADJUST THIS:
#define FPGA_MASTER FPGA_LEFT
#define FPGA_SLAVE FPGA_RIGHT

#define FPGA_TX FPGA_SLAVE
#define FPGA_RX FPGA_MASTER

#define PinSPICS 10
SPISettings settings(10000000, MSBFIRST, SPI_MODE0);

void waitForKeyPress() {
  Serial.println("\nPress key to continue\n");
  while (!Serial.available())
    ;
  while (Serial.available())
    Serial.read();
  Serial.flush();
}

   // a function that wraps -0.5 to 0.5 like signed numbers
  float modhalf(float x){
    return  x - floorf(x + 0.5f);
  }


void setup() {

  // Set all other pins to INPUT
  for (int k = 2; k <= 23; k++)
    pinMode(k, INPUT);


  const int CHANNEL_SLAVE_TX = 4;
  const int CHANNEL_SLAVE_RX = 5;

  const int CHANNEL_MASTER_TX = 4;
  const int CHANNEL_MASTER_RX = 2;




  

  // Output log back to PC
  Serial.begin(115200);
  Serial.print("\n");

  // Basic devices setup
  Serial.print("\nMaster: ");
  FPGA_MASTER.begin(115200);
  Serial.print("[UART] ");
  setup_reg(FPGA_MASTER);
  Serial.print("[FPGA Reg] ");
  write_max2851_base_regs(FPGA_MASTER);
  Serial.print("[MAX2851 Regs] ");
  write_max2850_base_regs(FPGA_MASTER);
  Serial.print("[MAX2850 Regs] ");


  Serial.print("\nSlave:  ");
  FPGA_SLAVE.begin(115200);
  Serial.print("[UART] ");
  setup_reg(FPGA_SLAVE);
  Serial.print("[FPGA Reg] ");

  write_max2851_base_regs(FPGA_SLAVE);
  Serial.print("[MAX2851 Regs] ");

  write_max2850_base_regs(FPGA_SLAVE);
  Serial.print("[MAX2850 Regs] "); 


  reg_write(FPGA_MASTER, 0x23, 0x02);  // mod in reset in case PLL is in weird state
  reg_write(FPGA_MASTER, 0x23, 0x00);
  reg_write(FPGA_MASTER, 0x01, 0x02);                   // TDD disabled, master (0x3 is TDD enabled)
  set_Rx(FPGA_MASTER, 0);                               // master starts in Rx, use existing gain
  reg_write(FPGA_MASTER, 0x65, 0);                      // ensure loopback disabled
  reg_write(FPGA_MASTER, 0x22, CHANNEL_MASTER_RX);  // set Rx MUX
  reg_write(FPGA_MASTER, 0x66, 2);                      // choose to read the rx register that tx increments for frame count
  reg_write(FPGA_MASTER, 0x4D, 1);                      // trigger a capture to clear out
  reg_write(FPGA_MASTER, 0x62, 0);                      // no disable rx sd fb
  reg_write(FPGA_MASTER, 0x51, 90); // RSSI set point
  reg_write(FPGA_MASTER, 0x23, 0x00);  // Force CW off 
  reg_write(FPGA_MASTER, 0x25, 0x00); // disable qec correlation
  reg_write(FPGA_MASTER, 0x64, 32);   // digital rx gain of unity  Necessary now as FPGA loads 0

  

  // Initialize FPGA (move this to common function)
  reg_write(FPGA_SLAVE, 0x23, 0x02);  // mod in reset in case PLL is in weird state
  reg_write(FPGA_SLAVE, 0x23, 0x00);
  reg_write(FPGA_SLAVE, 0x01, 0x00);                  // TDD disabled, slave (0x01 is TDD enabled)
  set_Rx(FPGA_SLAVE, 0);                              // slave in Rx, use existing gain
  reg_write(FPGA_SLAVE, 0x65, 0);                     // ensure loopback disabled
  reg_write(FPGA_SLAVE, 0x22, CHANNEL_SLAVE_RX);  // set Rx MUX
  reg_write(FPGA_SLAVE, 0x66, 2);                     // choose to read the rx register that tx increments for frame count
  reg_write(FPGA_SLAVE, 0x4D, 1);                     // trigger a capture to clear out
  reg_write(FPGA_SLAVE, 0x62, 0);                     // no disable rx sd fb
  reg_write(FPGA_SLAVE, 0x51, 90); // RSSI set point
  reg_write(FPGA_SLAVE, 0x23, 0x00);  // Force CW off 
  reg_write(FPGA_SLAVE, 0x25, 0x00); // disable qec correlation
 reg_write(FPGA_SLAVE, 0x64, 32);   // digital rx gain of unity  Necessary now as FPGA loads 0
   // Apparently new SDM filters aren't working with QPSK
  

//set_Tx(FPGA_MASTER, 0x03 /* PA Gain */, 1 << (CHANNEL_MASTER_TX-1)/* Channel mask */);
//set_Tx(FPGA_SLAVE, 0x03 /* PA Gain */, 1 << (CHANNEL_SLAVE_TX-1)/* Channel mask */);

//QEC(FPGA_SLAVE, 3);
//rx_debug_capture(FPGA_SLAVE);
//QEC(FPGA_MASTER, 3);

  //
  //waitForKeyPress();


//goto skip_master;
int RX_GAIN_MASTER;
//for(int CHAN_SLAVE = 1;CHAN_SLAVE<=4; CHAN_SLAVE++)
//{
  //for(int CHAN_MASTER = 1;CHAN_MASTER<=4; CHAN_MASTER++)
  //{
      //Serial.print("\n---\n ");Serial.print(CHAN_SLAVE,DEC);Serial.print("-->");Serial.print(CHAN_MASTER,DEC);Serial.print("\n---");

    reg_write(FPGA_MASTER, 0x22, CHANNEL_MASTER_RX);

  set_Rx(FPGA_SLAVE, 0);
  set_Tx(FPGA_SLAVE, 0x10 /* PA Gain */, 1 << (CHANNEL_SLAVE_TX - 1) /* Channel mask */);
  
  //while(1){ ///////////////////
  set_Rx(FPGA_MASTER, 0);
  RX_GAIN_MASTER = find_best_rx_vga(FPGA_MASTER);
  Serial.print("Best (");
  Serial.print(RX_GAIN_MASTER, DEC);
  Serial.print(")\n");
  set_Rx_gain(FPGA_MASTER, RX_GAIN_MASTER);


waitForKeyPress();

//}
//}

Serial.print("Self-cal");
waitForKeyPress();
  set_Rx(FPGA_SLAVE, 0); // disable interference
set_Tx(FPGA_MASTER, 0x1F /* PA Gain */, 1 << (CHANNEL_MASTER_TX - 1)); 
 QEC(FPGA_MASTER);
 reg_write(FPGA_MASTER, 0x22, CHANNEL_MASTER_RX);  // set Rx MUX back

 set_Rx(FPGA_MASTER, RX_GAIN_MASTER); // now actual Rx turns off PA
  //}
Serial.print("Back for link test");
set_Tx(FPGA_SLAVE, 0x10 /* PA Gain */, 1 << (CHANNEL_SLAVE_TX - 1));
waitForKeyPress();
  for (int spit = 0; spit < 20; spit++) {
    rx_telem(FPGA_MASTER);
    get_modem_metric(FPGA_MASTER, 0);
  }

// Measure spectral tilt
//calibrate_spectral_tilt(FPGA_MASTER);
waitForKeyPress();  /////////////////////////////////////////
skip_master:

  // Find best Rx VGA gains
  set_Tx(FPGA_MASTER, 0x10 /* PA Gain */, 1 << (CHANNEL_MASTER_TX - 1));
  int RX_GAIN_SLAVE;
  ///////////////////
  set_Rx(FPGA_SLAVE, 0);
  RX_GAIN_SLAVE = find_best_rx_vga(FPGA_SLAVE);
  Serial.print("Best (");
  Serial.print(RX_GAIN_SLAVE, DEC);
  Serial.print(")\n");
  set_Rx(FPGA_SLAVE, RX_GAIN_SLAVE);

Serial.print("Self-cal");
  set_Rx(FPGA_MASTER, 0); // disable interference
waitForKeyPress();
set_Tx(FPGA_SLAVE, 0x1F /* PA Gain */, 1 << (CHANNEL_SLAVE_TX - 1)); 
 QEC(FPGA_SLAVE);
 reg_write(FPGA_SLAVE, 0x22, CHANNEL_SLAVE_RX);  // set Rx MUX back
set_Tx(FPGA_SLAVE, 0x1F /* PA Gain */, 1 << (CHANNEL_SLAVE_TX - 1)); // put back
 set_Rx(FPGA_SLAVE, RX_GAIN_SLAVE); // now actual Rx turns off PA
  //}
Serial.print("Back for link test");
set_Tx(FPGA_MASTER, 0x10 /* PA Gain */, 1 << (CHANNEL_MASTER_TX - 1));
waitForKeyPress();
  for (int spit = 0; spit < 20; spit++) {
    rx_telem(FPGA_SLAVE);
    get_modem_metric(FPGA_SLAVE, 0);
  }

  //sdm_capture(FPGA_SLAVE);

  //rx_debug_capture(FPGA_SLAVE);
//sdm_capture(FPGA_SLAVE);
//calibrate_spectral_tilt(FPGA_SLAVE);
waitForKeyPress();  /////////////////////////////////////////


  //sdm_capture(FPGA_SLAVE);
  //waitForKeyPress();
  //
  //waitForKeyPress();
  //rx_debug_capture(FPGA_RX);
  //waitForKeyPress();

  /* const int N_LUT = 192;
  int8_t CICLUT[N_LUT] = {-11, 9, -57, -57, 9, -11, -11, 9, -57, -57, 9, -11, -13, 5, -23, -35, 5, 3, -13, 5, -23, -35, 5, 3, -9, 5, -25, -31, 3, -3, -9, 5, -25, -31, 3, -3, -11, 1, 9, -9, -1, 11, -11, 1, 9, -9, -1, 11, -3, 3, -31, -25, 5, -9, -3, 3, -31, -25, 5, -9, -5, -1, 3, -3, 1, 5, -5, -1, 3, -3, 1, 5, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, -3, -5, 35, 23, -5, 13, -3, -5, 35, 23, -5, 13, 3, 5, -35, -23, 5, -13, 3, 5, -35, -23, 5, -13, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 5, 1, -3, 3, -1, -5, 5, 1, -3, 3, -1, -5, 3, -3, 31, 25, -5, 9, 3, -3, 31, 25, -5, 9, 11, -1, -9, 9, 1, -11, 11, -1, -9, 9, 1, -11, 9, -5, 25, 31, -3, 3, 9, -5, 25, 31, -3, 3, 13, -5, 23, 35, -5, -3, 13, -5, 23, 35, -5, -3, 11, -9, 57, 57, -9, 11, 11, -9, 57, 57, -9, 11};

  spsa_adapt_CIC(FPGA_MASTER, 1, 50, CICLUT, N_LUT);
waitForKeyPress();
  spsa_adapt_CIC(FPGA_SLAVE, 1, 50, CICLUT, N_LUT);
waitForKeyPress(); */

  //loopback_timing_test(FPGA_MASTER, 5);
  //loopback_timing_test(FPGA_SLAVE, 5);

  //waitForKeyPress();





  /* ============TDD =============== */
  Serial.print("\n TDD...");
  waitForKeyPress();

  set_Rx(FPGA_MASTER, (1 << 15) | RX_GAIN_MASTER);  // enable AGC 
  set_Rx(FPGA_SLAVE, (1 << 15) | RX_GAIN_SLAVE);

  //set_bb_loopback(FPGA_RIGHT, 0x07);

  reg_write(FPGA_MASTER, 0x23, 0x0);  // no digital tx disable
  reg_write(FPGA_SLAVE, 0x23, 0x0);
  reg_write(FPGA_MASTER, 0x62, 0);  //no disable rx sd fb
  reg_write(FPGA_SLAVE, 0x62, 0);   //no disable rx sd fb

  reg_write(FPGA_MASTER, 0x65, 0x00);  // take out of digital loopback
  reg_write(FPGA_SLAVE, 0x65, 0x00);   // take out of digital loopback

  reg_write(FPGA_SLAVE, 0x01, 0x01);   // TDD enabled
  reg_write(FPGA_MASTER, 0x01, 0x03);  // TDD enabled

  // matlab version
  int cnt = 0;
  uint16_t rssi_in;


  


  while (1) {
    reg_write(FPGA_MASTER, 0x50, 1);  // Arm Timing Capture
    // wait until finished capture
    while (reg_read(FPGA_MASTER, 0x50)) { delay(1); };  //wait

delay(2);

    int16_t phase_rx = reg_read(FPGA_MASTER,0x56);
    float phase_rx_float = phase_rx / 65536.0 * 360.0;
    Serial.print(phase_rx_float, 2);Serial.print(",");

   //int16_t phase_rx_msg = reg_read(FPGA_MASTER,0x53);
    //float phase_rx_msg_float = phase_rx_msg / 65536.0 * 360.0;
    //Serial.print(phase_rx_msg_float, 2);Serial.print(" ");

    int32_t SAMP_RANGE = reg_read(FPGA_MASTER, 0x5B) << 16 | reg_read(FPGA_MASTER, 0x5A);
    Serial.print(SAMP_RANGE / 65536.0 - 4065.0, 3); Serial.print(",");
 
  // read tone correlations

   
   for(int k_ant = 1; k_ant<5; k_ant++)
   {

    /*
      //specify read register 
      reg_write(FPGA_MASTER, 0x04, (k_ant << 2) | 0x00); delay(1);
      int16_t corr_ii = reg_read(FPGA_MASTER, 0x04); 
      reg_write(FPGA_MASTER, 0x04, (k_ant << 2) | 0x01); delay(1);
      int16_t corr_iq = reg_read(FPGA_MASTER, 0x04); 
      reg_write(FPGA_MASTER, 0x04, (k_ant << 2) | 0x02); delay(1);
      int16_t corr_qi = reg_read(FPGA_MASTER, 0x04); 
      reg_write(FPGA_MASTER, 0x04, (k_ant << 2) | 0x03); delay(1);
      int16_t corr_qq = reg_read(FPGA_MASTER, 0x04); 

      // rx * conj(tone)
      // real: i*i + q*q
      // imag: -i*q + q*i

      float rx_hightone_corr_real = (float)corr_ii + (float)corr_qq;
      float rx_hightone_corr_imag = -(float)corr_iq + (float)corr_qi;
      float hightone_angle = atan2(rx_hightone_corr_imag, rx_hightone_corr_real)/(2.0*3.14159);

      float rx_lowtone_corr_real = (float)corr_ii - (float)corr_qq;
      float rx_lowtone_corr_imag = (float)corr_iq + (float)corr_qi;
      float lowtone_angle = atan2(rx_lowtone_corr_imag, rx_lowtone_corr_real)/(2.0*3.14159); // -0.5 to 0.5 range

      //Serial.print((fmod(hightone_angle+lowtone_angle + 2*3.14159, 4*3.14159 )/2 - 3.14159)/3.14159 * 180.0, 1); Serial.print(" ");
      */

      reg_write(FPGA_MASTER, 0x04, (k_ant << 2) | 0x01); delay(1);
      int16_t phase_mid_avg = reg_read(FPGA_MASTER, 0x04); 

 /*
      reg_write(FPGA_MASTER, 0x04, (k_ant << 2) | 0x02); delay(1);
      int16_t phase_low_avg = reg_read(FPGA_MASTER, 0x04); 
      reg_write(FPGA_MASTER, 0x04, (k_ant << 2) | 0x03); delay(1);
      int16_t phase_high_avg = reg_read(FPGA_MASTER, 0x04); 

      float lowtone_angle = (float)phase_low_avg / 65536.0; // -0.5 to 0.5 range
      float hightone_angle = (float)phase_high_avg / 65536.0; // -0.5 to 0.5 ranges
      */

    //Serial.print( lowtone_angle, 2); Serial.print("/"); Serial.print( hightone_angle, 2);

      // the right way to average two tones is to take their difference and move half way in that direction. 
      // this generalizes to the overflow exponential-averager
      //Serial.print( modhalf(0.5 + (lowtone_angle + modhalf(hightone_angle-lowtone_angle)/2.0)) * 360.0 , 1);

    Serial.print( (float)phase_mid_avg/ 65536.0 * 360.0 , 1);

      //Serial.print( modhalf(hightone_angle-lowtone_angle) * 360.0 , 1);

      // ok so sum gives bulk phase, but need to subtract from mod phase?

      //Serial.print(sqrtf(rx_lowtone_corr_real*rx_lowtone_corr_real + rx_lowtone_corr_imag*rx_lowtone_corr_imag), 1);Serial.print("/");
      //Serial.print(sqrtf(rx_hightone_corr_real*rx_hightone_corr_real + rx_hightone_corr_imag*rx_hightone_corr_imag), 1);
      


      Serial.print(",");

   } 




//reg_write(FPGA_SLAVE, 0x50, 1);  // Arm Timing Capture
 //   // wait until finished capture
  //  while (reg_read(FPGA_SLAVE, 0x50)) { delay(1); };  //wait

/*
     uint32_t tx_period_est = reg_read(FPGA_MASTER, 0x5D) << 16 | reg_read(FPGA_MASTER, 0x5C);
    Serial.print(tx_period_est, DEC); Serial.print(" ");
    Serial.print(((float)tx_period_est - 3758306099.0) / 3758306099.0 * 1.0e9, 3);Serial.print(" ");


   uint32_t tx_period_pll = reg_read(FPGA_MASTER, 0x5F) << 16 | reg_read(FPGA_MASTER, 0x5E);
    Serial.print(tx_period_pll, DEC);Serial.print(" ");
    Serial.print(((float)tx_period_pll - 3758306099.0) / 3758306099.0 * 1.0e9, 3);Serial.print(" ");
*/
    Serial.print("\n");



    //int32_t cw_delay = reg_read(FPGA_MASTER,0x58) << 16 | reg_read(FPGA_MASTER,0x57);
    //Serial.print(cw_delay / 65536.0, 4);Serial.print(" ");

    /*if (!(++cnt % 5)){
      Serial.print("\n");
      if(cnt>100){
      //Serial.print("\n");rx_debug_capture(FPGA_SLAVE); Serial.print("\n");
      }
    }*/
    /*
    if (!(cnt % 2000)){
      set_Rx_gain(FPGA_MASTER, (1 << 15));  // enable AGC
      set_Rx_gain(FPGA_SLAVE, (1 << 15));
      delay(2000);
      set_Rx_gain(FPGA_MASTER, (0 << 15));  // disable AGC
      set_Rx_gain(FPGA_SLAVE, (0 << 15));
    } */
  }

  float RX_GAIN_SLAVE_float = RX_GAIN_SLAVE;
  float RX_GAIN_MASTER_float = RX_GAIN_MASTER;

  float FPGA_value;
  while (1) {


    reg_write(FPGA_MASTER, 0x50, 1);  // Arm Timing Capture
    // wait until finished capture
    while (reg_read(FPGA_MASTER, 0x50)) { delay(1); };  //wait


    int32_t TX_ERR_EST_FPGA = reg_read(FPGA_MASTER, 0x5B) << 16 | reg_read(FPGA_MASTER, 0x5A);

    FPGA_value = TX_ERR_EST_FPGA / 65536.0;
    Serial.print("master_tx_would_adjust:");
    Serial.print(FPGA_value, 5);


    int16_t phase_adjust = reg_read(FPGA_MASTER, 0x59);
    float phase_adjust_float = phase_adjust / 1024.0 * 360.0;
    //Serial.print(",phase_adjust:"); // failed receive, using tx to update rx
    //Serial.print(phase_adjust_float, 2);


    int16_t phase_rx = reg_read(FPGA_MASTER, 0x56);
    float phase_rx_float = phase_rx / 65536.0 * 360.0;

    Serial.print(",phase_rx:");
    Serial.print(phase_rx_float, 2);


    int16_t phase_rx_self = reg_read(FPGA_MASTER, 0x53);
    float phase_rx_self_float = phase_rx_self / 65536.0 * 360.0;
    Serial.print(",phase_rx_self:");
    Serial.print(phase_rx_self_float, 2);





    reg_write(FPGA_SLAVE, 0x50, 1);  // Arm Timing Capture
    // wait until finished capture
    while (reg_read(FPGA_SLAVE, 0x50)) { delay(1); };  //wait

    phase_rx = reg_read(FPGA_SLAVE, 0x56);
    phase_rx_float = phase_rx / 65536.0 * 360.0;

    Serial.print(",slave_phase_rx:");
    Serial.print(phase_rx_float, 2);


    phase_rx_self = reg_read(FPGA_SLAVE, 0x53);
    phase_rx_self_float = phase_rx_self / 65536.0 * 360.0;
    Serial.print(",slave_phase_rx_self:");
    Serial.print(phase_rx_self_float, 2);

    /*
uint16_t rssi_in;

uint16_t rssi_in_prev_s;
rssi_in = reg_read(FPGA_SLAVE,0x4E);
Serial.print(",rssin_s:"); Serial.print(rssi_in, 1);


uint16_t rssi_in_prev_m;
rssi_in = reg_read(FPGA_MASTER,0x4E);
Serial.print(",rssin_m:"); Serial.print(rssi_in, 1);


uint16_t rx_gain_value;
rx_gain_value = reg_read(FPGA_SLAVE, 0x6A );
Serial.print(",rxgain_s:"); Serial.print(rx_gain_value, 1);

rx_gain_value = reg_read(FPGA_MASTER, 0x6A );
Serial.print(",rxgain_m:"); Serial.print(rx_gain_value, 1);
*/

    Serial.print("\n");
    continue;



    TX_ERR_EST_FPGA = reg_read(FPGA_SLAVE, 0x5B) << 16 | reg_read(FPGA_SLAVE, 0x5A);

    FPGA_value = TX_ERR_EST_FPGA / 65536.0;
    Serial.print(",slave_tx_adjust:");
    Serial.print(FPGA_value, 5);

    phase_adjust = reg_read(FPGA_SLAVE, 0x59);
    phase_adjust_float = phase_adjust / 1024.0 * 360.0;

    Serial.print(",slave_phase_adjust:");  // failed receive, using tx to update rx
    Serial.print(phase_adjust_float, 2);





    uint32_t rx_period_out_capture = reg_read(FPGA_SLAVE, 0x58) << 16 | reg_read(FPGA_SLAVE, 0x57);
    uint32_t tx_period_est_int64 = (int64_t)-67108864 - (((rx_period_out_capture) >> 1) + ((rx_period_out_capture) >> 2) + ((rx_period_out_capture) >> 6));
    float ppm_err = (tx_period_est_int64 - 3758096384) / 3758.0;  // assuming true Tx QPSK is default 0.875037500

    uint32_t tx_period_est_fpga_int64 = reg_read(FPGA_SLAVE, 0x5D) << 16 | reg_read(FPGA_SLAVE, 0x5C);

    float ppm_err_fpga = (tx_period_est_fpga_int64 - 3758096384) / 3758.0;  // assuming true Tx QPSK is default 0.875037500

    Serial.print(",ppm_ard:");
    Serial.print(ppm_err, 5);
    Serial.print(",detect_est_ppm_fpga:");  // valid receives, updates slave tx
    Serial.print(ppm_err_fpga, 5);

    ////////////////

    /*uint32_t tx_period_out_capture = reg_read(FPGA_SLAVE,0x5F) << 16 | reg_read(FPGA_SLAVE,0x5E);
int64_t rx_period_est_int64 =  (int64_t)9817068105 -  (( tx_period_out_capture * 5609753203 ) >> 32);

 float ppm_err2 = ((rx_period_est_int64%4294967296) - 613356400)/4294.0; // assuming true Tx QPSK is default 0.875037500
  Serial.print(",ppm_ard2:");
  Serial.print(ppm_err2, 5);*/

    int32_t rx_period_est_fpga_int64 = reg_read(FPGA_SLAVE, 0x52) << 16 | reg_read(FPGA_SLAVE, 0x51);

    float ppm_err2_fpga = (rx_period_est_fpga_int64 - 613566756) / 4294.0;  // relative to 7.0003/8  which is 300ppm from 7/8
    Serial.print(",failed_ppm_fpga:");                                      // failed receive, using tx to update rx
    Serial.print(ppm_err2_fpga, 5);




    /*
  int16_t phase_deltaint = reg_read(FPGA_SLAVE,0x59);
  float phase_delta = phase_deltaint / 1024.0 * 360.0;
  Serial.print(",phase_delta:"); // failed receive, using tx to update rx
  Serial.print(phase_delta, 5); */


    /*
int16_t phase_adjust = reg_read(FPGA_SLAVE,0x59);
  float phase_adjust_float = phase_adjust / 1024.0 * 360.0;
  Serial.print(",phase_adjust:"); // failed receive, using tx to update rx
  Serial.print(phase_adjust_float, 5);

int16_t phase_rx = reg_read(FPGA_SLAVE,0x56);
  float phase_rx_float = phase_rx / 1024.0 * 360.0;
  Serial.print(",phase_rx:"); // failed receive, using tx to update rx
  Serial.print(phase_rx_float, 5); */



    Serial.print("\n");
  }







  //delay(100);
  //rx_debug_capture();

  //sdm_capture();

  //write_CICLUTS(CICLUT);
  //delay(100);
  //rx_debug_capture();
  //delay(100);
  //rx_debug_capture();
  //waitForKeyPress();
  //sdm_capture();
  //waitForKeyPress();
  //rx_debug_capture();
  //waitForKeyPress();
  //delay(100);






  waitForKeyPress();

  Serial.println("\nDone. Now looping.");
}

void loop() {

  while (1) {
    //Serial.println(reg_read(0x55), DEC); //read back phase comparison
    delay(50);
  }
}
