#define MAX2850_REG_ADDR 0x42

void write_max2850_base_regs(Stream &FPGA) {
  //  5.6 GHz LO:
  // (Apparently) when writing registers, always need to restart at beginning and can end at any point.
  int16_t max2850_main_reg[32] = { 
            /* 0: */  0x00E, /* Tx: CH1=0x02E, CH2=0x04E, CH3=0x8E, CH4=0x10E,   0x1EA (Rx: ) */
            /* 1: */  0x000, /* Rx: min LNA+VGA gain (adjusted by call parameter) */
            /* 2: */  0x1C0, /* Rx: LNA Band 0x1A0=5.3GHz 0x1C0=5.6 GHz*/
            /* 3: */  0x000, /* Temp sensor stuff */
            /* 4: */  0x31C, /* Reserved */
            /* 5: */  0x000, /* RSSI settings default baseband */
            /* 6: */  0x3EB, /* Reserved */
            /* 7: */  0x024, /* Reserved */
            /* 8: */  0x000, /* Reserved */
            /* 9: */  0x00F, /* Tx: less gain Tx, full is 0x3FF, lower 4 bits are chan select */
            /* 10:*/  0x001, /* Reserved, but from eval kit a 1 needed for PA bias out = 2.85V */
            /* 11:*/  0x060, /* Reserved + Tx calibration selection */
            /* 12:*/  0x000, /* (skipped) */
            /* 13:*/  0x000, /* Reserved */
            /* 14:*/  0x160, /* no spi readback, give LOCK status instead */
            /* 15:*/  0x246, /* default synthesizer config 0x242 for 5.35 GHz, 0x245 for 5.52 GHz, 0x246 for 5.6 GHz, 0x23F for 5040 MHz */
            /* 16:*/  0x000, /* default synthesizer config 0x380 for 5.35 GHz */
            /* 17:*/  0x000, /* default synthesizer config */ 
            /* 18:*/  0x080, /* crystal tune, default */
            /* 19:*/  0x05F, /* VAS relock settings, Changed to use current subband (should readback subband and choose correct later) */
            /* 20:*/  0x1EA, /* Reserved */
            /* 21:*/  0x0BF, /* Reserved, die ID */
            /* 22:*/  0x1FE, /* Reserved higher VCO GM bias (helps) */
            /* 23:*/  0x065, /* Reserved (readback from eval and datasheet) */
            /* 24:*/  0x14B, /* Reserved , 0x7 for integer PLL with and +20% CP, 1.6mA current (better), default was 0x24F */
            /* 25:*/  0x3A8, /* Reserved */ 
            /* 26:*/  0x015, /* Reserved */
            /* 27:*/  0x180, /* Reserved, die read */
            /* 28:*/  0x061, /* Reserved + PA Delay, default 1us OK for now, note EvalKit readback was 0x001 */
            /* 29:*/  0x000, /* Reserved BUT (Tx mode) Eval Kit 0x1ee ?? */
            /* 30:*/  0x000, /* Reserved BUT (Tx mode) Eval Kit 0x1ee ?? */
            /* 31:*/  0x000  /* Reserved */
};

  //max2850_main_reg[1] = rx_gain_setting;
  uint16_t readback;
  for (int k = 0; k < 32; k++) {
    if (k == 12) {
      continue;
    }  // skipped in reg-map
    reg_write(FPGA, MAX2850_REG_ADDR, (k << 10) | max2850_main_reg[k]);
    delay(50);
    readback = reg_read(FPGA, MAX2850_REG_ADDR);
  }

  if(readback != 0xffff)
    Serial.print("\n(!!! Tx Failed Lock !!!)\n");

  // Now write local registers
  delay(10);
  reg_write(FPGA, MAX2850_REG_ADDR, (0 << 10) | max2850_main_reg[0] | 0x01);  // set the local register programming bit

  int16_t max2850_local_reg[32] = { /* 0: */ 0x0, 
      /* 1: */ 0x000, 
      /* 2: */ 0x000, 
      /* 3: */ 0x000, 
      /* 4: */ 0x380, 
      /* 5: */ 0x000, 
      /* 6: */ 0x000, 
      /* 7: */ 0x000,
      /* 8: */ 0x1AA,
      /* 9: */ 0x114, 
      /* 10:*/ 0x354, 
      /* 11:*/ 0x073, 
      /* 12:*/ 0x000, 
      /* 13:*/ 0x000, 
      /* 14:*/ 0x000, 
      /* 15:*/ 0x000,
      /* 16:*/ 0x000,
      /* 17:*/ 0x000,
      /* 18:*/ 0x000,
      /* 19:*/ 0x000,
      /* 20:*/ 0x000,
      /* 21:*/ 0x000,
      /* 22:*/ 0x000,
      /* 23:*/ 0x000,
      /* 24:*/ 0x0C4,
      /* 25:*/ 0x12B,
      /* 26:*/ 0x165,
      /* 27:*/ 0x002,
      /* 28:*/ 0x004,
      /* 29:*/ 0x0,
      /* 30:*/ 0x0,
      /* 31:*/ 0x200 /* charge pump trim */};

  for (int k = 0; k < 32; k++) {
    if (k == 0 || k == 29 || k == 30) {
      continue;
    }  // skipped in reg-map
    reg_write(FPGA, MAX2850_REG_ADDR, (k << 10) | max2850_local_reg[k]);
    delay(50);
    readback = reg_read(FPGA, MAX2850_REG_ADDR);
  }

if(readback != 0xffff)
    Serial.print("\n(!!! Tx Failed Lock (local) !!!)\n");
  delay(10);
  reg_write(FPGA, MAX2850_REG_ADDR, (0 << 10) | max2850_main_reg[0] | 0x00); // end with main register programming

  return;
}

// Right board: Ch1 (0x01) and CH4 (0x08) loopback stronger
// Left board: no loopback very good, some make detects go down. Maybe very weak
void set_bb_loopback(Stream &FPGA, uint16_t rx_gain_setting){
    reg_write(FPGA, MAX2850_REG_ADDR, (0 << 10) | 0x01A | (0x01 << 5)); // channel 1, 0x16 is rf, 0x1A is baseband loopback  // 
    
    if(rx_gain_setting)
      reg_write(FPGA, MAX2850_REG_ADDR, (1 << 10) | rx_gain_setting);

    reg_write(FPGA, 0x23, 0);  // Enable sigma delta driving LVDS
  return;
}

// Testing confirms the right most channel is 1 (top of array / electrolytic, labeled 1), left-most is physical 4 (bottom of array)
// Right board: Only Ch1 (0x02E) can stay in Tx - has PA
// Left board: Only Ch3 (0x08E) can stay in Tx - has PA
// CONFIRMs: matched PA needed connected or Tx shutsdown.
void set_Tx(Stream &FPGA, uint16_t tx_gain_setting, uint16_t tx_channels){

    reg_write(FPGA, MAX2850_REG_ADDR, (0 << 10) | (tx_channels << 5) | 0xE);  // Tx: CH1=0x02E, CH2=0x04E, CH3=0x8E, CH4=0x10E
    if(tx_gain_setting)
      reg_write(FPGA, MAX2850_REG_ADDR, (9 << 10) | (tx_gain_setting << 4) | 0x0F);
   
   reg_write(FPGA, 0x23, 0);  // enable delta sigma, no tone!  This also ensured RF Switches are set for Tx.

   reg_write(FPGA, MAX2850_REG_ADDR, (10 << 10) | 0x01); // Enable PA Bias

    reg_write(FPGA, 0x02, tx_channels);  // enable channel mask for baseband

  return;
}