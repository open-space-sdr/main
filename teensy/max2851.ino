#define MAX2851_REG_ADDR 0x43


// Rx Gain calculation function

// LNA gain options in dB
const int LNA_GAINS[] = {-40+40, -32+40, -24+40, -16+40, -8+40, 0+40};
const int LNA_VALUES[] = {0b000, 0b001, 0b100, 0b101, 0b110, 0b111};

// VGA gain step in dB
const int VGA_MIN_GAIN = 0;
const int VGA_MAX_GAIN = 30;
const int VGA_STEP = 2;

void calculate_gain_registers(int16_t desired_gain, uint16_t *lna_gain_reg, uint16_t *vga_gain_reg) {
    int best_lna_index = 0;
    int best_vga_gain = VGA_MIN_GAIN;

    int min_vga_distance = abs(desired_gain - (LNA_GAINS[0] + VGA_MIN_GAIN));

    // Iterate through LNA options to find the most centered VGA gain
    for (int i = 0; i < 6; i++) {
        int remaining_gain = desired_gain - LNA_GAINS[i];

        if (remaining_gain < VGA_MIN_GAIN) {
            continue; // Skip invalid LNA selections
        }

        if (remaining_gain > VGA_MAX_GAIN) {
            remaining_gain = VGA_MAX_GAIN;
        }

        int vga_distance = abs(remaining_gain - (VGA_MIN_GAIN + VGA_MAX_GAIN) / 2);

        if (vga_distance < min_vga_distance) {
            min_vga_distance = vga_distance;
            best_lna_index = i;
            best_vga_gain = remaining_gain;
        }
    }

    // Calculate register values
    *lna_gain_reg = LNA_VALUES[best_lna_index];
    *vga_gain_reg = (best_vga_gain / VGA_STEP);
    return;
}



void write_max2851_base_regs(Stream &FPGA) {
  //  5.6 GHz LO:
  // (Apparently) when writing registers, always need to restart at beginning and can end at any point.
  int16_t max2851_main_reg[32] = { 
            /* 0: */  0x00A, /* Tx: CH1=0x02E, CH2=0x04E, CH3=0x8E, CH4=0x10E,   0x1EA (Rx: ) */
            /* 1: */  0x000, /* Rx: min LNA+VGA gain (adjusted by call parameter) */
            /* 2: */  0x1C0, /* Rx: LNA Band 0x180=5.0GHz,  0x1A0=5.3GHz 0x1C0=5.6 GHz*/
            /* 3: */  0x000, /* Temp sensor stuff */
            /* 4: */  0x31C, /* Reserved */
            /* 5: */  0x000, /* RSSI settings default baseband */
            /* 6: */  0x3FF, /* Rx channel selects - all 5 rx channels */
            /* 7: */  0x024, /* Reserved */
            /* 8: */  0x000, /* Reserved */
            /* 9: */  0x00F, /* Tx: ---, lower 4 bits are chan select */
            /* 10:*/  0x000, /* Reserved, but from eval kit a 1 needed for PA bias out = 2.85V */
            /* 11:*/  0x060, /* Reserved + Tx calibration selection */
            /* 12:*/  0x000, /* (skipped) */
            /* 13:*/  0x000, /* Reserved */
            /* 14:*/  0x160, /* enable clk2 output, no spi readback, give LOCK status instead */
            /* 15:*/  0x246, /* default synthesizer config 0x242 for 5.35 GHz, 0x245 for 5.52 GHz, 0x246 for 5.6 GHz, 0x23F for 5040 MHz */
            /* 16:*/  0x000, /* default synthesizer config 0x380 for 5.35 GHz */
            /* 17:*/  0x000, /* default synthesizer config */ 
            /* 18:*/  0x080, /* crystal tune, default */
            /* 19:*/  0x0DF, /* VAS relock settings, Changed to use current subband (should readback subband and choose correct later) */
            /* 20:*/  0x1EA, /* Reserved */
            /* 21:*/  0x0BF, /* Reserved, die ID */
            /* 22:*/  0x1FE, /* Reserved higher VCO GM bias (helps) */
            /* 23:*/  0x065, /* Reserved (readback from eval and datasheet) */
            /* 24:*/  0x14B, /* Reserved , 0x7 for integer PLL with and +20% CP, 1.6mA current (better) */
            /* 25:*/  0x3A8, /* Reserved */ 
            /* 26:*/  0x015, /* Reserved */
            /* 27:*/  0x180, /* Reserved, die read */
            /* 28:*/  0x061, /* Reserved + PA Delay, default 1us OK for now, note EvalKit readback was 0x001 */
            /* 29:*/  0x000, /* Reserved BUT (Tx mode) Eval Kit 0x1ee ?? */
            /* 30:*/  0x000, /* Reserved BUT (Tx mode) Eval Kit 0x1ee ?? */
            /* 31:*/  0x000  /* Reserved */
};

  //max2851_main_reg[1] = rx_gain_setting;
  uint16_t readback;
  for (int k = 0; k < 32; k++) {
    if (k == 12) {
      continue;
    }  // skipped in reg-map
    reg_write(FPGA, MAX2851_REG_ADDR, (k << 10) | max2851_main_reg[k]);
    readback = reg_read(FPGA, MAX2851_REG_ADDR);
  }
  if(readback != 0xffff)
    Serial.print("\n(!!! Rx Failed Lock !!!)\n");

  // Now write local registers
  reg_write(FPGA, MAX2851_REG_ADDR, (0 << 10) | max2851_main_reg[0] | 0x01);  // set the local register programming bit

  int16_t max2851_local_reg[32] = { /* 0: */ 0x0, 
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
      /* 24:*/ 0x1C4,
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
    reg_write(FPGA, MAX2851_REG_ADDR, (k << 10) | max2851_local_reg[k]);
    readback = reg_read(FPGA, MAX2851_REG_ADDR); Serial.print("*");
  }

if(readback != 0xffff)
    Serial.print("\n(!!! Rx Failed Lock (local) !!!)\n");

  reg_write(FPGA, MAX2850_REG_ADDR, (0 << 10) | max2851_main_reg[0] | 0x00); // end with main register programming

  return;
}

int find_best_rx_vga(Stream &FPGA)
{
  //set_Rx_gain(FPGA,  0);
  //get_modem_metric(FPGA, 0); // initial flush
  int best_gain = 11; float best_metric = 0.0; float new_metric;
  for (int k=1;k<20;k++){ // Find best Rx VGA gain values
    int test_gain = k*2;
    set_Rx_gain(FPGA,  test_gain);
    Serial.print(test_gain, DEC);Serial.print(":");
    delay(1); new_metric = get_modem_metric(FPGA, 0);
    rx_telem(FPGA);
    //get_modem_metric(FPGA, 0);

    if(new_metric > best_metric)
    { best_metric = new_metric;
      best_gain = test_gain;
    }
  }
  set_Rx_gain(FPGA,  best_gain);
  return best_gain;
}


void set_Rx(Stream &FPGA, int16_t rx_gain_setting){
    reg_write(FPGA, MAX2851_REG_ADDR, (0 << 10) | 0x00A);
    
    if(rx_gain_setting){
      //reg_write(FPGA, MAX2851_REG_ADDR, (6 << 10) |0x1EF ); // gain channel selects - KEEP even with AGC, already set in initial

      set_Rx_gain(FPGA, rx_gain_setting);
    }

    reg_write(FPGA, 0x23, 0x1);  // Disable sigma delta driving LVDS on Tx (in case of self coupling interference)
    reg_write(FPGA, 0x42, (10 << 10) | 0x00); // Reach over to Tx chip and disable PA Bias

    // Switch RF SW to Rx - this is done with Tx Disable 0x23


  return;
}

void set_Rx_gain(Stream &FPGA, int16_t rx_gain_setting){
      //uint16_t lnaSetting, vgaSetting;
      //calculate_gain_registers(rx_gain_setting, &lnaSetting, &vgaSetting);
      //Serial.print("[");Serial.print(lnaSetting, DEC);Serial.print("/");Serial.print(vgaSetting, DEC);Serial.print("]");

      //reg_write(FPGA, MAX2851_REG_ADDR, (1 << 10) | (lnaSetting << 5) | vgaSetting );
    reg_write(FPGA, 0x6A, rx_gain_setting );
  return;
}

void set_RxIdle(Stream &FPGA) // puts MAX2851 into idle
{
   reg_write(FPGA, MAX2851_REG_ADDR, (0 << 10) | 0x022);

  return;
}


