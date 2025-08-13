/*
 * max2851.c
 *
 * driver for max2851 rx chip
 * mirrors the register write sequences from teensy project
 */

#include "max2851.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "app_log.h"
#include "main.h"

void write_max2851_base_regs(void) {
  app_log("info: configuring max2851 (rx) base regs...\r\n");

  const int16_t max2851_main_reg[32] = {
      /* 0: */  0x02A, /* 1: */  0x000, /* 2: */  0x1C0, /* 3: */  0x000,
      /* 4: */  0x31C, /* 5: */  0x000, /* 6: */  0x1EF, /* 7: */  0x024,
      /* 8: */  0x000, /* 9: */  0x00F, /* 10:*/  0x000, /* 11:*/  0x060,
      /* 12:*/  0x000, /* 13:*/  0x000, /* 14:*/  0x160, /* 15:*/  0x246,
      /* 16:*/  0x000, /* 17:*/  0x000, /* 18:*/  0x080, /* 19:*/  0x0DF,
      /* 20:*/  0x1EA, /* 21:*/  0x0BF, /* 22:*/  0x1F2, /* 23:*/  0x045,
      /* 24:*/  0x203, /* 25:*/  0x3A8, /* 26:*/  0x015, /* 27:*/  0x180,
      /* 28:*/  0x061, /* 29:*/  0x000, /* 30:*/  0x000, /* 31:*/  0x000
  };

  // write main regs
  app_log("info: max2851 writing main registers...\r\n");
  for (int k = 0; k < 32; k++) {
    if (k == 12) continue;
    reg_write(REG_SPI_MAX2851, (k << 10) | max2851_main_reg[k]);
    // no teensy delays here
  }

  uint16_t readback = reg_read(REG_SPI_MAX2851);
  if(readback != 0xffff) {
    app_log("warn: max2851 (rx) failed lock after main regs, status=0x%04X\r\n", readback);
  }

  // write local regs
  app_log("info: max2851 writing local regs...\r\n");

  // enable local reg programming mode
  reg_write(REG_SPI_MAX2851, (0 << 10) | max2851_main_reg[0] | 0x01);

  const int16_t max2851_local_reg[32] = {
      /* 0: */ 0x0,   /* 1: */ 0x000, /* 2: */ 0x000, /* 3: */ 0x000,
      /* 4: */ 0x380, /* 5: */ 0x000, /* 6: */ 0x000, /* 7: */ 0x000,
      /* 8: */ 0x1AA, /* 9: */ 0x114, /* 10:*/ 0x354, /* 11:*/ 0x073,
      /* 12:*/ 0x000, /* 13:*/ 0x000, /* 14:*/ 0x000, /* 15:*/ 0x000,
      /* 16:*/ 0x000, /* 17:*/ 0x000, /* 18:*/ 0x000, /* 19:*/ 0x000,
      /* 20:*/ 0x000, /* 21:*/ 0x000, /* 22:*/ 0x000, /* 23:*/ 0x000,
      /* 24:*/ 0x1C4, /* 25:*/ 0x12B, /* 26:*/ 0x165, /* 27:*/ 0x002,
      /* 28:*/ 0x004, /* 29:*/ 0x0,   /* 30:*/ 0x0,   /* 31:*/ 0x040
  };

  for (int k = 0; k < 32; k++) {
    if (k == 0 || k == 29 || k == 30) continue;
    reg_write(REG_SPI_MAX2851, (k << 10) | max2851_local_reg[k]);
  }

  // return to main rrg programming mode
  reg_write(REG_SPI_MAX2851, (0 << 10) | max2851_main_reg[0] | 0x00);

  app_log("info: max2851 configuration complete.\r\n");
}

// set_Rx, set_Rx_gain, and set_RxIdle functions remain unchanged
void set_Rx(int16_t rx_gain_setting) {
    reg_write(REG_SPI_MAX2851, (0 << 10) | 0x02A);
    if(rx_gain_setting){
      set_Rx_gain(rx_gain_setting);
    }
    reg_write(REG_CTRL, reg_read(REG_CTRL) | CTRL_TX_DISABLE_LVDS);
    reg_write(REG_SPI_MAX2850, (10 << 10) | 0x00); // Disable PA Bias
}

void set_Rx_gain(int16_t rx_gain_setting) {
    reg_write(0x6A, rx_gain_setting);
}

void set_RxIdle(void) {
   reg_write(REG_SPI_MAX2851, (0 << 10) | 0x022);
}
