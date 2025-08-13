/*
 * max2850.c
 *
 * driver for max2850 tx chip
 * mirrors the register write sequences from teensy
 */

#include "max2850.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "app_log.h"
#include "main.h"

void write_max2850_base_regs(void) {
  app_log("- configuring max2850 (tx) base registers...\r\n");

  const int16_t max2850_main_reg[32] = {
      /* 0: */  0x00E, /* 1: */  0x000, /* 2: */  0x1C0, /* 3: */  0x000,
      /* 4: */  0x31C, /* 5: */  0x000, /* 6: */  0x3EB, /* 7: */  0x024,
      /* 8: */  0x000, /* 9: */  0x00F, /* 10:*/  0x001, /* 11:*/  0x060,
      /* 12:*/  0x000, /* 13:*/  0x000, /* 14:*/  0x160, /* 15:*/  0x246,
      /* 16:*/  0x000, /* 17:*/  0x000, /* 18:*/  0x080, /* 19:*/  0x05F,
      /* 20:*/  0x1EA, /* 21:*/  0x0BF, /* 22:*/  0x238, /* VCO GM bias */
      /* 23:*/  0x065, /* 24:*/  0x24B, /* PLL + CP settings */
      /* 25:*/  0x3A8, /* 26:*/  0x015, /* 27:*/  0x180, /* 28:*/  0x061,
      /* 29:*/  0x000, /* 30:*/  0x000, /* 31:*/  0x000
  };

  // write main registers
  app_log("info: max2850 writing main regs...\r\n");
  for (int k = 0; k < 32; k++) {
    if (k == 12) continue;
    reg_write(REG_SPI_MAX2850, (k << 10) | max2850_main_reg[k]);
    HAL_Delay(5); // pacing delay from Teensy code
  }

  uint16_t readback = reg_read(REG_SPI_MAX2850);
  if(readback != 0xffff) {
    app_log("warn: max2850 (tx) failed lock after main regs, status=0x%04X\r\n", readback);
  } else {
    app_log("ok: max2850 (tx) pll locked after main regs\r\n");
  }

  // write local regs
  app_log("info: max2850 writing local regs...\r\n");

  // enable local reg programming mode
  reg_write(REG_SPI_MAX2850, (0 << 10) | max2850_main_reg[0] | 0x01);
  HAL_Delay(10);

  const int16_t max2850_local_reg[32] = {
      /* 0: */ 0x0,   /* 1: */ 0x000, /* 2: */ 0x000, /* 3: */ 0x000,
      /* 4: */ 0x380, /* 5: */ 0x000, /* 6: */ 0x000, /* 7: */ 0x000,
      /* 8: */ 0x1AA, /* 9: */ 0x114, /* 10:*/ 0x354, /* 11:*/ 0x073,
      /* 12:*/ 0x000, /* 13:*/ 0x000, /* 14:*/ 0x000, /* 15:*/ 0x000,
      /* 16:*/ 0x000, /* 17:*/ 0x000, /* 18:*/ 0x000, /* 19:*/ 0x000,
      /* 20:*/ 0x000, /* 21:*/ 0x000, /* 22:*/ 0x000, /* 23:*/ 0x000,
      /* 24:*/ 0x0C4, /* 25:*/ 0x12B, /* 26:*/ 0x165, /* 27:*/ 0x002,
      /* 28:*/ 0x004, /* 29:*/ 0x0,   /* 30:*/ 0x0,   /* 31:*/ 0x040
  };

  for (int k = 0; k < 32; k++) {
    if (k == 0 || k == 29 || k == 30) continue;
    reg_write(REG_SPI_MAX2850, (k << 10) | max2850_local_reg[k]);
    HAL_Delay(5);
  }

  // return to main reg programming mode
  reg_write(REG_SPI_MAX2850, (0 << 10) | max2850_main_reg[0] | 0x00);
  HAL_Delay(10);

  app_log("info: max2850 config complete\r\n");
}

// the set_Tx and set_bb_loopback functions remain unchanged
void set_bb_loopback(uint16_t rx_gain_setting) {
    reg_write(REG_SPI_MAX2850, (0 << 10) | 0x01A | (0x01 << 5));
    if(rx_gain_setting) {
      reg_write(REG_SPI_MAX2850, (1 << 10) | rx_gain_setting);
    }
    reg_write(REG_CTRL, (uint16_t)(reg_read(REG_CTRL) & ~0x0002));
}

void set_Tx(uint16_t tx_gain_setting, uint16_t tx_channels) {
    reg_write(REG_SPI_MAX2850, (0 << 10) | (tx_channels << 5) | 0xE);
    if(tx_gain_setting) {
      reg_write(REG_SPI_MAX2850, (9 << 10) | ((tx_gain_setting & 0x1F) << 4) | 0x0F);
    }
    reg_write(REG_SPI_MAX2850, (10 << 10) | 0x01); // Enable PA Bias
    reg_write(REG_CTRL, 0x00);
    reg_write(REG_CHAN_MASK, tx_channels);
}
