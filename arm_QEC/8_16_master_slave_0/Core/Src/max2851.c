/*
 * max2851.c
 *
 * driver for max2851 rx chip
 */

#include "max2851.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "app_log.h"
#include "main.h"
#include "modem.h" // For get_modem_metric

void write_max2851_base_regs(void) {
	app_log("- configuring max2851 (rx) base regs...\r\n");

	const int16_t max2851_main_reg[32] = {
	/* 0: */0x00A, /* Tx: CH1=0x02E, CH2=0x04E, CH3=0x8E, CH4=0x10E,   0x1EA (Rx: ) */
	/* 1: */0x000, /* Rx: min LNA+VGA gain (adjusted by call parameter) */
	/* 2: */0x1C0, /* Rx: LNA Band 0x1A0=5.3GHz 0x1C0=5.6 GHz*/
	/* 3: */0x000, /* Temp sensor stuff */
	/* 4: */0x31C, /* Reserved */
	/* 5: */0x000, /* RSSI settings default baseband */
	/* 6: */0x3FF, /* Rx channel selects - all 5 rx channels */
	/* 7: */0x024, /* Reserved */
	/* 8: */0x000, /* Reserved */
	/* 9: */0x00F, /* Tx: ---, lower 4 bits are chan select */
	/* 10:*/0x000, /* Reserved, but from eval kit a 1 needed for PA bias out = 2.85V */
	/* 11:*/0x060, /* Reserved + Tx calibration selection */
	/* 12:*/0x000, /* (skipped) */
	/* 13:*/0x000, /* Reserved */
	/* 14:*/0x160, /* enable clk2 output, no spi readback, give LOCK status instead */
	/* 15:*/0x246, /* default synthesizer config 0x242 for 5.35 GHz, 0x245 for 5.52 GHz, 0x246 for 5.6 GHz */
	/* 16:*/0x000, /* default synthesizer config 0x380 for 5.35 GHz */
	/* 17:*/0x000, /* default synthesizer config */
	/* 18:*/0x080, /* crystal tune, default */
	/* 19:*/0x0DF, /* VAS relock settings, Changed to use current subband (should readback subband and choose correct later) */
	/* 20:*/0x1EA, /* Reserved */
	/* 21:*/0x0BF, /* Reserved, die ID */
	/* 22:*/0x1FE, /* Reserved higher VCO GM bias (helps) */
	/* 23:*/0x065, /* Reserved (readback from eval and datasheet) */
	/* 24:*/0x14B, /* Reserved , 0x7 for integer PLL with and +20% CP, 1.6mA current (better) */
	/* 25:*/0x3A8, /* Reserved */
	/* 26:*/0x015, /* Reserved */
	/* 27:*/0x180, /* Reserved, die read */
	/* 28:*/0x061, /* Reserved + PA Delay, default 1us OK for now, note EvalKit readback was 0x001 */
	/* 29:*/0x000, /* Reserved BUT (Tx mode) Eval Kit 0x1ee ?? */
	/* 30:*/0x000, /* Reserved BUT (Tx mode) Eval Kit 0x1ee ?? */
	/* 31:*/0x000 /* Reserved */
	};

	app_log("- max2851 writing main registers...\r\n");
	for (int k = 0; k < 32; k++) {
		if (k == 12)
			continue;
		reg_write(REG_SPI_MAX2851, (k << 10) | max2851_main_reg[k]);
		HAL_Delay(10);
	}

	uint16_t readback = reg_read(REG_SPI_MAX2851);
	if (readback != 0xffff) {
		app_log(
				"- max2851 (rx) failed lock after main regs, status=0x%04X\r\n",
				readback);
	}

	app_log("- max2851 writing local regs...\r\n");
	reg_write(REG_SPI_MAX2851, (0 << 10) | max2851_main_reg[0] | 0x01);

	const int16_t max2851_local_reg[32] = {
	/* 1: */0x000, /* 2: */0x000, /* 3: */0x000, /* 4: */0x380, /* 5: */0x000,
	/* 6: */0x000, /* 7: */0x000, /* 8: */0x1AA, /* 9: */0x114, /* 10:*/0x354,
	/* 11:*/0x073, /* 12:*/0x000, /* 13:*/0x000, /* 14:*/0x000, /* 15:*/0x000,
	/* 16:*/0x000, /* 17:*/0x000, /* 18:*/0x000, /* 19:*/0x000, /* 20:*/0x000,
	/* 21:*/0x000, /* 22:*/0x000, /* 23:*/0x000, /* 24:*/0x1C4, /* 25:*/0x12B,
	/* 26:*/0x165, /* 27:*/0x002, /* 28:*/0x004, /* 29:*/0x0,   /* 30:*/0x0,
	/* 31:*/0x200
	};

	for (int k = 0; k < 32; k++) {
		if (k == 0 || k == 29 || k == 30)
			continue;
		reg_write(REG_SPI_MAX2851, (k << 10) | max2851_local_reg[k]);
	}

	reg_write(REG_SPI_MAX2851, (0 << 10) | max2851_main_reg[0] | 0x00);
	app_log("- max2851 config complete\r\n");
}

void set_Rx(int16_t rx_gain_setting) {
	reg_write(REG_SPI_MAX2851, (0 << 10) | 0x00A);
	if (rx_gain_setting) {
		set_Rx_gain(rx_gain_setting);
	}
	reg_write(REG_CTRL, reg_read(REG_CTRL) | CTRL_TX_DISABLE_LVDS);
	reg_write(REG_SPI_MAX2850, (10 << 10) | 0x00);
}

void set_Rx_gain(int16_t rx_gain_setting) {
	reg_write(0x6A, rx_gain_setting);
}

void set_RxIdle(void) {
	reg_write(REG_SPI_MAX2851, (0 << 10) | 0x022);
}

int16_t find_best_rx_vga(void)
{
  app_log("- Finding best RX VGA gain...\r\n");
  int best_gain = 11;
  float best_metric = 0.0;
  float new_metric;
  for (int k=1; k<20; k++){
    int test_gain = k*2;
    set_Rx_gain(test_gain);
    app_log("Gain %d: ", test_gain);
    HAL_Delay(1);
    new_metric = get_modem_metric(0);
    rx_telem();

    if(new_metric > best_metric)
    {
      best_metric = new_metric;
      best_gain = test_gain;
    }
  }
  set_Rx_gain(best_gain);
  app_log("- Best gain found: %d\r\n", best_gain);
  return best_gain;
}

void gain_track(int16_t initial_gain) {
    // Simple hill-climbing algorithm to dynamically track the optimal receiver gain.
    // - It starts at the gain found by find_best_rx_vga.
    // - It "dithers" the gain up and down, keeping any change that improves the checksum count.
    // - If the checksum drops to zero for 3 consecutive reads, it assumes the link is lost
    //   and re-runs the full find_best_rx_vga search to re-acquire the signal.
	// Note: does not work great, but better than keeping one gain.

    int16_t current_gain = initial_gain;
    float current_checksum = 0;
    int zero_count = 0;
    int direction = 1; // 1 for up, -1 for down

    app_log("--- Starting Gain Tracking ---\r\n");
    set_Rx_gain(current_gain);

    while(1) {
        current_checksum = get_modem_metric(1); // Get checksum at current gain

        int16_t next_gain = current_gain + (direction * 2); // Propose next gain
        if (next_gain < 0) next_gain = 0;
        if (next_gain > 38) next_gain = 38;

        set_Rx_gain(next_gain);
        HAL_Delay(50);
        float next_checksum = get_modem_metric(1);

        app_log("Gain: %d -> %d, Checksum: %.0f -> %.0f\r\n", current_gain, next_gain, current_checksum, next_checksum);

        if (next_checksum > current_checksum) {
            // Improvement, keep the new gain and continue in the same direction
            current_gain = next_gain;
        } else {
            // No improvement, revert gain and change direction
            set_Rx_gain(current_gain);
            direction *= -1;
        }

        // Check for lost link
        if (next_checksum == 0) {
            zero_count++;
        } else {
            zero_count = 0;
        }

        if (zero_count >= 3) {
            app_log("--- Link lost, re-running gain search ---\r\n");
            current_gain = find_best_rx_vga();
            set_Rx_gain(current_gain);
            zero_count = 0;
            direction = 1; // Reset direction
        }

        HAL_Delay(200);
    }
}
