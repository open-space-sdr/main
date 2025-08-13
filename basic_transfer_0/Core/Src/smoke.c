/*
 * smoke.c
 *
 * test-only functions
 */

#include "smoke.h"
#include "app_log.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "modem_metrics.h"
#include "rx_capture.h"
#include "qec.h"
#include "qec_math.h"
#include "telemetry.h"
#include "main.h"

// not working...
///**
// * @brief prepares the fpga modem for metrics and loopback tests.
// * @brief this is a port of the init_fpga_settings logic from teensy.
// * @param rx_channel the receiver channel to use (1-4).
// * @return void.
// * // test: smoke_fpga_init_modem(1);
// */
//void smoke_fpga_init_modem(uint8_t rx_channel)
//{
//    // This is a port of init_fpga_settings from basic_rf.ino [cite: 167-175]
//    if (rx_channel == 0 || rx_channel > 4) rx_channel = 1;
//
//    reg_write(REG_CTRL, 0x02); // modem in reset
//    reg_write(REG_CTRL, 0x00); // modem release
//
//    reg_write(REG_ROLE_MODE, 0x02); // tdd disabled, primary role
//    reg_write(0x65, 0);             // ensure loopback disabled
//    reg_write(REG_RX_MUX, rx_channel);
//    reg_write(REG_FRAME_SRC, 2);    // choose rx register for frame count
//    reg_write(REG_CAP_ARM, 1);      // trigger a capture to clear out
//    reg_write(REG_RX_SD_FEEDBACK, 0); // no disable rx sd fb
//    reg_write(REG_RSSI_SETPOINT, 90); // rssi set point
//    reg_write(REG_CTRL, 0x00);      // ensure cw tone is off
//    reg_write(REG_QEC_EN, 0x00);    // disable qec correlation
//    reg_write(0x64, 32);            // digital rx gain of unity
//    app_log("info: fpga modem initialized for rx channel %u.\r\n", rx_channel);
//}



/**
 * @brief runs a windowed modem metric test for a number of iterations
 * @param interval_coarse duration of the measurement window
 * @param iters the number of measurement iterations to run
 * @return void.
 * // test: smoke_modem_metrics(100, 8);
 */
void smoke_modem_metrics(uint16_t interval_coarse, uint32_t iters) {
    app_log("- starting modem metrics test -\r\n");
    for (uint32_t i = 0; i < iters; ++i) {
        mm_counts_t c = {0};
        if (mm_get_metric_q15(interval_coarse, NULL, &c, (i < iters - 1)) == MM_OK) {
            // only log the final iteration
            if (i == iters - 1) {
                 app_log("[mm] win=%u chk=%lu det=%lu sent=%lu\r\n",
                    (unsigned)interval_coarse, (unsigned long)c.check,
                    (unsigned long)c.detect, (unsigned long)c.sent);
            }
        } else {
            app_log("err: mm_get_metric timeout\r\n");
            break;
        }
        HAL_Delay(50);
    }
}


/**
 * @brief runs a timing and iq capture test
 * @brief reads and displays timing data and a block of iq samples with integer stats
 * @param iq_start_idx the starting index for the iq capture
 * @param iq_count the number of iq samples to capture
 * @return void.
 * // test: smoke_rx_capture(0, 64);
 */
void smoke_rx_capture(uint16_t iq_start_idx, uint16_t iq_count)
{
    app_log("- starting rx capture test -\r\n");

    // attempt to read timing capture data
    rx_timing_cap_t cap = {0};
    if (!rx_timing_capture_read(&cap, 100)) {
        app_log("warn: timing capture failed or timed out.\r\n");
    }

    // read IQ data block
    if (iq_count > 256) iq_count = 256;
    static int16_t I[256], Q[256];
    uint16_t n = rx_iq_read_block(iq_start_idx, iq_count, I, Q);
    if (n == 0) {
        app_log("err: iq capture failed, 0 samples read.\r\n");
        return;
    }

    // print first few samples
    app_log("[iq_dump] k=0, i=%d, q=%d\r\n", I[0], Q[0]);
    app_log("[iq_dump] k=1, i=%d, q=%d\r\n", I[1], Q[1]);
}


/**
 * @brief runs a pure software test of the qec fixed-point math kernel.
 * @return void.
 * // test: smoke_qec_math();
 */
void smoke_qec_math(void) {
    app_log("--- starting qec math sanity test ---\r\n");
    int32_t g_q31, p_q31;
    int16_t g_q15, p_q15;

    // case 1: ideal, no error
    qec_calc_q31(20000, 0, 20000, &g_q31, &p_q31);
    g_q15 = q31_to_q15_sat(g_q31);
    p_q15 = q31_to_q15_sat(p_q31);
    app_log("[qec_math] ideal: gain_q15=%d, phase_q15=%d (exp: 32767, 0)\r\n", g_q15, p_q15);

    // case 2: small phase error
    qec_calc_q31(20000, -1000, 20000, &g_q31, &p_q31);
    g_q15 = q31_to_q15_sat(g_q31);
    p_q15 = q31_to_q15_sat(p_q31);
    app_log("[qec_math] phase err: gain_q15=%d, phase_q15=%d (exp: positive phase)\r\n", g_q15, p_q15);
}


/**
 * @brief runs full hardware test of the rx-qec loop
 * @brief measures at +/- frequencies, calculates a correction, applies it, and re-measures //not giving great corrections
 * @return void.
 * // test: smoke_qec_hw();
 */
void smoke_qec_hw(void) {
    app_log("--- starting qec hardware flow test (fixed-point) ---\r\n");
    // const float freq_mhz = 10.123;
    const int32_t freq_q16 = (int32_t)(10.123f * 65536.0f); // 10.123 in Q16.16
    const uint16_t num_samples_pow2 = 20;
    int32_t p_ii, p_iq, p_qi, p_qq, n_ii, n_iq, n_qi, n_qq;

    // 1. reset corrections
    qec_apply_rx_correction_q15(32767, 0); // gain=1.0, phase=0.0
    qec_apply_tx_correction_q15(32767, 0, 0);
    HAL_Delay(50);

    // 2. measure at +f and -f
    qec_set_tx_tone_q16(freq_q16);
    qec_correlator_measure(0, num_samples_pow2, &p_ii, &p_iq, &p_qi, &p_qq);
    qec_set_tx_tone_q16(-freq_q16);
    qec_correlator_measure(0, num_samples_pow2, &n_ii, &n_iq, &n_qi, &n_qq);

    // 3. calculate initial error
    int32_t x11_pre = p_ii + n_ii;
    int32_t x12_pre = p_iq + n_iq;
    int32_t x22_pre = p_qq + n_qq;
    app_log("[qec_hw] pre-correction sums: x11=%ld, x12=%ld, x22=%ld\r\n", x11_pre, x12_pre, x22_pre);

    // 4. calculate and apply correction
    int32_t g_corr_q31, p_corr_q31;
    qec_calc_q31(x11_pre, x12_pre, x22_pre, &g_corr_q31, &p_corr_q31);

    int16_t g_corr_q15 = q31_to_q15_sat(g_corr_q31);
    int16_t p_corr_q15 = q31_to_q15_sat(p_corr_q31);

    app_log("[qec_hw] applied correction: gain_q15=%d, phase_q15=%d\r\n", g_corr_q15, p_corr_q15);
    qec_apply_rx_correction_q15(g_corr_q15, p_corr_q15);
    HAL_Delay(50);

    // 5. re-measure to verify
    qec_set_tx_tone_q16(freq_q16);
    qec_correlator_measure(0, num_samples_pow2, &p_ii, &p_iq, &p_qi, &p_qq);
    qec_set_tx_tone_q16(-freq_q16);
    qec_correlator_measure(0, num_samples_pow2, &n_ii, &n_iq, &n_qi, &n_qq);
    int32_t x12_post = p_iq + n_iq;
    app_log("[qec_hw] post-correction iq sum (x12): %ld (should be closer to 0)\r\n", x12_post);

    // 6. cleanup
    qec_set_tx_tone_q16(0);
}

/**
 * @brief forces the transmitter to output CW tone
 * @param freq_q16 the desired tone frequency in mhz, as a q16.16 fixed-point number
 * @return void.
 */
void smoke_force_tx_cw(int32_t freq_q16) {
    // 1. set the NCO frequency for the tone
    qec_set_tx_tone_q16(freq_q16);
    // 2. set the control bit to override the modem data with the CW tone
    reg_write(REG_CTRL, reg_read(REG_CTRL) | CTRL_FORCE_CW_TONE);
    app_log("info: forcing tx continuous wave (cw) tone.\r\n");
}

/**
 * @brief disables tx;s CW tone override
 * @brief returns the transmitter to sending data from the modem
 * @return void.
 */
void smoke_disable_tx_cw(void) {
    reg_write(REG_CTRL, reg_read(REG_CTRL) & ~CTRL_FORCE_CW_TONE);
    app_log("info: disabling tx cw tone, returning to modem data.\r\n");
}

/**
 * @brief complete setup for a packet loopback test
 * @param rx_channel the rx channel to use (1-4)
 * @param tx_gain the tx gain setting (0-31)
 * @param tx_mask the bitmask for the tx channel
 * @return void.
 */
void smoke_setup_for_packet_loopback(uint8_t rx_channel, uint16_t tx_gain, uint16_t tx_mask)
{
    app_log("info: starting complete packet loopback setup sequence...\r\n");

    // 1- configure rx Path and disable tx
    reg_write(REG_CTRL, CTRL_MODEM_RESET); // modem in reset
    HAL_Delay(1);
    reg_write(REG_CTRL, 0x00); // modem release
    HAL_Delay(1);

    reg_write(REG_ROLE_MODE, 0x02); // TDD disabled, primary role
    HAL_Delay(1);

    set_Rx(0); // set MAX2851 to RX, disable TX path (sets REG_CTRL bit 0)
    HAL_Delay(1);

    reg_write(0x65, 0); // ensure loopback path is off initially
    reg_write(REG_RX_MUX, rx_channel);
    reg_write(REG_FRAME_SRC, 2);
    reg_write(REG_CAP_ARM, 1);
    reg_write(REG_RX_SD_FEEDBACK, 0);
    reg_write(REG_RSSI_SETPOINT, 90);
    reg_write(REG_QEC_EN, 0);
    reg_write(0x64, 32); // digital RX gain unity
    HAL_Delay(1);

    app_log("info: rx path configured.\r\n");

    // 2- re-enable tx Path and loopback
    set_Tx(tx_gain, tx_mask); // re-enables TX path (clears REG_CTRL bit 0)
    HAL_Delay(1);

    reg_write(0x65, 0x0002); // enable FPGA's digital loopback path
    HAL_Delay(1);

    app_log("info: tx path and loopback enabled, setup complete.\r\n");
}
