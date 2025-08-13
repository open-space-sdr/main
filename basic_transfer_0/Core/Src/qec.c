/*
 * qec.c
 *
 * hardware control for QEC (fixed-point)
 */

#include "qec.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "app_log.h"
#include "main.h"
#include "math_utils.h" // includes the trig functions for fixed pont

#define SATURATE_S8(x) ((x) > 127 ? 127 : ((x) < -128 ? -128 : (x)))

void qec_set_tx_tone_q16(int32_t freq_q16) {
    // FPGA's NCO  requires a phase increment value
    // - calculated from the desired frequency relative to the sample clock
    // formula: phase_inc = (freq_hz / sample_rate_hz) * 2^16
    // Here, sample_rate is 80MHz. this simplifies to integer math:
    int16_t phase_inc = (int16_t)(freq_q16 / 80);
    reg_write(0x24, phase_inc);
}

void qec_correlator_measure(uint16_t rxqec0_txqec1, uint16_t qec_pow2_num_samples,
                            int32_t *corr_ii, int32_t *corr_iq, int32_t *corr_qi, int32_t *corr_qq)
{
    // configure correlator: set sample count and mode (RX vs TX).
    reg_write(REG_QEC_EN, (qec_pow2_num_samples << 2) | (rxqec0_txqec1 << 1) | 1);

    // trigger the measurement by forcing the CW tone on
    reg_write(REG_CTRL, reg_read(REG_CTRL) | CTRL_FORCE_CW_TONE);
    HAL_Delay(1); // Brief delay to ensure trigger is seen.
    // immediately turn the tone off to allow for subsequent re-triggering
    reg_write(REG_CTRL, reg_read(REG_CTRL) & ~CTRL_FORCE_CW_TONE);

    // wait for the measurement to complete, the duration depends on sample count
    HAL_Delay(20);

    // disable the correlator block
    reg_write(REG_QEC_EN, 0x00);

    // read back the 32-bit results. Each result is read in two 16-bit halves by writing the index of the half
    // to the register before reading.
    uint16_t temp_lsb;
    reg_write(REG_QEC_EN, 0 << 7); temp_lsb = reg_read(REG_QEC_EN);
    reg_write(REG_QEC_EN, 1 << 7); *corr_ii = (int32_t)((((uint32_t)reg_read(REG_QEC_EN)) << 16) | temp_lsb);

    reg_write(REG_QEC_EN, 2 << 7); temp_lsb = reg_read(REG_QEC_EN);
    reg_write(REG_QEC_EN, 3 << 7); *corr_iq = (int32_t)((((uint32_t)reg_read(REG_QEC_EN)) << 16) | temp_lsb);

    reg_write(REG_QEC_EN, 4 << 7); temp_lsb = reg_read(REG_QEC_EN);
    reg_write(REG_QEC_EN, 5 << 7); *corr_qi = (int32_t)((((uint32_t)reg_read(REG_QEC_EN)) << 16) | temp_lsb);

    reg_write(REG_QEC_EN, 6 << 7); temp_lsb = reg_read(REG_QEC_EN);
    reg_write(REG_QEC_EN, 7 << 7); *corr_qq = (int32_t)((((uint32_t)reg_read(REG_QEC_EN)) << 16) | temp_lsb);
}

void qec_apply_rx_correction_q15(int16_t gain_q15, int16_t phase_q15) {
    // FPGA expects correction values as signed 8-bit integers
    // we must scale Q1.15 values into this range
    // Formula: scaled = round(value_q15 * 256 / 32768) = round(value_q15 / 128)
    // scale phase: ((phase_q15 * 256) + rounding) >> 15
    int16_t phase_scaled = ((int32_t)phase_q15 * 256 + (1 << 14)) >> 15;

    // scale gain
    int16_t gain_minus_one = gain_q15 - 32767; // 1.0 in Q1.15 is 32767
    int16_t gain_scaled = ((int32_t)gain_minus_one * 256 + (1 << 14)) >> 15;

    // pack the 8-bit values into a single 16-bit word and write to the reg
    uint16_t write_val = ((uint8_t)SATURATE_S8(phase_scaled) << 8) | (uint8_t)SATURATE_S8(gain_scaled);
    reg_write(0x63, write_val);
    HAL_Delay(10);
}

void qec_apply_tx_correction_q15(int16_t gain_q15, int16_t phase_q15, int32_t delay_q16)
{
    // implements the TX QEC formulas using fixed-point
    // Original float logic:
    // gain_new = gain/cosf(phase) - 1.0;
    // phase_new = sinf(phase);

    // calculate sin(phase) and cos(phase) using the lookup table helpers (do we need more precision here?)
    int16_t cos_p = cos_q15_from_rad(phase_q15);
    int16_t sin_p = sin_q15_from_rad(phase_q15);

    // calculate gain/cos(phase) using fixed-point division
    int16_t gain_div_cos = q15_div(gain_q15, cos_p);

    // subtract 1.0 to get the final gain correction factor.
    int16_t gain_final = gain_div_cos - 32767; // 32767 is 1.0 in Q1.15

    // scale the final values to the format expected by the FPGA registers
    // teensy code used scaling factors of 512 for gain/phase and 1024 for delay.
    uint16_t gain_val  = ((int32_t)gain_final * 512) >> 15;
    uint16_t phase_val = ((int32_t)sin_p * 512) >> 15;
    uint16_t delay_val = (uint16_t)(((int64_t)delay_q16 * 1024) >> 16);

    // write the scaled values to the FPGA's windowed register.
    reg_write(0x27, delay_val);
    reg_write(0x27, phase_val);
    reg_write(0x27, gain_val);
}
