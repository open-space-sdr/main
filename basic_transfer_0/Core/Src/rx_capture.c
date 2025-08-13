/*
 * rx_capture.c
 *
 * mirrors teensy captures.c access pattern for reading iq and timing data.
 * the iq read uses a two passes, one to read i then q for each index.
 */

#include "rx_capture.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "main.h"

/**
 * @brief arms the timing capture block and reads latched vals
 * @brief polls the arm register until the capture is complete or a timeout occurs
 * @param cap pointer to a struct where the captured timing data will be stored
 * @param timeout_ms the maximum time to wait for the capture to complete
 * @return true if the capture was successful, false on timeout
 * // test: rx_timing_cap_t cap; bool ok = rx_timing_capture_read(&cap, 100);
 */
bool rx_timing_capture_read(rx_timing_cap_t *cap, uint32_t timeout_ms)
{
    if (!cap) return false;

    // arm timing capture
    reg_write(REG_TCAP_ARM, 1);

    // poll until finished (register reads 0) or timeout
    uint32_t start_time = HAL_GetTick();
    while (reg_read(REG_TCAP_ARM) != 0) {
        if (HAL_GetTick() - start_time > timeout_ms) {
            return false; // timeout
        }
    }

    // read back all latched registers
    cap->tx_time_out   = ((uint32_t)reg_read(REG_TCAP_TXTIME_MS) << 16) | reg_read(REG_TCAP_TXTIME_LS);
    cap->rx_symbol_idx = reg_read(REG_TCAP_RXSYM_IDX);
    cap->rx_frac_out   = reg_read(REG_TCAP_RXFRAC);
    cap->rx_period_out = ((uint32_t)reg_read(REG_TCAP_RXPER_MS) << 16) | reg_read(REG_TCAP_RXPER_LS);
    cap->phase_out     = reg_read(REG_TCAP_PHASE);
    cap->tx_err_est    = (int32_t)(((uint32_t)reg_read(REG_TCAP_TXERR_MS) << 16) | reg_read(REG_TCAP_TXERR_LS));
    cap->tx_period_est = ((uint32_t)reg_read(REG_TCAP_TXPER_MS) << 16) | reg_read(REG_TCAP_TXPER_LS);

    return true;
}

/**
 * @brief reads a block of iq samples from the fpga's capture ram
 * @brief implements the two-pass read required by the hardware (read i, then read q).
 * @param start_idx the starting index in the ram to read from.
 * @param count the number of iq samples to read
 * @param I pointer to an array to store the i-channel samples (signed 16 bit)
 * @param Q pointer to an array to store the q-channel samples (signed 16 bit)
 * @return the number of samples actually read.
 * // test: int16_t i[256], q[256]; rx_iq_read_block(0, 256, i, q);
 */
uint16_t rx_iq_read_block(uint16_t start_idx, uint16_t count, int16_t *I, int16_t *Q)
{
    if (!I || !Q || count == 0) return 0;

    for (uint16_t k = 0; k < count; ++k) {
        uint16_t addr = start_idx + k;

        // pass1: Read I sample
        reg_write(REG_IQ_IDX, addr);
        I[k] = (int16_t)reg_read(REG_I_RAM);

        // passs 2: Read Q sample
        reg_write(REG_IQ_IDX, addr);
        Q[k] = (int16_t)reg_read(REG_Q_RAM);
    }

    return count;
}
