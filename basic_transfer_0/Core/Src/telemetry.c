/*
 * telemetry.c
 *
 * integer-only helpers for reading telemetry from fpga
 * layered on the fpga register api
 */

#include "telemetry.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "app_log.h"
#include "main.h"

/**
 * @brief reads a single 16-bit value from the telemetry mux
 * @param sel the telemetry selector index (1-6)
 * @return the 16-bit value from the selected telemetry source
 * // test: uint16_t rssi = telem_read_sel(2);
 */
uint16_t telem_read_sel(uint8_t sel) {
    // write the selector to the mux register
    reg_write(REG_TLM_MUX, sel);
    return reg_read(REG_TLM_MUX);
}

/**
 * @brief reads all basic telemetry vals and populates a struct
 * @param out pointer to a telem_basic_t struct to be filled with data.
 * @return void.
 * // test: telem_basic_t telem; telem_read_basic(&telem);
 */
void telem_read_basic(telem_basic_t *out) {
    if (!out) return;

    // telem selectors defined in fpga_regs.h
    uint16_t v = telem_read_sel(TELEM_SEL_DC_IQ);
    out->dc_i = (int8_t)(v >> 8);
    out->dc_q = (int8_t)(v & 0xFF);

    out->rssi_in = telem_read_sel(TELEM_SEL_RSSI_D_IN) & 0x01FFu;
    out->rssi_rs = telem_read_sel(TELEM_SEL_RSSI_RESAMPLED) & 0x01FFu;
    out->evm_ave = telem_read_sel(TELEM_SEL_EVM_LEAKY) & 0x01FFu;

    out->rx_period_hi = telem_read_sel(TELEM_SEL_RX_PERIOD_HI);
    out->adjust_needed = telem_read_sel(TELEM_SEL_ADJUST_NEEDED) & 0x001Fu;
}

/**
 * @brief prints the contents of a telemetry struct
 * @param t pointer to the telem_basic_t struct to print
 * @return void.
 * // test: telem_print_compact(&telem);
 */
void telem_print_compact(const telem_basic_t *t) {
    if (!t) return;
    app_log("[telem] dc_i=%d dc_q=%d rssi_in=%u rssi_rs=%u evm=%u\r\n",
            (int)t->dc_i, (int)t->dc_q,
            (unsigned)t->rssi_in, (unsigned)t->rssi_rs,
            (unsigned)t->evm_ave);
}
