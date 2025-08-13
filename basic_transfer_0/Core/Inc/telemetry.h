/*
 * telemetry.h
 *
 * integer-only helpers for reading telemetry from fpga
 * layered on the fpga register api
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>

// struct to hold the basic telemetry values.
typedef struct {
    int8_t   dc_i;
    int8_t   dc_q;
    uint16_t rssi_in;
    uint16_t rssi_rs;
    uint16_t evm_ave;
    uint16_t rx_period_hi;
    uint16_t adjust_needed;
} telem_basic_t;

/**
 * @brief reads a single 16-bit value from the telemetry mux
 * @param sel the telemetry selector index (1-6)
 * @return the 16-bit value from the selected telemetry source
 * // test: uint16_t rssi = telem_read_sel(2);
 */
uint16_t telem_read_sel(uint8_t sel);

/**
 * @brief reads all basic telemetry vals and populates a struct
 * @param out pointer to a telem_basic_t struct to be filled with data.
 * @return void.
 * // test: telem_basic_t telem; telem_read_basic(&telem);
 */
void telem_read_basic(telem_basic_t *out);

/**
 * @brief prints the contents of a telemetry struct
 * @param t pointer to the telem_basic_t struct to print
 * @return void.
 * // test: telem_print_compact(&telem);
 */
void telem_print_compact(const telem_basic_t *t);

#endif /* INC_TELEMETRY_H_ */
