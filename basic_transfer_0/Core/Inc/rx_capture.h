/*
 * rx_capture.h
 *
 * mirrors teensy captures.c access pattern for reading iq and timing data.
 * the iq read uses a two passes, one to read i then q for each index.
 */

#ifndef INC_RX_CAPTURE_H_
#define INC_RX_CAPTURE_H_

#include <stdint.h>
#include <stdbool.h>

// struct to hold all vals from a timing capture event
typedef struct {
    uint32_t tx_time_out;
    uint16_t rx_symbol_idx;
    uint16_t rx_frac_out;
    uint32_t rx_period_out;
    uint32_t phase_out;
    int32_t  tx_err_est;
    uint32_t tx_period_est;
} rx_timing_cap_t;


/**
 * @brief arms the timing capture block and reads latched vals
 * @brief polls the arm register until the capture is complete or a timeout occurs
 * @param cap pointer to a struct where the captured timing data will be stored
 * @param timeout_ms the maximum time to wait for the capture to complete
 * @return true if the capture was successful, false on timeout
 * // test: rx_timing_cap_t cap; bool ok = rx_timing_capture_read(&cap, 100);
 */
bool rx_timing_capture_read(rx_timing_cap_t *cap, uint32_t timeout_ms);

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
uint16_t rx_iq_read_block(uint16_t start_idx, uint16_t count, int16_t *I, int16_t *Q);

#endif /* INC_RX_CAPTURE_H_ */
