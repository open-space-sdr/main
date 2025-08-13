/*
 * modem_metrics.h
 *
 * fixed-point (q15) modem metric readout via fpga regs
 * mirrors hdl: start/stop latches with 32-bit reads
 */

#ifndef INC_MODEM_METRICS_H_
#define INC_MODEM_METRICS_H_

#include <stdint.h>
#include <stdbool.h>

// status codes for the metric acquisition function.
typedef enum {
    MM_OK = 0,
    MM_ERR_TIMEOUT = -2,
    MM_ERR_INVALID_ARG = -1,
} mm_status_t;

// struct to hold the raw delta counts from a measurement window
typedef struct {
    uint32_t evm;
    uint32_t check;
    uint32_t detect;
    uint32_t sent;
    uint16_t interval_applied;
} mm_counts_t;


/**
 * @brief performs a windowed modem metric measurement
 * @brief triggers a measurement window, polls for completion, reads the
 * @brief start and stop counters, and calculates the deltas
 * @param interval_coarse duration of the measurement window in coarse units (of 65536 cycles) ?
 * @param[out] metric_q15_out pointer to store the calculated evm metric in q15 format. can be null.
 * @param[out] counts_out pointer to a struct to store the raw delta counts, can be null.
 * @param quiet if true, suppresses logging output for this call.
 * @return mm_status_t status code indicating success or failure.
 * // test: mm_counts_t counts; mm_get_metric_q15(100, NULL, &counts, false);
 */
mm_status_t mm_get_metric_q15(uint16_t interval_coarse, int32_t *metric_q15_out, mm_counts_t *counts_out, bool quiet);

#endif /* INC_MODEM_METRICS_H_ */
