/*
 * modem_metrics.c
 *
 * modem metric logic: start/stop latches, 32-bit reads.
 * integer-only; q15 output with careful rounding
 */

#include "modem_metrics.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "app_log.h"
#include "main.h" // For HAL_Delay

// read a 32-bit value from the metric block by selecting and then reading ms/ls parts.
static inline uint32_t mm_read32_sel(uint8_t sel) {
    reg_write(REG_MM_MUX_LS, sel);
    uint16_t ms = reg_read(REG_MM_TRIGGER); // corrected, was REG_MM_MS
    uint16_t ls = reg_read(REG_MM_MUX_LS);
    return ((uint32_t)ms << 16) | ls;
}

// calculates delta between stop and start counter, handling 32-bit wraparound
static inline uint32_t delta32(uint32_t stop, uint32_t start) {
    return (stop >= start) ? (stop - start) : (stop + (0xFFFFFFFFu - start) + 1u);
}

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
mm_status_t mm_get_metric_q15(uint16_t interval_coarse, int32_t *metric_q15_out, mm_counts_t *counts_out, bool quiet)
{
    if (metric_q15_out) *metric_q15_out = 0;
    if (counts_out) {
        counts_out->evm = counts_out->check = counts_out->detect = counts_out->sent = 0;
        counts_out->interval_applied = interval_coarse;
    }

    // program measurement window interval
    reg_write(REG_MM_INTERVAL, interval_coarse);

    // trigger with a 0 -> 1 edge on the trigger register
    reg_write(REG_MM_TRIGGER, 0x0000);
    reg_write(REG_MM_TRIGGER, 0x0001);

    // immediately read the latched 'start' values
    uint32_t evm_start   = mm_read32_sel(MM_SEL_EVM_START);
    uint32_t chk_start   = mm_read32_sel(MM_SEL_CHECK_START);
    uint32_t det_start   = mm_read32_sel(MM_SEL_DETECT_START);
    uint32_t sent_start  = mm_read32_sel(MM_SEL_MEM_START);

    // wait for window to complete by polling the interval register
    uint32_t start_time = HAL_GetTick();
    while (reg_read(REG_MM_INTERVAL) != 0) {
        if (HAL_GetTick() - start_time > 2000) { // 2 sec timeout
            if (!quiet) app_log("err: mm_get_metric timeout\r\n");
            return MM_ERR_TIMEOUT;
        }
    }

    // read the latched 'stop' values
    uint32_t evm_stop    = mm_read32_sel(MM_SEL_EVM_STOP);
    uint32_t chk_stop    = mm_read32_sel(MM_SEL_CHECK_STOP);
    uint32_t det_stop    = mm_read32_sel(MM_SEL_DETECT_STOP);
    uint32_t sent_stop   = mm_read32_sel(MM_SEL_MEM_STOP);

    // calculate wraparound-safe deltas
    uint32_t d_evm   = delta32(evm_stop, evm_start);
    uint32_t d_check = delta32(chk_stop, chk_start);
    uint32_t d_det   = delta32(det_stop, det_start);
    uint32_t d_sent  = delta32(sent_stop, sent_start);

    if (counts_out) {
        counts_out->evm = d_evm;
        counts_out->check = d_check;
        counts_out->detect = d_det;
        counts_out->sent = d_sent;
    }

    // calculate evm/check as a q15 value
    if (metric_q15_out) {
        uint32_t denom = d_check ? d_check : 1u;
        uint64_t num_scaled = ((uint64_t)d_evm << 15) + (denom >> 1); // with rounding
        *metric_q15_out = (int32_t)(num_scaled / denom);
    }

    if (!quiet) {
        app_log("[mm] win=%u chk=%lu det=%lu sent=%lu\r\n",
                (unsigned)interval_coarse, (unsigned long)d_check,
                (unsigned long)d_det, (unsigned long)d_sent);
    }

    return MM_OK;
}
