/*
 * modem.c
 *
 */

#include "modem.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "app_log.h"
#include "main.h" // For HAL_Delay
#include <math.h> // For sqrtf

float get_modem_metric(int quiet) {
    const uint16_t interval_count = 100;
    reg_write(REG_MM_INTERVAL, interval_count);

    reg_write(REG_MM_TRIGGER, 0);
    reg_write(REG_MM_TRIGGER, 1);

    reg_write(REG_MM_MUX_LS, MM_SEL_EVM_START);
    uint32_t evm_acc_start = ((uint32_t)reg_read(REG_MM_TRIGGER) << 16) | reg_read(REG_MM_MUX_LS);
    reg_write(REG_MM_MUX_LS, MM_SEL_CHECK_START);
    uint32_t check_count_start = ((uint32_t)reg_read(REG_MM_TRIGGER) << 16) | reg_read(REG_MM_MUX_LS);
    reg_write(REG_MM_MUX_LS, MM_SEL_DETECT_START);
    uint32_t detect_count_start = ((uint32_t)reg_read(REG_MM_TRIGGER) << 16) | reg_read(REG_MM_MUX_LS);
    reg_write(REG_MM_MUX_LS, MM_SEL_SENT_START);
    uint32_t sent_count_start = ((uint32_t)reg_read(REG_MM_TRIGGER) << 16) | reg_read(REG_MM_MUX_LS);

    uint32_t timeout_start = HAL_GetTick();
    while (reg_read(REG_MM_INTERVAL) != 0) {
        if (HAL_GetTick() - timeout_start > 200) {
            app_log("- Error: Timed out waiting for modem metric capture.\r\n");
            return 0.0f;
        }
        HAL_Delay(1);
    }

    reg_write(REG_MM_MUX_LS, MM_SEL_EVM_STOP);
    uint32_t evm_acc_stop = ((uint32_t)reg_read(REG_MM_TRIGGER) << 16) | reg_read(REG_MM_MUX_LS);
    reg_write(REG_MM_MUX_LS, MM_SEL_CHECK_STOP);
    uint32_t check_count_stop = ((uint32_t)reg_read(REG_MM_TRIGGER) << 16) | reg_read(REG_MM_MUX_LS);
    reg_write(REG_MM_MUX_LS, MM_SEL_DETECT_STOP);
    uint32_t detect_count_stop = ((uint32_t)reg_read(REG_MM_TRIGGER) << 16) | reg_read(REG_MM_MUX_LS);
    reg_write(REG_MM_MUX_LS, MM_SEL_SENT_STOP);
    uint32_t sent_count_stop = ((uint32_t)reg_read(REG_MM_TRIGGER) << 16) | reg_read(REG_MM_MUX_LS);

    uint32_t sent_increase = (sent_count_stop >= sent_count_start) ? sent_count_stop - sent_count_start : sent_count_stop - sent_count_start + 0xFFFFFFFF;
    uint32_t detect_increase = (detect_count_stop >= detect_count_start) ? detect_count_stop - detect_count_start : detect_count_stop - detect_count_start + 0xFFFFFFFF;
    uint32_t check_increase = (check_count_stop >= check_count_start) ? check_count_stop - check_count_start : check_count_stop - check_count_start + 0xFFFFFFFF;
    // evm calculation removed from log to match teensy code
    // uint32_t evm = (evm_acc_stop >= evm_acc_start) ? evm_acc_stop - evm_acc_start : evm_acc_stop - evm_acc_start + 0xFFFFFFFF;

    if (!quiet) {
        app_log("checksum:%lu, detects:%lu, sent:%lu\r\n", check_increase, detect_increase, sent_increase);
        // 32-bit counter wrapping around (from 0xFFFFFFFF back to 0)
        // calculates difference using unsigned arithmetic (stop_count - start_count)
        // can give large numbers near max 32-bit int
        // way to fix this?
    }

    return (float)check_increase;
}

void rx_telem(void)
{
    reg_write(REG_TLM_MUX, TELEM_SEL_DC_IQ);
    uint16_t dc_iq = reg_read(REG_TLM_MUX);
    float dc_i = (float)(int8_t)(dc_iq >> 8);
    float dc_q = (float)(int8_t)(dc_iq & 0xFF);
    float dc_mag = sqrtf(dc_i*dc_i + dc_q*dc_q);
    app_log("dc_i:%.1f, dc_q:%.1f, dc:%.1f,", dc_i, dc_q, dc_mag);

    reg_write(REG_TLM_MUX, TELEM_SEL_RSSI_D_IN);
    uint16_t rssi_in = reg_read(REG_TLM_MUX);
    app_log("rssin:%u,", rssi_in);

    reg_write(REG_TLM_MUX, TELEM_SEL_RSSI_RESAMPLED);
    uint16_t rssi_res = reg_read(REG_TLM_MUX);
    app_log("rssres:%u,", rssi_res);

    reg_write(REG_TLM_MUX, TELEM_SEL_EVM_LEAKY);
    uint16_t evm = reg_read(REG_TLM_MUX);
    app_log("evm:%u,", evm);

    reg_write(REG_TLM_MUX, TELEM_SEL_RX_PERIOD_HI);
    uint16_t rxp = reg_read(REG_TLM_MUX);
    app_log("rxp:%u,", rxp);

    reg_write(REG_TLM_MUX, TELEM_SEL_ADJUST_NEEDED);
    uint16_t adj = reg_read(REG_TLM_MUX);
    app_log("adj:%u\r\n", adj);
}
