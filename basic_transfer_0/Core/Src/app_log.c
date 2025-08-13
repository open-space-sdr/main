/*
 * app_log.c
 *
 * logging/Uart utility
 */

#include "app_log.h"
#include "main.h" // Required for UART_HandleTypeDef and HAL_UART_Transmit

extern UART_HandleTypeDef* get_debug_uart(void);


/**
 * @brief sends formatted string to uart.
 * @brief wrapper around vsnprintf and hal_uart_transmit.
 * @param fmt the printf-style format string.
 * @param ... variable arguments for the format string.
 * @return void.
 * // test: app_log("system init with status %d\r\n", status);
 */
void app_log(const char *fmt, ...) {
    static char buf[256];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        UART_HandleTypeDef* huart = get_debug_uart();
        if (huart) {
            HAL_UART_Transmit(huart, (uint8_t*)buf, n, 100);
        }
    }
}
