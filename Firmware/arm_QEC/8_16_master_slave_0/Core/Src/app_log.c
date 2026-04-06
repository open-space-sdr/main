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

/**
 * @brief Halts execution until a character is received on the debug UART.
 * @brief This is a port of the waitForKeyPress function from the Teensy code.
 * @return void.
 */
void waitForKeyPress(void) {
    app_log("\r\nPress any key to continue...\r\n");
    UART_HandleTypeDef* huart = get_debug_uart();
    uint8_t received_char;

    // Flush any old data in the buffer
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        HAL_UART_Receive(huart, &received_char, 1, 1);
    }

    // Wait for a new character
    while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        HAL_Delay(10);
    }

    // Read and discard the character
    HAL_UART_Receive(huart, &received_char, 1, 100);
}
