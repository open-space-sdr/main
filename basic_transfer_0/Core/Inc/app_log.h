/*
 * app_log.h
 *
 * logging/Uart utility
 */

#ifndef INC_APP_LOG_H_
#define INC_APP_LOG_H_

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief sends formatted string to uart.
 * @brief wrapper around vsnprintf and hal_uart_transmit.
 * @param fmt the printf-style format string.
 * @param ... variable arguments for the format string.
 * @return void.
 * // test: app_log("system init with status %d\r\n", status);
 */
void app_log(const char *fmt, ...);


#endif /* INC_APP_LOG_H_ */
