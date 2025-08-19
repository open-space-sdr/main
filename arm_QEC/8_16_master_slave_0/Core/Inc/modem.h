/*
 * modem.h
 * Core modem functions
 */

#ifndef INC_MODEM_H_
#define INC_MODEM_H_

#include <stdint.h>

/**
 * @brief Reads modem performance counters over a measurement interval.
 * @param quiet If 0, prints the metrics to the debug log.
 * @return The number of successful checksums during the interval.
 */
float get_modem_metric(int quiet);

/**
 * @brief Reads and prints telemetry values from the FPGA.
 * @return void.
 */
void rx_telem(void);


#endif /* INC_MODEM_H_ */
