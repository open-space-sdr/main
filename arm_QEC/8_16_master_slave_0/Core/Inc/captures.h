/*
 * captures.h
 * Functions for capturing IQ and debug data from the FPGA.
 */

#ifndef INC_CAPTURES_H_
#define INC_CAPTURES_H_

#include <stdint.h>

/**
 * @brief Captures and prints IQ data from the FPGA's debug RAM.
 * @return void.
 */
void rx_debug_capture(void);


#endif /* INC_CAPTURES_H_ */
