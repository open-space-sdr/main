/*
 * qec.h
 *
 * Functions for Quadrature Error Correction and related tone generation.
 */

#ifndef INC_QEC_H_
#define INC_QEC_H_

#include <stdint.h>

/**
 * @brief Sets the frequency of the CW tone generator in the FPGA.
 * @param freq_mhz The desired frequency in MHz, relative to the LO.
 * The FPGA expects a scaled 16-bit integer.
 * @return void.
 */
void set_tx_tone_freq(float freq_mhz);

/**
 * @brief Performs the full Quadrature Error Correction sequence.
 * @brief This routine measures and corrects for gain/phase imbalances in the
 * @brief receiver and transmitter paths for all active channels.
 * @return void.
 */
void QEC(void);


#endif /* INC_QEC_H_ */
