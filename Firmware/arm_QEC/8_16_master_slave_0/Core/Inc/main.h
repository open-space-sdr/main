/*
 * main.h
 *
 */

#ifndef INC_MAIN_H_
#define INC_MAIN_H_

// Board's role:
// Ex: to test link, flash one board as ROLE_MASTER and another as ROLE_SLAVE.
#define ROLE_MASTER 1
#define ROLE_SLAVE  0

// Set the role for this build
#define DEVICE_ROLE ROLE_SLAVE

// feature toggles:
// Select which test to run in the main loop.
// Only one test should be set to 1 at a time.
// RUN_DEFAULT_SEQ will perform QEC, then receive packets with that gain
// RUN_GAIN_TRACK_TEST will perform QEC then enter gain tracking mode.
#define RUN_DEFAULT_SEQ         0
#define RUN_QEC_TEST            0
#define RUN_VGA_TEST            0
#define RUN_IQ_CAPTURE_TEST     0
#define RUN_GAIN_TRACK_TEST     1


// HAL includes to be accessible by all modules
#include "stm32g4xx_hal.h"


#endif /* INC_MAIN_H_ */
