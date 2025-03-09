/*
 * ENCODER.h
 *
 *  Created on: Oct 20, 2022
 *      Author: Luke Gutierrez
 *
 *      Encoder Driver Tailored for NGCP
 *
 *		Ver. 1.1
 */
#ifndef UGV_ENCODER_H
#define UGV_ENCODER_H

#include "stm32f7xx_hal.h"
#include <stdlib.h>

#define PI 3.141592654
#define ft_in 0.083333333
#define ftpm_mph 0.0113636


float encoder(uint32_t e);


#endif
