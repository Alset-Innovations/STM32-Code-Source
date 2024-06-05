/*
 * Functions.h
 *
 *  Created on: May 16, 2024
 *      Author: luuk
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_



#endif /* INC_FUNCTIONS_H_ */

/* Private includes ----------------------------------------------------------*/

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "adc.h"

/* Private function prototypes -----------------------------------------------*/

uint8_t PrepareCommutation (char Direction);
uint8_t StartupSequence (char Direction);
uint8_t StopSequence(void);
uint8_t ChangePWM (void);
