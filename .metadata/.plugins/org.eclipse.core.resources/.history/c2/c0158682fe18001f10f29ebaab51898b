/*
 * Functions.c
 *
 *  Created on: May 16, 2024
 *      Author: luuk
 */

/* Private includes ----------------------------------------------------------*/

#include "Functions.h"

/* Private variables ---------------------------------------------------------*/

uint16_t Commutation[6 * 4][3] = {
		// Backwards
		{0x0405, 0x0858, 0x4868}, // 010 Phase 3 Low,  Phase 2 Off,  Phase 1 High
		{0x0540, 0x0868, 0x5848}, // 100 Phase 3 High, Phase 2 Low,  Phase 1 Off
		{0x0045, 0x0848, 0x5868}, // 110 Phase 3 Off,  Phase 2 Low,  Phase 1 High
		{0x0054, 0x0848, 0x6858}, // 001 Phase 3 Off,  Phase 2 High, Phase 1 Low
		{0x0450, 0x0858, 0x6848}, // 011 Phase 3 Low,  Phase 2 High, Phase 1 Off
		{0x0504, 0x0868, 0x4858}, // 101 Phase 3 High, Phase 2 Off,  Phase 1 Low

		// Forwards (To be done)
		{0x0405, 0x0858, 0x4868}, // 010 Phase 3 Low,  Phase 2 Off,  Phase 1 High
		{0x0540, 0x0868, 0x5848}, // 100 Phase 3 High, Phase 2 Low,  Phase 1 Off
		{0x0045, 0x0848, 0x5868}, // 110 Phase 3 Off,  Phase 2 Low,  Phase 1 High
		{0x0054, 0x0848, 0x6858}, // 001 Phase 3 Off,  Phase 2 High, Phase 1 Low
		{0x0450, 0x0858, 0x6848}, // 011 Phase 3 Low,  Phase 2 High, Phase 1 Off
		{0x0504, 0x0868, 0x4858}, // 101 Phase 3 High, Phase 2 Off,  Phase 1 Low

		// Initial Backwards (To be done)
		{0x0405, 0x0858, 0x4868}, // 010 Phase 3 Low,  Phase 2 Off,  Phase 1 High
		{0x0540, 0x0868, 0x5848}, // 100 Phase 3 High, Phase 2 Low,  Phase 1 Off
		{0x0045, 0x0848, 0x5868}, // 110 Phase 3 Off,  Phase 2 Low,  Phase 1 High
		{0x0054, 0x0848, 0x6858}, // 001 Phase 3 Off,  Phase 2 High, Phase 1 Low
		{0x0450, 0x0858, 0x6848}, // 011 Phase 3 Low,  Phase 2 High, Phase 1 Off
		{0x0504, 0x0868, 0x4858}, // 101 Phase 3 High, Phase 2 Off,  Phase 1 Low

		// Initial Forwards (To be done)
		{0x0405, 0x0858, 0x4868}, // 010 Phase 3 Low,  Phase 2 Off,  Phase 1 High
		{0x0540, 0x0868, 0x5848}, // 100 Phase 3 High, Phase 2 Low,  Phase 1 Off
		{0x0045, 0x0848, 0x5868}, // 110 Phase 3 Off,  Phase 2 Low,  Phase 1 High
		{0x0054, 0x0848, 0x6858}, // 001 Phase 3 Off,  Phase 2 High, Phase 1 Low
		{0x0450, 0x0858, 0x6848}, // 011 Phase 3 Low,  Phase 2 High, Phase 1 Off
		{0x0504, 0x0868, 0x4858}, // 101 Phase 3 High, Phase 2 Off,  Phase 1 Low
};

// USB communication
uint16_t len = 0;
char buf[64];

uint32_t Fapb1tclk = 0;
uint32_t Fapb2tclk = 0;
uint32_t RPMConst = 0;

uint32_t RPMValue = 0;

/* Private function prototypes -----------------------------------------------*/

uint8_t PrepareCommutation (char Direction);
uint8_t StartupSequence (char Direction);
uint8_t StopSequence(void);
uint8_t ChangePWM (void);

/* Private function ----------------------------------------------------------*/

uint8_t PrepareCommutation (char Direction) {

	// Read IDR for Hall Sensor status
	uint16_t Hall = (GPIOA->IDR & 0b111) - 1 + 6 * Direction;

	// Set Registers to required values
	TIM1->CCER  = Commutation[Hall][0];
	TIM1->CCMR1 = Commutation[Hall][2];
	TIM1->CCMR2 = Commutation[Hall][1];

	return HAL_OK;

}

uint8_t StartupSequence (char Direction) {

	// Initialize some variables
	Fapb1tclk = HAL_RCC_GetPCLK1Freq() * 2;
	Fapb2tclk = HAL_RCC_GetPCLK2Freq() * 2;
	RPMConst = (Fapb2tclk / (TIM9->PSC + 1)) * 2;

	// Set first commutation state according to Hall sensors
	if (PrepareCommutation (Direction + 6 * 2)) {
		return HAL_ERROR;
	}

	// Start HallSensor timer
	HAL_TIMEx_HallSensor_Start (&htim2);

	// Start all PWM signals on TIM1
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_3);

	// Disable all interrupts
	TIM1->DIER &= ~TIM_DIER_COMIE;	// Disable Commutation events in DIER register
	TIM1->DIER &= ~TIM_DIER_BIE; 	// Disable break interrupt as this is shared with timer 9 interrupt
	TIM2->DIER &= ~TIM_DIER_TIE; 	// Disable interrupt on timer 2
	TIM9->DIER &= ~TIM_DIER_TIE; 	// Disable interrupt on timer 9

	// Start Interrupts
	HAL_TIM_Base_Start_IT (&htim1);
	HAL_TIM_Base_Start_IT (&htim2);
	HAL_TIM_Base_Start_IT (&htim9);
	HAL_TIM_IC_Start_IT (&htim9, TIM_CHANNEL_2);

	// Clear all interrupt triggers
	TIM1->SR &= ~TIM_SR_COMIF;		// Clear Commutation interrupt flag
	TIM1->SR &= ~TIM_SR_BIF;		// Clear Break interrupt flag
	TIM2->SR &= ~TIM_SR_TIF;		// Clear timer 2 interrupt flag
	TIM9->SR &= ~TIM_SR_TIF;		// Clear timer 9 interrupt flag

	// Enable interrupts
	TIM1->DIER |= TIM_DIER_COMIE;	// Enable Commutation events in DIER register
	TIM2->DIER |= TIM_DIER_TIE; 	// Enable interrupt on timer 2
	TIM9->DIER |= TIM_DIER_TIE; 	// Enable interrupt on timer 9

	// Write some registers
	TIM1->CR2  |= TIM_CR2_CCPC; 	// Set CCPC in CR2 to preload CCxE, CCxNE and OCxM bits
	TIM1->BDTR &= ~TIM_BDTR_DTG;	// Reset DTG bits
	TIM1->BDTR |= 0x800A;			// Set dead-time to 100ns and make sure to enable MOE bit
	TIM1->EGR  |= TIM_EGR_COMG; 	// Set COMG bit in EGR for first commutation
	TIM1->DIER |= TIM_DIER_COMIE; 	// Enable commutation events in DIER register

	return HAL_OK;

}

uint8_t StopSequence(void) {

	// Stop HallSensor timer
	HAL_TIMEx_HallSensor_Stop (&htim2);

	// Disable all output channels
	TIM1->CCER  = 0x0000;
	TIM1->CCMR1 = 0x0000;
	TIM1->CCMR2 = 0x0000;

	// Perform one last commutation event if not already disabled
	if ( (TIM1->DIER & TIM_DIER_COMIE) >= 1 ) { 		// If COMIE bit in DIER is set commutation events are still enabled

		TIM1->EGR |= TIM_EGR_COMG; 						// Trigger commutation event
		// while ( (TIM1->SR & TIM_SR_COMIF) >= 1 ); 	// Wait until commutation event has happened

	}

	// Disable all interrupts
	TIM1->DIER &= ~TIM_DIER_COMIE;	// Disable Commutation events in DIER register
	TIM1->DIER &= ~TIM_DIER_BIE; 	// Disable break interrupt as this is shared with timer 9 interrupt
	TIM2->DIER &= ~TIM_DIER_TIE; 	// Disable interrupt on timer 2
	TIM9->DIER &= ~TIM_DIER_TIE; 	// Disable interrupt on timer 9

	// Stop interrupts
	HAL_TIM_Base_Stop_IT (&htim1);
	HAL_TIM_Base_Stop_IT (&htim2);
	HAL_TIM_Base_Stop_IT (&htim9);
	HAL_TIM_IC_Stop_IT (&htim9, TIM_CHANNEL_2);

	// Stop PWM Timers
	HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_3);

	Registers[RPMReg] = 0;

	return HAL_OK;

}

uint8_t ChangePWM (void) {

	uint32_t PWM = (Registers[PWMReg] * TIM1->ARR) / 100; // Calculate required CCRx value

	if (PWM >= TIM1->ARR - 60) {
		PWM = TIM1->ARR - 60;
	}

	TIM1->CR1 |= TIM_CR1_UDIS; 	// Disable Update Events
	TIM1->CCR1 = PWM;	  		// Set new PWM for channel 1
	TIM1->CCR2 = PWM;	  		// Set new PWM for channel 2
	TIM1->CCR3 = PWM;	  		// Set new PWM for channel 3
	TIM1->CR1 &= ~TIM_CR1_UDIS; // Enable Update Events

	return HAL_OK;

}
