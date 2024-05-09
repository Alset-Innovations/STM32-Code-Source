/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f4xx_hal_rcc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Commutation table for motor control
// Hex values are for CCER, CCMR2 and CCMR1 in this order.
// CCER enables output while CCMRx sets PWM mode, forced active or forced inactive.
/*
uint16_t Commutation[6][3] = {
		{0x0405, 0x0858, 0x4868}, // 010 Phase 3 Low,  Phase 2 Off,  Phase 1 High
		{0x0504, 0x0868, 0x4858}, // 101 Phase 3 High, Phase 2 Off,  Phase 1 Low
		{0x0450, 0x0858, 0x6848}, // 011 Phase 3 Low,  Phase 2 High, Phase 1 Off
		{0x0054, 0x0848, 0x6858}, // 001 Phase 3 Off,  Phase 2 High, Phase 1 Low
		{0x0045, 0x0848, 0x5868}, // 110 Phase 3 Off,  Phase 2 Low,  Phase 1 High
		{0x0540, 0x0868, 0x5848}  // 100 Phase 3 High, Phase 2 Low,  Phase 1 Off
};
*/

/*
uint16_t Commutation[6][3] = {
		{0x0540, 0x0868, 0x5848}, // 100 Phase 3 High, Phase 2 Low,  Phase 1 Off
		{0x0054, 0x0848, 0x6858}, // 001 Phase 3 Off,  Phase 2 High, Phase 1 Low
		{0x0504, 0x0868, 0x4858}, // 101 Phase 3 High, Phase 2 Off,  Phase 1 Low
		{0x0405, 0x0858, 0x4868}, // 010 Phase 3 Low,  Phase 2 Off,  Phase 1 High
		{0x0045, 0x0848, 0x5868}, // 110 Phase 3 Off,  Phase 2 Low,  Phase 1 High
		{0x0450, 0x0858, 0x6848}, // 011 Phase 3 Low,  Phase 2 High, Phase 1 Off
};
*/

/*
uint16_t Commutation[6][3] = {
		{0x0504, 0x0868, 0x4858}, // 101 Phase 3 High, Phase 2 Off,  Phase 1 Low
		{0x0540, 0x0868, 0x5848}, // 100 Phase 3 High, Phase 2 Low,  Phase 1 Off
		{0x0405, 0x0858, 0x4868}, // 010 Phase 3 Low,  Phase 2 Off,  Phase 1 High
		{0x0054, 0x0848, 0x6858}, // 001 Phase 3 Off,  Phase 2 High, Phase 1 Low
		{0x0450, 0x0858, 0x6848}, // 011 Phase 3 Low,  Phase 2 High, Phase 1 Off
		{0x0045, 0x0848, 0x5868}, // 110 Phase 3 Off,  Phase 2 Low,  Phase 1 High
};
*/

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

// Storing data in a register
uint16_t Registers[RegSize] = {0, 0, 0, 0}; // PWM, Direction, Current, RPM
int StartReg = 0;
int NumReg = 0;
int EndReg = 0;

// I2C communication
uint8_t TxCount = 0;
uint8_t RxCount = 0;
uint8_t RxData[RxSize];

uint32_t Fapb1tclk = 0;
uint32_t Fapb2tclk = 0;
uint32_t RPMConst = 0;

uint32_t RPMValue = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



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

void ProcessData (void) {

	StartReg = RxData[0]; 			// Start address of registers to be written
	NumReg = RxCount - 1; 			// Number of registers to be written
	EndReg = StartReg + NumReg - 1; // Last register to be written

	// If the last register to be wriiten is larger than the size of the register call the error handler
	if (EndReg > RxSize) {
		Error_Handler();
	}

	// Write data into the register using a for loop
	for (int i = 1; i < NumReg + 1; i++) {
		Registers[StartReg++] = RxData[i + 1];
	}

	// If the PWM is higher than 0 but the motor is not turning then startup
	if ( Registers[PWMReg] > 0 && Registers[RPMReg] == 0) {
		StartupSequence(Registers[DirReg]);
	}

	// If the PWM is 0 but the motor is still turning shutdown
	if ( Registers[PWMReg] == 0 && Registers[RPMReg] > 0 ) {
		StopSequence();
	}

	// Call some functions
	ChangePWM(); 				// Update PWM values
	memset(RxData, 0, RxSize); 	// Empty the RxData array

}

extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c) {

	HAL_I2C_EnableListen_IT (hi2c);

}

extern void HAL_I2C_AddrCallback (I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {

	if ( TransferDirection == I2C_DIRECTION_TRANSMIT ) { // If the master wants to transmit the data

		RxCount = 0;
		HAL_I2C_Slave_Sequential_Receive_IT (hi2c, RxData + RxCount, 1, I2C_FIRST_FRAME);

	} else { // If the master wants tot recieve data

		TxCount = 0;
		StartReg = RxData[0];
		HAL_I2C_Slave_Seq_Transmit_IT (hi2c, (uint8_t *) (Registers[TxCount + StartReg] >> 8), 1, I2C_FIRST_FRAME);
		HAL_I2C_Slave_Seq_Transmit_IT (hi2c, (uint8_t *) (Registers[TxCount + StartReg] & 0xFF), 1, I2C_NEXT_FRAME);

	}
}

void HAL_I2C_SlaveTxCpltCallback (I2C_HandleTypeDef *hi2c) {

	TxCount++;
	HAL_I2C_Slave_Seq_Transmit_IT (hi2c, (uint8_t *) (Registers[TxCount + StartReg] >> 8), 1, I2C_NEXT_FRAME);
	HAL_I2C_Slave_Seq_Transmit_IT (hi2c, (uint8_t *) (Registers[TxCount + StartReg] & 0xFF), 1, I2C_NEXT_FRAME);

}

void HAL_I2C_SlaveRxCpltCallback (I2C_HandleTypeDef *hi2c) {

	RxCount++;

	if ( RxCount < RxSize ) {

		if (RxCount == RxSize - 1) {
			HAL_I2C_Slave_Sequential_Receive_IT (hi2c, RxData + RxCount, 1, I2C_LAST_FRAME);
		} else {
			HAL_I2C_Slave_Sequential_Receive_IT (hi2c, RxData + RxCount, 1, I2C_NEXT_FRAME);
		}
	}

	if ( RxCount == RxSize) {
		ProcessData();
	}

}

void HAL_I2C_ErrorCallback (I2C_HandleTypeDef *hi2c) {

	if ( HAL_I2C_GetError (hi2c) == 4) {

		__HAL_I2C_CLEAR_FLAG (hi2c, I2C_FLAG_AF); 	// Clear AF flag

		if ( TxCount == 0) { 						// Error while recieving
			ProcessData();
		} else { 									// Error while transmitting
			TxCount--;
		}

	}

	HAL_I2C_EnableListen_IT(hi2c);
}

// extern uint8_t CDC_Transmit_FS (uint8_t* buf, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	HAL_I2C_EnableListen_IT (&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	/*
	// Read Potentiometer data from ADC for RPM control.
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t PWM = HAL_ADC_GetValue(&hadc1) / 2;
	*/

	//ChangePWM();

	/*
	// Transmit RPM value to PC via USB
	len = snprintf(buf, sizeof(buf), "\n\rCurrent PWM: %4d", PWM);
	CDC_Transmit_FS ((uint8_t *) buf, len);

	HAL_Delay(10);
	*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
