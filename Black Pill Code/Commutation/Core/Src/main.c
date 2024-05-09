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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

// Hex values are for CCER, CCMR2 and CCMR1 in this order.
// CCER enables output while CCMRx sets PWM mode, forced active or forced inactive.
uint16_t Commutation[6][3] = {
		{0x0C01, 0x0868, 0x4868}, // 010 Phase 3 Low,  Phase 2 Off,  Phase 1 High
		{0x010C, 0x0868, 0x4868}, // 101 Phase 3 High, Phase 2 Off,  Phase 1 Low
		{0x0C10, 0x0868, 0x6848}, // 011 Phase 3 Low,  Phase 2 High, Phase 1 Off
		{0x001C, 0x0848, 0x6868}, // 001 Phase 3 Off,  Phase 2 High, Phase 1 Low
		{0x00C1, 0x0848, 0x6868}, // 110 Phase 3 Off,  Phase 2 Low,  Phase 1 High
		{0x01C0, 0x0868, 0x6848}  // 100 Phase 3 High, Phase 2 Low,  Phase 1 Off
};

uint16_t len = 0;
uint16_t TargetRPM = 0;
uint16_t PWM = 1000;
uint16_t count = 0;

uint8_t RxData[RxSize];
uint8_t Mode = 1;

double CurrentRPM = 0;
char buf[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
	{
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData, 6, I2C_FIRST_AND_LAST_FRAME);
		PWM = RxData[0] * 12;
	}
	else  // master requesting the data is not supported yet
	{
		Error_Handler();
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	count++;
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

HAL_StatusTypeDef StartupSequence(char Direction);
HAL_StatusTypeDef StopSequence();
HAL_StatusTypeDef PrepareCommutation(char Direction);

extern uint8_t CDC_Transmit_FS(uint8_t* buf, uint16_t len);

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	StartupSequence('F');

	// I2C1->CR1 |= 0x0003;

	/*
	// Registers for XOR function (Done in IOC)
	TIM4->ARR = 0xFFFFFFFF; // Set ARR to max value (32 bits)
	TIM4->PSC = 0xFFFF; 	// Set PSC to max value (16 bits)
	TIM4->CR2 |= 0x00D0; 	// Turn on XOR function.
	TIM4->CCMR1 &= 0xFCFF; 	// Can also be done in the ioc.
	TIM4->CCMR1 |= 0x7003; 	// Can also be done in the ioc.
	*/

	// Dead-time can be controlled using DTG[7:0] in TIM1->BDTR register.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	/*
	// Read Hall sensor for new PWM calculation
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	TargetRPM = HAL_ADC_GetValue(&hadc1);

	// Keep minimum RPM
	if ( TargetRPM < MinimumRPM ) {
		TargetRPM = MinimumRPM;
	*/

	TIM1->CCR1 = PWM;	  // Set new PWM for channel 1
	TIM1->CCR2 = PWM;	  // Set new PWM for channel 2
	TIM1->CCR3 = PWM;	  // Set new PWM for channel 3

	// Transmit RPM value to PC via USB
	len = snprintf(buf, sizeof(buf), "\n\rCurrent RPM: %04.2lf", CurrentRPM);
	CDC_Transmit_FS((uint8_t *) buf, len);

	// HAL_Delay(100);

	// Update RPM
	// Disable UDIS in CR1
	// Change DC
	// Enable UDIS in CR1

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

HAL_StatusTypeDef StartupSequence(char Direction) {

	// Enable I2C in Interrupt mode
	HAL_I2C_EnableListen_IT(&hi2c1);

	// Set first commutation state according to Hall sensors
	if (PrepareCommutation(Direction) == HAL_ERROR) {
		return HAL_ERROR;
	}

	// Start HallSensor timer
	HAL_TIMEx_HallSensor_Start(&htim2);

	// Start all PWM signals on TIM1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	// Disable all interrupts
	TIM1->DIER &= ~TIM_DIER_COMIE;	// Disable Commutation events in DIER register
	TIM1->DIER &= ~TIM_DIER_BIE; 	// Disable break interrupt as this is shared with timer 9 interrupt
	TIM2->DIER &= ~TIM_DIER_TIE; 	// Disable interrupt on timer 2
	TIM9->DIER &= ~TIM_DIER_TIE; 	// Disable interrupt on timer 9

	// Start Interrupts
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim9);

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
	TIM1->CR2  |= 0x0005; 			// Set CCPC 1 and CCUS 1 in CR2
	TIM1->EGR  |= TIM_EGR_COMG; 	// Set COMG bit in EGR for first commutation
	TIM1->DIER |= TIM_DIER_COMIE; 	// Enable commutation events in DIER register
	// TIM1->BDTR |= TIM_BDTR_OSSR;

	return HAL_OK;

}

HAL_StatusTypeDef StopSequence() {

	// Stop HallSensor timer
	HAL_TIMEx_HallSensor_Stop(&htim2);

	// Disable all output channels
	TIM1->CCER  = 0x0000;
	TIM1->CCMR1 = 0x0808;
	TIM1->CCMR2 = 0x0808;

	// Disable commutation
	TIM1->EGR |= TIM_EGR_COMG; 				// Trigger one last commutation event
	while (((TIM1->SR >> 5) & 0x1) == 1); 	// Wait until commutation event has happened
	TIM1->DIER &= ~TIM_DIER_COMIE; 			// Disable commutation events in DIER register

	// Stop interrupts
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim9);

	// Stop PWM Timers
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

	return HAL_OK;

}

HAL_StatusTypeDef PrepareCommutation(char Direction) {

	// Read IDR for Hall Sensor status
	uint8_t Hall = (GPIOA->IDR & 0x0007) - 1;

	// Edit Hall data according to direction.
	switch (Direction) {
	case 'F':
		Hall += 1; // Select next value in the array to go forward
		Hall %= 6; // If original was 5 it needs to be 0 to we use % 6
	break;
	case 'B':
		Hall += 6; // To not go negative in the next step we add 6
		Hall -= 1; // Select previous value to go backwards
		Hall %= 6; // If original was 0 it needs to become 5, this also negates the 6 we added previously
	break;
	default:
		return HAL_ERROR; // If F or B is not supplied the function should return with an error
	break;
	}

	// Set Registers to required values
	TIM1->CCER  = Commutation[Hall][0];
	TIM1->CCMR1 = Commutation[Hall][2];
	TIM1->CCMR2 = Commutation[Hall][1];

	return HAL_OK;

}

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
