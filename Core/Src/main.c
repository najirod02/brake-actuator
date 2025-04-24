/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOCKWISE GPIO_PIN_SET
#define COUNTERCLOCKWISE GPIO_PIN_RESET

#define STEPS 1000 //each step moves the shaft of 0.01mm meaning that 1000 steps = 1 cm
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// uint32_t rawCounter = 0; // value read from the board
// uint32_t lastRawCounter = 0; // last value read form the board
// int32_t counter = 0; // a signed value for the encoder
// uint32_t motorSteps = 0; // number of steps done by the motor
// bool isStepHigh = false; // to toggle step pin

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
//   if(htim->Instance == TIM2){
//     //callback for encoder
//     //with this logic we can have also negative values
//     rawCounter = __HAL_TIM_GET_COUNTER(htim);

//     if(rawCounter > lastRawCounter){
//       //clockwise rotation
//       counter += (rawCounter - lastRawCounter);
//     } else if (rawCounter < lastRawCounter){
//       //couterclockwise rotation
//       counter -= (lastRawCounter - rawCounter);
//     }

//     lastRawCounter = rawCounter;
//   }
//   else if(htim->Instance == TIM3){
//     //callback for step motor
//     //make a single step
//     HAL_GPIO_WritePin(GPIOC, ActuatorStep_Pin, isStepHigh ? GPIO_PIN_RESET : GPIO_PIN_SET);
//     isStepHigh = !isStepHigh;
//     // not a good solution but for testing is enough
//     //try using another timer of PWM for generating pulses
//     if (!isStepHigh) motorSteps++;  // count only when setting HIGH
//   }
//   else if(htim->Instance == TIM4){
//     //callback for pwm step motor
//     //the step has already been done we just increment the number of steps done
//     motorSteps++;
//   }
// }

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //setting up driver pins
  //enable, sleep and reset must be set to high in order to "activate" the driver
  HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ActuatorSleep_GPIO_Port, ActuatorSleep_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ActuatorReset_GPIO_Port, ActuatorReset_Pin, GPIO_PIN_SET);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
    for testing, move the actuator for a constant number of steps in both direction
    so that from an homing position we move of a certain distance to then go back to the homing position.

    when rotating counterclockwise, the actuator should extend the screw
    when rotating clockwise, the actuator should retract the screw
    */
    HAL_GPIO_WritePin(ActuatorDir_GPIO_Port, ActuatorDir_Pin, COUNTERCLOCKWISE);
    for(uint16_t i = 0; i<STEPS; i++){
      HAL_GPIO_WritePin(ActuatorStep_GPIO_Port, ActuatorStep_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(ActuatorStep_GPIO_Port, ActuatorStep_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
    }

    HAL_Delay(2000);//wait some time before rotating in opposite direction

    HAL_GPIO_WritePin(ActuatorDir_GPIO_Port, ActuatorDir_Pin, CLOCKWISE);
    for(uint16_t i = 0; i<STEPS; i++){
      HAL_GPIO_WritePin(ActuatorStep_GPIO_Port, ActuatorStep_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(ActuatorStep_GPIO_Port, ActuatorStep_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
    }

    HAL_Delay(2000);//wait some time before rotating in opposite direction

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
