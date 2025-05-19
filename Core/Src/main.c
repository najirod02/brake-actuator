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
#include "tim.h"
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

#define EXTEND GPIO_PIN_RESET // the actutor "extends" (cw rotation w.r.t. back of actuator)
#define RETRACT GPIO_PIN_SET // the actuator "retracts" (ccw rotation w.r.t. back of actuator)

#define MOTOR_GO GPIO_PIN_RESET // the driver is enabled, a step command will be accepted
#define MOTOR_STOP GPIO_PIN_SET // the driver is disabled, any step command will be discarded

#define DISTANCE 20 //mm - how much you want to move the actuator
#define MM_STEP (0.01) // mm/step actuator
#define ENC_MM_TICK (1.22e-4) // mm/tick encoder

#define PPR 65536
#define CPR 16384

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t msg[100] = {'\0'};
  int32_t enc_counter = 0;
  
  //starting timer for encoder reading
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  //starting timer for pwm
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  //setting up pins of the driver
  //at startup, disable it
  HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_STOP);
  HAL_GPIO_WritePin(ActuatorSleep_GPIO_Port, ActuatorSleep_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ActuatorReset_GPIO_Port, ActuatorReset_Pin, GPIO_PIN_SET);

  /**
   * given that the phase angle of the actuator is 1.8°. 200 steps are needed for a full rotation
   * with the given information of the encoder we can derive the relation tick/mm
   * 
   * total encoder counts per revolution = 16384
   * steps per revolution = 200
   * encoder counts per actuator step ~ 82 counts/steps (81.92)
   * distance per encoder count 0.122 µm/step
   * 
   */

  /**
   * 
   * the implementation set the pwm at 500Hz meaning that each second
   * we have 500 steps of the motor
   * 
   * Clock 84MHZ
   * Prescaler 167
   * AAR 999
   * Pulse 500
   */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // -- EXTEND --------------------------------------------------------

    HAL_GPIO_WritePin(ActuatorDir_GPIO_Port, ActuatorDir_Pin, EXTEND);
    HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_GO);

    // while(enc_counter * ENC_MM_TICK < DISTANCE){
    //   //no need to manually step the motor
    //   //just check the position using the encoder
    //   enc_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    // }

    HAL_Delay(2000);
    HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_STOP);


    enc_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    sprintf((char*)msg, "Encoder Ticks = %ld\tDistance = %.3f\n\r", enc_counter, enc_counter * ENC_MM_TICK);
    HAL_UART_Transmit(&huart2, (char*)msg, strlen(msg), 100);

    HAL_Delay(500);//wait some time before rotating in opposite direction

    // -- RETRACT --------------------------------------------------------

    HAL_GPIO_WritePin(ActuatorDir_GPIO_Port, ActuatorDir_Pin, RETRACT);
    HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_GO);

    // while(enc_counter * ENC_MM_TICK > 0){ // assuming that the position 0 is the initial one
    //   //no need to manually step the motor
    //   //just check the position using the encoder
    //   enc_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    // }

    HAL_Delay(2000);
    HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_STOP);

    enc_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    sprintf((char*)msg, "Encoder Ticks = %ld\tDistance = %.3f\n\r", enc_counter, enc_counter * ENC_MM_TICK);
    HAL_UART_Transmit(&huart2, (char*)msg, strlen(msg), 100);

    HAL_Delay(500);//wait some time before rotating in opposite direction
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
