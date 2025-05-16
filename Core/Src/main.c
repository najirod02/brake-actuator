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
#include "brake_actuator_pid.h"
#include "pid.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EXTEND GPIO_PIN_SET // the actutor "extends" (cw rotation w.r.t. back of actuator)
#define RETRACT GPIO_PIN_RESET // the actuator "retracts" (ccw rotation w.r.t. back of actuator)

#define MOTOR_GO GPIO_PIN_RESET // the driver is enabled, a step command will be accepted
#define MOTOR_STOP GPIO_PIN_SET // the driver is disabled, any step command will be discarded

#define DISTANCE 40 //mm - target distance

#define MIN_ERR 5 //mm - just for testing, range for actuator disabling

#define KP_MAX 0.25

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
  uint8_t msg[200] = {'\0'};
  
  //setting up pins of the driver
  HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_GO);
  HAL_GPIO_WritePin(ActuatorSleep_GPIO_Port, ActuatorSleep_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ActuatorReset_GPIO_Port, ActuatorReset_Pin, GPIO_PIN_SET);
  
  //starting timer for encoder reading
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  
  //starting timer for pwm
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  //the pid will change the pwm "speed"
  //TODO: test pid controller, looking for potential jittering or overshoting
  //TODO: with negative distances, the speed will be negative?
  brake_actuator_pid_init(0.97 * KP_MAX, 0.1 * KP_MAX, 0.0, ENC_BRAKE_PERIOD_MS / 1000.0, 5.0);
  brake_actuator_update_set_point(DISTANCE);

  brake_actuator_enable();
  uint32_t last_update = HAL_GetTick();

  while (1)
  {
    /* USER CODE END WHILE */
    uint32_t now = HAL_GetTick();

    if (now - last_update >= ENC_BRAKE_PERIOD_MS) {
      last_update = now;
        
      uint32_t ticks = get_absolute_counter(&htim2);
      float distance = get_distance(ticks);
  
      sprintf((char*)msg, "Encoder Ticks = %ld\tDistance = %f\tARR = %ld\tCCR1 = %ld\n\r", ticks, distance, TIM3->ARR, TIM3->CCR1);
      HAL_UART_Transmit(&huart2, (char*)msg, strlen(msg), 100);
      
      if(fabs(distance - DISTANCE) <= MIN_ERR)
        brake_actuator_disable();
  
      brake_actuator_update_pid();
      brake_actuator_update_speed();
    }

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
