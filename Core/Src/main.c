/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
#ifndef BUF_SIZE
#define BUF_SIZE 2000
#endif
enum laser_status{
  NOT_MODE_LOCKED,
  Q_SWITCHING,
  MODE_LOCKED
};
enum cv_status{
  UNSTABLE,
  STABLE
};
typedef struct {
  float threshold_cv;
  uint16_t stability_window;
}laser_par_t;

typedef struct {
  uint16_t stability_counter;
  float current_cv;
  enum laser_status laser_status;
  enum cv_status cv_status;
}laser_state_t;

uint16_t ADCval[BUF_SIZE];
laser_state_t laser_state = {0,0.0f, NOT_MODE_LOCKED,UNSTABLE};
laser_par_t laser_par = {1.0f, 100};
volatile uint8_t ADC_flag = 0;
volatile uint8_t ADC_overrun = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void calcCV(uint16_t* buf, int size, laser_par_t* par, laser_state_t* state) {
  if (size > 1) {
    uint32_t sum = 0;
    for (int i = 0; i < size; i++ ) {
      sum += buf[i];
    }
    if (sum != 0) {
      float avg = 0.0f;
      float standard_deviation = 0.0f;
      avg = (float)sum / (float)size;
      for (int i = 0; i < size; i++ ) {
        standard_deviation += ((float)buf[i] - avg) * ((float)buf[i] - avg);
      }
      standard_deviation = sqrtf(standard_deviation / (float)(size - 1));
      state->current_cv = (standard_deviation / avg) * 100.0f;
      if (state->current_cv < par->threshold_cv) {
        state->cv_status = STABLE;
        if (state->laser_status == NOT_MODE_LOCKED || state->laser_status == Q_SWITCHING) {
          state->laser_status = Q_SWITCHING;
          state->stability_counter++;
        }
      }
      else {
        state->cv_status = UNSTABLE;
        state->laser_status = NOT_MODE_LOCKED;
        state->stability_counter = 0;
      }
      if (state->stability_counter >= par->stability_window) {
        state->laser_status = MODE_LOCKED;
      }
    }
    else {
      state->current_cv = 0.0f;
      state->cv_status = UNSTABLE;
      state->laser_status = NOT_MODE_LOCKED;
      state->stability_counter = 0;
    }
  }
  else {
        state->current_cv = 0.0f;
        state->cv_status = UNSTABLE;
        state->laser_status = NOT_MODE_LOCKED;
        state->stability_counter = 0;
  }
}
// void calcCV(uint16_t* buf, int size, laser_par_t* par, laser_state_t* state) {
//   if (size > 1) {
//     float sum_power = 0.0f;
//
//     // Step 1: Calculate sum of powers (Power is proportional to Voltage^2)
//     for (int i = 0; i < size; i++) {
//       float voltage = (float)buf[i];
//       float power = voltage * voltage;
//       sum_power += power;
//     }
//
//     if (sum_power > 0.0f) {
//       float avg_power = sum_power / (float)size;
//       float variance_power = 0.0f;
//
//       // Step 2: Calculate variance of the power
//       for (int i = 0; i < size; i++) {
//         float voltage = (float)buf[i];
//         float power = voltage * voltage;
//         // Summing the squared differences from the mean
//         variance_power += (power - avg_power) * (power - avg_power);
//       }
//
//       // Calculate standard deviation
//       float std_dev_power = sqrtf(variance_power / (float)(size - 1));
//
//       // Step 3: Calculate true Power CV as a percentage
//       state->current_cv = (std_dev_power / avg_power) * 100.0f;
//
//       // State machine logic (preserved exactly as you wrote it)
//       if (state->current_cv < par->threshold_cv) {
//         state->cv_status = STABLE;
//         if (state->laser_status == NOT_MODE_LOCKED || state->laser_status == Q_SWITCHING) {
//           state->laser_status = Q_SWITCHING;
//           state->stability_counter++;
//         }
//       }
//       else {
//         state->cv_status = UNSTABLE;
//         state->laser_status = NOT_MODE_LOCKED;
//         state->stability_counter = 0;
//       }
//
//       if (state->stability_counter >= par->stability_window) {
//         state->laser_status = MODE_LOCKED;
//       }
//     }
//   }
// }
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if (ADC_flag != 0) {
    ADC_overrun += 1;
  }
  ADC_flag = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (ADC_flag != 0) {
    ADC_overrun += 1;
  }
  ADC_flag = 2;
}
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart == &huart1)
//   {
//     TxCount++;
//     HAL_UART_Transmit_IT(&huart1, (uint8_t*)buf, buf_len);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  //HAL_OPAMP_Start(&hopamp1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCval, BUF_SIZE);

  // HAL_UART_Transmit_IT(&huart1, (uint8_t*)buf, buf_len);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch (ADC_flag) {
      case 1:
        calcCV(ADCval, BUF_SIZE/2, &laser_par, &laser_state);
        ADC_flag = 0;
        break;
      case 2:
        calcCV(ADCval + BUF_SIZE/2, BUF_SIZE/2, &laser_par, &laser_state);
        ADC_flag = 0;
        break;
      default:
        break;
    }
    if (laser_state.laser_status == MODE_LOCKED) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    if (laser_state.cv_status == STABLE) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    }
    else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    if (ADCval[0] >= 4080) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    }
    else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
