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
#include <stdio.h>
#include <string.h>
#include "CV.h"
#include "laser.h"
#include <math.h>
#include <float.h>
#include <sys/stat.h>

#include "util.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONFIG_FLASH_ADDR 0x0807F800
#define CV_MODEL_FLASH_ADDR 0x0807F000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include "fit.h"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
typedef enum {
    APP_STATE_NORMAL,
    APP_STATE_ACQUIRING,
    APP_STATE_FITTING
} app_state_t;
app_state_t app_state = APP_STATE_NORMAL;

float cv_capture[CV_LUT_SIZE];
#if CV_CAPTURE_MODE == CV_CAPTURE_MODE_AVG
uint32_t cv_count_capture[CV_LUT_SIZE];
#endif
laser_calibration_t calibration = { DEFAULT_CV_PARAMS, CW_CALIBRATION_DEFAULT };
float fit_final_mse = 0.0f;
uint32_t fit_epochs_run = 0;
float fit_r_squared = 0.0f;

// cw threshold floors captured during the sweep (min cw once ml mean passes each ref).
float cal_cw_low = FLT_MAX;
float cal_cw_sat = FLT_MAX;

#ifndef BUF_SIZE
#define BUF_SIZE 2000
#endif

#define ENABLE_PROFILING 1   // DWT cycle profiling

#ifdef ENABLE_PROFILING
volatile uint32_t process_laser_logic_cycles = 0;
#endif

/* Hardware & DMA Buffers */
uint16_t adc1_val[BUF_SIZE];
uint16_t adc2_val;

sliding_cv_t sliding_cv = {0};
float cv_threshold_lut[CV_LUT_SIZE] = {0};

/* ISR flags (global & volatile) */
volatile uint8_t adc_flag = 0;
volatile uint16_t adc_overrun = 0;
volatile uint16_t uart_underrun = 0;
volatile uint8_t uart_flag = 0;

laser_state_t current_laser_state = {0};
laser_frame_t laser_frame = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint16_t uart_rx_flag = 0;
laser_config_t rx_config_buffer;
laser_config_t current_config = LASER_CONFIG_DEFAULT;  // Defaults

// DMA landing buffer for inbound config frames; oversized so a burst with a stray
// leading byte is still captured whole and can be re-synchronized below.
#define UART_RX_BUF_SIZE 32
uint8_t uart_rx_buf[UART_RX_BUF_SIZE];

// (Re)arm idle-terminated DMA reception (restarts at index 0).
static void uart_rx_arm(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, UART_RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);  // only act on the idle/complete event
}

void laser_frame_send(laser_frame_t* frame, sliding_cv_t* scv, laser_state_t* state) {
  frame->frame_start = 0xAA;
  frame->ml_rms_mv = state->ml_rms_mv;
  frame->cw_mv = state->cw_mv;
  frame->current_cv = scv->current_cv;
  frame->current_cv_threshold = state->current_cv_threshold;
  frame->status = state->status;
  frame->frame_end = 0xBB;
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)frame, sizeof(laser_frame_t));
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc == &hadc1) {
    if (adc_flag != 0) {
      adc_overrun += 1;
    }
    adc_flag = 1;
  }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc == &hadc1) {
    if (adc_flag != 0) {
      adc_overrun += 1;
    }
    adc_flag = 2;
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    uart_flag = 1;
  }
}

// Fires on idle line or DMA completion. Scan for a 0xAA..0xBB frame to re-sync on
// the start byte so a stray leading byte can't permanently break framing.
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart1)
  {
    for (uint16_t i = 0; i + sizeof(laser_config_t) <= Size; i++) {
      if (uart_rx_buf[i] == 0xAA &&
          uart_rx_buf[i + sizeof(laser_config_t) - 1] == 0xBB) {
        memcpy(&rx_config_buffer, &uart_rx_buf[i], sizeof(laser_config_t));
        uart_rx_flag = 1;
        break;
      }
    }
    uart_rx_arm();  // always re-arm
  }
}

// Recover reception after any UART error, else a single error stops config intake.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                   UART_CLEAR_FEF  | UART_CLEAR_PEF);
    uart_rx_arm();
  }
}
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
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
#ifdef ENABLE_PROFILING
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif

  // Load calibration from Flash, falling back to defaults if empty/invalid.
  Flash_Read_Data(CV_MODEL_FLASH_ADDR, (uint32_t*)&calibration, sizeof(laser_calibration_t) / 4);

  uint32_t* p_a = (uint32_t*)&calibration.cv.a;
  if (*p_a == 0xFFFFFFFF) {
      calibration.cv = DEFAULT_CV_PARAMS;
      calibration.cw = CW_CALIBRATION_DEFAULT;
  }
  // !(x > 0) also catches NaN from erased flash.
  if (!(calibration.cw.cw_saturation > 0.0f) || !(calibration.cw.cw_threshold_low > 0.0f)) {
      calibration.cw = CW_CALIBRATION_DEFAULT;
  }

  generate_cv_threshold_lut(cv_threshold_lut, &calibration.cv);

  Flash_Read_Data(CONFIG_FLASH_ADDR, (uint32_t*)&current_config, sizeof(laser_config_t) / 4);
  if (current_config.frame_start != 0xAA || current_config.frame_end != 0xBB) {
    current_config = LASER_CONFIG_DEFAULT;
  }

  uart_rx_arm();
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_val, BUF_SIZE);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc2_val, 1);
  uart_flag = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (uart_rx_flag) {
      uart_rx_flag = 0;

      // Persist config to flash only when it actually changed (blocking, ~20-40ms).
      if (memcmp((void*)&current_config, (void*)&rx_config_buffer, sizeof(laser_config_t)) != 0) {
        memcpy((void*)&current_config, (void*)&rx_config_buffer, sizeof(laser_config_t));
        uint16_t word_count = (sizeof(laser_config_t) + 3) / 4;
        Flash_Write_Data(CONFIG_FLASH_ADDR, (uint32_t*)&current_config, word_count);
      }
    }

    // --- Button state machine: hold to sweep/acquire, release to fit ---
    if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET) {
        if (app_state == APP_STATE_NORMAL) {
            app_state = APP_STATE_ACQUIRING;
            for (int i = 0; i < CV_LUT_SIZE; i++) {
                cv_capture[i] = -1.0f;  // -1 = empty bin
            }
            cal_cw_low = FLT_MAX;
            cal_cw_sat = FLT_MAX;
        }
    } else {
        if (app_state == APP_STATE_ACQUIRING) {
            app_state = APP_STATE_FITTING;
        }
    }

    if (app_state == APP_STATE_FITTING) {
        cv_model_params_t optimized_params = DEFAULT_CV_PARAMS;

        fit_cv_curve(cv_capture, &optimized_params, &fit_final_mse, &fit_epochs_run, &fit_r_squared);

        calibration.cv = optimized_params;

        // Only commit cw thresholds for reference levels reached during the sweep.
        if (cal_cw_low < FLT_MAX)  calibration.cw.cw_threshold_low = cal_cw_low;
        if (cal_cw_sat < FLT_MAX)  calibration.cw.cw_saturation   = cal_cw_sat;

        uint16_t cal_word_count = (sizeof(laser_calibration_t) + 3) / 4;
        Flash_Write_Data(CV_MODEL_FLASH_ADDR, (uint32_t*)&calibration, cal_word_count);

        generate_cv_threshold_lut(cv_threshold_lut, &calibration.cv);
        app_state = APP_STATE_NORMAL;
    }

    uint8_t flag = adc_flag;
    if (flag == 1 || flag == 2) {
      uint16_t* buf_ptr = (flag == 1) ? adc1_val : (adc1_val + BUF_SIZE / 2);

#ifdef ENABLE_PROFILING
      uint32_t start_cycles = DWT->CYCCNT;
#endif

      laser_config_t active_config = current_config;
      if (app_state == APP_STATE_ACQUIRING) {
          active_config.target_window_ms = 1;   // acquisition override
      }

      process_laser_logic(buf_ptr, BUF_SIZE/2, &sliding_cv, cv_threshold_lut, &active_config, &calibration.cw, adc2_val, &current_laser_state);

      if (app_state == APP_STATE_ACQUIRING) {
          float mean_mv = sliding_cv.current_mean;
          float ml_sat  = ML_THRESHOLD_HIGH_MV * current_config.saturation_percent;

          // Skip saturated samples: clipped ADC distorts variance and the LUT is
          // never consulted above ml_sat anyway.
          if (mean_mv < ml_sat) {
              float norm = mean_mv / ADC_VREF_MV;
              if (norm < 0.0f) norm = 0.0f;
              int idx = (int)roundf(norm * (float)(CV_LUT_SIZE - 1));
              if (idx >= 0 && idx < CV_LUT_SIZE) {
                  int bin_size  = CV_LUT_SIZE / CV_CAPTURE_RESOLUTION;
                  int mapped_idx = (idx / bin_size) * bin_size;
                  if (mapped_idx >= CV_LUT_SIZE) mapped_idx = CV_LUT_SIZE - 1;
                  if (cv_capture[mapped_idx] < 0.0f || sliding_cv.current_cv < cv_capture[mapped_idx])
                      cv_capture[mapped_idx] = sliding_cv.current_cv;
              }
          }

          // cw threshold floors (min cw once ml mean passes each ref); skip dropouts.
          float cw = current_laser_state.cw_mv;
          if (cw > 0.0f) {
              if (mean_mv >= CW_THRESHOLD_LOW_MV && cw < cal_cw_low) cal_cw_low = cw;
              if (mean_mv >= CW_THRESHOLD_HIGH_MV && cw < cal_cw_sat) cal_cw_sat = cw;
          }
      }

#ifdef ENABLE_PROFILING
      process_laser_logic_cycles = DWT->CYCCNT - start_cycles;
      process_laser_logic_cycles /= 120;
#endif

      if (adc_flag == flag) {
        adc_flag = 0;
      }

      if (uart_flag == 1) {
        uart_flag = 0;
        static uint8_t config_sync_counter = 0;
        config_sync_counter++;
        if (config_sync_counter >= 50) {
          config_sync_counter = 0;
          HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&current_config, sizeof(laser_config_t));
        } else {
          laser_frame_send(&laser_frame, &sliding_cv, &current_laser_state);
        }
      }
      else {
        uart_underrun++;
      }
    }
    // LED UI Logic
    uint32_t current_tick = HAL_GetTick();
    static uint32_t last_led_tick = 0;
    static uint8_t red_led_state = 0;

    switch (current_laser_state.status) {
        case LOW_SIGNAL: {
            // Red briefly pulses, yellow off
            uint32_t period_ms = 250;
            uint32_t on_time_ms = 10;
            uint32_t time_in_period = (current_tick - last_led_tick) % period_ms;
            if (time_in_period < on_time_ms) {
                if (red_led_state == 0) {
                    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                    red_led_state = 1;
                }
            } else {
                if (red_led_state == 1) {
                    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                    red_led_state = 0;
                }
            }
            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
            break;
        }

        case SATURATED:
            // Red solid, yellow off
            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            red_led_state = 1;
            break;

        case CW:
            // Both off
            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            red_led_state = 0;
            break;

        case MODE_LOCKED:
            // Yellow solid, red off
            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            red_led_state = 0;
            break;

        case UNSTABLE:
            // Both off
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
            break;
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
  sConfig.Channel = ADC_CHANNEL_15;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_7;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A175AB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_RED_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

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
