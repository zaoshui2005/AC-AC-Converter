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
#include "adc.h"
#include "comp.h"
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

#include "pfc_adc.h"
#include "pfc_app.h"
#include "inverter_adc.h"
#include "inverter_adc_watchdog.h"
#include "inverter_control.h"
#include "inverter_svpwm.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Average_Update(float new_value)
{
  static uint64_t sample_count = 0U;
  static double average = 0.0;

  sample_count++;

  average += ((double)new_value - average) /
             (double)sample_count;

  return (float)average;
}
float GetVoltageRms(float voltage_v)
{
  static float sum = 0.0f;
  static float rms = 0.0f;
  static uint16_t count = 0U;

  sum += voltage_v * voltage_v;
  count++;

  if (count >= 400U) {
    rms = sqrtf(sum / 400.0f);
    sum = 0.0f;
    count = 0U;
  }

  return rms;
}
float input_voltage_rms_test;
float average_value_v = 0.0f;
float average_value_i = 0.0f;


/**
 * @brief 逆变输出频率切换状态
 */
typedef enum
{
    INVERTER_FREQ_SWITCH_IDLE = 0U,
    INVERTER_FREQ_SWITCH_WAIT_STOP
} Inverter_FrequencySwitchStateTypeDef;


/** 最近一次频率切换执行结果，供调试器观察。 */
volatile HAL_StatusTypeDef
    inverter_frequency_switch_status = HAL_OK;

/**
 * @brief          检测PA4按键是否产生一次有效按下
 * @note           PA4内部上拉，按键另一端接GND，低电平表示按下。
 * @retval         1 检测到一次新的按下
 * @retval         0 没有新的按下
 */
static uint8_t KEY_FrequencyToggle_IsPressed(void)
{
    static GPIO_PinState last_raw_state = GPIO_PIN_SET;
    static GPIO_PinState stable_state = GPIO_PIN_SET;
    static uint32_t debounce_start_tick = 0U;
    GPIO_PinState current_state;

    current_state = HAL_GPIO_ReadPin(
        KEY_FREQ_DOWN_GPIO_Port,
        KEY_FREQ_DOWN_Pin);

    if (current_state != last_raw_state) {
        last_raw_state = current_state;
        debounce_start_tick = HAL_GetTick();
    }

    if ((HAL_GetTick() - debounce_start_tick) >= 20U) {
        if (current_state != stable_state) {
            stable_state = current_state;

            if (stable_state == GPIO_PIN_RESET) {
                return 1U;
            }
        }
    }

    return 0U;
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
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_COMP3_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_COMP4_Init();
  MX_DAC2_Init();
  MX_DAC3_Init();
  MX_TIM1_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);

  /* 任何ADC采样和校准开始前，先强制关闭三相逆变六路功率输出。 */
  Inverter_SVPWM_Disable();

  if (Inverter_App_Init(pfc_adc_state.dma.sample) != HAL_OK) {
    Error_Handler();
  }

   /*
    * 只启动ADC1 DMA和HRTIM采样时基。
    * PFC控制器、PFC启动请求以及全部功率输出此时均未启用。
    */
   if (PFC_App_StartSamplingForCalibration() != HAL_OK) {
     (void)Inverter_ADC_Stop();
     Error_Handler();
   }

   /* 只在本次上电启动阶段校准v1、v2、a1、a2四路零点。 */
   if (Inverter_ADC_WaitForZeroCalibration(
           INVERTER_ADC_ZERO_CALIB_TIMEOUT_MS) != HAL_OK) {
     (void)PFC_ADC_Stop();
     (void)Inverter_ADC_Stop();
     Error_Handler();
   }

   /* 停止ADC2后写入a1、a2动态保护阈值，再恢复采样和看门狗。 */
   if (Inverter_ADC_Stop() != HAL_OK) {
     (void)PFC_ADC_Stop();
     Error_Handler();
   }

   if (Inverter_ADC_Watchdog_Init() != HAL_OK) {
     (void)PFC_ADC_Stop();
     Error_Handler();
   }

   if (Inverter_ADC_Start() != HAL_OK) {
     (void)PFC_ADC_Stop();
     Error_Handler();
   }

   /* 四路校准及逆变保护完成后，才正式初始化PFC。 */
   if (PFC_App_Init() != HAL_OK) {
     (void)PFC_ADC_Stop();
     (void)Inverter_ADC_Stop();
     Error_Handler();
   }


   if (Inverter_Control_Init() != HAL_OK) {
     (void)Inverter_ADC_Stop();
     Error_Handler();
   }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // average_value_i = Average_Update(inverter_adc_state.measurement.current_2_a );
    // average_value_v = Average_Update(input_voltage_rms_test);
    PFC_App_Loop();
      if (KEY_FrequencyToggle_IsPressed() != 0U) {
          Inverter_Control_RequestFrequencyToggle();
      }
    Inverter_Control_Service(pfc_adc_state.measurement.bus_voltage_v);
    // input_voltage_rms_test = GetVoltageRms(inverter_adc_state.measurement.voltage_2_v);
    average_value_v = Average_Update(inverter_control_state.line_voltage_ab_rms_v);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 80;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
