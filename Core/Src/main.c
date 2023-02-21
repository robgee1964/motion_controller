/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include "motion.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_FREQ						3U		// Hz
#define LED_DUTY						20U		// percentage
#define BGLOOP_RATE                     20U
#define ACCEL_STEPS                     10U
#define ACCEL_K                         220U    // Fraction of 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static struct
{
  uint32_t on_ticks;
  uint32_t off_ticks;
  uint32_t next_change;
  GPIO_PinState state;
} sled =  { 0 };

static struct
{
volatile  bool running;
  uint32_t n_pulse;
  uint32_t psc;
  uint32_t ccr1;
} spwm = {0};

static uint32_t a_psc[ACCEL_STEPS] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static void set_led_rate(uint32_t freq, uint32_t dr);
static void bg_proc(void);
static void motion_test(void);
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


  set_led_rate(LED_FREQ, LED_DUTY);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

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
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
  spwm.psc = htim1.Init.Prescaler;
  spwm.ccr1 = htim1.Instance->CCR1;
  HAL_GPIO_WritePin(pin_debug_GPIO_Port, pin_debug_Pin, GPIO_PIN_RESET);
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);



  // Prepare accel steps
  uint32_t psc = spwm.psc;
  for (uint32_t i = 0; i < ACCEL_STEPS ; i++)
  {
    psc = (psc * ACCEL_K) >> 8;
    a_psc[i] = psc;
  }

  motion_test();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t bg_tick;
  bg_tick = HAL_GetTick() + BGLOOP_RATE;
  while (1)
  {
    if (HAL_GetTick() >= bg_tick)
    {
      bg_tick += BGLOOP_RATE;
      bg_proc();
    }

    if (HAL_GetTick() >= sled.next_change)
    {
      if (sled.state == GPIO_PIN_SET)
      {
        sled.state = GPIO_PIN_RESET;
        sled.next_change += sled.on_ticks;
      }
      else
      {
        sled.state = GPIO_PIN_SET;
        sled.next_change += sled.off_ticks;
      }
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, sled.state);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, pin_debug_Pin|USB_DISC_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : pin_debug_Pin USB_DISC_Pin LED_Pin */
  GPIO_InitStruct.Pin = pin_debug_Pin|USB_DISC_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  // TODO: Don't use the HAL as it combines the **individual** timer IRQs into
  // unified callback, as a result I have to check the timer instance
  if (htim->Instance == TIM3)
  {
//    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
    htim1.Instance->CCER &= ~TIM_CCER_CC1E;
    htim1.Instance->CR1 &= ~(TIM_CR1_CEN);
    htim3.Instance->CNT = 0;
    htim1.Instance->CNT = 0;
    htim1.Instance->CCR1 = spwm.ccr1;
    htim1.Instance->PSC = spwm.psc;
    htim1.Instance->EGR |= TIM_EGR_UG;
    set_led_rate(LED_FREQ*2, 50);
    HAL_GPIO_WritePin(pin_debug_GPIO_Port, pin_debug_Pin, GPIO_PIN_RESET);
    spwm.running = false;
    htim1.Init.Prescaler = spwm.psc;
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  // See comment in OC delay callback
  if (htim->Instance == TIM1)
  {
    //HAL_GPIO_TogglePin(pin_debug_GPIO_Port, pin_debug_Pin);
    uint32_t psc_new = (htim->Init.Prescaler * 7) >> 3;
    htim->Instance->PSC = psc_new;
    htim->Init.Prescaler = psc_new;
  }
}

static void set_led_rate(uint32_t freq, uint32_t dr)
{
  uint32_t period_ticks = uwTickFreq * 1000U / freq;
  sled.on_ticks = (period_ticks * dr) / 100;
  sled.off_ticks = (period_ticks * (100 - dr) / 100);
  sled.next_change = HAL_GetTick() + sled.off_ticks;
}

static void bg_proc(void)
{
  // Check for button press
  static GPIO_PinState btn_prev = GPIO_PIN_RESET;
  static uint8_t buff[30];
  GPIO_PinState btn_state = HAL_GPIO_ReadPin(button_GPIO_Port, button_Pin);
  bool btn_press = false;
  if (btn_state != btn_prev)
  {
    if (btn_prev == GPIO_PIN_RESET)
    {
      btn_press = true;
    }
    btn_prev = btn_state;
  }

  if (!spwm.running)
  {
    if (btn_press)
    {
      sprintf((void*)buff, "Starting pulses\r\n");
      (void)CDC_Transmit_FS(buff, strlen((void*)buff));
      htim3.Instance->CNT = 0;
      HAL_GPIO_WritePin(pin_debug_GPIO_Port, pin_debug_Pin, GPIO_PIN_SET);
      HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
      htim1.Init.Prescaler = spwm.psc;
      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_CC1);
//      HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, a_psc, ACCEL_STEPS);
      HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
      spwm.running = true;
    }
  }

}

#define NUM_STEPS       600
#define ACCEL           2000
#define MAX_SPEED       500

static void motion_test(void)
{
  HAL_GPIO_WritePin(pin_debug_GPIO_Port, pin_debug_Pin, GPIO_PIN_SET);
  uint32_t *p_motion = motion_plan_fp(NUM_STEPS, ACCEL, MAX_SPEED);
  HAL_GPIO_WritePin(pin_debug_GPIO_Port, pin_debug_Pin, GPIO_PIN_RESET);
  p_motion = motion_plan_fp(5, ACCEL, MAX_SPEED);
  p_motion = motion_plan_fp(10, ACCEL, MAX_SPEED);

  for (uint32_t i = 0; i < NUM_STEPS; i++)
  {
    printf("step %lu, speed %lu\n\r", i, p_motion[i]);
  }

  motion_complete();
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
