/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TICK_FREQ_D 72000000.

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile int last_dt = 0;
volatile int position = 0;
volatile uint8_t btn_is_pressed = 0, btn_was_pressed = 0;
double tunings[1][6] = {
  { 82.41, 110., 146.83, 196., 246.94, 329.63 } // E Major
};

typedef struct {
  uint32_t pause_ticks;
  uint32_t light_ticks;
} tick_delay;

volatile tick_delay diode_tick_delays[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// === Functions for tuning frequencies & delays

void setupDiodeTickDelays(double* tuning) {
  for (size_t i = 0; i < 6; i ++) {
    uint32_t total_ticks = (uint32_t) (TICK_FREQ_D / tuning[i]);
    diode_tick_delays[i].light_ticks = total_ticks / 10;
    diode_tick_delays[i].pause_ticks = total_ticks - diode_tick_delays[i].light_ticks;
  }
}

// === Functions for timers

volatile uint8_t tim2UpdateInterruptFlag = 0;
void tim2_32BitDelay(uint32_t delay_ticks) {
  uint16_t tim2_st = __HAL_TIM_GET_COUNTER(&htim2);
  uint16_t ticksUntilInterr = 0xFFFF - tim2_st;

  if (delay_ticks < ((uint32_t) ticksUntilInterr)) {
    while (((uint16_t) (__HAL_TIM_GET_COUNTER(&htim2) - tim2_st)) < delay_ticks) {}
    return;
  }

  delay_ticks -= ticksUntilInterr;
  tim2UpdateInterruptFlag = 0;
  while (!tim2UpdateInterruptFlag) {}

  while (delay_ticks > 0xFFFF) {
    delay_ticks -= 0xFFFF;
    tim2UpdateInterruptFlag = 0;
    while (!tim2UpdateInterruptFlag) {}
  }

  while (((uint16_t) __HAL_TIM_GET_COUNTER(&htim2)) < delay_ticks) {}
}

// === Interrupt Handlers

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ENC_BTN_Pin) {
    static uint32_t enc_btn_last_it_tick;
    if (HAL_GetTick() - enc_btn_last_it_tick < 50) return;
    enc_btn_last_it_tick = HAL_GetTick();
    btn_is_pressed = !btn_is_pressed;
    if (btn_is_pressed) btn_was_pressed = 1;
    HAL_GPIO_TogglePin(S1_GPIO_Port, S1_Pin);
  }
  if (GPIO_Pin == ENC_CLK_Pin) {
    static uint32_t enc_clk_last_it_tick;
    GPIO_PinState dt = HAL_GPIO_ReadPin(ENC_DT_GPIO_Port, ENC_DT_Pin);
    if (HAL_GetTick() - enc_clk_last_it_tick < 50) return;
    if (HAL_GetTick() - enc_clk_last_it_tick < 150 && dt != last_dt) return;
    enc_clk_last_it_tick = HAL_GetTick();
    position += (dt == GPIO_PIN_RESET) ? 1 : -1;
    last_dt = dt;
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, position & 1);
    HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, (position >> 1) & 1);
    HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, (position >> 2) & 1);
    HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, (position >> 3) & 1);
    HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, (position >> 4) & 1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    tim2UpdateInterruptFlag = 1;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // === setup timers

  HAL_TIM_Base_Start_IT(&htim2);

  // === setup all pins

  HAL_Delay(10);  // blink C13 for restart confirmation
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // reset all diodes
  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, GPIO_PIN_RESET);

  // === setup initial string frequencies

  setupDiodeTickDelays(tunings[0]);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(1) {__asm("nop");}

  uint32_t delays[6];
  uint32_t lighted[6] = {0, 0, 0, 0, 0, 0};
  for (uint8_t i = 0; i < 6; i++) {
    delays[i] = diode_tick_delays[i].pause_ticks;
  }

  uint32_t min;
  uint8_t minind;
  uint16_t pin_num;
  uint32_t delay_dt = __HAL_TIM_GET_COUNTER(&htim2);
  while(1) {
    min = delays[0]; minind = 0;
    for (uint8_t i = 1; i < 6; i++) {
      if (delays[i] < min) {
        min = delays[i];
        minind = i;
      }
    }

    if (minind == 0) pin_num = GPIO_PIN_8;
    else if (minind == 1) pin_num = GPIO_PIN_9;
    else if (minind == 2) pin_num = GPIO_PIN_10;
    else if (minind == 3) pin_num = GPIO_PIN_11;
    else if (minind == 4) pin_num = GPIO_PIN_12;
    else pin_num = GPIO_PIN_15;

    delay_dt = __HAL_TIM_GET_COUNTER(&htim2) - delay_dt;
    if (delay_dt < min) tim2_32BitDelay(min - delay_dt);
    delay_dt = __HAL_TIM_GET_COUNTER(&htim2);

    for (uint8_t i = 0; i < 6; i++) {
      delays[i] -= min;
    }

    if (lighted[minind]) {
      HAL_GPIO_WritePin(GPIOA, pin_num, GPIO_PIN_RESET);
      delays[minind] = diode_tick_delays[minind].pause_ticks;
    } else {
      HAL_GPIO_WritePin(GPIOA, pin_num, GPIO_PIN_SET);
      delays[minind] = diode_tick_delays[minind].light_ticks;
    }

    lighted[minind] = !lighted[minind];

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
