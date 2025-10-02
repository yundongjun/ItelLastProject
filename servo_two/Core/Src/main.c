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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 제어 모드 */
typedef enum { MODE_IDLE = 0, MODE_GRID, MODE_TRACK } control_mode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 서보 펄스폭(us) 보정: 필요하면 미세 조정 */
#define SERVO1_MIN_US   1000   /* Pan  0°  */
#define SERVO1_MAX_US   2000  /* Pan  180° */
#define SERVO2_MIN_US   1000   /* Tilt 0°  */
#define SERVO2_MAX_US   2000  /* Tilt 180° */

/* 방향이 반대면 1로 바꿔서 반전 */
#define SERVO1_INVERT   1
#define SERVO2_INVERT   1

/* 기본 자세(미검출일 때) */
#define PAN_DEFAULT_DEG    90   /* 정면 */
#define TILT_DEFAULT_DEG    90
//#define PAN_MIN_DEG    45
//#define PAN_MAX_DEG   135
//#define TILT_MIN_DEG   70
//#define TILT_MAX_DEG  110

/* 스무딩 속도(한 틱당 각도 변화) & 제어주기 */
#define STEP_DEG            3.0f
#define CONTROL_PERIOD_MS  20u

/* 추적 신호 타임아웃(미검출 판정) */
#define DETECT_TIMEOUT_MS 1000u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CLAMP_U16(v,lo,hi) ((uint16_t)((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v))))
static inline uint8_t clamp_deg_int(int v){ return (uint8_t)(v<0?0:(v>180?180:v)); }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* UART 수신 라인 버퍼 */
static volatile uint8_t rx_byte;
static char  rx_line[64];
static uint8_t rx_idx = 0;

/* 제어 상태 */
static control_mode_t mode = MODE_IDLE;
static float pan_cur  = PAN_DEFAULT_DEG, pan_tgt  = PAN_DEFAULT_DEG;
static float tilt_cur = TILT_DEFAULT_DEG, tilt_tgt = TILT_DEFAULT_DEG;

/* 마지막 추적(T) 명령이 온 시각 */
static uint32_t last_detect_ms = 0;

/* 9분할 프리셋 (좌/중/우, 상/중/하) — 필요시 값 튜닝 */
static const uint8_t GRID_PAN[3]  = { 30,  90, 150 };  /* 좌 중 우 */
static const uint8_t GRID_TILT[3] = {120,  90,  60 };  /* 상 중 하 (기구에 맞게 조절) */
//static uint8_t boost_ticks = 0;   // 킥을 줄 틱 수(20ms 단위)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t angle_to_us(uint8_t deg, uint16_t us_min, uint16_t us_max, uint8_t invert);
static void     servo1_set_deg(uint8_t deg);
static void     servo2_set_deg(uint8_t deg);
static void     move_toward(float *cur, float tgt, float step_deg);
static void     apply_default_pose(void);
static void     set_grid_cell(uint8_t cell_1to9);
static void     set_track_xy(float x, float y);
static void     handle_line(char *line);
//static inline float clampf(float v, float lo, float hi);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t angle_to_us(uint8_t deg, uint16_t us_min, uint16_t us_max, uint8_t invert)
{
  if (deg > 180) deg = 180;
  uint8_t d = invert ? (uint8_t)(180 - deg) : deg;
  int us = us_min + (int)((us_max - us_min) * (float)d / 180.0f + 0.5f);
  return CLAMP_U16(us, us_min, us_max);
}

static void servo1_set_deg(uint8_t deg)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
    angle_to_us(deg, SERVO1_MIN_US, SERVO1_MAX_US, SERVO1_INVERT));
}

static void servo2_set_deg(uint8_t deg)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
    angle_to_us(deg, SERVO2_MIN_US, SERVO2_MAX_US, SERVO2_INVERT));
}

static void move_toward(float *cur, float tgt, float step_deg)
{
  if (*cur < tgt) { *cur += step_deg; if (*cur > tgt) *cur = tgt; }
  else if (*cur > tgt) { *cur -= step_deg; if (*cur < tgt) *cur = tgt; }
}

static void apply_default_pose(void)
{
  mode = MODE_IDLE;
  pan_tgt  = PAN_DEFAULT_DEG;
  tilt_tgt = TILT_DEFAULT_DEG;
}

/* 1..9: 좌상(1) ~ 우하(9) */
static void set_grid_cell(uint8_t cell_1to9)
{
  if (cell_1to9 < 1 || cell_1to9 > 9) return;
  mode = MODE_GRID;
  uint8_t idx = (uint8_t)(cell_1to9 - 1);
  uint8_t row = idx / 3;   /* 0=상,1=중,2=하 */
  uint8_t col = idx % 3;   /* 0=좌,1=중,2=우 */
  pan_tgt  = GRID_PAN[col];
  tilt_tgt = GRID_TILT[row];
//  pan_tgt  = clampf(pan_tgt,  PAN_MIN_DEG,  PAN_MAX_DEG);
//  tilt_tgt = clampf(tilt_tgt, TILT_MIN_DEG, TILT_MAX_DEG);
//  boost_ticks = 15; // 15*20ms = 300ms 동안 가속
}

/* 정규화 좌표 기반 추적 (x:0~1 좌→우, y:0~1 상→하) */
static void set_track_xy(float x, float y)
{
  if (x < 0) x = 0; if (x > 1) x = 1;
  if (y < 0) y = 0; if (y > 1) y = 1;
  mode = MODE_TRACK;
  pan_tgt  = 180.0f * x;
  tilt_tgt = 180.0f * y;
  //pan_tgt  = clampf(pan_tgt,  PAN_MIN_DEG,  PAN_MAX_DEG);
  //tilt_tgt = clampf(tilt_tgt, TILT_MIN_DEG, TILT_MAX_DEG);
  last_detect_ms = HAL_GetTick();
}

/* 한 줄 명령: "N" / "C 5" / "T 0.45 0.62" */
static void handle_line(char *line)
{
  char *cmd = strtok(line, " \t\r\n");
  if (!cmd) return;

  if (strcmp(cmd, "N") == 0) {
    apply_default_pose();
  } else if (strcmp(cmd, "C") == 0) {
    char *sidx = strtok(NULL, " \t\r\n");
    if (sidx) set_grid_cell((uint8_t)atoi(sidx));
  } else if (strcmp(cmd, "T") == 0) {
    char *sx = strtok(NULL, " \t\r\n");
    char *sy = strtok(NULL, " \t\r\n");
    if (sx && sy) set_track_xy(strtof(sx, NULL), strtof(sy, NULL));
  }
}

/* UART 인터럽트: '\n' 기준으로 라인 수신 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    uint8_t c = rx_byte;

    if (c == '\n' || rx_idx >= sizeof(rx_line) - 1) {
      rx_line[rx_idx] = '\0';
      handle_line(rx_line);            // ← 여기서 N/C/T 명령 처리
      rx_idx = 0;

      // 개행(한 줄 완성) 시에만 OK 에코
      const char ok[] = "OK\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)ok, sizeof(ok)-1, 10);
    } else if (c != '\r') {
      rx_line[rx_idx++] = (char)c;
    }

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // 다음 글자 수신 재개
  }
}

//static inline float clampf(float v, float lo, float hi){
//  return v < lo ? lo : (v > hi ? hi : v);
//}
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
  /* USER CODE BEGIN 2 */
  /* PWM 시작 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); /* servo1: Pan */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); /* servo2: Tilt */
  // ===== Servo self-test (임시) =====
  servo1_set_deg(90);  servo2_set_deg(90);  HAL_Delay(800);
  servo1_set_deg(60);  servo2_set_deg(60);  HAL_Delay(800);
  servo1_set_deg(120); servo2_set_deg(120); HAL_Delay(800);
  // ===== end self-test =====

  /* 기본 자세 적용 */
  servo1_set_deg(PAN_DEFAULT_DEG);
  servo2_set_deg(TILT_DEFAULT_DEG);

  /* UART 라인 수신 시작 */
  const char *hello = "READY\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)hello, strlen(hello), 100);
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  uint32_t t_prev = HAL_GetTick();
  last_detect_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 20 ms 주기로 스무딩 적용 */
    uint32_t now = HAL_GetTick();
    if (now - t_prev >= CONTROL_PERIOD_MS) {
      t_prev = now;

      /* 추적 신호 타임아웃 → 기본 자세 */
      if (mode == MODE_TRACK && (now - last_detect_ms) > DETECT_TIMEOUT_MS) {
        apply_default_pose();
      }

      /* 각도 범위 제한 & 스무딩 이동 */
      pan_tgt  = clamp_deg_int((int)lroundf(pan_tgt));
      tilt_tgt = clamp_deg_int((int)lroundf(tilt_tgt));
      move_toward(&pan_cur,  pan_tgt,  STEP_DEG);
      move_toward(&tilt_cur, tilt_tgt, STEP_DEG);
      //float step_now = (boost_ticks > 0) ? 6.0f : STEP_DEG;
      //move_toward(&pan_cur,  pan_tgt,  step_now);
      //move_toward(&tilt_cur, tilt_tgt, step_now);
      //if (boost_ticks > 0) boost_ticks--;

      servo1_set_deg((uint8_t)lroundf(pan_cur));
      servo2_set_deg((uint8_t)lroundf(tilt_cur));
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // UART polling fallback (non-blocking)
    static char pline[64]; static uint8_t pidx=0; uint8_t ch;
    while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) {
      if (ch == '\n' || pidx >= sizeof(pline)-1) {
        pline[pidx] = '\0'; handle_line(pline); pidx = 0;
        const char ok[]="OK\r\n"; HAL_UART_Transmit(&huart2,(uint8_t*)ok,sizeof(ok)-1,10);
      } else if (ch != '\r') pline[pidx++] = (char)ch;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
