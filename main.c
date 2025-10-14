/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
typedef enum { MODE_IDLE = 0, MODE_GRID, MODE_TRACK } control_mode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO1_MIN_US   1000
#define SERVO1_MAX_US   2000
#define SERVO2_MIN_US   1000
#define SERVO2_MAX_US   2000

#define SERVO1_INVERT   1
#define SERVO2_INVERT   1

#define PAN_DEFAULT_DEG   90
#define TILT_DEFAULT_DEG  90

#define STEP_DEG            3.0f
#define CONTROL_PERIOD_MS  20u
#define DETECT_TIMEOUT_MS 1000u

/* Water spray: Active-Low on PB7 (펄스 토글형 제어 가정) */
#define WATER_ON()   HAL_GPIO_WritePin(water_GPIO_Port, water_Pin, GPIO_PIN_RESET) /* Low  */
#define WATER_OFF()  HAL_GPIO_WritePin(water_GPIO_Port, water_Pin, GPIO_PIN_SET)   /* High */

/* 요구사항: 3초 자동 OFF */
#define SPRAY_DURATION_MS   3000U
/* 토글 모듈용 펄스 폭(낮은 레벨 유지 시간) */
#define SPRAY_PULSE_MS        50U
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
static volatile uint8_t rx_byte;
static char  rx_line[64];
static uint8_t rx_idx = 0;

static control_mode_t mode = MODE_IDLE;
static float pan_cur  = PAN_DEFAULT_DEG, pan_tgt  = PAN_DEFAULT_DEG;
static float tilt_cur = TILT_DEFAULT_DEG, tilt_tgt = TILT_DEFAULT_DEG;

static uint32_t last_detect_ms = 0;

static const uint8_t GRID_PAN[3]  = { 30,  90, 150 };
static const uint8_t GRID_TILT[3] = {120,  90,  60 };

/* ===== SPRAY 상태 (토글형 펄스 제어) =====
   spray_active : 모듈이 현재 ON(1)/OFF(0) 상태라고 "우리가" 추적
   spray_pulsing: 지금 토글 펄스를 출력 중(LOW 유지 중)
   spray_off_tick: 자동 OFF(토글 펄스 내보낼) 시각
   spray_pulse_release_tick: 펄스 해제 시각(LOW→HIGH로 복귀) */
static volatile uint8_t  spray_active = 0;
static volatile uint8_t  spray_pulsing = 0;
static uint32_t          spray_off_tick = 0;
static uint32_t          spray_pulse_release_tick = 0;
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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_log(const char *s){
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 10);
}

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

static void set_grid_cell(uint8_t cell_1to9)
{
  if (cell_1to9 < 1 || cell_1to9 > 9) return;
  mode = MODE_GRID;
  uint8_t idx = (uint8_t)(cell_1to9 - 1);
  uint8_t row = idx / 3;
  uint8_t col = idx % 3;
  pan_tgt  = GRID_PAN[col];
  tilt_tgt = GRID_TILT[row];
}

static void set_track_xy(float x, float y)
{
  if (x < 0) x = 0; if (x > 1) x = 1;
  if (y < 0) y = 0; if (y > 1) y = 1;
  mode = MODE_TRACK;
  pan_tgt  = 180.0f * x;
  tilt_tgt = 180.0f * y;
  last_detect_ms = HAL_GetTick();
}

/* "N" / "C 5" / "T 0.45 0.62" / "SPRAY" / "STOP" */
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

  /* ===== SPRAY: 토글형 모듈에 낮은 펄스 50ms를 보내 ON, 3초 후 자동으로 또 펄스 보내 OFF ===== */
  } else if (strcmp(cmd, "SPRAY") == 0) {
    uint32_t now = HAL_GetTick();

    /* 이미 ON이면 타이머만 연장, 펄스 중이면 그대로 둠 */
    if (!spray_active && !spray_pulsing) {
      /* ON 토글 펄스 시작 (LOW 유지) */
      WATER_ON();
      spray_pulsing = 1;
      spray_pulse_release_tick = now + SPRAY_PULSE_MS;
      spray_active = 1;
      uart_log("SPRAY_ON\r\n");
    }
    /* 자동 OFF 시각(3초 뒤) 갱신 */
    spray_off_tick = now + SPRAY_DURATION_MS;

  } else if (strcmp(cmd, "STOP") == 0) {
    /* 강제 정지: 즉시 OFF 토글 펄스 수행 */
    if (spray_active && !spray_pulsing) {
      uint32_t now = HAL_GetTick();
      WATER_ON();
      spray_pulsing = 1;
      spray_pulse_release_tick = now + SPRAY_PULSE_MS;
      spray_active = 0;
      uart_log("SPRAY_FORCE_OFF\r\n");
    }
  }
}

/* CR 또는 LF를 줄끝으로 처리 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    uint8_t c = rx_byte;

    if (c == '\r' || c == '\n' || rx_idx >= sizeof(rx_line) - 1) {
      if (rx_idx > 0) {
        rx_line[rx_idx] = '\0';
        handle_line(rx_line);
        rx_idx = 0;
        const char ok[] = "OK\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)ok, sizeof(ok)-1, 10);
      }
    } else {
      rx_line[rx_idx++] = (char)c;
    }
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  servo1_set_deg(90);  servo2_set_deg(90);  HAL_Delay(300);
  servo1_set_deg(60);  servo2_set_deg(60);  HAL_Delay(300);
  servo1_set_deg(120); servo2_set_deg(120); HAL_Delay(300);

  servo1_set_deg(PAN_DEFAULT_DEG);
  servo2_set_deg(TILT_DEFAULT_DEG);

  WATER_OFF(); /* 유휴시는 High */
  const char *hello = "READY\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)hello, strlen(hello), 100);
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  uint32_t t_prev = HAL_GetTick();
  last_detect_ms = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    /* === SPRAY 펄스 관리 및 자동 OFF === */
    uint32_t now = HAL_GetTick();

    /* 펄스 해제: LOW 유지 시간을 넘기면 High로 복귀 */
    if (spray_pulsing && (int32_t)(now - spray_pulse_release_tick) >= 0) {
      WATER_OFF();              /* 펄스 종료: 라인 High로 */
      spray_pulsing = 0;
      /* uart_log("PULSE_RELEASE\r\n"); // 필요하면 사용 */
    }

    /* 자동 OFF: ON 상태이고 펄스 중이 아니며, 3초가 지나면 OFF 토글 펄스 수행 */
    if (spray_active && !spray_pulsing && (int32_t)(now - spray_off_tick) >= 0) {
      WATER_ON();                               /* OFF를 위해 또 한 번 LOW 펄스 */
      spray_pulsing = 1;
      spray_pulse_release_tick = now + SPRAY_PULSE_MS;
      spray_active = 0;                         /* 논리상 OFF로 전이 */
      uart_log("SPRAY_OFF\r\n");
    }

    /* === 서보 스무딩/타임아웃은 20ms 주기 === */
    if (now - t_prev >= CONTROL_PERIOD_MS) {
      t_prev = now;

      if (mode == MODE_TRACK && (now - last_detect_ms) > DETECT_TIMEOUT_MS) {
        apply_default_pose();
      }

      pan_tgt  = clamp_deg_int((int)lroundf(pan_tgt));
      tilt_tgt = clamp_deg_int((int)lroundf(tilt_tgt));
      move_toward(&pan_cur,  pan_tgt,  STEP_DEG);
      move_toward(&tilt_cur, tilt_tgt, STEP_DEG);

      servo1_set_deg((uint8_t)lroundf(pan_cur));
      servo2_set_deg((uint8_t)lroundf(tilt_cur));
    }

    /* === UART polling fallback (선택) === */
    static char pline[64]; static uint8_t pidx=0; uint8_t ch;
    while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) {
      if (ch == '\r' || ch == '\n' || pidx >= sizeof(pline)-1) {
        if (pidx > 0) {
          pline[pidx] = '\0'; handle_line(pline); pidx = 0;
          const char ok[]="OK\r\n"; HAL_UART_Transmit(&huart2,(uint8_t*)ok,sizeof(ok)-1,10);
        }
      } else {
        pline[pidx++] = (char)ch;
      }
    }
  }
}

/* SystemClock, TIM2, USART2, GPIO 초기화부는 그대로 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = HAL_RCC_GetPCLK2Freq()?RCC_HCLK_DIV1:RCC_HCLK_DIV1; /* 그대로 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(water_GPIO_Port, water_Pin, GPIO_PIN_SET); /* default High (idle) */

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = water_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(water_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
