/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (stable polling version)
  ******************************************************************************
  * @attention
  * Copyright (c) 2025.
  * All rights reserved.
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

#define STEP_DEG              3.0f
#define CONTROL_PERIOD_MS    20u
#define DETECT_TIMEOUT_MS  1000u

/* Water spray: Active-Low on PB7 (pulse toggle-type assumed) */
#define WATER_ON()   HAL_GPIO_WritePin(water_GPIO_Port, water_Pin, GPIO_PIN_RESET) /* Low  */
#define WATER_OFF()  HAL_GPIO_WritePin(water_GPIO_Port, water_Pin, GPIO_PIN_SET)   /* High */

/* Spray timing */
#define SPRAY_DURATION_MS   3000U
#define SPRAY_PULSE_MS         50U

/* ====== Stepper 핀 매핑 ====== */
#define STEPPER_STEP_PORT   GPIOA
#define STEPPER_STEP_PIN    GPIO_PIN_6   /* 일반 GPIO 토글 */
#define STEPPER_DIR_PORT    GPIOB
#define STEPPER_DIR_PIN     GPIO_PIN_0
#define STEPPER_EN_PORT     GPIOB
#define STEPPER_EN_PIN      GPIO_PIN_1

/* EN 극성: 0=High가 Enable, 1=Low가 Enable */
#define STEPPER_EN_ACTIVE_LOW  0
/* DIR 반전: 1이면 논리 UP/DOWN을 물리적으로 뒤집음 */
#define STEPPER_DIR_INVERT     1

/* TIM3: 64MHz, PSC=63 → 1MHz 타이머 베이스 */
#define TIM3_BASE_HZ        1000000UL
/* 고정 스텝 주파수 (토글이라 실제 STEP은 f/2) */
#define STEPPER_FIXED_FREQ_HZ  1000.0f   /* 업데이트 1kHz → STEP 500Hz */

/* UART1에서 동작 유지 시간(자동 정지) */
#define FORK_RUN_MS             2000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CLAMP_U16(v,lo,hi) ((uint16_t)((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v))))
static inline uint8_t clamp_deg_int(int v){ return (uint8_t)(v<0?0:(v>180?180:v)); }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1; /* RPi (fork) */
UART_HandleTypeDef huart2; /* Jetson (servo/spray) */

/* USER CODE BEGIN PV */
/* === USART2 (servo/spray) 수신 더블버퍼 === */
static uint8_t  rx2_byte;
static char     rx2_line_work[64];
static uint8_t  rx2_idx = 0;
static volatile uint8_t rx2_line_ready = 0;
static char     rx2_line_ready_buf[64];

/* === USART1 (fork) 수신 더블버퍼 === */
static uint8_t  rx1_byte;
static char     rx1_line_work[16];
static uint8_t  rx1_idx = 0;
static volatile uint8_t rx1_line_ready = 0;
static char     rx1_line_ready_buf[16];

static control_mode_t mode = MODE_IDLE;
static float pan_cur  = PAN_DEFAULT_DEG, pan_tgt  = PAN_DEFAULT_DEG;
static float tilt_cur = TILT_DEFAULT_DEG, tilt_tgt = TILT_DEFAULT_DEG;
static uint32_t last_detect_ms = 0;

static const uint8_t GRID_PAN[3]  = { 30,  90, 150 };
static const uint8_t GRID_TILT[3] = {120,  90,  60 };

/* ===== SPRAY 상태 ===== */
static volatile uint8_t  spray_active = 0;
static volatile uint8_t  spray_pulsing = 0;
static uint32_t          spray_off_tick = 0;
static uint32_t          spray_pulse_release_tick = 0;

/* ===== STEP 모터 상태 ===== */
static volatile uint8_t  fork_running = 0;      /* 1=동작 중(연속 스텝) */
static volatile uint8_t  fork_dir_up  = 1;      /* 1=UP, 0=DOWN */
static uint32_t fork_block_until_ms = 0;        /* DIR/EN 바꿈 직후 STEP 토글 지연 */
static uint32_t fork_auto_off_tick  = 0;        /* 자동 정지 시각 */
static int8_t   fork_last_cmd_up    = -1;       /* 최근 명령: 1=UP, 0=DOWN, -1=없음 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t angle_to_us(uint8_t deg, uint16_t us_min, uint16_t us_max, uint8_t invert);
static void     servo1_set_deg(uint8_t deg);
static void     servo2_set_deg(uint8_t deg);
static void     move_toward(float *cur, float tgt, float step_deg);
static void     apply_default_pose(void);
static void     set_grid_cell(uint8_t cell_1to9);
static void     set_track_xy(float x, float y);

/* 파서 */
static void     handle_line_uart2(char *line);  /* servo/spray (USART2) */
static void     handle_line_uart1(char *line);  /* fork (USART1) */

/* Stepper helpers */
static void     stepper_enable(uint8_t en);
static void     stepper_set_dir(uint8_t up);
static void     stepper_set_fixed_freq(float f_hz);
static void     stepper_start_up_with_timer(void);
static void     stepper_start_down_with_timer(void);
static void     stepper_stop(void);

/* 로그 */
static void     uart2_log(const char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart2_log(const char *s){ HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 10); }

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
  if (x < 0) x = 0; else if (x > 1) x = 1;
  if (y < 0) y = 0; else if (y > 1) y = 1;
  mode = MODE_TRACK;
  pan_tgt  = 180.0f * x;
  tilt_tgt = 180.0f * y;
  last_detect_ms = HAL_GetTick();
}

/* ===== Stepper low-level ===== */
static void stepper_enable(uint8_t en)
{
#if STEPPER_EN_ACTIVE_LOW
  HAL_GPIO_WritePin(STEPPER_EN_PORT, STEPPER_EN_PIN, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(STEPPER_EN_PORT, STEPPER_EN_PIN, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static void stepper_set_dir(uint8_t up)
{
  uint8_t logical_up = up ? 1 : 0;
#if STEPPER_DIR_INVERT
  logical_up ^= 1;  /* 방향 뒤집기 */
#endif
  HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN,
                    logical_up ? GPIO_PIN_SET : GPIO_PIN_RESET);
  fork_dir_up = up ? 1 : 0;
}

static void stepper_set_fixed_freq(float f_hz)
{
  if (f_hz < 1.0f) f_hz = 1.0f;
  /* TIM3 업데이트 주파수 = 1e6/(ARR+1), 스텝은 토글이므로 f_step = f_up/2 */
  uint32_t arr = (uint32_t)((TIM3_BASE_HZ / (2.0f * f_hz)) - 1.0f);
  if (arr < 1) arr = 1;
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
}

static void stepper_start_up_with_timer(void)
{
  stepper_set_fixed_freq(STEPPER_FIXED_FREQ_HZ);
  stepper_set_dir(1);
  stepper_enable(1);
  fork_running = 1;
  uint32_t now = HAL_GetTick();
  fork_block_until_ms = now + 1;              /* DIR/EN 세팅 후 최소 1ms 대기 */
  fork_auto_off_tick  = now + FORK_RUN_MS;    /* 2초 후 자동 정지 */
  fork_last_cmd_up    = 1;
  uart2_log("FORK_UP\r\n");
}

static void stepper_start_down_with_timer(void)
{
  stepper_set_fixed_freq(STEPPER_FIXED_FREQ_HZ);
  stepper_set_dir(0);
  stepper_enable(1);
  fork_running = 1;
  uint32_t now = HAL_GetTick();
  fork_block_until_ms = now + 1;
  fork_auto_off_tick  = now + FORK_RUN_MS;    /* 2초 후 자동 정지 */
  fork_last_cmd_up    = 0;
  uart2_log("FORK_DOWN\r\n");
}

static void stepper_stop(void)
{
  fork_running = 0;
  stepper_enable(0);
  HAL_GPIO_WritePin(STEPPER_STEP_PORT, STEPPER_STEP_PIN, GPIO_PIN_RESET); /* STEP Low 유지 */
  uart2_log("FORK_STOP\r\n");
}

/* ===== 파서: USART2 (servo/spray) ===== */
static void handle_line_uart2(char *line)
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

  } else if (strcmp(cmd, "SPRAY") == 0) {
    /* 토글-펄스형: LOW 50ms 한 번 → 모듈 ON, 3초 뒤 LOW 50ms 한 번 더 → OFF */
    uint32_t now = HAL_GetTick();

    /* 이미 ON이면 타이머만 연장. 펄스 중이면 그대로 둠 */
    if (!spray_active && !spray_pulsing) {
      /* ON 토글 펄스 시작 (LOW 유지) */
      WATER_ON();
      spray_pulsing = 1;
      spray_pulse_release_tick = now + SPRAY_PULSE_MS;
      spray_active = 1;                      /* 논리 상태: ON */
      uart2_log("SPRAY_ON\r\n");
    }
    /* 자동 OFF 시각(3초 뒤) 갱신 */
    spray_off_tick = now + SPRAY_DURATION_MS;

  } else if (strcmp(cmd, "STOP") == 0) {
    /* 스프레이 강제 정지: 현재 ON이고 펄스 중이 아니면 즉시 OFF 토글 펄스 */
    if (spray_active && !spray_pulsing) {
      uint32_t now = HAL_GetTick();
      WATER_ON();                             /* OFF 토글 펄스 시작(LOW) */
      spray_pulsing = 1;
      spray_pulse_release_tick = now + SPRAY_PULSE_MS;
      spray_active = 0;                       /* 논리 상태: OFF */
      uart2_log("SPRAY_FORCE_OFF\r\n");
    }
    /* 포크도 정지 (수동) */
    fork_auto_off_tick = 0;
    stepper_stop();
  }
}

/* ===== 파서: USART1 (fork) ===== */
static void handle_line_uart1(char *line)
{
  if (!line || !*line) return;

  if (strcmp(line, "UP") == 0) {
    stepper_start_up_with_timer();
    const char ack[] = "ACK:UP\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)ack, sizeof(ack)-1, 10);
  } else if (strcmp(line, "DOWN") == 0) {
    stepper_start_down_with_timer();
    const char ack[] = "ACK:DOWN\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)ack, sizeof(ack)-1, 10);
  }
}

/* 공용 UART Rx Complete 콜백: 라인 버퍼링만 수행 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    uint8_t c = rx2_byte;

    if (c == '\r' || c == '\n') {
      if (rx2_idx > 0 && !rx2_line_ready) {
        rx2_line_work[rx2_idx] = '\0';
        strncpy(rx2_line_ready_buf, rx2_line_work, sizeof(rx2_line_ready_buf)-1);
        rx2_line_ready_buf[sizeof(rx2_line_ready_buf)-1] = '\0';
        rx2_line_ready = 1;
        rx2_idx = 0;
      }
    } else {
      if (rx2_idx < sizeof(rx2_line_work)-1) {
        rx2_line_work[rx2_idx++] = (char)c;
      } else { rx2_idx = 0; } /* overflow 시 라인 드롭 */
    }

    HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
  }
  else if (huart->Instance == USART1) {
    uint8_t c = rx1_byte;

    if (c == '\r' || c == '\n') {
      if (rx1_idx > 0 && !rx1_line_ready) {
        rx1_line_work[rx1_idx] = '\0';
        strncpy(rx1_line_ready_buf, rx1_line_work, sizeof(rx1_line_ready_buf)-1);
        rx1_line_ready_buf[sizeof(rx1_line_ready_buf)-1] = '\0';
        rx1_line_ready = 1;
        rx1_idx = 0;
      }
    } else {
      if (rx1_idx < sizeof(rx1_line_work)-1) {
        rx1_line_work[rx1_idx++] = (char)c;
      } else { rx1_idx = 0; }
    }

    HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();  /* Jetson (servo/spray) */
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();  /* RPi (fork) */

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* 스텝 토글 타이머 시작: 폴링 방식 (인터럽트 X) */
  stepper_set_fixed_freq(STEPPER_FIXED_FREQ_HZ);
  HAL_TIM_Base_Start(&htim3);

  /* 초기 포즈 시퀀스 */
  servo1_set_deg(90);  servo2_set_deg(90);  HAL_Delay(300);
  servo1_set_deg(60);  servo2_set_deg(60);  HAL_Delay(300);
  servo1_set_deg(120); servo2_set_deg(120); HAL_Delay(300);
  servo1_set_deg(PAN_DEFAULT_DEG);
  servo2_set_deg(TILT_DEFAULT_DEG);

  WATER_OFF();                   /* 유휴시는 High */
  stepper_enable(0);             /* 포크 EN 비활성으로 시작 */

  /* 수신 인터럽트 시작 */
  HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
  HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);

  const char *hello1 = "HELLO1\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)hello1, (uint16_t)strlen(hello1), 100);

  const char *hello2 = "READY\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)hello2, (uint16_t)strlen(hello2), 100);

  uint32_t t_ctrl = HAL_GetTick();
  last_detect_ms = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* === TIM3 업데이트 플래그 폴링으로 STEP 토글 === */
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
      __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

      if (fork_running && (int32_t)(now - fork_block_until_ms) >= 0) {
        HAL_GPIO_TogglePin(STEPPER_STEP_PORT, STEPPER_STEP_PIN);
      } else {
        HAL_GPIO_WritePin(STEPPER_STEP_PORT, STEPPER_STEP_PIN, GPIO_PIN_RESET);
      }
    }

    /* === 2초 자동 정지 & STOP1/STOP2 통지(USART1) === */
    if (fork_running && fork_auto_off_tick &&
        (int32_t)(now - fork_auto_off_tick) >= 0) {
      stepper_stop();
      if (fork_last_cmd_up == 1) {
        const char s[]="STOP1\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)s, sizeof(s)-1, 10);
      } else if (fork_last_cmd_up == 0) {
        const char s[]="STOP2\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)s, sizeof(s)-1, 10);
      }
      fork_last_cmd_up = -1;
      fork_auto_off_tick = 0;
    }

    /* ===== 주기 제어(20ms) : 서보 추종 + 스프레이 상태 ===== */
    if ((int32_t)(now - t_ctrl) >= (int32_t)CONTROL_PERIOD_MS) {
      t_ctrl += CONTROL_PERIOD_MS;

      /* 서보 보간 */
      move_toward(&pan_cur,  pan_tgt,  STEP_DEG);
      move_toward(&tilt_cur, tilt_tgt, STEP_DEG);
      servo1_set_deg((uint8_t)clamp_deg_int((int)lroundf(pan_cur)));
      servo2_set_deg((uint8_t)clamp_deg_int((int)lroundf(tilt_cur)));

      /* 스프레이: 펄스 해제 & 자동 OFF(토글 펄스) */
      if (spray_pulsing && now >= spray_pulse_release_tick) {
        WATER_OFF();                  /* 펄스 종료: 라인 High 복귀 */
        spray_pulsing = 0;
      }

      if (spray_active && !spray_pulsing && now >= spray_off_tick) {
        /* 3초 만료 → OFF 토글 펄스 한 번 더 */
        WATER_ON();                   /* LOW 펄스 시작 */
        spray_pulsing = 1;
        spray_pulse_release_tick = now + SPRAY_PULSE_MS;
        spray_active = 0;
        uart2_log("SPRAY_OFF\r\n");
      }
    }

    /* ===== 라인 처리(USART2) ===== */
    if (rx2_line_ready) {
      rx2_line_ready = 0;
      char line2[64];
      strncpy(line2, rx2_line_ready_buf, sizeof(line2)-1);
      line2[sizeof(line2)-1] = '\0';
      handle_line_uart2(line2);

      const char ok[] = "OK\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)ok, sizeof(ok)-1, 10);
    }

    /* ===== 라인 처리(USART1) ===== */
    if (rx1_line_ready) {
      rx1_line_ready = 0;
      char line1[16];
      strncpy(line1, rx1_line_ready_buf, sizeof(line1)-1);
      line1[sizeof(line1)-1] = '\0';
      handle_line_uart1(line1);
    }
  }
}

/* ======================= HAL 초기화 코드들 ======================= */

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
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
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

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64-1;       /* 64MHz / 64 = 1MHz */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;           /* 초기값: f_up=1kHz → f_step≈500Hz (가동 시 재설정) */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
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

  /* 초기 출력 레벨 */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|STEP_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DIR_PIN_Pin|EN_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(water_GPIO_Port, water_Pin, GPIO_PIN_SET); /* spray idle High */

  /* User Button */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* LD2 */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* STEP = PA6 : 고속 권장 */
  GPIO_InitStruct.Pin = STEP_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEP_PIN_GPIO_Port, &GPIO_InitStruct);

  /* DIR / EN / water = PBx */
  GPIO_InitStruct.Pin = DIR_PIN_Pin|EN_PIN_Pin|water_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* Error & Assert */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
