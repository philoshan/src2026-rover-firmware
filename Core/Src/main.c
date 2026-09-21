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
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "motor.h"
#include "encoder.h"
#include "sts3215.h"
#include "joint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int32_t  last_raw_count;  /**< Previous raw encoder tick */
    int64_t  total_count;     /**< Cumulative tick count (no rollover) */
    float    rpm;             /**< Wheel RPM */
    float    rad_per_sec;     /**< Angular velocity [rad/s] */
    float    linear_vel;      /**< Linear velocity [m/s] = w * R */
} WheelState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Motor & Encoder Specifications */
#define GEAR_RATIO          131.0f             /**< 131:1 Gearbox reduction (SPG30E-GR131) */
#define ENCODER_PPR         13.0f              /**< 13 pulses per motor rev */
#define ENCODER_CPR         (ENCODER_PPR * 4.0f) /**< 52 ticks in 4x mode */
#define TICKS_PER_REV       (ENCODER_CPR * GEAR_RATIO) /**< 6812.0 ticks per wheel rev */

/* Wheel radius R (Diameter 96mm -> Radius 48mm = 0.048m) */
#define WHEEL_RADIUS_M      0.048f             /**< Radius R = 48mm = 0.048m (Diameter 96mm) */
#define PI                  3.1415926535f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */

/* === 터미널 명령 수신 버퍼 (USART2 RX 인터럽트) === */
#define CMD_BUF_SIZE  64
static char     cmd_buf[CMD_BUF_SIZE];  /**< 줄 단위 명령 버퍼 */
static uint8_t  cmd_idx = 0;            /**< 현재 버퍼 인덱스 */
static volatile uint8_t cmd_ready = 0;  /**< 1이면 완성된 명령이 있음 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
static void MX_USART2_Init(void);
static void MX_USART3_Init(void);
static void servo_boot_sequence(void);
static void process_command(const char *cmd);
static void print_help(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void MX_USART2_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  /* PA2: USART2_TX, PA3: USART2_RX (Connected to ST-LINK VCP) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure Baud rate 115200 (Auto-calculated based on APB1 clock) */
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  USART2->BRR = (pclk1 + (115200 / 2)) / 115200;
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

int __io_putchar(int ch)
{
  while (!(USART2->SR & USART_SR_TXE));
  USART2->DR = (uint8_t)ch;
  return ch;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  for (int i = 0; i < len; i++)
  {
    if (*ptr == '\n')
    {
      __io_putchar('\r');
    }
    __io_putchar(*ptr++);
  }
  return len;
}

/* Newlib-nano syscall stubs (required when using printf with nano.specs) */
int _read(int file, char *ptr, int len)   { (void)file; (void)ptr; (void)len; return 0; }
int _close(int file)                      { (void)file; return -1; }
int _isatty(int file)                     { (void)file; return 1; }
int _lseek(int file, int ptr, int dir)    { (void)file; (void)ptr; (void)dir; return 0; }
#include <sys/stat.h>
int _fstat(int file, struct stat *st)     { (void)file; st->st_mode = S_IFCHR; return 0; }
int _getpid(void)                         { return 1; }
int _kill(int pid, int sig)               { (void)pid; (void)sig; return -1; }

/* ================================================================== */
/*        USART3 초기화 — STS3215 서보 통신 (PB10/PB11, 1Mbps)        */
/* ================================================================== */
static void MX_USART3_Init(void)
{
  /* GPIO 클럭 및 USART3 클럭 활성화 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();

  /* PB10: USART3_TX (Single-Wire Half-Duplex 모드, 이 핀 하나로 송수신 동시 처리) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10;                 /* PB10 하나만 사용 */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;            /* ★ 반드시 Open-Drain으로 설정 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;                /* 내부 풀업 활성화 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure Baud rate 1000000 (1Mbps, Auto-calculated based on APB1 clock) */
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  USART3->BRR = (pclk1 + (1000000 / 2)) / 1000000;
  
  /* CR3: HDSEL (Half-Duplex Selection) 비트 세트 */
  USART3->CR3 |= USART_CR3_HDSEL;
  
  /* CR1: 송수신 활성화 및 USART 활성화 */
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* ================================================================== */
/*        USART2 RX 인터럽트 초기화 및 바이트 처리 루틴               */
/* ================================================================== */
static void MX_USART2_RxInt_Init(void)
{
  /* USART2 RXNE 인터럽트 활성화 */
  USART2->CR1 |= USART_CR1_RXNEIE;
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_Process_Rx_Byte(uint8_t byte)
{
  char ch = (char)byte;

  /* 엔터(\r 또는 \n) → 명령 완성 */
  if (ch == '\r' || ch == '\n')
  {
    if (cmd_idx > 0 && !cmd_ready)
    {
      cmd_buf[cmd_idx] = '\0';
      cmd_ready = 1;
    }
  }
  else if (cmd_idx < CMD_BUF_SIZE - 1 && !cmd_ready)
  {
    cmd_buf[cmd_idx++] = ch;
  }
}

/* ================================================================== */
/*              서보 부팅 시퀀스 — PING → 토크 ON → 준비              */
/* ================================================================== */
static void servo_boot_sequence(void)
{
  printf("\r\n--------------------------------------------------\r\n");
  printf("       STS3215 Servo Bus Initialization            \r\n");
  printf("--------------------------------------------------\r\n");
  printf("USART3: PB10(TX)/PB11(RX), 1,000,000 bps\r\n");
  printf("Servo Count: %d\r\n\r\n", SERVO_COUNT);

  /* 1. 각 서보 PING */
  uint8_t servo_ids[] = {1, 2, 3, 4};  /* SERVO_COUNT=4 기준 */
  uint8_t all_ok = 1;

  for (int i = 0; i < SERVO_COUNT; i++)
  {
    STS_Status ret = sts_ping(servo_ids[i]);
    if (ret == STS_OK)
    {
      printf("  Servo ID=%d: OK (응답 정상)\r\n", servo_ids[i]);
    }
    else
    {
      printf("  Servo ID=%d: FAIL (err=%d)\r\n", servo_ids[i], ret);
      all_ok = 0;
    }
  }

  if (!all_ok)
  {
    printf("\r\n[경고] 일부 서보 응답 없음. 배선/전원/ID 확인 필요.\r\n");
    printf("  점검: TX/RX 교차, GND 공통, 1Mbps 보레이트, 어댑터 모드\r\n");
  }

  /* 2. 관절 모듈 초기화 (가속도 설정 + 토크 ON) */
  joint_init();

  printf("\r\n>>> Servo System Ready! <<<\r\n");
  printf("터미널 명령어: T / P / S / ON / OFF / PING / HELP\r\n");
  printf("--------------------------------------------------\r\n\r\n");
}

/* ================================================================== */
/*              터미널 명령 파서 — 줄 단위 텍스트 처리                 */
/* ================================================================== */
static void process_command(const char *cmd)
{
  /* --- "T 10.5 -20" : 목표 각도 설정 --- */
  if (cmd[0] == 'T' || cmd[0] == 't')
  {
    float angles[SERVO_COUNT] = {0};
    int parsed = 0;

    /* "T" 이후의 숫자들을 파싱 */
    const char *p = cmd + 1;
    for (int i = 0; i < SERVO_COUNT; i++)
    {
      char *endp;
      float val = strtof(p, &endp);
      if (endp == p) break;  /* 더 이상 숫자 없음 */
      angles[i] = val;
      parsed++;
      p = endp;
    }

    if (parsed > 0)
    {
      printf("[CMD] 목표 각도:");
      for (int i = 0; i < parsed; i++)
      {
        printf(" %.1f", angles[i]);
      }
      printf(" (도)\r\n");

      on_target_angles_received(angles, parsed);
    }
    else
    {
      printf("[CMD] 사용법: T <각1> <각2> <각3> <각4>  (예: T 45 -45 45 -45)\r\n");
    }
  }
  /* --- "P" : 현재 각도 출력 --- */
  else if (cmd[0] == 'P' || cmd[0] == 'p')
  {
    printf("[현재 각도]\r\n");
    for (int i = 0; i < SERVO_COUNT; i++)
    {
      float angle = joint_get_angle(i);
      int16_t pos = sts_read_position(i + 1);  /* ID = i+1 */
      printf("  Servo %d: %.1f deg (raw pos=%d)\r\n", i + 1, angle, pos);
    }
  }
  /* --- "S 200" : 이동 속도 설정 --- */
  else if (cmd[0] == 'S' || cmd[0] == 's')
  {
    int speed = 0;
    if (sscanf(cmd + 1, "%d", &speed) == 1 && speed >= 0)
    {
      joint_set_speed((uint16_t)speed);
    }
    else
    {
      printf("[CMD] 사용법: S <속도>  (예: S 200, S 0=최대속도)\r\n");
    }
  }
  /* --- "ON" : 토크 ON --- */
  else if ((cmd[0] == 'O' || cmd[0] == 'o') && (cmd[1] == 'N' || cmd[1] == 'n'))
  {
    joint_set_torque_all(1);
  }
  /* --- "OFF" : 토크 OFF --- */
  else if ((cmd[0] == 'O' || cmd[0] == 'o') && (cmd[1] == 'F' || cmd[1] == 'f'))
  {
    joint_set_torque_all(0);
  }
  /* --- "PING" : 연결 확인 --- */
  else if (strncmp(cmd, "PING", 4) == 0 || strncmp(cmd, "ping", 4) == 0)
  {
    for (int i = 0; i < SERVO_COUNT; i++)
    {
      STS_Status ret = sts_ping(i + 1);
      printf("  Servo ID=%d: %s\r\n", i + 1, (ret == STS_OK) ? "OK" : "FAIL");
    }
  }
  /* --- "HELP" : 사용법 --- */
  else if (strncmp(cmd, "HELP", 4) == 0 || strncmp(cmd, "help", 4) == 0)
  {
    print_help();
  }
  /* --- 알 수 없는 명령 --- */
  else
  {
    printf("[CMD] 알 수 없는 명령: '%s'\r\n", cmd);
    print_help();
  }
}

static void print_help(void)
{
  printf("========== 서보 명령어 도움말 ==========\r\n");
  printf("  T <a1> <a2>  : 목표 각도 설정 (도)\r\n");
  printf("                 예) T 10.5 -20\r\n");
  printf("  P            : 현재 각도 출력\r\n");
  printf("  S <speed>    : 이동 속도 설정 (0=최대)\r\n");
  printf("                 예) S 200\r\n");
  printf("  ON           : 토크 ON\r\n");
  printf("  OFF          : 토크 OFF\r\n");
  printf("  PING         : 서보 연결 확인\r\n");
  printf("  HELP         : 이 도움말\r\n");
  printf("========================================\r\n");
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  MX_USART2_Init();
  Motor_Init();
  Encoder_Init();

  printf("\r\n==================================================\r\n");
  printf("     STM32F446RE Rover Motor & Encoder Test       \r\n");
  printf("==================================================\r\n");
  printf("Baud rate: 115200 bps\r\n");
  printf("Motor Speed: +/-400 (PWM Duty ~50%%)\r\n\r\n");

  /* === 서보 모터 초기화 === */
  MX_USART3_Init();           /* USART3 (PB10/PB11, 1Mbps) 레지스터 초기화 */
  sts_init();                 /* STS3215 드라이버 초기화 */
  MX_USART2_RxInt_Init();     /* USART2 RX 인터럽트 활성화 */
  servo_boot_sequence();      /* PING → 토크 ON → 준비 완료 메시지 */

  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  WheelState_t wheels[MOTOR_COUNT] = {0};
  uint32_t last_log_time = HAL_GetTick();
  uint32_t step_start_time = HAL_GetTick();
  uint8_t step = 0;

  /* Initialize last raw counts */
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    wheels[i].last_raw_count = Encoder_GetCount((MotorID)i);
  }

  /* Test parameters */
  const int16_t TEST_SPEED = 400; /* ~50% PWM Duty (PWM_MAX is 799) */

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* 1. Periodic Kinematics & Encoder Log (every 100ms) */
    if (now - last_log_time >= 100)
    {
      float dt = (float)(now - last_log_time) / 1000.0f;
      last_log_time = now;

      for (uint8_t i = 0; i < MOTOR_COUNT; i++)
      {
        int32_t current_raw = Encoder_GetCount((MotorID)i);
        int32_t delta_tick = current_raw - wheels[i].last_raw_count;
        wheels[i].last_raw_count = current_raw;
        wheels[i].total_count += delta_tick;

        /* Wheel RPM = (delta_tick / TICKS_PER_REV) * (60.0f / dt) */
        wheels[i].rpm = ((float)delta_tick / TICKS_PER_REV) * (60.0f / dt);

        /* Angular velocity w [rad/s] = RPM * (2*pi / 60) */
        wheels[i].rad_per_sec = wheels[i].rpm * (2.0f * PI / 60.0f);

        /* Linear velocity v [m/s] = w * R */
        wheels[i].linear_vel = wheels[i].rad_per_sec * WHEEL_RADIUS_M;
      }

      /* Print detailed state with Timestamp */
      printf("[TIME:%6lums | dt:%3dms | R=0.048m]\r\n", now, (int)(dt * 1000.0f));
      for (uint8_t i = 0; i < MOTOR_COUNT; i++)
      {
        int rpm_int = (int)wheels[i].rpm;
        int rpm_frac = (int)(fabsf(wheels[i].rpm) * 10.0f) % 10;
        int rad_int = (int)wheels[i].rad_per_sec;
        int rad_frac = (int)(fabsf(wheels[i].rad_per_sec) * 100.0f) % 100;
        int vel_int = (int)wheels[i].linear_vel;
        int vel_frac = (int)(fabsf(wheels[i].linear_vel) * 1000.0f) % 1000;

        printf(" M%d | Raw:%7ld | %4d.%1d RPM | %3d.%02d rad/s | %2d.%03d m/s\r\n",
               i + 1,
               wheels[i].last_raw_count,
               rpm_int, rpm_frac,
               rad_int, rad_frac,
               vel_int, vel_frac);
      }
      printf("-----------------------------------------------------------------\r\n");
    }

    /* 2. Step-by-Step Motor Test Sequence */
    switch (step)
    {
      /* --- Motor 1 Test --- */
      case 0:
        printf("\r\n>>> [Step 0] Motor 1 Forward (+%d) for 2s <<<\r\n", TEST_SPEED);
        Motor_SetSpeed(MOTOR1, TEST_SPEED);
        step++;
        step_start_time = now;
        break;
      case 1:
        if (now - step_start_time >= 2000) {
          printf(">>> [Step 1] Motor 1 Reverse (-%d) for 2s <<<\r\n", TEST_SPEED);
          Motor_SetSpeed(MOTOR1, -TEST_SPEED);
          step++;
          step_start_time = now;
        }
        break;
      case 2:
        if (now - step_start_time >= 2000) {
          printf(">>> [Step 2] Motor 1 Stop (1s pause) <<<\r\n");
          Motor_Stop(MOTOR1);
          step++;
          step_start_time = now;
        }
        break;

      /* --- Motor 2 Test --- */
      case 3:
        if (now - step_start_time >= 1000) {
          printf("\r\n>>> [Step 3] Motor 2 Forward (+%d) for 2s <<<\r\n", TEST_SPEED);
          Motor_SetSpeed(MOTOR2, TEST_SPEED);
          step++;
          step_start_time = now;
        }
        break;
      case 4:
        if (now - step_start_time >= 2000) {
          printf(">>> [Step 4] Motor 2 Reverse (-%d) for 2s <<<\r\n", TEST_SPEED);
          Motor_SetSpeed(MOTOR2, -TEST_SPEED);
          step++;
          step_start_time = now;
        }
        break;
      case 5:
        if (now - step_start_time >= 2000) {
          printf(">>> [Step 5] Motor 2 Stop (1s pause) <<<\r\n");
          Motor_Stop(MOTOR2);
          step++;
          step_start_time = now;
        }
        break;

      /* --- Motor 3 Test --- */
      case 6:
        if (now - step_start_time >= 1000) {
          printf("\r\n>>> [Step 6] Motor 3 Forward (+%d) for 2s <<<\r\n", TEST_SPEED);
          Motor_SetSpeed(MOTOR3, TEST_SPEED);
          step++;
          step_start_time = now;
        }
        break;
      case 7:
        if (now - step_start_time >= 2000) {
          printf(">>> [Step 7] Motor 3 Reverse (-%d) for 2s <<<\r\n", TEST_SPEED);
          Motor_SetSpeed(MOTOR3, -TEST_SPEED);
          step++;
          step_start_time = now;
        }
        break;
      case 8:
        if (now - step_start_time >= 2000) {
          printf(">>> [Step 8] Motor 3 Stop (1s pause) <<<\r\n");
          Motor_Stop(MOTOR3);
          step++;
          step_start_time = now;
        }
        break;

      /* --- Motor 4 Test --- */
      case 9:
        if (now - step_start_time >= 1000) {
          printf("\r\n>>> [Step 9] Motor 4 Forward (+%d) for 2s <<<\r\n", TEST_SPEED);
          Motor_SetSpeed(MOTOR4, TEST_SPEED);
          step++;
          step_start_time = now;
        }
        break;
      case 10:
        if (now - step_start_time >= 2000) {
          printf(">>> [Step 10] Motor 4 Reverse (-%d) for 2s <<<\r\n", TEST_SPEED);
          Motor_SetSpeed(MOTOR4, -TEST_SPEED);
          step++;
          step_start_time = now;
        }
        break;
      case 11:
        if (now - step_start_time >= 2000) {
          printf(">>> [Step 11] Motor 4 Stop (1s pause) <<<\r\n");
          Motor_Stop(MOTOR4);
          step++;
          step_start_time = now;
        }
        break;

      /* --- All Motors Forward Test --- */
      case 12:
        if (now - step_start_time >= 1000) {
          printf("\r\n>>> [Step 12] All Motors Forward (+%d) for 3s <<<\r\n", TEST_SPEED);
          for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            Motor_SetSpeed((MotorID)i, TEST_SPEED);
          }
          step++;
          step_start_time = now;
        }
        break;
      case 13:
        if (now - step_start_time >= 3000) {
          printf(">>> [Step 13] All Motors Stop & Reset Encoders (2s pause) <<<\r\n");
          Motor_StopAll();
          for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            Encoder_Reset((MotorID)i);
            wheels[i].last_raw_count = 0;
            wheels[i].total_count = 0;
          }
          step = 0; /* Loop test sequence */
          step_start_time = now + 1000; /* 2s pause */
        }
        break;

      default:
        step = 0;
        break;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 3. 서보 터미널 명령 처리 (비블로킹) */
    if (cmd_ready)
    {
      process_command(cmd_buf);
      cmd_idx = 0;
      cmd_ready = 0;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 799;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

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
