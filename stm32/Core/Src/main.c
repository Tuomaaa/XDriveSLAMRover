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
#include "mcp2515.h"
#include "can.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float kp, ki, kd;
  float integral;
  float prev_error;
} PID_t;

typedef struct {
  TIM_HandleTypeDef* enc_tim;   // encoder timer
  uint32_t pwm_ch;              // TIM1 PWM channel
  uint16_t fwd_pin;             // direction forward
  uint16_t rev_pin;             // direction reverse
  int16_t target_rpm;           // from CAN 0x100
  int32_t cum_ticks;            // cumulative encoder ticks
  uint16_t prev_cnt;            // previous raw counter (for 16-bit overflow)
  uint8_t is_32bit;             // 1=32-bit timer, 0=16-bit
  PID_t pid;
} Motor_t;

// ── Motor index <-> physical wheel position ──
// This is the single source of truth for which motor index corresponds
// to which physical corner of the x-drive chassis. It MUST stay in sync
// with:
//   - odometry.py's MOTOR_MAP (RPi side, {0:"fl",1:"fr",2:"rl",3:"rr"})
//   - ps2_drive_test.py's inverse-kinematics ordering (m0=fl,m1=fr,m2=rl,m3=rr)
//   - CAN 0x100 (velocity command) / 0x200,0x201 (encoder feedback) payload order
// Confirmed against physical wiring 2026-07-03: see PROJECT_STATE.md decision log.
typedef enum {
  MOTOR_FL = 0,  // front-left  -> TIM2 (32-bit encoder, PA15/PB3)
  MOTOR_FR = 1,  // front-right -> TIM3 (16-bit encoder, PB4/PA7)
  MOTOR_RL = 2,  // rear-left   -> TIM4 (16-bit encoder, PB6/PB7)
  MOTOR_RR = 3,  // rear-right  -> TIM5 (32-bit encoder, PA0/PA1)
} MotorPosition;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_MOTORS       4
#define PWM_PERIOD       4999
#define MAX_RPM          300

// Motor control mode:
//   0 = open-loop  -> joystick target RPM maps straight to PWM duty. No
//       encoder feedback in the control path, so the reversed encoders
//       can't cause PID runaway. This is what the RC + odometry-calibration
//       phase needs; odometry reads encoders directly and does NOT depend
//       on closed-loop speed control.
//   1 = closed-loop speed PID (see pid_compute + ENC_SIGN). Re-enable later
//       for autonomous cmd_vel driving. The PID code is retained below.
#define USE_PID          0

// Encoder counts per output-shaft revolution
// GA12-N20: 7 PPR motor × gear_ratio × 4 (TI12 mode)
// Adjust this after measuring your actual encoder!
#define ENCODER_CPR      2800

// PID gains (tune these on real hardware) -- only used when USE_PID = 1
#define PID_KP           8.0f
#define PID_KI           2.0f
#define PID_KD           0.1f

// Timing
#define PID_INTERVAL_MS  20     // control + encoder send rate (20ms = 50Hz)
#define HEARTBEAT_TIMEOUT_MS 200

// CAN IDs
#define CAN_ID_VEL_CMD   0x100   // payload: int16 target RPM x4, order = [MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR]
#define CAN_ID_ENCODER_0 0x200   // payload: int32 cum_ticks x2 = [MOTOR_FL, MOTOR_FR]
#define CAN_ID_ENCODER_1 0x201   // payload: int32 cum_ticks x2 = [MOTOR_RL, MOTOR_RR]
#define CAN_ID_HEARTBEAT 0x300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Motor_t motors[NUM_MOTORS];
uint32_t last_heartbeat = 0;
uint32_t last_pid_time = 0;

#if USE_PID
// Encoder direction vs. motor-command direction, per motor index.
// CALIBRATED 2026-07-04: all four encoders count DOWN when the motor is
// driven forward (target_rpm > 0). Without this correction the PID uses
// the raw (reversed) count as feedback, which turns speed control into
// POSITIVE feedback and makes the motors run away to full speed on the
// slightest disturbance. This sign is ONLY applied to the PID feedback;
// the cumulative ticks sent over CAN (0x200/0x201) stay raw, so the RPi
// odometry keeps its own ENCODER_SIGN = -1 and needs no change.
static const int8_t ENC_SIGN[NUM_MOTORS] = {-1, -1, -1, -1};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ── Read encoder ticks (handles 16-bit overflow) ──
void encoder_update(Motor_t* m) {
  if (m->is_32bit) {
    m->cum_ticks = (int32_t)__HAL_TIM_GET_COUNTER(m->enc_tim);
  } else {
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(m->enc_tim);
    int16_t delta = (int16_t)(cnt - m->prev_cnt);
    m->cum_ticks += delta;
    m->prev_cnt = cnt;
  }
}

#if USE_PID
// ── PID compute: returns PWM duty (signed, +=forward) ──
// Retained for the future closed-loop phase; compiled only when USE_PID=1.
float pid_compute(PID_t* pid, float target, float actual, float dt) {
  float error = target - actual;
  pid->integral += error * dt;

  // Anti-windup: clamp integral
  if (pid->integral > 500.0f) pid->integral = 500.0f;
  if (pid->integral < -500.0f) pid->integral = -500.0f;

  float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
  pid->prev_error = error;

  return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
#endif

// ── Set motor direction and PWM ──
void motor_set_pwm(Motor_t* m, int32_t duty) {
  if (duty > 0) {
    HAL_GPIO_WritePin(GPIOB, m->fwd_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, m->rev_pin, GPIO_PIN_RESET);
  } else if (duty < 0) {
    HAL_GPIO_WritePin(GPIOB, m->fwd_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, m->rev_pin, GPIO_PIN_SET);
    duty = -duty;
  } else {
    HAL_GPIO_WritePin(GPIOB, m->fwd_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, m->rev_pin, GPIO_PIN_RESET);
  }
  if (duty > PWM_PERIOD) duty = PWM_PERIOD;
  __HAL_TIM_SET_COMPARE(&htim1, m->pwm_ch, (uint32_t)duty);
}

// ── Stop all motors ──
void motors_stop(void) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].target_rpm = 0;
    motor_set_pwm(&motors[i], 0);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // ── Motor config table ──
  // Indexed by MotorPosition (see typedef above) -- NOT arbitrary order.
  // Wiring confirmed 2026-07-03: TIM2=FL, TIM3=FR, TIM4=RL, TIM5=RR.
  motors[MOTOR_FL] = (Motor_t){&htim2, TIM_CHANNEL_1, GPIO_PIN_1,  GPIO_PIN_0,  0, 0, 0, 1, {PID_KP,PID_KI,PID_KD, 0,0}};
  motors[MOTOR_FR] = (Motor_t){&htim3, TIM_CHANNEL_2, GPIO_PIN_10, GPIO_PIN_2,  0, 0, 0, 0, {PID_KP,PID_KI,PID_KD, 0,0}};
  motors[MOTOR_RL] = (Motor_t){&htim4, TIM_CHANNEL_3, GPIO_PIN_12, GPIO_PIN_13, 0, 0, 0, 0, {PID_KP,PID_KI,PID_KD, 0,0}};
  motors[MOTOR_RR] = (Motor_t){&htim5, TIM_CHANNEL_4, GPIO_PIN_14, GPIO_PIN_15, 0, 0, 0, 1, {PID_KP,PID_KI,PID_KD, 0,0}};

  // ── Start 4 encoder timers ──
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // ── Start 4 PWM channels (duty=0) ──
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // ── MCP2515 init ──
  CAN_Error err;
  err = MCP_reset();
  if (err == ERROR_OK) err = MCP_setBitrateClock(CAN_500KBPS, MCP_8MHZ);
  if (err == ERROR_OK) err = MCP_setNormalMode();

  last_heartbeat = HAL_GetTick();
  last_pid_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN WHILE */

    // ── Poll CAN messages ──
    can_frame rx_frame;
    if (MCP_readMessage(&rx_frame) == ERROR_OK) {

      // Velocity command (0x100): 4× int16 target RPM, order = MOTOR_FL/FR/RL/RR
      if (rx_frame.can_id == CAN_ID_VEL_CMD && rx_frame.can_dlc == 8) {
        int16_t rpm[4];
        memcpy(rpm, rx_frame.data, 8);
        for (int i = 0; i < NUM_MOTORS; i++) {
          motors[i].target_rpm = rpm[i];  // i indexes MotorPosition directly
        }
      }

      // Heartbeat (0x300)
      if (rx_frame.can_id == CAN_ID_HEARTBEAT) {
        last_heartbeat = HAL_GetTick();
      }
    }

    // ── Heartbeat alive? ──
    // If not, we must skip the control update entirely (below), otherwise the
    // control block would immediately re-drive the PWM that motors_stop() just
    // cleared, defeating the emergency stop.
    uint8_t hb_ok = (HAL_GetTick() - last_heartbeat <= HEARTBEAT_TIMEOUT_MS);
    if (!hb_ok) {
      motors_stop();
    }

    // ── Motor control + encoder send @ 20ms ──
    uint32_t now = HAL_GetTick();
    if (now - last_pid_time >= PID_INTERVAL_MS) {
#if USE_PID
      float dt = (now - last_pid_time) / 1000.0f;
      static int32_t prev_ticks[NUM_MOTORS] = {0};
#endif
      last_pid_time = now;

      for (int i = 0; i < NUM_MOTORS; i++) {
        // Always update the encoder (needed for cumulative ticks + CAN, and
        // so 16-bit timers don't silently wrap between reads), even when the
        // heartbeat is dead and motors are stopped.
        encoder_update(&motors[i]);

#if USE_PID
        // Closed-loop speed control with encoder-direction correction.
        int32_t delta = motors[i].cum_ticks - prev_ticks[i];
        prev_ticks[i] = motors[i].cum_ticks;
        float actual_rpm = (ENC_SIGN[i] * delta * 60.0f) / (dt * ENCODER_CPR);

        if (hb_ok) {
          float target = (float)motors[i].target_rpm;
          float output = pid_compute(&motors[i].pid, target, actual_rpm, dt);
          int32_t duty = (int32_t)(output * PWM_PERIOD / MAX_RPM);
          motor_set_pwm(&motors[i], duty);
        } else {
          // Emergency stop: keep motors off and reset PID state so the
          // integrator can't wind up while stopped (no bump on restart).
          motors[i].pid.integral = 0.0f;
          motors[i].pid.prev_error = 0.0f;
          motor_set_pwm(&motors[i], 0);
        }
#else
        // Open-loop: joystick target RPM maps straight to PWM duty. No
        // encoder feedback in the control path, so the reversed-encoder
        // polarity cannot cause runaway. motor_set_pwm() handles direction
        // sign and clamps magnitude to PWM_PERIOD.
        if (hb_ok) {
          int32_t duty = (int32_t)((float)motors[i].target_rpm * PWM_PERIOD / MAX_RPM);
          motor_set_pwm(&motors[i], duty);
        } else {
          motor_set_pwm(&motors[i], 0);
        }
#endif
      }

      // ── Send encoder data ──
      // Frame 0x200: MOTOR_FL + MOTOR_FR
      can_frame tx0;
      tx0.can_id = CAN_ID_ENCODER_0;
      tx0.can_dlc = 8;
      memcpy(&tx0.data[0], &motors[MOTOR_FL].cum_ticks, 4);
      memcpy(&tx0.data[4], &motors[MOTOR_FR].cum_ticks, 4);
      MCP_sendMessage(&tx0);

      // Frame 0x201: MOTOR_RL + MOTOR_RR
      can_frame tx1;
      tx1.can_id = CAN_ID_ENCODER_1;
      tx1.can_dlc = 8;
      memcpy(&tx1.data[0], &motors[MOTOR_RL].cum_ticks, 4);
      memcpy(&tx1.data[4], &motors[MOTOR_RR].cum_ticks, 4);
      MCP_sendMessage(&tx1);
    }

    HAL_Delay(1);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor1_1_Pin|Motor1_2_Pin|Motor2_1_Pin|Motor2_2_Pin
                          |Motor3_1_Pin|Motor3_2_Pin|Motor4_1_Pin|Motor4_2_Pin
                          |CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Motor1_1_Pin Motor1_2_Pin Motor2_1_Pin Motor2_2_Pin
                           Motor3_1_Pin Motor3_2_Pin Motor4_1_Pin Motor4_2_Pin
                           CS_Pin */
  GPIO_InitStruct.Pin = Motor1_1_Pin|Motor1_2_Pin|Motor2_1_Pin|Motor2_2_Pin
                          |Motor3_1_Pin|Motor3_2_Pin|Motor4_1_Pin|Motor4_2_Pin
                          |CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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