/*
 * Sequential HC-SR04 input-capture driver for TIM9 CH1/CH2.
 *
 * CubeMX regeneration overwrites the generated CMake source list. Re-add
 * both Core/Src/hc_sr04.c and Core/Src/mcp2515.c after regenerating.
 *
 * Hardware note: a shared TRIG wire fires both modules on every pulse.
 * This driver gates which ECHO channel is accepted, but true acoustic
 * sequencing requires separate trigger GPIOs or external gating.
 */

#include "hc_sr04.h"

#include "main.h"

extern TIM_HandleTypeDef htim9;

#define HC_SR04_TIMEOUT_MS 40U
#define HC_SR04_TRIGGER_SPACING_MS 60U
#define HC_SR04_TRIGGER_US 12U
#define HC_SR04_STATUS_RIGHT_VALID (1U << 0)
#define HC_SR04_STATUS_BACK_VALID  (1U << 1)

typedef enum {
  HC_STATE_IDLE = 0,
  HC_STATE_TRIG_RIGHT,
  HC_STATE_WAIT_RIGHT_RISING,
  HC_STATE_WAIT_RIGHT_FALLING,
  HC_STATE_TRIG_BACK,
  HC_STATE_WAIT_BACK_RISING,
  HC_STATE_WAIT_BACK_FALLING,
  HC_STATE_DONE,
} HC_SR04_State;

static volatile HC_SR04_State state = HC_STATE_IDLE;
static volatile uint16_t rising_capture = 0U;
static volatile uint16_t right_pulse_us = 0U;
static volatile uint16_t back_pulse_us = 0U;
static volatile uint32_t echo_deadline_ms = 0U;
static volatile uint32_t next_trigger_ms = 0U;
static volatile HC_SR04_Result result = {
  HC_SR04_INVALID_MM,
  HC_SR04_INVALID_MM,
  0U,
};

static void delay_us(uint16_t delay)
{
  uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim9);
  while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim9) - start) < delay) {
  }
}

static uint16_t pulse_to_mm(uint16_t pulse_width_us)
{
  /* distance_mm = pulse_width_us / 5.8, rounded to nearest millimeter. */
  uint32_t distance = ((uint32_t)pulse_width_us * 10U + 29U) / 58U;
  return (distance < HC_SR04_INVALID_MM) ? (uint16_t)distance
                                         : (HC_SR04_INVALID_MM - 1U);
}

static void set_rising_polarity(uint32_t channel)
{
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim9, channel, TIM_INPUTCHANNELPOLARITY_RISING);
}

static void trigger(uint8_t right_sensor)
{
  uint32_t channel = right_sensor ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
  uint32_t interrupt = right_sensor ? TIM_IT_CC1 : TIM_IT_CC2;

  state = right_sensor ? HC_STATE_TRIG_RIGHT : HC_STATE_TRIG_BACK;
  set_rising_polarity(channel);
  __HAL_TIM_CLEAR_IT(&htim9, interrupt);

  uint32_t now = HAL_GetTick();
  echo_deadline_ms = now + HC_SR04_TIMEOUT_MS;
  next_trigger_ms = now + HC_SR04_TRIGGER_SPACING_MS;
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  delay_us(HC_SR04_TRIGGER_US);
  state = right_sensor ? HC_STATE_WAIT_RIGHT_RISING
                       : HC_STATE_WAIT_BACK_RISING;
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

static uint8_t deadline_reached(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

void HC_SR04_Init(void)
{
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  set_rising_polarity(TIM_CHANNEL_1);
  set_rising_polarity(TIM_CHANNEL_2);

  if (HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  state = HC_STATE_IDLE;
}

void HC_SR04_StartMeasurement(void)
{
  if (state != HC_STATE_IDLE && state != HC_STATE_DONE) {
    return;
  }

  result.right_mm = HC_SR04_INVALID_MM;
  result.back_mm = HC_SR04_INVALID_MM;
  result.status = 0U;
  state = HC_STATE_TRIG_RIGHT;
  if (deadline_reached(HAL_GetTick(), next_trigger_ms)) {
    trigger(1U);
  }
}

uint8_t HC_SR04_IsComplete(void)
{
  uint32_t now = HAL_GetTick();

  if (state == HC_STATE_TRIG_RIGHT &&
      deadline_reached(now, next_trigger_ms)) {
    trigger(1U);
  } else if (state == HC_STATE_TRIG_BACK &&
             deadline_reached(now, next_trigger_ms)) {
    trigger(0U);
  }

  if ((state == HC_STATE_WAIT_RIGHT_RISING ||
       state == HC_STATE_WAIT_RIGHT_FALLING) &&
      deadline_reached(now, echo_deadline_ms)) {
    result.right_mm = HC_SR04_INVALID_MM;
    set_rising_polarity(TIM_CHANNEL_1);
    state = HC_STATE_TRIG_BACK;
  } else if ((state == HC_STATE_WAIT_BACK_RISING ||
              state == HC_STATE_WAIT_BACK_FALLING) &&
             deadline_reached(now, echo_deadline_ms)) {
    result.back_mm = HC_SR04_INVALID_MM;
    set_rising_polarity(TIM_CHANNEL_2);
    state = HC_STATE_DONE;
  }

  return (state == HC_STATE_DONE) ? 1U : 0U;
}

HC_SR04_Result HC_SR04_GetResult(void)
{
  HC_SR04_Result snapshot;
  snapshot.status = result.status;
  snapshot.right_mm = (snapshot.status & HC_SR04_STATUS_RIGHT_VALID)
      ? pulse_to_mm(right_pulse_us) : HC_SR04_INVALID_MM;
  snapshot.back_mm = (snapshot.status & HC_SR04_STATUS_BACK_VALID)
      ? pulse_to_mm(back_pulse_us) : HC_SR04_INVALID_MM;
  return snapshot;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint16_t capture;

  if (htim->Instance != TIM9) {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if (state == HC_STATE_WAIT_RIGHT_RISING) {
      rising_capture = capture;
      __HAL_TIM_SET_CAPTUREPOLARITY(
          htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      state = HC_STATE_WAIT_RIGHT_FALLING;
    } else if (state == HC_STATE_WAIT_RIGHT_FALLING) {
      right_pulse_us = (uint16_t)(capture - rising_capture);
      result.status |= HC_SR04_STATUS_RIGHT_VALID;
      set_rising_polarity(TIM_CHANNEL_1);
      state = HC_STATE_TRIG_BACK;
    }
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    if (state == HC_STATE_WAIT_BACK_RISING) {
      rising_capture = capture;
      __HAL_TIM_SET_CAPTUREPOLARITY(
          htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
      state = HC_STATE_WAIT_BACK_FALLING;
    } else if (state == HC_STATE_WAIT_BACK_FALLING) {
      back_pulse_us = (uint16_t)(capture - rising_capture);
      result.status |= HC_SR04_STATUS_BACK_VALID;
      set_rising_polarity(TIM_CHANNEL_2);
      state = HC_STATE_DONE;
    }
  }
}