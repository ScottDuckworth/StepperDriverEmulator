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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmd.h"
#include "usbd_cdc_if.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

// Defined in usb_device.c
extern USBD_HandleTypeDef hUsbDeviceFS;

static volatile uint32_t now;

typedef struct {
  int32_t step_pos;
  int32_t encoder_pos;
  uint16_t step_cnt_prev;
  uint16_t step_dcnt;
  bool step_reverse;
} PositionCounters_t;

static PositionCounters_t position;

static volatile uint32_t step_period_cnt;
static volatile int32_t load_tension = 0;
static volatile bool stall_tripped = false;
static volatile bool blink_mode = false;
static volatile uint32_t last_step_time = 0;

#define CHUNK_SIZE 8
#define TOTAL_BUFFER_SIZE (2 * CHUNK_SIZE)

static uint32_t quad_buffer[TOTAL_BUFFER_SIZE];
static uint8_t current_quad_state = 0;
static int8_t half_0_delta = 0;
static int8_t half_1_delta = 0;
static volatile bool motion_active = false;
static volatile int32_t planned_encoder_pos = 0;

static const uint32_t QUAD_BSRR_STATES[4] = {
    GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5, // State 0: EA=0, EB=0
    GPIO_BSRR_BS_4 | GPIO_BSRR_BR_5, // State 1: EA=1, EB=0
    GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5, // State 2: EA=1, EB=1
    GPIO_BSRR_BR_4 | GPIO_BSRR_BS_5  // State 3: EA=0, EB=1
};

#define usb_output_data UserTxBufferFS
#define usb_input_data UserRxBufferFS

static volatile uint16_t usb_output_size;
static volatile uint16_t usb_input_size;

static RunConfig_t config = DEFAULT_RUN_CONFIG;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

RunConfig_t* GetConfig() {
  return &config;
}

static int8_t USB_ReceiveCallback(uint8_t* buf, uint32_t* len) {
  if (buf != usb_input_data && len && *len <= sizeof(usb_input_data)) {
    memcpy(usb_input_data, buf, *len);
  }
  usb_input_size = *len;
  return USBD_OK;
}

static void USB_ReceiveReady() {
  usb_input_size = 0;
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_input_data);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

static void USB_Flush(void) {
  if (usb_output_size == 0) return;
  uint8_t result = CDC_Transmit_FS(usb_output_data, usb_output_size);
  if (result == USBD_OK || result == USBD_FAIL) {
    usb_output_size = 0;
  }
}

uint16_t WriteData(const uint8_t* data, uint16_t size) {
  uint16_t cap = sizeof(usb_output_data) - usb_output_size;
  if (cap < size) size = cap;
  memcpy(usb_output_data + usb_output_size, data, size);
  usb_output_size += size;
  return size;
}

uint16_t WriteString(const char* text) {
  return WriteData((uint8_t*) text, strlen(text));
}

void ReportString(const char* var, const char* value) {
  WriteString(var);
  WriteString(" ");
  WriteString(value);
  WriteString("\r\n");
}

void ReportI32(const char* var, int32_t value) {
  int size;
  char output[12];
  size = snprintf(output, sizeof(output), " %ld\r\n", value);
  WriteString(var);
  WriteData((uint8_t*) output, size);
}

void ReportU32(const char* var, uint32_t value) {
  int size;
  char output[12];
  size = snprintf(output, sizeof(output), " %lu\r\n", value);
  WriteString(var);
  WriteData((uint8_t*) output, size);
}

void ReportU16(const char* var, uint16_t value) {
  int size;
  char output[12];
  size = snprintf(output, sizeof(output), " %u\r\n", value);
  WriteString(var);
  WriteData((uint8_t*) output, size);
}

void ReportU8(const char* var, uint8_t value) {
  int size;
  char output[8];
  size = snprintf(output, sizeof(output), " %u\r\n", value);
  WriteString(var);
  WriteData((uint8_t*) output, size);
}

void SetLED(uint8_t r, uint8_t g) {
  TIM17->CCR1 = r;
  TIM16->CCR1 = g;
}

void GetLED(uint8_t* r, uint8_t* g) {
  if (r) *r = TIM17->CCR1;
  if (g) *g = TIM16->CCR1;
}

bool GetBlinkMode(void) {
  return blink_mode;
}

void SetBlinkMode(bool enable) {
  blink_mode = enable;
  ReportBlinkMode();
}

void ReportBlinkMode(void) {
  ReportU8("blink", GetBlinkMode());
}

static void UpdateLEDs(void) {
  bool blink_phase = ((now / 125) & 1) != 0;
  if (blink_mode) {
    SetLED(blink_phase ? 255 : 0, blink_phase ? 255 : 0);
  } else if (stall_tripped) {
    SetLED(blink_phase ? 255 : 0, 0);
  } else if (!GetStepEnabled()) {
    SetLED(0, 0);
  } else if ((now - last_step_time) < 200) {
    SetLED(0, blink_phase ? 255 : 0);
  } else {
    SetLED(0, 255);
  }
}

int32_t GetTension(void) {
  return load_tension;
}

void SetTension(int32_t tension) {
  load_tension = tension;
  ReportTension();
  if (!motion_active) {
    Motion_Start();
  }
}

void ReportTension(void) {
  ReportI32("t", GetTension());
}

uint16_t GetOdr(void) {
  return config.odr;
}

void SetOdr(uint16_t odr) {
  config.odr = odr;
  ReportOdr();
}

void ReportOdr(void) {
  ReportU16("odr", GetOdr());
}

uint16_t GetEpr(void) {
  return config.epr;
}

void SetEpr(uint16_t epr) {
  config.epr = epr;
  ReportEpr();
}

void ReportEpr(void) {
  ReportU16("epr", GetEpr());
}

uint16_t GetSpr(void) {
  return config.spr;
}

void SetSpr(uint16_t spr) {
  config.spr = spr;
  ReportSpr();
}

void ReportSpr(void) {
  ReportU16("spr", GetSpr());
}

void GetTorqueCurve(int32_t* t0, uint32_t* v_knee, uint32_t* v_max, int32_t* t_min) {
  if (t0) *t0 = config.torque_t0;
  if (v_knee) *v_knee = config.torque_v_knee;
  if (v_max) *v_max = config.torque_v_max;
  if (t_min) *t_min = config.torque_t_min;
}

void SetTorqueCurve(int32_t t0, uint32_t v_knee, uint32_t v_max, int32_t t_min) {
  if (v_max <= v_knee) v_max = v_knee + 1;
  config.torque_t0 = t0;
  config.torque_v_knee = v_knee;
  config.torque_v_max = v_max;
  config.torque_t_min = t_min;
  ReportTorqueCurve();
}

void ReportTorqueCurve(void) {
  char buf[48];
  int size = snprintf(buf, sizeof(buf), "tcurve %ld %lu %lu %ld\r\n",
                      config.torque_t0, config.torque_v_knee, config.torque_v_max, config.torque_t_min);
  WriteData((uint8_t*) buf, size);
}

uint32_t GetStallThreshold(void) {
  return config.stall_threshold;
}

void SetStallThreshold(uint32_t threshold) {
  config.stall_threshold = threshold;
  ReportStallThreshold();
}

void ReportStallThreshold(void) {
  ReportU32("stall", GetStallThreshold());
}

float GetKfree(void) {
  return config.kfree;
}

void SetKfree(float k) {
  config.kfree = k;
  ReportKfree();
}

void ReportKfree(void) {
  ReportFloat("kfree", GetKfree());
}

bool GetStallTrip(void) {
  return stall_tripped;
}

void ReportStallTrip(void) {
  ReportU8("stall_trip", GetStallTrip());
}

int32_t CalcMotorTorqueConfig(const RunConfig_t* cfg, float speed_abs) {
  uint32_t v = (uint32_t) speed_abs;
  if (v <= cfg->torque_v_knee) {
    return cfg->torque_t0;
  }
  if (v >= cfg->torque_v_max) {
    return cfg->torque_t_min;
  }
  int64_t num = (int64_t)(cfg->torque_t0 - cfg->torque_t_min) * (v - cfg->torque_v_knee);
  int64_t den = (int64_t)(cfg->torque_v_max - cfg->torque_v_knee);
  return (int32_t)(cfg->torque_t0 - (num / den));
}

int32_t CalcMotorTorque(float speed_abs) {
  return CalcMotorTorqueConfig(&config, speed_abs);
}

void ReportFloat(const char* var, float value) {
  int size;
  char output[24];
  if (value < 0.0f) {
    float abs_val = -value;
    int32_t int_part = (int32_t) abs_val;
    int32_t frac_part = (int32_t) ((abs_val - (float) int_part) * 10000.0f + 0.5f);
    size = snprintf(output, sizeof(output), " -%ld.%04ld\r\n", int_part, frac_part);
  } else {
    int32_t int_part = (int32_t) value;
    int32_t frac_part = (int32_t) ((value - (float) int_part) * 10000.0f + 0.5f);
    size = snprintf(output, sizeof(output), " %ld.%04ld\r\n", int_part, frac_part);
  }
  WriteString(var);
  WriteData((uint8_t*) output, size);
}

float GetKp(void) {
  return config.kp;
}

void SetKp(float kp) {
  config.kp = kp;
  ReportKp();
}

void ReportKp(void) {
  ReportFloat("kp", GetKp());
}

float GetKff(void) {
  return config.kff;
}

void SetKff(float kff) {
  config.kff = kff;
  ReportKff();
}

void ReportKff(void) {
  ReportFloat("kff", GetKff());
}

int32_t GetEncoderPosition(void) {
  return position.encoder_pos;
}

void SetEncoderPosition(int32_t pos) {
  __disable_irq();
  position.encoder_pos = pos;
  planned_encoder_pos = pos;
  if (config.epr != 0) {
    position.step_pos = (int32_t)(((int64_t)pos * config.spr) / config.epr);
  } else {
    position.step_pos = 0;
  }
  position.step_cnt_prev = UINT16_MAX - (uint16_t) DMA1_Channel5->CNDTR;
  position.step_dcnt = 0;
  half_0_delta = 0;
  half_1_delta = 0;
  for (int i = 0; i < TOTAL_BUFFER_SIZE; i++) {
    quad_buffer[i] = QUAD_BSRR_STATES[current_quad_state];
  }
  TIM3->CR1 &= ~TIM_CR1_CEN;
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;
  motion_active = false;
  TIM2->SR = ~TIM_SR_CC1IF;
  TIM2->DIER = TIM_DIER_CC1DE | TIM_DIER_CC1IE;
  __enable_irq();
  ReportEncoderPosition();
}

void ReportEncoderPosition(void) {
  ReportI32("pos", GetEncoderPosition());
}

int32_t GetStepPosition(void) {
  return position.step_pos;
}

bool GetStepReverse(void) {
  return position.step_reverse;
}

void ReportStepReverse(void) {
  ReportU8("rev", GetStepReverse());
}

bool GetStepEnabled(void) {
  return READ_BIT(ENA_GPIO_Port->IDR, ENA_Pin) != 0;
}

void ReportStepEnabled(void) {
  ReportU8("ena", GetStepEnabled());
}

static int32_t StepToEncoderPosition(int32_t step_position) {
  if (config.spr == 0) return 0;
  return (int32_t)(((int64_t) step_position * config.epr) / config.spr);
}

void UpdateStepEnabled(void) {
  bool cleared_stall = false;
  __disable_irq();
  bool enabled = GetStepEnabled();
  if (enabled) {
    TIM2->CR1 |= TIM_CR1_CEN;
    if (stall_tripped) {
      stall_tripped = false;
      cleared_stall = true;
    }
    if (config.epr != 0) {
      position.step_pos = (int32_t)(((int64_t)position.encoder_pos * config.spr) / config.epr);
    }
    planned_encoder_pos = position.encoder_pos;
    position.step_cnt_prev = UINT16_MAX - (uint16_t) DMA1_Channel5->CNDTR;
    position.step_dcnt = 0;
  } else {
    TIM2->CR1 &= ~TIM_CR1_CEN;
    if (stall_tripped) {
      stall_tripped = false;
      cleared_stall = true;
    }
  }
  __enable_irq();
  if (cleared_stall) {
    ReportStallTrip();
  }
  ReportStepEnabled();
  if (!enabled && load_tension != 0 && !motion_active) {
    Motion_Start();
  }
}

static void UpdatePositionCounters(void) {
  uint16_t step_cnt = UINT16_MAX - (uint16_t) DMA1_Channel5->CNDTR;
  uint16_t step_dcnt = step_cnt - position.step_cnt_prev;
  position.step_cnt_prev = step_cnt;
  position.step_dcnt = step_dcnt;
  if (step_dcnt != 0) {
    last_step_time = now;
  }
  if (position.step_reverse) {
    position.step_pos -= step_dcnt;
  } else {
    position.step_pos += step_dcnt;
  }
}

void UpdateStepDirection(void) {
  bool reverse = READ_BIT(DIR_GPIO_Port->IDR, DIR_Pin) == 0;
  __disable_irq();
  UpdatePositionCounters();
  position.step_reverse = reverse;
  __enable_irq();
  ReportStepReverse();
}

static void FillQuadChunk(uint32_t* chunk, int8_t* out_delta) {
  UpdatePositionCounters();

  bool is_freewheeling = stall_tripped || !GetStepEnabled();
  int32_t commanded_pos = StepToEncoderPosition(position.step_pos);
  int32_t error = commanded_pos - planned_encoder_pos;

  int count_to_emit = 0;
  int dir = 0;
  float target_velocity = 0.0f;

  if (is_freewheeling) {
    float v_free = (float) load_tension * config.kfree;
    if (v_free > (float) config.torque_v_max) {
      v_free = (float) config.torque_v_max;
    } else if (v_free < -(float) config.torque_v_max) {
      v_free = -(float) config.torque_v_max;
    }

    if (v_free != 0.0f) {
      dir = (v_free > 0.0f) ? 1 : -1;
      count_to_emit = CHUNK_SIZE;
      target_velocity = v_free / 1000.0f;
    } else {
      dir = 0;
      count_to_emit = 0;
      target_velocity = 0.0f;
    }
  } else {
    float input_rate = 0.0f;
    if ((now - last_step_time) <= 50 && step_period_cnt > 0) {
      input_rate = (48000.0f / (float) step_period_cnt) * ((float) config.epr / (float) config.spr);
      if (position.step_reverse) {
        input_rate = -input_rate;
      }
    }

    target_velocity = config.kff * input_rate + config.kp * (float) error;

    if (error != 0) {
      dir = (error > 0) ? 1 : -1;
      float speed_hz = fabsf(target_velocity) * 1000.0f;
      int32_t t_motor = CalcMotorTorque(speed_hz);
      int32_t t_net = t_motor + dir * load_tension;

      if (t_net >= 0) {
        // Sufficient torque: motor drives normally toward target
        int abs_error = (error > 0) ? error : -error;
        count_to_emit = (abs_error < CHUNK_SIZE) ? abs_error : CHUNK_SIZE;
      } else {
        // Torque deficit: motor cannot advance in commanded direction
        int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;
        if (abs_tension > config.torque_t0) {
          dir = (load_tension > 0) ? 1 : -1;
          count_to_emit = CHUNK_SIZE;
          float v_slip = (float)(abs_tension - config.torque_t0) * config.kfree;
          if (v_slip > (float) config.torque_v_max) v_slip = (float) config.torque_v_max;
          target_velocity = (dir > 0) ? (v_slip / 1000.0f) : -(v_slip / 1000.0f);
        } else {
          dir = 0;
          count_to_emit = 0;
        }

        // Under torque deficit, motor stalls and accumulates lag against commanded steps
        uint32_t lag = (error >= 0) ? (uint32_t) error : (uint32_t)(-error);
        if (config.stall_threshold > 0 && lag >= config.stall_threshold) {
          stall_tripped = true;
          ReportStallTrip();
          is_freewheeling = true;
        }
      }
    } else {
      // error == 0: motor is at target
      int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;
      if (abs_tension > config.torque_t0) {
        dir = (load_tension > 0) ? 1 : -1;
        count_to_emit = CHUNK_SIZE;
        float v_slip = (float)(abs_tension - config.torque_t0) * config.kfree;
        if (v_slip > (float) config.torque_v_max) v_slip = (float) config.torque_v_max;
        target_velocity = (dir > 0) ? (v_slip / 1000.0f) : -(v_slip / 1000.0f);

        uint32_t lag = abs(commanded_pos - planned_encoder_pos);
        if (config.stall_threshold > 0 && lag >= config.stall_threshold) {
          stall_tripped = true;
          ReportStallTrip();
          is_freewheeling = true;
        }
      } else {
        dir = 0;
        count_to_emit = 0;
      }
    }
  }

  for (int i = 0; i < count_to_emit; i++) {
    if (dir > 0) {
      current_quad_state = (current_quad_state + 1) & 3;
    } else {
      current_quad_state = (current_quad_state - 1) & 3;
    }
    chunk[i] = QUAD_BSRR_STATES[current_quad_state];
  }

  for (int i = count_to_emit; i < CHUNK_SIZE; i++) {
    chunk[i] = QUAD_BSRR_STATES[current_quad_state];
  }

  *out_delta = (dir > 0) ? count_to_emit : (dir < 0) ? -count_to_emit : 0;
  planned_encoder_pos += *out_delta;

  // Calculate pacing frequency
  float abs_rate = fabsf(target_velocity);
  if (abs_rate < 0.1f) {
    abs_rate = 0.1f;
  }
  if (abs_rate > 100.0f) {
    abs_rate = 100.0f;
  }

  uint32_t ticks = (uint32_t)(48000.0f / abs_rate);
  if (ticks < 480) {
    ticks = 480; // max 100 kHz
  }

  uint32_t psc = 0;
  if (ticks > 65536) {
    psc = (ticks >> 16);
    ticks = ticks / (psc + 1);
  }
  if (ticks > 65536) ticks = 65536;

  TIM3->PSC = (uint16_t) psc;
  TIM3->ARR = (uint16_t)(ticks - 1);
}

void Motion_Start(void) {
  if (motion_active) return;

  UpdatePositionCounters();
  int32_t commanded = StepToEncoderPosition(position.step_pos);
  int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;

  bool should_start = false;
  if (stall_tripped || !GetStepEnabled()) {
    should_start = (load_tension != 0);
  } else {
    should_start = (commanded != position.encoder_pos) || (position.step_dcnt != 0) || (abs_tension > config.torque_t0);
  }

  if (!should_start) {
    return;
  }

  motion_active = true;
  planned_encoder_pos = position.encoder_pos;

  FillQuadChunk(&quad_buffer[0], &half_0_delta);
  FillQuadChunk(&quad_buffer[CHUNK_SIZE], &half_1_delta);

  if (half_0_delta == 0 && half_1_delta == 0 && !stall_tripped && GetStepEnabled() && abs_tension <= config.torque_t0 && position.step_dcnt == 0 && commanded == position.encoder_pos) {
    motion_active = false;
    return;
  }

  DMA1_Channel3->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF3;
  DMA1_Channel3->CNDTR = TOTAL_BUFFER_SIZE;
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  TIM3->CNT = 0;
  TIM3->CR1 |= TIM_CR1_CEN;
}

void Motion_Wakeup_Handler(void) {
  TIM2->SR = ~TIM_SR_CC1IF;
  TIM2->DIER &= ~TIM_DIER_CC1IE;

  UpdatePositionCounters();
  int32_t commanded = StepToEncoderPosition(position.step_pos);
  int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;

  bool should_start = false;
  if (stall_tripped || !GetStepEnabled()) {
    should_start = (load_tension != 0);
  } else {
    should_start = (commanded != planned_encoder_pos) || (position.step_dcnt != 0) || (abs_tension > config.torque_t0);
  }

  if (should_start) {
    Motion_Start();
  } else {
    if (GetStepEnabled()) {
      TIM2->SR = ~TIM_SR_CC1IF;
      TIM2->DIER |= TIM_DIER_CC1IE;
    }
  }
}

static void CheckMotionIdle(void) {
  if (half_0_delta == 0 && half_1_delta == 0) {
    if (stall_tripped || !GetStepEnabled()) {
      if (load_tension == 0) {
        TIM3->CR1 &= ~TIM_CR1_CEN;
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
        motion_active = false;
        if (GetStepEnabled()) {
          TIM2->SR = ~TIM_SR_CC1IF;
          TIM2->DIER |= TIM_DIER_CC1IE;
        }
      }
      return;
    }
    int32_t commanded = StepToEncoderPosition(position.step_pos);
    int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;
    if (commanded == position.encoder_pos && position.encoder_pos == planned_encoder_pos && position.step_dcnt == 0 && abs_tension <= config.torque_t0) {
      TIM3->CR1 &= ~TIM_CR1_CEN;
      DMA1_Channel3->CCR &= ~DMA_CCR_EN;
      motion_active = false;
      TIM2->SR = ~TIM_SR_CC1IF;
      TIM2->DIER |= TIM_DIER_CC1IE;
    }
  }
}

void DMA_HalfTransfer_Handler(void) {
  position.encoder_pos += half_0_delta;
  FillQuadChunk(&quad_buffer[0], &half_0_delta);
  CheckMotionIdle();
}

void DMA_TransferComplete_Handler(void) {
  position.encoder_pos += half_1_delta;
  FillQuadChunk(&quad_buffer[CHUNK_SIZE], &half_1_delta);
  CheckMotionIdle();
}

void Motion_Init(void) {
  current_quad_state = 0;
  half_0_delta = 0;
  half_1_delta = 0;
  motion_active = false;
  planned_encoder_pos = position.encoder_pos;

  for (int i = 0; i < TOTAL_BUFFER_SIZE; i++) {
    quad_buffer[i] = QUAD_BSRR_STATES[0];
  }

  GPIOB->BSRR = QUAD_BSRR_STATES[0];

  TIM3->CR1 &= ~TIM_CR1_CEN;
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;

  TIM2->SR = 0;
  TIM2->DIER = TIM_DIER_CC1DE | TIM_DIER_CC1IE;
}

void UpdateTick(void) {
  ++now;
  UpdateLEDs();

  if (!motion_active) {
    uint16_t step_cnt = UINT16_MAX - (uint16_t) DMA1_Channel5->CNDTR;
    int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;
    bool should_start = false;
    if (stall_tripped || !GetStepEnabled()) {
      should_start = (load_tension != 0);
    } else {
      int32_t commanded = StepToEncoderPosition(position.step_pos);
      should_start = (commanded != position.encoder_pos) || (step_cnt != position.step_cnt_prev) || (abs_tension > config.torque_t0);
    }
    if (should_start) {
      if (GetStepEnabled()) {
        TIM2->DIER &= ~TIM_DIER_CC1IE;
      }
      Motion_Start();
    }
  }
}

void SetLimit1(bool active) {
  HAL_GPIO_WritePin(LIM1_GPIO_Port, LIM1_Pin, active);
  ReportLimit1();
}

void SetLimit2(bool active) {
  HAL_GPIO_WritePin(LIM2_GPIO_Port, LIM2_Pin, active);
  ReportLimit2();
}

bool GetLimit1(void) {
  return HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin);
}

bool GetLimit2(void) {
  return HAL_GPIO_ReadPin(LIM2_GPIO_Port, LIM2_Pin);
}

void ReportLimit1(void) {
  ReportU8("lim1", GetLimit1());
}

void ReportLimit2(void) {
  ReportU8("lim2", GetLimit2());
}

uint32_t HAL_GetTick(void) {
  return now;
}

void InitSystemClock(void) {
  if (1) {
    // Enable HSI48
    __HAL_RCC_HSI48_ENABLE();
    while(!__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY));

    // Configure Flash latency (required for 48MHz).
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);
    while (!READ_BIT(FLASH->ACR, FLASH_LATENCY_1));

    // Select HSI48 as system clock source.
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI48);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48);

    // HAL_RCCEx_PeriphCLKConfig
    __HAL_RCC_USB_CONFIG(RCC_USBCLKSOURCE_HSI48);

    // Update SystemCoreClock
    SystemCoreClock = 48000000;

    // Configure the source of time base considering new system clocks settings
    HAL_InitTick(TICK_INT_PRIORITY);
  } else {
    SystemClock_Config();
  }
}

static void InitPeripherals(void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN
              |  RCC_AHBENR_GPIOBEN
              |  RCC_AHBENR_DMA1EN
              ;

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN
               |  RCC_APB1ENR_TIM3EN
               ;

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN
               |  RCC_APB2ENR_TIM16EN
               |  RCC_APB2ENR_TIM17EN
               ;

  GPIOA->AFR[0] = (2 << GPIO_AFRL_AFSEL5_Pos)  // PA5 alternate function TIM2 ETR (PUL)
                | (5 << GPIO_AFRL_AFSEL6_Pos)  // PA6 alternate function TIM16 (LED_G)
                | (5 << GPIO_AFRL_AFSEL7_Pos)  // PA7 alternate function TIM17 (LED_R)
                ;

  GPIOA->AFR[1] = (0 << GPIO_AFRH_AFSEL13_Pos)  // PA13 alternate function SWDIO (SWDIO)
                | (0 << GPIO_AFRH_AFSEL14_Pos)  // PA14 alternate function SWCLK (SWCLK)
                ;

  GPIOB->AFR[0] = 0; // PB4 and PB5 are GPIO outputs, no alternate function

  GPIOA->MODER = (0 << GPIO_MODER_MODER3_Pos)   // PA3 input (ENA)
               | (0 << GPIO_MODER_MODER4_Pos)   // PA4 input (DIR)
               | (2 << GPIO_MODER_MODER5_Pos)   // PA5 alternate function TIM2 ETR (PUL)
               | (2 << GPIO_MODER_MODER6_Pos)   // PA6 alternate function TIM16 (LED_G)
               | (2 << GPIO_MODER_MODER7_Pos)   // PA7 alternate function TIM17 (LED_R)
               | (2 << GPIO_MODER_MODER13_Pos)  // PA13 alternate function SWDIO (SWDIO)
               | (2 << GPIO_MODER_MODER14_Pos)  // PA14 alternate function SWCLK (SWCLK)
               ;

  GPIOB->MODER = (1 << GPIO_MODER_MODER0_Pos)  // PB0 output (LIM1)
               | (1 << GPIO_MODER_MODER1_Pos)  // PB1 output (LIM2)
               | (1 << GPIO_MODER_MODER4_Pos)  // PB4 output (EA)
               | (1 << GPIO_MODER_MODER5_Pos)  // PB5 output (EB)
               ;

  GPIOB->OSPEEDR = (3 << GPIO_OSPEEDR_OSPEEDR4_Pos)  // PB4 high speed
                 | (3 << GPIO_OSPEEDR_OSPEEDR5_Pos); // PB5 high speed

  GPIOB->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5;     // Initialize EA=0, EB=0

  GPIOA->PUPDR = (1 << GPIO_PUPDR_PUPDR4_Pos)  // Enable internal pull-up on PA4
               | (1 << GPIO_PUPDR_PUPDR5_Pos); // Enable internal pull-up on PA5

  SYSCFG->EXTICR[0] = 0;  // EXTI0-3 from PORTA (ENA on EXTI3)
  SYSCFG->EXTICR[1] = 0;  // EXTI4-7 from PORTA (DIR on EXTI4)

  // DMA1 Channel 5: Transfers TIM2->CCR1 captured period to step_period_cnt
  DMA1_Channel5->CPAR = (uint32_t) &TIM2->CCR1;
  DMA1_Channel5->CMAR = (uint32_t) &step_period_cnt;
  DMA1_Channel5->CNDTR = UINT16_MAX;
  DMA1_Channel5->CCR = (0b10 << DMA_CCR_MSIZE_Pos)  // 32-bit memory
                     | (0b10 << DMA_CCR_PSIZE_Pos)  // 32-bit peripheral
                     | DMA_CCR_CIRC                 // Circular mode
                     | DMA_CCR_EN;                  // Start DMA

  // TIM2 measures pulse period on the PUL pin.
  TIM2->PSC = 0;
  TIM2->ARR = UINT32_MAX;
  TIM2->CCMR1 = (0b01 << TIM_CCMR1_CC1S_Pos)   // CC1 channel is configured as input, IC1 is mapped on TI1
              | (0b10 << TIM_CCMR1_CC2S_Pos);  // CC2 channel is configured as input, IC2 is mapped on TI1.
  TIM2->CCER = TIM_CCER_CC1P                   // Invert polarity
             | TIM_CCER_CC1E                   // Period captured in CCR1
             | TIM_CCER_CC2E;                  // Pulse width captured in CCR2
  TIM2->SMCR = (0b101 << TIM_SMCR_TS_Pos)      // Filtered Timer Input 1 (TI1FP1)
             | (0b100 << TIM_SMCR_SMS_Pos);    // Reset mode
  TIM2->DIER = TIM_DIER_CC1DE | TIM_DIER_CC1IE;// Link DMA to CCR1 + arm CC1 interrupt for wakeup
  TIM2->CR1 = TIM_CR1_CEN;

  // TIM3 acts as periodic heartbeat triggering DMA1 Channel 3 on Update Events
  TIM3->PSC = 0;
  TIM3->ARR = 480 - 1;                          // Default rate (100 kHz)
  TIM3->CCMR1 = 0;
  TIM3->CCER = 0;
  TIM3->DIER = TIM_DIER_UDE;                    // Trigger DMA on update event
  TIM3->CR2 = 0;
  TIM3->CR1 = TIM_CR1_ARPE;                     // Auto-reload preload enabled, CEN=0 initially

  // DMA1 Channel 3: Streams quad_buffer states to GPIOB->BSRR
  DMA1_Channel3->CPAR = (uint32_t) &GPIOB->BSRR;
  DMA1_Channel3->CMAR = (uint32_t) quad_buffer;
  DMA1_Channel3->CNDTR = TOTAL_BUFFER_SIZE;
  DMA1_Channel3->CCR = (0b10 << DMA_CCR_PL_Pos)     // High priority
                     | (0b10 << DMA_CCR_MSIZE_Pos)  // 32-bit memory
                     | (0b10 << DMA_CCR_PSIZE_Pos)  // 32-bit peripheral
                     | DMA_CCR_MINC                 // Memory increment
                     | DMA_CCR_CIRC                 // Circular mode
                     | DMA_CCR_DIR                  // Memory to peripheral
                     | DMA_CCR_HTIE                 // Half-transfer interrupt
                     | DMA_CCR_TCIE;                // Transfer-complete interrupt

  // TIM16 is used to control LED_G
  TIM16->PSC = 8 - 1;                           // Prescaler: count at 6MHz
  TIM16->ARR = 256 - 1;                         // Auto-reload: update at 23.4375kHz
  TIM16->CCR1 = 0;
  TIM16->CCMR1 = (0b110 << TIM_CCMR1_OC1M_Pos)  // PWM mode 1
               | TIM_CCMR1_OC1PE;               // Preload enable
  TIM16->CCER = TIM_CCER_CC1E;                  // Enable CH1 output
  TIM16->BDTR = TIM_BDTR_MOE;                   // Main output enable
  TIM16->CR1 = TIM_CR1_CEN;

  // TIM17 is used to control LED_R
  TIM17->PSC = 8 - 1;                           // Prescaler: count at 6MHz
  TIM17->ARR = 256 - 1;                         // Auto-reload: update at 23.4375kHz
  TIM17->CCR1 = 0;
  TIM17->CCMR1 = (0b110 << TIM_CCMR1_OC1M_Pos)  // PWM mode 1
               | TIM_CCMR1_OC1PE;               // Preload enable
  TIM17->CCER = TIM_CCER_CC1E;                  // Enable CH1 output
  TIM17->BDTR = TIM_BDTR_MOE;                   // Main output enable
  TIM17->CR1 = TIM_CR1_CEN;

  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_SetPriority(EXTI2_3_IRQn, 2);
  NVIC_SetPriority(EXTI4_15_IRQn, 2);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(EXTI2_3_IRQn);
  NVIC_EnableIRQ(EXTI4_15_IRQn);

  EXTI->RTSR = ENA_Pin | DIR_Pin;  // Enable rising trigger
  EXTI->FTSR = ENA_Pin | DIR_Pin;  // Enable falling trigger
  EXTI->IMR  = ENA_Pin | DIR_Pin;  // Enable interrupt
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

  /* USER CODE BEGIN SysInit */

  InitSystemClock();
  InitPeripherals();

  USBD_Interface_fops_FS.Receive = USB_ReceiveCallback;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  UpdatePositionCounters();
  position.step_pos = 0;
  position.encoder_pos = 0;
  UpdateStepDirection();
  UpdateStepEnabled();
  Motion_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_output = 0;
  while (1) {
    if (usb_input_size) {
      CmdProcessInput((char*) usb_input_data, usb_input_size);
      USB_ReceiveReady();
    }

    if (config.odr != 0 && now != last_output && (now - last_output) % config.odr == 0) {
      last_output = now;
      ReportEncoderPosition();
    }

    USB_Flush();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    __WFI();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

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
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 20 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 20 - 1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100 - 1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LIM1_Pin|LIM2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENA_Pin DIR_Pin */
  GPIO_InitStruct.Pin = ENA_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM1_Pin LIM2_Pin */
  GPIO_InitStruct.Pin = LIM1_Pin|LIM2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
