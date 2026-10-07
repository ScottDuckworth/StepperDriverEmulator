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
#include "config_store.h"
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

static PositionCounters_t position;

#define STEP_BUF_SIZE 32
static volatile uint32_t step_period_buf[STEP_BUF_SIZE];
static uint16_t step_buf_tail = 0;
static uint32_t blanking_accum = 168; // Primed for standstill
static q12_t step_blank_us = Q12_INIT_RATIO(7, 2); // Default 3.5 us blanking (7/2 us)
static uint32_t min_blanking_ticks = 168; // 3.5 us @ 48 MHz
static volatile uint32_t step_period_cnt = 0;
static volatile int32_t load_tension = 0;
static volatile bool stall_tripped = false;
static volatile bool blink_mode = false;
static volatile uint32_t last_step_time = 0;

static uint32_t quad_buffer[TOTAL_BUFFER_SIZE];
static uint8_t current_quad_state = 0;
static int8_t half_0_delta = 0;
static int8_t half_1_delta = 0;
static volatile bool motion_active = false;
static volatile int64_t planned_encoder_pos = 0;
static volatile uint8_t startup_sync_count = 0;
static volatile int32_t current_motion_velocity = 0;


#define usb_output_data UserTxBufferFS
#define usb_input_data UserRxBufferFS

static volatile uint16_t usb_output_size;
static volatile uint16_t usb_input_size;

static EmulatorConfig_t config = DEFAULT_EMULATOR_CONFIG;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

EmulatorConfig_t* GetConfig(void) {
  return &config;
}

bool SaveConfig(void) {
  return ConfigStore_Save(ConfigStore_GetStm32FlashDriver(), CONFIG_FLASH_PAGE_ADDR, &config.persistent);
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

void ReportQ12(const char* var, q12_t value) {
  char formatted[20];
  MathUtil_FormatQ12(formatted, sizeof(formatted), value, 4);
  WriteString(var);
  WriteString(" ");
  WriteString(formatted);
  WriteString("\r\n");
}

void ReportQ16(const char* var, q16_t value) {
  char formatted[20];
  MathUtil_FormatQ16(formatted, sizeof(formatted), value, 4);
  WriteString(var);
  WriteString(" ");
  WriteString(formatted);
  WriteString("\r\n");
}

void ReportI64(const char* var, int64_t value) {
  char digits[24];
  MathUtil_FormatI64(digits, sizeof(digits), value);
  WriteString(var);
  WriteString(" ");
  WriteString(digits);
  WriteString("\r\n");
}

void ReportI32(const char* var, int32_t value) {
  char digits[16];
  MathUtil_FormatI32(digits, sizeof(digits), value);
  WriteString(var);
  WriteString(" ");
  WriteString(digits);
  WriteString("\r\n");
}

void ReportU32(const char* var, uint32_t value) {
  char digits[16];
  MathUtil_FormatU32(digits, sizeof(digits), value);
  WriteString(var);
  WriteString(" ");
  WriteString(digits);
  WriteString("\r\n");
}

void ReportU16(const char* var, uint16_t value) {
  char digits[8];
  MathUtil_FormatU16(digits, sizeof(digits), value);
  WriteString(var);
  WriteString(" ");
  WriteString(digits);
  WriteString("\r\n");
}

void ReportU8(const char* var, uint8_t value) {
  char digits[8];
  MathUtil_FormatU8(digits, sizeof(digits), value);
  WriteString(var);
  WriteString(" ");
  WriteString(digits);
  WriteString("\r\n");
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
  uint8_t r = 0, g = 0;
  EvalLEDState(now, blink_mode, stall_tripped, GetStepEnabled(), last_step_time, &r, &g);
  SetLED(r, g);
}

int32_t GetTension(void) {
  return load_tension;
}

void SetTension(int32_t tension) {
  load_tension = tension;
  ReportTension();
  MaybeStartMotion();
}

void ReportTension(void) {
  ReportI32("t", GetTension());
}

void GetDefaultHardwareName(char* out_name, size_t max_len) {
  if (!out_name || max_len == 0) return;
#if defined(UNIT_TEST)
  strncpy(out_name, "001122334455", max_len);
  out_name[max_len - 1] = '\0';
#else
  uint32_t deviceserial0 = *(uint32_t *) UID_BASE;
  uint32_t deviceserial1 = *(uint32_t *) (UID_BASE + 4U);
  uint32_t deviceserial2 = *(uint32_t *) (UID_BASE + 8U);
  deviceserial0 += deviceserial2;

  static const char hex_digits[] = "0123456789ABCDEF";
  char serial[13];
  for (int i = 0; i < 8; i++) {
    serial[i] = hex_digits[(deviceserial0 >> (28 - i * 4)) & 0xF];
  }
  for (int i = 0; i < 4; i++) {
    serial[8 + i] = hex_digits[(deviceserial1 >> (28 - i * 4)) & 0xF];
  }
  serial[12] = '\0';
  strncpy(out_name, serial, max_len);
  out_name[max_len - 1] = '\0';
#endif
}

const char* GetName(void) {
  if (config.persistent.name[0] == '\0') {
    GetDefaultHardwareName(config.persistent.name, sizeof(config.persistent.name));
  }
  return config.persistent.name;
}

void SetName(const char* name) {
  if (name && name[0] != '\0') {
    strncpy(config.persistent.name, name, sizeof(config.persistent.name) - 1);
    config.persistent.name[sizeof(config.persistent.name) - 1] = '\0';
  } else {
    GetDefaultHardwareName(config.persistent.name, sizeof(config.persistent.name));
  }
  ReportName();
}

void ReportName(void) {
  ReportString("name", GetName());
}

uint16_t GetOdr(void) {
  return config.persistent.odr;
}

void SetOdr(uint16_t odr) {
  config.persistent.odr = odr;
  ReportOdr();
}

void ReportOdr(void) {
  ReportU16("odr", GetOdr());
}

bool SetRatio(uint16_t spr, uint16_t epr) {
  if (spr == 0 || epr == 0) return false;
  uint16_t g = MathUtil_GCD(spr, epr);
  __disable_irq();
  config.persistent.ratio_spr = spr / g;
  config.persistent.ratio_epr = epr / g;
  ConfigStore_ComputeCachedValues(&config.persistent, &config.cached);
  position.step_rem = 0;
  __enable_irq();
  ReportRatio();
  return true;
}

void GetRatio(uint16_t* out_spr, uint16_t* out_epr) {
  if (out_spr) *out_spr = config.persistent.ratio_spr;
  if (out_epr) *out_epr = config.persistent.ratio_epr;
}

void ReportRatio(void) {
  char buf[32];
  int size = snprintf(buf, sizeof(buf), "ratio %u %u\r\n", config.persistent.ratio_spr, config.persistent.ratio_epr);
  WriteData((uint8_t*) buf, size);
}

void SetTorqueLUT(uint32_t delta_v, uint8_t count, const int32_t* table) {
  if (delta_v == 0 || count < 2 || count > TCURVE_MAX_POINTS || !table) return;
  config.persistent.tcurve_delta_v = delta_v;
  config.persistent.tcurve_point_count = count;
  memcpy(config.persistent.tcurve_table, table, count * sizeof(int32_t));
  ConfigStore_ComputeCachedValues(&config.persistent, &config.cached);
  ReportTorqueLUT();
  MaybeStartMotion();
}

void ReportTorqueLUT(void) {
  WriteString("tlut ");
  char num_buf[16];
  MathUtil_FormatU32(num_buf, sizeof(num_buf), config.persistent.tcurve_delta_v);
  WriteString(num_buf);
  for (uint8_t i = 0; i < config.persistent.tcurve_point_count; ++i) {
    WriteString(" ");
    MathUtil_FormatI32(num_buf, sizeof(num_buf), config.persistent.tcurve_table[i]);
    WriteString(num_buf);
  }
  WriteString("\r\n");
}

uint32_t GetStallThreshold(void) {
  return config.persistent.stall_threshold;
}

void SetStallThreshold(uint32_t threshold) {
  config.persistent.stall_threshold = threshold;
  ReportStallThreshold();
}

void ReportStallThreshold(void) {
  ReportU32("stall", GetStallThreshold());
}

int64_t GetMinStop(void) {
  return config.persistent.minstop;
}

bool SetMinStop(int64_t min_stop) {
  if (min_stop > config.persistent.maxstop) {
    return false;
  }
  config.persistent.minstop = min_stop;
  ReportMinStop();
  MaybeStartMotion();
  return true;
}

void ReportMinStop(void) {
  int64_t stop = GetMinStop();
  if (stop == INT64_MIN) {
    WriteString("minstop none\r\n");
  } else {
    ReportI64("minstop", stop);
  }
}

int64_t GetMaxStop(void) {
  return config.persistent.maxstop;
}

bool SetMaxStop(int64_t max_stop) {
  if (max_stop < config.persistent.minstop) {
    return false;
  }
  config.persistent.maxstop = max_stop;
  ReportMaxStop();
  MaybeStartMotion();
  return true;
}

void ReportMaxStop(void) {
  int64_t stop = GetMaxStop();
  if (stop == INT64_MAX) {
    WriteString("maxstop none\r\n");
  } else {
    ReportI64("maxstop", stop);
  }
}

q12_t GetKfree(void) {
  return config.persistent.kfree;
}

void SetKfree(q12_t k) {
  config.persistent.kfree = k;
  ConfigStore_ComputeCachedValues(&config.persistent, &config.cached);
  ReportKfree();
}

void ReportKfree(void) {
  ReportQ12("kfree", GetKfree());
}

bool GetStallTrip(void) {
  return stall_tripped;
}

void ReportStallTrip(void) {
  ReportU8("stall_trip", GetStallTrip());
}

q12_t GetKp(void) {
  return config.persistent.kp;
}

void SetKp(q12_t kp) {
  config.persistent.kp = kp;
  ConfigStore_ComputeCachedValues(&config.persistent, &config.cached);
  ReportKp();
}

void ReportKp(void) {
  ReportQ12("kp", GetKp());
}

q12_t GetKff(void) {
  return config.persistent.kff;
}

void SetKff(q12_t kff) {
  config.persistent.kff = kff;
  ConfigStore_ComputeCachedValues(&config.persistent, &config.cached);
  ReportKff();
}

void ReportKff(void) {
  ReportQ12("kff", GetKff());
}

q12_t GetStepBlanking(void) {
  return step_blank_us;
}

void SetStepBlanking(q12_t blank_us) {
  if (blank_us.raw < Q12_RAW_RATIO(1, 2)) blank_us.raw = Q12_RAW_RATIO(1, 2);       // 0.5 us min
  if (blank_us.raw > Q12_RAW_INT(1000)) blank_us.raw = Q12_RAW_INT(1000);           // 1000.0 us max
  step_blank_us = blank_us;
  min_blanking_ticks = (uint32_t)((blank_us.raw * 3) >> 8);
  if (min_blanking_ticks < 24) min_blanking_ticks = 24;

  // Adapt hardware timer filter IC1F/CKD to match blanking window
  if (blank_us.raw <= Q12_RAW_INT(5)) { // <= 5.0 us
    TIM2->CR1 &= ~TIM_CR1_CKD; // CKD = 0 (div 1)
    TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F))
                | (0b1000 << TIM_CCMR1_IC1F_Pos)  // 1.0 us filter (fDTS/8, N=6)
                | (0b1000 << TIM_CCMR1_IC2F_Pos);
  } else if (blank_us.raw <= Q12_RAW_INT(15)) { // <= 15.0 us
    TIM2->CR1 &= ~TIM_CR1_CKD; // CKD = 0 (div 1)
    TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F))
                | (0b1111 << TIM_CCMR1_IC1F_Pos)  // 5.33 us filter (fDTS/32, N=8)
                | (0b1111 << TIM_CCMR1_IC2F_Pos);
  } else {
    TIM2->CR1 |= TIM_CR1_CKD_1; // CKD = div4
    TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F))
                | (0b1111 << TIM_CCMR1_IC1F_Pos)  // 21.33 us filter (fDTS/32, N=8, CKD=4)
                | (0b1111 << TIM_CCMR1_IC2F_Pos);
  }
  ReportStepBlanking();
}

void ReportStepBlanking(void) {
  ReportQ12("blank", GetStepBlanking());
}

int64_t GetEncoderPosition(void) {
  __disable_irq();
  int64_t pos = position.encoder_pos;
  __enable_irq();
  return pos;
}

void SetEncoderPosition(int64_t pos) {
  __disable_irq();
  uint16_t head = (STEP_BUF_SIZE - (uint16_t) DMA1_Channel5->CNDTR) & (STEP_BUF_SIZE - 1);
  step_buf_tail = head;
  blanking_accum = min_blanking_ticks;
  Position_RealignCounters(&position, pos, &config, head);
  planned_encoder_pos = pos;
  half_0_delta = 0;
  half_1_delta = 0;
  startup_sync_count = 0;
  step_period_cnt = 0;
  for (int i = 0; i < TOTAL_BUFFER_SIZE; i++) {
    quad_buffer[i] = Quadrature_GetBsrrValue(current_quad_state);
  }
  TIM3->CR1 &= ~TIM_CR1_CEN;
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;
  motion_active = false;
  current_motion_velocity = 0;
  TIM2->SR = ~TIM_SR_CC1IF;
  TIM2->DIER = TIM_DIER_CC1DE | TIM_DIER_CC1IE;
  __enable_irq();
  ReportEncoderPosition();
}

void ReportEncoderPosition(void) {
  ReportI64("pos", GetEncoderPosition());
}

void GetInstantaneousMotionState(int32_t* out_velocity_hz, int32_t* out_motor_torque, int32_t* out_net_torque) {
  __disable_irq();
  bool active = motion_active;
  int32_t vel = active ? current_motion_velocity : 0;
  bool is_free = stall_tripped || !GetStepEnabled();
  int32_t tension = load_tension;
  int64_t enc_pos = position.encoder_pos;
  int64_t cmd_pos = position.commanded_pos;
  __enable_irq();

  int32_t t_motor;
  if (is_free) {
    t_motor = 0;
  } else if (!active) {
    t_motor = config.persistent.tcurve_table[0];
  } else {
    int32_t speed = MathUtil_AbsI32(vel);
    t_motor = Motion_CalcMotorTorque(&config, speed);
  }

  int dir = (vel > 0) ? 1 : ((vel < 0) ? -1 : 0);
  int32_t t_net = Motion_CalcNetTorque(t_motor, dir, tension);

  if (config.persistent.maxstop != INT64_MAX && enc_pos >= config.persistent.maxstop) {
    if (cmd_pos > enc_pos || vel > 0 || (is_free && tension > 0)) {
      t_net = 0;
    }
  } else if (config.persistent.minstop != INT64_MIN && enc_pos <= config.persistent.minstop) {
    if (cmd_pos < enc_pos || vel < 0 || (is_free && tension < 0)) {
      t_net = 0;
    }
  }

  if (out_velocity_hz) *out_velocity_hz = vel;
  if (out_motor_torque) *out_motor_torque = t_motor;
  if (out_net_torque) *out_net_torque = t_net;
}

void ReportPvt(void) {
  int64_t pos = GetEncoderPosition();
  int32_t vel = 0, t_motor = 0, t_net = 0;
  GetInstantaneousMotionState(&vel, &t_motor, &t_net);

  char pos_str[24];
  MathUtil_FormatI64(pos_str, sizeof(pos_str), pos);

  char tail[48];
  snprintf(tail, sizeof(tail), " %ld %ld %ld\r\n", (long) vel, (long) t_motor, (long) t_net);

  WriteString("pvt ");
  WriteString(pos_str);
  WriteString(tail);
}

bool GetStepReverse(void) {
  return position.step_reverse;
}

void ReportStepReverse(void) {
  ReportU8("rev", GetStepReverse());
}

static inline bool IsStepEnabled(void) {
  return READ_BIT(ENA_GPIO_Port->IDR, ENA_Pin) != 0;
}

bool GetStepEnabled(void) {
  return IsStepEnabled();
}

void ReportStepEnabled(void) {
  ReportU8("ena", GetStepEnabled());
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
    uint16_t head = (STEP_BUF_SIZE - (uint16_t) DMA1_Channel5->CNDTR) & (STEP_BUF_SIZE - 1);
    step_buf_tail = head;
    blanking_accum = min_blanking_ticks;
    Position_RealignCounters(&position, position.encoder_pos, &config, head);
    planned_encoder_pos = position.encoder_pos;
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
  MaybeStartMotion();
}

static void UpdatePositionCounters(void) {
  uint16_t head = (STEP_BUF_SIZE - (uint16_t) DMA1_Channel5->CNDTR) & (STEP_BUF_SIZE - 1);
  uint16_t valid_steps = 0;

  while (step_buf_tail != head) {
    uint32_t period = step_period_buf[step_buf_tail];
    step_buf_tail = (step_buf_tail + 1) & (STEP_BUF_SIZE - 1);

    uint32_t valid_period = 0;
    if (Position_FilterStepWithBlanking(period, min_blanking_ticks, &blanking_accum, &valid_period)) {
      valid_steps++;
      step_period_cnt = valid_period;
    }
  }

  position.step_dcnt = valid_steps;
  if (valid_steps != 0) {
    last_step_time = now;
    int32_t step_delta = position.step_reverse ? -(int32_t) valid_steps : (int32_t) valid_steps;
    int32_t count_delta = Position_ConvertStepDeltaToCounts(step_delta, config.persistent.ratio_spr, config.persistent.ratio_epr, &position.step_rem);
    position.commanded_pos += count_delta;
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

bool MaybeStartMotion(void) {
  if (motion_active) return false;

  UpdatePositionCounters();
  bool is_freewheeling = stall_tripped || !GetStepEnabled();
  Motion_StartRequest_t req = {
      .commanded_pos = position.commanded_pos,
      .encoder_pos = position.encoder_pos,
      .load_tension = load_tension,
      .step_dcnt = position.step_dcnt,
      .now = now,
      .last_step_time = last_step_time,
      .step_reverse = position.step_reverse,
      .is_freewheeling = is_freewheeling,
      .stall_tripped = stall_tripped
  };
  Motion_StartBuffers_t buf = {
      .chunk_size = CHUNK_SIZE,
      .quad_buffer = quad_buffer,
      .inout_quad_state = &current_quad_state
  };
  Motion_StartResult_t res;
  if (!Motion_PrepareStart(&config, &req, &buf, &res)) {
    return false;
  }

  motion_active = true;
  startup_sync_count = 1;
  current_motion_velocity = res.target_velocity;
  planned_encoder_pos = res.planned_encoder_pos;
  half_0_delta = res.half_0_delta;
  half_1_delta = res.half_1_delta;

  if (res.stall_trip_event) {
    stall_tripped = true;
    ReportStallTrip();
  }

  DMA1_Channel3->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF3;
  DMA1_Channel3->CNDTR = TOTAL_BUFFER_SIZE;
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  // Arm TIM3 with Chunk 0 pacing, since Chunk 0 is the chunk transmitted first by DMA!
  TIM3->PSC = res.psc;
  TIM3->ARR = res.arr;
  TIM3->CNT = 0;
  TIM3->CR1 |= TIM_CR1_CEN;
  TIM3->EGR = TIM_EGR_UG;

  if (GetStepEnabled()) {
    TIM2->SR = 0;
    TIM2->DIER |= TIM_DIER_CC1IE;
  }
  return true;
}

void Motion_Wakeup_Handler(void) {
  TIM2->SR = 0;

  UpdatePositionCounters();

  if (motion_active) {
    if (position.step_dcnt == 0) {
      return;
    }

    if (startup_sync_count > 0) {
      startup_sync_count--;
    }

    uint32_t cndtr = DMA1_Channel3->CNDTR;
    uint32_t* chunk = (cndtr > CHUNK_SIZE) ? &quad_buffer[CHUNK_SIZE] : &quad_buffer[0];
    int8_t* out_delta = (cndtr > CHUNK_SIZE) ? &half_1_delta : &half_0_delta;

    uint32_t period_cnt = (startup_sync_count > 0) ? 0 : step_period_cnt;
    Motion_PlanStepRequest_t req = {
        .commanded_pos = position.commanded_pos,
        .planned_encoder_pos = planned_encoder_pos,
        .load_tension = load_tension,
        .now = HAL_GetTick(),
        .last_step_time = last_step_time,
        .step_period_cnt = period_cnt,
        .step_reverse = position.step_reverse,
        .is_freewheeling = stall_tripped || !IsStepEnabled(),
        .stall_tripped = stall_tripped,
        .chunk_size = CHUNK_SIZE
    };
    Motion_PlanStepResult_t res;
    Motion_PlanStep(&config, &req, &res);
    current_motion_velocity = res.target_velocity;

    if (res.stall_trip_event) {
      stall_tripped = true;
      ReportStallTrip();
    }

    if (*out_delta == 0) {
      *out_delta = Quadrature_GenerateChunk(chunk, CHUNK_SIZE, &current_quad_state, res.dir, res.count_to_emit);
      planned_encoder_pos += *out_delta;
    }

    uint32_t pace_velocity = 0;
    if (res.target_velocity == 0) {
      pace_velocity = (uint32_t) MathUtil_MulQ12(1000, config.cached.counts_per_step);
      if (pace_velocity == 0) {
        pace_velocity = 4000;
      }
    } else {
      pace_velocity = (uint32_t) MathUtil_AbsI32(res.target_velocity);
    }
    uint16_t psc = 0;
    uint16_t arr = 0;
    Quadrature_CalcTimerPacing(pace_velocity, &psc, &arr);
    TIM3->PSC = psc;
    TIM3->ARR = arr;
    TIM3->EGR = TIM_EGR_UG;

    if (period_cnt > 0 && period_cnt < 48000 && startup_sync_count == 0) {
      TIM2->DIER &= ~TIM_DIER_CC1IE;
    }
    return;
  }

  if (!MaybeStartMotion()) {
    if (IsStepEnabled()) {
      TIM2->SR = 0;
      TIM2->DIER |= TIM_DIER_CC1IE;
    }
  }
}

/*
 * FillQuadChunk:
 * Service routine invoked by DMA half-transfer and transfer-complete interrupts.
 * Updates position tracking, performs step planning and quadrature pattern generation,
 * updates TIM3 pacing registers for the next chunk, and returns the emitted delta counts.
 *
 * Performance Note:
 * This function intentionally bypasses the higher-level Motion_PlanAndEmitChunk wrapper
 * and directly invokes Motion_PlanStep, Quadrature_GenerateChunk, and Quadrature_CalcTimerPacing.
 * On the ARM Cortex-M0 core (ARMv6-M), delegating to Motion_PlanAndEmitChunk incurs severe register
 * starvation and stack-shuffling overhead (Motion_ChunkRequest_t, Motion_ChunkResult_t, memset,
 * and pointer validations), which consumed ~180 additional cycles (~3.8 us) per chunk. Inlining
 * the dispatch here eliminates that overhead within this critical 50 kHz ISR path.
 */
static int8_t FillQuadChunk(uint32_t* chunk) {
  UpdatePositionCounters();

  bool step_enabled = IsStepEnabled();
  bool is_freewheeling = stall_tripped || !step_enabled;
  uint32_t period_cnt = (startup_sync_count > 0) ? 0 : step_period_cnt;

  Motion_PlanStepRequest_t plan_req = {
      .commanded_pos = position.commanded_pos,
      .planned_encoder_pos = planned_encoder_pos,
      .load_tension = load_tension,
      .now = now,
      .last_step_time = last_step_time,
      .step_period_cnt = period_cnt,
      .step_reverse = position.step_reverse,
      .is_freewheeling = is_freewheeling,
      .stall_tripped = stall_tripped,
      .chunk_size = CHUNK_SIZE
  };

  Motion_PlanStepResult_t plan_res;
  Motion_PlanStep(&config, &plan_req, &plan_res);

  int8_t delta = Quadrature_GenerateChunk(chunk, CHUNK_SIZE, &current_quad_state, plan_res.dir, plan_res.count_to_emit);
  planned_encoder_pos += delta;
  current_motion_velocity = plan_res.target_velocity;

  if (plan_res.stall_trip_event) {
    stall_tripped = true;
    ReportStallTrip();
  }

  uint32_t pace_velocity = 0;
  if (plan_res.target_velocity == 0) {
    pace_velocity = (uint32_t) MathUtil_MulQ12(1000, config.cached.counts_per_step);
    if (pace_velocity == 0) {
      pace_velocity = 4000;
    }
  } else {
    pace_velocity = (uint32_t) MathUtil_AbsI32(plan_res.target_velocity);
  }

  uint16_t psc = 0;
  uint16_t arr = 0;
  Quadrature_CalcTimerPacing(pace_velocity, &psc, &arr);
  TIM3->PSC = psc;
  TIM3->ARR = arr;

  if (step_period_cnt >= 48000 && step_enabled) {
    TIM2->SR = 0;
    TIM2->DIER |= TIM_DIER_CC1IE;
  }

  return delta;
}

static void CheckMotionIdle(void) {
  bool step_enabled = IsStepEnabled();
  bool is_freewheeling = stall_tripped || !step_enabled;

  if (is_freewheeling && load_tension != 0) {
    if (load_tension > 0 && config.persistent.maxstop != INT64_MAX && position.encoder_pos >= config.persistent.maxstop) {
      // Blocked at maxstop
    } else if (load_tension < 0 && config.persistent.minstop != INT64_MIN && position.encoder_pos <= config.persistent.minstop) {
      // Blocked at minstop
    } else {
      return;
    }
  }

  uint32_t elapsed_ms = now - last_step_time;
  if (!is_freewheeling && elapsed_ms < 50) {
    return;
  }

  uint32_t step_timeout_ms = (startup_sync_count == 0)
                                 ? Motion_CalcStepTimeoutMs(step_period_cnt)
                                 : 50;
  if (!is_freewheeling && elapsed_ms < step_timeout_ms) {
    return;
  }

  bool blocked_by_stop = false;
  if (position.commanded_pos > position.encoder_pos && config.persistent.maxstop != INT64_MAX && position.encoder_pos >= config.persistent.maxstop) {
    blocked_by_stop = true;
  } else if (position.commanded_pos < position.encoder_pos && config.persistent.minstop != INT64_MIN && position.encoder_pos <= config.persistent.minstop) {
    blocked_by_stop = true;
  }

  if (!is_freewheeling) {
    if (!blocked_by_stop && position.commanded_pos != position.encoder_pos) {
      return;
    }
    if (position.encoder_pos != planned_encoder_pos || position.step_dcnt != 0) {
      return;
    }
  }

  Motion_StopRequest_t req = {
      .commanded_pos = position.commanded_pos,
      .encoder_pos = position.encoder_pos,
      .planned_encoder_pos = planned_encoder_pos,
      .step_dcnt = position.step_dcnt,
      .load_tension = load_tension,
      .time_since_last_step_ms = elapsed_ms,
      .step_timeout_ms = step_timeout_ms,
      .is_freewheeling = is_freewheeling
  };

  if (Motion_ShouldStop(&config, &req)) {
    TIM3->CR1 &= ~TIM_CR1_CEN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    motion_active = false;
    current_motion_velocity = 0;
    startup_sync_count = 0;
    step_period_cnt = 0;
    if (step_enabled) {
      TIM2->SR = 0;
      TIM2->DIER |= TIM_DIER_CC1IE;
    }
  }
}

void DMA_HalfTransfer_Handler(void) {
  position.encoder_pos += half_0_delta;
  half_0_delta = FillQuadChunk(&quad_buffer[0]);
  if (half_0_delta == 0 && half_1_delta == 0) {
    CheckMotionIdle();
  }
}

void DMA_TransferComplete_Handler(void) {
  position.encoder_pos += half_1_delta;
  half_1_delta = FillQuadChunk(&quad_buffer[CHUNK_SIZE]);
  if (half_0_delta == 0 && half_1_delta == 0) {
    CheckMotionIdle();
  }
}

void InitMotion(void) {
  current_quad_state = 0;
  half_0_delta = 0;
  half_1_delta = 0;
  motion_active = false;
  current_motion_velocity = 0;
  startup_sync_count = 0;
  step_period_cnt = 0;
  planned_encoder_pos = position.encoder_pos;

  for (int i = 0; i < TOTAL_BUFFER_SIZE; i++) {
    quad_buffer[i] = Quadrature_GetBsrrValue(0);
  }

  GPIOB->BSRR = Quadrature_GetBsrrValue(0);

  TIM3->CR1 &= ~TIM_CR1_CEN;
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;

  TIM2->SR = 0;
  TIM2->DIER = TIM_DIER_CC1DE | TIM_DIER_CC1IE;
}

void UpdateTick(void) {
  ++now;
  UpdateLEDs();
  MaybeStartMotion();
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

  // DMA1 Channel 5: Transfers TIM2->CCR1 captured period to step_period_buf circular ring
  DMA1_Channel5->CPAR = (uint32_t) &TIM2->CCR1;
  DMA1_Channel5->CMAR = (uint32_t) step_period_buf;
  DMA1_Channel5->CNDTR = STEP_BUF_SIZE;
  DMA1_Channel5->CCR = (0b10 << DMA_CCR_MSIZE_Pos)  // 32-bit memory
                     | (0b10 << DMA_CCR_PSIZE_Pos)  // 32-bit peripheral
                     | DMA_CCR_MINC                 // Increment memory pointer
                     | DMA_CCR_CIRC                 // Circular mode
                     | DMA_CCR_EN;                  // Start DMA

  // TIM2 measures pulse period on the PUL pin.
  TIM2->PSC = 0;
  TIM2->ARR = UINT32_MAX;
  TIM2->CCMR1 = (0b01 << TIM_CCMR1_CC1S_Pos)   // CC1 channel is configured as input, IC1 is mapped on TI1
              | (0b10 << TIM_CCMR1_CC2S_Pos)   // CC2 channel is configured as input, IC2 is mapped on TI1.
              | (0b1000 << TIM_CCMR1_IC1F_Pos) // Hardware digital filter: fDTS/8, N=6 (1.0 us filter)
              | (0b1000 << TIM_CCMR1_IC2F_Pos);// Hardware digital filter: fDTS/8, N=6 (1.0 us filter)
  TIM2->CCER = TIM_CCER_CC1P                   // Invert polarity
             | TIM_CCER_CC1E                   // Period captured in CCR1
             | TIM_CCER_CC2E;                  // Pulse width captured in CCR2
  TIM2->SMCR = (0b101 << TIM_SMCR_TS_Pos)      // Filtered Timer Input 1 (TI1FP1)
             | (0b100 << TIM_SMCR_SMS_Pos);    // Reset mode
  TIM2->DIER = TIM_DIER_CC1DE | TIM_DIER_CC1IE;// Link DMA to CCR1 + arm CC1 interrupt for wakeup
  TIM2->CR1 = TIM_CR1_CEN;                      // Enable counter (CKD=div1 for 1.0 us filter)

  // TIM3 acts as periodic heartbeat triggering DMA1 Channel 3 on Update Events
  TIM3->PSC = 0;
  TIM3->ARR = 480 - 1;                          // Default rate (100 kHz)
  TIM3->CCMR1 = 0;
  TIM3->CCER = 0;
  TIM3->DIER = TIM_DIER_UDE;                    // Trigger DMA on update event
  TIM3->CR2 = 0;
  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;       // Auto-reload preload enabled, only overflow/underflow generates DMA/interrupt

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

  ConfigStore_Load(ConfigStore_GetStm32FlashDriver(), CONFIG_FLASH_PAGE_ADDR, &config);
  ConfigStore_ComputeCachedValues(&config.persistent, &config.cached);
  if (config.persistent.name[0] == '\0') {
    GetDefaultHardwareName(config.persistent.name, sizeof(config.persistent.name));
  }

  UpdatePositionCounters();
  position.commanded_pos = 0;
  position.encoder_pos = 0;
  position.step_rem = 0;
  UpdateStepDirection();
  UpdateStepEnabled();
  InitMotion();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_output = 0;
  while (1) {
    if (usb_input_size) {
      CmdProcessInput((char*) usb_input_data, usb_input_size);
      USB_ReceiveReady();
    }

    if (config.persistent.odr != 0 && now != last_output && (now - last_output) % config.persistent.odr == 0) {
      last_output = now;
      ReportPvt();
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
