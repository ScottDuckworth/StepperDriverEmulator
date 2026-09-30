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
  uint16_t encoder_cnt_prev;
  bool step_reverse;
} PositionCounters_t;

static PositionCounters_t position;

static volatile uint32_t step_period_cnt;
static volatile int32_t override_target_pos;
static volatile int32_t override_rate;
static volatile int32_t override_start_pos;
static volatile uint32_t override_start_time;
static volatile OverrideState_t override_state;

#define usb_output_data UserTxBufferFS
#define usb_input_data UserRxBufferFS

static uint16_t usb_output_size;
static uint16_t usb_input_size;

static RunConfig_t config = {
    .odr = 1000,
    .epr = 2000,
    .spr = 1000,
};

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
  assert(buf == usb_input_data);
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
  if (result == USBD_OK) {
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

int32_t GetOverrideTarget(void) {
  return override_target_pos;
}

int32_t GetOverrideRate(void) {
  return override_rate;
}

OverrideState_t GetOverrideState(void) {
  return override_state;
}

const char* OverrideStateToString(OverrideState_t state) {
  switch (state) {
    case OverrideEnabling:
      return "enabling";
    case OverrideEnabled:
      return "enabled";
    case OverrideDisabling:
      return "disabling";
    case OverrideDisabled:
      // fall-through
  }
  return "disabled";
}

void ReportOverrideTarget(void) {
  ReportI32("ot", GetOverrideTarget());
}

void ReportOverrideRate(void) {
  ReportI32("or", GetOverrideRate());
}

void ReportOverrideState(void) {
  ReportString("os", OverrideStateToString(GetOverrideState()));
}

static void SetOverrideEnable(void) {
  override_start_pos = position.encoder_pos;
  override_start_time = now;
  override_state = OverrideEnabling;
  ReportOverrideState();
}

void SetOverrideDisable(void) {
  override_start_pos = position.encoder_pos;
  override_start_time = now;
  override_state = OverrideDisabling;
  ReportOverrideState();
}

void SetOverrideTarget(int32_t target) {
  override_target_pos = target;
  ReportOverrideTarget();
  SetOverrideEnable();
}

void SetOverrideRate(int32_t rate) {
  override_rate = rate;
  ReportOverrideRate();
  if (override_state == OverrideEnabling) {
    SetOverrideEnable();
  } else if (override_state == OverrideDisabling) {
    SetOverrideDisable();
  }
}

int32_t GetEncoderPosition(void) {
  return position.encoder_pos;
}

void SetEncoderPosition(int32_t pos) {
  position.encoder_pos = pos;
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
  int32_t full = step_position / config.spr * config.epr;
  int32_t part = step_position % config.spr * config.epr / config.spr;
  return full + part;
}

static int32_t GetOverridenPosision(int32_t current, int32_t target) {
  int32_t rate = override_rate;
  if ((target < current) != (rate < 0)) {
    rate = -rate;
  }
  uint32_t ms = now - override_start_time;
  return override_start_pos + rate * (ms / 1000.0f) + 0.5f;
}

static bool MetOrCrossed(int32_t started, int32_t current, int32_t target) {
  return current == target || ((current < target) != (started < target));
}

static int32_t ApplyOverride(int32_t current, int32_t intent) {
  switch (override_state) {
    case OverrideEnabling: {
      int32_t override = GetOverridenPosision(current, override_target_pos);
      if (!MetOrCrossed(override_start_pos, current, override_target_pos)) {
        return override;
      }
      override_state = OverrideEnabled;
      ReportOverrideState();
      // fall-through
    }
    case OverrideEnabled: {
      return override_target_pos;
    }
    case OverrideDisabling: {
      int32_t override = GetOverridenPosision(current, intent);
      if (!MetOrCrossed(override_start_pos, current, intent)) {
        return override;
      }
      override_state = OverrideDisabled;
      ReportOverrideState();
      // fall-through
    }
    case OverrideDisabled:
      // fall-through
  }
  return intent;
}

void UpdateStepEnabled(void) {
  if (GetStepEnabled()) {
    TIM2->CR1 = TIM_CR1_CEN;
  } else {
    TIM2->CR1 = 0;
  }
  ReportStepEnabled();
}

static void UpdatePositionCounters(void) {
  // Step
  uint16_t step_cnt = UINT16_MAX - (uint16_t) DMA1_Channel5->CNDTR;
  uint16_t step_dcnt = step_cnt - position.step_cnt_prev;
  position.step_cnt_prev = step_cnt;
  position.step_dcnt = step_dcnt;
  if (position.step_reverse) {
    position.step_pos -= step_dcnt;
  } else {
    position.step_pos += step_dcnt;
  }

  // Encoder
  uint16_t encoder_cnt = (uint16_t) TIM1->CNT;
  int16_t encoder_dcnt = encoder_cnt - position.encoder_cnt_prev;
  position.encoder_cnt_prev = encoder_cnt;
  position.encoder_pos += encoder_dcnt;
}

void UpdateStepDirection(void) {
  bool reverse = READ_BIT(DIR_GPIO_Port->IDR, DIR_Pin) == 0;
  UpdatePositionCounters();
  position.step_reverse = reverse;

  __disable_irq();
  TIM3->CR1 = 0;
  if (position.step_reverse) {
    TIM3->CCR1 = 1;
    TIM3->CCR2 = 0;
    TIM1->CR1 = TIM_CR1_CEN;
  } else {
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 1;
    TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_DIR;
  }
  // TIM3->CR1 = TIM_CR1_CEN;
  __enable_irq();

  ReportStepReverse();
}

static void UpdateVelocity(void) {
  UpdatePositionCounters();

  int32_t current = position.encoder_pos;
  int32_t intent = StepToEncoderPosition(position.step_pos);
  intent = ApplyOverride(current, intent);
  int32_t error = intent - current;

  // All rates below are per millisecond, therefore the 48 MHz system clock
  // becomes 48000 ticks per millisecond.

  float input_rate = 0;
  if (position.step_dcnt) {
    input_rate = 48000.0f / (float) step_period_cnt;
    if (position.step_reverse) {
      input_rate = -input_rate;
    }
  }

  const float K_ff = 1;
  const float K_p = 0.1;

  float output_rate = K_ff * input_rate + K_p * error;
  float abs_output_rate = fabs(output_rate);
  if (abs_output_rate < 0.001f) {
    TIM3->CR1 = 0;
    return;
  }

  uint32_t psc = (uint32_t) (48000.0f / (2.0f * abs_output_rate)) - 1;
  if (psc > 0xFFFF) psc = 0xFFFF;
  TIM3->PSC = (uint16_t) psc;
  TIM3->CR1 = TIM_CR1_CEN;
}

void UpdateTick(void) {
  ++now;
  UpdateVelocity();
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
               |  RCC_APB2ENR_TIM1EN
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

  GPIOB->AFR[0] = (1 << GPIO_AFRL_AFSEL4_Pos)  // PB4 alternate function TIM3 CH1 (EA)
                | (1 << GPIO_AFRL_AFSEL5_Pos)  // PB5 alternate function TIM3 CH2 (EB)
                ;

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
               | (2 << GPIO_MODER_MODER4_Pos)  // PB4 alternate function TIM3 CH1 (EA)
               | (2 << GPIO_MODER_MODER5_Pos)  // PB5 alternate function TIM3 CH2 (EB)
               ;

  GPIOA->PUPDR = (1 << GPIO_PUPDR_PUPDR4_Pos)  // Enable internal pull-up on PA4
               | (1 << GPIO_PUPDR_PUPDR5_Pos); // Enable internal pull-up on PA5

  SYSCFG->EXTICR[0] = 0;  // EXTI0-3 from PORTA (ENA on EXTI3)
  SYSCFG->EXTICR[1] = 0;  // EXTI4-7 from PORTA (DIR on EXTI4)

  DMA1_Channel5->CPAR = (uint32_t) &TIM2->CCR1;
  DMA1_Channel5->CMAR = (uint32_t) &step_period_cnt;
  DMA1_Channel5->CNDTR = UINT16_MAX;
  DMA1_Channel5->CCR = (0b10 << DMA_CCR_MSIZE_Pos)  // 32-bit memory
                     | (0b10 << DMA_CCR_PSIZE_Pos)  // 32-bit peripheral
                     | DMA_CCR_CIRC                 // Circular mode
                     | DMA_CCR_EN;                  // Start DMA

  // TIM1 counts the number of toggles of the EA pin, or half-cycles of the quadrature encoder.
  TIM1->PSC = 0;
  TIM1->ARR = UINT16_MAX;
  TIM1->SMCR = (0b010 << TIM_SMCR_TS_Pos)    // ITR2 (TIM3)
             | (0b111 << TIM_SMCR_SMS_Pos);  // External clock mode 1
  TIM1->CR1 = TIM_CR1_CEN;

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
  TIM2->DIER = TIM_DIER_CC1DE;                 // Link DMA to CCR1
  TIM2->CR1 = TIM_CR1_CEN;

  // TIM3 outputs a quadrature encoder signal on the EA and EB pins.
  // Additionally, it attaches TRGO (sent to TIM1) to update events on every toggle of EA.
  TIM3->PSC = 0;
  TIM3->ARR = 2 - 1;
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 1;
  TIM3->CCMR1 = (0b011 << TIM_CCMR1_OC1M_Pos)   // CH1 toggle
              | (0b011 << TIM_CCMR1_OC2M_Pos);  // CH2 toggle
  TIM3->CCER = TIM_CCER_CC1E |                  // Enable CH1 output
               TIM_CCER_CC2E;                   // Enable CH2 output
  TIM3->DIER = 0;                               // Disable interrupts
  TIM3->CR2 = TIM_CR2_MMS_1;                    // Update event is trigger output (TRGO to TIM1)
  TIM3->CR1 = 0;

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

  NVIC_SetPriority(EXTI2_3_IRQn, 2);
  NVIC_SetPriority(EXTI4_15_IRQn, 2);
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
