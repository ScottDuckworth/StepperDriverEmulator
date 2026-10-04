/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UNIT_TEST
#include "stm32f0xx_hal.h"
#else
typedef void TIM_HandleTypeDef;
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include "emulator_config.h"
#include "position.h"
#include "motion.h"
#include "quadrature.h"
#include "led_control.h"
#include "mathutil.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define CHUNK_SIZE 16
#define TOTAL_BUFFER_SIZE (2 * CHUNK_SIZE)

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM1_Init(void);
void MX_TIM16_Init(void);
void MX_TIM17_Init(void);

void InitSystemClock(void);
void UpdateStepDirection(void);
void UpdateStepEnabled(void);
void UpdateTick(void);

void Motion_Init(void);
bool MaybeStartMotion(void);
void Motion_Wakeup_Handler(void);
void DMA_HalfTransfer_Handler(void);
void DMA_TransferComplete_Handler(void);

q12_t GetKp(void);
void SetKp(q12_t kp);
q12_t GetKff(void);
void SetKff(q12_t kff);
void ReportKp(void);
void ReportKff(void);
void ReportQ12(const char* var, q12_t value);
void ReportQ16(const char* var, q16_t value);

EmulatorConfig_t* GetConfig(void);

uint16_t GetOdr(void);
void SetOdr(uint16_t odr);
void ReportOdr(void);

bool SetRatio(uint16_t spr, uint16_t epr);
void GetRatio(uint16_t* out_spr, uint16_t* out_epr);
void ReportRatio(void);

int64_t GetEncoderPosition(void);
void SetEncoderPosition(int64_t pos);

bool GetStepReverse(void);
bool GetStepEnabled(void);

bool GetLimit1(void);
void SetLimit1(bool active);

bool GetLimit2(void);
void SetLimit2(bool active);

void GetLED(uint8_t* r, uint8_t* g);
void SetLED(uint8_t r, uint8_t g);

bool GetBlinkMode(void);
void SetBlinkMode(bool enable);
void ReportBlinkMode(void);

int32_t GetTension(void);
void SetTension(int32_t tension);
void ReportTension(void);

void GetTorqueCurve(int32_t* t0, uint32_t* v_knee, uint32_t* v_max, int32_t* t_min);
void SetTorqueCurve(int32_t t0, uint32_t v_knee, uint32_t v_max, int32_t t_min);
void ReportTorqueCurve(void);

uint32_t GetStallThreshold(void);
void SetStallThreshold(uint32_t threshold);
void ReportStallThreshold(void);

q12_t GetKfree(void);
void SetKfree(q12_t kfree);
void ReportKfree(void);

q12_t GetStepBlanking(void);
void SetStepBlanking(q12_t blank_us);
void ReportStepBlanking(void);


bool GetStallTrip(void);
void ReportStallTrip(void);

bool SaveConfig(void);

uint16_t WriteData(const uint8_t* data, uint16_t size);
uint16_t WriteString(const char* text);
void ReportString(const char* var, const char* value);
void ReportI64(const char* var, int64_t value);
void ReportI32(const char* var, int32_t value);
void ReportU32(const char* var, uint32_t value);
void ReportU16(const char* var, uint16_t value);
void ReportU8(const char* var, uint8_t value);
void ReportEncoderPosition(void);
void ReportStepPosition(void);
void ReportStepReverse(void);
void ReportStepEnabled(void);
void ReportLimit1(void);
void ReportLimit2(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENA_Pin GPIO_PIN_3
#define ENA_GPIO_Port GPIOA
#define ENA_EXTI_IRQn EXTI2_3_IRQn
#define DIR_Pin GPIO_PIN_4
#define DIR_GPIO_Port GPIOA
#define DIR_EXTI_IRQn EXTI4_15_IRQn
#define PUL_Pin GPIO_PIN_5
#define PUL_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_6
#define LED_G_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOA
#define LIM1_Pin GPIO_PIN_0
#define LIM1_GPIO_Port GPIOB
#define LIM2_Pin GPIO_PIN_1
#define LIM2_GPIO_Port GPIOB
#define EA_Pin GPIO_PIN_4
#define EA_GPIO_Port GPIOB
#define EB_Pin GPIO_PIN_5
#define EB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
