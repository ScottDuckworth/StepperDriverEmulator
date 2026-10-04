#include "config_store.h"
#include "mathutil.h"
#include <string.h>

uint32_t ConfigStore_CalcCRC32(const void* data, size_t length) {
  const uint8_t* p = (const uint8_t*) data;
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t i = 0; i < length; i++) {
    crc ^= p[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320U & (-(int32_t)(crc & 1)));
    }
  }
  return ~crc;
}

bool ConfigStore_Validate(const EmulatorConfig_t* cfg) {
  if (!cfg) return false;
  if (cfg->ratio_spr == 0 || cfg->ratio_epr == 0) return false;
  if (cfg->kp < 0.0f || cfg->kff < 0.0f || cfg->kfree < 0.0f) return false;
  if (cfg->torque_v_max < cfg->torque_v_knee) return false;
  return true;
}

void ConfigStore_RefreshCachedValues(EmulatorConfig_t* cfg) {
  if (!cfg) return;

  // Float representations for CLI reporting
  cfg->counts_per_step = MathUtil_CalcRatioFloat(cfg->ratio_epr, cfg->ratio_spr);
  cfg->inv_counts_per_step = MathUtil_CalcRatioFloat(cfg->ratio_spr, cfg->ratio_epr);

  // Cached fixed-point fields for zero-float ISR execution
  cfg->kp_q12 = MathUtil_FloatToQ12(cfg->kp);
  cfg->kp_velocity_q12 = MathUtil_FloatToQ12(cfg->kp * 1000.0f);
  cfg->kff_q12 = MathUtil_FloatToQ12(cfg->kff);
  cfg->kfree_q12 = MathUtil_FloatToQ12(cfg->kfree);

  cfg->counts_per_step_q12 = MathUtil_CalcRatioQ12(cfg->ratio_epr, cfg->ratio_spr);
  cfg->inv_counts_per_step_q16 = MathUtil_CalcRatioQ16(cfg->ratio_spr, cfg->ratio_epr);

  uint32_t span_v = (cfg->torque_v_max > cfg->torque_v_knee) ? (cfg->torque_v_max - cfg->torque_v_knee) : 0;
  cfg->inv_torque_span_v_q16 = (span_v > 0) ? MathUtil_FromRawQ16((int32_t)(65536U / span_v)) : MathUtil_FromRawQ16(0);
}

bool ConfigStore_Load(const FlashDriver_t* flash, uint32_t page_addr, EmulatorConfig_t* out_cfg) {
  if (!flash || !flash->read || !out_cfg) return false;

  ConfigRecord_t record;
  if (!flash->read(page_addr, &record, sizeof(ConfigRecord_t))) {
    return false;
  }

  if (record.magic != CONFIG_STORE_MAGIC) {
    return false;
  }
  if (record.version != CONFIG_STORE_VERSION) {
    return false;
  }
  if (record.length != sizeof(EmulatorConfig_t)) {
    return false;
  }

  uint32_t computed_crc = ConfigStore_CalcCRC32(&record.config, sizeof(EmulatorConfig_t));
  if (computed_crc != record.crc32) {
    return false;
  }

  if (!ConfigStore_Validate(&record.config)) {
    return false;
  }

  *out_cfg = record.config;
  ConfigStore_RefreshCachedValues(out_cfg);
  return true;
}

bool ConfigStore_Save(const FlashDriver_t* flash, uint32_t page_addr, const EmulatorConfig_t* in_cfg) {
  if (!flash || !flash->write || !flash->erase_page || !in_cfg) return false;
  if (!ConfigStore_Validate(in_cfg)) return false;

  ConfigStore_RefreshCachedValues((EmulatorConfig_t*) in_cfg);

  ConfigRecord_t record;
  record.magic = CONFIG_STORE_MAGIC;
  record.version = CONFIG_STORE_VERSION;
  record.length = (uint16_t) sizeof(EmulatorConfig_t);
  record.config = *in_cfg;
  record.crc32 = ConfigStore_CalcCRC32(&record.config, sizeof(EmulatorConfig_t));

  if (!flash->erase_page(page_addr)) {
    return false;
  }

  if (!flash->write(page_addr, &record, sizeof(ConfigRecord_t))) {
    return false;
  }

  // Read back and verify integrity
  if (flash->read) {
    ConfigRecord_t verify;
    if (!flash->read(page_addr, &verify, sizeof(ConfigRecord_t))) {
      return false;
    }
    if (verify.magic != record.magic ||
        verify.version != record.version ||
        verify.length != record.length ||
        verify.crc32 != record.crc32) {
      return false;
    }
  }

  return true;
}

#ifndef UNIT_TEST
#include "stm32f0xx_hal.h"

static bool Stm32Flash_Read(uint32_t address, void* buffer, size_t length) {
  memcpy(buffer, (const void*) (uintptr_t) address, length);
  return true;
}

static bool Stm32Flash_ErasePage(uint32_t page_address) {
  if (HAL_FLASH_Unlock() != HAL_OK) return false;

  FLASH_EraseInitTypeDef erase_init;
  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = page_address;
  erase_init.NbPages = 1;

  uint32_t page_error = 0;
  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
  HAL_FLASH_Lock();

  return (status == HAL_OK && page_error == 0xFFFFFFFFU);
}

static bool Stm32Flash_Write(uint32_t address, const void* buffer, size_t length) {
  if (HAL_FLASH_Unlock() != HAL_OK) return false;

  size_t words = (length + 3) / 4;
  const uint32_t* src = (const uint32_t*) buffer;
  uint32_t dst = address;
  HAL_StatusTypeDef status = HAL_OK;

  for (size_t i = 0; i < words; i++) {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dst, src[i]);
    if (status != HAL_OK) {
      break;
    }
    dst += 4;
  }

  HAL_FLASH_Lock();
  return (status == HAL_OK);
}

static const FlashDriver_t s_stm32_flash_driver = {
    .read = Stm32Flash_Read,
    .erase_page = Stm32Flash_ErasePage,
    .write = Stm32Flash_Write
};

const FlashDriver_t* ConfigStore_GetStm32FlashDriver(void) {
  return &s_stm32_flash_driver;
}
#endif
