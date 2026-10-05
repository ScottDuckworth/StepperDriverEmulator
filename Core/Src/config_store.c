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

bool ConfigStore_Validate(const PersistentConfig_t* cfg) {
  if (!cfg) return false;
  if (cfg->ratio_spr == 0 || cfg->ratio_epr == 0) return false;
  if (cfg->kp.raw < 0 || cfg->kff.raw < 0 || cfg->kfree.raw < 0) return false;
  if (cfg->torque_v_max < cfg->torque_v_knee) return false;
  if (memchr(cfg->name, '\0', sizeof(cfg->name)) == NULL) return false;
  return true;
}

void ConfigStore_ComputeCachedValues(const PersistentConfig_t* persistent, CachedConfig_t* cached) {
  if (!persistent || !cached) return;

  cached->kp_velocity = MathUtil_FromRawQ12(persistent->kp.raw * 1000);
  cached->counts_per_step = MathUtil_RatioQ12(persistent->ratio_epr, persistent->ratio_spr);
  cached->inv_counts_per_step = MathUtil_RatioQ16(persistent->ratio_spr, persistent->ratio_epr);

  cached->step_counts_int = MathUtil_Q12ToInt(cached->counts_per_step);
  cached->ff_window = MathUtil_Q12ToInt(MathUtil_MulQ12_Q12(persistent->kff, cached->counts_per_step));
  cached->max_kp_step_v = MathUtil_MulQ12(cached->step_counts_int, cached->kp_velocity);

  // Precompute clock_counts_sec for direct step_period_cnt -> input_rate conversion.
  // Fits in uint32_t without overflow if (ratio_epr / ratio_spr) <= 89 counts/step.
  if (persistent->ratio_spr > 0 && ((uint32_t) persistent->ratio_epr / (uint32_t) persistent->ratio_spr) <= 89U) {
    uint32_t q = (uint32_t) persistent->ratio_epr / (uint32_t) persistent->ratio_spr;
    uint32_t r = (uint32_t) persistent->ratio_epr % (uint32_t) persistent->ratio_spr;
    uint32_t rem_high = (48000U * r) / (uint32_t) persistent->ratio_spr;
    uint32_t rem_low = (48000U * r) % (uint32_t) persistent->ratio_spr;
    uint32_t r_part = rem_high * 1000U + (rem_low * 1000U) / (uint32_t) persistent->ratio_spr;
    cached->clock_counts_sec = 48000000U * q + r_part;
  } else {
    cached->clock_counts_sec = 0; // Fallback to dynamic MathUtil_MulQ12 in Motion_PlanStep
  }

  uint32_t span_v = (persistent->torque_v_max > persistent->torque_v_knee) ? (persistent->torque_v_max - persistent->torque_v_knee) : 0;
  cached->inv_torque_span_v = (span_v > 0) ? MathUtil_FromRawQ16((int32_t)(65536U / span_v)) : MathUtil_FromRawQ16(0);

  int32_t delta_t = persistent->torque_t0 - persistent->torque_t_min;
  if (span_v > 0 && delta_t > 0) {
    uint32_t dt = (uint32_t) delta_t;
    uint32_t slope;
    if (dt <= 65535U) {
      slope = (dt * 65536U + (span_v / 2)) / span_v;
    } else {
      uint32_t int_part = dt / span_v;
      uint32_t rem = dt % span_v;
      slope = (int_part << 16) + (rem * 65536U + (span_v / 2)) / span_v;
    }
    cached->torque_derate_slope = MathUtil_FromRawQ16((int32_t) slope);
  } else {
    cached->torque_derate_slope = MathUtil_FromRawQ16(0);
  }
}

bool ConfigStore_LoadPersistent(const FlashDriver_t* flash, uint32_t page_addr, PersistentConfig_t* out_cfg) {
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
  if (record.length != sizeof(PersistentConfig_t)) {
    return false;
  }

  uint32_t computed_crc = ConfigStore_CalcCRC32(&record.config, sizeof(PersistentConfig_t));
  if (computed_crc != record.crc32) {
    return false;
  }

  if (!ConfigStore_Validate(&record.config)) {
    return false;
  }

  *out_cfg = record.config;
  return true;
}

bool ConfigStore_Load(const FlashDriver_t* flash, uint32_t page_addr, EmulatorConfig_t* out_cfg) {
  if (!out_cfg) return false;
  if (!ConfigStore_LoadPersistent(flash, page_addr, &out_cfg->persistent)) {
    return false;
  }
  ConfigStore_ComputeCachedValues(&out_cfg->persistent, &out_cfg->cached);
  return true;
}

bool ConfigStore_Save(const FlashDriver_t* flash, uint32_t page_addr, const PersistentConfig_t* in_cfg) {
  if (!flash || !flash->write || !flash->erase_page || !in_cfg) return false;
  if (!ConfigStore_Validate(in_cfg)) return false;

  ConfigRecord_t record;
  record.magic = CONFIG_STORE_MAGIC;
  record.version = CONFIG_STORE_VERSION;
  record.length = (uint16_t) sizeof(PersistentConfig_t);
  record.config = *in_cfg;
  record.crc32 = ConfigStore_CalcCRC32(&record.config, sizeof(PersistentConfig_t));

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
