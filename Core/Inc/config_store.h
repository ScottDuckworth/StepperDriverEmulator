#ifndef __CONFIG_STORE_H
#define __CONFIG_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "emulator_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_FLASH_PAGE_ADDR  0x08007C00U  /* Page 31 start address */
#define CONFIG_FLASH_PAGE_SIZE  1024U        /* 1 KB per page */
#define CONFIG_STORE_MAGIC      0x53544550U  /* "STEP" ASCII in little-endian */
#define CONFIG_STORE_VERSION    1U

#pragma pack(push, 1)
typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t length;
  uint32_t crc32;
  EmulatorConfig_t config;
} ConfigRecord_t;
#pragma pack(pop)

typedef struct {
  bool (*read)(uint32_t address, void* buffer, size_t length);
  bool (*erase_page)(uint32_t page_address);
  bool (*write)(uint32_t address, const void* buffer, size_t length);
} FlashDriver_t;

uint32_t ConfigStore_CalcCRC32(const void* data, size_t length);
bool ConfigStore_Validate(const EmulatorConfig_t* cfg);
bool ConfigStore_Load(const FlashDriver_t* flash, uint32_t page_addr, EmulatorConfig_t* out_cfg);
bool ConfigStore_Save(const FlashDriver_t* flash, uint32_t page_addr, const EmulatorConfig_t* in_cfg);

#ifndef UNIT_TEST
const FlashDriver_t* ConfigStore_GetStm32FlashDriver(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_STORE_H */
