#include "unity.h"
#include "config_store.h"
#include <string.h>

#define MOCK_PAGE_ADDR 0x08007C00U
#define MOCK_PAGE_SIZE 1024U

static uint8_t mock_flash_mem[MOCK_PAGE_SIZE];
static bool mock_erase_fail = false;
static bool mock_write_fail = false;

static bool MockFlash_Read(uint32_t address, void* buffer, size_t length) {
  if (address < MOCK_PAGE_ADDR || (address + length) > (MOCK_PAGE_ADDR + MOCK_PAGE_SIZE)) {
    return false;
  }
  uint32_t offset = address - MOCK_PAGE_ADDR;
  memcpy(buffer, &mock_flash_mem[offset], length);
  return true;
}

static bool MockFlash_ErasePage(uint32_t page_address) {
  if (page_address != MOCK_PAGE_ADDR) return false;
  if (mock_erase_fail) return false;
  memset(mock_flash_mem, 0xFF, MOCK_PAGE_SIZE);
  return true;
}

static bool MockFlash_Write(uint32_t address, const void* buffer, size_t length) {
  if (address < MOCK_PAGE_ADDR || (address + length) > (MOCK_PAGE_ADDR + MOCK_PAGE_SIZE)) {
    return false;
  }
  if (mock_write_fail) return false;
  uint32_t offset = address - MOCK_PAGE_ADDR;

  // Flash write simulation: bits can only transition 1 -> 0 without erase
  const uint8_t* src = (const uint8_t*) buffer;
  for (size_t i = 0; i < length; i++) {
    mock_flash_mem[offset + i] &= src[i];
  }
  return true;
}

static const FlashDriver_t mock_flash_driver = {
    .read = MockFlash_Read,
    .erase_page = MockFlash_ErasePage,
    .write = MockFlash_Write
};

void setUp(void) {
  memset(mock_flash_mem, 0xFF, sizeof(mock_flash_mem));
  mock_erase_fail = false;
  mock_write_fail = false;
}

void tearDown(void) {}

void test_crc32_standard_check(void) {
  // Known IEEE 802.3 CRC32 check: "123456789" -> 0xCBF43926
  const char* check_str = "123456789";
  uint32_t crc = ConfigStore_CalcCRC32(check_str, 9);
  TEST_ASSERT_EQUAL_HEX32(0xCBF43926U, crc);
}

void test_fresh_erased_flash_fails_to_load(void) {
  EmulatorConfig_t loaded_cfg;
  // All 0xFF in flash
  bool ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_save_and_load_roundtrip(void) {
  EmulatorConfig_t original_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  original_cfg.spr = 800;
  original_cfg.epr = 3200;
  original_cfg.odr = 250;
  original_cfg.torque_t0 = 1200;
  original_cfg.torque_v_knee = 1500;
  original_cfg.torque_v_max = 9000;
  original_cfg.torque_t_min = 350;
  original_cfg.stall_threshold = 2000;
  original_cfg.kp = 0.25f;
  original_cfg.kff = 0.95f;
  original_cfg.kfree = 0.008f;

  bool save_ok = ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &original_cfg);
  TEST_ASSERT_TRUE(save_ok);

  EmulatorConfig_t loaded_cfg = {0};
  bool load_ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_cfg);
  TEST_ASSERT_TRUE(load_ok);

  // Assert exact match of all fields
  TEST_ASSERT_EQUAL_UINT16(800, loaded_cfg.spr);
  TEST_ASSERT_EQUAL_UINT16(3200, loaded_cfg.epr);
  TEST_ASSERT_EQUAL_UINT16(250, loaded_cfg.odr);
  TEST_ASSERT_EQUAL_INT32(1200, loaded_cfg.torque_t0);
  TEST_ASSERT_EQUAL_UINT32(1500, loaded_cfg.torque_v_knee);
  TEST_ASSERT_EQUAL_UINT32(9000, loaded_cfg.torque_v_max);
  TEST_ASSERT_EQUAL_INT32(350, loaded_cfg.torque_t_min);
  TEST_ASSERT_EQUAL_UINT32(2000, loaded_cfg.stall_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.25f, loaded_cfg.kp);
  TEST_ASSERT_EQUAL_FLOAT(0.95f, loaded_cfg.kff);
  TEST_ASSERT_EQUAL_FLOAT(0.008f, loaded_cfg.kfree);
}

void test_crc_bitflip_detected(void) {
  EmulatorConfig_t original_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &original_cfg);

  // Flip a bit in the config data section (offset 20 bytes into record)
  mock_flash_mem[20] ^= 0x01;

  EmulatorConfig_t loaded_cfg;
  bool ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_bad_magic_detected(void) {
  EmulatorConfig_t original_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &original_cfg);

  // Corrupt magic header (first 4 bytes)
  mock_flash_mem[0] = 'X';

  EmulatorConfig_t loaded_cfg;
  bool ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_bad_version_detected(void) {
  EmulatorConfig_t original_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &original_cfg);

  // Version is at offset 4 (uint16_t)
  mock_flash_mem[4] = 99;

  EmulatorConfig_t loaded_cfg;
  bool ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_validation_rejects_invalid_config(void) {
  EmulatorConfig_t bad_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;

  bad_cfg.spr = 0;
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));
  TEST_ASSERT_FALSE(ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &bad_cfg));

  bad_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  bad_cfg.epr = 0;
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));

  bad_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  bad_cfg.kp = -0.5f;
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));

  bad_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  bad_cfg.kfree = -0.01f;
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));

  bad_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  bad_cfg.torque_v_knee = 5000;
  bad_cfg.torque_v_max = 2000; // v_max < v_knee
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));
}

void test_flash_erase_failure_handled(void) {
  EmulatorConfig_t cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  mock_erase_fail = true;

  bool ok = ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_flash_write_failure_handled(void) {
  EmulatorConfig_t cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  mock_write_fail = true;

  bool ok = ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_null_safety(void) {
  EmulatorConfig_t cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;

  TEST_ASSERT_FALSE(ConfigStore_Load(NULL, MOCK_PAGE_ADDR, &cfg));
  TEST_ASSERT_FALSE(ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, NULL));
  TEST_ASSERT_FALSE(ConfigStore_Save(NULL, MOCK_PAGE_ADDR, &cfg));
  TEST_ASSERT_FALSE(ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, NULL));
  TEST_ASSERT_FALSE(ConfigStore_Validate(NULL));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_crc32_standard_check);
  RUN_TEST(test_fresh_erased_flash_fails_to_load);
  RUN_TEST(test_save_and_load_roundtrip);
  RUN_TEST(test_crc_bitflip_detected);
  RUN_TEST(test_bad_magic_detected);
  RUN_TEST(test_bad_version_detected);
  RUN_TEST(test_validation_rejects_invalid_config);
  RUN_TEST(test_flash_erase_failure_handled);
  RUN_TEST(test_flash_write_failure_handled);
  RUN_TEST(test_null_safety);
  return UNITY_END();
}
