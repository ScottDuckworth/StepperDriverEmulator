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
  PersistentConfig_t original_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  original_cfg.ratio_spr = 1;
  original_cfg.ratio_epr = 4;
  original_cfg.odr = 250;
  original_cfg.torque_t0 = 1200;
  original_cfg.torque_v_knee = 1500;
  original_cfg.torque_v_max = 9000;
  original_cfg.torque_t_min = 350;
  original_cfg.stall_threshold = 2000;
  original_cfg.kp = (q12_t){ .raw = 1024 };
  original_cfg.kff = (q12_t){ .raw = 3891 };
  original_cfg.kfree = (q12_t){ .raw = 33 };
  strcpy(original_cfg.name, "CustomAxis");

  bool save_ok = ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &original_cfg);
  TEST_ASSERT_TRUE(save_ok);

  PersistentConfig_t loaded_p_cfg = {0};
  bool load_p_ok = ConfigStore_LoadPersistent(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_p_cfg);
  TEST_ASSERT_TRUE(load_p_ok);

  // Assert exact match of all persistent fields
  TEST_ASSERT_EQUAL_STRING("CustomAxis", loaded_p_cfg.name);
  TEST_ASSERT_EQUAL_UINT16(1, loaded_p_cfg.ratio_spr);
  TEST_ASSERT_EQUAL_UINT16(4, loaded_p_cfg.ratio_epr);
  TEST_ASSERT_EQUAL_UINT16(250, loaded_p_cfg.odr);
  TEST_ASSERT_EQUAL_INT32(1200, loaded_p_cfg.torque_t0);
  TEST_ASSERT_EQUAL_UINT32(1500, loaded_p_cfg.torque_v_knee);
  TEST_ASSERT_EQUAL_UINT32(9000, loaded_p_cfg.torque_v_max);
  TEST_ASSERT_EQUAL_INT32(350, loaded_p_cfg.torque_t_min);
  TEST_ASSERT_EQUAL_UINT32(2000, loaded_p_cfg.stall_threshold);
  TEST_ASSERT_EQUAL_INT32(1024, loaded_p_cfg.kp.raw);
  TEST_ASSERT_EQUAL_INT32(3891, loaded_p_cfg.kff.raw);
  TEST_ASSERT_EQUAL_INT32(33, loaded_p_cfg.kfree.raw);

  // Verify ConfigStore_Load loads into EmulatorConfig_t and populates cached fields
  EmulatorConfig_t loaded_emu = {0};
  bool load_emu_ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_emu);
  TEST_ASSERT_TRUE(load_emu_ok);
  TEST_ASSERT_EQUAL_INT32(16384, loaded_emu.cached.counts_per_step.raw);
  TEST_ASSERT_EQUAL_INT32(16384, loaded_emu.cached.inv_counts_per_step.raw);
}

void test_crc_bitflip_detected(void) {
  PersistentConfig_t original_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &original_cfg);

  // Flip a bit in the config data section (offset 20 bytes into record)
  mock_flash_mem[20] ^= 0x01;

  EmulatorConfig_t loaded_cfg;
  bool ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_bad_magic_detected(void) {
  PersistentConfig_t original_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &original_cfg);

  // Corrupt magic header (first 4 bytes)
  mock_flash_mem[0] = 'X';

  EmulatorConfig_t loaded_cfg;
  bool ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_bad_version_detected(void) {
  PersistentConfig_t original_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &original_cfg);

  // Version is at offset 4 (uint16_t)
  mock_flash_mem[4] = 99;

  EmulatorConfig_t loaded_cfg;
  bool ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded_cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_validation_rejects_invalid_config(void) {
  PersistentConfig_t bad_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;

  bad_cfg.ratio_spr = 0;
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));
  TEST_ASSERT_FALSE(ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &bad_cfg));

  bad_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  bad_cfg.ratio_epr = 0;
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));

  bad_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  bad_cfg.kp = (q12_t){ .raw = -500 };
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));

  bad_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  bad_cfg.kfree = (q12_t){ .raw = -10 };
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));

  bad_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  bad_cfg.torque_v_knee = 5000;
  bad_cfg.torque_v_max = 2000; // v_max < v_knee
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));

  bad_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  memset(bad_cfg.name, 'A', sizeof(bad_cfg.name)); // Missing null-terminator
  TEST_ASSERT_FALSE(ConfigStore_Validate(&bad_cfg));
}

void test_flash_erase_failure_handled(void) {
  PersistentConfig_t cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  mock_erase_fail = true;

  bool ok = ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_flash_write_failure_handled(void) {
  PersistentConfig_t cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  mock_write_fail = true;

  bool ok = ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_null_safety(void) {
  PersistentConfig_t p_cfg = (PersistentConfig_t) DEFAULT_PERSISTENT_CONFIG;
  EmulatorConfig_t e_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;

  TEST_ASSERT_FALSE(ConfigStore_Load(NULL, MOCK_PAGE_ADDR, &e_cfg));
  TEST_ASSERT_FALSE(ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, NULL));
  TEST_ASSERT_FALSE(ConfigStore_LoadPersistent(NULL, MOCK_PAGE_ADDR, &p_cfg));
  TEST_ASSERT_FALSE(ConfigStore_LoadPersistent(&mock_flash_driver, MOCK_PAGE_ADDR, NULL));
  TEST_ASSERT_FALSE(ConfigStore_Save(NULL, MOCK_PAGE_ADDR, &p_cfg));
  TEST_ASSERT_FALSE(ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, NULL));
  TEST_ASSERT_FALSE(ConfigStore_Validate(NULL));
}

void test_cached_fixed_point_refresh(void) {
  EmulatorConfig_t cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  cfg.persistent.kp = (q12_t){ .raw = 819 };
  cfg.persistent.kff = (q12_t){ .raw = 6144 };
  cfg.persistent.kfree = (q12_t){ .raw = 41 };
  cfg.persistent.ratio_spr = 200;
  cfg.persistent.ratio_epr = 1000; // 5 counts per step
  cfg.persistent.torque_v_knee = 2000;
  cfg.persistent.torque_v_max = 10000; // span 8000

  ConfigStore_ComputeCachedValues(&cfg.persistent, &cfg.cached);

  TEST_ASSERT_EQUAL_INT32(819, cfg.persistent.kp.raw);
  TEST_ASSERT_EQUAL_INT32(819000, cfg.cached.kp_velocity.raw);   // 819 * 1000 = 819000
  TEST_ASSERT_EQUAL_INT32(6144, cfg.persistent.kff.raw);
  TEST_ASSERT_EQUAL_INT32(41, cfg.persistent.kfree.raw);
  TEST_ASSERT_EQUAL_INT32(20480, cfg.cached.counts_per_step.raw); // 5.0 * 4096 = 20480
  TEST_ASSERT_EQUAL_INT32(13107, cfg.cached.inv_counts_per_step.raw); // 0.2 * 65536 = 13107.2 -> 13107
  TEST_ASSERT_EQUAL_INT32(8, cfg.cached.inv_torque_span_v.raw); // 65536 / 8000 = 8.19 -> 8
  TEST_ASSERT_EQUAL_INT32(6554, cfg.cached.torque_derate_slope.raw); // (800 * 65536 + 4000) / 8000 = 6554
  TEST_ASSERT_EQUAL_INT32(5, cfg.cached.step_counts_int);
  TEST_ASSERT_EQUAL_INT32(8, cfg.cached.ff_window);
  TEST_ASSERT_EQUAL_INT32(999, cfg.cached.max_kp_step_v);
  TEST_ASSERT_EQUAL_UINT32(240000000U, cfg.cached.clock_counts_sec);

  // Verify round-trip load refreshes cached values
  ConfigStore_Save(&mock_flash_driver, MOCK_PAGE_ADDR, &cfg.persistent);
  EmulatorConfig_t loaded = {0};
  bool ok = ConfigStore_Load(&mock_flash_driver, MOCK_PAGE_ADDR, &loaded);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT32(819, loaded.persistent.kp.raw);
  TEST_ASSERT_EQUAL_INT32(819000, loaded.cached.kp_velocity.raw);
  TEST_ASSERT_EQUAL_INT32(6144, loaded.persistent.kff.raw);
  TEST_ASSERT_EQUAL_INT32(41, loaded.persistent.kfree.raw);
  TEST_ASSERT_EQUAL_INT32(20480, loaded.cached.counts_per_step.raw);
  TEST_ASSERT_EQUAL_INT32(13107, loaded.cached.inv_counts_per_step.raw);
  TEST_ASSERT_EQUAL_INT32(8, loaded.cached.inv_torque_span_v.raw);
  TEST_ASSERT_EQUAL_INT32(6554, loaded.cached.torque_derate_slope.raw);
  TEST_ASSERT_EQUAL_INT32(5, loaded.cached.step_counts_int);
  TEST_ASSERT_EQUAL_INT32(8, loaded.cached.ff_window);
  TEST_ASSERT_EQUAL_INT32(999, loaded.cached.max_kp_step_v);
  TEST_ASSERT_EQUAL_UINT32(240000000U, loaded.cached.clock_counts_sec);

  // High ratio > 89 counts/step fallback: clock_counts_sec set to 0 to avoid 32-bit overflow
  cfg.persistent.ratio_spr = 1;
  cfg.persistent.ratio_epr = 100; // 100 counts/step > 89
  ConfigStore_ComputeCachedValues(&cfg.persistent, &cfg.cached);
  TEST_ASSERT_EQUAL_INT32(100, cfg.cached.step_counts_int);
  TEST_ASSERT_EQUAL_UINT32(0, cfg.cached.clock_counts_sec);
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
  RUN_TEST(test_cached_fixed_point_refresh);
  return UNITY_END();
}
