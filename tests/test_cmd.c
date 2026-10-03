#include "unity.h"
#include "cmd.h"
#include "main.h"
#include "motion_math.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

// --- Mock Hardware / State Environment ---
static EmulatorConfig_t mock_config;
static int32_t mock_tension;
static int64_t mock_pos;
static bool mock_lim1;
static bool mock_lim2;
static bool mock_blink;
static bool mock_stall_trip;
static bool mock_save_success;

static char captured_output[4096];
static size_t captured_len;

// --- Mock Implementations of main.h / Hardware Functions ---

uint16_t WriteData(const uint8_t* data, uint16_t size) {
  if (captured_len + size < sizeof(captured_output) - 1) {
    memcpy(captured_output + captured_len, data, size);
    captured_len += size;
    captured_output[captured_len] = '\0';
  }
  return size;
}

uint16_t WriteString(const char* text) {
  return WriteData((const uint8_t*) text, (uint16_t) strlen(text));
}

void ReportString(const char* var, const char* value) {
  WriteString(var);
  WriteString(" ");
  WriteString(value);
  WriteString("\r\n");
}

void ReportI64(const char* var, int64_t value) {
  char buf[48];
  char* p = buf;
  WriteString(var);
  *p++ = ' ';
  uint64_t mag;
  if (value < 0) {
    *p++ = '-';
    mag = (uint64_t)(-(value + 1)) + 1ULL;
  } else {
    mag = (uint64_t) value;
  }
  char digits[24];
  int d_cnt = 0;
  if (mag == 0) {
    digits[d_cnt++] = '0';
  } else {
    uint16_t w[4];
    w[0] = (uint16_t)(mag & 0xFFFFULL);
    w[1] = (uint16_t)((mag >> 16) & 0xFFFFULL);
    w[2] = (uint16_t)((mag >> 32) & 0xFFFFULL);
    w[3] = (uint16_t)((mag >> 48) & 0xFFFFULL);
    while (w[0] | w[1] | w[2] | w[3]) {
      uint32_t rem = 0;
      for (int i = 3; i >= 0; i--) {
        uint32_t cur = (rem << 16) | w[i];
        w[i] = (uint16_t)(cur / 10);
        rem = cur % 10;
      }
      digits[d_cnt++] = (char)('0' + rem);
    }
  }
  while (d_cnt > 0) {
    *p++ = digits[--d_cnt];
  }
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  WriteString(buf);
}

void ReportI32(const char* var, int32_t value) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s %" PRId32 "\r\n", var, value);
  WriteString(buf);
}

void ReportU32(const char* var, uint32_t value) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s %" PRIu32 "\r\n", var, value);
  WriteString(buf);
}

void ReportU16(const char* var, uint16_t value) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s %u\r\n", var, value);
  WriteString(buf);
}

void ReportU8(const char* var, uint8_t value) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s %u\r\n", var, value);
  WriteString(buf);
}

void ReportFloat(const char* var, float value) {
  char buf[32];
  if (value < 0.0f) {
    float abs_val = -value;
    int32_t int_part = (int32_t) abs_val;
    int32_t frac_part = (int32_t) ((abs_val - (float) int_part) * 10000.0f + 0.5f);
    snprintf(buf, sizeof(buf), "%s -%" PRId32 ".%04" PRId32 "\r\n", var, int_part, frac_part);
  } else {
    int32_t int_part = (int32_t) value;
    int32_t frac_part = (int32_t) ((value - (float) int_part) * 10000.0f + 0.5f);
    snprintf(buf, sizeof(buf), "%s %" PRId32 ".%04" PRId32 "\r\n", var, int_part, frac_part);
  }
  WriteString(buf);
}

int32_t GetTension(void) { return mock_tension; }
void SetTension(int32_t tension) { mock_tension = tension; ReportTension(); }
void ReportTension(void) { ReportI32("t", GetTension()); }

void GetTorqueCurve(int32_t* t0, uint32_t* v_knee, uint32_t* v_max, int32_t* t_min) {
  if (t0) *t0 = mock_config.torque_t0;
  if (v_knee) *v_knee = mock_config.torque_v_knee;
  if (v_max) *v_max = mock_config.torque_v_max;
  if (t_min) *t_min = mock_config.torque_t_min;
}
void SetTorqueCurve(int32_t t0, uint32_t v_knee, uint32_t v_max, int32_t t_min) {
  if (v_max <= v_knee) v_max = v_knee + 1;
  mock_config.torque_t0 = t0;
  mock_config.torque_v_knee = v_knee;
  mock_config.torque_v_max = v_max;
  mock_config.torque_t_min = t_min;
  ReportTorqueCurve();
}
void ReportTorqueCurve(void) {
  char buf[64];
  snprintf(buf, sizeof(buf), "tcurve %" PRId32 " %" PRIu32 " %" PRIu32 " %" PRId32 "\r\n",
           mock_config.torque_t0, mock_config.torque_v_knee,
           mock_config.torque_v_max, mock_config.torque_t_min);
  WriteString(buf);
}

uint32_t GetStallThreshold(void) { return mock_config.stall_threshold; }
void SetStallThreshold(uint32_t threshold) { mock_config.stall_threshold = threshold; ReportStallThreshold(); }
void ReportStallThreshold(void) { ReportU32("stall", GetStallThreshold()); }

float GetKfree(void) { return mock_config.kfree; }
void SetKfree(float kfree) { mock_config.kfree = kfree; ReportKfree(); }
void ReportKfree(void) { ReportFloat("kfree", GetKfree()); }

bool GetStallTrip(void) { return mock_stall_trip; }
void ReportStallTrip(void) { ReportU8("stall_trip", GetStallTrip()); }

bool GetBlinkMode(void) { return mock_blink; }
void SetBlinkMode(bool enable) { mock_blink = enable; ReportBlinkMode(); }
void ReportBlinkMode(void) { ReportU8("blink", GetBlinkMode()); }

uint16_t GetOdr(void) { return mock_config.odr; }
void SetOdr(uint16_t odr) { mock_config.odr = odr; ReportOdr(); }
void ReportOdr(void) { ReportU16("odr", GetOdr()); }

bool SetRatio(uint16_t spr, uint16_t epr) {
  if (spr == 0 || epr == 0) return false;
  uint16_t g = CalcGCD(spr, epr);
  mock_config.ratio_spr = spr / g;
  mock_config.ratio_epr = epr / g;
  mock_config.counts_per_step = (float) mock_config.ratio_epr / (float) mock_config.ratio_spr;
  ReportRatio();
  return true;
}

void GetRatio(uint16_t* out_spr, uint16_t* out_epr) {
  if (out_spr) *out_spr = mock_config.ratio_spr;
  if (out_epr) *out_epr = mock_config.ratio_epr;
}

void ReportRatio(void) {
  char buf[32];
  int size = snprintf(buf, sizeof(buf), "ratio %u %u\r\n", mock_config.ratio_spr, mock_config.ratio_epr);
  if (size > 0) WriteData((const uint8_t*) buf, (uint16_t) size);
}

float GetKp(void) { return mock_config.kp; }
void SetKp(float kp) { mock_config.kp = kp; ReportKp(); }
void ReportKp(void) { ReportFloat("kp", GetKp()); }

float GetKff(void) { return mock_config.kff; }
void SetKff(float kff) { mock_config.kff = kff; ReportKff(); }
void ReportKff(void) { ReportFloat("kff", GetKff()); }

void SetLimit1(bool active) { mock_lim1 = active; ReportLimit1(); }
void SetLimit2(bool active) { mock_lim2 = active; ReportLimit2(); }
bool GetLimit1(void) { return mock_lim1; }
bool GetLimit2(void) { return mock_lim2; }
void ReportLimit1(void) { ReportU8("lim1", GetLimit1()); }
void ReportLimit2(void) { ReportU8("lim2", GetLimit2()); }

bool GetStepReverse(void) { return false; }
void ReportStepReverse(void) { ReportU8("rev", GetStepReverse()); }

bool GetStepEnabled(void) { return true; }
void ReportStepEnabled(void) { ReportU8("ena", GetStepEnabled()); }

int64_t GetEncoderPosition(void) { return mock_pos; }
void SetEncoderPosition(int64_t pos) { mock_pos = pos; ReportEncoderPosition(); }
void ReportEncoderPosition(void) { ReportI64("pos", GetEncoderPosition()); }

static float mock_blank = 3.5f;

float GetStepBlanking(void) { return mock_blank; }
void SetStepBlanking(float blank_us) { mock_blank = blank_us; ReportStepBlanking(); }
void ReportStepBlanking(void) { ReportFloat("blank", GetStepBlanking()); }

bool SaveConfig(void) {
  return mock_save_success;
}

// --- Test Helper Functions ---

static void clear_output(void) {
  captured_len = 0;
  captured_output[0] = '\0';
}

static void send_cmd(const char* cmd) {
  clear_output();
  CmdProcessInput(cmd, (uint16_t) strlen(cmd));
}

void setUp(void) {
  mock_config = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  mock_tension = 0;
  mock_pos = 0;
  mock_lim1 = false;
  mock_lim2 = false;
  mock_blink = false;
  mock_stall_trip = false;
  mock_save_success = true;
  mock_blank = 3.5f;
  clear_output();
}

void tearDown(void) {}

// --- Unit Tests ---

void test_cmd_blank_set_and_query(void) {
  send_cmd("blank 5.0\r\n");
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, mock_blank);
  TEST_ASSERT_EQUAL_STRING("blank 5.0000\r\n", captured_output);

  send_cmd("blank 3.5\r\n");
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.5f, mock_blank);
  TEST_ASSERT_EQUAL_STRING("blank 3.5000\r\n", captured_output);

  send_cmd("blank\r\n");
  TEST_ASSERT_EQUAL_STRING("blank 3.5000\r\n", captured_output);
}

void test_cmd_t_set_and_query(void) {
  send_cmd("t 500\r\n");
  TEST_ASSERT_EQUAL_INT32(500, mock_tension);
  TEST_ASSERT_EQUAL_STRING("t 500\r\n", captured_output);

  send_cmd("t -1200\r\n");
  TEST_ASSERT_EQUAL_INT32(-1200, mock_tension);
  TEST_ASSERT_EQUAL_STRING("t -1200\r\n", captured_output);

  send_cmd("t\r\n");
  TEST_ASSERT_EQUAL_STRING("t -1200\r\n", captured_output);
}

void test_cmd_tcurve_set_and_query(void) {
  send_cmd("tcurve 1500 2000 9000 300\r\n");
  TEST_ASSERT_EQUAL_INT32(1500, mock_config.torque_t0);
  TEST_ASSERT_EQUAL_UINT32(2000, mock_config.torque_v_knee);
  TEST_ASSERT_EQUAL_UINT32(9000, mock_config.torque_v_max);
  TEST_ASSERT_EQUAL_INT32(300, mock_config.torque_t_min);
  TEST_ASSERT_EQUAL_STRING("tcurve 1500 2000 9000 300\r\n", captured_output);

  send_cmd("tcurve\r\n");
  TEST_ASSERT_EQUAL_STRING("tcurve 1500 2000 9000 300\r\n", captured_output);
}

void test_cmd_stall_set_and_query(void) {
  send_cmd("stall 2500\r\n");
  TEST_ASSERT_EQUAL_UINT32(2500, mock_config.stall_threshold);
  TEST_ASSERT_EQUAL_STRING("stall 2500\r\n", captured_output);

  send_cmd("stall\r\n");
  TEST_ASSERT_EQUAL_STRING("stall 2500\r\n", captured_output);
}

void test_cmd_kfree_set_and_query(void) {
  send_cmd("kfree 0.0125\r\n");
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0125f, mock_config.kfree);
  TEST_ASSERT_EQUAL_STRING("kfree 0.0125\r\n", captured_output);

  send_cmd("kfree\r\n");
  TEST_ASSERT_EQUAL_STRING("kfree 0.0125\r\n", captured_output);
}

void test_cmd_blink_set_and_query(void) {
  send_cmd("blink 1\r\n");
  TEST_ASSERT_TRUE(mock_blink);
  TEST_ASSERT_EQUAL_STRING("blink 1\r\n", captured_output);

  send_cmd("blink 0\r\n");
  TEST_ASSERT_FALSE(mock_blink);
  TEST_ASSERT_EQUAL_STRING("blink 0\r\n", captured_output);

  send_cmd("blink\r\n");
  TEST_ASSERT_EQUAL_STRING("blink 0\r\n", captured_output);
}

void test_cmd_odr_set_and_query(void) {
  send_cmd("odr 250\r\n");
  TEST_ASSERT_EQUAL_UINT16(250, mock_config.odr);
  TEST_ASSERT_EQUAL_STRING("odr 250\r\n", captured_output);

  send_cmd("odr\r\n");
  TEST_ASSERT_EQUAL_STRING("odr 250\r\n", captured_output);
}

void test_cmd_ratio_set_and_query(void) {
  // Set with ratio 1000 4000 -> reduced to 1 4
  send_cmd("ratio 1000 4000\r\n");
  TEST_ASSERT_EQUAL_UINT16(1, mock_config.ratio_spr);
  TEST_ASSERT_EQUAL_UINT16(4, mock_config.ratio_epr);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, mock_config.counts_per_step);
  TEST_ASSERT_EQUAL_STRING("ratio 1 4\r\n", captured_output);

  // Set with ratio 200 1024 -> reduced to 25 128 (GCD 8)
  send_cmd("ratio 200 1024\r\n");
  TEST_ASSERT_EQUAL_UINT16(25, mock_config.ratio_spr);
  TEST_ASSERT_EQUAL_UINT16(128, mock_config.ratio_epr);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.12f, mock_config.counts_per_step);
  TEST_ASSERT_EQUAL_STRING("ratio 25 128\r\n", captured_output);

  // Query ratio
  send_cmd("ratio\r\n");
  TEST_ASSERT_EQUAL_STRING("ratio 25 128\r\n", captured_output);

  // Error handling: ratio with 1 arg -> invalid usage
  send_cmd("ratio 100\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid usage: ratio [spr] [epr]\r\n"));

  // Error handling: ratio with 0 value -> invalid uint16 > 0
  send_cmd("ratio 0 100\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid uint16 > 0: 0\r\n"));

  send_cmd("ratio 100 0\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid uint16 > 0: 0\r\n"));
}

void test_cmd_kp_kff_set_and_query(void) {
  send_cmd("kp 0.25\r\n");
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.25f, mock_config.kp);
  TEST_ASSERT_EQUAL_STRING("kp 0.2500\r\n", captured_output);

  send_cmd("kff 1.5\r\n");
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.5f, mock_config.kff);
  TEST_ASSERT_EQUAL_STRING("kff 1.5000\r\n", captured_output);

  send_cmd("kp\r\n");
  TEST_ASSERT_EQUAL_STRING("kp 0.2500\r\n", captured_output);
  send_cmd("kff\r\n");
  TEST_ASSERT_EQUAL_STRING("kff 1.5000\r\n", captured_output);
}

void test_cmd_lim1_lim2(void) {
  send_cmd("lim1 1\r\n");
  TEST_ASSERT_TRUE(mock_lim1);
  TEST_ASSERT_EQUAL_STRING("lim1 1\r\n", captured_output);

  send_cmd("lim1 0\r\n");
  TEST_ASSERT_FALSE(mock_lim1);
  TEST_ASSERT_EQUAL_STRING("lim1 0\r\n", captured_output);

  send_cmd("lim2 1\r\n");
  TEST_ASSERT_TRUE(mock_lim2);
  TEST_ASSERT_EQUAL_STRING("lim2 1\r\n", captured_output);
}

void test_cmd_zero(void) {
  mock_pos = 123456789012LL;
  send_cmd("zero\r\n");
  TEST_ASSERT_EQUAL_INT64(0, mock_pos);
  TEST_ASSERT_EQUAL_STRING("pos 0\r\n", captured_output);
}

void test_cmd_pos_report_int64(void) {
  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", 0LL);
  TEST_ASSERT_EQUAL_STRING("pos 0\r\n", captured_output);

  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", 42LL);
  TEST_ASSERT_EQUAL_STRING("pos 42\r\n", captured_output);

  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", -42LL);
  TEST_ASSERT_EQUAL_STRING("pos -42\r\n", captured_output);

  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", 2147483647LL);
  TEST_ASSERT_EQUAL_STRING("pos 2147483647\r\n", captured_output);

  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", -2147483648LL);
  TEST_ASSERT_EQUAL_STRING("pos -2147483648\r\n", captured_output);

  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", 5000000000000LL);
  TEST_ASSERT_EQUAL_STRING("pos 5000000000000\r\n", captured_output);

  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", -5000000000000LL);
  TEST_ASSERT_EQUAL_STRING("pos -5000000000000\r\n", captured_output);

  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", 9223372036854775807LL);
  TEST_ASSERT_EQUAL_STRING("pos 9223372036854775807\r\n", captured_output);

  captured_len = 0;
  captured_output[0] = '\0';
  ReportI64("pos", -9223372036854775807LL - 1LL);
  TEST_ASSERT_EQUAL_STRING("pos -9223372036854775808\r\n", captured_output);
}

void test_cmd_r_state_report(void) {
  send_cmd("r\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "odr 1000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "ratio 1 4\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "kp 0.1000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "kff 1.0000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "blank 3.5000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "tcurve 1000 1000 8000 200\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "stall 4000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "kfree 0.0050\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "stall_trip 0\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "blink 0\r\n"));
}

void test_cmd_help(void) {
  send_cmd("help\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "t [int32]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "tcurve [T0] [V_knee] [V_max] [T_min]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "stall [uint32]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "blank [float]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "zero"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "ratio [spr] [epr]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "save"));
}

void test_cmd_save_success(void) {
  mock_save_success = true;
  send_cmd("save\r\n");
  TEST_ASSERT_EQUAL_STRING("save ok\r\n", captured_output);
}

void test_cmd_save_failure(void) {
  mock_save_success = false;
  send_cmd("save\r\n");
  TEST_ASSERT_EQUAL_STRING("error: save failed\r\n", captured_output);
}

void test_cmd_save_invalid_usage(void) {
  send_cmd("save extra\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid usage: save\r\n"));
}

void test_cmd_error_invalid_usage(void) {
  send_cmd("t 1 2 3\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid usage: t [int32]\r\n"));

  send_cmd("zero 123\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid usage: zero\r\n"));
}

void test_cmd_error_invalid_value(void) {
  send_cmd("t abc\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid int32: abc\r\n"));

  send_cmd("blink 2\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid bool (0 or 1): 2\r\n"));

  send_cmd("kp invalid_flt\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid float: invalid_flt\r\n"));
}

void test_cmd_error_unknown_command(void) {
  send_cmd("bogus_command 123\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: unknown command: bogus_command\r\n"));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_cmd_blank_set_and_query);
  RUN_TEST(test_cmd_t_set_and_query);
  RUN_TEST(test_cmd_tcurve_set_and_query);
  RUN_TEST(test_cmd_stall_set_and_query);
  RUN_TEST(test_cmd_kfree_set_and_query);
  RUN_TEST(test_cmd_blink_set_and_query);
  RUN_TEST(test_cmd_odr_set_and_query);
  RUN_TEST(test_cmd_ratio_set_and_query);
  RUN_TEST(test_cmd_kp_kff_set_and_query);
  RUN_TEST(test_cmd_lim1_lim2);
  RUN_TEST(test_cmd_zero);
  RUN_TEST(test_cmd_pos_report_int64);
  RUN_TEST(test_cmd_r_state_report);
  RUN_TEST(test_cmd_help);
  RUN_TEST(test_cmd_save_success);
  RUN_TEST(test_cmd_save_failure);
  RUN_TEST(test_cmd_save_invalid_usage);
  RUN_TEST(test_cmd_error_invalid_usage);
  RUN_TEST(test_cmd_error_invalid_value);
  RUN_TEST(test_cmd_error_unknown_command);
  return UNITY_END();
}

