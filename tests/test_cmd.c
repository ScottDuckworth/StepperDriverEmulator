#include "unity.h"
#include "cmd.h"
#include "main.h"
#include "mathutil.h"

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

int32_t GetTension(void) { return mock_tension; }
void SetTension(int32_t tension) { mock_tension = tension; ReportTension(); }
void ReportTension(void) { ReportI32("t", GetTension()); }

void SetTorqueLUT(uint32_t delta_v, uint8_t count, const int32_t* table) {
  if (delta_v == 0 || count < 2 || count > TCURVE_MAX_POINTS || !table) return;
  mock_config.persistent.tcurve_delta_v = delta_v;
  mock_config.persistent.tcurve_point_count = count;
  memcpy(mock_config.persistent.tcurve_table, table, count * sizeof(int32_t));
  ReportTorqueLUT();
}

void ReportTorqueLUT(void) {
  WriteString("tlut ");
  char num_buf[16];
  MathUtil_FormatU32(num_buf, sizeof(num_buf), mock_config.persistent.tcurve_delta_v);
  WriteString(num_buf);
  for (uint8_t i = 0; i < mock_config.persistent.tcurve_point_count; ++i) {
    WriteString(" ");
    MathUtil_FormatI32(num_buf, sizeof(num_buf), mock_config.persistent.tcurve_table[i]);
    WriteString(num_buf);
  }
  WriteString("\r\n");
}

uint32_t GetStallThreshold(void) { return mock_config.persistent.stall_threshold; }
void SetStallThreshold(uint32_t threshold) { mock_config.persistent.stall_threshold = threshold; ReportStallThreshold(); }
void ReportStallThreshold(void) { ReportU32("stall", GetStallThreshold()); }

q12_t GetKfree(void) { return mock_config.persistent.kfree; }
void SetKfree(q12_t kfree) { mock_config.persistent.kfree = kfree; ReportKfree(); }
void ReportKfree(void) { ReportQ12("kfree", GetKfree()); }

bool GetStallTrip(void) { return mock_stall_trip; }
void ReportStallTrip(void) { ReportU8("stall_trip", GetStallTrip()); }

bool GetBlinkMode(void) { return mock_blink; }
void SetBlinkMode(bool enable) { mock_blink = enable; ReportBlinkMode(); }
void ReportBlinkMode(void) { ReportU8("blink", GetBlinkMode()); }

void GetDefaultHardwareName(char* out_name, size_t max_len) {
  if (!out_name || max_len == 0) return;
  strncpy(out_name, "001122334455", max_len);
  out_name[max_len - 1] = '\0';
}

const char* GetName(void) {
  if (mock_config.persistent.name[0] == '\0') {
    GetDefaultHardwareName(mock_config.persistent.name, sizeof(mock_config.persistent.name));
  }
  return mock_config.persistent.name;
}

void SetName(const char* name) {
  if (name && name[0] != '\0') {
    strncpy(mock_config.persistent.name, name, sizeof(mock_config.persistent.name) - 1);
    mock_config.persistent.name[sizeof(mock_config.persistent.name) - 1] = '\0';
  } else {
    GetDefaultHardwareName(mock_config.persistent.name, sizeof(mock_config.persistent.name));
  }
  ReportName();
}

void ReportName(void) {
  ReportString("name", GetName());
}

uint16_t GetOdr(void) { return mock_config.persistent.odr; }
void SetOdr(uint16_t odr) { mock_config.persistent.odr = odr; ReportOdr(); }
void ReportOdr(void) { ReportU16("odr", GetOdr()); }

bool SetRatio(uint16_t spr, uint16_t epr) {
  if (spr == 0 || epr == 0) return false;
  uint16_t g = MathUtil_GCD(spr, epr);
  mock_config.persistent.ratio_spr = spr / g;
  mock_config.persistent.ratio_epr = epr / g;
  mock_config.cached.counts_per_step = MathUtil_RatioQ12(mock_config.persistent.ratio_epr, mock_config.persistent.ratio_spr);
  mock_config.cached.inv_counts_per_step = MathUtil_RatioQ16(mock_config.persistent.ratio_spr, mock_config.persistent.ratio_epr);
  ReportRatio();
  return true;
}

void GetRatio(uint16_t* out_spr, uint16_t* out_epr) {
  if (out_spr) *out_spr = mock_config.persistent.ratio_spr;
  if (out_epr) *out_epr = mock_config.persistent.ratio_epr;
}

void ReportRatio(void) {
  char buf[32];
  int size = snprintf(buf, sizeof(buf), "ratio %u %u\r\n", mock_config.persistent.ratio_spr, mock_config.persistent.ratio_epr);
  if (size > 0) WriteData((const uint8_t*) buf, (uint16_t) size);
}

q12_t GetKp(void) { return mock_config.persistent.kp; }
void SetKp(q12_t kp) { mock_config.persistent.kp = kp; ReportKp(); }
void ReportKp(void) { ReportQ12("kp", GetKp()); }

q12_t GetKff(void) { return mock_config.persistent.kff; }
void SetKff(q12_t kff) { mock_config.persistent.kff = kff; ReportKff(); }
void ReportKff(void) { ReportQ12("kff", GetKff()); }

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

static int32_t mock_velocity = 0;
static int32_t mock_motor_torque = 1000;
static int32_t mock_net_torque = 1000;

void GetInstantaneousMotionState(int32_t* out_velocity_hz, int32_t* out_motor_torque, int32_t* out_net_torque) {
  if (out_velocity_hz) *out_velocity_hz = mock_velocity;
  if (out_motor_torque) *out_motor_torque = mock_motor_torque;
  if (out_net_torque) *out_net_torque = mock_net_torque;
}

void ReportPvt(void) {
  char buf[80];
  snprintf(buf, sizeof(buf), "pvt %lld %ld %ld %ld\r\n", (long long) mock_pos, (long) mock_velocity, (long) mock_motor_torque, (long) mock_net_torque);
  WriteString(buf);
}

static q12_t mock_blank = Q12_INIT_RATIO(7, 2);

q12_t GetStepBlanking(void) { return mock_blank; }
void SetStepBlanking(q12_t blank_us) { mock_blank = blank_us; ReportStepBlanking(); }
void ReportStepBlanking(void) { ReportQ12("blank", GetStepBlanking()); }

bool SaveConfig(void) {
  return mock_save_success;
}

int64_t GetMinStop(void) { return mock_config.persistent.minstop; }
bool SetMinStop(int64_t min_stop) {
  if (min_stop > mock_config.persistent.maxstop) return false;
  mock_config.persistent.minstop = min_stop;
  ReportMinStop();
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

int64_t GetMaxStop(void) { return mock_config.persistent.maxstop; }
bool SetMaxStop(int64_t max_stop) {
  if (max_stop < mock_config.persistent.minstop) return false;
  mock_config.persistent.maxstop = max_stop;
  ReportMaxStop();
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
  mock_velocity = 0;
  mock_motor_torque = 1000;
  mock_net_torque = 1000;
  mock_lim1 = false;
  mock_lim2 = false;
  mock_blink = false;
  mock_stall_trip = false;
  mock_save_success = true;
  mock_blank = Q12_RATIO(7, 2);
  clear_output();
}

void tearDown(void) {}

// --- Unit Tests ---

void test_cmd_blank_set_and_query(void) {
  send_cmd("blank 5.0\r\n");
  TEST_ASSERT_EQUAL_INT32(20480, mock_blank.raw);
  TEST_ASSERT_EQUAL_STRING("blank 5.0000\r\n", captured_output);

  send_cmd("blank 3.5\r\n");
  TEST_ASSERT_EQUAL_INT32(14336, mock_blank.raw);
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

void test_cmd_tlut_set_and_query(void) {
  send_cmd("tlut 100 9400 8800 7300 5700 4400 3500 2800 2400 2000 1800 1500 1400 1050 1000 950 900\r\n");
  TEST_ASSERT_EQUAL_UINT32(100, mock_config.persistent.tcurve_delta_v);
  TEST_ASSERT_EQUAL_UINT8(16, mock_config.persistent.tcurve_point_count);
  TEST_ASSERT_EQUAL_INT32(9400, mock_config.persistent.tcurve_table[0]);
  TEST_ASSERT_EQUAL_INT32(900, mock_config.persistent.tcurve_table[15]);
  TEST_ASSERT_EQUAL_STRING("tlut 100 9400 8800 7300 5700 4400 3500 2800 2400 2000 1800 1500 1400 1050 1000 950 900\r\n", captured_output);

  send_cmd("tlut\r\n");
  TEST_ASSERT_EQUAL_STRING("tlut 100 9400 8800 7300 5700 4400 3500 2800 2400 2000 1800 1500 1400 1050 1000 950 900\r\n", captured_output);

  send_cmd("tlut 0 100 200\r\n");
  TEST_ASSERT_EQUAL_STRING("error: invalid uint32: 0\r\n", captured_output);

  send_cmd("tlut 100 1000\r\n");
  TEST_ASSERT_EQUAL_STRING("error: invalid usage: tlut [delta_v] [T0] [T1] ... [TN]\r\n", captured_output);
}

void test_cmd_stall_set_and_query(void) {
  send_cmd("stall 2500\r\n");
  TEST_ASSERT_EQUAL_UINT32(2500, mock_config.persistent.stall_threshold);
  TEST_ASSERT_EQUAL_STRING("stall 2500\r\n", captured_output);

  send_cmd("stall\r\n");
  TEST_ASSERT_EQUAL_STRING("stall 2500\r\n", captured_output);
}

void test_cmd_kfree_set_and_query(void) {
  send_cmd("kfree 0.0125\r\n");
  TEST_ASSERT_EQUAL_INT32(51, mock_config.persistent.kfree.raw);
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
  TEST_ASSERT_EQUAL_UINT16(250, mock_config.persistent.odr);
  TEST_ASSERT_EQUAL_STRING("odr 250\r\n", captured_output);

  send_cmd("odr\r\n");
  TEST_ASSERT_EQUAL_STRING("odr 250\r\n", captured_output);
}

void test_cmd_ratio_set_and_query(void) {
  // Set with ratio 1000 4000 -> reduced to 1 4
  send_cmd("ratio 1000 4000\r\n");
  TEST_ASSERT_EQUAL_UINT16(1, mock_config.persistent.ratio_spr);
  TEST_ASSERT_EQUAL_UINT16(4, mock_config.persistent.ratio_epr);
  TEST_ASSERT_EQUAL_INT32(16384, mock_config.cached.counts_per_step.raw);
  TEST_ASSERT_EQUAL_INT32(16384, mock_config.cached.inv_counts_per_step.raw);
  TEST_ASSERT_EQUAL_STRING("ratio 1 4\r\n", captured_output);

  // Set with ratio 200 1024 -> reduced to 25 128 (GCD 8)
  send_cmd("ratio 200 1024\r\n");
  TEST_ASSERT_EQUAL_UINT16(25, mock_config.persistent.ratio_spr);
  TEST_ASSERT_EQUAL_UINT16(128, mock_config.persistent.ratio_epr);
  TEST_ASSERT_EQUAL_INT32(20971, mock_config.cached.counts_per_step.raw);
  TEST_ASSERT_EQUAL_INT32(12800, mock_config.cached.inv_counts_per_step.raw);
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
  TEST_ASSERT_EQUAL_INT32(1024, mock_config.persistent.kp.raw);
  TEST_ASSERT_EQUAL_STRING("kp 0.2500\r\n", captured_output);

  send_cmd("kff 1.5\r\n");
  TEST_ASSERT_EQUAL_INT32(6144, mock_config.persistent.kff.raw);
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

void test_cmd_pos(void) {
  // Query current position
  mock_pos = 123456789012LL;
  send_cmd("pos\r\n");
  TEST_ASSERT_EQUAL_STRING("pos 123456789012\r\n", captured_output);

  // Set to 0
  send_cmd("pos 0\r\n");
  TEST_ASSERT_EQUAL_INT64(0, mock_pos);
  TEST_ASSERT_EQUAL_STRING("pos 0\r\n", captured_output);

  // Set to positive
  send_cmd("pos 42\r\n");
  TEST_ASSERT_EQUAL_INT64(42, mock_pos);
  TEST_ASSERT_EQUAL_STRING("pos 42\r\n", captured_output);

  // Set to negative
  send_cmd("pos -100\r\n");
  TEST_ASSERT_EQUAL_INT64(-100, mock_pos);
  TEST_ASSERT_EQUAL_STRING("pos -100\r\n", captured_output);

  // Set to large 64-bit value
  send_cmd("pos 5000000000000\r\n");
  TEST_ASSERT_EQUAL_INT64(5000000000000LL, mock_pos);
  TEST_ASSERT_EQUAL_STRING("pos 5000000000000\r\n", captured_output);

  // Set to large negative 64-bit value
  send_cmd("pos -5000000000000\r\n");
  TEST_ASSERT_EQUAL_INT64(-5000000000000LL, mock_pos);
  TEST_ASSERT_EQUAL_STRING("pos -5000000000000\r\n", captured_output);

  // Set to INT64_MAX
  send_cmd("pos 9223372036854775807\r\n");
  TEST_ASSERT_EQUAL_INT64(9223372036854775807LL, mock_pos);
  TEST_ASSERT_EQUAL_STRING("pos 9223372036854775807\r\n", captured_output);

  // Set to INT64_MIN
  send_cmd("pos -9223372036854775808\r\n");
  TEST_ASSERT_EQUAL_INT64(-9223372036854775807LL - 1LL, mock_pos);
  TEST_ASSERT_EQUAL_STRING("pos -9223372036854775808\r\n", captured_output);
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

void test_cmd_name_set_and_query(void) {
  // Query default name (hardware ID mock)
  send_cmd("name\r\n");
  TEST_ASSERT_EQUAL_STRING("name 001122334455\r\n", captured_output);

  // Set new name
  send_cmd("name StepperX\r\n");
  TEST_ASSERT_EQUAL_STRING("StepperX", mock_config.persistent.name);
  TEST_ASSERT_EQUAL_STRING("name StepperX\r\n", captured_output);

  // Query updated name
  send_cmd("name\r\n");
  TEST_ASSERT_EQUAL_STRING("name StepperX\r\n", captured_output);

  // Name longer than 31 chars should truncate safely to 31 chars
  send_cmd("name 12345678901234567890123456789012345\r\n");
  TEST_ASSERT_EQUAL_STRING("1234567890123456789012345678901", mock_config.persistent.name);
  TEST_ASSERT_EQUAL_STRING("name 1234567890123456789012345678901\r\n", captured_output);
}

void test_cmd_r_state_report(void) {
  send_cmd("r\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "name 001122334455\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "odr 1000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "ratio 1 4\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "kp 0.1001\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "kff 1.0000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "blank 3.5000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "tlut 250 1000 "));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "stall 4000\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "minstop none\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "maxstop none\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "kfree 0.0049\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "stall_trip 0\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "blink 0\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "pos 0\r\n"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "pvt 0 0 1000 1000\r\n"));
}

void test_cmd_pvt(void) {
  mock_pos = 12345;
  mock_velocity = 20000;
  mock_motor_torque = 850;
  mock_net_torque = 350;

  send_cmd("pvt\r\n");
  TEST_ASSERT_EQUAL_STRING("pvt 12345 20000 850 350\r\n", captured_output);

  mock_velocity = -15000;
  send_cmd("pvt\r\n");
  TEST_ASSERT_EQUAL_STRING("pvt 12345 -15000 850 350\r\n", captured_output);

  mock_velocity = 0;
  send_cmd("pvt\r\n");
  TEST_ASSERT_EQUAL_STRING("pvt 12345 0 850 350\r\n", captured_output);
}

void test_cmd_help(void) {
  send_cmd("help\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "name [string]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "t [int32]"));
  TEST_ASSERT_NULL(strstr(captured_output, "tcurve"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "tlut [delta_v] [T0] [T1] ... [TN]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "stall [uint32]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "blank [float]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "pos [int64]"));
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "pvt"));
  TEST_ASSERT_NULL(strstr(captured_output, "zero"));
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
  send_cmd("name foo bar\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid usage: name [string]\r\n"));

  send_cmd("t 1 2 3\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid usage: t [int32]\r\n"));

  send_cmd("pos 1 2\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid usage: pos [int64]\r\n"));

  send_cmd("pvt 123\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid usage: pvt\r\n"));
}

void test_cmd_error_invalid_value(void) {
  send_cmd("t abc\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid int32: abc\r\n"));

  send_cmd("blink 2\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid bool (0 or 1): 2\r\n"));

  send_cmd("kp invalid_flt\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid float: invalid_flt\r\n"));

  send_cmd("pos abc\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid int64: abc\r\n"));

  send_cmd("pos 9223372036854775808\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid int64: 9223372036854775808\r\n"));

  send_cmd("pos -9223372036854775809\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: invalid int64: -9223372036854775809\r\n"));
}

void test_cmd_error_unknown_command(void) {
  send_cmd("bogus_command 123\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: unknown command: bogus_command\r\n"));

  send_cmd("zero\r\n");
  TEST_ASSERT_NOT_NULL(strstr(captured_output, "error: unknown command: zero\r\n"));
}

void test_cmd_minstop_set_and_query(void) {
  send_cmd("minstop\r\n");
  TEST_ASSERT_EQUAL_STRING("minstop none\r\n", captured_output);

  send_cmd("minstop -5000\r\n");
  TEST_ASSERT_EQUAL_INT64(-5000, mock_config.persistent.minstop);
  TEST_ASSERT_EQUAL_STRING("minstop -5000\r\n", captured_output);

  send_cmd("minstop\r\n");
  TEST_ASSERT_EQUAL_STRING("minstop -5000\r\n", captured_output);

  send_cmd("minstop clear\r\n");
  TEST_ASSERT_EQUAL_INT64(INT64_MIN, mock_config.persistent.minstop);
  TEST_ASSERT_EQUAL_STRING("minstop none\r\n", captured_output);

  send_cmd("minstop -1234567890123\r\n");
  TEST_ASSERT_EQUAL_INT64(-1234567890123LL, mock_config.persistent.minstop);
  TEST_ASSERT_EQUAL_STRING("minstop -1234567890123\r\n", captured_output);

  send_cmd("minstop none\r\n");
  TEST_ASSERT_EQUAL_INT64(INT64_MIN, mock_config.persistent.minstop);
  TEST_ASSERT_EQUAL_STRING("minstop none\r\n", captured_output);

  // Invalid value parsing
  send_cmd("minstop abc\r\n");
  TEST_ASSERT_EQUAL_STRING("error: invalid int64: abc\r\n", captured_output);

  // Invalid usage (too many args)
  send_cmd("minstop 100 200\r\n");
  TEST_ASSERT_EQUAL_STRING("error: invalid usage: minstop [int64|none|clear]\r\n", captured_output);
}

void test_cmd_maxstop_set_and_query(void) {
  send_cmd("maxstop\r\n");
  TEST_ASSERT_EQUAL_STRING("maxstop none\r\n", captured_output);

  send_cmd("maxstop 5000\r\n");
  TEST_ASSERT_EQUAL_INT64(5000, mock_config.persistent.maxstop);
  TEST_ASSERT_EQUAL_STRING("maxstop 5000\r\n", captured_output);

  send_cmd("maxstop\r\n");
  TEST_ASSERT_EQUAL_STRING("maxstop 5000\r\n", captured_output);

  send_cmd("maxstop clear\r\n");
  TEST_ASSERT_EQUAL_INT64(INT64_MAX, mock_config.persistent.maxstop);
  TEST_ASSERT_EQUAL_STRING("maxstop none\r\n", captured_output);

  send_cmd("maxstop 9876543210987\r\n");
  TEST_ASSERT_EQUAL_INT64(9876543210987LL, mock_config.persistent.maxstop);
  TEST_ASSERT_EQUAL_STRING("maxstop 9876543210987\r\n", captured_output);

  send_cmd("maxstop none\r\n");
  TEST_ASSERT_EQUAL_INT64(INT64_MAX, mock_config.persistent.maxstop);
  TEST_ASSERT_EQUAL_STRING("maxstop none\r\n", captured_output);

  // Invalid value parsing
  send_cmd("maxstop xyz\r\n");
  TEST_ASSERT_EQUAL_STRING("error: invalid int64: xyz\r\n", captured_output);

  // Invalid usage (too many args)
  send_cmd("maxstop 100 200\r\n");
  TEST_ASSERT_EQUAL_STRING("error: invalid usage: maxstop [int64|none|clear]\r\n", captured_output);
}

void test_cmd_stops_order_validation(void) {
  send_cmd("minstop 1000\r\n");
  TEST_ASSERT_EQUAL_INT64(1000, mock_config.persistent.minstop);

  // maxstop 500 violates minstop 1000 <= maxstop
  send_cmd("maxstop 500\r\n");
  TEST_ASSERT_EQUAL_STRING("error: minstop exceeds maxstop\r\n", captured_output);
  TEST_ASSERT_EQUAL_INT64(INT64_MAX, mock_config.persistent.maxstop);

  send_cmd("maxstop 2000\r\n");
  TEST_ASSERT_EQUAL_INT64(2000, mock_config.persistent.maxstop);

  // minstop 3000 violates minstop <= maxstop 2000
  send_cmd("minstop 3000\r\n");
  TEST_ASSERT_EQUAL_STRING("error: minstop exceeds maxstop\r\n", captured_output);
  TEST_ASSERT_EQUAL_INT64(1000, mock_config.persistent.minstop);

  // minstop == maxstop is valid
  send_cmd("minstop 2000\r\n");
  TEST_ASSERT_EQUAL_INT64(2000, mock_config.persistent.minstop);
  TEST_ASSERT_EQUAL_INT64(2000, mock_config.persistent.maxstop);
  TEST_ASSERT_EQUAL_STRING("minstop 2000\r\n", captured_output);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_cmd_name_set_and_query);
  RUN_TEST(test_cmd_blank_set_and_query);
  RUN_TEST(test_cmd_t_set_and_query);
  RUN_TEST(test_cmd_tlut_set_and_query);
  RUN_TEST(test_cmd_stall_set_and_query);
  RUN_TEST(test_cmd_minstop_set_and_query);
  RUN_TEST(test_cmd_maxstop_set_and_query);
  RUN_TEST(test_cmd_stops_order_validation);
  RUN_TEST(test_cmd_kfree_set_and_query);
  RUN_TEST(test_cmd_blink_set_and_query);
  RUN_TEST(test_cmd_odr_set_and_query);
  RUN_TEST(test_cmd_ratio_set_and_query);
  RUN_TEST(test_cmd_kp_kff_set_and_query);
  RUN_TEST(test_cmd_lim1_lim2);
  RUN_TEST(test_cmd_pos);
  RUN_TEST(test_cmd_pvt);
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

