#include "cmd.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ARGC 6
static int argc;
static char* argv[MAX_ARGC];
static char cmd_buffer[80];
static uint16_t cmd_buffer_size;

typedef struct Command {
  const char* name;
  const char* usage;
  void (*impl)(const struct Command* self);
} Command_t;

static bool StrToI32(const char* str, int32_t* dst) {
  long value;
  char* end;
  value = strtol(str, &end, 0);
  if (*end != '\0' || value > INT32_MAX || value < INT32_MIN) return false;
  *dst = value;
  return true;
}

static bool StrToU32(const char* str, uint32_t* dst) {
  unsigned long value;
  char* end;
  value = strtoul(str, &end, 0);
  if (*end != '\0' || value > UINT32_MAX) return false;
  *dst = (uint32_t) value;
  return true;
}

static bool StrToU16(const char* str, uint16_t* dst) {
  unsigned long value;
  char* end;
  value = strtoul(str, &end, 0);
  if (*end != '\0' || value > UINT16_MAX) return false;
  *dst = value;
  return true;
}

static bool StrToU8(const char* str, uint8_t* dst) {
  unsigned long value;
  char* end;
  value = strtoul(str, &end, 0);
  if (*end != '\0' || value > UINT8_MAX) return false;
  *dst = value;
  return true;
}

static bool StrToFloat(const char* str, float* dst) {
  if (!str || !*str) return false;
  float sign = 1.0f;
  if (*str == '-') {
    sign = -1.0f;
    str++;
  } else if (*str == '+') {
    str++;
  }
  float val = 0.0f;
  bool has_digits = false;
  while (*str >= '0' && *str <= '9') {
    val = val * 10.0f + (float)(*str - '0');
    str++;
    has_digits = true;
  }
  if (*str == '.') {
    str++;
    float scale = 0.1f;
    while (*str >= '0' && *str <= '9') {
      val += (float)(*str - '0') * scale;
      scale *= 0.1f;
      str++;
      has_digits = true;
    }
  }
  if (!has_digits || *str != '\0') return false;
  *dst = sign * val;
  return true;
}

static void InvalidUsage(const char* usage) {
  WriteString("error: invalid usage: ");
  WriteString(usage);
  WriteString("\r\n");
}

static void InvalidValue(const char* type, const char* value) {
  WriteString("error: invalid ");
  WriteString(type);
  WriteString(": ");
  WriteString(value);
  WriteString("\r\n");
}

static void Cmd_lim1(const Command_t* self) {
  if (argc != 2) { InvalidUsage(self->usage); return; }
  uint8_t limit;
  if (!StrToU8(argv[1], &limit)) { InvalidValue("uint8", argv[1]); return; }
  SetLimit1(limit);
}

static void Cmd_lim2(const Command_t* self) {
  if (argc != 2) { InvalidUsage(self->usage); return; }
  uint8_t limit;
  if (!StrToU8(argv[1], &limit)) { InvalidValue("uint8", argv[1]); return; }
  SetLimit2(limit);
}

static void Cmd_t(const Command_t* self) {
  if (argc == 1) {
    ReportTension();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  int32_t value;
  if (!StrToI32(argv[1], &value)) { InvalidValue("int32", argv[1]); return; }
  SetTension(value);
}

static void Cmd_tcurve(const Command_t* self) {
  if (argc == 1) {
    ReportTorqueCurve();
    return;
  }
  if (argc != 5) { InvalidUsage(self->usage); return; }
  int32_t t0, t_min;
  uint32_t v_knee, v_max;
  if (!StrToI32(argv[1], &t0)) { InvalidValue("int32", argv[1]); return; }
  if (!StrToU32(argv[2], &v_knee)) { InvalidValue("uint32", argv[2]); return; }
  if (!StrToU32(argv[3], &v_max)) { InvalidValue("uint32", argv[3]); return; }
  if (!StrToI32(argv[4], &t_min)) { InvalidValue("int32", argv[4]); return; }
  SetTorqueCurve(t0, v_knee, v_max, t_min);
}

static void Cmd_stall(const Command_t* self) {
  if (argc == 1) {
    ReportStallThreshold();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  uint32_t value;
  if (!StrToU32(argv[1], &value)) { InvalidValue("uint32", argv[1]); return; }
  SetStallThreshold(value);
}

static void Cmd_kfree(const Command_t* self) {
  if (argc == 1) {
    ReportKfree();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  float value;
  if (!StrToFloat(argv[1], &value)) { InvalidValue("float", argv[1]); return; }
  SetKfree(value);
}

static void Cmd_zero(const Command_t* self) {
  if (argc != 1) { InvalidUsage(self->usage); return; }
  SetEncoderPosition(0);
}

static void Cmd_blink(const Command_t* self) {
  if (argc == 1) {
    ReportBlinkMode();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  uint8_t value;
  if (!StrToU8(argv[1], &value) || (value != 0 && value != 1)) {
    InvalidValue("bool (0 or 1)", argv[1]);
    return;
  }
  SetBlinkMode(value != 0);
}

static void Cmd_odr(const Command_t* self) {
  if (argc == 1) {
    ReportOdr();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  uint16_t value;
  if (!StrToU16(argv[1], &value)) { InvalidValue("uint16", argv[1]); return; }
  SetOdr(value);
}

static void Cmd_ratio(const Command_t* self) {
  if (argc == 1) {
    ReportRatio();
    return;
  }
  if (argc != 3) { InvalidUsage(self->usage); return; }
  uint16_t spr, epr;
  if (!StrToU16(argv[1], &spr) || spr == 0) { InvalidValue("uint16 > 0", argv[1]); return; }
  if (!StrToU16(argv[2], &epr) || epr == 0) { InvalidValue("uint16 > 0", argv[2]); return; }
  SetRatio(spr, epr);
}

static void Cmd_kp(const Command_t* self) {
  if (argc == 1) {
    ReportKp();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  float value;
  if (!StrToFloat(argv[1], &value)) { InvalidValue("float", argv[1]); return; }
  SetKp(value);
}

static void Cmd_kff(const Command_t* self) {
  if (argc == 1) {
    ReportKff();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  float value;
  if (!StrToFloat(argv[1], &value)) { InvalidValue("float", argv[1]); return; }
  SetKff(value);
}

static void Cmd_blank(const Command_t* self) {
  if (argc == 1) {
    ReportStepBlanking();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  float value;
  if (!StrToFloat(argv[1], &value)) { InvalidValue("float", argv[1]); return; }
  SetStepBlanking(value);
}

static void Cmd_r(const Command_t* self) {
  if (argc != 1) { InvalidUsage(self->usage); return; }
  ReportOdr();
  ReportRatio();
  ReportKp();
  ReportKff();
  ReportStepBlanking();
  ReportLimit1();
  ReportLimit2();
  ReportTension();
  ReportTorqueCurve();
  ReportStallThreshold();
  ReportKfree();
  ReportStallTrip();
  ReportBlinkMode();
  ReportStepReverse();
  ReportStepEnabled();
  ReportEncoderPosition();
}

static void Cmd_save(const Command_t* self) {
  if (argc != 1) { InvalidUsage(self->usage); return; }
  if (SaveConfig()) {
    WriteString("save ok\r\n");
  } else {
    WriteString("error: save failed\r\n");
  }
}

static void Cmd_help(const Command_t* self);

static const Command_t commands[] = {
    {"lim1", "lim1 <uint8>", Cmd_lim1},
    {"lim2", "lim2 <uint8>", Cmd_lim2},
    {"t", "t [int32]", Cmd_t},
    {"tcurve", "tcurve [T0] [V_knee] [V_max] [T_min]", Cmd_tcurve},
    {"stall", "stall [uint32]", Cmd_stall},
    {"kfree", "kfree [float]", Cmd_kfree},
    {"blank", "blank [float]", Cmd_blank},
    {"zero", "zero", Cmd_zero},
    {"blink", "blink [0|1]", Cmd_blink},
    {"odr", "odr <uint16>", Cmd_odr},
    {"ratio", "ratio [spr] [epr]", Cmd_ratio},
    {"kp", "kp [float]", Cmd_kp},
    {"kff", "kff [float]", Cmd_kff},
    {"save", "save", Cmd_save},
    {"r", "r", Cmd_r},
    {"help", "help", Cmd_help},
};

static void Cmd_help(const Command_t* self) {
  for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
    WriteString("  ");
    WriteString(commands[i].usage);
    WriteString("\r\n");
  }
}

static void CmdRun(void) {
  for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
    const Command_t* cmd = &commands[i];
    if (strcmp(argv[0], cmd->name) == 0) {
      (*cmd->impl)(cmd);
      return;
    }
  }

  WriteString("error: unknown command: ");
  WriteString(argv[0]);
  WriteString("\r\n");
}

static bool IsSpace(char c) {
  switch (c) {
    case ' ':
    case '\t':
    case '\r':
    case '\n':
      return true;
    default:
      return false;
  }
}

static void CmdParse(char* cmd, uint16_t size) {
  bool in_arg = false;
  argc = 0;

  for (uint16_t i = 0; i < size; ++i) {
    char* c = cmd + i;
    if (in_arg) {
      if (IsSpace(*c)) {
        *c = '\0';
        in_arg = false;
      }
    } else {
      if (!IsSpace(*c)) {
        if (argc < MAX_ARGC) {
          argv[argc++] = c;
          in_arg = true;
        }
      }
    }
  }

  if (argc > 0) {
    CmdRun();
  }
}

void CmdProcessInput(const char* input, uint16_t size) {
  for (uint16_t i = 0; i < size; ++i) {
    if (cmd_buffer_size < sizeof(cmd_buffer) - 1) {
      cmd_buffer[cmd_buffer_size++] = input[i];
    }
    if (input[i] == '\r' || input[i] == '\n') {
      cmd_buffer[sizeof(cmd_buffer) - 1] = '\0';
      CmdParse(cmd_buffer, cmd_buffer_size);
      memset(cmd_buffer, 0, sizeof(cmd_buffer));
      cmd_buffer_size = 0;
    }
  }
}
