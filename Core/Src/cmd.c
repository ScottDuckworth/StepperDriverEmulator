#include "cmd.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ARGC 4
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
    float div = 10.0f;
    while (*str >= '0' && *str <= '9') {
      val += (float)(*str - '0') / div;
      div *= 10.0f;
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
  if (argc != 2) return InvalidUsage(self->usage);
  uint8_t limit;
  if (!StrToU8(argv[1], &limit)) return InvalidValue("uint8", argv[1]);
  SetLimit1(limit);
}

static void Cmd_lim2(const Command_t* self) {
  if (argc != 2) return InvalidUsage(self->usage);
  uint8_t limit;
  if (!StrToU8(argv[1], &limit)) return InvalidValue("uint8", argv[1]);
  SetLimit2(limit);
}

static void Cmd_od(const Command_t* self) {
  if (argc != 1) return InvalidUsage(self->usage);
  SetOverrideDisable();
}

static void Cmd_ot(const Command_t* self) {
  if (argc != 2) return InvalidUsage(self->usage);
  int32_t value;
  if (!StrToI32(argv[1], &value)) return InvalidValue("int32", argv[1]);
  SetOverrideTarget(value);
}

static void Cmd_or(const Command_t* self) {
  if (argc != 2) return InvalidUsage(self->usage);
  int32_t value;
  if (!StrToI32(argv[1], &value)) return InvalidValue("int32", argv[1]);
  SetOverrideRate(value);
}

static void Cmd_zero(const Command_t* self) {
  if (argc != 1) return InvalidUsage(self->usage);
  SetEncoderPosition(0);
}

static void Cmd_led(const Command_t* self) {
  if (argc != 3) return InvalidUsage(self->usage);
  uint8_t r, g;
  if (!StrToU8(argv[1], &r)) return InvalidValue("uint8", argv[1]);
  if (!StrToU8(argv[2], &g)) return InvalidValue("uint8", argv[2]);
  SetLED(r, g);
}

static void Cmd_odr(const Command_t* self) {
  if (argc != 2) return InvalidUsage(self->usage);
  if (!StrToU16(argv[1], &GetConfig()->odr)) return InvalidValue("uint16", argv[1]);
  ReportU16("odr", GetConfig()->odr);
}

static void Cmd_epr(const Command_t* self) {
  if (argc != 2) return InvalidUsage(self->usage);
  if (!StrToU16(argv[1], &GetConfig()->epr)) return InvalidValue("uint16", argv[1]);
  ReportU16("epr", GetConfig()->epr);
}

static void Cmd_spr(const Command_t* self) {
  if (argc != 2) return InvalidUsage(self->usage);
  if (!StrToU16(argv[1], &GetConfig()->spr)) return InvalidValue("uint16", argv[1]);
  ReportU16("spr", GetConfig()->spr);
}

static void Cmd_kp(const Command_t* self) {
  if (argc == 1) {
    ReportKp();
    return;
  }
  if (argc != 2) return InvalidUsage(self->usage);
  float value;
  if (!StrToFloat(argv[1], &value)) return InvalidValue("float", argv[1]);
  SetKp(value);
}

static void Cmd_kff(const Command_t* self) {
  if (argc == 1) {
    ReportKff();
    return;
  }
  if (argc != 2) return InvalidUsage(self->usage);
  float value;
  if (!StrToFloat(argv[1], &value)) return InvalidValue("float", argv[1]);
  SetKff(value);
}

static void Cmd_r(const Command_t* self) {
  if (argc != 1) return InvalidUsage(self->usage);
  ReportU16("odr", GetConfig()->odr);
  ReportU16("epr", GetConfig()->epr);
  ReportU16("spr", GetConfig()->spr);
  ReportKp();
  ReportKff();
  ReportLimit1();
  ReportLimit2();
  ReportOverrideTarget();
  ReportOverrideRate();
  ReportOverrideState();
  ReportStepReverse();
  ReportStepEnabled();
  ReportEncoderPosition();
}

static void Cmd_help(const Command_t* self);

static const Command_t commands[] = {
    {"lim1", "lim1 <uint8>", Cmd_lim1},
    {"lim2", "lim2 <uint8>", Cmd_lim2},
    {"od", "od", Cmd_od},
    {"ot", "ot <int32>", Cmd_ot},
    {"or", "or <int32>", Cmd_or},
    {"zero", "zero", Cmd_zero},
    {"led", "led <red:uint8> <green:uint8>", Cmd_led},
    {"odr", "odr <uint16>", Cmd_odr},
    {"epr", "epr <uint16>", Cmd_epr},
    {"spr", "spr <uint16>", Cmd_spr},
    {"kp", "kp [float]", Cmd_kp},
    {"kff", "kff [float]", Cmd_kff},
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
      return (*cmd->impl)(cmd);
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
