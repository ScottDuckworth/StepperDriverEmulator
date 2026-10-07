#include "cmd.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ARGC 36
static int argc;
static char* argv[MAX_ARGC];
static char cmd_buffer[256];
static uint16_t cmd_buffer_size;

typedef struct Command {
  const char* name;
  const char* usage;
  void (*impl)(const struct Command* self);
} Command_t;



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
  if (!MathUtil_ParseU8(argv[1], &limit)) { InvalidValue("uint8", argv[1]); return; }
  SetLimit1(limit);
}

static void Cmd_lim2(const Command_t* self) {
  if (argc != 2) { InvalidUsage(self->usage); return; }
  uint8_t limit;
  if (!MathUtil_ParseU8(argv[1], &limit)) { InvalidValue("uint8", argv[1]); return; }
  SetLimit2(limit);
}

static void Cmd_t(const Command_t* self) {
  if (argc == 1) {
    ReportTension();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  int32_t value;
  if (!MathUtil_ParseI32(argv[1], &value)) { InvalidValue("int32", argv[1]); return; }
  SetTension(value);
}

static void Cmd_tlut(const Command_t* self) {
  if (argc == 1) {
    ReportTorqueLUT();
    return;
  }
  if (argc < 4 || argc > 35) { InvalidUsage(self->usage); return; }
  uint32_t delta_v;
  if (!MathUtil_ParseU32(argv[1], &delta_v) || delta_v == 0) { InvalidValue("uint32", argv[1]); return; }

  uint8_t count = (uint8_t)(argc - 2);
  int32_t table[TCURVE_MAX_POINTS];
  for (uint8_t i = 0; i < count; ++i) {
    if (!MathUtil_ParseI32(argv[2 + i], &table[i])) {
      InvalidValue("int32", argv[2 + i]);
      return;
    }
  }
  SetTorqueLUT(delta_v, count, table);
}

static void Cmd_stall(const Command_t* self) {
  if (argc == 1) {
    ReportStallThreshold();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  uint32_t value;
  if (!MathUtil_ParseU32(argv[1], &value)) { InvalidValue("uint32", argv[1]); return; }
  SetStallThreshold(value);
}

static void Cmd_kfree(const Command_t* self) {
  if (argc == 1) {
    ReportKfree();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  q12_t value;
  if (!MathUtil_ParseQ12(argv[1], &value)) { InvalidValue("float", argv[1]); return; }
  SetKfree(value);
}

static void Cmd_pos(const Command_t* self) {
  if (argc == 1) {
    ReportEncoderPosition();
    return;
  }
  if (argc != 2) {
    InvalidUsage(self->usage);
    return;
  }
  int64_t value;
  if (!MathUtil_ParseI64(argv[1], &value)) {
    InvalidValue("int64", argv[1]);
    return;
  }
  SetEncoderPosition(value);
}

static void Cmd_pvt(const Command_t* self) {
  if (argc != 1) {
    InvalidUsage(self->usage);
    return;
  }
  ReportPvt();
}

static void Cmd_blink(const Command_t* self) {
  if (argc == 1) {
    ReportBlinkMode();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  uint8_t value;
  if (!MathUtil_ParseU8(argv[1], &value) || (value != 0 && value != 1)) {
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
  if (!MathUtil_ParseU16(argv[1], &value)) { InvalidValue("uint16", argv[1]); return; }
  SetOdr(value);
}

static void Cmd_ratio(const Command_t* self) {
  if (argc == 1) {
    ReportRatio();
    return;
  }
  if (argc != 3) { InvalidUsage(self->usage); return; }
  uint16_t spr, epr;
  if (!MathUtil_ParseU16(argv[1], &spr) || spr == 0) { InvalidValue("uint16 > 0", argv[1]); return; }
  if (!MathUtil_ParseU16(argv[2], &epr) || epr == 0) { InvalidValue("uint16 > 0", argv[2]); return; }
  SetRatio(spr, epr);
}

static void Cmd_kp(const Command_t* self) {
  if (argc == 1) {
    ReportKp();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  q12_t value;
  if (!MathUtil_ParseQ12(argv[1], &value)) { InvalidValue("float", argv[1]); return; }
  SetKp(value);
}

static void Cmd_kff(const Command_t* self) {
  if (argc == 1) {
    ReportKff();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  q12_t value;
  if (!MathUtil_ParseQ12(argv[1], &value)) { InvalidValue("float", argv[1]); return; }
  SetKff(value);
}

static void Cmd_blank(const Command_t* self) {
  if (argc == 1) {
    ReportStepBlanking();
    return;
  }
  if (argc != 2) { InvalidUsage(self->usage); return; }
  q12_t value;
  if (!MathUtil_ParseQ12(argv[1], &value)) { InvalidValue("float", argv[1]); return; }
  SetStepBlanking(value);
}

static void Cmd_name(const Command_t* self) {
  if (argc == 1) {
    ReportName();
    return;
  }
  if (argc != 2) {
    InvalidUsage(self->usage);
    return;
  }
  SetName(argv[1]);
}

static void Cmd_r(const Command_t* self) {
  if (argc != 1) { InvalidUsage(self->usage); return; }
  ReportName();
  ReportOdr();
  ReportRatio();
  ReportKp();
  ReportKff();
  ReportStepBlanking();
  ReportLimit1();
  ReportLimit2();
  ReportTension();
  ReportTorqueLUT();
  ReportStallThreshold();
  ReportKfree();
  ReportStallTrip();
  ReportBlinkMode();
  ReportStepReverse();
  ReportStepEnabled();
  ReportEncoderPosition();
  ReportPvt();
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
    {"name", "name [string]", Cmd_name},
    {"lim1", "lim1 <uint8>", Cmd_lim1},
    {"lim2", "lim2 <uint8>", Cmd_lim2},
    {"t", "t [int32]", Cmd_t},
    {"tlut", "tlut [delta_v] [T0] [T1] ... [TN]", Cmd_tlut},
    {"stall", "stall [uint32]", Cmd_stall},
    {"kfree", "kfree [float]", Cmd_kfree},
    {"blank", "blank [float]", Cmd_blank},
    {"pos", "pos [int64]", Cmd_pos},
    {"pvt", "pvt", Cmd_pvt},
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
