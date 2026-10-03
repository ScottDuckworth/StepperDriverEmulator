#!/usr/bin/env python3
"""
Inspects an STM32 ELF binary and verifies that no floating-point division
or 64-bit integer division routines are linked into the executable.

On the ARM Cortex-M0 core (STM32F042), hardware division is not available
for floats or 64-bit integers. Using these operations invokes slow software
emulation routines (__aeabi_fdiv, __aeabi_ldivmod, etc.) that consume significant
Flash memory and CPU cycles in critical motion interrupt handlers.
"""

import sys
import struct
from pathlib import Path

DISALLOWED_SYMBOLS: dict[str, str] = {
    # Single-precision float division
    "__aeabi_fdiv": "Single-precision float division (causes ~140 cycles/div and ~556 bytes Flash bloat)",
    "__divsf3":     "GCC single-precision float division",

    # Double-precision float division
    "__aeabi_ddiv": "Double-precision float division (causes slow software emulation)",
    "__divdf3":     "GCC double-precision float division",

    # 64-bit integer division / modulo
    "__aeabi_ldivmod":  "Signed 64-bit integer division/modulo (causes slow software division)",
    "__aeabi_uldivmod": "Unsigned 64-bit integer division/modulo (causes slow software division)",
    "__divdi3":         "GCC signed 64-bit integer division",
    "__udivdi3":        "GCC unsigned 64-bit integer division",
    "__moddi3":         "GCC signed 64-bit integer modulo",
    "__umoddi3":        "GCC unsigned 64-bit integer modulo",
}


def parse_elf_symbols(elf_path: Path) -> dict[str, int]:
    """Parses defined symbols from an ELF32 binary and returns {symbol_name: address}."""
    data = elf_path.read_bytes()
    if len(data) < 52 or data[:4] != b"\x7fELF":
        raise ValueError(f"Invalid ELF file: {elf_path}")

    # ELF32 header parsing
    e_shoff, = struct.unpack_from("<I", data, 32)
    e_shentsize, e_shnum, e_shstrndx = struct.unpack_from("<HHH", data, 46)

    strtab_off, = struct.unpack_from("<I", data, e_shoff + e_shstrndx * e_shentsize + 16)
    strtab = data[strtab_off:]

    symtab_off = 0
    symtab_size = 0
    symstr_off = 0

    for i in range(e_shnum):
        sh = data[e_shoff + i * e_shentsize : e_shoff + (i + 1) * e_shentsize]
        name_idx, sh_type, sh_flags, sh_addr, sh_offset, sh_size, sh_link = struct.unpack_from("<IIIIIII", sh)
        name = strtab[name_idx:].split(b"\0", 1)[0].decode("ascii", "ignore")
        if name == ".symtab":
            symtab_off = sh_offset
            symtab_size = sh_size
            str_sh = data[e_shoff + sh_link * e_shentsize : e_shoff + (sh_link + 1) * e_shentsize]
            symstr_off = struct.unpack_from("<I", str_sh, 16)[0]

    if not symtab_off or not symstr_off:
        return {}

    symstr = data[symstr_off:]
    symbols: dict[str, int] = {}
    for off in range(symtab_off, symtab_off + symtab_size, 16):
        st_name, st_value, st_size, st_info, st_other, st_shndx = struct.unpack_from("<IIIBBH", data, off)
        # st_shndx == 0 means SHN_UNDEF (undefined symbol). We only care about defined symbols in the binary.
        if st_shndx != 0:
            sym_name = symstr[st_name:].split(b"\0", 1)[0].decode("ascii", "ignore")
            if sym_name:
                symbols[sym_name] = st_value

    return symbols


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    default_elf = repo_root / "build" / "Release" / "StepperDriverEmulator.elf"
    elf_path = Path(sys.argv[1]) if len(sys.argv) > 1 else default_elf

    if not elf_path.exists():
        print(f"[ERROR] ELF binary not found at {elf_path}", file=sys.stderr)
        return 1

    try:
        symbols = parse_elf_symbols(elf_path)
    except Exception as e:
        print(f"[ERROR] Failed to parse symbols from {elf_path}: {e}", file=sys.stderr)
        return 1

    violations: list[tuple[str, str, int]] = []
    for sym_name, description in DISALLOWED_SYMBOLS.items():
        if sym_name in symbols:
            addr = symbols[sym_name]
            violations.append((sym_name, description, addr))

    print(f"=== Disallowed Symbols Check: {elf_path.name} ===")
    if violations:
        print(f"\n[FAIL] Found {len(violations)} disallowed division symbol(s) in {elf_path.name}:", file=sys.stderr)
        for sym_name, desc, addr in violations:
            print(f"  - {sym_name} @ 0x{addr:08x}: {desc}", file=sys.stderr)
        print("\nFix: Use 32-bit integer arithmetic, fixed-point math, or multiply by inverted constants.", file=sys.stderr)
        return 1

    print("\n[PASS] No float division or int64 division symbols found in binary.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
