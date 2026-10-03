#!/usr/bin/env python3
"""
Inspects an STM32 ELF binary and verifies that flash and RAM allocations fit within
the STM32F042 microcontroller physical limits.

Memory constraints for STM32F042G6:
- FLASH: 31 KB (31,744 bytes) for executable code (Pages 0-30; Page 31 is reserved for CONFIG_FLASH).
- RAM:   6 KB (6,144 bytes).
"""

import sys
import struct
from pathlib import Path

FLASH_LIMIT_BYTES = 31744  # 31 KB code flash (32 KB total - 1 KB config page)
RAM_LIMIT_BYTES = 6144    # 6 KB SRAM

def parse_elf_sections(elf_path: Path) -> dict[str, int]:
    data = elf_path.read_bytes()
    if len(data) < 52 or data[:4] != b"\x7fELF":
        raise ValueError(f"Invalid ELF file: {elf_path}")

    # ELF32 header parsing
    e_shoff, = struct.unpack_from("<I", data, 32)
    e_shentsize, e_shnum, e_shstrndx = struct.unpack_from("<HHH", data, 46)

    strtab_off, = struct.unpack_from("<I", data, e_shoff + e_shstrndx * e_shentsize + 16)
    strtab = data[strtab_off:]

    sections: dict[str, int] = {}
    for i in range(e_shnum):
        sh = data[e_shoff + i * e_shentsize : e_shoff + (i + 1) * e_shentsize]
        name_idx, sh_type, sh_flags, sh_addr, sh_offset, sh_size = struct.unpack_from("<IIIIII", sh)
        name = strtab[name_idx:].split(b"\0", 1)[0].decode("ascii", "ignore")
        if sh_flags & 2:  # SHF_ALLOC
            sections[name] = sh_size

    return sections


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    default_elf = repo_root / "build" / "Release" / "StepperDriverEmulator.elf"
    elf_path = Path(sys.argv[1]) if len(sys.argv) > 1 else default_elf

    if not elf_path.exists():
        print(f"[ERROR] ELF binary not found at {elf_path}", file=sys.stderr)
        return 1

    try:
        sections = parse_elf_sections(elf_path)
    except Exception as e:
        print(f"[ERROR] Failed to parse ELF file {elf_path}: {e}", file=sys.stderr)
        return 1

    # Standard GNU/ARM size calculation
    # Flash = sum of read-only code/data loaded in ROM + data initializers
    flash_sections = [".isr_vector", ".text", ".rodata", ".ARM", ".preinit_array", ".init_array", ".fini_array"]
    text_size = sum(sections.get(name, 0) for name in flash_sections)
    data_size = sections.get(".data", 0)
    bss_size = sections.get(".bss", 0) + sections.get("._user_heap_stack", 0)

    flash_used = text_size + data_size
    ram_used = data_size + bss_size

    flash_pct = (flash_used / FLASH_LIMIT_BYTES) * 100.0
    ram_pct = (ram_used / RAM_LIMIT_BYTES) * 100.0
    flash_free = FLASH_LIMIT_BYTES - flash_used
    ram_free = RAM_LIMIT_BYTES - ram_used

    print(f"=== Firmware Memory Report: {elf_path.name} ===")
    print(f"Flash: {flash_used} / {FLASH_LIMIT_BYTES} bytes ({flash_pct:.2f}%) [{flash_free} bytes free]")
    print(f"  .text / .rodata: {text_size} bytes")
    print(f"  .data:           {data_size} bytes")
    print(f"RAM:   {ram_used} / {RAM_LIMIT_BYTES} bytes ({ram_pct:.2f}%) [{ram_free} bytes free]")
    print(f"  .data:           {data_size} bytes")
    print(f"  .bss / stack:    {bss_size} bytes")

    errors = []
    if flash_used > FLASH_LIMIT_BYTES:
        overflow = flash_used - FLASH_LIMIT_BYTES
        errors.append(f"FLASH overflowed by {overflow} bytes ({flash_used} > {FLASH_LIMIT_BYTES})!")

    if ram_used > RAM_LIMIT_BYTES:
        overflow = ram_used - RAM_LIMIT_BYTES
        errors.append(f"RAM overflowed by {overflow} bytes ({ram_used} > {RAM_LIMIT_BYTES})!")

    if errors:
        print("\n[FAIL] Firmware memory limits exceeded:", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        return 1

    print("\n[PASS] Firmware fits within STM32F042 memory constraints.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
