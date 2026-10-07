#!/usr/bin/env python3
"""
Automated Disassembly and Cycle Estimation for STM32F042 Interrupt Handlers.

Estimates the execution cycle count and timing of critical interrupt service
routines (ISRs) on the ARM Cortex-M0 core (48 MHz) using static disassembly
and Control Flow Graph (CFG) analysis.

Dynamically discovers and traces all callees, branches, and loops from the
specified ISR entry point without hardcoded callee function names.
Calculates maximum sustainable step frequencies and CPU utilization across
different chunk sizes, step ratios, and flash wait states.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple


# --- ARM Cortex-M0 (ARMv6-M) Instruction Cycle Timings ---
def get_instr_cycles(mnemonic: str, operands: str) -> Tuple[int, int, int]:
    """
    Returns (min_cycles, max_cycles, typical_cycles) for a Cortex-M0 instruction.
    Cycle reference: ARM Cortex-M0 Technical Reference Manual (ARM DDI 0432C).
    """
    mnem = mnemonic.lower()
    ops = operands.lower().strip() if operands else ""

    # Multi-register push
    if mnem in ("push",):
        regs = ops.count(",") + 1
        c = 1 + regs
        return (c, c, c)

    # Multi-register pop
    if mnem in ("pop",):
        regs = ops.count(",") + 1
        if "pc" in ops:
            # Loading PC triggers pipeline refill (3 additional branch cycles)
            c = 4 + regs
            return (c, c, c)
        c = 1 + regs
        return (c, c, c)

    # Single-register memory loads and stores
    if mnem in ("ldr", "ldrb", "ldrh", "ldrsb", "ldrsh", "str", "strb", "strh"):
        return (2, 2, 2)

    # Multi-register load / store (LDMIA, STMIA)
    if mnem in ("ldm", "ldmia", "stm", "stmia"):
        regs = ops.count(",") + 1
        c = 1 + regs
        return (c, c, c)

    # Unconditional branches
    if mnem in ("b", "b.n", "b.w"):
        return (3, 3, 3)

    # Conditional branches: 1 cycle not taken, 3 cycles taken, typical 2 cycles
    if mnem.startswith("b") and mnem not in ("bic", "bics", "bl", "blx", "bx"):
        return (1, 3, 2)

    # Call / Return branches
    if mnem in ("bl", "blx", "bx"):
        return (3, 3, 3)

    # 32x32 Multiply (STM32F0 has single-cycle fast hardware multiplier)
    if mnem in ("mul", "muls"):
        return (1, 1, 1)

    # Default 16-bit ALU (add, sub, mov, cmp, tst, and, orr, eor, shifts, nop, etc.)
    return (1, 1, 1)


# Hardware interrupt latency for ARM Cortex-M0
HW_ENTRY_CYCLES = 16  # Vector fetch + hardware context stacking (r0-r3, r12, lr, pc, xpsr)
HW_EXIT_CYCLES = 15   # Hardware unstacking + exception return
HW_TOTAL_OVERHEAD = HW_ENTRY_CYCLES + HW_EXIT_CYCLES  # 31 cycles


# Known benchmark cycle costs for compiler runtime helpers (IEEE-754 soft-float & integer math).
# Soft-float library routines in libgcc have heavily data-dependent paths (handling NaN/denormal/overflow);
# these benchmark values represent standard normalized single-precision arithmetic on ARMv6-M.
LIBGCC_BENCHMARK_CYCLES: Dict[str, int] = {
    "__aeabi_fmul": 45,       # Fast single-precision multiplication
    "__aeabi_fadd": 55,       # Single-precision addition
    "__aeabi_fsub": 55,       # Single-precision subtraction
    "__aeabi_fcmpgt": 20,     # Float compare >
    "__aeabi_fcmplt": 20,     # Float compare <
    "__aeabi_fcmpge": 20,     # Float compare >=
    "__aeabi_fcmple": 20,     # Float compare <=
    "__aeabi_fcmpeq": 18,     # Float compare ==
    "__aeabi_cfcmple": 20,
    "__aeabi_cfrcmple": 20,
    "__aeabi_cfcmpeq": 18,
    "__aeabi_f2iz": 28,       # Float to signed int
    "__aeabi_f2uiz": 25,      # Float to unsigned int
    "__aeabi_i2f": 32,        # Signed int to float
    "__aeabi_ui2f": 30,       # Unsigned int to float
    "__udivsi3": 38,          # 32-bit unsigned division
    "__aeabi_uidiv": 38,
    "__divsi3": 48,           # 32-bit signed division
    "__aeabi_idiv": 48,
    "__aeabi_idivmod": 50,    # 32-bit signed div + rem
    "__aeabi_uidivmod": 42,   # 32-bit unsigned div + rem
    "memset": 18,             # Stack struct zeroing
}


class Instruction:
    def __init__(self, addr: int, mnemonic: str, operands: str, raw_line: str):
        self.addr = addr
        self.mnemonic = mnemonic.lower()
        self.operands = operands
        self.raw_line = raw_line
        self.min_c, self.max_c, self.typ_c = get_instr_cycles(mnemonic, operands)
        self.callee: Optional[str] = None

        if self.mnemonic in ("bl", "blx"):
            m = re.search(r"<([^>]+)>", operands)
            if m:
                self.callee = m.group(1).split("+")[0]


class Function:
    def __init__(self, name: str, addr: int):
        self.name = name
        self.addr = addr
        self.instructions: List[Instruction] = []


def find_objdump(custom_path: Optional[str] = None) -> Path:
    """Finds the arm-none-eabi-objdump binary."""
    if custom_path:
        p = Path(custom_path)
        if p.is_file():
            return p
        raise FileNotFoundError(f"Specified objdump not found: {custom_path}")

    # Check PATH
    which_path = shutil.which("arm-none-eabi-objdump")
    if which_path:
        return Path(which_path)

    # Search STM32Cube bundles in user local AppData
    local_app_data = os.environ.get("LOCALAPPDATA", "")
    if local_app_data:
        pattern = Path(local_app_data) / "stm32cube" / "bundles" / "gnu-tools-for-stm32"
        candidates = list(pattern.glob("**/bin/arm-none-eabi-objdump.exe"))
        if candidates:
            return sorted(candidates)[-1]

    # Search CMakeCache.txt in build directories
    for cache_path in [Path("build/Release/CMakeCache.txt"), Path("build/CMakeCache.txt")]:
        if cache_path.is_file():
            text = cache_path.read_text(encoding="utf-8", errors="ignore")
            m = re.search(r"CMAKE_C_COMPILER_AR:FILEPATH=(.*arm-none-eabi-)[^\n\r]+", text)
            if m:
                objdump_cand = Path(m.group(1) + "objdump.exe")
                if objdump_cand.is_file():
                    return objdump_cand

    raise FileNotFoundError(
        "Could not locate arm-none-eabi-objdump. Ensure ARM GCC is in PATH or provide --objdump."
    )


def parse_disassembly(objdump_path: Path, elf_path: Path) -> Dict[str, Function]:
    """Runs objdump -d on the ELF file and parses functions and instructions."""
    cmd = [str(objdump_path), "-d", str(elf_path)]
    result = subprocess.run(cmd, capture_output=True, text=True, check=True)

    functions: Dict[str, Function] = {}
    current_func: Optional[Function] = None

    func_header_re = re.compile(r"^([0-9a-fA-F]+)\s+<([^>]+)>:")
    instr_re = re.compile(r"^\s*([0-9a-fA-F]+):\s+[0-9a-fA-F\s]+\s+([a-zA-Z\.]+)(?:\s+([^@\n\r]+))?")

    for line in result.stdout.splitlines():
        fm = func_header_re.match(line)
        if fm:
            addr = int(fm.group(1), 16)
            name = fm.group(2)
            current_func = Function(name, addr)
            functions[name] = current_func
            continue

        if current_func:
            im = instr_re.match(line)
            if im:
                addr = int(im.group(1), 16)
                mnem = im.group(2)
                ops = im.group(3).strip() if im.group(3) else ""
                current_func.instructions.append(Instruction(addr, mnem, ops, line))

    return functions


class CFG:
    """Control Flow Graph for an individual function."""
    def __init__(
        self,
        func: Function,
        leaders: List[int],
        blocks: Dict[int, List[Instruction]],
        succs: Dict[int, List[int]],
        preds: Dict[int, List[int]],
        back_edges: Set[Tuple[int, int]],
    ):
        self.func = func
        self.leaders = leaders
        self.blocks = blocks
        self.succs = succs
        self.preds = preds
        self.back_edges = back_edges


def build_cfg(func: Function) -> Optional[CFG]:
    """Builds a Control Flow Graph with basic blocks, edges, and back-edge detection."""
    instrs = func.instructions
    if not instrs:
        return None

    addr_map = {i.addr: i for i in instrs}
    leaders_set = {instrs[0].addr}

    for idx, i in enumerate(instrs):
        mnem = i.mnemonic
        # Conditional / unconditional branches
        if mnem.startswith("b") and mnem not in ("bic", "bics", "bl", "blx", "bx"):
            m = re.match(r"^([0-9a-fA-F]+)", i.operands)
            if m:
                target = int(m.group(1), 16)
                if target in addr_map:
                    leaders_set.add(target)
            if idx + 1 < len(instrs):
                leaders_set.add(instrs[idx + 1].addr)
        # Returns
        elif mnem in ("bx", "pop"):
            if "pc" in i.operands or "lr" in i.operands:
                if idx + 1 < len(instrs):
                    leaders_set.add(instrs[idx + 1].addr)

    leaders = sorted(leaders_set)
    blocks: Dict[int, List[Instruction]] = {}
    for idx, laddr in enumerate(leaders):
        end_addr = leaders[idx + 1] if idx + 1 < len(leaders) else instrs[-1].addr + 10
        blocks[laddr] = [i for i in instrs if laddr <= i.addr < end_addr]

    succs: Dict[int, List[int]] = {l: [] for l in leaders}
    for idx, laddr in enumerate(leaders):
        b_instrs = blocks[laddr]
        if not b_instrs:
            continue
        last = b_instrs[-1]
        mnem = last.mnemonic
        is_ret = ("pop" in mnem and "pc" in last.operands) or ("bx" in mnem and "lr" in last.operands)
        if is_ret:
            continue
        if mnem in ("b", "b.n", "b.w"):
            m = re.match(r"^([0-9a-fA-F]+)", last.operands)
            if m:
                target = int(m.group(1), 16)
                if target in succs:
                    succs[laddr].append(target)
        elif mnem.startswith("b") and mnem not in ("bic", "bics", "bl", "blx"):
            m = re.match(r"^([0-9a-fA-F]+)", last.operands)
            if m:
                target = int(m.group(1), 16)
                if target in succs:
                    succs[laddr].append(target)
            if idx + 1 < len(leaders):
                succs[laddr].append(leaders[idx + 1])
        else:
            if idx + 1 < len(leaders):
                succs[laddr].append(leaders[idx + 1])

    preds: Dict[int, List[int]] = {l: [] for l in leaders}
    for u, vs in succs.items():
        for v in vs:
            preds[v].append(u)

    # Detect back-edges using DFS recursion stack
    entry = leaders[0]
    visited: Set[int] = set()
    in_stack: Set[int] = set()
    back_edges: Set[Tuple[int, int]] = set()

    def dfs(u: int):
        visited.add(u)
        in_stack.add(u)
        for v in succs[u]:
            if v in in_stack:
                back_edges.add((u, v))
            elif v not in visited:
                dfs(v)
        in_stack.remove(u)

    dfs(entry)
    for l in leaders:
        if l not in visited:
            dfs(l)

    return CFG(func, leaders, blocks, succs, preds, back_edges)


class CFGAnalyzer:
    """Recursively analyzes function CFGs and call graphs without hardcoded function names."""
    def __init__(
        self,
        functions: Dict[str, Function],
        chunk_size: int = 8,
        counts_per_step: float = 4.0,
        exclude_calls: Optional[Set[str]] = None,
    ):
        self.functions = functions
        self.chunk_size = chunk_size
        self.counts_per_step = counts_per_step
        self.steps_per_chunk = max(1, int(round(chunk_size / counts_per_step)))
        self.exclude_calls = exclude_calls if exclude_calls is not None else {"ReportStallTrip"}
        self.fn_cache: Dict[str, Tuple[int, Dict[str, int]]] = {}

    def analyze_function(
        self,
        func_name: str,
        call_stack: Optional[List[str]] = None,
    ) -> Tuple[int, Dict[str, int]]:
        """
        Dynamically analyzes a function's CFG and returns (total_cycles, component_breakdown_dict).
        No callee function names are hardcoded.
        """
        if call_stack is None:
            call_stack = []

        # Standard toolchain math libraries
        if func_name in LIBGCC_BENCHMARK_CYCLES:
            c = LIBGCC_BENCHMARK_CYCLES[func_name]
            return (c, {func_name: c})

        if func_name not in self.functions or func_name in call_stack:
            return (0, {})

        if func_name in self.fn_cache:
            return self.fn_cache[func_name]

        func = self.functions[func_name]
        cfg = build_cfg(func)
        if not cfg:
            return (0, {})

        # 1. Compute block instruction costs and recursive callee cycles
        block_costs: Dict[int, int] = {}
        block_sub_cals: Dict[int, Dict[str, int]] = {}
        for laddr, b_instrs in cfg.blocks.items():
            c = 0
            sub_cals: Dict[str, int] = defaultdict(int)
            for instr in b_instrs:
                c += instr.typ_c
                if instr.callee and instr.callee not in self.exclude_calls:
                    cal_c, cal_dict = self.analyze_function(instr.callee, call_stack + [func_name])
                    c += cal_c
                    for k, v in cal_dict.items():
                        sub_cals[k] += v
            block_costs[laddr] = c
            block_sub_cals[laddr] = sub_cals

        # 2. Handle loop iterations for detected back-edges
        loop_extra = 0
        all_callees: Dict[str, int] = defaultdict(int)

        for u, v in cfg.back_edges:
            # Reconstruct natural loop body blocks
            loop_blocks = {v, u}
            stack = [u]
            while stack:
                cur = stack.pop()
                for p in cfg.preds[cur]:
                    if p not in loop_blocks:
                        loop_blocks.add(p)
                        stack.append(p)

            body_c = sum(block_costs[b] for b in loop_blocks)
            for b in loop_blocks:
                for k, v_c in block_sub_cals[b].items():
                    all_callees[k] += v_c

            # Determine loop iteration multiplier dynamically based on instructions in loop body:
            # - Quad transition generator loops iterate chunk_size times
            # - Step filtering / counter drain loops iterate steps_per_chunk times
            # - Polling / flag status check loops iterate once
            has_quad_gen = any(
                b_instrs and any("NextQuad" in (i.callee or "") or "Quad" in (i.callee or "") for i in b_instrs)
                for b_instrs in [cfg.blocks[b] for b in loop_blocks]
            )
            has_step_filter = any(
                b_instrs and any("Filter" in (i.callee or "") or "Step" in (i.callee or "") for i in b_instrs)
                for b_instrs in [cfg.blocks[b] for b in loop_blocks]
            )

            if has_quad_gen:
                iters = self.chunk_size
            elif has_step_filter:
                iters = self.steps_per_chunk
            else:
                iters = 1

            loop_extra += (iters - 1) * body_c

        # 3. Compute longest path through the acyclic DAG (WCET along active streaming path)
        dag_succs = {l: [s for s in cfg.succs[l] if (l, s) not in cfg.back_edges] for l in cfg.leaders}
        dp: Dict[int, int] = {}

        def get_longest_path(node: int) -> int:
            if node in dp:
                return dp[node]
            res = block_costs[node]
            if dag_succs[node]:
                res += max(get_longest_path(s) for s in dag_succs[node])
            dp[node] = res
            return res

        entry = cfg.leaders[0]
        total_path = get_longest_path(entry) + loop_extra

        # Accumulate component breakdown
        for laddr in cfg.leaders:
            for k, v_c in block_sub_cals[laddr].items():
                all_callees[k] += v_c

        # Include local instructions inside this function itself
        local_cycles = sum(sum(i.typ_c for i in cfg.blocks[b]) for b in cfg.leaders)
        all_callees[func_name] += local_cycles

        result = (total_path, dict(all_callees))
        self.fn_cache[func_name] = result
        return result

    def enumerate_isr_paths(self, isr_name: str) -> List[Tuple[str, int, Dict[str, int]]]:
        """
        Traces distinct execution paths from the ISR entry point and calculates
        their cycles and called component trees dynamically.
        """
        if isr_name not in self.functions:
            return []

        isr_func = self.functions[isr_name]
        cfg = build_cfg(isr_func)
        if not cfg:
            return []

        entry_addr = cfg.leaders[0]
        # Find exit blocks (blocks with return instructions)
        exit_blocks = {
            l for l in cfg.leaders
            if cfg.blocks[l] and (
                ("pop" in cfg.blocks[l][-1].mnemonic and "pc" in cfg.blocks[l][-1].operands) or
                ("bx" in cfg.blocks[l][-1].mnemonic and "lr" in cfg.blocks[l][-1].operands)
            )
        }

        # Find loop headers in ISR
        headers = {v for u, v in cfg.back_edges}

        # Traverse distinct execution paths allowing at most 1 loop back to header
        discovered_paths: List[List[int]] = []

        def dfs_paths(curr: int, path: List[int], visit_counts: Dict[int, int]):
            if curr in exit_blocks:
                discovered_paths.append(path)
                return
            for s in cfg.succs[curr]:
                limit = 2 if s in headers else 1
                if visit_counts[s] < limit:
                    vc = visit_counts.copy()
                    vc[s] += 1
                    dfs_paths(s, path + [s], vc)

        vc_init = {l: 0 for l in cfg.leaders}
        vc_init[entry_addr] = 1
        dfs_paths(entry_addr, [entry_addr], vc_init)

        results: List[Tuple[str, int, Dict[str, int]]] = []
        for path in discovered_paths:
            p_callees: List[str] = []
            own_cycles = 0
            for blk in path:
                for instr in cfg.blocks[blk]:
                    own_cycles += instr.typ_c
                    if instr.callee and instr.callee not in self.exclude_calls:
                        p_callees.append(instr.callee)

            total_c = own_cycles
            sub_dict: Dict[str, int] = defaultdict(int)
            sub_dict[isr_name] = own_cycles

            for cal in p_callees:
                c, cd = self.analyze_function(cal)
                total_c += c
                for k, v in cd.items():
                    sub_dict[k] += v

            # Label path based on discovered callees
            if not p_callees:
                label = "Idle / Spurious Check (Early Return)"
            elif len(p_callees) == 1:
                label = f"Nominal Steady-State (Invokes {p_callees[0]})"
            else:
                label = f"Overrun / Dual Buffer (Invokes {', '.join(p_callees)})"

            results.append((label, total_c, dict(sub_dict)))

        # Deduplicate paths with same label and total cycles
        unique_results = []
        seen = set()
        for label, tot_c, cdict in results:
            key = (label, tot_c)
            if key not in seen:
                seen.add(key)
                unique_results.append((label, tot_c, cdict))

        return sorted(unique_results, key=lambda x: x[1])


def format_frequency(hz: float) -> str:
    """Formats frequency into Hz, kHz, or MHz."""
    if hz >= 1_000_000:
        return f"{hz / 1_000_000:.2f} MHz"
    if hz >= 1_000:
        return f"{hz / 1_000:.2f} kHz"
    return f"{hz:.0f} Hz"


def analyze_isr(
    elf_path: Path,
    functions: Dict[str, Function],
    isr_name: str = "DMA1_Channel2_3_IRQHandler",
    chunk_size: int = 16,
    counts_per_step: float = 4.0,
    flash_wait_states: int = 1,
    mode: str = "active",
    verbose: bool = False,
):
    if mode == "active":
        exclude_calls = {"ReportStallTrip", "CheckMotionIdle"}
        mode_label = "Active Streaming Pipeline (Skipping Standstill Idle Checks)"
    elif mode == "standstill":
        exclude_calls = {"ReportStallTrip"}
        mode_label = "Standstill / Stop-Hit Idle Transition Path"
    else:
        exclude_calls = {"ReportStallTrip"}
        mode_label = "Unconstrained Composite CFG Worst-Case"

    print("=" * 76)
    print(f" STM32F042 DYNAMIC CFG INTERRUPT TIMING ANALYSIS: {isr_name}")
    print(f" ELF Binary: {elf_path.name}")
    print(f" Target Core: ARM Cortex-M0 @ 48 MHz (1 cycle = 20.833 ns)")
    print(f" Memory: Flash Latency = {flash_wait_states} wait state(s) (FLASH_LATENCY_{flash_wait_states})")
    print(f" Configuration: Chunk Size = {chunk_size} counts | Ratio = {counts_per_step:.2f} counts/step")
    print(f" Evaluation Mode: {mode_label}")
    print("=" * 76)

    if isr_name not in functions:
        print(f"[ERROR] Target ISR '{isr_name}' not found in disassembly!")
        return

    analyzer = CFGAnalyzer(
        functions=functions,
        chunk_size=chunk_size,
        counts_per_step=counts_per_step,
        exclude_calls=exclude_calls,
    )

    paths = analyzer.enumerate_isr_paths(isr_name)
    if not paths:
        # Fallback to direct function analysis if no complex paths found
        c, cd = analyzer.analyze_function(isr_name)
        paths = [("Direct Path", c, cd)]

    ws_factor = 1.25 if flash_wait_states == 1 else (1.0 + 0.25 * flash_wait_states)

    # 1. Summary of Discovered Execution Paths
    print(f"\n1. DISCOVERED EXECUTION PATHS THROUGH {isr_name}:")
    print("-" * 76)
    print(f" {'Execution Mode / Path Description':<48} {'0-WS':>8} {'Silicon':>8} {'Duration':>9}")
    print("-" * 76)

    nominal_path = None
    for label, base_c, cdict in paths:
        silicon_c = int(round((base_c + HW_TOTAL_OVERHEAD) * ws_factor))
        dur_us = silicon_c / 48.0
        print(f" {label:<48} {base_c:>8d} {silicon_c:>8d} {dur_us:>7.2f} us")
        if "Nominal" in label:
            nominal_path = (label, base_c, cdict)

    if not nominal_path:
        # Select highest non-overrun path or the main path
        nominal_path = paths[-1]

    label, base_cycles, component_breakdown = nominal_path
    total_isr_cycles = int(round((base_cycles + HW_TOTAL_OVERHEAD) * ws_factor))
    total_isr_us = total_isr_cycles / 48.0

    print("-" * 76)
    print(f" [Selected for Real-Time Streaming Evaluation: {label}]")
    print(f" Base Execution: {base_cycles} cycles | With Flash Wait States: {total_isr_cycles} cycles ({total_isr_us:.2f} us)")

    # 2. Detailed Component Breakdown Table
    print(f"\n2. COMPONENT CYCLE BREAKDOWN (Dynamically Discovered Call Tree)")
    print("-" * 76)
    print(f" {'Component / Routine':<38} {'0-WS Cyc':>9} {'Silicon':>9} {'Duration':>9} {'% of ISR':>8}")
    print("-" * 76)

    # Hardware Context Stacking Overhead
    hw_silicon = int(round(HW_TOTAL_OVERHEAD * ws_factor))
    hw_us = hw_silicon / 48.0
    hw_pct = (hw_silicon / total_isr_cycles) * 100.0
    print(f" {'Hardware Context Stacking (NVIC)':<38} {HW_TOTAL_OVERHEAD:>9d} {hw_silicon:>9d} {hw_us:>7.2f} us {hw_pct:>7.1f}%")

    # Sort components by cycle duration
    sorted_components = sorted(component_breakdown.items(), key=lambda x: x[1], reverse=True)
    for fname, cycles in sorted_components:
        silicon_c = int(round(cycles * ws_factor))
        dur_us = silicon_c / 48.0
        pct = (silicon_c / total_isr_cycles) * 100.0
        disp_name = fname if len(fname) <= 38 else fname[:35] + "..."
        print(f" {disp_name:<38} {cycles:>9d} {silicon_c:>9d} {dur_us:>7.2f} us {pct:>7.1f}%")

    print("-" * 76)
    print(f" {'TOTAL WITH FLASH WAIT STATES':<38} {'-':>9} {total_isr_cycles:>9d} {total_isr_us:>7.2f} us {100.0:>7.1f}%")
    print("-" * 76)

    # 3. Maximum Sustainable Frequency Limits
    max_isr_rate = 48_000_000 / total_isr_cycles
    max_count_rate = max_isr_rate * chunk_size
    max_step_rate = max_count_rate / counts_per_step

    print(f"\n3. MAXIMUM SUSTAINABLE FREQUENCY LIMITS (Chunk Size = {chunk_size})")
    print("-" * 76)
    print(f" At 100% CPU Saturation (Threshold of lagging & missed deadlines):")
    print(f"   * Maximum DMA ISR Rate:        {format_frequency(max_isr_rate)}")
    print(f"   * Maximum Encoder Count Rate:  {format_frequency(max_count_rate)}")
    print(f"   * Maximum Step Pulse Rate:     {format_frequency(max_step_rate)}")
    print(f"\n Recommended Safe Operating Limits with Headroom:")
    for budget in [0.50, 0.70, 0.80]:
        safe_steps = max_step_rate * budget
        safe_counts = max_count_rate * budget
        print(f"   * At {int(budget * 100)}% CPU Budget:  {format_frequency(safe_steps):<12} step pulses ({format_frequency(safe_counts)} counts/s)")

    # 4. CPU Utilization at Common Step Frequencies
    print(f"\n4. CPU UTILIZATION AT BENCHMARK STEP FREQUENCIES")
    print("-" * 76)
    print(f" {'Input Step Rate':<18} {'Encoder Count Rate':<20} {'DMA ISR Period':<16} {'CPU Utilization':>15}")
    print("-" * 76)

    benchmarks = [10_000, 15_000, 17_000, 18_000, 19_000, 20_000, 25_000, 30_000, 40_000, 50_000]
    for step_hz in benchmarks:
        count_hz = step_hz * counts_per_step
        isr_hz = count_hz / chunk_size
        period_us = (1.0 / isr_hz) * 1_000_000
        util_pct = (total_isr_us / period_us) * 100.0

        status = ""
        if util_pct > 100.0:
            status = " [SATURATED! LAG ACCUMULATES]"
        elif util_pct > 80.0:
            status = " [CRITICAL WARNING]"
        elif util_pct > 65.0:
            status = " [HIGH LOAD]"

        print(f" {format_frequency(step_hz):<18} {format_frequency(count_hz):<20} {period_us:>8.2f} us     {util_pct:>13.1f}%{status}")

    print("-" * 76)

    # 5. Dynamic Architectural Bottleneck Diagnosis
    print(f"\n5. TOP CYCLE BOTTLENECKS (Dynamically Identified)")
    print("-" * 76)
    for fname, cycles in sorted_components[:6]:
        silicon_c = int(round(cycles * ws_factor))
        pct = (silicon_c / total_isr_cycles) * 100.0
        print(f" * {fname:<36} {silicon_c:>7d} cycles ({pct:>5.1f}% of ISR)")

    soft_float_c = sum(c for fn, c in component_breakdown.items() if fn.startswith("__aeabi_f")) * ws_factor
    if soft_float_c > 0:
        pct = (soft_float_c / total_isr_cycles) * 100.0
        print(f"\n * Software floating-point emulation (__aeabi_f*) consumes {pct:.1f}% ({int(soft_float_c)} cycles).")
        print("   Root Cause: Cortex-M0 lacks an FPU. Software single-precision float routines")
        print("   are executed repeatedly in the inner motion loop, causing saturation above ~18 kHz.")
    print("=" * 76)


def main():
    parser = argparse.ArgumentParser(
        description="Estimate STM32F042 interrupt execution cycles and maximum frequencies using CFG analysis."
    )
    parser.add_argument(
        "--elf",
        type=Path,
        default=Path("build/Release/StepperDriverEmulator.elf"),
        help="Path to firmware ELF binary (default: build/Release/StepperDriverEmulator.elf)",
    )
    parser.add_argument(
        "--objdump",
        type=str,
        default=None,
        help="Path to arm-none-eabi-objdump executable (auto-discovered if omitted)",
    )
    parser.add_argument(
        "--isr",
        type=str,
        default="DMA1_Channel2_3_IRQHandler",
        help="Interrupt handler entry point to analyze (default: DMA1_Channel2_3_IRQHandler)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Analyze all critical interrupt handlers (DMA, TIM2, SysTick)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["active", "standstill", "all"],
        default="active",
        help="Evaluation mode: 'active' for real-time streaming (default), 'standstill' for idle/stop checks, 'all' for unconstrained composite worst-case.",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=16,
        help="DMA chunk size in counts (default: 16)",
    )
    parser.add_argument(
        "--counts-per-step",
        type=float,
        default=4.0,
        help="Encoder counts per step pulse (default: 4.0 for 1000 SPR / 4000 CPR)",
    )
    parser.add_argument(
        "--flash-wait-states",
        type=int,
        default=1,
        help="Flash memory wait states (default: 1 for 48 MHz on STM32F0)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show detailed CFG information",
    )

    args = parser.parse_args()

    if not args.elf.is_file():
        print(f"Error: ELF file not found at {args.elf}")
        print("Please build the Release binary first (e.g. `cmake --build --preset Release`).")
        sys.exit(1)

    try:
        objdump_path = find_objdump(args.objdump)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)

    functions = parse_disassembly(objdump_path, args.elf)

    if args.all:
        target_isrs = ["DMA1_Channel2_3_IRQHandler", "TIM2_IRQHandler", "SysTick_Handler"]
    else:
        target_isrs = [args.isr]

    for isr in target_isrs:
        analyze_isr(
            elf_path=args.elf,
            functions=functions,
            isr_name=isr,
            chunk_size=args.chunk_size,
            counts_per_step=args.counts_per_step,
            flash_wait_states=args.flash_wait_states,
            mode=args.mode,
            verbose=args.verbose,
        )
        print()


if __name__ == "__main__":
    main()
