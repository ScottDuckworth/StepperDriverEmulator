# Stepper Driver Emulator — Architecture & Internals

This document details the hardware pinout, internal firmware architecture, fixed-point precision strategy, real-time interrupt cycle budgets, memory footprint, verification pipelines, and release process of the **Stepper Driver Emulator**.

> For end-user documentation, physical dynamics modeling, CLI commands, and firmware flashing instructions, see [README.md](README.md).

---

## Hardware Pinout

Target Microcontroller: **STM32F042G6Ux** (UFQFPN28 package, 32 KB Flash, 6 KB SRAM, 48 MHz Cortex-M0).

| Pin | Function | Peripheral / Mode | Description |
| :--- | :--- | :--- | :--- |
| **PA3** | `ENA` | GPIO Input | Motor controller Enable input (Active High) |
| **PA4** | `DIR` | GPIO Input | Motor controller Direction input (Pull-up) |
| **PA5** | `PUL` / `STEP` | TIM2 ETR / TI1 | Motor controller Step input (Filtered input capture) |
| **PB4** | `EA` | GPIO Output | Encoder Channel A output (High speed) |
| **PB5** | `EB` | GPIO Output | Encoder Channel B output (High speed) |
| **PB0** | `LIM1` | GPIO Output | Simulated Limit Switch 1 output |
| **PB1** | `LIM2` | GPIO Output | Simulated Limit Switch 2 output |
| **PA6** | `LED_G` | TIM16 CH1 (PWM) | Green status LED |
| **PA7** | `LED_R` | TIM17 CH1 (PWM) | Red status LED |
| **PA11**| `USB_DM`| USB FS | USB D- line |
| **PA12**| `USB_DP`| USB FS | USB D+ line |

---

## Real-Time Performance & Timing Specifications

The firmware operates on an **STM32F042G6Ux** (ARM Cortex-M0 core) running at 48 MHz (20.833 ns per clock cycle) with 1 Flash wait state (`FLASH_LATENCY_1`).

### Real-Time Streaming Architecture

* **Direct DMA-to-GPIO Quadrature Generation:**
  * Uses **TIM3** as a dynamic pacing heartbeat triggering **DMA1 Channel 3** to push atomic bitmasks directly into `GPIOB->BSRR`.
  * Eliminates phase flip errors, toggle-mode polarity memory issues, and glitch transitions across sudden direction changes.
* **Dual-Buffer Circular DMA Pacing:** The quadrature generator streams phase bitmasks via a 32-sample circular DMA buffer split into two 16-sample halves (`CHUNK_SIZE = 16`). When one half-buffer completes transmission, an interrupt triggers calculation and synthesis of the next chunk.
* **TIM2 Input Capture Bypass:** High-frequency step inputs (> 1 kHz) captured on PA5 via **TIM2** (running at 48 MHz) and **DMA1 Channel 5** are paced through dynamic timer chunk synthesis, avoiding per-pulse CPU interrupt overhead.
* **Pure Integer Motion Core:** All position tracking, feedforward synthesis, soft-knee filtering, and torque calculations execute strictly using 32-bit and 64-bit fixed-point math ($Q12$ and $Q16$). No IEEE-754 floating-point emulation routines (`__aeabi_f*`, `__aeabi_d*`) or 64-bit integer divisions are executed during real-time streaming.

### Fixed-Point Data Types & Precision Strategy

To maintain deterministic execution within the real-time ISR without floating-point hardware or 64-bit software emulation, the motion engine employs discrete, strongly typed fixed-point representations (`q12_t` and `q16_t` in `mathutil.h`):

| Type | Representation | Why Used |
| :--- | :--- | :--- |
| **`q12_t`** | General rates & gains (`kp`, `kff`, `kfree`, `counts_per_step`, `step_blank_us`) | Prevents 32-bit overflow when multiplied by large velocities (up to 200,000 counts/s) on Cortex-M0. |
| **`q16_t`** | Inverses (`inv_counts_per_step`, `inv_torque_span_v`) | Provides 16× higher resolution to prevent small fractional reciprocals from quantizing to 0 or 1; safe from 32-bit overflow because the operands are strictly bounded. |

#### Design Rationale

* **32-bit Dynamic Range on Cortex-M0 (`q12_t`):**
  The ARM Cortex-M0 core features a single-cycle 32-bit hardware multiplier (`muls`) but lacks a 64-bit hardware multiplier (`smull`) and hardware divider (`sdiv`). Operating at 50 kHz step pulses with 4 counts/step yields a velocity of $V_{\mathrm{in}} = 200,000\text{ counts/s}$. Scaling this velocity by a $Q12$ gain produces an intermediate product of $200,000 \times 4096 = 819,200,000$, which safely fits within signed 32-bit limits ($< 2.14 \times 10^9$). Using $Q16$ generally would produce $200,000 \times 65,536 = 1.31 \times 10^{10}$, causing 32-bit integer overflow and requiring slow 64-bit software emulation (`__aeabi_lmul`). Thus, `q12_t` is the optimal system default.

* **Quantization Prevention for Reciprocals (`q16_t`):**
  Parameters representing mathematical inverses (such as `inv_torque_span_v` $= 1 / (V_{\mathrm{max}} - V_{\mathrm{knee}})$ and `inv_counts_per_step` $= 1 / \text{counts per step}$) evaluate to small fractions ($\ll {1.0}$). For example, a torque derating span of 8,000 counts/s has a reciprocal of 0.000125. In $Q12$ (where $\text{1 LSB} \approx {0.000244}$), this fraction truncates to 0 or rounds to 1 (50% to 100% quantization error). In $Q16$ (where $\text{1 LSB} \approx {0.0000153}$), the value is represented accurately as 8. Because these inverses are exclusively multiplied by bounded, small quantities (such as quadratic error $\le (\text{counts per step})^{2}$ or speed offsets $\le \text{span}$), their intermediate products never exceed 32 bits, allowing $Q16$ precision to be used safely without overflow risk.

### Interrupt Execution Budget & Operating Regimes

Interrupt latency and execution cycles were characterized via static disassembly and Control Flow Graph (CFG) analysis using `scripts/estimate_isr_cycles.py` on the ARM Cortex-M0 Release binary. The firmware architecture divides real-time execution into two mutually exclusive regimes:

1. **Active High-Speed Streaming Pipeline:** Runs during continuous rotor motion ($\Delta \gt 0$). Executes 64-bit position updates, velocity feedforward, soft-knee quadratic error attenuation, uniform torque LUT interpolation, physical stop boundary clamping, **full rotor slip modeling** (or normal tracking), quadrature Gray-code synthesis, and timer pacing calculation. Standstill idle checks (`CheckMotionIdle` and `Motion_ShouldStop`) are skipped.
2. **Standstill & Stop-Hit Idle Transition Path:** Runs when emitted chunk deltas drop to zero ($\Delta = 0$), such as when hitting a physical travel stop or when input step pulses cease. Pacing defaults to fallback pacing ($4{,}000\text{ counts/s}$), stretching the chunk interval from $\le 80\ \mu\mathrm{s}$ out to **$4{,}000\ \mu\mathrm{s}$ (4.0 ms)**. Once the step timeout elapses, the timer and DMA are shut down entirely ($0.0\%\text{ CPU load}$).

#### Active High-Speed Streaming Pipeline Breakdown

* **Nominal Steady-State ISR Execution (with Slip Modeling):** **2,231 cycles** ($46.48\ \mu\mathrm{s}$) in silicon (including Flash wait states and hardware NVIC context stacking).
* **Normal Driving Execution ($T_{\text{net}} \ge 0$, no slip):** **2,160 cycles** ($45.00\ \mu\mathrm{s}$) in silicon.
* **Available Budget per Chunk at 50 kHz Step Rate:** $80.00\ \mu\mathrm{s}$ ($16\text{ counts} / 200\text{ kHz counts/s}$ at 4 counts/step).
* **Active CPU Utilization at 50 kHz:** $\frac{46.48\ \mu\mathrm{s}}{80.00\ \mu\mathrm{s}} = \mathbf{58.1\%}$ (**41.9% CPU headroom**).

| Component / Routine | 0-WS Cycles | Silicon Cycles (1-WS) | Duration (@ 48 MHz) | % of ISR | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `Motion_PlanStep` | 884 | 1,105 | 23.02 µs | 49.5% | Feedforward, error compensation, LUT, physical stops & **full slip modeling** |
| `FillQuadChunk` | 254 | 318 | 6.62 µs | 14.3% | DMA chunk buffer dispatch, quadrature generation, and pacing |
| `__udivsi3` | 152 | 190 | 3.96 µs | 8.5% | 32-bit hardware-assisted unsigned division helper (pacing & input rate) |
| `Quadrature_GenerateChunk` | 148 | 185 | 3.85 µs | 8.3% | Gray-code quadrature transition bitmask synthesis (16 samples) |
| `UpdatePositionCounters` | 129 | 161 | 3.35 µs | 7.2% | 64-bit commanded step and encoder position accumulation |
| `Position_FilterStepWithBlanking` | 99 | 124 | 2.58 µs | 5.6% | Hardware step capture blanking filter |
| `Motion_CalcMotorTorque` | 91 | 114 | 2.38 µs | 5.1% | Uniform lookup table linear interpolation across torque knots |
| `Quadrature_CalcTimerPacing` | 70 | 88 | 1.83 µs | 3.9% | TIM3 timer reload prescaler and auto-reload configuration |
| DMA ISR entry & exit glue | 112 | 140 | 2.92 µs | 6.3% | `DMA1_Channel2_3_IRQHandler` and `DMA_TransferComplete_Handler` |
| Math helpers & status checks | 133 | 166 | 3.46 µs | 7.4% | `__aeabi_idivmod`, `Position_ConvertStepDeltaToCounts`, timeout & enable checks |
| Hardware Context Stacking (NVIC) | 31 | 39 | 0.81 µs | 1.7% | ARMv6-M hardware exception entry/exit overhead |
| Standstill Checks (`CheckMotionIdle`, `Motion_ShouldStop`) | 0 | 0 | 0.00 µs | 0.0% | **Bypassed during active streaming** (`half_0_delta != 0`) |
| **Total Active Streaming ISR** | **1,754** | **2,231** | **46.48 µs** | **100.0%** | **Full real-time chunk synthesis pipeline** |

> **Note on Slip Modeling:** The 1,105 cycles measured for `Motion_PlanStep` explicitly evaluates the **torque deficit slip branch** (`t_net < 0`). When external load tension exceeds motor torque capacity ($|\tau_{\text{load}}| \gt T_{\text{motor}}$), the planner evaluates inlined `Motion_CalcSlipVelocity`, clamps the slip trajectory against active physical stops (`minstop`/`maxstop`), and tests the accumulated position lag against `stall_threshold`. During normal driving ($T_{\text{net}} \ge 0$), this branch is bypassed, reducing `Motion_PlanStep` execution by 71 cycles down to 1,034 cycles ($21.54\ \mu\mathrm{s}$).

### CPU Utilization Across Step Frequencies

With a default ratio of 4 counts/step (1000 SPR / 4000 CPR) and a chunk size of 16 counts:

| Input Step Rate | Encoder Count Rate | DMA ISR Period | CPU Utilization | Operating Status |
| :--- | :--- | :--- | :--- | :--- |
| 10.00 kHz | 40.00 kHz | 400.00 µs | 11.6% | Nominal load |
| 15.00 kHz | 60.00 kHz | 266.67 µs | 17.4% | Nominal load |
| 20.00 kHz | 80.00 kHz | 200.00 µs | 23.2% | Nominal load |
| 30.00 kHz | 120.00 kHz | 133.33 µs | 34.9% | Nominal load |
| 40.00 kHz | 160.00 kHz | 100.00 µs | 46.5% | Nominal load |
| 50.00 kHz | 200.00 kHz | 80.00 µs | 58.1% | Nominal load |
| 60.00 kHz | 240.00 kHz | 66.67 µs | 69.7% | High load |
| **75.00 kHz** | **300.00 kHz** | **53.33 µs** | **79.7%** | **Maximum supported streaming rate (pacing clamp, > 20% headroom)** |
| **86.06 kHz** | **344.24 kHz** | **46.48 µs** | **100.0%** | **Theoretical active streaming saturation ceiling** |

#### Firmware Pacing Limit & High-Speed Division Bypass

In `Quadrature_CalcTimerPacing` (`Core/Src/quadrature.c`), timer reload ticks are clamped to a minimum of 160 (`ticks >= 160` at 48 MHz), enforcing a maximum output count rate of **300 kHz** (75 kHz input step pulse rate at 4 counts/step).
* **Division Short-Circuit at 300 kHz:** At or above 300 kHz, `Quadrature_CalcTimerPacing` short-circuits the 32-bit integer division (`__udivsi3`), directly assigning minimum reload ticks. This saves 190 cycles ($3.96\ \mu\mathrm{s}$), bringing total ISR latency down to **$42.52\ \mu\mathrm{s}$** and ensuring CPU load remains safely bounded at **79.7%** (leaving $> 20\%$ CPU headroom).
* **DMA Underrun Prevention:** With double-buffered DMA transmitting 16 counts per chunk, keeping active chunk computation ($42.52\text{--}46.48\ \mu\mathrm{s}$) strictly below the chunk period ($53.33\text{--}80.00\ \mu\mathrm{s}$) guarantees the DMA controller never exhausts the buffer prematurely or transmits corrupt states.
* **Background Task Responsiveness:** Operating with $> 20\%$ to $> 41\%$ CPU headroom during sustained streaming ensures the main loop and USB interrupts have ample time to service CDC commands (`pvt`, `pos`, `r`, `save`), periodic position reports (`odr`), and physical stop status updates without dropping packets.

#### Standstill & Stop-Hit Idle Transition Path

When the rotor reaches a physical stop or input steps cease, the operating regime changes:
* **Stop Hit:** `Motion_PlanStep` detects `blocked_by_stop`, clamps emitted counts and velocity to 0, and returns early ($\approx 25\text{ cycles}$). Emitted chunk delta is 0.
* **Pacing Interval Relaxes:** With target velocity 0, `FillQuadChunk` configures fallback pacing ($4{,}000\text{ counts/s}$), expanding the chunk period to **$4{,}000\ \mu\mathrm{s}$**.
* **Standstill Execution:** When both half-buffer deltas are 0, `CheckMotionIdle` and `Motion_ShouldStop` execute ($816\text{ cycles} / 17.0\ \mu\mathrm{s}$). Even under the composite worst-case duration ($58.58\ \mu\mathrm{s}$), CPU utilization across the $4{,}000\ \mu\mathrm{s}$ period is only **$1.46\%$** (and $< 0.4\%$ when blocked at a stop).
* **Timer Shutdown:** Once input pulses cease and the step timeout expires, `CheckMotionIdle` disables TIM3 and DMA entirely, dropping CPU utilization to **$0.0\%$**.

### Firmware Memory Utilization

Memory footprint of the Release build (`build/Release/StepperDriverEmulator.elf`):

| Memory Region | Used Bytes | Total Bytes | Utilization | Free Space |
| :--- | :--- | :--- | :--- | :--- |
| **Flash** (`.text` + `.rodata` + `.data`) | **30,820 B** | 31,744 B | **97.09%** | 924 B free |
| **RAM** (`.data` + `.bss` + stack) | **5,976 B** | 6,144 B | **97.27%** | 168 B free |

* **Flash Savings:** Complete elimination of soft-float runtime helpers (`__aeabi_fmul`, `__aeabi_fadd`, `__aeabi_fsub`, `__aeabi_fdiv`, `__aeabi_f2iz`, `__aeabi_i2f`, etc.) reclaimed **3,972 bytes** of Flash memory.
* **Deterministic Timing:** Disallowance of software floating point and 64-bit integer division eliminates variable, data-dependent software emulation loops from the motion control path.

---

## Unit Tests

Unit tests are written using the [Unity](https://github.com/ThrowTheSwitch/Unity) test framework and execute natively on the host PC using CMake and CTest. Unity is fetched automatically via CMake's `FetchContent`.

### Running Unit Tests

```powershell
# 1. Configure the host test preset
cmake --preset host-test

# 2. Build the test suite
cmake --build --preset host-test

# 3. Run all tests via CTest
ctest --preset host-test

# Or run the test executable directly for verbose breakdown:
./build/host-test/tests/Debug/test_position.exe
```

---

## Presubmit Checks & Git Hooks

The repository includes automated presubmit checks for local development and continuous integration (CI):

* **`pre-commit`**: Executes a complete multi-stage validation pipeline:
  1. Validates `README.md` and `ARCHITECTURE.md` (GitHub LaTeX compatibility, balanced math delimiters, and CLI command synchronization).
  2. Builds and executes host unit tests (`ctest --preset host-test`).
  3. Builds the ARM Cortex-M0 Release firmware (`cmake --build --preset Release`).
  4. Scans ELF symbols to prevent software floating-point or 64-bit integer division routines (`scripts/check_disallowed_symbols.py`).
  5. Validates Flash and SRAM memory consumption against STM32F042 hardware limits (`scripts/check_firmware_size.py`).
* **`pre-push`**: Compiles the ARM Cortex-M0 Release firmware and validates flash and RAM sizing constraints against STM32F042 limits.
* **GitHub Actions CI**: Executes both test and build suites on every push and pull request.

To activate the repository's git hooks locally, configure your git path:

```bash
git config core.hooksPath .githooks
```

### Static Analysis & Verification Scripts

The following helper scripts under `scripts/` can be executed manually:

```bash
# Validate documentation formatting and CLI command table sync
python scripts/validate_readme.py

# Verify zero disallowed software float / 64-bit division symbols in ELF binary
python scripts/check_disallowed_symbols.py

# Check firmware memory usage against STM32F042 flash limits
python scripts/check_firmware_size.py build/Release/StepperDriverEmulator.elf

# Estimate interrupt execution cycle counts, silicon wait states, and CPU utilization
python scripts/estimate_isr_cycles.py --chunk-size 16
```

---

## Release Process & Versioning

Releases follow [Semantic Versioning](https://semver.org) (`vX.Y.Z`). Firmware embeds the exact version string queried via `git describe --tags --always --dirty` at compile time and reports it via the `version` CLI command.

### Creating a Release (`scripts/release.py`)

A release helper script validates all presubmit checks, tags the commit, and pushes to GitHub:

```powershell
# Bump semantic version component (patch, minor, or major):
python scripts/release.py --bump patch

# Or specify explicit version tag:
python scripts/release.py v1.0.0

# Dry-run mode (runs all checks without tagging or pushing):
python scripts/release.py --bump patch --dry-run
```

### Automated GitHub Releases

Pushing a `v*` tag triggers the automated `.github/workflows/release.yml` workflow, which:
1. Validates documentation, runs host unit tests, and verifies memory sizing and disallowed symbols.
2. Compiles ARM Cortex-M0 Release firmware (`.bin`, `.hex`, `.elf`).
3. Cross-compiles standalone Go flasher binaries for Windows (`x86_64`), Linux (`x86_64`), and macOS (`x86_64` / `arm64`).
4. Generates a GitHub Release with auto-generated release notes and attaches all binary assets.
