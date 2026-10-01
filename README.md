# Stepper Driver Emulator

A high-performance firmware emulator running on an **STM32F042** microcontroller that simulates a stepper motor with an attached quadrature incremental encoder.

It monitors standard stepper controller signals (**Step/PUL**, **Direction/DIR**, and **Enable/ENA**), tracks motor position, simulates physical motor dynamics (torque curves, external load tension, rotor stall, slip, and fault disengagement), and outputs clean, phase-deterministic quadrature encoder signals (**EA** and **EB**) driven via DMA direct-to-GPIO.

---

## Features

* **Direct DMA-to-GPIO Quadrature Generation:**
  * Uses **TIM3** as a dynamic pacing heartbeat triggering **DMA1 Channel 3** to push atomic bitmasks directly into `GPIOB->BSRR`.
  * Eliminates phase flip errors, toggle-mode polarity memory issues, and glitch transitions across sudden direction changes.
  * Supports quadrature streaming rates up to 100 kHz.
* **Accurate Input Step Tracking:**
  * Captures pulse periods on PA5 via **TIM2** (running at 48 MHz) and streams captured timestamps through **DMA1 Channel 5**.
  * Direction is sampled with interrupt-level precision on PA4.
* **Configurable Gear/Resolution Ratio:**
  * Independent configuration of steps per revolution (`spr`) and encoder counts per revolution (`epr`).
* **Physics & Torque Curve Modeling:**
  * Piecewise linear torque vs. speed curve ($T_0$, $V_{\text{knee}}$, $V_{\text{max}}$, $T_{\text{min}}$).
  * Real-time external load tension/torque input via USB (`t`).
  * Stall and slip dynamics when external load exceeds motor torque capability.
  * Stall trip detection when rotor lag exceeds a threshold (`stall`), disengaging the motor into a fault state.
* **Viscous Freewheeling:**
  * Rotor free-wheels under external tension whenever the motor is de-energized (either during a stall trip or when `ENA` is disabled).
  * Streams genuine quadrature output pulses at terminal velocity $V_{\text{freewheel}} = \tau_{\text{tension}} \cdot K_{\text{free}}$ (clamped to $\pm V_{\text{max}}$).
* **Automatic Re-engagement & Realignment:**
  * Toggling the `ENA` pin from disabled to enabled clears the stall fault and automatically realigns commanded step position to the rotor's current physical resting position.
* **4 Hz Multi-State Status LED Engine:**
  * **Blink Yellow (4 Hz):** Device identify / locate mode (`blink 1`).
  * **Blink Red (4 Hz):** Motor stalled / fault tripped.
  * **Off:** Motor driver disabled (`ENA` low).
  * **Blink Green (4 Hz):** Enabled and actively receiving step pulses ($< 200\text{ ms}$).
  * **Solid Green:** Enabled and idle ($\ge 200\text{ ms}$).
* **USB CDC Virtual COM Port:**
  * Full-speed USB (crystal-less via HSI48 internal oscillator and clock recovery system).
  * Interactive serial CLI supporting configuration, monitoring, and real-time control.

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

## Physical Dynamics Model

### Torque vs. Speed Curve (`tcurve`)

Available motor torque $T_{\text{motor}}(v)$ is evaluated as a function of instantaneous speed $v = |\text{velocity}|$ (in encoder counts/sec):

```text
Torque (T)
  ^
T0|=============== (Holding torque flat shelf)
  |               \
  |                \ (Inductance & back-EMF derating)
  |                 \
T_min|               \___________
  +---------------------------------> Velocity (v)
  0             V_knee      V_max
```

$$\begin{cases}
T_{\text{motor}}(v) = T_0, & v \le V_{\text{knee}} \\
T_{\text{motor}}(v) = T_0 - \frac{T_0 - T_{\text{min}}}{V_{\text{max}} - V_{\text{knee}}} \cdot (v - V_{\text{knee}}), & V_{\text{knee}} < v < V_{\text{max}} \\
T_{\text{motor}}(v) = T_{\text{min}}, & v \ge V_{\text{max}}
\end{cases}$$

### Load Tension & Net Torque (`t`)

* $\tau_{\text{tension}} > 0$: Pulls in forward (+) direction.
* $\tau_{\text{tension}} < 0$: Pulls in reverse (-) direction.
* In commanded travel direction $\text{dir} \in \{+1, -1\}$:
  $$T_{\text{net}} = T_{\text{motor}}(v) + (\text{dir} \cdot \tau_{\text{tension}})$$

* **$T_{\text{net}} \ge 0$:** Motor drives normally, tracking commanded steps.
* **$T_{\text{net}} < 0$:**
  * Motor cannot advance.
  * If $|\tau_{\text{tension}}| > T_0$: Opposing load exceeds holding torque; rotor is pulled backward by the load.
  * If $|\tau_{\text{tension}}| \le T_0$: Rotor stalls in place.
  * Rotor position lag accumulates: $\text{lag} = |\text{StepToEncoderPosition}(\text{step\_pos}) - \text{planned\_encoder\_pos}|$.

### Stall Fault Trip (`stall`)

When engaged and $\text{lag} \ge \text{stall\_threshold}$:
* Motor enters fault state (`stall_tripped = true`).
* Status LED blinks red at 4 Hz.
* Asynchronous USB message emitted: `stall_trip 1\r\n`.
* Motor driving torque is cut to zero ($T_{\text{motor}} = 0$).
* Rotor enters viscous freewheeling under $\tau_{\text{tension}}$.

### Disengaged Freewheeling (`kfree`)

Active whenever the motor is disengaged (either `stall_tripped == true` or `ENA` is disabled):
$$V_{\text{freewheel}} = \tau_{\text{tension}} \cdot K_{\text{free}}$$
(clamped to $\pm V_{\text{max}}$). The DMA continuously streams quadrature pulses corresponding to this shaft rotation.

---

## Serial CLI Commands

Commands are sent via the USB Virtual COM Port (terminated with `\r` or `\n`).

| Command | Syntax | Description | Example |
| :--- | :--- | :--- | :--- |
| `t` | `t [int32]` | Query or set external load tension/torque | `t -1200` |
| `tcurve` | `tcurve [T0] [V_knee] [V_max] [T_min]` | Query or set torque-speed curve parameters | `tcurve 1000 1000 8000 200` |
| `stall` | `stall [uint32]` | Query or set stall threshold in encoder counts (`0` disables trip) | `stall 16` |
| `kfree` | `kfree [float]` | Query or set viscous freewheel coefficient | `kfree 0.005` |
| `blink` | `blink [0\|1]` | Query or toggle yellow identify/locate blink mode | `blink 1` |
| `zero` | `zero` | Zero physical encoder and commanded step positions | `zero` |
| `odr` | `odr [uint16]` | Set periodic position report rate in ms (`0` = disable) | `odr 500` |
| `epr` | `epr [uint16]` | Set encoder counts (pulses) per revolution | `epr 4000` |
| `spr` | `spr [uint16]` | Set input steps per revolution | `spr 1000` |
| `kp` | `kp [float]` | Set tracking proportional gain | `kp 0.1` |
| `kff` | `kff [float]` | Set feedforward velocity gain | `kff 1.0` |
| `lim1` | `lim1 <0\|1>` | Drive simulated limit switch 1 output | `lim1 1` |
| `lim2` | `lim2 <0\|1>` | Drive simulated limit switch 2 output | `lim2 0` |
| `r` | `r` | Dump full configuration and runtime status | `r` |
| `help` | `help` | Print command usage list | `help` |

### Asynchronous Event Messages

The emulator transmits asynchronous notifications when hardware states change:
* `stall_trip <0|1>`: Emitted when entering (`1`) or leaving (`0`) the stall trip state.
* `ena <0|1>`: Emitted when the `ENA` pin changes state.
* `rev <0|1>`: Emitted when the `DIR` pin changes state.
* `pos <int32>`: Emitted periodically at the configured `odr` interval.

---

## Building and Flashing

### Prerequisites

* **GNU Arm Embedded Toolchain** (`arm-none-eabi-gcc`)
* **CMake** (v3.20+) and **Ninja**
* **STM32CubeProgrammer** or **OpenOCD** / **ST-Link**

### Build Commands

```powershell
# Build Debug
ninja -C build/Debug

# Build Release
ninja -C build/Release
```
