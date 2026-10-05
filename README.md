# Stepper Driver Emulator

A high-performance firmware emulator running on an **STM32F042** microcontroller that simulates a stepper motor with an attached quadrature incremental encoder.

It monitors standard stepper controller signals (**Step/PUL**, **Direction/DIR**, and **Enable/ENA**), tracks motor position, simulates physical motor dynamics (torque curves, external load tension, rotor stall, slip, and fault disengagement), and outputs clean, phase-deterministic quadrature encoder signals (**EA** and **EB**) driven via DMA direct-to-GPIO.

---

## Features

* **Direct DMA-to-GPIO Quadrature Generation:**
  * Uses **TIM3** as a dynamic pacing heartbeat triggering **DMA1 Channel 3** to push atomic bitmasks directly into `GPIOB->BSRR`.
  * Eliminates phase flip errors, toggle-mode polarity memory issues, and glitch transitions across sudden direction changes.
  * Supports quadrature streaming rates up to 200 kHz sustained (50 kHz input step pulse rate at 4 counts/step ratio).
* **Zero-Float Real-Time Architecture:**
  * 100% fixed-point integer math ($Q12$ and $Q16$ types), eliminating all IEEE-754 software emulation library routines (`__aeabi_f*`, `__aeabi_d*`).
  * Reclaimed 3,972 bytes of Flash space and eliminated unbounded soft-float latency in critical motion paths.
  * Automated CFG cycle estimation verifies a 50.85 µs nominal DMA ISR budget at `CHUNK_SIZE = 16` (63.6% CPU load at 50 kHz step rate).
* **Accurate Input Step Tracking:**
  * Captures pulse periods on PA5 via **TIM2** (running at 48 MHz) and streams captured timestamps through **DMA1 Channel 5**.
  * Direction is sampled with interrupt-level precision on PA4.
* **Configurable Gear/Resolution Ratio:**
  * Configure steps per revolution and encoder counts per revolution via `ratio <spr> <epr>`, automatically reduced to canonical coprime integers.
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
  * **Blink Green (4 Hz):** Enabled and actively receiving step pulses (< 200 ms).
  * **Solid Green:** Enabled and idle (≥ 200 ms).
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

```math
\begin{cases}
T_{\text{motor}}(v) = T_0, & v \le V_{\text{knee}} \\
T_{\text{motor}}(v) = T_0 - \frac{T_0 - T_{\text{min}}}{V_{\text{max}} - V_{\text{knee}}} \cdot (v - V_{\text{knee}}), & V_{\text{knee}} \lt v \lt V_{\text{max}} \\
T_{\text{motor}}(v) = T_{\text{min}}, & v \ge V_{\text{max}}
\end{cases}
```

### Load Tension and Net Torque (`t`)

The `t` parameter models an external directional force or torque vector $\tau_{\text{tension}}$ acting continually along the axis in the emulator's coordinate frame.

#### Coordinate Direction Conventions

* **Forward (+ / $+1$):**
  * `DIR` input pin (PA4) is HIGH (`rev 0`).
  * Commanded step position (`step_pos`) increases ($+\Delta \text{step}$).
  * Encoder position increases ($+\Delta \text{pos}$); quadrature output streams Phase A leading Phase B.
* **Reverse (- / $-1$):**
  * `DIR` input pin (PA4) is LOW (`rev 1`).
  * Commanded step position (`step_pos`) decreases ($-\Delta \text{step}$).
  * Encoder position decreases ($-\Delta \text{pos}$); quadrature output streams Phase B leading Phase A.

#### Sign of Load Tension (`t`)

* **$\tau_{\text{tension}} \gt 0$ (Positive Tension):**
  * Exerts an external force/torque pulling continuously in the **positive direction** (+ counts / forward).
  * **Assists** forward motion; **opposes** reverse motion.
  * Pulls the rotor in the $+C$ direction during freewheeling or slip.
* **$\tau_{\text{tension}} \lt 0$ (Negative Tension):**
  * Exerts an external force/torque pulling continuously in the **negative direction** (- counts / reverse).
  * **Opposes** forward motion; **assists** reverse motion.
  * Pulls the rotor in the $-C$ direction during freewheeling or slip.
* **$\tau_{\text{tension}} = 0$:**
  * Free unloaded shaft; zero external force.

#### Directional Interaction Matrix & Net Torque

In the commanded travel direction $`\text{dir} \in \{+1, -1\}`$ (where $`\text{dir} = \mathrm{sgn}(P_{\text{cmd}} - P_{\text{enc}})`$):

$$
T_{\text{net}} = T_{\text{motor}}(v) + (\text{dir} \cdot \tau_{\text{tension}})
$$

| Commanded Travel (`dir`) | `DIR` Pin (PA4) | Tension Sign (`t`) | Force Vector Direction | Effect on Motor | Net Torque ($T_{\text{net}}$) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Forward** (`+1`) | HIGH | **Positive** (`t > 0`) | Pulls forward (+ counts) | **Assists / Aids** motor | $T_{\text{motor}}(v) + \lvert t \rvert$ |
| **Forward** (`+1`) | HIGH | **Negative** (`t < 0`) | Pulls backward (- counts) | **Resists / Opposes** motor | $T_{\text{motor}}(v) - \lvert t \rvert$ |
| **Reverse** (`-1`) | LOW | **Positive** (`t > 0`) | Pulls forward (+ counts) | **Resists / Opposes** motor | $T_{\text{motor}}(v) - \lvert t \rvert$ |
| **Reverse** (`-1`) | LOW | **Negative** (`t < 0`) | Pulls backward (- counts) | **Assists / Aids** motor | $T_{\text{motor}}(v) + \lvert t \rvert$ |

#### Motor Response Regimes

* **$T_{\text{net}} \ge 0$ (Sufficient Torque):**
  * Motor drives normally toward commanded position, tracking input step pulses.
* **$T_{\text{net}} \lt 0$ (Torque Deficit):**
  * Opposing load exceeds current motor torque capability ($`|t| \gt T_{\text{motor}}(v)`$); motor cannot advance in the commanded direction.
  * **Slip / Back-driving ($|t| \gt T_0$):** If the opposing load exceeds static holding torque $T_0$, the load overpowers the motor and back-drives the rotor in the direction of the load at slip speed: $V_{\text{slip}} = (|t| - T_0) \cdot K_{\text{free}} \quad [C/s]$. Direction of slip matches the sign of `t` (`t > 0` slips in $+C$, `t < 0` slips in $-C$).
  * **Static Stall ($|t| \le T_0$):** If the opposing load does not exceed holding torque, the rotor locks in place ($V = 0$).
  * In both cases, rotor lag accumulates against commanded steps: $\text{lag} = |P_{\text{cmd}} - P_{\text{enc}}|$.

#### Testing Quick-Reference (Common Scenarios)

* **Opposing load against forward travel:** Use **negative** tension (e.g. `t -1200` opposes forward motion, stalling if $|-1200| \gt T_0$).
* **Opposing load against reverse travel:** Use **positive** tension (e.g. `t 1200` opposes reverse motion, stalling if $|1200| \gt T_0$).
* **Vertical / Gravity load (pulling downward in negative direction):** Use **negative** tension (e.g. `t -500`). Moving up (+ counts) requires overcoming load; moving down (- counts) is assisted; disabling the drive (`ENA` low) causes downward freewheeling.

### Stall Fault Trip (`stall`)

When engaged and $\text{lag} \ge \text{stall}$ (where $\text{stall}$ is the configured threshold):
* Motor enters fault state (`stall_tripped = true`).
* Status LED blinks red at 4 Hz.
* Asynchronous USB message emitted: `stall_trip 1\r\n`.
* Motor driving torque is cut to zero ($T_{\text{motor}} = 0$).
* Rotor enters viscous freewheeling under $\tau_{\text{tension}}$.

### Disengaged Freewheeling (`kfree`)

Active whenever the motor is disengaged (either `stall_tripped == true` or `ENA` is disabled):

$$
V_{\text{freewheel}} = \tau_{\text{tension}} \cdot K_{\text{free}} \quad [C/s]
$$

(clamped to $\pm V_{\text{max}}$). The DMA continuously streams quadrature pulses corresponding to this shaft rotation:
* $\tau_{\text{tension}} \gt 0$: Freewheels in the forward (+ counts) direction ($+V_{\text{freewheel}}$).
* $\tau_{\text{tension}} \lt 0$: Freewheels in the reverse (- counts) direction ($-V_{\text{freewheel}}$).
* $\tau_{\text{tension}} = 0$: Shaft remains stationary ($V = 0$).

### Closed-Loop Motion Tracking & Feedforward Control (`kff`, `kp`)

When the motor is enabled and operating under sufficient torque ($T_{\text{net}} \ge 0$), the motion planner synthesizes quadrature encoder output pulses to precisely track incoming step pulses. The target velocity $V_{\text{target}}$ (expressed in encoder counts per millisecond, or kHz) controlling the pacing timer (**TIM3**) is computed using a combined velocity feedforward and proportional position feedback control law:

$$
V_{\text{target}} = K_{\text{ff}} \cdot V_{\text{in}} + K_{\text{p}} \cdot e_{\text{eff}} \quad [C / \text{ms}]
$$

where $V_{\text{in}}$ is the instantaneous measured input step velocity scaled to encoder counts/ms, evaluated from the 48 MHz timer (**TIM2**) input capture on pin PA5:

$$
V_{\text{in}} = \pm \left( \frac{48{,}000}{\text{period}} \right) \cdot \left( \frac{\text{epr}}{\text{spr}} \right) \quad [C / \text{ms}]
$$

(where `period` is the timer tick count captured on PA5, and the sign matches the direction sampled on PA4: `+` forward, `-` reverse).

The tracking error parameters are:
* $e = P_{\text{cmd}} - P_{\text{enc}}$ is the instantaneous position tracking error in encoder counts ($[C]$).
* $e_{\text{eff}}$ is the effective tracking error processed through a soft-knee jitter attenuation filter.

#### Role of Velocity Feedforward Gain ($K_{\text{ff}}$)

Velocity feedforward ($K_{\text{ff}}$) provides open-loop, predictive speed matching based directly on the measured input step pulse frequency:
* **Zero-Lag Synchronous Tracking ($K_{\text{ff}} = 1.0$, Default):** The emulator immediately matches the input step frequency on the quadrature output, scaled by the gear ratio $\text{epr} / \text{spr}$. During constant-velocity travel, the motor tracks with zero steady-state phase lag—it does not need to accumulate position error before generating output motion.
* **Lagging Dynamics ($K_{\text{ff}} \lt 1.0$):** Output velocity runs below input velocity until an accumulating position error ($e$) generates sufficient restoring velocity via $K_{\text{p}}$. This simulates inertial rotor lag or compliance during velocity transients.
* **Pure Feedback Mode ($K_{\text{ff}} = 0.0$):** Disables velocity anticipation entirely. Motion is driven strictly by accumulated position discrepancy ($e$).

#### Role of Proportional Position Gain ($K_{\text{p}}$)

Proportional feedback gain ($K_{\text{p}}$) acts as the restoring stiffness that pulls the physical encoder output position ($P_{\text{enc}}$) into alignment with the commanded step position ($P_{\text{cmd}}$):
* **Units & Scaling:** $K_{\text{p}}$ has dimensions of inverse time ($[s^{-1}]$), configured in firmware as $1 / \text{ms}$ (or $1000 \cdot \text{s}^{-1}$).
* **Restoring Authority:** For any tracking error, $K_{\text{p}}$ contributes a corrective velocity $\Delta V = K_{\text{p}} \cdot e_{\text{eff}}$. For example, with the default $K_{\text{p}} = 0.1000\text{ ms}^{-1}$, an instantaneous error of $10\text{ counts}$ produces an additional corrective velocity of $0.1 \times 10 = 1.0\text{ count/ms} = 1{,}000\text{ counts/s}$ toward eliminating the discrepancy.
* **Transient & Stop Settling:** During start/stop transients, direction reversals, or step-rate changes, $K_{\text{p}}$ eliminates accumulated phase error. When step pulses cease ($V_{\text{in}} = 0$), $K_{\text{p}}$ remains active to drive any residual error counts to zero, guaranteeing that the final resting encoder position exactly matches the commanded step position.
* **Low-Frequency & Single-Step Pacing:** At low step pulse frequencies (e.g. 50 Hz, inter-pulse period $T = 20\text{ ms}$), $K_{\text{ff}}$ paces output transitions continuously across the measured step period (e.g. $50\text{ Hz} \times 4 = 200\text{ counts/s}$, or $5\text{ ms}$ per quadrature count) rather than bursting all counts into a 1 ms cluster. For isolated single steps from standstill where no incoming frequency has been established yet, $K_{\text{p}}$ governs the transition rate ($V = K_{\text{p}} \cdot e$), allowing tunable step response dynamics without artificial rate clamping.

#### Soft-Knee Small-Signal Attenuation

Because incoming step pulses are discrete events arriving at finite intervals, the discrete error $e = P_{\text{cmd}} - P_{\text{enc}}$ inherently fluctuates between $0$ and $\frac{\text{epr}}{\text{spr}}\text{ counts}$ (the count width of one input step) as the current step is actively being paced across the inter-step interval by velocity feedforward ($K_{\text{ff}} \cdot V_{\text{in}}$).

Applying unfiltered proportional gain directly to this single-step execution window causes cyclic pacing timer frequency modulation across the phase, compressing quadrature edges into the first half of the step period.

To eliminate this phase modulation while preserving full restoring authority for genuine tracking errors, the motion planner deducts the feedforward-managed step window ($W_{\text{ff}} = K_{\text{ff}} \cdot \Delta P_{\text{step}}$) during active pulse streaming ($V_{\text{in}} \ne 0$) and applies a quadratic soft-knee attenuation profile to any excess tracking lag:

$$
e_{\text{lag}} = \begin{cases}
e - W_{\text{ff}}, & e \gt W_{\text{ff}} \text{ (forward streaming)} \\
e + W_{\text{ff}}, & e \lt -W_{\text{ff}} \text{ (reverse streaming)} \\
0, & \text{within feedforward window} \\
e, & \text{overshoot / leading error}
\end{cases}
$$

$$
e_{\text{eff}} = \begin{cases}
\frac{e_{\text{lag}} \cdot |e_{\text{lag}}|}{\Delta P_{\text{step}}}, & |e_{\text{lag}}| \le \Delta P_{\text{step}} \\
e_{\text{lag}}, & |e_{\text{lag}}| \gt \Delta P_{\text{step}}
\end{cases}
$$

where $\Delta P_{\text{step}} = \frac{\text{epr}}{\text{spr}}$ is the count equivalent of one input step.

* **Within feedforward window:** Output transitions pace with uniform frequency across the entire inter-step period governed by $K_{\text{ff}} \cdot V_{\text{in}}$, eliminating cyclic intra-step velocity modulation on the oscilloscope.
* **Excess tracking errors ($|e_{\text{lag}}| \gt 0$):** The profile transitions smoothly to provide proportional stiffness ($K_{\text{p}}$) to eliminate accumulated lag during accelerations or torque disturbances.
* **Streaming catch-up ceiling:** During active pulse streaming ($V_{\text{in}} \ne 0$), proportional catch-up authority is bounded to prevent transient lags from triggering positive-feedback interrupt saturation at high step frequencies ($> 6\text{ kHz}$).
* **At rest ($V_{\text{in}} = 0$):** Feedforward windowing is automatically bypassed ($e_{\text{eff}} = e$), ensuring rapid, exact zero-error static settling.

### Dimensional Analysis & Unit Relationships

The firmware uses a generalized, dimensionless coordinate system that models physical mechanics through proportional scaling equations. Rather than enforcing fixed metric or imperial units, all parameters operate consistently across three primary dimensions:

* **$[C]$ — Displacement:** Quantum of rotor angle or position measured in **encoder counts**.
  * $1\text{ count} = \frac{1}{\text{epr}}\text{ revolutions} = \frac{360^\circ}{\text{epr}} = \frac{2\pi}{\text{epr}}\text{ radians}$.
  * For linear systems (leadscrews, timing belts, rack-and-pinion), each count corresponds to a linear step distance $\Delta x = \frac{\text{lead}}{\text{epr}}$ or $\frac{2\pi r_{\text{pulley}}}{\text{epr}}$.
* **$[s]$ — Time:** Measured in **seconds** (with milliseconds used for timer intervals).
* **$[T]$ — Torque / Force:** Arbitrary consistent unit of effort (e.g. $\text{mN}\cdot\text{m}$, $\text{N}\cdot\text{cm}$, $\text{oz}\cdot\text{in}$, or linear force $\text{N}$ scaled by actuator radius).

#### Dimensional Parameter Mapping

| Parameter / Variable | CLI Command | Firmware Unit | Dimension | Physical Meaning & Proportional Relationship |
| :--- | :--- | :--- | :--- | :--- |
| $\text{ratio}$ | `ratio` | counts / step | $[C / S]$ | Canonical gear ratio: reduced $\text{spr}$ (steps/rev) to $\text{epr}$ (counts/rev) |
| $T_0$ | `tcurve` | torque units | $[T]$ | Maximum holding torque capability at speeds $v \le V_{\text{knee}}$ |
| $T_{\text{min}}$ | `tcurve` | torque units | $[T]$ | Residual pull-out torque at high speeds $v \ge V_{\text{max}}$ |
| $V_{\text{knee}}$ | `tcurve` | counts / sec | $[C / s]$ | Knee speed below which torque is flat: $\omega_{\text{knee}} = \frac{V_{\text{knee}}}{\text{epr}}\text{ rev/s}$ |
| $V_{\text{max}}$ | `tcurve` | counts / sec | $[C / s]$ | Cutoff speed where torque drops to $T_{\text{min}}$: $\omega_{\text{max}} = \frac{V_{\text{max}}}{\text{epr}}\text{ rev/s}$ |
| $\tau_{\text{tension}}$ | `t` | torque units | $[T]$ | External load torque or tension (signed: `+` pulls forward in $+C$, `-` pulls reverse in $-C$) |
| $\text{stall}$ | `stall` | counts | $[C]$ | Permissible rotor position lag: $\Delta \theta_{\text{lag}} = \frac{\text{stall}}{\text{epr}}\text{ rev} = \text{stall} \cdot \Delta x$ |
| $K_{\text{free}}$ | `kfree` | $\frac{\text{counts/s}}{\text{torque unit}}$ | $[C \cdot s^{-1} \cdot T^{-1}]$ | Viscous freewheel mobility coefficient (inverse damping $1/b$) |
| $K_{\text{ff}}$ | `kff` | dimensionless | $[-]$ | Velocity feedforward gain (scales measured input step rate to target velocity) |
| $K_{\text{p}}$ | `kp` | $1 / \text{ms}$ | $[s^{-1}]$ | Proportional position restoring gain (corrective velocity per count of tracking error) |

#### Core Governing Equations

##### 1. Torque Homogeneity Requirement

$$
T_{\text{net}} = T_{\text{motor}}(v) + \text{dir} \cdot \tau_{\text{tension}} \quad [T]
$$

Because $T_{\text{motor}}(v)$ and $\tau_{\text{tension}}$ are algebraically summed to determine torque deficit, $T_0$, $T_{\text{min}}$, and $\tau_{\text{tension}}$ **must share the exact same torque unit** $[T]$.

##### 2. Viscous Terminal Velocity & Slip Compliance

$$
V_{\text{freewheel}} = \tau_{\text{tension}} \cdot K_{\text{free}} \quad [C / s]
$$

$$
V_{\text{slip}} = (|\tau_{\text{tension}}| - T_0) \cdot K_{\text{free}} \quad [C / s]
$$

$K_{\text{free}}$ converts torque deficit or freewheeling load directly into rotor velocity. In classical mechanics with viscous damping torque $\tau = b \cdot \omega$, where $\omega = \frac{2\pi}{\text{epr}} V$:

$$
K_{\text{free}} = \frac{\text{epr}}{2\pi \cdot b_{\text{angular}}} \quad\text{or for linear actuators:}\quad K_{\text{free}} = \frac{1}{\Delta x \cdot b_{\text{linear}}}
$$

##### 3. Stall Threshold to Physical Motion

$$
\text{lag} = |P_{\text{cmd}} - P_{\text{enc}}| \quad [C]
$$

$$
\text{Angular Error} = \frac{\text{lag}}{\text{epr}} \times 360^\circ, \qquad \text{Linear Error} = \text{lag} \times \Delta x
$$

##### 4. Closed-Loop Velocity Synthesis & Soft-Knee Tracking

$$
V_{\text{target}} = K_{\text{ff}} \cdot V_{\text{in}} + K_{\text{p}} \cdot e_{\text{eff}} \quad [C / \text{ms}]
$$

$$
e_{\text{eff}} = \frac{e \cdot |e|}{\text{epr} / \text{spr}} \quad (\text{for } |e| \le \text{epr} / \text{spr} \text{ during active pulse streaming})
$$

#### Parameter Sizing & Calibration Recipe

To configure consistent parameters for any target motor and mechanism:

1. **Choose Torque Resolution $[T]$:**
   * Pick an integer scale where $T_0$ represents nominal holding torque (e.g., $T_0 = 1000$).
   * Scale external load commands (`t`) to match this unit (e.g. if the motor holds $1.0\text{ N}\cdot\text{m}$, setting $T_0 = 1000$ means $1\text{ unit} = 1\text{ mN}\cdot\text{m}$, so a $0.5\text{ N}\cdot\text{m}$ load is `t 500`).
2. **Set Velocity Range $[C/s]$:**
   * Desired knee speed (RPM): $V_{\text{knee}} = \frac{\mathrm{RPM}_{\text{knee}} \cdot \text{epr}}{60}$.
   * Maximum speed (RPM): $V_{\text{max}} = \frac{\mathrm{RPM}_{\text{max}} \cdot \text{epr}}{60}$.
3. **Tune Freewheel Mobility $K_{\text{free}}$:**
   * Decide the terminal freewheel velocity $V_{\text{target}}$ $[C/s]$ when subjected to a nominal load $\tau_{\text{test}}$ $[T]$: $K_{\text{free}} = \frac{V_{\text{target}}}{\tau_{\text{test}}}$.
   * Example: If an external load of $1000\text{ units}$ should free-wheel the motor at $5\text{ rev/s}$ ($20{,}000\text{ counts/s}$ with $\text{epr} = 4000$): $K_{\text{free}} = \frac{20000}{1000} = 20.0$.
4. **Set Stall Trip Sensitivity:**
   * To trip after $\Phi$ revolutions of slip: $\text{stall} = \Phi \cdot \text{epr}$.
   * Example: To trip after a half-rotation of slip with $\text{epr} = 4000$: $\text{stall} = 0.5 \times 4000 = 2000$.
5. **Tune Dynamic Tracking Gains ($K_{\text{ff}}$ and $K_{\text{p}}$):**
   * Keep $K_{\text{ff}} = 1.0$ (default) for standard synchronous tracking with zero steady-state phase lag.
   * If step pulse sources suffer from jitter, reduce $K_{\text{p}}$ (e.g. $0.05$ to $0.08\text{ ms}^{-1}$) for greater filtering compliance.
   * For rapid transient tracking or stiff mechanical coupling, increase $K_{\text{p}}$ (e.g. $0.15$ to $0.25\text{ ms}^{-1}$).

---

## Serial CLI Commands

Commands are sent via the USB Virtual COM Port (terminated with `\r` or `\n`).

| Command | Syntax | Description | Example Command | Serial Response |
| :--- | :--- | :--- | :--- | :--- |
| `t` | `t [int32]` | Query or set load tension/torque (signed: `+` pulls forward, `-` pulls reverse; e.g. `t -1200` opposes forward motion) | `t -1200` | `t -1200\r\n` |
| `tcurve` | `tcurve [T0] [V_knee] [V_max] [T_min]` | Query or set torque-speed parameters | `tcurve 1000 1000 8000 200` | `tcurve 1000 1000 8000 200\r\n` |
| `stall` | `stall [uint32]` | Query or set stall threshold (`0` = disable) | `stall 4000` | `stall 4000\r\n` |
| `kfree` | `kfree [float]` | Query or set viscous freewheel coefficient | `kfree 0.005` | `kfree 0.0050\r\n` |
| `blank` | `blank [float]` | Query or set step blanking / hold-off window in µs (e.g. `3.5` for 200 kHz) | `blank 3.5` | `blank 3.5000\r\n` |
| `blink` | `blink [0\|1]` | Query or toggle yellow identify blink | `blink 1` | `blink 1\r\n` |
| `pos` | `pos [int64]` | Query or set encoder position (int64) | `pos 0` | `pos 0\r\n` |
| `pvt` | `pvt` | Query instantaneous position, velocity, and torque | `pvt` | `pvt 0 0 1000 1000\r\n` |
| `odr` | `odr [uint16]` | Periodic position report rate in ms (`0` = off) | `odr 500` | `odr 500\r\n` |
| `ratio` | `ratio [spr] [epr]` | Query or set canonical gear ratio (steps/rev and encoder counts/rev) | `ratio 1000 4000` | `ratio 1 4\r\n` |
| `kp` | `kp [float]` | Query or set proportional position restoring gain in 1/ms (default: `0.1000`) | `kp 0.1` | `kp 0.1000\r\n` |
| `kff` | `kff [float]` | Query or set velocity feedforward gain (default: `1.0000` for zero-lag tracking) | `kff 1.0` | `kff 1.0000\r\n` |
| `lim1` | `lim1 <0\|1>` | Drive simulated limit switch 1 pin | `lim1 1` | `lim1 1\r\n` |
| `lim2` | `lim2 <0\|1>` | Drive simulated limit switch 2 pin | `lim2 0` | `lim2 0\r\n` |
| `save` | `save` | Save configuration to non-volatile flash | `save` | `save ok\r\n` |
| `r` | `r` | Dump full configuration and runtime status | `r` | Multi-line report (see below) |
| `help` | `help` | Print command usage list | `help` | Usage list (see below) |

> **Note on Queries:** Commands that accept optional parameters (`pos`, `t`, `tcurve`, `stall`, `kfree`, `blank`, `blink`, `odr`, `ratio`, `kp`, `kff`) return the current value when issued with no arguments (e.g. typing `pos` replies `pos 0\r\n`, typing `t` replies `t 0\r\n`, typing `ratio` replies `ratio 1 4\r\n`, typing `odr` replies `odr 1000\r\n`).

### Full State Report (`r` command)

Executes `r` to print all parameters and live hardware states:

```text
odr 1000
ratio 1 4
kp 0.1001
kff 1.0000
blank 3.5000
lim1 0
lim2 0
t 0
tcurve 1000 1000 8000 200
stall 4000
kfree 0.0049
stall_trip 0
blink 0
rev 0
ena 1
pos 0
pvt 0 0 1000 1000
```

### Error Responses

The CLI validates argument count, formatting, and numeric ranges, returning descriptive errors:
* **Invalid Argument Count / Usage:** `error: invalid usage: <usage_string>\r\n`
  * Example: `t 10 20` $\to$ `error: invalid usage: t [int32]\r\n`
* **Invalid Data Type / Value:** `error: invalid <type>: <bad_value>\r\n`
  * Example: `t abc` $\to$ `error: invalid int32: abc\r\n`
  * Example: `blink 5` $\to$ `error: invalid bool (0 or 1): 5\r\n`
* **Unrecognized Command:** `error: unknown command: <input>\r\n`

### Asynchronous Event Messages

The emulator transmits asynchronous notifications over the Virtual COM Port as physical and logical states change:
* `stall_trip <0|1>\r\n`: Emitted when entering (`1`) or leaving (`0`) the stall trip fault state.
* `ena <0|1>\r\n`: Emitted when the `ENA` pin (PA3) transitions (`1` = enabled, `0` = disabled).
* `rev <0|1>\r\n`: Emitted when the `DIR` pin (PA4) transitions (`1` = reverse, `0` = forward).
* `pvt <int64> <int32> <int32> <int32>\r\n`: Emitted periodically at the configured `odr` interval (e.g. `pvt 4000 20000 850 350\r\n`), reporting instantaneous position, output step velocity in Hz, motor torque, and net torque.

---

## Real-Time Performance & Timing Specifications

The firmware operates on an ARM Cortex-M0 core running at 48 MHz (20.833 ns per clock cycle) with 1 Flash wait state (`FLASH_LATENCY_1`).

### Real-Time Streaming Architecture

* **Dual-Buffer Circular DMA Pacing:** The quadrature generator streams phase bitmasks via a 32-sample circular DMA buffer split into two 16-sample halves (`CHUNK_SIZE = 16`). When one half-buffer completes transmission, an interrupt triggers calculation and synthesis of the next chunk.
* **TIM2 Input Capture Bypass:** High-frequency step inputs (> 1 kHz) are paced through dynamic timer chunk synthesis, avoiding per-pulse CPU interrupt overhead.
* **Pure Integer Motion Core:** All position tracking, feedforward synthesis, soft-knee filtering, and torque calculations execute strictly using 32-bit and 64-bit fixed-point math ($Q12$ and $Q16$). No floating-point emulation routines or 64-bit integer divisions are executed during real-time streaming.

### Fixed-Point Data Types & Precision Strategy

To maintain deterministic execution within the real-time ISR without floating-point hardware or 64-bit software emulation, the motion engine employs discrete, strongly typed fixed-point representations (`q12_t` and `q16_t` in `mathutil.h`):

| Type | Representation | Why Used |
| :--- | :--- | :--- |
| **`q12_t`** | General rates & gains (`kp`, `kff`, `kfree`, `counts_per_step`, `step_blank_us`) | Prevents 32-bit overflow when multiplied by large velocities (up to 200,000 counts/s) on Cortex-M0. |
| **`q16_t`** | Inverses (`inv_counts_per_step`, `inv_torque_span_v`) | Provides 16× higher resolution to prevent small fractional reciprocals from quantizing to 0 or 1; safe from 32-bit overflow because the operands are strictly bounded. |

#### Design Rationale

* **32-bit Dynamic Range on Cortex-M0 (`q12_t`):**
  The ARM Cortex-M0 core features a single-cycle 32-bit hardware multiplier (`muls`) but lacks a 64-bit hardware multiplier (`smull`) and hardware divider (`sdiv`). Operating at 50 kHz step pulses with 4 counts/step yields a maximum velocity of $V_{\mathrm{in}} = 200,000\text{ counts/s}$. Scaling this velocity by a $Q12$ gain produces an intermediate product of $200,000 \times 4096 = 819,200,000$, which safely fits within signed 32-bit limits ($< 2.14 \times 10^9$). Using $Q16$ generally would produce $200,000 \times 65,536 = 1.31 \times 10^{10}$, causing 32-bit integer overflow and requiring slow 64-bit software emulation (`__aeabi_lmul`). Thus, `q12_t` is the optimal system default.

* **Quantization Prevention for Reciprocals (`q16_t`):**
  Parameters representing mathematical inverses (such as $\mathrm{inv\_torque\_span\_v} = 1 / (V_{\mathrm{max}} - V_{\mathrm{knee}})$ and $\mathrm{inv\_counts\_per\_step} = 1 / \mathrm{counts\_per\_step}$) evaluate to small fractions $\ll 1.0$. For example, a torque derating span of 8,000 counts/s has a reciprocal of $0.000125$. In $Q12$ ($1\text{ LSB} \approx 0.000244$), this fraction truncates to 0 or rounds to 1 (50% to 100% quantization error). In $Q16$ ($1\text{ LSB} \approx 0.0000153$), the value is represented accurately as 8. Because these inverses are exclusively multiplied by bounded, small quantities (such as quadratic error $\le (\text{counts per step})^2$ or speed offsets $\le \text{span}$), their intermediate products never exceed 32 bits, allowing $Q16$ precision to be used safely without overflow risk.


### Interrupt Execution Budget & Call Tree Breakdown

Interrupt latency and execution cycles were characterized via static disassembly and Control Flow Graph (CFG) analysis using `scripts/estimate_isr_cycles.py` on the ARM Cortex-M0 Release binary:

* **Nominal Steady-State ISR Execution:** **2,441 cycles** ($50.85\ \mu\mathrm{s}$) in silicon (including Flash wait states and hardware NVIC context stacking).
* **Available Budget per Chunk at 50 kHz Step Rate:** $80.00\ \mu\mathrm{s}$ ($16\text{ counts} / 200\text{ kHz counts/s}$ at 4 counts/step).
* **Steady-State CPU Utilization at 50 kHz:** $\frac{50.85\ \mu\mathrm{s}}{80.00\ \mu\mathrm{s}} = 63.6\%$.

| Component / Routine | 0-WS Cycles | Silicon Cycles (1-WS) | Duration (@ 48 MHz) | % of ISR | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `Motion_PlanStep` | 452 | 565 | 11.77 µs | 23.1% | Velocity feedforward, error compensation, and pacing calculation |
| `FillQuadChunk` | 252 | 315 | 6.56 µs | 12.9% | DMA chunk buffer dispatch, quadrature generation, and pacing |
| `CheckMotionIdle` | 226 | 282 | 5.88 µs | 11.6% | Step activity timeout and motion state transition detection |
| `Quadrature_GenerateChunk` | 148 | 185 | 3.85 µs | 7.6% | Gray-code quadrature transition bitmask synthesis |
| `UpdatePositionCounters` | 129 | 161 | 3.35 µs | 6.6% | 64-bit commanded step and encoder position accumulation |
| `__udivsi3` | 114 | 142 | 2.96 µs | 5.8% | 32-bit hardware-assisted unsigned division helper |
| `Position_FilterStepWithBlanking` | 99 | 124 | 2.58 µs | 5.1% | Hardware step capture blanking filter |
| `Motion_ShouldStop` | 86 | 108 | 2.25 µs | 4.4% | Boundary limit switch and deceleration check |
| `Quadrature_CalcTimerPacing` | 70 | 88 | 1.83 µs | 3.6% | TIM3 timer reload prescaler and auto-reload configuration |
| `Motion_CalcStepTimeoutMs` | 56 | 70 | 1.46 µs | 2.9% | Adaptive inter-step timeout computation |
| Hardware Context Stacking (NVIC) | 31 | 39 | 0.81 µs | 1.6% | ARMv6-M hardware exception entry/exit overhead |
| Other subroutines & handlers | 259 | 322 | 6.71 µs | 13.2% | DMA interrupt dispatcher, signed integer division, torque model |
| **Total Steady-State ISR** | **1,922** | **2,441** | **50.85 µs** | **100.0%** | **Full real-time chunk synthesis pipeline** |

### CPU Utilization Across Step Frequencies

With a default ratio of 4 counts/step (1000 SPR / 4000 CPR) and a chunk size of 16 counts:

| Input Step Rate | Encoder Count Rate | DMA ISR Period | CPU Utilization | Operating Status |
| :--- | :--- | :--- | :--- | :--- |
| 10.00 kHz | 40.00 kHz | 400.00 µs | 12.7% | Nominal load |
| 15.00 kHz | 60.00 kHz | 266.67 µs | 19.1% | Nominal load |
| 20.00 kHz | 80.00 kHz | 200.00 µs | 25.4% | Nominal load |
| 30.00 kHz | 120.00 kHz | 133.33 µs | 38.1% | Nominal load |
| 40.00 kHz | 160.00 kHz | 100.00 µs | 50.9% | Nominal load |
| **50.00 kHz** | **200.00 kHz** | **80.00 µs** | **63.6%** | **Target benchmark (sustained operation)** |
| 60.00 kHz | 240.00 kHz | 66.67 µs | 76.3% | High load |
| **78.66 kHz** | **314.63 kHz** | **50.85 µs** | **100.0%** | **Maximum theoretical saturation limit** |

### Firmware Memory Utilization

Memory footprint of the Release build (`build/Release/StepperDriverEmulator.elf`):

| Memory Region | Used Bytes | Total Bytes | Utilization | Free Space |
| :--- | :--- | :--- | :--- | :--- |
| **Flash** (`.text` + `.rodata` + `.data`) | 25,908 B | 31,744 B | **81.62%** | 5,836 B free |
| **RAM** (`.data` + `.bss` + stack) | 5,480 B | 6,144 B | **89.19%** | 664 B free |

* **Flash Savings:** Complete elimination of soft-float runtime helpers (`__aeabi_fmul`, `__aeabi_fadd`, `__aeabi_fsub`, `__aeabi_fdiv`, `__aeabi_f2iz`, `__aeabi_i2f`, etc.) reclaimed **3,972 bytes** of Flash memory.
* **Deterministic Timing:** Disallowance of software floating point and 64-bit integer division eliminates variable, data-dependent software emulation loops from the motion control path.

---

## Building and Flashing

### Prerequisites

* **GNU Arm Embedded Toolchain** (`arm-none-eabi-gcc`)
* **CMake** (v3.20+) and **Ninja**
* **STM32CubeProgrammer** or **OpenOCD** / **ST-Link**

### Build Commands

Build using CMake (which automatically invokes the underlying build tool specified in `CMakePresets.json`):

```powershell
# Build Debug
cmake --build build/Debug
# or using CMake presets:
cmake --build --preset Debug

# Build Release
cmake --build build/Release
# or using CMake presets:
cmake --build --preset Release
```

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
  1. Validates `README.md` (GitHub LaTeX compatibility, balanced math delimiters, and CLI command synchronization).
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
# Validate README.md formatting and CLI command table sync
python scripts/validate_readme.py

# Verify zero disallowed software float / 64-bit division symbols in ELF binary
python scripts/check_disallowed_symbols.py

# Check firmware memory usage against STM32F042 flash limits
python scripts/check_firmware_size.py build/Release/StepperDriverEmulator.elf

# Estimate interrupt execution cycle counts, silicon wait states, and CPU utilization
python scripts/estimate_isr_cycles.py --chunk-size 16
```


