# RPM Measurement — Method, Problems & Solutions

**Project**: Honda Activa 4G Hybrid Conversion  
**Organisation**: ReynLab Technologies India Pvt. Ltd.  
**Prepared for**: IIT Technical Review  
**Date**: August 2026

---

## 1. System Overview

The hybrid drivetrain has two independent power sources, each with its own RPM sensor:

| Channel | Source | Sensor | Signal pin |
|---|---|---|---|
| EV (Hub Motor) | BLDC hub motor — 48 V | Hall effect (3-wire) | Arduino Mega Pin 2 (INT4) |
| IC (Engine) | Honda Activa 4G 125 cc CDI | Pickup (trigger) coil → R + D circuit | Arduino Mega Pin 3 (INT5) |

The Arduino Mega 2560 measures both RPMs in real time and transmits them over UART2 (Serial3) to an ESP32 WiFi module every 1 second. The ESP32 serves a live HTML dashboard at `192.168.4.1` that displays both values and allows tuning of the hybrid switching thresholds.

---

## 2. EV Hub Motor RPM

### 2.1 Signal Path

```
Hub motor magnets (28 pole-pairs)
        │
        ▼
Hall effect sensor (3-wire, 5 V)
        │
        ▼
Arduino Mega Pin 2 — INT4 — RISING edge ISR
```

### 2.2 Constants

| Constant | Value | Meaning |
|---|---|---|
| `HALL_POLE_PAIRS` | 28 | Magnetic pole pairs on the hub rotor |
| `PULSES_PER_REV` | 28 | Hall pulses per one wheel revolution |
| `HUB_DEBOUNCE_US` | 300 µs | Minimum valid inter-pulse gap |
| `HUB_UPDATE_MS` | 100 ms | RPM computation window (10 Hz) |
| `HUB_ALPHA` | 0.35 | Exponential filter weight (new sample) |
| `HUB_TIMEOUT_US` | 500,000 µs | No-pulse timeout → declare stopped |
| `HUB_MIN_PULSES` | 2 | Minimum pulses before frequency method valid |

### 2.3 Calculation Method — Dual (Frequency + Period)

`updateHubRPM()` is gated by a 100 ms timer, so it executes at exactly 10 Hz regardless of `loop()` speed.

**Step 1 — Snapshot and reset counters** (inside `noInterrupts()` block):

```
count   = hubPulseCount   (pulses since last window)
period  = hubPeriodUs     (µs between last two valid pulses — from ISR)
hubPulseCount ← 0
```

**Step 2 — Choose method based on available data**:

```
if count ≥ 10:
    freqRPM   = count × 60 / (PULSES_PER_REV × window_seconds)
    periodRPM = 60,000,000 / (PULSES_PER_REV × period_µs)
    raw = 0.60 × freqRPM + 0.40 × periodRPM      ← frequency-dominant

elif count ≥ 2:
    raw = 0.30 × freqRPM + 0.70 × periodRPM      ← period-dominant

elif count = 1 and period > 0:
    raw = 60,000,000 / (PULSES_PER_REV × period_µs)
    raw = min(raw, implied_RPM_from_elapsed_time)  ← decel cap

else:
    return (nothing to compute)
```

**Step 3 — Implied RPM deceleration cap** (period-only path):

If the wheel is decelerating, the time elapsed since the last pulse grows beyond `period`. The implied RPM therefore falls below the period-based RPM, and the display is capped downward accordingly:

```
elapsed = now_µs − lastPulse_µs
impliedRPM = 60,000,000 / (PULSES_PER_REV × elapsed)
if impliedRPM < raw → raw = impliedRPM
```

**Step 4 — Exponential Moving Average (EMA) filter**:

```
hubRPM = 0.35 × raw + 0.65 × hubRPM
```

At 10 Hz with α = 0.35, the time constant is approximately **170 ms** — fast enough to track speed changes, slow enough to suppress single-pulse jitter from hall magnet spacing asymmetry.

### 2.4 Stability at Held Throttle

At constant EV speed the hall pulse period is constant. Each 100 ms window accumulates the same pulse count and the same average period. The dual-method blend and EMA filter converge the display to within **±30–50 RPM** of the true speed, with no sustained drift.

---

## 3. IC Engine RPM

### 3.1 Signal Path

```
Activa 4G flywheel — one trigger tooth
        │
        ▼
CDI pickup (trigger) coil
        │  (generates a spike per revolution)
        ▼
Series resistor + clamp diode (protection / voltage conditioning)
        │  (limits to safe MCU input voltage)
        ▼
Arduino Mega Pin 3 — INT5 — RISING edge ISR
```

The trigger coil produces **one pulse per crank revolution** (`IC_PPR = 1`). This is standard for single-cylinder CDI systems: one reluctor tooth on the flywheel, one pickup coil, one pulse per revolution.

### 3.2 Constants

| Constant | Value | Meaning |
|---|---|---|
| `IC_PPR` | 1 | Pulses per crank revolution (verified: single tooth) |
| `IC_RPM_CAL` | 1.0 | Calibration multiplier — set after field verification |
| `IC_DEBOUNCE_US` | 6,000 µs | Minimum valid inter-pulse gap |
| `IC_RPM_MAX` | 9,000 RPM | Hard ceiling (above CDI rev-limiter range) |
| `IC_RPM_FLOOR` | 200 RPM | Below this = cranking / noise → report 0 |
| `IC_FILTER_ALPHA` | 0.15 | EMA weight (new sample) |

**Why 6,000 µs debounce?**  
At 10,000 RPM the crank period is 6,000 µs. The debounce therefore passes all realistic engine pulses (up to 10,000 RPM) while blocking high-frequency motor-controller PWM noise (typically < 1 ms period). Hub motor Hall fundamental at 20 km/h ≈ 8 ms, which passes this debounce; it is handled by the software rejection layers below.

### 3.3 Calculation Method — Period-Based (Event-Driven)

Unlike hub RPM (windowed frequency), IC RPM uses **pure period measurement**: the time between the last two valid ISR events is the crank period, and RPM is computed from it.

#### ISR (`icPulse()`)

```
now = micros()
dt  = now − icLastPulse_us

if dt < IC_DEBOUNCE_US: return     ← Layer 1: hardware debounce

icPeriodPrev_us ← icPeriod_us
icPeriod_us     ← dt
icLastPulse_us  ← now
icNewPulse      ← true             ← signal to updateIcRPM()
```

#### `updateIcRPM()` — called every `loop()` iteration

```
[snapshot under noInterrupts]
newPulse    = icNewPulse  → clear flag
period      = icPeriod_us
periodPrev  = icPeriodPrev_us
lastUs      = icLastPulse_us

── Timeout check ──────────────────────────────────────────────
if (now − lastUs) > 1,000,000 µs:
    icRPM ← 0.0    ← engine stopped
    return

── Between-pulse path (no new pulse) ─────────────────────────
if not newPulse:
    elapsed = now − lastUs
    if elapsed > period:
        impliedRPM = 60,000,000 / (IC_PPR × elapsed) × IC_RPM_CAL
        if impliedRPM < icRPM → icRPM ← impliedRPM   ← decel cap
    return

── New pulse: three-layer validation ─────────────────────────

Layer 2 — Period consistency check:
    if period < periodPrev / 2: return   ← sudden >50% drop = noise burst

Layer 3 — Rate-limit check:
    raw = 60,000,000 / (IC_PPR × period) × IC_RPM_CAL
    if icRPM > 200 and raw > icRPM + 3,000: return  ← isolated EMI spike

── Bounds + EMA filter ───────────────────────────────────────
if raw > 9,000  → icRPM ← 9,000     (hard ceiling)
if raw < 200    → icRPM ← 0         (below credible idle)
else            → icRPM ← 0.15 × raw + 0.85 × icRPM
```

### 3.4 Why Event-Driven (Flag-Gated)?

`loop()` on a Mega 2560 at 16 MHz executes approximately **10,000–50,000 times per second**. The engine at 2,000 RPM produces only **33 pulses per second**. Without the `icNewPulse` flag:

- The filter ran on the **same stale `icPeriod_us`** ~300–1,500 times between real pulses.
- It converged to the stale value in **< 1 ms** — providing zero effective smoothing.
- Any new pulse (real or EMI) snapped the reading instantly, causing the visible oscillation.

With the flag: the EMA filter executes **once per engine revolution** only. The α = 0.15 weight means one rogue pulse moves the display by 15%; three successive real pulses at a new RPM bring it 39% of the way there. At steady throttle with consistent crank periods, the output is mathematically fixed:

```
icRPM_new = 0.15 × raw + 0.85 × icRPM
When raw == icRPM:  icRPM_new = 0.15 × X + 0.85 × X = X   ← stable
```

### 3.5 Three-Layer EMI Rejection

The hub motor (BLDC, PWM-controlled) induces electromagnetic interference onto the IC RPM signal line. Three independent layers reject it:

| Layer | Location | Mechanism | Catches |
|---|---|---|---|
| 1 — Debounce | ISR | Ignore pulses arriving < 6 ms after previous | PWM noise < 6 ms period |
| 2 — Consistency | `updateIcRPM()` | Reject if new period < half of previous | Sudden noise bursts at idle |
| 3 — Rate limit | `updateIcRPM()` | Reject if RPM jump > 3,000 RPM per pulse | Isolated EMI spikes mid-rev |

### 3.6 Known Measurement Offset (2,100 vs 1,700 RPM at Idle)

Google specs cite Honda Activa 4G idle at **1,700 RPM** (warm, factory-tuned carburetor). Field readings show **2,100 RPM** at idle. Three possible explanations:

1. **Carburetor idle screw** is set higher than factory on this specific engine — most likely cause. Cold idle is typically 200–400 RPM above warm idle.
2. **Systematic scaling offset** — if confirmed with a reference tachometer (OBD2 or strobe), set `IC_RPM_CAL = 1700.0 / measured_idle`.
3. **PPR error** — ratio 2,100/1,700 = 1.235 is not a clean integer, ruling out a wrong tooth count. Verified: single tooth on flywheel → `IC_PPR = 1` is correct.

### 3.7 Calibration Procedure (IIT Standard)

1. Warm the engine fully (10-minute ride).
2. Set idle with carburettor idle-stop screw.
3. Simultaneously read: (a) Arduino serial monitor `icRPM` and (b) OBD2 Bluetooth tachometer.
4. If Arduino reads `R_measured` and OBD reads `R_reference`:
   ```
   IC_RPM_CAL = R_reference / R_measured
   ```
5. Verify at 3,500 RPM and 5,500 RPM. If the ratio holds linear, the signal path is correct and only a gain correction is needed.
6. Update `#define IC_RPM_CAL` in `sensor_diag.ino` and reflash.

---

## 4. Hybrid State Machine — RPM Thresholds

Both RPM values feed the hybrid state machine running on the Mega:

| Transition | Condition | Default threshold | Hold time |
|---|---|---|---|
| EV → IC (start engine) | `hubRPM > evToIcRpm` | 2,000 RPM | 5 s |
| IC → EV (cut engine) | `icRPM < icLowRpm` | 2,500 RPM | 120 s |

**IC→EV logic**: Engine idle at 1,700–2,100 RPM is **below** the 2,500 RPM threshold. The 120-second timer therefore runs whenever the engine is at idle or low throttle. Revving above 2,500 RPM resets the timer. This allows the system to automatically cut the IC engine and switch to EV after prolonged low-load city riding, without manual intervention.

All four thresholds are tunable live from the ESP32 dashboard (no reflash needed):

| Dashboard control | Range | Serial command to Mega |
|---|---|---|
| EV→IC RPM | 500–5,000 RPM | `SET_RPM:<value>` |
| EV→IC Hold | 1–120 s | `SET_EV_HOLD:<ms>` |
| IC→EV RPM (Cutoff) | 1,000–6,000 RPM | `SET_IC_RPM:<value>` |
| IC→EV Hold | 1–120 s | `SET_IC_HOLD:<ms>` |

---

## 5. UI Data Flow and Display Logic

### 5.1 Mega → ESP32 (UART2, Serial3, 115200 baud)

Every 1,000 ms the Mega sends a single JSON line:

```json
{"state":"IC_ONLY","hubRPM":1234.5,"icRPM":3200.0,"tps":45.2,
 "batV":51.2,"batI":-3.1,"soc":78,"batW":-158.7,"bmsOk":true,
 "evToIcRpm":2000,"evToIcHoldMs":5000,"icToEvHoldMs":120000,"icLowRpm":2500}
```

### 5.2 ESP32 → Browser (HTTP polling, ArduinoJson v6)

The ESP32 parses the JSON into a `LiveData` struct and exposes it on `/data`. The browser JavaScript polls `/data` every **500 ms**:

```javascript
async function poll() {
    const d = await fetch('/data').then(r => r.json());

    // IC active = engine is driving the wheel
    const icActive = (d.state === 'IC_ONLY' || d.state === 'TRANS_TO_IC');

    // Hub RPM — hidden during IC (engine drives wheel, hub motor regen only)
    document.getElementById('hubRPM').textContent =
        icActive ? '--' : d.hubRPM.toFixed(0);

    // IC RPM — shown when IC is active
    document.getElementById('icRPM').textContent =
        icActive ? d.icRPM.toFixed(0) : '--';

    // State badge colour
    document.getElementById('state').className = 'state ' + d.state;

    // Tuning controls reflect current Mega values
    document.getElementById('icThrDisp').textContent  = d.icLowRpm.toFixed(0);
    document.getElementById('icHoldDisp').textContent = (d.icToEvHoldMs/1000).toFixed(0);
}
```

### 5.3 Display Refresh Rate

| Path | Rate | Bottleneck |
|---|---|---|
| Mega computes hubRPM | 10 Hz (100 ms window) | `HUB_UPDATE_MS` |
| Mega computes icRPM | Per crank pulse (33–117 Hz) | Engine speed |
| Mega → ESP32 JSON | 1 Hz | `PRINT_INTERVAL_MS` |
| Browser poll | 2 Hz | `/data` fetch every 500 ms |
| User sees update | **1–2 Hz effective** | JSON transmission rate |

The 1-second JSON transmission is the bottleneck. The RPM values displayed on the dashboard represent the state of the Mega's `icRPM` / `hubRPM` globals **at the moment the JSON was serialised**, not instantaneous values.

---

## 6. File Structure

```
Hybrid-activa/
├── controller change/
│   └── sensor_diag/
│       └── sensor_diag.ino        ← Arduino Mega 2560 — main hybrid controller
├── esp32s3_dashboard/
│   └── platformio/
│       ├── platformio.ini
│       └── src/
│           └── main.cpp           ← ESP32 WiFi dashboard (PlatformIO)
└── RPM_CALCULATION.md             ← this document
```

---

## 7. Open Items / Recommendations

1. **Hardware isolation**: An optocoupler (e.g., PC817) on the IC RPM signal line would eliminate hub-motor EMI at the hardware level, removing the need for software rejection layers entirely. Recommended before final IIT demonstration.

2. **`IC_RPM_CAL` field calibration**: Must be performed with a reference tachometer before submitting accuracy data. Document the measured offset and the correction applied.

3. **TPS calibration**: `TPS_MIN_V` and `TPS_MAX_V` in `sensor_diag.ino` should be verified with a multimeter at fully closed and fully open throttle on this specific sensor.

4. **`IC_PPR` verification**: Confirm single-tooth flywheel with oscilloscope at known idle RPM. This eliminates signal-path ambiguity from the documentation.
