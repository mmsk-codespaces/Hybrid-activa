/**
 * sensor_diag.ino
 * Arduino Mega 2560 — Hub Motor Hall + TPS + IC Engine RPM + Daly CAN BMS Diagnostic
 * ReynLab Technologies India Pvt. Ltd.
 *
 * Hub Motor RPM: Purpose-built for Hybrid Activa (open-loop, no PID).
 *   Adaptive period/frequency blend, implied deceleration, exp filter.
 *   PPR=28 verified by pulse-count test (283 pulses / 10 revolutions).
 *
 * ================================================================
 *  WIRING
 * ================================================================
 *
 *  HUB MOTOR HALL SENSOR:
 *    Hall signal wire (tap)   ──────►  Pin 2  (INT4)
 *    Motor controller GND     ──────►  GND
 *    (Do NOT connect Arduino 5V — motor controller powers the sensor)
 *
 *  TPS:
 *    TPS Signal               ──────►  Pin A0
 *    TPS GND                  ──────►  GND
 *
 *  IC ENGINE RPM (Pickup coil circuit):
 *    Blue/Yellow wire (tap)
 *      → [4.4kΩ] → NODE → [4.4kΩ] → GND
 *                   │
 *                   ├── 1N4007 cathode(stripe) → 5V
 *                   ├── 1N4007 anode → GND (stripe faces NODE)
 *                   └──────────────────────────► Pin 3  (INT5)
 *    Engine chassis / battery negative          ► GND
 *
 *  MCP2515 CAN MODULE (Daly Smart BMS — 500 kbps, 8 MHz crystal):
 *    VCC  ──────►  5V
 *    GND  ──────►  GND
 *    CS   ──────►  Pin 10
 *    SO   ──────►  Pin 50  (MISO)
 *    SI   ──────►  Pin 51  (MOSI)
 *    SCK  ──────►  Pin 52
 *    INT  ──────►  not connected  (polling mode)
 *    CAN_H ─────►  Daly BMS CAN_H
 *    CAN_L ─────►  Daly BMS CAN_L
 *
 *  NOTE: Enable the 120Ω termination jumper on the MCP2515 module.
 *        The Daly BMS has its own internal 120Ω at its end.
 *
 *  SERIAL MONITOR: 115200 baud
 *
 * ================================================================
 *  INTERRUPT PINS ON MEGA 2560
 * ================================================================
 *   Pin 2  → INT4   Pin 3  → INT5   Pin 18 → INT3
 *   Pin 19 → INT2   Pin 20 → INT1   Pin 21 → INT0
 * ================================================================
 */

#include <SPI.h>
#include <mcp_can.h>

// ================================================================
// HUB MOTOR CONFIGURATION
// ================================================================

#define HALL_SENSORS_USED   1
#define HALL_POLE_PAIRS     28      // Verified: 283 pulses / 10 revolutions

#if HALL_SENSORS_USED == 1
  #define PULSES_PER_REV    (HALL_POLE_PAIRS)        // 28
#else
  #define PULSES_PER_REV    (HALL_POLE_PAIRS * 3)    // 84
#endif

#define HALL_PIN_A          2

// Debounce: rejects pulses closer than this.
// PPR=28 at 7143 RPM → period = 300µs. Sets hard ceiling at 7143 RPM.
// Motor controller EMI spikes are typically < 200µs so 300µs is safe.
#define HUB_DEBOUNCE_US     300UL

// RPM update window: 100ms — faster than MIDC's 200ms for quicker display response.
// At 28 PPR, even 300 RPM gives 28×300/60×0.1 = 14 pulses per window. Plenty.
#define HUB_UPDATE_MS       100

// No pulse received for this long → wheel has stopped.
#define HUB_TIMEOUT_US      500000UL    // 500ms

// Exponential filter coefficient. Higher = faster response, less smoothing.
// 0.35 gives ~3 updates (~300ms) to reach 95% of a step change.
#define HUB_ALPHA           0.35f

// Minimum pulses per window to trust the frequency method.
#define HUB_MIN_PULSES      2

// ================================================================
// TPS CONFIGURATION
// ================================================================

#define TPS_PIN             A0
#define TPS_MIN_V           1.10f
#define TPS_MAX_V           2.70f
#define TPS_SAMPLES         50
#define TPS_AVG_WINDOW      8
#define TPS_DEADBAND_V      0.02f

// ================================================================
// IC ENGINE RPM CONFIGURATION
// ================================================================

#define IC_RPM_PIN          3

// Signal path: CDI pickup (trigger) coil → series resistor + clamp diode → Pin 3.
// The trigger coil fires 1 pulse per crank revolution (single-tooth reluctor on flywheel).
// If reading scales by a fixed ratio vs a reference tachometer, verify the tooth count
// on the flywheel physically — then update IC_PPR and/or IC_RPM_CAL accordingly.
#define IC_PPR              1

// Calibration multiplier applied after period→RPM conversion.
// Default 1.0 = no correction. Adjust after comparing against a reference
// tachometer at a known stable RPM (e.g. warm idle with OBD or strobe tach).
// Example: if code reads 2100 but reference reads 1700, set CAL = 1700.0/2100.0 = 0.810f
#define IC_RPM_CAL          1.0f

// RPM bounds — Activa 4G engine:
//   Idle (warm, stock tune): ~1700 RPM.  Cold/high-idle: up to ~2200 RPM.
//   Peak power:              ~7500 RPM.
//   CDI rev limiter:         ~8500–9000 RPM.
// Floor below 200 RPM = cranking / noise → report 0.
// Ceiling at 9000 RPM = above CDI cut → discard as noise.
#define IC_RPM_MAX          9000.0f
#define IC_RPM_FLOOR        200.0f

// Debounce: blocks pulses arriving faster than 6 ms apart.
//   → Passes engine pulses up to ~10,000 RPM (period ≥ 6ms at 10k RPM).
//   → Rejects high-frequency hub-motor PWM noise (typically < 1ms period).
//   → Hub motor Hall fundamental at 20 km/h ≈ 8 ms > 6 ms, so it passes debounce.
//     The period-consistency check in updateIcRPM() rejects those residual bursts.
#define IC_DEBOUNCE_US      6000UL

// ================================================================
// RELAY CONFIGURATION
// ================================================================

// Relay A and B — Active-LOW relay module
#define RELAY_A_PIN   7       // IC Engine Power
#define RELAY_B_PIN   9       // IC Engine Ignition Cutoff
#define RELAY_ON      LOW
#define RELAY_OFF     HIGH

// EV Key Cutoff — FRFER SSR-10DD (DC-DC Solid State Relay, Active-HIGH)
// HIGH = SSR conducts = KEY line live at 48V = EV controller ON
// LOW  = SSR open     = KEY line broken      = EV controller OFF
#define EV_CUT_PIN    6
#define EV_CUT_ON     LOW     // SSR off  → KEY broken  → EV OFF
#define EV_CUT_OFF    HIGH    // SSR on   → KEY live    → EV ON

// ================================================================
// HYBRID STATE MACHINE THRESHOLDS
// ================================================================

// All four tunable via ESP32 dashboard (SET_RPM / SET_EV_HOLD / SET_IC_HOLD / SET_IC_RPM)
float         evToIcRpm     = 2000.0f;  // hub RPM threshold to start IC
unsigned long evToIcHoldMs  = 5000UL;   // ms hub RPM must stay above threshold
float         icLowRpm      = 2500.0f;  // IC RPM below which IC→EV hold timer runs (idle ~1700 < 2500, revving resets it)
unsigned long icToEvHoldMs  = 120000UL; // ms IC RPM must stay below threshold (default 2 min)

// IC engine confirm after start
#define IC_MIN_RUN_RPM        1500.0f
#define IC_START_CONFIRM_MS   500UL
#define IC_STARTUP_TIMEOUT_MS 10000UL


// TRANS_TO_EV step timings (ms from entry — EV is restored at entry, IC cut after)
#define TEV_RELAY_B_ON        500UL     // IC ignition cut 500ms after EV restore
#define TEV_RELAY_A_OFF       1500UL    // IC power off at 1.5s
#define TEV_RELAY_B_OFF       2000UL    // Relay B off at 2s
#define TEV_TO_EV_ONLY        2500UL    // → EV_ONLY at 2.5s

// ================================================================
// DALY SMART BMS — CAN CONFIGURATION
// ================================================================

#define CAN_CS_PIN          10
#define BMS_QUERY_MS        2000    // send query every 2 seconds

// Daly Smart CAN IDs (from can_daly_smart_reader.py source)
#define DALY_WAKEUP_ID      0x35CUL          // 11-bit standard — PYLON heartbeat wakeup
#define DALY_QUERY_ID       0x0400FF80UL     // 29-bit extended — query trigger
#define DALY_PACK_ID        0x04028001UL     // 29-bit extended — pack V / I / SoC (primary)
#define DALY_CELL_ID        0x04008001UL     // 29-bit extended — cell voltages (voltage fallback)
#define DALY_SOC_ID         0x355UL          // 11-bit standard — SoC fallback (auto-broadcast)

MCP_CAN CAN(CAN_CS_PIN);

// ================================================================
// ENGINE EMISSIONS LUT
// ================================================================

#define LUT_SIZE 10

struct EngineOutput {
  float torque_Nm;
  float CO_ppm;
  float CO2_pct;
  float NOx_ppm;
};

float icRPMAxis[LUT_SIZE];
float throttleAxis[LUT_SIZE];
EngineOutput engineLUT[LUT_SIZE][LUT_SIZE];

// ================================================================
// TIMING
// ================================================================

#define PRINT_INTERVAL_MS   1000

// ================================================================
// GLOBALS — HUB MOTOR
// ================================================================

volatile uint32_t hubPulseCount   = 0;
volatile uint32_t hubTotalPulses  = 0;   // cumulative, never resets — PPR calibration
volatile uint32_t hubLastPulseUs  = 0;
volatile uint32_t hubPeriodUs     = 0;

float hubRPM = 0.0f;

// ================================================================
// GLOBALS — IC ENGINE
// ================================================================

volatile unsigned long icLastPulse_us  = 0;
volatile unsigned long icPeriod_us     = 0;   // period between last two valid pulses
volatile unsigned long icPeriodPrev_us = 0;   // previous period (consistency check)
volatile bool          icNewPulse      = false; // set by ISR, cleared by updateIcRPM()

// ================================================================
// GLOBALS — TPS
// ================================================================

float tpsHistory[TPS_AVG_WINDOW] = {0};
uint8_t tpsIndex = 0;
float tpsStable  = 0.0f;

// ================================================================
// GLOBALS — DALY BMS
// ================================================================

float batVoltage = 0.0f;
float batCurrent = 0.0f;
float batSoC     = 0.0f;
float batPower   = 0.0f;
bool  bmsValid   = false;   // false until first successful frame decoded

// ================================================================
// GLOBALS — HYBRID STATE MACHINE
// ================================================================

enum HybridState { EV_ONLY, TRANS_TO_IC, IC_ONLY, TRANS_TO_EV };
HybridState hybridState = EV_ONLY;

unsigned long stateEntryTime  = 0;   // when we entered current state
unsigned long conditionTimer  = 0;   // for hysteresis conditions
bool          conditionArmed  = false;

// IC start confirmation
unsigned long icConfirmStart  = 0;
bool          icConfirmArmed  = false;

// ================================================================
// GLOBALS — TIMING
// ================================================================

unsigned long lastPrintTime = 0;

// Shared sensor values — updated in print block, read by state machine
float icRPM      = 0.0f;
float throttlePct = 0.0f;

// ================================================================
// ISR — Hub Motor Hall
// ================================================================

void hallPulse() {
  uint32_t now = micros();

  // First edge: initialise timestamp only (need two edges for a period)
  if (hubLastPulseUs == 0) {
    hubLastPulseUs = now;
    return;
  }

  uint32_t dt = now - hubLastPulseUs;

  // Reject pulses that arrived too soon — motor controller EMI
  if (dt < HUB_DEBOUNCE_US) return;

  hubPeriodUs    = dt;
  hubLastPulseUs = now;
  hubPulseCount++;
  hubTotalPulses++;
}

// ================================================================
// ISR — IC Engine Pickup Coil
// ================================================================

void icPulse() {
  unsigned long now = micros();
  unsigned long dt  = now - icLastPulse_us;
  if (dt < IC_DEBOUNCE_US) return;
  icPeriodPrev_us = icPeriod_us;
  icPeriod_us     = dt;
  icLastPulse_us  = now;
  icNewPulse      = true;
}

// ================================================================
// Hub Motor RPM Update
//
// Adaptive blend strategy:
//   count >= 10 : frequency-heavy  (60% freq + 40% period)
//   count 2–9   : period-heavy     (30% freq + 70% period)
//   count 1     : period only
//   count 0     : implied RPM from elapsed time (deceleration tracking)
//
// No outlier rejection. No step clamping.
// The motor controller's own ramp limits acceleration naturally.
// HUB_DEBOUNCE_US already gates EMI at the ISR level.
// ================================================================

void updateHubRPM() {
  static uint32_t lastUpdate = 0;

  uint32_t nowMs = millis();
  if (nowMs - lastUpdate < HUB_UPDATE_MS) return;

  noInterrupts();
  uint32_t count     = hubPulseCount;
  uint32_t period    = hubPeriodUs;
  uint32_t lastPulse = hubLastPulseUs;
  uint32_t nowUs     = micros();
  hubPulseCount = 0;
  interrupts();

  float dt = (nowMs - lastUpdate) / 1000.0f;
  lastUpdate = nowMs;

  // --- Timeout: no pulse for HUB_TIMEOUT_US → stopped ---
  if (lastPulse > 0 && (nowUs - lastPulse) > HUB_TIMEOUT_US) {
    hubRPM = 0.0f;
    noInterrupts();
    hubPeriodUs    = 0;
    hubLastPulseUs = 0;
    interrupts();
    return;
  }

  float raw = 0.0f;

  if (count >= HUB_MIN_PULSES) {
    // --- Both methods available ---
    float freqRPM   = (count * 60.0f) / ((float)PULSES_PER_REV * dt);
    float periodRPM = (period > 0)
                    ? (60000000.0f / ((float)PULSES_PER_REV * (float)period))
                    : freqRPM;

    // Adaptive weighting: more pulses = frequency method is more reliable
    float fw = (count >= 10) ? 0.60f : 0.30f;
    raw = fw * freqRPM + (1.0f - fw) * periodRPM;

  } else if (period > 0) {
    // --- Only period method (1 pulse this window or carry-over period) ---
    raw = 60000000.0f / ((float)PULSES_PER_REV * (float)period);

    // Cap with implied RPM from elapsed time since last pulse.
    // If wheel is decelerating, elapsed grows → implied shrinks → display drops.
    if (lastPulse > 0) {
      uint32_t elapsed = nowUs - lastPulse;
      if (elapsed > 0) {
        float impliedRPM = 60000000.0f / ((float)PULSES_PER_REV * (float)elapsed);
        if (impliedRPM < raw) raw = impliedRPM;
      }
    }

  } else {
    return;   // Nothing to compute yet
  }

  // --- Exponential low-pass filter ---
  hubRPM = HUB_ALPHA * raw + (1.0f - HUB_ALPHA) * hubRPM;
  if (hubRPM < 0.0f) hubRPM = 0.0f;
}

void updateIcRPM() {
  noInterrupts();
  bool          newPulse   = icNewPulse;
  unsigned long period     = icPeriod_us;
  unsigned long periodPrev = icPeriodPrev_us;
  unsigned long lastUs     = icLastPulse_us;
  if (newPulse) icNewPulse = false;   // consume flag while interrupts are off
  interrupts();

  unsigned long nowUs = micros();

  // Engine stopped: no pulse for >1 second → hard zero
  if (lastUs == 0 || (nowUs - lastUs) > 1000000UL) {
    icRPM = 0.0f;
    return;
  }

  if (!newPulse) {
    // Between pulses: track deceleration by capping with elapsed-time implied RPM.
    // As time grows without a new pulse, implied RPM falls → display follows down.
    if (icRPM > 0.0f && period > 0) {
      unsigned long elapsed = nowUs - lastUs;
      if (elapsed > period) {
        float impliedRPM = (60000000.0f / ((float)IC_PPR * (float)elapsed)) * IC_RPM_CAL;
        if (impliedRPM < icRPM) icRPM = impliedRPM;
      }
    }
    return;
  }

  // ── New pulse: validate then compute ──────────────────────────────

  if (period == 0) return;

  // Layer 1 — period consistency check.
  // Real engine RPM changes gradually; noise causes sudden 50%+ period drop.
  if (periodPrev > 0 && period < periodPrev / 2) return;

  float raw = (60000000.0f / ((float)IC_PPR * (float)period)) * IC_RPM_CAL;

  // Layer 2 — rate-limit check.
  // Rejects single EMI pulses that jump the reading by >3000 RPM in one revolution.
  // Genuine throttle blips accelerate through consecutive accepted pulses, so real
  // rapid revving still tracks correctly; only isolated EMI spikes are blocked.
  if (icRPM > IC_RPM_FLOOR && raw > icRPM + 3000.0f) return;

  // Layer 3 — bounds and 0.15/0.85 exponential filter.
  // 0.15 weight means one rogue pulse moves the reading only 15%; three successive
  // real pulses at a new RPM will have it 39% of the way there — smooth and stable.
  if      (raw > IC_RPM_MAX)   icRPM = IC_RPM_MAX;
  else if (raw < IC_RPM_FLOOR) icRPM = 0.0f;
  else                         icRPM = raw * 0.15f + icRPM * 0.85f;
}

// ================================================================
// ENGINE EMISSIONS LUT — FUNCTIONS
//
// X-axis: IC engine RPM  (1000–8000, 10 points)
// Y-axis: throttle %     (0–100,     10 points)
//
// Emission model (Honda 110cc single-cylinder approximation):
//   Torque : peaks ~4500 RPM, scales with throttle
//   CO     : highest at idle (rich/incomplete), drops at cruise
//   CO2    : load-driven, rises with throttle
//   NOx    : peaks at high RPM + high throttle (peak combustion temp)
// ================================================================

void initializeLUT() {
  for (int i = 0; i < LUT_SIZE; i++) {
    icRPMAxis[i]   = 1000.0f + i * (7000.0f / (LUT_SIZE - 1));  // 1000–8000 RPM
    throttleAxis[i] = i * (100.0f / (LUT_SIZE - 1));              // 0–100 %
  }

  for (int i = 0; i < LUT_SIZE; i++) {
    for (int j = 0; j < LUT_SIZE; j++) {
      float rpm  = icRPMAxis[i];
      float thr  = throttleAxis[j];
      float nRPM = (rpm - 1000.0f) / 7000.0f;   // 0.0 at 1000 RPM, 1.0 at 8000 RPM
      float tFac = thr / 100.0f;

      // Torque: bell curve peaking at nRPM=0.5 (~4500 RPM)
      float curve = -4.0f * (nRPM - 0.5f) * (nRPM - 0.5f) + 1.0f;
      curve = constrain(curve, 0.1f, 1.0f);
      engineLUT[i][j].torque_Nm = 8.8f * curve * tFac + 0.5f * tFac;

      // CO: high at idle, falls as RPM/load rise (combustion efficiency improves)
      float coBase = 1.0f - nRPM * 0.55f;
      engineLUT[i][j].CO_ppm = 800.0f + 2500.0f * coBase * (1.0f - tFac * 0.4f);

      // CO2: load-driven
      engineLUT[i][j].CO2_pct = 5.0f + 9.0f * tFac;

      // NOx: high RPM + high throttle = high temp = high NOx
      engineLUT[i][j].NOx_ppm = (20.0f + 150.0f * tFac) * (0.5f + 1.5f * nRPM);
    }
  }
}

EngineOutput lookupEngine(float rpm, float thr) {
  rpm = constrain(rpm, 1000.0f, 8000.0f);
  thr = constrain(thr, 0.0f, 100.0f);

  int i1 = 0, i2 = 1, j1 = 0, j2 = 1;
  for (int i = 0; i < LUT_SIZE - 1; i++) {
    if (rpm >= icRPMAxis[i] && rpm <= icRPMAxis[i + 1]) { i1 = i; i2 = i + 1; break; }
  }
  for (int j = 0; j < LUT_SIZE - 1; j++) {
    if (thr >= throttleAxis[j] && thr <= throttleAxis[j + 1]) { j1 = j; j2 = j + 1; break; }
  }

  float dx = (rpm - icRPMAxis[i1])   / (icRPMAxis[i2]   - icRPMAxis[i1]);
  float dy = (thr - throttleAxis[j1]) / (throttleAxis[j2] - throttleAxis[j1]);

  EngineOutput Q11 = engineLUT[i1][j1], Q12 = engineLUT[i1][j2];
  EngineOutput Q21 = engineLUT[i2][j1], Q22 = engineLUT[i2][j2];

  EngineOutput out;
  out.torque_Nm = (1-dx)*(1-dy)*Q11.torque_Nm + (1-dx)*dy*Q12.torque_Nm
                + dx*(1-dy)*Q21.torque_Nm     + dx*dy*Q22.torque_Nm;
  out.CO_ppm    = (1-dx)*(1-dy)*Q11.CO_ppm    + (1-dx)*dy*Q12.CO_ppm
                + dx*(1-dy)*Q21.CO_ppm         + dx*dy*Q22.CO_ppm;
  out.CO2_pct   = (1-dx)*(1-dy)*Q11.CO2_pct   + (1-dx)*dy*Q12.CO2_pct
                + dx*(1-dy)*Q21.CO2_pct        + dx*dy*Q22.CO2_pct;
  out.NOx_ppm   = (1-dx)*(1-dy)*Q11.NOx_ppm   + (1-dx)*dy*Q12.NOx_ppm
                + dx*(1-dy)*Q21.NOx_ppm        + dx*dy*Q22.NOx_ppm;
  return out;
}

// ================================================================
// DALY BMS — NON-BLOCKING  (no blocked collection window)
//
// Query sequence (state machine, non-blocking):
//   State 0 → send wakeup 0x35C every BMS_QUERY_MS → State 1
//   State 1 → wait 100ms → send query 0x0400FF80   → State 2
//   State 2 → listen for 1500ms then back to State 0
//
// Frames decoded in receiveBMSFrame() called every loop():
//   0x04028001 (ext) PRIMARY  — V = BE_u16[0:2]×0.1, I = -(BE_u16[2:4]-30000)×0.1, SoC = BE_u16[4:6]×0.1
//   0x355      (std) SoC FALLBACK — LE_u16[0:2] = SoC %
//   0x04008001 (ext) VOLTAGE FALLBACK — cell sum ×0.001 V (deduplicated by group)
// ================================================================

static uint8_t  bmsQueryState  = 0;
static uint32_t bmsStateTimer  = 0;
static bool     bmsGroupSeen[16] = {false};
static uint32_t bmsCellSum_mV  = 0;
static uint8_t  bmsCellCount   = 0;

void sendBMSQuery() {
  unsigned long now = millis();

  if (bmsQueryState == 0 && now - bmsStateTimer >= BMS_QUERY_MS) {
    byte wake[8] = {0xC2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN.sendMsgBuf(DALY_WAKEUP_ID, 0, 8, wake);
    // reset cell fallback accumulators for new cycle
    memset(bmsGroupSeen, 0, sizeof(bmsGroupSeen));
    bmsCellSum_mV = 0;
    bmsCellCount  = 0;
    bmsStateTimer = now;
    bmsQueryState = 1;
  }
  else if (bmsQueryState == 1 && now - bmsStateTimer >= 100) {
    byte qry[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN.sendMsgBuf(DALY_QUERY_ID, 1, 8, qry);
    bmsQueryState = 2;
  }
  else if (bmsQueryState == 2 && now - bmsStateTimer >= 1600) {
    // listening window closed — use cell fallback voltage if primary never arrived
    if (!bmsValid && bmsCellCount > 0) {
      batVoltage = bmsCellSum_mV * 0.001f;
      batPower   = batVoltage * batCurrent;
      bmsValid   = true;
    }
    bmsStateTimer = now;
    bmsQueryState = 0;
  }
}

void receiveBMSFrame() {
  if (CAN.checkReceive() != CAN_MSGAVAIL) return;

  unsigned long rxId;
  byte len;
  byte buf[8];
  CAN.readMsgBuf(&rxId, &len, buf);

  uint32_t arbId = (uint32_t)(rxId & 0x1FFFFFFFUL);

  // PRIMARY: pack V / I / SoC — all three in one frame
  if (arbId == DALY_PACK_ID && len >= 6) {
    uint16_t rawV   = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t rawI   = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t rawSoC = ((uint16_t)buf[4] << 8) | buf[5];
    batVoltage = rawV  * 0.1f;
    batCurrent = -((int32_t)rawI - 30000) * 0.1f;   // positive = discharge
    batSoC     = rawSoC * 0.1f;
    batPower   = batVoltage * batCurrent;
    bmsValid   = true;
    return;
  }

  // SoC FALLBACK: 0x355 auto-broadcast — LE uint16 bytes[0:2] = SoC %
  if (arbId == DALY_SOC_ID && len >= 2) {
    uint16_t rawSoC = ((uint16_t)buf[1] << 8) | buf[0];
    batSoC = (float)rawSoC;
    if (bmsCellCount > 0) {
      batVoltage = bmsCellSum_mV * 0.001f;
      batPower   = batVoltage * batCurrent;
      bmsValid   = true;
    }
    return;
  }

  // VOLTAGE FALLBACK: 0x04008001 cell voltages — 3 per frame, deduplicated
  if (arbId == DALY_CELL_ID && len >= 7) {
    byte grp = buf[0];
    if (grp >= 1 && grp <= 16 && !bmsGroupSeen[grp - 1]) {
      bmsGroupSeen[grp - 1] = true;
      for (byte c = 0; c < 3; c++) {
        uint16_t cellmV = ((uint16_t)buf[1 + c * 2] << 8) | buf[2 + c * 2];
        if (cellmV > 0 && cellmV != 0xFFFF) {
          bmsCellSum_mV += cellmV;
          bmsCellCount++;
        }
      }
    }
  }
}

// ================================================================
// HYBRID STATE MACHINE
//
// EV_ONLY   : EV running. Relay A=OFF, Relay B=OFF, EV_CUT=OFF.
//             → TRANS_TO_IC  when hubRPM > evToIcRpm for EV_TO_IC_HOLD_MS.
//
// TRANS_TO_IC : Starting IC engine — EV stays running during crank.
//   entry      → Relay A=ON  (IC cranks, EV still running — no dead gap)
//   confirm    → icRPM >= IC_MIN_RUN_RPM for IC_START_CONFIRM_MS
//              → EV_CUT=ON (cut EV only after IC confirmed) → IC_ONLY
//   timeout    → IC_STARTUP_TIMEOUT_MS → Relay A=OFF → back to EV_ONLY
//              (EV_CUT was never set, EV never stopped)
//
// IC_ONLY   : IC driving. Relay A=ON, Relay B=OFF, EV_CUT=ON.
//             → TRANS_TO_EV  when icRPM < icLowRpm for icToEvHoldMs.
//
// TRANS_TO_EV : Handing back to EV — EV restored first, IC cut after.
//   entry       → EV_CUT=OFF (EV motor restores immediately — no dead gap)
//   t=+500ms    → Relay B=ON  (IC ignition cut)
//   t=+1500ms   → Relay A=OFF (IC power off)
//   t=+2000ms   → Relay B=OFF
//   t=+2500ms   → EV_ONLY
// ================================================================

void updateStateMachine() {
  unsigned long now     = millis();
  unsigned long elapsed = now - stateEntryTime;

  switch (hybridState) {

    // ── EV ONLY ──────────────────────────────────────────────────────────
    case EV_ONLY:
      if (hubRPM >= evToIcRpm) {
        if (!conditionArmed) {
          conditionTimer = now;
          conditionArmed = true;
        } else if (now - conditionTimer >= evToIcHoldMs) {
          conditionArmed = false;
          icConfirmArmed = false;
          stateEntryTime = now;
          hybridState    = TRANS_TO_IC;
          digitalWrite(RELAY_A_PIN, RELAY_ON);   // IC cranks; EV still running (EV_CUT=OFF)
        }
      } else {
        conditionArmed = false;
      }
      break;

    // ── TRANS_TO_IC ───────────────────────────────────────────────────────
    case TRANS_TO_IC:
      // IC confirm: icRPM must hold above IC_MIN_RUN_RPM for IC_START_CONFIRM_MS
      // EV KEY is never cut — hub motor regens freely while IC runs
      if (icRPM >= IC_MIN_RUN_RPM) {
        if (!icConfirmArmed) {
          icConfirmStart = now;
          icConfirmArmed = true;
        } else if (now - icConfirmStart >= IC_START_CONFIRM_MS) {
          conditionArmed = false;
          icConfirmArmed = false;
          stateEntryTime = now;
          hybridState    = IC_ONLY;
        }
      } else {
        icConfirmArmed = false;
      }

      // Timeout: IC didn't fire — abort
      if (elapsed >= IC_STARTUP_TIMEOUT_MS) {
        digitalWrite(RELAY_A_PIN, RELAY_OFF);
        conditionArmed = false;
        icConfirmArmed = false;
        stateEntryTime = now;
        hybridState    = EV_ONLY;
      }
      break;

    // ── IC ONLY ───────────────────────────────────────────────────────────
    case IC_ONLY:
      {
        // Hub motor regens freely — EV KEY always ON — no SSR toggling
        // IC→EV: IC RPM drops below icLowRpm (default 2500) for icToEvHoldMs
        if (icRPM < icLowRpm) {
          if (!conditionArmed) {
            conditionTimer = now;
            conditionArmed = true;
          }
          if (now - conditionTimer >= icToEvHoldMs) {
            conditionArmed = false;
            icConfirmArmed = false;
            stateEntryTime = now;
            hybridState    = TRANS_TO_EV;
          }
        } else {
          conditionArmed = false;
        }
      }
      break;

    // ── TRANS_TO_EV ───────────────────────────────────────────────────────
    case TRANS_TO_EV:
      if (elapsed >= TEV_RELAY_B_ON)  digitalWrite(RELAY_B_PIN, RELAY_ON);   // IC ignition cut
      if (elapsed >= TEV_RELAY_A_OFF) digitalWrite(RELAY_A_PIN, RELAY_OFF);  // IC power off
      if (elapsed >= TEV_RELAY_B_OFF) digitalWrite(RELAY_B_PIN, RELAY_OFF);
      if (elapsed >= TEV_TO_EV_ONLY) {
        conditionArmed = false;
        stateEntryTime = now;
        hybridState    = EV_ONLY;
      }
      break;
  }
}

// ================================================================
// SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);   // TX3 Pin14 → ESP32-S3 RX (via 1kΩ+2.2kΩ divider)

  // Relays A/B: write HIGH before pinMode → no LOW glitch on boot
  digitalWrite(RELAY_A_PIN, RELAY_OFF);
  digitalWrite(RELAY_B_PIN, RELAY_OFF);
  pinMode(RELAY_A_PIN, OUTPUT);
  pinMode(RELAY_B_PIN, OUTPUT);

  // EV cut SSR: HIGH before pinMode → SSR conducts → KEY live → EV ON at boot
  // 10kΩ pull-up (Pin6→5V) holds this HIGH during Arduino reset for safety
  digitalWrite(EV_CUT_PIN, EV_CUT_OFF);
  pinMode(EV_CUT_PIN, OUTPUT);

  pinMode(HALL_PIN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_A), hallPulse, RISING);

  pinMode(IC_RPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IC_RPM_PIN), icPulse, RISING);

  pinMode(TPS_PIN, INPUT);

  initializeLUT();

  delay(300);

  // --- CAN / BMS init ---
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    Serial.println(F("  MCP2515 CAN     : OK  (500 kbps, 8 MHz)"));
  } else {
    Serial.println(F("  MCP2515 CAN     : FAIL — check wiring/CS pin"));
  }

  Serial.println(F("============================================"));
  Serial.println(F("  ReynLab — Sensor Diagnostic"));
  Serial.println(F("  Hybrid Activa | Arduino Mega 2560"));
  Serial.println(F("============================================"));
  Serial.println(F("  Hub Motor Hall  : Pin 2  (INT4)"));
  Serial.println(F("  IC Engine RPM   : Pin 3  (INT5)"));
  Serial.println(F("  TPS             : Pin A0"));
  Serial.println(F("  Relay A (IC Pwr) : Pin 7"));
  Serial.println(F("  Relay B (IC IGN) : Pin 8"));
  Serial.println(F("  EV Key SSR      : Pin 6  (FRFER SSR-10DD, HIGH=EV ON, LOW=EV OFF)"));
  Serial.println(F("  Daly BMS (CAN)  : MCP2515 CS=Pin10, 500kbps"));
  Serial.print  (F("  PPR (verified)  : ")); Serial.println(PULSES_PER_REV);
  Serial.print  (F("  Debounce        : ")); Serial.print(HUB_DEBOUNCE_US);   Serial.println(F(" us  (max ~7143 RPM)"));
  Serial.print  (F("  Update window   : ")); Serial.print(HUB_UPDATE_MS);     Serial.println(F(" ms"));
  Serial.print  (F("  Filter alpha    : ")); Serial.println(HUB_ALPHA, 2);
  Serial.print  (F("  Timeout         : ")); Serial.print(HUB_TIMEOUT_US);    Serial.println(F(" us"));
  Serial.print  (F("  IC debounce     : ")); Serial.print(IC_DEBOUNCE_US);    Serial.println(F(" us"));
  Serial.print  (F("  TPS 0%%         : ")); Serial.print(TPS_MIN_V, 2);      Serial.println(F("V"));
  Serial.print  (F("  TPS 100%%       : ")); Serial.print(TPS_MAX_V, 2);      Serial.println(F("V"));
  Serial.println(F("============================================"));
  Serial.println();
  Serial.println(F("  PULSES | HUB RPM  | IC RPM   | TPS V  | Throttle% | Bat V  | Bat I  | SoC%  | Bat W    | STATE"));
  Serial.println(F("  ------   --------   --------   ------   ---------   ------   ------   -----   -----     -------"));

  lastPrintTime = millis();
}

// ================================================================
// LOOP
// ================================================================

void loop() {
  updateHubRPM();
  updateIcRPM();
  sendBMSQuery();
  receiveBMSFrame();
  updateStateMachine();

  unsigned long now     = millis();
  unsigned long elapsed = now - lastPrintTime;

  if (elapsed >= PRINT_INTERVAL_MS) {

    // --- TPS (updates global throttlePct) ---
    long adcSum = 0;
    for (int i = 0; i < TPS_SAMPLES; i++) {
      adcSum += analogRead(TPS_PIN);
      delayMicroseconds(200);
    }
    float rawVoltage = (adcSum / (float)TPS_SAMPLES / 1023.0f) * 5.0f;

    tpsHistory[tpsIndex] = rawVoltage;
    tpsIndex = (tpsIndex + 1) % TPS_AVG_WINDOW;
    float avgVoltage = 0.0f;
    for (int i = 0; i < TPS_AVG_WINDOW; i++) avgVoltage += tpsHistory[i];
    avgVoltage /= TPS_AVG_WINDOW;

    float voltage = tpsStable;
    if (fabsf(avgVoltage - tpsStable) > TPS_DEADBAND_V) {
      voltage   = avgVoltage;
      tpsStable = avgVoltage;
    }

    throttlePct = 0.0f;
    if      (voltage <= TPS_MIN_V) throttlePct = 0.0f;
    else if (voltage >= TPS_MAX_V) throttlePct = 100.0f;
    else throttlePct = ((voltage - TPS_MIN_V) / (TPS_MAX_V - TPS_MIN_V)) * 100.0f;

    // --- Print ---
    char buf[12];

    Serial.print(F("  "));
    sprintf(buf, "%-11lu", (unsigned long)hubTotalPulses);
    Serial.print(buf);
    Serial.print(F("  | "));

    dtostrf(hubRPM, 7, 1, buf);
    Serial.print(buf);
    Serial.print(F("  | "));

    dtostrf(icRPM, 7, 1, buf);
    Serial.print(buf);
    Serial.print(F("  |  "));

    dtostrf(voltage, 5, 3, buf);
    Serial.print(buf);
    Serial.print(F("V  |  "));

    dtostrf(throttlePct, 5, 1, buf);
    Serial.print(buf);
    Serial.print(F("%  |  "));

    if (bmsValid) {
      dtostrf(batVoltage, 5, 1, buf);
      Serial.print(buf);
      Serial.print(F("V  |  "));

      dtostrf(batCurrent, 5, 1, buf);
      Serial.print(buf);
      Serial.print(F("A  |  "));

      dtostrf(batSoC, 4, 1, buf);
      Serial.print(buf);
      Serial.print(F("%  |  "));

      dtostrf(batPower, 6, 1, buf);
      Serial.print(buf);
      Serial.print(F("W  | "));
    } else {
      Serial.print(F("BMS: waiting...          | "));
    }

    // --- Hybrid state ---
    switch (hybridState) {
      case EV_ONLY:     Serial.println(F("EV_ONLY  ")); break;
      case TRANS_TO_IC: Serial.println(F("TO_IC    ")); break;
      case IC_ONLY:     Serial.println(F("IC_ONLY  ")); break;
      case TRANS_TO_EV: Serial.println(F("TO_EV    ")); break;
    }

    // --- Engine emissions (IC_ONLY only) ---
    if (hybridState == IC_ONLY && icRPM >= 1000.0f) {
      EngineOutput eng = lookupEngine(icRPM, throttlePct);
      Serial.print(F("  IC> "));
      dtostrf(icRPM, 6, 0, buf); Serial.print(buf); Serial.print(F(" RPM"));
      Serial.print(F("  Torque: ")); dtostrf(eng.torque_Nm, 4, 1, buf); Serial.print(buf); Serial.print(F(" Nm"));
      Serial.print(F("  CO: "));    dtostrf(eng.CO_ppm,    5, 0, buf); Serial.print(buf); Serial.print(F(" ppm"));
      Serial.print(F("  CO2: "));   dtostrf(eng.CO2_pct,   4, 1, buf); Serial.print(buf); Serial.print(F("%"));
      Serial.print(F("  NOx: "));   dtostrf(eng.NOx_ppm,   5, 0, buf); Serial.print(buf); Serial.println(F(" ppm"));
    }

    // --- JSON output on Serial3 → ESP32-S3 dashboard ---
    Serial3.print(F("{\"hubRPM\":"));  Serial3.print(hubRPM, 1);
    Serial3.print(F(",\"icRPM\":"));   Serial3.print(icRPM, 1);
    Serial3.print(F(",\"thr\":"));     Serial3.print(throttlePct, 1);
    Serial3.print(F(",\"batV\":"));    Serial3.print(bmsValid ? batVoltage : 0.0f, 1);
    Serial3.print(F(",\"batI\":"));    Serial3.print(bmsValid ? batCurrent : 0.0f, 1);
    Serial3.print(F(",\"soc\":"));     Serial3.print(bmsValid ? batSoC     : 0.0f, 1);
    Serial3.print(F(",\"batW\":"));    Serial3.print(bmsValid ? batPower   : 0.0f, 1);
    Serial3.print(F(",\"bms\":"));     Serial3.print(bmsValid ? 1 : 0);
    Serial3.print(F(",\"evToIcRpm\":"));    Serial3.print(evToIcRpm, 0);
    Serial3.print(F(",\"evToIcHoldMs\":")); Serial3.print(evToIcHoldMs);
    Serial3.print(F(",\"icToEvHoldMs\":")); Serial3.print(icToEvHoldMs);
    Serial3.print(F(",\"icLowRpm\":"));    Serial3.print(icLowRpm, 0);
    Serial3.print(F(",\"state\":\""));
    switch (hybridState) {
      case EV_ONLY:     Serial3.print(F("EV_ONLY"));     break;
      case TRANS_TO_IC: Serial3.print(F("TRANS_TO_IC")); break;
      case IC_ONLY:     Serial3.print(F("IC_ONLY"));     break;
      case TRANS_TO_EV: Serial3.print(F("TRANS_TO_EV")); break;
    }
    Serial3.println(F("\"}"));

    lastPrintTime = now;
  }

  // ── Serial3 RX: commands from ESP32 dashboard ──────────────────
  static String rxBuf;
  while (Serial3.available()) {
    char c = (char)Serial3.read();
    if (c == '\n') {
      rxBuf.trim();
      if (rxBuf.startsWith("SET_RPM:")) {
        float v = rxBuf.substring(8).toFloat();
        if (v >= 500.0f && v <= 5000.0f) evToIcRpm = v;
      } else if (rxBuf.startsWith("SET_EV_HOLD:")) {
        unsigned long v = (unsigned long)rxBuf.substring(12).toInt();
        if (v >= 1000UL && v <= 120000UL) evToIcHoldMs = v;
      } else if (rxBuf.startsWith("SET_IC_HOLD:")) {
        unsigned long v = (unsigned long)rxBuf.substring(12).toInt();
        if (v >= 1000UL && v <= 120000UL) icToEvHoldMs = v;
      } else if (rxBuf.startsWith("SET_IC_RPM:")) {
        float v = rxBuf.substring(11).toFloat();
        if (v >= 1000.0f && v <= 6000.0f) icLowRpm = v;
      }
      rxBuf = "";
    } else if (c != '\r' && rxBuf.length() < 32) {
      rxBuf += c;
    }
  }
}
