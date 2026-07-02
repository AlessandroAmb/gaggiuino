/* 09:32 15/03/2023 - change triggering comment */
#include "pump.h"
#include "pindef.h"
#include <PSM.h>
#include "utils.h"
#include "internal_watchdog.h"

PSM pump(zcPin, dimmerPin, PUMP_RANGE, ZC_MODE, 1, 6);

// PUMP CALIBRATION & CHARACTERIZATION
float flowPerClickAtZeroBar = 0.27f;  // ml/s at zero bar - hardware specific
int maxPumpClicksPerSecond = 50;      // Power line frequency dependent
float fpc_multiplier = 1.2f;          // Calculated from frequency: 60/frequency

// PUMP PRESSURE REGULATION TUNING
// Thresholds and gains for multi-tier pressure control algorithm
const float PRESSURE_LARGE_ERROR_THRESHOLD = 2.5f;   // Error > this = aggressive ramp
const float PRESSURE_LARGE_ERROR_BASE = 0.16f;       // Base output for large errors
const float PRESSURE_LARGE_ERROR_MULTIPLIER = 0.12f; // Scaling factor per bar of error

const float PRESSURE_MODERATE_ERROR_THRESHOLD = 0.5f;  // Error between 0.5-2.5
const float PRESSURE_MODERATE_PROPORTIONAL_GAIN = 0.14f;  // Proportional control gain
const float PRESSURE_MODERATE_BASEFLOW_FACTOR = 0.90f;    // Slight backoff to prevent overshoot

const float PRESSURE_SMALL_ERROR_THRESHOLD = 0.0f;    // Error between 0-0.5
const float PRESSURE_SMALL_DERIVATIVE_GAIN = 0.07f;   // How much to dampen based on pressure rate
const float PRESSURE_SMALL_PROPORTIONAL_GAIN = 0.06f;  // Minimal proportional adjustment
const float PRESSURE_SMALL_FLOWMAINT_FACTOR = 0.98f;   // Maintain most of current flow

const float PRESSURE_SLIGHT_OVERSHOOT_THRESHOLD = -0.3f;  // Slight overshoot: -0.3 to 0
const float PRESSURE_OVERSHOOT_BACKOFF_FACTOR = 0.08f;    // How much to back off
uint8_t currentPumpValue = 0;  // Track current pump value for smooth ramping
uint8_t targetPumpValue = 0;   // Target pump value
unsigned long lastPumpUpdateTime = 0;
constexpr uint16_t PUMP_RAMP_TIME_MS = 50;  // Time to ramp between values (ms)
constexpr uint8_t PUMP_MAX_STEP = 2;  // Max step size per ramp cycle to ensure smooth transitions
//https://www.desmos.com/calculator/axyl70gjae  - blue curve
constexpr std::array<float, 7> pressureInefficiencyCoefficient {{
  0.045f,
  0.015f,
  0.0033f,
  0.000685f,
  0.000045f,
  0.009f,
  -0.0018f
}};

// Initialising some pump specific specs, mainly:
// - max pump clicks(dependant on region power grid spec)
// - pump clicks at 0 pressure in the system
void pumpInit(const int powerLineFrequency, const float pumpFlowAtZero) {
  // Guard against invalid frequency detection that could cause division by zero
  maxPumpClicksPerSecond = (powerLineFrequency > 0) ? powerLineFrequency : 50u;  // Default to 50 Hz if invalid
  flowPerClickAtZeroBar = pumpFlowAtZero;
  fpc_multiplier = 60.f / (float)maxPumpClicksPerSecond;
}

// Static variables for improved pressure regulation
static float previousPressureError = 0.f;

// Function that returns the percentage of clicks the pump makes in it's current phase
// Improved algorithm with better pressure stability and overshoot prevention
inline float getPumpPct(const float targetPressure, const float flowRestriction, const SensorState &currentState) {
  if (targetPressure == 0.f) {
      previousPressureError = 0.f;
      return 0.f;
  }

  float pressureError = targetPressure - currentState.smoothedPressure;  // How far below target
  float pressureErrorRate = pressureError - previousPressureError;
  float maxPumpPct = flowRestriction <= 0.f ? 1.f : getClicksPerSecondForFlow(flowRestriction, currentState.smoothedPressure) / (float) maxPumpClicksPerSecond;
  float pumpPctToMaintainFlow = getClicksPerSecondForFlow(currentState.smoothedPumpFlow, currentState.smoothedPressure) / (float) maxPumpClicksPerSecond;
  float steadyStateBoost = 0.f;
  if (pressureError > 0.15f && pressureError < 1.5f && fabsf(currentState.pressureChangeSpeed) < 0.08f) {
    // When pressure stalls below target, add a small kick to overcome static losses.
    steadyStateBoost = 0.03f + (0.03f * pressureError);
  }

  float pumpOutput = 0.f;

  // IMPROVED PRESSURE REGULATION ALGORITHM
  // Uses proportional + derivative control for smooth, stable regulation
  
  if (pressureError > PRESSURE_LARGE_ERROR_THRESHOLD) {
    // Large undershoot: aggressive ramp-up to reach target quickly
    // Scale up gradually with pressure deficit
    float derivativeDamping = pressureErrorRate < 0.f ? 0.03f * (-pressureErrorRate) : 0.f;
    float pressureDrivenOutput = PRESSURE_LARGE_ERROR_BASE + PRESSURE_LARGE_ERROR_MULTIPLIER * pressureError - derivativeDamping;
    pumpOutput = fminf(maxPumpPct * 0.95f, pressureDrivenOutput);
  }
  else if (pressureError > PRESSURE_MODERATE_ERROR_THRESHOLD) {
    // Moderate undershoot: smooth proportional control
    // Maintain current flow + proportional boost based on error
    float baseFlow = pumpPctToMaintainFlow * PRESSURE_MODERATE_BASEFLOW_FACTOR;
    float derivativeDamping = pressureErrorRate < 0.f ? 0.02f * (-pressureErrorRate) : 0.f;
    pumpOutput = fminf(maxPumpPct, baseFlow + (PRESSURE_MODERATE_PROPORTIONAL_GAIN * pressureError) - derivativeDamping + steadyStateBoost);
  }
  else if (pressureError > PRESSURE_SMALL_ERROR_THRESHOLD) {
    // Small undershoot: minimal adjustment, mostly maintain current flow
    // Derivative control: reduce if pressure rising quickly
    float pressureVelocity = currentState.pressureChangeSpeed;
    float flowMaintenance = pumpPctToMaintainFlow * PRESSURE_SMALL_FLOWMAINT_FACTOR;
    float adjustment = (PRESSURE_SMALL_PROPORTIONAL_GAIN * pressureError) - (PRESSURE_SMALL_DERIVATIVE_GAIN * pressureVelocity);
    pumpOutput = fminf(maxPumpPct, flowMaintenance + adjustment + steadyStateBoost);
  }
  else if (pressureError > PRESSURE_SLIGHT_OVERSHOOT_THRESHOLD) {
    // Slight overshoot: back off significantly to prevent further rise
    pumpOutput = fminf(maxPumpPct, pumpPctToMaintainFlow * PRESSURE_OVERSHOOT_BACKOFF_FACTOR);
  }
  else {
    // Significant overshoot: turn off or run minimal to let pressure drop
    pumpOutput = 0.f;
  }

  previousPressureError = pressureError;
  return fmaxf(0.f, pumpOutput);  // Clamp to non-negative
}

// Sets the pump output based on a couple input params:
// - live system pressure
// - expected target
// - flow
// - pressure direction
void setPumpPressure(const float targetPressure, const float flowRestriction, const SensorState &currentState) {
  float pumpPct = getPumpPct(targetPressure, flowRestriction, currentState);
  uint8_t targetValue = (uint8_t)(pumpPct * PUMP_RANGE);
  
#if PUMP_SMOOTH_MODE
  // Smooth phase modulation: apply gentle ramping for stable pressure profiles
  // This creates smooth pressure curves without clicks/spikes but slightly slower response
  unsigned long currentTime = millis();
  if (currentTime - lastPumpUpdateTime >= PUMP_RAMP_TIME_MS) {
    lastPumpUpdateTime = currentTime;
    
    int difference = (int)targetValue - (int)currentPumpValue;
    if (difference > PUMP_MAX_STEP) {
      currentPumpValue += PUMP_MAX_STEP;
    } else if (difference < -PUMP_MAX_STEP) {
      currentPumpValue -= PUMP_MAX_STEP;
    } else {
      currentPumpValue = targetValue;
    }
  }
  targetPumpValue = targetValue;
  pump.set(currentPumpValue);
#else
  // Direct click mode: immediate response for fast pressure control (original behavior)
  // Responds instantly to pressure changes but may cause more clicks/spikes
  setPumpToRawValue(targetValue);
#endif
}

void setPumpOff(void) {
  pump.set(0);
}

void setPumpFullOn(void) {
  pump.set(PUMP_RANGE);
}

void setPumpToRawValue(const uint8_t val) {
  pump.set(val);
  currentPumpValue = val;
  targetPumpValue = val;
}

// Smooth manual dimmer control for manual mode - direct mapping without complex flow calculations
// This provides smooth, evenly-distributed pump control without pressure fluctuations
void setPumpManualDimmer(const uint8_t pumpPercent) {
  // Directly set pump to the requested percentage with PSM's smooth phase-shift modulation
  // The PSM library handles the AC cycle distribution internally
  uint8_t constrainedValue = constrain(pumpPercent, 0, PUMP_RANGE);
  
  // For manual mode, we want immediate response but smooth transitions
  // Apply gentle ramping to avoid sudden changes that could cause pressure spikes
  unsigned long currentTime = millis();
  if (currentTime - lastPumpUpdateTime >= PUMP_RAMP_TIME_MS) {
    lastPumpUpdateTime = currentTime;
    
    int difference = (int)constrainedValue - (int)currentPumpValue;
    if (difference > PUMP_MAX_STEP) {
      currentPumpValue += PUMP_MAX_STEP;
    } else if (difference < -PUMP_MAX_STEP) {
      currentPumpValue -= PUMP_MAX_STEP;
    } else {
      currentPumpValue = constrainedValue;
    }
  }
  
  targetPumpValue = constrainedValue;
  pump.set(currentPumpValue);
}

void pumpStopAfter(const uint8_t val) {
  pump.stopAfter(val);
}

long getAndResetClickCounter(void) {
  long counter = pump.getCounter();
  pump.resetCounter();
  return counter;
}

int getCPS(void) {
  watchdogReload();
  unsigned int cps = pump.cps();
  watchdogReload();
  if (cps > 80u) {
    pump.setDivider(2);
    pump.initTimer(cps > 110u ? 5000u : 6000u, TIM9);
  }
  else {
    pump.initTimer(cps > 55u ? 5000u : 6000u, TIM9);
  }
  return cps;
}

void pumpPhaseShift(void) {
  pump.shiftDividerCounter();
}

// Models the flow per click, follows a compromise between the schematic and recorded findings
// plotted: https://www.desmos.com/calculator/eqynzclagu
float getPumpFlowPerClick(const float pressure) {
  // float fpc = 0.f;
  // fpc = (pressureInefficiencyCoefficient[5] / pressure + pressureInefficiencyCoefficient[6]) * ( -pressure * pressure ) + ( flowPerClickAtZeroBar - pressureInefficiencyCoefficient[0]) - (pressureInefficiencyCoefficient[1] + (pressureInefficiencyCoefficient[2] - (pressureInefficiencyCoefficient[3] - pressureInefficiencyCoefficient[4] * pressure) * pressure) * pressure) * pressure;
  // Faaster: no division, less multiply
  float fpc =
    (((-0.000054f * pressure
      + 0.000822f) * pressure
      - 0.0018f) * pressure
      - 0.0288f) * pressure
      + 0.27f;
return fpc;
  //return fpc * fpc_multiplier;
}

// Follows the schematic from https://www.cemegroup.com/solenoid-pump/e5-60 modified to per-click
float getPumpFlow(const float cps, const float pressure) {
  return cps * getPumpFlowPerClick(pressure);
}

// Currently there is no compensation for pressure measured at the puck, resulting in incorrect estimates
float getClicksPerSecondForFlow(const float flow, const float pressure) {
  if (flow == 0.f) return 0;
  float flowPerClick = getPumpFlowPerClick(pressure);
  float cps = flow / flowPerClick;
  return fminf(cps, (float)maxPumpClicksPerSecond);
}

// Calculates pump percentage for the requested flow and updates the pump raw value
void setPumpFlow(const float targetFlow, const float pressureRestriction, const SensorState &currentState) {
  // If a pressure restriction exists then the we go into pressure profile with a flowRestriction
  // which is equivalent but will achieve smoother pressure management
  if (pressureRestriction > 0.f && currentState.smoothedPressure > pressureRestriction * 0.5f) {
    setPumpPressure(pressureRestriction, targetFlow, currentState);
  }
  else {
    float pumpPct = getClicksPerSecondForFlow(targetFlow, currentState.smoothedPressure) / (float)maxPumpClicksPerSecond;
    uint8_t targetValue = (uint8_t)(pumpPct * PUMP_RANGE);
    
#if PUMP_SMOOTH_MODE
    // Smooth phase modulation: apply ramping for flow-based control
    unsigned long currentTime = millis();
    if (currentTime - lastPumpUpdateTime >= PUMP_RAMP_TIME_MS) {
      lastPumpUpdateTime = currentTime;
      
      int difference = (int)targetValue - (int)currentPumpValue;
      if (difference > PUMP_MAX_STEP) {
        currentPumpValue += PUMP_MAX_STEP;
      } else if (difference < -PUMP_MAX_STEP) {
        currentPumpValue -= PUMP_MAX_STEP;
      } else {
        currentPumpValue = targetValue;
      }
    }
    targetPumpValue = targetValue;
    pump.set(currentPumpValue);
#else
    // Direct click mode: immediate response
    setPumpToRawValue(targetValue);
#endif
  }
}
