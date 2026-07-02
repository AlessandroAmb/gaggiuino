/* 09:32 15/03/2023 - change triggering comment */
#include "just_do_coffee.h"
#include "../lcd/lcd.h"

extern unsigned long steamTime;

// TEMPERATURE CONTROL TUNING PARAMETERS
// Proportional gains for PID-like control
const float TEMP_PROPORTIONAL_GAIN_BREW = 0.5f;   // Brew mode: conservative to avoid overshoot
const float TEMP_PROPORTIONAL_GAIN_STEAM = 0.8f;  // Steam mode: more aggressive
// Feedforward control: compensates for cooling during extraction
const float TEMP_FEEDFORWARD_GAIN = 1.2f;         // How much to anticipate cooling
const float TEMP_COOLING_COEFFICIENT = 0.8f;      // °C cooling per ml/s flow
// Derivative control: dampens overshoot
const float TEMP_DERIVATIVE_GAIN = 0.2f;          // How much to dampen rapid changes

// Tracking for improved boiler control
static float previousTempError = 0.f;
static constexpr uint32_t BOILER_MODULATION_TICK_MS = 200; // 5 Hz boiler power update cadence
static constexpr float BOILER_FULL_POWER_WARMUP_DELTA = 20.f;

// SIMPLIFIED & CORRECTED FLUX ESTIMATION
// Returns estimated water flow in ml/s, corrected for the 2x overestimation in original code
inline static float getEstimatedFlux(const SensorState &currentState) {
  // Use smoothedPumpFlow directly (already in ml/s) but divided by 2 to correct overestimation
  return currentState.smoothedPumpFlow * 0.5f;
}

// UNIFIED TEMPERATURE REGULATOR with feedforward + proportional control
// Handles both brew and steam modes with adaptive parameters
inline static float calculateBoilerPower(
  const float targetTemp,
  const float currentTemp,
  const float flux,                    // Estimated water flow in ml/s (corrected)
  const bool brewActive,
  const bool isSteamMode
) {
  float tempError = targetTemp - currentTemp;  // How far below target
  
  // Adaptive gains based on mode
  float proportionalGain = isSteamMode ? TEMP_PROPORTIONAL_GAIN_STEAM : TEMP_PROPORTIONAL_GAIN_BREW;
  float feedforwardGain = brewActive && !isSteamMode ? TEMP_FEEDFORWARD_GAIN : 0.f;  // Only feedforward during brew
  float derivativeGain = TEMP_DERIVATIVE_GAIN;
  
  // Base power from proportional control
  float proportionalPower = proportionalGain * tempError;
  
  // Feedforward boost: higher flow = more cooling, need more heat
  // feedforwardGain coefficient × flux × cooling_coefficient = power needed
  float feedforwardPower = feedforwardGain * flux * TEMP_COOLING_COEFFICIENT;
  
  // Derivative damping: reduce if temp rising quickly
  float derivativePower = -derivativeGain * (tempError - previousTempError);
  previousTempError = tempError;
  
  // Combined power output (0-1 = 0-100%)
  float boilerPower = proportionalPower + feedforwardPower + derivativePower;
  
  return constrain(boilerPower, 0.f, 1.f);
}

// Single actuator layer for heater output.
// Distributes ON cycles evenly over time (sigma-delta), avoiding a hidden second regulator.
void applyBoilerPower(const float boilerPower) {
  static float cycleAccumulator = 0.f;
  static uint32_t lastModulationTick = 0;

  const float clampedPower = constrain(boilerPower, 0.f, 1.f);

  if (clampedPower <= 0.f) {
    setBoilerOff();
    cycleAccumulator = 0.f;
    return;
  }

  if (clampedPower >= 1.f) {
    setBoilerOn();
    cycleAccumulator = 0.f;
    return;
  }

  const uint32_t now = millis();
  if ((uint32_t)(now - lastModulationTick) < BOILER_MODULATION_TICK_MS) {
    return;
  }
  lastModulationTick = now;

  cycleAccumulator += clampedPower;
  if (cycleAccumulator >= 1.f) {
    setBoilerOn();
    cycleAccumulator -= 1.f;
  } else {
    setBoilerOff();
  }
}

void justDoCoffee(const eepromValues_t &runningCfg, const SensorState &currentState, const bool brewActive) {
  lcdTargetState((int)HEATING::MODE_brew); // setting the target mode to "brew temp"
  float brewTempSetPoint = ACTIVE_PROFILE(runningCfg).setpoint;
  float sensorTemperature = currentState.temperature;
  float tempError = brewTempSetPoint - sensorTemperature;
  float estimatedFlux = getEstimatedFlux(currentState);

  if (brewActive) {
    if (tempError >= BOILER_FULL_POWER_WARMUP_DELTA) {
      applyBoilerPower(1.f);
    } else {
    // During brew: use feedforward control to anticipate cooling from water flow
    float boilerPower = calculateBoilerPower(
      brewTempSetPoint,
      sensorTemperature,
      estimatedFlux,
      true,   // brewActive
      false   // not steam mode
    );
    
    applyBoilerPower(boilerPower);
    }
  } else {
    if (tempError >= BOILER_FULL_POWER_WARMUP_DELTA) {
      applyBoilerPower(1.f);
    } else {
    // Standby mode: maintain temperature with lower power range
    float boilerPower = calculateBoilerPower(
      brewTempSetPoint,
      sensorTemperature,
      0.f,    // No flux during standby
      false,  // Not brewing
      false   // Not steam mode
    );
    
    // Keep standby softer by scaling commanded power with configured divider.
    const float standbyScale = runningCfg.mainDivider > 0 ? 1.f / runningCfg.mainDivider : 1.f;
    applyBoilerPower(boilerPower * standbyScale);
    }
  }

  if (brewActive || !currentState.brewSwitchState) {
    setSteamValveRelayOff();
  }
  setSteamBoilerRelayOff();
}

//#############################################################################################
//################################____STEAM_POWER_CONTROL____##################################
//#############################################################################################
void steamCtrl(const eepromValues_t &runningCfg, SensorState &currentState) {
  currentState.steamSwitchState ? lcdTargetState((int)HEATING::MODE_steam) : lcdTargetState((int)HEATING::MODE_brew);
  float steamTempSetPoint = runningCfg.steamSetPoint;
  float sensorTemperature = currentState.temperature;

  // Aggressive pressure/temperature control for stable steam generation
  if (currentState.smoothedPressure > steamThreshold_ || sensorTemperature > steamTempSetPoint) {
    // Pressure too high or temperature too high: turn off heating
    setBoilerOff();
    setSteamBoilerRelayOff();
    setSteamValveRelayOff();
    setPumpOff();
  } else {
    // Use unified regulator with more aggressive steam gains
    float boilerPower = calculateBoilerPower(
      steamTempSetPoint,
      sensorTemperature,
      0.f,    // No feedforward for steam
      false,  // Not brewing
      true    // isSteamMode - triggers higher proportional gain
    );
    
    applyBoilerPower(boilerPower);
    
    setSteamValveRelayOn();
    setSteamBoilerRelayOn();
    
    #ifndef DREAM_STEAM_DISABLED
      if (currentState.smoothedPressure < activeSteamPressure_) {
        setPumpToRawValue(3);
      } else {
        setPumpOff();
      }
    #endif
  }

  /*In case steam is forgotten ON for more than 15 min*/
  if (currentState.smoothedPressure > passiveSteamPressure_) {
    currentState.isSteamForgottenON = millis() - steamTime >= STEAM_TIMEOUT;
  } else steamTime = millis();
}

/*Water mode and all that*/
void hotWaterMode(const SensorState &currentState) {
  closeValve();
  setPumpToRawValue(80);
  // Simple on/off control for hot water - use unified regulator with brew gains
  float boilerPower = calculateBoilerPower(
    MAX_WATER_TEMP,
    currentState.temperature,
    0.f,
    false,
    false
  );
  applyBoilerPower(boilerPower);
}
