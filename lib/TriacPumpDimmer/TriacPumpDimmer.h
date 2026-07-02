#ifndef TRIAC_PUMP_DIMMER_H
#define TRIAC_PUMP_DIMMER_H

#include <Arduino.h>

class HardwareTimer;

class TriacPumpDimmer {
public:
  enum class ControlMode : uint8_t {
    PulseSkip,
    PhaseAngle,
  };

  TriacPumpDimmer(uint8_t zeroCrossPin, uint8_t gatePin, int zeroCrossMode);

  void begin();
  void setPulseSkip(uint8_t value, uint8_t range);
  void setPhaseAngleLinearized(uint8_t value, uint8_t range);
  void setOff();
  void setFullOn();
  long getCounter();
  void resetCounter();
  int cps(uint32_t timeoutMs = 250);
  void stopAfter(uint8_t halfCycles);
  void shiftDividerCounter();

private:
  static constexpr uint16_t DEFAULT_HALF_PERIOD_US = 10000;
  static constexpr uint16_t MIN_HALF_PERIOD_US = 7000;
  static constexpr uint16_t MAX_HALF_PERIOD_US = 12000;
  static constexpr uint16_t GATE_PULSE_US = 120;
  static constexpr uint16_t FULL_ON_PULSE_DELAY_US = 100;
  static constexpr uint8_t MAX_SAMPLE_COUNT = 16;

  static TriacPumpDimmer *instance_;

  uint8_t zeroCrossPin_;
  uint8_t gatePin_;
  int zeroCrossMode_;
  HardwareTimer *timer_;
  bool initialized_;
  bool zeroCrossInterruptEnabled_;

  volatile ControlMode mode_;
  volatile uint8_t targetValue_;
  volatile uint8_t valueRange_;
  volatile uint16_t targetDelayUs_;
  volatile uint16_t halfPeriodUs_;
  volatile uint32_t lastZeroCrossUs_;
  volatile uint32_t halfPeriodAccumulatorUs_;
  volatile uint8_t halfPeriodSamples_;
  volatile long deliveredHalfCycles_;
  volatile uint16_t skipAccumulator_;
  volatile uint16_t phaseAccumulator_;
  volatile int32_t remainingHalfCycles_;
  volatile bool timerGateOnPhase_;

  void ensureInitialized();
  void enableZeroCrossInterrupt();
  void disableZeroCrossInterrupt();
  void scheduleTimer(uint16_t delayUs);
  uint16_t computeLinearizedDelayUs(uint8_t value, uint8_t range) const;
  static float computeNormalizedPower(float firingAngleRad);

  void onZeroCross();
  void onTimer();

  static void handleZeroCross();
  static void handleTimer();
};

#endif