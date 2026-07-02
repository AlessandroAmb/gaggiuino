#include "TriacPumpDimmer.h"

#include <HardwareTimer.h>
#include <math.h>

TriacPumpDimmer *TriacPumpDimmer::instance_ = nullptr;

TriacPumpDimmer::TriacPumpDimmer(uint8_t zeroCrossPin, uint8_t gatePin, int zeroCrossMode)
  : zeroCrossPin_(zeroCrossPin),
    gatePin_(gatePin),
    zeroCrossMode_(zeroCrossMode),
    timer_(nullptr),
    initialized_(false),
    zeroCrossInterruptEnabled_(false),
    mode_(ControlMode::PulseSkip),
    targetValue_(0),
    valueRange_(100),
    targetDelayUs_(DEFAULT_HALF_PERIOD_US),
    halfPeriodUs_(DEFAULT_HALF_PERIOD_US),
    lastZeroCrossUs_(0),
    halfPeriodAccumulatorUs_(0),
    halfPeriodSamples_(0),
    deliveredHalfCycles_(0),
    skipAccumulator_(0),
    phaseAccumulator_(0),
    remainingHalfCycles_(-1),
    timerGateOnPhase_(false) {}

void TriacPumpDimmer::begin() {
  ensureInitialized();
}

void TriacPumpDimmer::setPulseSkip(uint8_t value, uint8_t range) {
  ensureInitialized();
  const uint8_t newRange = range > 0 ? range : 1;
  const uint8_t newTarget = constrain(value, 0, newRange);

  noInterrupts();
  mode_ = ControlMode::PulseSkip;
  valueRange_ = newRange;
  targetValue_ = newTarget;
  if (targetValue_ == 0) {
    skipAccumulator_ = 0;
  }
  interrupts();

  if (newTarget == 0) {
    disableZeroCrossInterrupt();
  } else {
    enableZeroCrossInterrupt();
  }
}

void TriacPumpDimmer::setPhaseAngleLinearized(uint8_t value, uint8_t range) {
  ensureInitialized();

  const uint8_t newRange = range > 0 ? range : 1;
  const uint8_t constrainedValue = constrain(value, 0, newRange);
  const uint16_t linearizedDelayUs = computeLinearizedDelayUs(constrainedValue, newRange);

  noInterrupts();
  mode_ = ControlMode::PhaseAngle;
  valueRange_ = newRange;
  targetValue_ = constrainedValue;
  targetDelayUs_ = linearizedDelayUs;
  if (targetValue_ == 0) {
    phaseAccumulator_ = 0;
  }
  interrupts();

  if (constrainedValue == 0) {
    disableZeroCrossInterrupt();
  } else {
    enableZeroCrossInterrupt();
  }
}

void TriacPumpDimmer::setOff() {
  setPulseSkip(0, valueRange_);
  digitalWrite(gatePin_, LOW);
}

void TriacPumpDimmer::setFullOn() {
  setPulseSkip(valueRange_, valueRange_);
}

long TriacPumpDimmer::getCounter() {
  noInterrupts();
  const long counter = deliveredHalfCycles_;
  interrupts();
  return counter;
}

void TriacPumpDimmer::resetCounter() {
  noInterrupts();
  deliveredHalfCycles_ = 0;
  interrupts();
}

int TriacPumpDimmer::cps(uint32_t timeoutMs) {
  ensureInitialized();
  // During startup the pump is off; avoid arming zero-cross IRQ in that phase.
  // Use the latest measured or default half-period as a stable CPS estimate.
  if (!zeroCrossInterruptEnabled_ && targetValue_ == 0) {
    noInterrupts();
    const uint16_t currentHalfPeriodUs = halfPeriodUs_;
    interrupts();
    return currentHalfPeriodUs > 0 ? (int)(1000000UL / currentHalfPeriodUs) : 0;
  }

  const bool wasEnabled = zeroCrossInterruptEnabled_;
  if (!wasEnabled) {
    enableZeroCrossInterrupt();
  }

  const uint32_t startMs = millis();
  while (millis() - startMs < timeoutMs) {
    noInterrupts();
    const uint8_t sampleCount = halfPeriodSamples_;
    const uint32_t periodAccumulatorUs = halfPeriodAccumulatorUs_;
    interrupts();

    if (sampleCount >= 4 && periodAccumulatorUs > 0) {
      const uint32_t averageHalfPeriodUs = periodAccumulatorUs / sampleCount;
      if (averageHalfPeriodUs > 0) {
        return (int)(1000000UL / averageHalfPeriodUs);
      }
    }
  }

  noInterrupts();
  const uint16_t currentHalfPeriodUs = halfPeriodUs_;
  interrupts();

  if (!wasEnabled && targetValue_ == 0) {
    disableZeroCrossInterrupt();
  }
  return currentHalfPeriodUs > 0 ? (int)(1000000UL / currentHalfPeriodUs) : 0;
}

void TriacPumpDimmer::stopAfter(uint8_t halfCycles) {
  noInterrupts();
  remainingHalfCycles_ = halfCycles;
  interrupts();
}

void TriacPumpDimmer::shiftDividerCounter() {
  // No divider-based phase shifting is needed for timer-driven phase control.
}

void TriacPumpDimmer::ensureInitialized() {
  if (initialized_) {
    return;
  }

  instance_ = this;
  pinMode(gatePin_, OUTPUT);
  digitalWrite(gatePin_, LOW);
  pinMode(zeroCrossPin_, INPUT);

  timer_ = new HardwareTimer(TIM9);
  timer_->setOverflow(DEFAULT_HALF_PERIOD_US, MICROSEC_FORMAT);
  timer_->attachInterrupt(handleTimer);
  timer_->pause();

  initialized_ = true;
}

void TriacPumpDimmer::enableZeroCrossInterrupt() {
  ensureInitialized();
  if (zeroCrossInterruptEnabled_) {
    return;
  }

  noInterrupts();
  lastZeroCrossUs_ = 0;
  interrupts();
  attachInterrupt(digitalPinToInterrupt(zeroCrossPin_), handleZeroCross, zeroCrossMode_);
  zeroCrossInterruptEnabled_ = true;
}

void TriacPumpDimmer::disableZeroCrossInterrupt() {
  if (!initialized_ || !zeroCrossInterruptEnabled_) {
    return;
  }

  detachInterrupt(digitalPinToInterrupt(zeroCrossPin_));
  zeroCrossInterruptEnabled_ = false;
  if (timer_) {
    timer_->pause();
  }
  digitalWrite(gatePin_, LOW);
}

void TriacPumpDimmer::scheduleTimer(uint16_t delayUs) {
  if (!timer_) {
    return;
  }

  timer_->pause();
  timer_->setCount(0, MICROSEC_FORMAT);
  timer_->setOverflow(delayUs, MICROSEC_FORMAT);
  timer_->refresh();
  timer_->resume();
}

uint16_t TriacPumpDimmer::computeLinearizedDelayUs(uint8_t value, uint8_t range) const {
  if (value == 0) {
    return halfPeriodUs_;
  }
  if (value >= range) {
    return FULL_ON_PULSE_DELAY_US;
  }

  const float normalizedTarget = (float)value / (float)range;
  float low = 0.f;
  float high = (float)M_PI;

  for (uint8_t iteration = 0; iteration < 14; iteration++) {
    const float mid = (low + high) * 0.5f;
    const float power = computeNormalizedPower(mid);
    if (power > normalizedTarget) {
      low = mid;
    } else {
      high = mid;
    }
  }

  const float firingAngle = (low + high) * 0.5f;
  const float normalizedDelay = firingAngle / (float)M_PI;
  const uint16_t computedDelayUs = (uint16_t)((float)halfPeriodUs_ * normalizedDelay);
  return constrain(computedDelayUs, FULL_ON_PULSE_DELAY_US, (uint16_t)(halfPeriodUs_ - GATE_PULSE_US));
}

float TriacPumpDimmer::computeNormalizedPower(float firingAngleRad) {
  return 1.f - (firingAngleRad / (float)M_PI) + (sinf(2.f * firingAngleRad) / (2.f * (float)M_PI));
}

void TriacPumpDimmer::onZeroCross() {
  const uint32_t nowUs = micros();
  if (lastZeroCrossUs_ != 0) {
    const uint32_t measuredHalfPeriodUs = nowUs - lastZeroCrossUs_;
    if (measuredHalfPeriodUs >= MIN_HALF_PERIOD_US && measuredHalfPeriodUs <= MAX_HALF_PERIOD_US) {
      halfPeriodUs_ = (uint16_t)measuredHalfPeriodUs;
      if (halfPeriodSamples_ < MAX_SAMPLE_COUNT) {
        halfPeriodAccumulatorUs_ += measuredHalfPeriodUs;
        halfPeriodSamples_++;
      }
    }
  }
  lastZeroCrossUs_ = nowUs;

  digitalWrite(gatePin_, LOW);
  if (timer_) {
    timer_->pause();
  }

  if (remainingHalfCycles_ == 0) {
    targetValue_ = 0;
  }
  if (targetValue_ == 0) {
    return;
  }

  uint16_t delayUs = FULL_ON_PULSE_DELAY_US;
  bool shouldFire = false;

  if (mode_ == ControlMode::PulseSkip) {
    skipAccumulator_ += targetValue_;
    if (skipAccumulator_ >= valueRange_) {
      skipAccumulator_ -= valueRange_;
      shouldFire = true;
      deliveredHalfCycles_++;
    }
  } else {
    shouldFire = true;
    delayUs = targetDelayUs_;
    phaseAccumulator_ += targetValue_;
    while (phaseAccumulator_ >= valueRange_) {
      phaseAccumulator_ -= valueRange_;
      deliveredHalfCycles_++;
    }
  }

  if (!shouldFire) {
    return;
  }

  if (remainingHalfCycles_ > 0) {
    remainingHalfCycles_--;
  }

  timerGateOnPhase_ = true;
  scheduleTimer(delayUs);
}

void TriacPumpDimmer::onTimer() {
  if (timerGateOnPhase_) {
    digitalWrite(gatePin_, HIGH);
    timerGateOnPhase_ = false;
    scheduleTimer(GATE_PULSE_US);
    return;
  }

  digitalWrite(gatePin_, LOW);
  if (timer_) {
    timer_->pause();
  }
}

void TriacPumpDimmer::handleZeroCross() {
  if (instance_) {
    instance_->onZeroCross();
  }
}

void TriacPumpDimmer::handleTimer() {
  if (instance_) {
    instance_->onTimer();
  }
}