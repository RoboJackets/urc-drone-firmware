#include "open_loop_speed_control.h"
#include <Wire.h>
#include <math.h>

// --------- Singleton instance (adjust pins/address here or from main) ----------
static OpenLoopSpeedController::Pins kDefaultPins = {
  /*enablePin*/ 8,
  /*phaseA   */ 9,
  /*phaseB   */ 10,
  /*phaseC   */ 11
};
static constexpr uint8_t kDefaultI2CAddr = 0x10;

// Construct the singleton with defaults
static OpenLoopSpeedController g_controller(kDefaultPins, kDefaultI2CAddr);

// Expose accessor
OpenLoopSpeedController& OL() { return g_controller; }

// Wire callback trampoline
void OL_onI2CReceive(int numBytes) {
  g_controller.onReceiveISR(numBytes);
}

// --------------------- Class impl -------------------------

OpenLoopSpeedController::OpenLoopSpeedController(const Pins& pins, uint8_t i2cAddr)
: pins_(pins),
  i2cAddr_(i2cAddr),
  cmdVel_(0),
  aDeg_(0), bDeg_(120), cDeg_(240),
  pwmScale_(0.7f),
  delaySlowUs_(5000),
  delayFastUs_(100) {}

void OpenLoopSpeedController::begin() {
  Serial.begin(115200);

  // Keep Timer0 default prescaler (affects delay/millis)
  TCCR0B = (TCCR0B & 0b11111000) | 0x03;
  // High-frequency PWM on D9/D10 (Timer1) and D11 (Timer2)
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;

  pinMode(pins_.phaseA, OUTPUT);
  pinMode(pins_.phaseB, OUTPUT);
  pinMode(pins_.phaseC, OUTPUT);
  pinMode(pins_.enablePin, OUTPUT);
  digitalWrite(pins_.enablePin, HIGH);

  Wire.begin(i2cAddr_);
  Wire.onReceive(OL_onI2CReceive);

  Serial.print(F("Open-loop BLDC I2C slave ready @0x"));
  Serial.println(i2cAddr_, HEX);
  Serial.println(F("Send int16 velocity [-1000..1000]. 0 = hold."));
}

void OpenLoopSpeedController::tick() {
  noInterrupts();
  int16_t vel = cmdVel_;
  interrupts();

  if (vel == 0) {
    holdOutput_();
    delay(1);
    return;
  }

  int8_t dir = (vel > 0) ? +1 : -1;
  int16_t mag = (vel > 0) ? vel : -vel;
  stepAndOutput_(dir, mag);
}

void OpenLoopSpeedController::onReceiveISR(int numBytes) {
  if (numBytes <= 0) return;

  int16_t v = 0;

  if (numBytes >= 2) {
    uint8_t b0 = Wire.read();
    uint8_t b1 = Wire.read();
    v = (int16_t)((uint16_t)b0 | ((uint16_t)b1 << 8));
  } else {
    int8_t b = (int8_t)Wire.read();
    v = (int16_t)b * 10; // scale shorthand
  }

  cmdVel_ = clamp16(v, -1000, 1000);
}

int16_t OpenLoopSpeedController::commandVelocity() const { return cmdVel_; }
int16_t OpenLoopSpeedController::angleA() const { return aDeg_; }
int16_t OpenLoopSpeedController::angleB() const { return bDeg_; }
int16_t OpenLoopSpeedController::angleC() const { return cDeg_; }

void OpenLoopSpeedController::setPwmScale(float s) {
  if (s < 0.f) s = 0.f;
  if (s > 1.f) s = 1.f;
  pwmScale_ = s;
}

void OpenLoopSpeedController::setDelayBounds(uint16_t slow_us, uint16_t fast_us) {
  if (fast_us < 10) fast_us = 10;
  if (slow_us < fast_us) slow_us = fast_us;
  delaySlowUs_ = slow_us;
  delayFastUs_ = fast_us;
}

void OpenLoopSpeedController::stepAndOutput_(int8_t dir, int16_t mag) {
  // Step electrical angles (1 degree per tick)
  aDeg_ += dir;
  bDeg_  = aDeg_ + 120;
  cDeg_  = bDeg_ + 120;

  aDeg_ %= 360; if (aDeg_ < 0) aDeg_ += 360;
  bDeg_ %= 360; if (bDeg_ < 0) bDeg_ += 360;
  cDeg_ %= 360; if (cDeg_ < 0) cDeg_ += 360;

  uint8_t pwmA = sinePwmFromDeg(aDeg_, pwmScale_);
  uint8_t pwmB = sinePwmFromDeg(bDeg_, pwmScale_);
  uint8_t pwmC = sinePwmFromDeg(cDeg_, pwmScale_);

  analogWrite(pins_.phaseA, pwmA);
  analogWrite(pins_.phaseB, pwmB);
  analogWrite(pins_.phaseC, pwmC);

  uint16_t dly = mapVelToDelayUs(mag, delaySlowUs_, delayFastUs_);
  delayMicroseconds(dly);
}

void OpenLoopSpeedController::holdOutput_() {
  uint8_t pwmA = sinePwmFromDeg(aDeg_, pwmScale_);
  uint8_t pwmB = sinePwmFromDeg(bDeg_, pwmScale_);
  uint8_t pwmC = sinePwmFromDeg(cDeg_, pwmScale_);

  analogWrite(pins_.phaseA, pwmA);
  analogWrite(pins_.phaseB, pwmB);
  analogWrite(pins_.phaseC, pwmC);
}

// ----------- statics -----------

inline uint8_t OpenLoopSpeedController::sinePwmFromDeg(int16_t deg, float scale) {
  int16_t d = deg % 360; if (d < 0) d += 360;
  double rad = (double)d * M_PI / 180.0;
  double s = sin(rad);                 // [-1,1]
  double pwm = s * 127.5 + 127.5;      // [0,255]
  pwm *= (double)scale;                // scale amplitude
  if (pwm < 0.0) pwm = 0.0;
  if (pwm > 255.0) pwm = 255.0;
  return (uint8_t)(pwm + 0.5);
}

inline int16_t OpenLoopSpeedController::clamp16(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline uint16_t OpenLoopSpeedController::mapVelToDelayUs(int16_t absVel, uint16_t slow_us, uint16_t fast_us) {
  if (absVel < 1) absVel = 1;
  if (absVel > 1000) absVel = 1000;
  long spanIn  = 999; // 1000-1
  long spanOut = (long)slow_us - (long)fast_us;
  long out = slow_us - ((long)(absVel - 1) * spanOut / spanIn);
  return (uint16_t)out;
}
