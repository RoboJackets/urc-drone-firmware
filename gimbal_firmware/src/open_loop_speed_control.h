#ifndef OPEN_LOOP_SPEED_CONTROL_H
#define OPEN_LOOP_SPEED_CONTROL_H

#include <Arduino.h>
#include <stdint.h>

class OpenLoopSpeedController {
public:
  struct Pins {
    uint8_t enablePin;
    uint8_t phaseA;
    uint8_t phaseB;
    uint8_t phaseC;
  };

  // Construct with pins and I2C address
  OpenLoopSpeedController(const Pins& pins, uint8_t i2cAddr);

  // Initialize timers, pins, I2C (as slave), and print banner
  void begin();

  // Call from loop() continuously
  void tick();

  // I2C receive ISR entry (called by a free function wrapper)
  void onReceiveISR(int numBytes);

  // Optional: expose command velocity / angles for debugging
  int16_t commandVelocity() const;
  int16_t angleA() const;
  int16_t angleB() const;
  int16_t angleC() const;

  // Tuning helpers
  void setPwmScale(float scale_0_to_1);             // default 0.7
  void setDelayBounds(uint16_t slow_us, uint16_t fast_us); // default 5000..100

private:
  // Internal helpers
  static inline uint8_t sinePwmFromDeg(int16_t deg, float scale);
  static inline int16_t clamp16(int16_t v, int16_t lo, int16_t hi);
  static inline uint16_t mapVelToDelayUs(int16_t absVel, uint16_t slow_us, uint16_t fast_us);

  void stepAndOutput_(int8_t dir, int16_t mag);
  void holdOutput_();

  Pins pins_;
  uint8_t i2cAddr_;

  volatile int16_t cmdVel_; // [-1000,1000], updated in ISR

  // electrical angles (deg 0..359)
  int16_t aDeg_;
  int16_t bDeg_;
  int16_t cDeg_;

  // tuning
  float pwmScale_;          // 0..1 (default 0.7)
  uint16_t delaySlowUs_;    // default 5000
  uint16_t delayFastUs_;    // default 100
};

// Global trampoline for Wire.onReceive()
void OL_onI2CReceive(int numBytes);

// Access to the singleton (created in open_loop_speed_control.cpp)
OpenLoopSpeedController& OL();

#endif // OPEN_LOOP_SPEED_CONTROL_H
