#include "mbed.h"
#include "FastPWM.h"

#ifndef DriverlBLDC_H
  #define DriverlBLDC_H

  #define PWM_FREQUENCY      50000
  #define PWM_PULSE_WIDTH    (1.0 / PWM_FREQUENCY)
  #define PWM_PULSE_WIDTH_US (PWM_PULSE_WIDTH * 1000000)

class DriverBLDC {
 public:
  DriverBLDC(PinName pwm, PinName dir, PinName rst, PinName mode, PinName coast, PinName brk);

  void init(void);
  double run(double speed);
  void stop(void);
  void reset();
  int getRst();

 private:
  FastPWM _m;
  DigitalOut _dir;
  DigitalOut _rst;
  DigitalOut _mode;
  DigitalOut _coast;
  DigitalOut _break;
};

#endif