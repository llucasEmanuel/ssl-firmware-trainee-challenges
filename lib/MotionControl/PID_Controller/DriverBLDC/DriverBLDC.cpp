#include "DriverBLDC.h"
#include "utils.h"

// PWM - DIR - RST - MODE - COAST - BRK
double map(double x, double in_min, double in_max, double out_min, double out_max) {
  double out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return out;
}

DriverBLDC::DriverBLDC(PinName pwm,
                       PinName dir,
                       PinName rst,
                       PinName mode,
                       PinName coast,
                       PinName brk) :
    _m(pwm),
    _dir(dir),
    _rst(rst),
    _mode(mode),
    _coast(coast),
    _break(brk) {
  _rst.write(0);   // Reset Motor
  _mode.write(1);  // Low Decay (Tiggers we trust)
  _coast.write(0); // Coast
  _break.write(1); // ! Break
  _m.period_us(PWM_PULSE_WIDTH_US);
}

void DriverBLDC::init() {
  _coast.write(0);
  _rst.write(1);
  _m.write(0);
}

void DriverBLDC::reset() {
  run(0);
  _coast.write(1);
  _rst.write(0);
}

int DriverBLDC::getRst() {
  return _rst.read();
}

double DriverBLDC::run(double speed) {
  _coast.write(1);
  double send_pwm;
  if (_mode == 0) {
    // PWM: 1 Counter clockwise <-> 0 Closkwise
    _dir.write(0);
    send_pwm = map(speed, -100, 100, 1.0, 0);
  }
  // default: mode==1
  else {
    // Dir chooses side.
    if (speed < 0) {
// invert motor direction if using direct transmission
#if defined(DIRECT_TRANSMISSION)
      _dir.write(1);
#else
      _dir.write(0);
#endif
      speed *= -1;
    } else {
// invert motor direction if using direct transmission
#if defined(DIRECT_TRANSMISSION)
      _dir.write(0);
#else
      _dir.write(1);
#endif
    }
    send_pwm = map(speed, 0, 100, 0, 1.0);
  }

  _m.write(send_pwm);
  return send_pwm;
}
void DriverBLDC::stop() {
  _coast.write(0);
}
