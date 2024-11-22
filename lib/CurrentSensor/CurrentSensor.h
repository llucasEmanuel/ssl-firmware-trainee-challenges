#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <utils.h>

#define SENSITIVITY 0.025

using Voltage = std::pair<double, double>;

class CurrentSensor {
 public:
  class Wiremap {
   public:
    int a0;
    int a1;
    int a2;

    Wiremap(int a0, int a1, int a2) : a0(a0), a1(a1), a2(a2){};
  };

  CurrentSensor();
  double getDribblerCurrent();
  Motors getMotorsCurrent();

 private:
  DigitalOut A2 = CURRENT_SENSOR_A2;
  DigitalOut A1 = CURRENT_SENSOR_A1;
  DigitalOut A0 = CURRENT_SENSOR_A0;
  AnalogIn vReference = PIN_CURR_REF;
  AnalogIn vOut = PIN_CURR_OUT;
  Wiremap motorOneWiremap{0, 0, 1};
  Wiremap motorTwoWiremap{0, 1, 1};
  Wiremap motorThreeWiremap{1, 1, 1};
  Wiremap motorFourWiremap{1, 0, 0};
  Wiremap dribblerWiremap{0, 1, 0};

  void setMuxWiremap(Wiremap& wiremap);
  double calculateCurrentFromVoltage(Voltage voltage);
  Voltage readVoltageFromMuxOutput();
};

#endif // CURRENT_SENSOR_H