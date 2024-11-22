#ifndef KICKER_H
#define KICKER_H

#include "mbed.h"
#include <utils.h>

class Kicker {
 public:
  Kicker(PinName charge_kick,
         PinName load_kick,
         PinName front_kick,
         PinName chip_kick,
         PinName IR_LED,
         PinName IR_ADC);
  void init();
  void update(KickFlags& flagsKick, float minLevel, float maxLevel);
  void charge(float level = 0.5);
  void stopCharge();
  void discharge();
  float getLoad();
  void clear(KickFlags& isKick);
  bool isCharging(KickFlags& isKick);
  float readIR();
  void kick(bool kickFront, bool kickChip, float strength);
  void front(float strength);
  void chip(float strength);
  void stopKick();
  void printKick(KickFlags& isKick);
  void getKickerInfo(double& kickLoad, bool& ball);
  double getDribblerSpeed(double speed);

 private:
  DigitalOut _charge;
  AnalogIn _load;
  DigitalOut _front;
  DigitalOut _chip;

  DigitalOut _irLED;
  AnalogIn _irADC;
  float _irReading = 0;
  Timer kickerTimeOut;

  bool hysteresis_flag = false;
  float currentLoad = 0;
};

#endif /* KICKER_H */
