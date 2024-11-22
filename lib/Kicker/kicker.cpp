#include "kicker.h"

Kicker::Kicker(PinName charge,
               PinName load,
               PinName front,
               PinName chip,
               PinName IR_LED,
               PinName IR_ADC) :
    _charge(charge, 0),
    _load(load),
    _front(front, 1),
    _chip(chip, 1),
    _irLED(IR_LED),
    _irADC(IR_ADC) {
  this->stopKick();
  this->stopCharge();
  _irLED.write(0);
}

void Kicker::init() {

  while(this->getLoad() > NO_LOAD){
    this->discharge();
    ThisThread::sleep_for(200ms);
  }
  this->stopCharge();
  kickerTimeOut.start();
}

void Kicker::update(KickFlags& flagsKick, float minLevel, float maxLevel) {

  flagsKick.ball = (this->readIR() < IR_THRESHOLD);

  if (flagsKick.front || flagsKick.chip) {
    if ((flagsKick.ball || flagsKick.bypassIR) && this->getLoad() > minLevel &&
        utils::timerRead<chrono::milliseconds>(kickerTimeOut) > KICKER_TIMEOUT) {
      // Kick with Front or Chip
      this->kick(flagsKick.front, flagsKick.chip, flagsKick.kickStrength);
      this->clear(flagsKick);
    } else {
      // Just Charge
      this->charge(maxLevel);
      this->stopKick();
    }
  } else if (flagsKick.charge) {
    // Just Charge
    this->charge(maxLevel);
    this->stopKick();
  } else {
    this->stopCharge();
    this->stopKick();
  }

  if (utils::timerRead<chrono::milliseconds>(kickerTimeOut) > KICKER_TIMEOUT) {
    kickerTimeOut.stop();
  }
}

void Kicker::clear(KickFlags& isKick) {
  isKick.front = false;
  isKick.chip = false;
  isKick.charge = false;
  isKick.dribbler = false;
  isKick.ball = (this->readIR() < IR_THRESHOLD);
}

bool Kicker::isCharging(KickFlags& isKick) {
  return isKick.charge || isKick.front || isKick.chip;
}

float Kicker::getLoad() {
  return (_load.read() * 3.3) / MAX_CAP_V_LOAD;
}

void Kicker::charge(float level) {
  this->stopKick();
  currentLoad = (_load.read() * 3.3) / MAX_CAP_V_LOAD;

  if (currentLoad < level)
    _charge.write(1);
  else
    _charge.write(0);
}

void Kicker::stopCharge() {
  _charge.write(0);
  wait_us(10);
}

void Kicker::discharge() {
  this->stopCharge();
  _front.write(0);
  wait_us(1000);
  _front.write(1);
}

void Kicker::kick(bool kickFront, bool kickChip, float strength) {
  kickerTimeOut.reset();
  kickerTimeOut.start();

  if (kickFront) {
    this->front(strength);
  } else if (kickChip) {
    this->chip(strength);
  }
}

void Kicker::front(float strength) {
  _charge.write(0);
  wait_us(1);
  _front.write(0); // Activate Front
  wait_us(int(1000 * fmax(strength, 1)));
  _front.write(1); // Activate Front
}

void Kicker::chip(float strength) {
  _charge.write(0);
  wait_us(1);
  _chip.write(0); // Activate Chip
  wait_us(int(1000 * fmax(strength, 1)));
  _chip.write(1);
}

void Kicker::stopKick() {
  _front.write(1);
  _chip.write(1);
}

float Kicker::readIR() {
  _irLED.write(1);
  wait_us(70);
  _irReading = _irADC.read();
  wait_us(1);
  _irLED.write(0);
  return _irReading;
}

void Kicker::printKick(KickFlags& isKick) {
  printf(
      "Kicker - Load: %f | Ball: %d | IR %f | Front: %d | Chip: %d | Charge: %d | BypassIR: %d | Strength: %f \n",
      this->getLoad(),
      isKick.ball,
      this->readIR(),
      isKick.front,
      isKick.chip,
      isKick.charge,
      isKick.bypassIR,
      isKick.kickStrength);
  printf("Kicker - Dribbler:%d | Speed: %f  \n", isKick.dribbler, isKick.dribblerSpeed);
}

void Kicker::getKickerInfo(double& kickLoad, bool& ball) {
  kickLoad = this->getLoad();
  ball = (this->readIR() < IR_THRESHOLD);
}

double Kicker::getDribblerSpeed(double speed) {
  return -(speed * (MAX_DRIBBLER_SPEED - MIN_DRIBBLER_SPEED) / 100 + MIN_DRIBBLER_SPEED);
}
