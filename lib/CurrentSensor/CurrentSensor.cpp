#include "CurrentSensor.h"

CurrentSensor::CurrentSensor(){};

void CurrentSensor::setMuxWiremap(Wiremap& wiremap) {
  this->A2.write(wiremap.a2);
  this->A1.write(wiremap.a1);
  this->A0.write(wiremap.a0);
}

double CurrentSensor::calculateCurrentFromVoltage(Voltage voltage) {
  double vref = voltage.first * 3.3;
  double vout = voltage.second * 3.3;

  return (vout - vref) / SENSITIVITY;
}

Voltage CurrentSensor::readVoltageFromMuxOutput() {
  return std::make_pair(vReference.read(), vOut.read());
}

Motors CurrentSensor::getMotorsCurrent() {
  Voltage voltage;
  Motors motorCurrent;

  // Motor 1
  this->setMuxWiremap(this->motorOneWiremap);
  voltage = this->readVoltageFromMuxOutput();
  motorCurrent.m1 = this->calculateCurrentFromVoltage(voltage);

  // Motor 2
  this->setMuxWiremap(this->motorTwoWiremap);
  voltage = this->readVoltageFromMuxOutput();
  motorCurrent.m2 = this->calculateCurrentFromVoltage(voltage);

  // Motor 3
  this->setMuxWiremap(this->motorThreeWiremap);
  voltage = this->readVoltageFromMuxOutput();
  motorCurrent.m3 = this->calculateCurrentFromVoltage(voltage);

  // Motor 4
  this->setMuxWiremap(this->motorFourWiremap);
  voltage = this->readVoltageFromMuxOutput();
  motorCurrent.m4 = this->calculateCurrentFromVoltage(voltage);

  return motorCurrent;
}

double CurrentSensor::getDribblerCurrent() {
  // Setup wiremap to dribbler output
  this->setMuxWiremap(this->dribblerWiremap);

  // Read output
  Voltage voltage = this->readVoltageFromMuxOutput();

  // Do not use utils getCurrent, refactor this later on
  return this->calculateCurrentFromVoltage(voltage);
}