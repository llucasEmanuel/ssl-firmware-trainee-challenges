
#include "Odometry.h"
#include "MPU6050/MPU6050.h"
#include <cmath>

Odometry::Odometry(MotionControl* motion, Kinematics* kinematics, int tsample, int tGyroSample) :
    _MPU(MPU_SDA, MPU_SCL) {
  _motion = motion;
  _kinematics = kinematics;
  _tsample = tsample;
  _tGyroSample = tGyroSample;
}

void Odometry::init() {

  _activeIMU = _MPU.initialize();

  freqUpdate.attach(callback(this, &Odometry::update), chrono::milliseconds(_tsample));
  if (_activeIMU) {
    _gyroFlag = false;
    // calibrateGyroOffset();
    gyroUpdate.attach(callback(this, &Odometry::updateGyroFlag),
                      chrono::milliseconds(_tGyroSample));
  }

  gyroTimer.start();
}

void Odometry::calibrateGyroOffset() {
  double gyroRead[3] = {0, 0, 0};
  double k = 0.05;
  int count = 0;
  while (1) {
    getAllGyroRead(gyroRead);
    if (fabs(gyroRead[0]) < GYRO_TOLERANCE && fabs(gyroRead[1]) < GYRO_TOLERANCE &&
        fabs(gyroRead[2]) < GYRO_TOLERANCE) {
      if (count > 10)
        break;
      count++;
    }
    _gyroOffset[0] += gyroRead[0] * k;
    _gyroOffset[1] += gyroRead[1] * k;
    _gyroOffset[2] += gyroRead[2] * k;

    utils::beep(_tGyroSample, 370, 0.1);
    debug_if(DEBUG,
             "Odometry: Calibrating GyroOffset [0]: %lf, [1]: %lf, [2]: %lf\n",
             _gyroOffset[0],
             _gyroOffset[1],
             _gyroOffset[2]);
  }
  utils::beep(100, 370, 0.1);
  printf("Odometry GyroOffset [0]: %lf, [1]: %lf, [2]: %lf\n",
         _gyroOffset[0],
         _gyroOffset[1],
         _gyroOffset[2]);
  utils::beep(100, 370, 0.1);
}

double Odometry::getGyroRead() {
  double gyro[3] = {0, 0, 0};
  _MPU.readGyro(gyro);
  return gyro[2] - _gyroOffset[2];
}

void Odometry::getAllGyroRead(double* gyroRead) {
  double gyro[3] = {0, 0, 0};
  _MPU.readGyro(gyro);
  gyroRead[0] = gyro[0] - _gyroOffset[0];
  gyroRead[1] = gyro[1] - _gyroOffset[1];
  gyroRead[2] = gyro[2] - _gyroOffset[2];
}

void Odometry::update() {
  Vector move;
  _motion->getRobotSpeed(move, false);
  move = (move * double(_tsample / 1000.0));

  // Using Gyro for delta W
  if (_activeIMU) {
    // Only use Gyro angle if IMU active.
    double deltaGyro = getGyroMove();
    _gyroAngle += deltaGyro;
    limitW(_gyroAngle);
    move.w = deltaGyro;
  }
  _currentPosition = _currentPosition + _kinematics->rotateToGlobal(_currentPosition.w, move);
  limitW(_currentPosition);
  _movement = _currentPosition - _initialPosition;

  _currentOdometry = _currentOdometry + _kinematics->rotateToGlobal(_currentOdometry.w, move);
  limitW(_currentOdometry);
}

void Odometry::updateGyroFlag() {
  _gyroFlag = true;
}

void Odometry::processGyro() {
  // Should read Gyro?
  if (_gyroFlag && _activeIMU) {
    double accumulator = 0.0;
    for (int i = 0; i < _nGyroSample; i++) {
      accumulator += getGyroRead();
    }
    accumulator /= (_nGyroSample / 1.0);
    accumulator = accumulator * (M_PI / 180.0); // TO RAD
    if (fabs(accumulator) < 0.03) {
      accumulator = 0;
    }
    double gyroElapsedTime = utils::timerRead<chrono::milliseconds>(gyroTimer);
    _gyroMove += (accumulator * double(gyroElapsedTime / 1000.0));

    gyroTimer.reset();
    _gyroFlag = false; // Clear Gyro Flag
  }
}

void Odometry::updatePosition(Vector pos) {
  _initialPosition = pos;
  _currentPosition = pos;
  _movement = Vector();
  _gyroAngle = pos.w;
}

void Odometry::resetPosition() {
  _currentPosition = _initialPosition;
  _movement = Vector();
  _gyroAngle = 0;
}

void Odometry::updatePosition() {
  Vector pos;
  this->updatePosition(pos);
}

void Odometry::getCurrentPosition(Vector& v) {
  v = _currentPosition;
}

Vector Odometry::getCurrentPosition() {
  return _currentPosition;
}

Vector Odometry::getInitialPosition() {
  return _initialPosition;
}

Vector Odometry::getMovement() {
  return this->_movement;
}

Vector Odometry::getCurrentOdometry() {
  return _currentOdometry;
}

void Odometry::getGyroAngle(double& w) {
  w = _gyroAngle;
}

double Odometry::getGyroMove() {
  double w = _gyroMove;
  _gyroMove = 0; // Resets gyro move.
  return w;
}

void Odometry::limitW(double& w) {
  if (w > M_PI)
    w -= 2 * M_PI;
  else if (w < -M_PI)
    w += 2 * M_PI;
}

void Odometry::limitW(Vector& v) {
  this->limitW(v.w);
}

void Odometry::printCurrentPosition() {
  printf("Current X: %lf - Y: %lf - W: %lf \n",
         _currentPosition.x,
         _currentPosition.y,
         _currentPosition.w);
}

void Odometry::printMovement() {
  printf("Movement X: %lf - Y: %lf - W: %lf \n", _movement.x, _movement.y, _movement.w);
}
