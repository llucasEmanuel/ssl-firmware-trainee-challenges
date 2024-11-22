#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "mbed.h"
#include <MPU6050.h>
#include <MotionControl.h>
#include <utils.h>

class Odometry {
 public:
  Odometry(MotionControl* motion, Kinematics* kinematics, int tsample, int gyroSample);
  void init();
  void updatePosition(Vector pos);
  void updatePosition();
  void resetPosition();
  void getCurrentPosition(Vector& v);
  void getSpeedIntPosition(Vector& v);
  Vector getMovement();
  Vector getInitialPosition();
  Vector getCurrentPosition();
  Vector getCurrentOdometry();
  Vector getSpeedIntPosition();

  void getGyroAngle(double& w); // Absolute angle
  double getGyroRead();
  void getAllGyroRead(double* gyroRead);
  double getGyroMove(); // Relative angular movement

  void printCurrentPosition();
  void printSpeedIntPosition();
  void printMovement();
  void printIntegralMovement();
  void processGyro();

 private:
  void update();
  void updateGyroFlag();
  void calibrateGyroOffset();
  void limitW(Vector& v);
  void limitW(double& w);

  Ticker freqUpdate;
  int _tsample;

  bool _gyroFlag = true;
  bool _activeIMU = false;
  Ticker gyroUpdate;
  Timer gyroTimer;
  int _tGyroSample;
  double _nGyroSample = 3.0;

  double _gyroMove = 0;
  double _gyroAngle = 0;
  std::vector<double> _gyroOffset = {0, 0, 0};

  Vector _initialPosition;
  Vector _currentPosition;
  Vector _movement;
  Vector _currentOdometry;
  MPU6050 _MPU;
  MotionControl* _motion;
  Kinematics* _kinematics;
};

#endif /* ODOMETRY_H */
