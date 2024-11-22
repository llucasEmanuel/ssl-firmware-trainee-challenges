#ifndef NAVIGATION_H
#define NAVIGATION_H
#define MIN_ROTATE_ANGLE       5     // degrees
#define TRASITION_ROTATE_ANGLE 180   // degrees     (ideal: 180º)
#define OUTER_ORBIT_RADIUS     0.160 // meters
#define INNER_ORBIT_RADIUS     0.090 // meters      (ideal: 90mm)

#include "mbed.h"
#include <MotionControl.h>
#include <Odometry.h>
#include <kinematics.h>
#include <utils.h>

class Navigation {
 public:
  Navigation(Kinematics* kinematics, MotionControl* motion, Odometry* odometry);

  void update(Vector& motionSpeed, Vector radioSpeed);
  void update(RobotPosition lastTargetPacket);
  void move(Vector& motionSpeed);
  void rotateControl(Vector& speed, Vector robot, float targetAngle, float angleKp);
  void rotateControl(Vector& speed, Vector robot);
  void motionControlOld(Vector& speed, Vector robot, Vector target, float maxSpeed, float minSpeed);
  void motionControlOld(Vector& speed, Vector robot);
  void rotateInPoint(Vector& speed,
                     Vector robot,
                     Vector target,
                     float maxSpeed,
                     float propSpeedFactor,
                     bool rotateInClockWise,
                     float orbitRadius,
                     float angleKp,
                     float approachKp);
  void rotateInPoint(Vector& speed, Vector robot);
  void rotateInPoint(Vector& speed, Vector robot, Vector target, float maxSpeed, float minSpeed);
  void printTarget();
  void printVelocity(Vector speed);

 private:
  Kinematics* _kinematics;
  MotionControl* _motion;
  Odometry* _odometry;
  RobotPosition _target;
};

#endif /* NAVIGATION_H */
