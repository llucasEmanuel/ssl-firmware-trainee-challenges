#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "mbed.h"
#define _USE_MATH_DEFINES
#include <vector>
#include <utils.h>

class Kinematics {
 public:
  Kinematics();
  Motors convertToWheel(double vx, double vy, double w);
  Motors convertToWheel(Vector v);
  Motors clearSpeeds();
  Vector rotateToLocal(double robot_w, Vector v);
  Vector rotateToGlobal(double robot_w, Vector v);
  Vector rotateVector(double rotateAngle, Vector v);
  Vector convertToVector(Motors m);
  Vector convertToVectorOriginal(Motors m);
  Vector clearVector();
  void validateSpeeds(Vector v);

 private:
#if defined(DIRECT_TRANSMISSION)
  double wheel_radius = 0.03005;
  double robot_radius = 0.08105;
#else
  double robot_radius = 0.02475;
  double half_axis = 0.0831;
#endif
  double angle_m1 = M_PI / 3;
  double angle_m2 = M_PI / 4;
  double angle_m3 = M_PI / 4;
  double angle_m4 = M_PI / 3;

  std::vector<double> i_kinematic_m1;
  std::vector<double> i_kinematic_m2;
  std::vector<double> i_kinematic_m3;
  std::vector<double> i_kinematic_m4;

  std::vector<double> kinematic_x;
  std::vector<double> kinematic_y;
  std::vector<double> kinematic_w;

  std::vector<double> opt_kinematic_x;
  std::vector<double> opt_kinematic_y;
  std::vector<double> opt_kinematic_w;

  // ROBOT 0 second optimization (23-01-10)
  std::vector<double> opt_parameters = std::vector<double>{0.42447140713655457,
                                                           0.35482242703016686,
                                                           -0.1823307411279881,
                                                           -0.274779656546075,
                                                           0.25745996970744844,
                                                           -0.4438060201135028,
                                                           -0.35102689244950486,
                                                           0.23426282528626358,
                                                           3.611082557939859,
                                                           2.6806232596725357,
                                                           2.5552490512551853,
                                                           3.5933353127553165,
                                                           0.022848272302565545};
};

#endif /* KINEMATICS_H */
