#include "kinematics.h"
#include <utils.h>

Kinematics::Kinematics() {

  /**
   * Attention:
   * 1. The formal matrix calculetes motors in anti-clock order.
   * 2. The robot board uses motors in clock order.
   */

  // Matrix for calculating Inverse Kinematics
  i_kinematic_m1.push_back(sin(angle_m1) / wheel_radius);     // vx
  i_kinematic_m1.push_back(cos(angle_m1) / wheel_radius);     // vy
  i_kinematic_m1.push_back(robot_radius / wheel_radius);      // w

  i_kinematic_m2.push_back(sin(angle_m2) / wheel_radius);      // vx
  i_kinematic_m2.push_back(-1 * cos(angle_m2) / wheel_radius); // vy
  i_kinematic_m2.push_back(robot_radius / wheel_radius);       // w

  i_kinematic_m3.push_back(-1 * sin(angle_m3) / wheel_radius); // vx
  i_kinematic_m3.push_back(-1 * cos(angle_m3) / wheel_radius); // vy
  i_kinematic_m3.push_back(robot_radius / wheel_radius);       // w

  i_kinematic_m4.push_back(-1 * sin(angle_m4) / wheel_radius); // vx
  i_kinematic_m4.push_back(cos(angle_m4) / wheel_radius);      // vy
  i_kinematic_m4.push_back(robot_radius / wheel_radius);       // w

  // Matrix for calculating Direct Kinematics
  kinematic_x.push_back(0.34641 * wheel_radius);   // m1(robot)
  kinematic_x.push_back(0.282843 * wheel_radius);  // m2(robot)
  kinematic_x.push_back(-0.282843 * wheel_radius); // m3(robot)
  kinematic_x.push_back(-0.34641 * wheel_radius);  // m4(robot)

  kinematic_y.push_back(0.414214 * wheel_radius);  // m1(robot)
  kinematic_y.push_back(-0.414216 * wheel_radius); // m2(robot)
  kinematic_y.push_back(-0.414214 * wheel_radius); // m3(robot)
  kinematic_y.push_back(0.414214 * wheel_radius);  // m4(robot)

#if defined(DIRECT_TRANSMISSION)
  kinematic_w.push_back(3.56751789 * wheel_radius); // m1(robot)
  kinematic_w.push_back(2.52261609 * wheel_radius); // m2(robot)
  kinematic_w.push_back(2.52261609 * wheel_radius); // m3(robot)
  kinematic_w.push_back(3.56751789 * wheel_radius); // m4(robot)
#else
  kinematic_w.push_back(3.616966 * wheel_radius); // m1(robot)
  kinematic_w.push_back(2.556874 * wheel_radius); // m2(robot)
  kinematic_w.push_back(2.556874 * wheel_radius); // m3(robot)
  kinematic_w.push_back(3.616966 * wheel_radius); // m4(robot)
#endif

  opt_kinematic_x.push_back(opt_parameters[0] * opt_parameters[12]); // m1(robot)
  opt_kinematic_x.push_back(opt_parameters[1] * opt_parameters[12]); // m2(robot)
  opt_kinematic_x.push_back(opt_parameters[2] * opt_parameters[12]); // m3(robot)
  opt_kinematic_x.push_back(opt_parameters[3] * opt_parameters[12]); // m4(robot)

  opt_kinematic_y.push_back(opt_parameters[4] * opt_parameters[12]); // m1(robot)
  opt_kinematic_y.push_back(opt_parameters[5] * opt_parameters[12]); // m2(robot)
  opt_kinematic_y.push_back(opt_parameters[6] * opt_parameters[12]); // m3(robot)
  opt_kinematic_y.push_back(opt_parameters[7] * opt_parameters[12]); // m4(robot)

  opt_kinematic_w.push_back(opt_parameters[8] * opt_parameters[12]);  // m1(robot)
  opt_kinematic_w.push_back(opt_parameters[9] * opt_parameters[12]);  // m2(robot)
  opt_kinematic_w.push_back(opt_parameters[10] * opt_parameters[12]); // m3(robot)
  opt_kinematic_w.push_back(opt_parameters[11] * opt_parameters[12]); // m4(robot)
}

Motors Kinematics::convertToWheel(double vx, double vy, double w) {
  Motors m;
  m.m1 = ((i_kinematic_m1[0] * vx) + (i_kinematic_m1[1] * vy) + (i_kinematic_m1[2] * w));
  m.m2 = ((i_kinematic_m2[0] * vx) + (i_kinematic_m2[1] * vy) + (i_kinematic_m2[2] * w));
  m.m3 = ((i_kinematic_m3[0] * vx) + (i_kinematic_m3[1] * vy) + (i_kinematic_m3[2] * w));
  m.m4 = ((i_kinematic_m4[0] * vx) + (i_kinematic_m4[1] * vy) + (i_kinematic_m4[2] * w));
  return m;
}

Motors Kinematics::convertToWheel(Vector v) {
  return this->convertToWheel(v.x, v.y, v.w);
}

Motors Kinematics::clearSpeeds() {
  Motors m;
  m.m1 = 0;
  m.m2 = 0;
  m.m3 = 0;
  m.m4 = 0;
  return m;
}

Vector Kinematics::convertToVector(Motors m) {
  Vector v;
  v.x = ((opt_kinematic_x[0] * m.m1) + (opt_kinematic_x[1] * m.m2) + (opt_kinematic_x[2] * m.m3) +
         (opt_kinematic_x[3] * m.m4));
  v.y = ((opt_kinematic_y[0] * m.m1) + (opt_kinematic_y[1] * m.m2) + (opt_kinematic_y[2] * m.m3) +
         (opt_kinematic_y[3] * m.m4));
  v.w = ((opt_kinematic_w[0] * m.m1) + (opt_kinematic_w[1] * m.m2) + (opt_kinematic_w[2] * m.m3) +
         (opt_kinematic_w[3] * m.m4));
  v.x = fabs(v.x) < 0.0001 ? 0 : v.x;
  v.y = fabs(v.y) < 0.0001 ? 0 : v.y;
  v.w = fabs(v.w) < 0.0001 ? 0 : v.w;

  return v;
}

Vector Kinematics::convertToVectorOriginal(Motors m) {
  Vector v;
  v.x = ((kinematic_x[0] * m.m1) + (kinematic_x[1] * m.m2) + (kinematic_x[2] * m.m3) +
         (kinematic_x[3] * m.m4));
  v.y = ((kinematic_y[0] * m.m1) + (kinematic_y[1] * m.m2) + (kinematic_y[2] * m.m3) +
         (kinematic_y[3] * m.m4));
  v.w = ((kinematic_w[0] * m.m1) + (kinematic_w[1] * m.m2) + (kinematic_w[2] * m.m3) +
         (kinematic_w[3] * m.m4));
  v.x = fabs(v.x) < 0.0001 ? 0 : v.x;
  v.y = fabs(v.y) < 0.0001 ? 0 : v.y;
  v.w = fabs(v.w) < 0.0001 ? 0 : v.w;

  return v;
}

Vector Kinematics::rotateToLocal(double robot_w, Vector v) {
  Vector l;
  l.x = v.x * cos(robot_w) + v.y * sin(robot_w);
  l.y = -v.x * sin(robot_w) + v.y * cos(robot_w);
  l.w = v.w;
  return l;
}

Vector Kinematics::rotateToGlobal(double robot_w, Vector v) {
  Vector g;
  g.x = v.x * cos(robot_w + v.w / 2) - v.y * sin(robot_w + v.w / 2);
  g.y = v.x * sin(robot_w + v.w / 2) + v.y * cos(robot_w + v.w / 2);
  g.w = v.w;
  return g;
}

Vector Kinematics::rotateVector(double rotateAngle, Vector v) {
  Vector g;
  g.x = v.x * cos(rotateAngle) - v.y * sin(rotateAngle);
  g.y = v.x * sin(rotateAngle) + v.y * cos(rotateAngle);
  g.w = v.w;
  return g;
}

Vector Kinematics::clearVector() {
  Vector v;
  v.x = 0;
  v.y = 0;
  v.w = 0;
  return v;
}

void Kinematics::validateSpeeds(Vector v) {
  Motors m = this->convertToWheel(v);
  printf("v-M1: %.3lf  | v-M2: %.3lf | v-M3: %.3lf  | v-M4: %.3lf\n", m.m1, m.m2, m.m3, m.m4);
  Vector v_converted = this->convertToVector(m);
  printf("Sent Vect: v-x: %.3lf  | v-y: %.3lf | v-w: %.3lf\n", v.x, v.y, v.w);
  printf("Converted: v-x: %.3lf  | v-y: %.3lf | v-w: %.3lf\n\n\n",
         v_converted.x,
         v_converted.y,
         v_converted.w);
}
