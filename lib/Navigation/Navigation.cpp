
#include "Navigation.h"

Navigation::Navigation(Kinematics* kinematics, MotionControl* motion, Odometry* odometry) {
  _kinematics = kinematics;
  _motion = motion;
  _odometry = odometry;
}

void Navigation::update(Vector& motionSpeed, Vector radioSpeed) {
  motionSpeed = radioSpeed;
}

void Navigation::update(RobotPosition lastPositionPacket) {
  // updates source position or target position
  if (lastPositionPacket.type == PositionType::source) {
    _odometry->updatePosition(lastPositionPacket.v);
  } else {
    this->_target = lastPositionPacket;
  }
}

void Navigation::move(Vector& motionSpeed) {
  // updates desired speed based on current position, target position and trajectory type
  if (this->_target.type == PositionType::motionControl) {
    this->motionControlOld(motionSpeed, _odometry->getCurrentPosition());
  } else if (this->_target.type == PositionType::rotateInPoint) {
    this->rotateInPoint(motionSpeed, _odometry->getCurrentPosition());
  } else if (this->_target.type == PositionType::rotateControl) {
    this->rotateControl(motionSpeed, _odometry->getCurrentPosition());
  }
}

void Navigation::rotateControl(Vector& speed, Vector robot, float targetAngle, float angleKp) {
  float dtheta = math::smallestAngleDiff(targetAngle, robot.w);
  float kp = angleKp > 0.01f ? angleKp : ANGLE_KP;

  speed.x = 0.0;
  speed.y = 0.0;
  speed.w = (kp * dtheta);
  return;
}

void Navigation::rotateControl(Vector& speed, Vector robot) {
  return this->rotateControl(speed, robot, this->_target.v.w, ANGLE_KP);
}

void Navigation::motionControlOld(Vector& speed,
                                  Vector robot,
                                  Vector target,
                                  float maxSpeed,
                                  float minSpeed) {
  float dist = math::distance(robot, target);

  // If Player send max speed, this max speed has to be respected, otherwise use
  // 1 m/s.
  maxSpeed = maxSpeed > 0.001 ? maxSpeed : 1;

  maxSpeed = math::map(dist, 0, 1.5, minSpeed, maxSpeed);

  if ((math::distance(robot, target) > 0.05)) {
    // Uses an angle Proportional,to get the right angle using only angular
    // speed. and then use linear speed to get into the point.
    float dx = target.x - robot.x;
    float dy = target.y - robot.y;

    float theta_v = atan2(dy, dx);

    float dtheta = math::smallestAngleDiff(target.w, robot.w);

    // Proportional to prioritize the angle correction
    float v_prop = fabs(math::smallestAngleDiff(dtheta, M_PI - 0.1)) * (maxSpeed / (M_PI - 0.1));

    // Global speed
    Vector global(v_prop * cos(theta_v), v_prop * sin(theta_v), (ANGLE_KP * dtheta));

    // Rotate to robot speed.
    speed = _kinematics->rotateToLocal(robot.w, global);
  } else {
    return this->rotateControl(speed, robot, target.w, ANGLE_KP);
  }

  return;
}

void Navigation::motionControlOld(Vector& speed, Vector robot) {
  RobotPosition pos = this->_target;
  this->motionControlOld(speed, robot, pos.v, pos.maxSpeed, pos.minSpeed);
}

void Navigation::rotateInPoint(Vector& speed,
                               Vector robot,
                               Vector target,
                               float maxSpeed,
                               float propSpeedFactor,
                               bool rotateInClockWise,
                               float orbitRadius,
                               float angleKp,
                               float approachKp) {

  float robotRadius = math::distance(robot, target);
  float dtheta = math::smallestAngleDiff(target.w, robot.w);
  float kp = angleKp > 0.01f ? angleKp : ANGLE_KP;
  // Rotate in the smaller angle
  float clockSignal = rotateInClockWise ? 1 : -1;
  float velocity = maxSpeed;
  velocity *= propSpeedFactor;

  speed.x = approachKp * (robotRadius - orbitRadius);
  speed.y = clockSignal * velocity;
  speed.w = (-(clockSignal * velocity) / (orbitRadius)) + (kp * dtheta);

  return;
}

void Navigation::rotateInPoint(Vector& speed,
                               Vector robot,
                               Vector target,
                               float maxSpeed,
                               float minSpeed) {

  // vetor (robô => ponto de rotação)
  Vector diffVector = target - robot;
  diffVector.w = atan2(diffVector.y, diffVector.x);
  float R = math::distance(target, robot);

  // erro de orientação:
  float alpha = math::smallestAngleDiff(diffVector.w, robot.w);
  // quanto falta girar em torno do ponto de referência:
  float dtheta = math::smallestAngleDiff(target.w, diffVector.w);

  if (fabs(dtheta + alpha) < MIN_ROTATE_ANGLE * M_PI / 180 && fabs(R - INNER_ORBIT_RADIUS) < 0.03 &&
      fabs(alpha) < 2 * MIN_ROTATE_ANGLE * M_PI / 180) {
    speed = Vector(0, 0, 0);
    return;
  }

  // Reduce maxSpeed to prioritize the angle correction
  float v_prop = fabs(math::smallestAngleDiff(alpha, M_PI - 0.0001)) / (M_PI - 0.0001);
  maxSpeed = v_prop * maxSpeed;

  // proportional desired radius:
  float desired_radius;
  desired_radius = math::map(fabs(dtheta),
                             0,
                             TRASITION_ROTATE_ANGLE * M_PI / 180,
                             INNER_ORBIT_RADIUS,
                             OUTER_ORBIT_RADIUS);

  // radial velocity proportional to difference between current and desired radius:
  float Rdiff = desired_radius - R;
  float v_r = math::map(fabs(Rdiff), 0, maxSpeed / 2, 0, maxSpeed);
  if (Rdiff > 0) {
    v_r = -v_r;
  }

  // tangential velocity proportional to current radius and limits resulting translational speed to
  // maxSpeed:
  float vy_prop = 1 - math::map(fabs(Rdiff), 0, (OUTER_ORBIT_RADIUS - INNER_ORBIT_RADIUS), 0, 1);
  float vy_max2 = maxSpeed * maxSpeed - v_r * v_r;
  float vy_max = vy_max2 > 0.001 ? sqrt(vy_max2) : 0;

  // vy mapped according to dtheta:
  float v_tan = vy_prop * math::map(fabs(dtheta), 0, M_PI, 0, vy_max);
  if (dtheta > 0) {
    v_tan = -v_tan;
  }

  // w for rotating around reference point and compensating orientation error:
  float w = -v_tan / R + ANGLE_KP * alpha;

  // project v_r and v_tan on correct directions:
  float local_vx = v_r * cos(alpha) - v_tan * sin(alpha);
  float local_vy = v_r * sin(alpha) + v_tan * cos(alpha);

  // rotate to robot orientation:
  Vector local(local_vx, local_vy, w);
  speed = local;
}

void Navigation::rotateInPoint(Vector& speed, Vector robot) {
  RobotPosition pos = this->_target;
  this->rotateInPoint(speed, robot, pos.v, pos.maxSpeed, pos.minSpeed);
}

void Navigation::printTarget() {
  RobotPosition pos = this->_target;
  printf("Type: %d |  X: %lf, Y: %lf, W: %lf  | max: %lf, min %lf\n",
         pos.type,
         pos.v.x,
         pos.v.y,
         pos.v.w,
         pos.maxSpeed,
         pos.minSpeed);
}

void Navigation::printVelocity(Vector speed) {
  printf("Vx: %lf, Vy: %lf, w: %lf\n", speed.x, speed.y, speed.w);
}
