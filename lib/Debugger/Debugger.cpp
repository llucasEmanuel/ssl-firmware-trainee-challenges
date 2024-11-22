#include "Debugger.h"
#include <chrono>
#include <cstdio>

Debugger::Debugger(MotionControl* motionControl,
                   Odometry* odometry,
                   Kicker* kicker,
                   nRF24Communication* radio_recv,
                   nRF24Communication* radio_send,
                   CurrentSensor* current_sensor) {

  _motion = motionControl;
  _odometry = odometry;
  _kick = kicker;
  _radio_recv = radio_recv;
  _radio_send = radio_send;
  _current_sensor = current_sensor;
}

Debugger::Debugger(MotionControl* motionControl,
                   Odometry* odometry,
                   Kicker* kicker,
                   nRF24Communication* radio_recv,
                   nRF24Communication* radio_send) {
  _motion = motionControl;
  _odometry = odometry;
  _kick = kicker;
  _radio_recv = radio_recv;
  _radio_send = radio_send;
}

void Debugger::processRobot(ExecMode robotMode) {
  // to do: check if any robotMode is broken
  if (robotMode == ExecMode::GAME_ON) {
    printf("Starting game...\n");
  } else if (robotMode == ExecMode::TEST) {
    runAllTests();
  } else if (robotMode == ExecMode::VISION_BLACKOUT) {
    printf("Starting Vision Blackout...\n");
  } else if (robotMode == ExecMode::TEST_BASICS) {
    testRobotBasics(30);
  } else if (robotMode == ExecMode::TEST_PWM) {
    testRobotMotors();
  } else if (robotMode == ExecMode::TEST_PID) {
    testRobotPID();
  } else if (robotMode == ExecMode::TEST_KICK) {
    testKick();
  } else if (robotMode == ExecMode::TEST_DRIBBLER) {
    testDribblerPWM(-80); // default: -50
  } else if (robotMode == ExecMode::TEST_ODOMETRY) {
    testOdometry();
  } else if (robotMode == ExecMode::TEST_MPU) {
    _odometry->init();
    testMPU();
  } else if (robotMode == ExecMode::TEST_IR) {
    testIR();
  } else if (robotMode == ExecMode::TEST_RECEIVER_BALL) {
    testReceiveBall();
  } else if (robotMode == ExecMode::TEST_CURRENT) {
    testCurrent();
  } else if (robotMode == ExecMode::GET_CURRENT) {
    getMotorsCurrent();
  } else {
    printf("Unknown robot mode: %d\n", static_cast<int>(robotMode));
  }
}

void Debugger::getMotorsCurrent(double maxPWM, double minPWM, uint8_t step) {
  while (utils::PBT1)
    ThisThread::sleep_for(300ms);

  Motors motorsCurrent;
  double pwm = minPWM;

  printf("PWM_VALUE,M1_CURRENT,M2_CURRENT,M3_CURRENT,M4_CURRENT\n");

  while (1) {
    _motion->moveRobotPWM(pwm);
    motorsCurrent = _current_sensor->getMotorsCurrent();
    printf("%f,%f,%f,%f,%f\n",
           pwm,
           motorsCurrent.m1,
           motorsCurrent.m2,
           motorsCurrent.m3,
           motorsCurrent.m4);

    if (!utils::PBT2) {
      while (utils::PBT1)
        ;
      ThisThread::sleep_for(500ms);
      pwm += step;
      if (pwm > maxPWM) {
        pwm = minPWM;
      }
    }
  }
}

void Debugger::testCurrent() {
  while (1) {
    Motors motorsCurrent = _current_sensor->getMotorsCurrent();
    printf("M1: %f - M2: %f - M3: %f - M4: %f\n",
           motorsCurrent.m1,
           motorsCurrent.m2,
           motorsCurrent.m3,
           motorsCurrent.m4);
    ThisThread::sleep_for(100ms);
  }
}

void Debugger::printMotorsSpeed() {
  _motion->printMotorsSpeed();
}
void Debugger::printKickerBasics() {
  printf("IR ADC: %f   |   ", _kick->readIR());
  printf("Kicker Load: %f\n", _kick->getLoad());
}
void Debugger::printRadioConnection() {
  printf("nRF Connected? %d\n", (int) _radio_recv->compareChannel(SSL_1_ROBOT_RECV_CH));
}
void Debugger::printRadiosConnections() {
  printf("nRF 1: %d   |   nRF 2: %d\n",
         (int) _radio_recv->compareChannel(SSL_1_ROBOT_RECV_CH),
         (int) _radio_send->compareChannel(SSL_2_ROBOT_SEND_CH));
}
void Debugger::printRadioDetails() {
  _radio_recv->printDetails();
}
void Debugger::printRadioReceivedInfos() {

  printf("Updated ?\n");
  Vector v;
  KickFlags kick;
  v = _radio_recv->getVectorSpeed();
  _radio_recv->getKick(kick);
  printf("Vx    = %.3f\n", v.x);
  printf("Vy    = %.3f\n", v.y);
  printf("W     = %.3f\n", v.w);
  printf("Front = %d\n", kick.front);
  printf("Chip  = %d\n", kick.chip);
  printf("Charge = %d\n", kick.charge);
  printf("Strength = %.1f\n", kick.kickStrength);
  printf("Dribbler = %d\n", kick.dribbler);
  printf("Dribbler Speed = %f\n", kick.dribblerSpeed);
  printf("\n");
}
void Debugger::printBattery() {
  printf("Battery: %.3f ---->  %.3f  V!\n", utils::BATT.read(), utils::getBattery());
}
void Debugger::printSwitchs() {
  printf("SW> 1: %d | 2: %d | 3: %d | 4: %d | ID: %d - \n",
         utils::SW1.read(),
         utils::SW2.read(),
         utils::SW3.read(),
         utils::SW4.read(),
         utils::getRobotId());
}
void Debugger::printButtons() {
  printf("BT 1: %d | BT 2: %d\n", utils::PBT1.read(), utils::PBT2.read());
}
void Debugger::printRobotBasics() {
  printBattery();
  printSwitchs();
  printButtons();
  printf("\n");
  printRadioConnection();
  printMotorsSpeed();
  printKickerBasics();
}

void Debugger::runAllTests() {

  // Signal robotMode TEST
  Status::sendRoundColor(Color::WHITE, 100);
  Status::sendRoundColor(Color::PINK, 100);
  Status::sendRoundColor(Color::WHITE, 100);

  // beep to signal the start of the tests
  utils::beep(180);

  ExecMode currentMode = ExecMode::TEST_PWM;
  while (1) {
    // signals transition
    Status::send(Color::WHITE, ALL_LEDS);

    if (currentMode == ExecMode::TEST_PWM) {
      printf("Testing PWM\n");
      currentMode = testRobotMotors();
    } else if (currentMode == ExecMode::TEST_PID) {
      printf("Testing PID\n");
      currentMode = testRobotPID();
    } else if (currentMode == ExecMode::TEST_IR) {
      printf("Testing IR\n");
      currentMode = testIR();
    } else if (currentMode == ExecMode::TEST_KICK) {
      printf("Testing Kicker\n");
      currentMode = testKick();
    } else {
      printf("Unknown robot mode: %d\n", (int) currentMode);
    }
  }
}

void Debugger::testRobotBasics(int v) {
  // TODO: Reactivate MPU when all robots receive the MPU interface.
  // utils::initMPU();
  while (1) {
    if (!utils::PBT1) {
      _motion->moveRobotPWM(v);
      v = -v;
    } else if (!utils::PBT2) {
      _motion->stopRobot();
    }
    printRobotBasics();
    // utils::printMPU();
    ThisThread::sleep_for(300ms);
  }
}

ExecMode Debugger::testRobotMotors(bool start, double limitSpeed, uint8_t repeats) {
  Status::send(Color::PINK, ALL_LEDS);

  // Wait for release
  while (!utils::PBT1)
    ;
  while (!utils::PBT2)
    ;
  ThisThread::sleep_for(10ms); // debounce

  bool running = start;
  int speed = 1;
  int v = speed;
  int step = 1;
  int count = 0;
  Timer motor_pid;
  motor_pid.start();
  ButtonState PBT1State = ButtonState::UNPRESSED;
  ButtonState PBT2State = ButtonState::UNPRESSED;

  printf("Press PBT1 to start, PBT2 to stop\n");
  while (1) {
    ThisThread::sleep_for(1ms);

    PBT1State = utils::pressedFor(utils::PBT1, 2000);
    PBT2State = utils::pressedFor(utils::PBT2, 2000);

    if (PBT1State == ButtonState::HELD) {
      _motion->stopRobot();
      return ExecMode::TEST_IR;
    }
    if (PBT2State == ButtonState::HELD) {
      _motion->stopRobot();
      return ExecMode::TEST_PID;
    }

    if (PBT1State == ButtonState::RELEASED) {
      running = true;
      printf("Starting...\n");
      ThisThread::sleep_for(200ms);
    }
    if (PBT2State == ButtonState::RELEASED) {
      printf("Stopping...\n");
      ThisThread::sleep_for(200ms);
      running = false;
      _motion->stopRobot();
    }

    // Move motors
    if (!running)
      continue;

    _motion->moveRobotPWM(v);

    // Update speed
    if (utils::timerRead<chrono::milliseconds>(motor_pid) < 300)
      continue;

    printf("%d ->   ", v);
    printMotorsSpeed();
    motor_pid.reset();
    v += step;

    // Change direction
    if (v < limitSpeed && v > -limitSpeed)
      continue;

    _motion->stopRobot();
    ThisThread::sleep_for(2s);
    v = 0;
    step = -step;
    if (count < 3) {
      count++;
    }

    // Stop if its the first test
    if (count != 2)
      continue;

    printf("Completed first test\n");
    printf("Press PBT1 to start looping");
    running = false;
  }
}

ExecMode Debugger::testIR() {
  Status::send(Color::BLUE, ALL_LEDS);

  // Wait for release
  while (!utils::PBT1)
    ;
  while (!utils::PBT2)
    ;
  ThisThread::sleep_for(10ms); // debounce

  Timer beepTime;
  beepTime.start();
  bool shouldBeep = true;
  ButtonState PBT1State = ButtonState::UNPRESSED;
  ButtonState PBT2State = ButtonState::UNPRESSED;

  printf("Press PBT1 to turn on beep, PBT2 to turn off beep\n");
  while (1) {
    ThisThread::sleep_for(100ms);

    PBT1State = utils::pressedFor(utils::PBT1, 2000);
    PBT2State = utils::pressedFor(utils::PBT2, 2000);

    if (PBT1State == ButtonState::HELD)
      return ExecMode::TEST_KICK;
    if (PBT2State == ButtonState::HELD)
      return ExecMode::TEST_PWM;

    if (PBT1State == ButtonState::RELEASED) {
      shouldBeep = true;
      printf("Turn on beep\n");
      ThisThread::sleep_for(100ms);
    }
    if (PBT2State == ButtonState::RELEASED) {
      shouldBeep = false;
      printf("Turn off beep\n");
      ThisThread::sleep_for(100ms);
    }

    printf("IR ADC: %f   |  with ball: %d\n", _kick->readIR(), (_kick->readIR() < IR_THRESHOLD));

    // Beep
    if (!shouldBeep)
      continue;

    // Detect ball
    if (_kick->readIR() > IR_THRESHOLD)
      continue;

    // Delay between beeps
    if (utils::timerRead<chrono::milliseconds>(beepTime) < 300)
      continue;

    utils::beep(70);
    beepTime.start();
  }
}

long long now() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

void Debugger::testReceiveBall() {
  const double INITIAL_SPEED = 80; // velocidade do dribbler
  const double SECONDS = 1.3;      // segundos para dominar a bola

  const double MICROSECONDS = SECONDS * 1e6;

  bool ir_was_activacted = false;
  double current_speed = -INITIAL_SPEED;
  long long started = now();

  while (true) {
    if (ir_was_activacted) {
      current_speed =
          -std::max<long long>(0,
                               INITIAL_SPEED * (MICROSECONDS - (now() - started)) / MICROSECONDS);
    } else {
      started = now();
    }
    _motion->moveMotorPWM(5, current_speed);
    if (_kick->readIR() < IR_THRESHOLD) {
      ir_was_activacted = true;
    }
  }

  printf("%.3f ", (double) current_speed);
  _motion->printDriblerSpeed();
  ThisThread::sleep_for(70ms);
}

ExecMode Debugger::testRobotPID(bool start, double limitSpeed, uint8_t repeats, uint32_t delay) {
  Status::send(Color::YELLOW, ALL_LEDS);

  // Wait for release
  while (!utils::PBT1)
    ;
  while (!utils::PBT2)
    ;
  ThisThread::sleep_for(10ms); // debounce

  bool running = start;
  int count = 0;
  double v = limitSpeed;
  Timer motor_pid;
  motor_pid.start();
  ButtonState PBT1State = ButtonState::UNPRESSED;
  ButtonState PBT2State = ButtonState::UNPRESSED;

  printf("Press PBT1 to start, PBT2 to stop\n");
  while (1) {
    PBT1State = utils::pressedFor(utils::PBT1, 2000);
    PBT2State = utils::pressedFor(utils::PBT2, 2000);

    if (PBT1State == ButtonState::HELD) {
      _motion->stopRobot();
      return ExecMode::TEST_PWM;
    }
    if (PBT2State == ButtonState::HELD) {
      _motion->stopRobot();
      return ExecMode::TEST_KICK;
    }

    if (PBT1State == ButtonState::RELEASED) {
      running = true;
      motor_pid.reset();
      printf("Starting...\n");
      ThisThread::sleep_for(200ms);
    }
    if (PBT2State == ButtonState::RELEASED) {
      printf("Stopping...\n");
      ThisThread::sleep_for(200ms);
      running = false;
      _motion->stopRobot();
    }

    // Move motors
    if (!running)
      continue;

    Vector desiredVector(0, 0, r_w[count]);

    _motion->accelRobot(desiredVector);
    _motion->printDebugPID(desiredVector, true); // print with debug

    // Update speed
    if (utils::timerRead<chrono::milliseconds>(motor_pid) < delay)
      continue;

    v = -v;
    count++;
    motor_pid.reset();

    if (count == repeats) {
      count = 0;
      running = false;
      _motion->stopRobot();
      ThisThread::sleep_for(200ms);
      printf("Test completed\n");
    }
  }
}

ExecMode Debugger::testKick(bool start) {
  Status::send(Color::GREEN, ALL_LEDS);

  // Wait for release
  while (!utils::PBT1)
    ;
  while (!utils::PBT2)
    ;
  ThisThread::sleep_for(10ms); // debounce

  int kicked = 0;
  KickFlags toKick;
  bool running = start;
  toKick.charge = true;
  toKick.kickStrength = 7.0; // max strength
  float kickLoad = MAX_CHARGE - MAX_CHARGE_MARGIN;
  ButtonState PBT1State = ButtonState::UNPRESSED;
  ButtonState PBT2State = ButtonState::UNPRESSED;

  printf("Press PBT1 to charge and kick, PBT2 to discharge\n");
  while (1) {
    printKickerBasics();
    ThisThread::sleep_for(100ms);

    PBT1State = utils::pressedFor(utils::PBT1, 2000);
    PBT2State = utils::pressedFor(utils::PBT2, 2000);

    if (PBT1State == ButtonState::HELD) {
      printf("Discharging...\n");
      for (int i = 0; i < 5; i++) {
        _kick->discharge();
        ThisThread::sleep_for(500ms);
      }
      return ExecMode::TEST_PID;
    }
    if (PBT2State == ButtonState::HELD) {
      printf("Discharging...\n");
      for (int i = 0; i < 5; i++) {
        _kick->discharge();
        ThisThread::sleep_for(500ms);
      }
      return ExecMode::TEST_IR;
    }

    if (PBT1State == ButtonState::RELEASED) {
      running = true;
    }
    if (PBT2State == ButtonState::RELEASED) {
      printf("Discharging...\n");
      for (int i = 0; i < 5; i++) {
        _kick->discharge();
        ThisThread::sleep_for(500ms);
      }
    }

    // // Kick
    // if (!running) continue;

    printf("Charging...\n");
    while (_kick->getLoad() < kickLoad)
      _kick->update(toKick, DEFAULT_MIN_CHARGE, MAX_CHARGE);
    _kick->stopCharge();
    ThisThread::sleep_for(200ms);

    if (kicked == 0 && _kick->readIR() < IR_THRESHOLD) {
      printf("\n\n >>>>> FRONT <<<<<< \n\n");
      _kick->front(toKick.kickStrength);
      kicked = 1;
    } else if (kicked == 1 && _kick->readIR() < IR_THRESHOLD) {
      printf("\n\n >>>>> CHIP <<<<<< \n\n");
      _kick->chip(toKick.kickStrength);
      kicked = 0;
    }

    running = false;
  }
}

void Debugger::testDribblerPWM(float pwm) {
  pwm = pwm < (-5) ? pwm : DEFAULT_DRIBLER_SPEED;

  while (utils::PBT1)
    ;

  while (1) {
    _motion->moveMotorPWM(5, pwm);

    if (!utils::PBT2) {
      _motion->stopMotor(5);
      ThisThread::sleep_for(100ms);
      while (utils::PBT1)
        ;
    }
    ThisThread::sleep_for(70ms);
  }
}

void Debugger::testOdometry() {
  Timer sampleTime;
  Vector v;
  Vector move;
  unsigned int count = 0;
  unsigned int totalTime = 0;
  v.x = 0.5;
  // v.y = 0.2;
  // v.w = 5;
  while (utils::PBT1)
    ;
  ThisThread::sleep_for(200ms);

  sampleTime.start();
  while (1) {
    _motion->moveRobot(v);
    if (!utils::PBT1) {
      v.x = -v.x;
      v.y = -v.y;
      v.w = -v.w;
      ThisThread::sleep_for(100ms);
    } else if (!utils::PBT2) {
      _motion->stopRobot();
      totalTime = 0;
      while (utils::PBT1)
        ;
      _odometry->updatePosition();
      sampleTime.reset();
    }
    if (utils::timerRead<chrono::milliseconds>(sampleTime) > 500) {
      count++;
      float time = utils::timerRead<chrono::milliseconds>(sampleTime);
      totalTime += time;
      move = v * (totalTime / 1000);
      _motion->printMotorsSpeed();
      printf("Count: %d - Time: %f (ms)\n", count, time);
      printf("Theory: X: %lf- Y: %lf - W: %lf\n", move.x, move.y, move.w);
      _odometry->printMovement();
      printf("\n\n");
      sampleTime.reset();
    }
    Vector currentPos = _odometry->getCurrentPosition();
    if (currentPos.x >= 0.8) {
      totalTime += utils::timerRead<chrono::milliseconds>(sampleTime);
      move = v * (totalTime / 1000.0);
      for (int i = 0; i < 20; i++) {
        _motion->breakRobot(5);
        ThisThread::sleep_for(5ms);
      }
      _motion->stopRobot();
      printf("Theory: X: %lf- Y: %lf - W: %lf\n", move.x, move.y, move.w);
      _odometry->printCurrentPosition();
      printf("\n\n");
      totalTime = 0;
      while (utils::PBT1)
        ;
      _odometry->updatePosition();
      ThisThread::sleep_for(200ms);
      sampleTime.reset();
    }
    ThisThread::sleep_for(5ms);
  }
}

void Debugger::testPosition() {
  Timer sampleTime;
  Vector move;
  unsigned int count = 0;
  unsigned int totalTime = 0;

  while (utils::PBT1)
    ;
  ThisThread::sleep_for(200ms);

  sampleTime.start();
  while (1) {
    _motion->stopRobot();
    if (!utils::PBT1) {
      _motion->stopRobot();
      printf("Theory: X: %lf- Y: %lf - W: %lf\n", move.x, move.y, move.w);
      _odometry->printSpeedIntPosition();
      _odometry->printCurrentPosition();
      printf("\n\n");
      totalTime = 0;
      ThisThread::sleep_for(200ms);
      while (utils::PBT1)
        ;
      _odometry->updatePosition();
      ThisThread::sleep_for(200ms);
      sampleTime.reset();
    } else if (!utils::PBT2) {
      while (utils::PBT1)
        ;
      _odometry->updatePosition();
      sampleTime.reset();
    }
    if (utils::timerRead<chrono::milliseconds>(sampleTime) > 500) {
      count++;
      float time = utils::timerRead<chrono::milliseconds>(sampleTime);
      totalTime += time;
      _motion->printMotorsSpeed();
      _odometry->printIntegralMovement();
      _odometry->printMovement();
      printf("\n\n");
      sampleTime.reset();
    }
    ThisThread::sleep_for(5ms);
  }
}

void Debugger::testMPU() {
  double angle;
  while (1) {
    _odometry->getGyroAngle(angle);
    printf("Angle: %lf\n", angle);
    ThisThread::sleep_for(50ms);
    _odometry->processGyro();
  }
}