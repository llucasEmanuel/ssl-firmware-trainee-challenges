#ifndef DEBUGGER_H
#define DEBUGGER_H

#include "utils.h"
#include <MotionControl.h>
#include <Odometry.h>
#include <kicker.h>
#include <mbed.h>
#include <nRF24Communication.h>
#include <CurrentSensor.h>

class Debugger {
 public:
  Debugger(MotionControl* motionControl,
           Odometry* odometry,
           Kicker* kicker,
           nRF24Communication* radio_recv,
           nRF24Communication* radio_send);
  Debugger(MotionControl* motionControl,
           Odometry* odometry,
           Kicker* kicker,
           nRF24Communication* radio_recv,
           nRF24Communication* radio_send,
           CurrentSensor* current_sensor);

  void printRobotBasics();
  void printMotorsSpeed();
  void printKickerBasics();
  void printRadioConnection();
  void printRadiosConnections();
  void printRadioDetails();
  void printRadioReceivedInfos();
  void printMPU();
  void processRobot(ExecMode robotMode);
  void printBattery();
  void printSwitchs();
  void printButtons();

  void getMotorsCurrent(double maxPWM = 30.,
                        double minPWM = 10., 
                        uint8_t step = 10);       

  void runAllTests();
  void testRobotBasics(int v = 30);
  ExecMode testRobotMotors(bool start = false, double limitSpeed = 15, uint8_t repeats = 1);
  ExecMode testRobotPID(bool start = false,
                    double limitSpeed = 40.0,
                    uint8_t repeats = 30,
                    uint32_t delay = 3000);
  ExecMode testKick(bool start = false);
  void testDribblerPWM(float pwm);
  void testOdometry();
  void testPosition();
  void testMPU();
  ExecMode testIR();
  void testReceiveBall();
  void testCurrent();

 private:
  MotionControl* _motion;
  Kicker* _kick;
  Odometry* _odometry;
  nRF24Communication* _radio_recv;
  nRF24Communication* _radio_send;
  CurrentSensor* _current_sensor;

  double r_vx[30] = {-1.8237, -1.0451, 1.3245,  -2.0714, 1.8870,  1.0135,  -0.0501, 0.3455,
                     -1.1560, -0.1811, 2.0376,  0.2059,  0.0930,  -1.1810, -0.0488, 0.5459,
                     0.7882,  -0.4597, -0.5833, 2.1471,  -2.0339, 1.6947,  1.8185,  1.3032,
                     -1.7657, -1.0478, -0.7244, 0.7908,  -1.5992, 0.9734};
  double r_vy[30] = {-1.7302, 0.6765,  -0.0256, 1.2278, 0.9462,  1.7764,  1.7201, -0.7297,
                     0.8745,  -1.3296, -2.0656, 1.0739, 0.0001,  -0.0883, 1.7808, 0.4834,
                     0.5177,  1.5815,  1.3442,  0.3376, -1.3951, -1.1443, 1.7007, -2.0738,
                     -0.0444, -1.4611, 2.1062,  0.9359, 0.0021,  -0.1272};
  double r_w[30] = {-4.4038, 1.8197,  -4.5757, -4.2855, 0.2165,  -4.0327, 3.1815, 3.1755,
                    2.2244,  -3.5013, 1.5961,  0.1859,  4.7297,  1.4899,  3.0033, -0.4620,
                    -0.6761, 3.2531,  -4.1653, -3.6683, -3.2661, -1.0906, 3.3138, 3.0336,
                    -4.3953, -1.0074, 0.2688,  -0.8320, 1.5686,  1.2797};
};

#endif /* DEBUGGER_H */
