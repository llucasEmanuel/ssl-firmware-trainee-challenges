#include <mbed.h>
#include <Sseg.h>

Serial pc(USBTX, USBRX);
Timer delta;

DigitalOut irLed_1(p15);
AnalogIn irAdc_1(p16);

DigitalOut irLed_2(p17);
AnalogIn irAdc_2(p18);

// 7seg
#define pinA  p19
#define pinB  p20
#define pinC  p21
#define pinD  p22
#define pinE  p23
#define pinF  p24
#define pinG  p25
#define pinDP p30
#define pinD1 p26
#define pinD2 p27
#define pinD3 p28
#define pinD4 p29

Sseg Setseg = Sseg(pinA, pinB, pinC, pinD, pinE, pinF, pinG, pinDP, pinD1, pinD2, pinD3, pinD4);

int main() {
  irLed_1.write(1);
  irLed_2.write(1);
  double delay;
  double speed;
  int speedToDisplay;
  Setseg.begin();

  Setseg.writeNum(0);
  while (1) {

    while (irAdc_1.read() > 0.01) {
      Setseg.update();
    }
    delta.start();

    while (irAdc_2.read() > 0.01) {
      Setseg.update();
    }
    delay = delta.read_ms();
    delay = delay / 1000;
    speed = (0.1 / delay);
    speedToDisplay = (int) (speed * 1000);

    pc.printf("Speed(m/s): %f \n", speed);

    // overflow case
    if (speedToDisplay > 9999) {
      Setseg.writeRawData(NUM_PAT_F, NUM_PAT_F, NUM_PAT_F, NUM_PAT_F);
    } else {
      Setseg.writeNum(speedToDisplay);
      Setseg.setDot(0);
    }

    delta.stop();
    delta.reset();
  }
}
