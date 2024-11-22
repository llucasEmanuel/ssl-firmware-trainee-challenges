/*
 * Quadrature Encoder Interface.

 * This interface can also use X4 encoding which calculates the pulse count
 * based on reading the current state after each rising and falling edge of
 * either channel.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |     |     |     |     |     |
 *            ---+     +-----+     +-----+     +-----
 *               ^     ^     ^     ^     ^
 *               ^  +-----+  ^  +-----+  ^  +-----+
 * Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
 *            ------+  ^  +-----+  ^  +-----+     +--
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 */

#ifndef QEI_H
#define QEI_H

#include "Encoder.h"

#define PULSES_RESET 32767

/**
 * Quadrature Encoder Interface.
 */
class QEI : public Encoder {

 public:
  QEI(int pulsesPerRev, int tsample, TIM_TypeDef* timer);

  /**
   * Init the encoder.
   */
  void init(void);

  /**
   * Reset the encoder: sets the pulses and revolutions count to zero.
   */
  void resetPulses(void);

  /**
   * @return Number of pulses which have occured in tSample.
   */
  int getPulses(void);

  /**
   * @return Frequency in Hz.
   */
  double getFrequency(void);

  /**
   * @return Wheel speed.
   */
  double getSpeed();

  /* Update the rotation frequecy count */
  void frequency(void);

 private:
  static void init_enc_timer(TIM_TypeDef* timer);
  static void init_enc_timer1(); // PE_9 PA_9 - Encoder of M1
  static void init_enc_timer2(); // PA_5 PB_3 - Encoder of M2
  static void init_enc_timer8(); // PC_6 PC_7 - Encoder of M3
  static void init_enc_timer3(); // PA_6 PB_5 - Encoder of M4

  TIM_TypeDef* timer_;
  int pulsesPerRev_;

  volatile int pulses_;
  volatile double frequency_;
  int tsample_;
};

#endif /* QEI_H */
