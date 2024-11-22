
//    --- TODO: Test it at the Dribbler Motor !!!!! ----

#ifndef QEInterruption_H
#define QEInterruption_H

#include "Encoder.h"

/* Defines */
#define PREV_MASK 0x1 // Mask for the previous state in determining direction
#define CURR_MASK 0x2 // Mask for the current state in determining direction
#define INVALID   0x3 // XORing two states where both bits have changed.

/* Quadrature Encoder Interruption */
class QEInterruption : public Encoder {

 public:
  typedef enum Encoding { TAC_DIRO_ENCODING, X1_ENCODING, X2_ENCODING, X4_ENCODING } Encoding;

  /**
   * Reads the current values on channel A and channel B to determine the initial state.
   *
   * Attaches the encode function to the rise/fall interrupt edges of channels A and B.
   *
   * Attaches the frequenci callback function.
   *
   * @param channelA mbed pin for channel A input.
   * @param channelB mbed pin for channel B input.
   * @param pulsesPerRev Number of pulses in one revolution.
   * @param encoding The encoding to use.
   */

  QEInterruption(PinName channelA,
                 PinName channelB,
                 int pulsesPerRev,
                 int tsample,
                 Encoding encoding = TAC_DIRO_ENCODING);

  void init(void);

  /* Reset the encoder:  Sets the pulses and revolutions count to zero. */
  void resetPulses(void);

  /* Read the state of the encoder.
   *
   * @return The current state of the encoder as a 2-bit number, where:
   *         bit 1 = The reading from channel B
   *         bit 2 = The reading from channel A */
  int getCurrentState(void);

  /* Read the number of pulses recorded by the encoder.
   * @return Number of pulses which have occured.
   */
  int getPulses(void);

  /* @return pre-calculated Frequency in Hz. */
  double getFrequency(void);

  /* @return Speed of wheel in rad/s by calculated frequency. */
  double getSpeed();

  /* @return Calculates the wheel frequency based on encoded pulses. */
  void frequency(void);

 private:
  /* Called on every rising/falling edge of channels A/B.
   *
   * Determines whether a pulse forward or backward has occured.
   */
  void encode(void);

  Encoding encoding_;

  Ticker freqTicker_;

  InterruptIn channelA_;
  DigitalIn channelB_;

  int pulsesPerRev_;
  int prevState_;
  int currState_;

  volatile int pulses_;
  volatile double frequency_;
  int tsample_;
};

#endif /* QEI_H */
