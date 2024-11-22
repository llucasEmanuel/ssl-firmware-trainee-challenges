#ifndef Encoder_H
#define Encoder_H

#include "mbed.h"
#include "utils.h"

/**
 * Quadrature Encoder Interface.
 */
class Encoder {
  typedef enum Encoding { X2_ENCODING, X4_ENCODING } Encoding;

 public:
  virtual void init(void);

  virtual void resetPulses(void);

  virtual int getPulses(void);

  virtual double getFrequency(void);

  virtual double getSpeed();

  virtual void frequency(void);
};

#endif /* Encoder_H */
