#include "QEInterruption.h"

#include "mbed.h"
#include "utils.h"

QEInterruption::QEInterruption(PinName channelA,
                               PinName channelB,
                               int pulsesPerRev,
                               int tsample,
                               Encoding encoding) :
    channelA_(channelA),
    channelB_(channelB) {
  pulses_ = 0;
  pulsesPerRev_ = pulsesPerRev;
  encoding_ = encoding;
  tsample_ = tsample;
}

void QEInterruption::init(void) {
  // Workout what the current state is.
  int chanA = channelA_.read();
  int chanB = channelB_.read();

  // 2-bit state.
  currState_ = (chanA << 1) | (chanB);
  prevState_ = currState_;

  // X1 encoders uses rise interrupts on channel A, and X2 or tacho/diro rise and fall.
  channelA_.rise(callback(this, &QEInterruption::encode));
  if (encoding_ != X1_ENCODING)
    channelA_.fall(callback(this, &QEInterruption::encode));

  /*  If we're using X4 encoding, then attach interrupts to channel B.
   *   Remember of setting channelB_ as Interrupt Pin (on .h).
   */
  // if (encoding_ == X4_ENCODING)
  // {
  //     channelB_.rise(callback(this, &QEInterruption::encode));
  //     channelB_.fall(callback(this, &QEInterruption::encode));
  // }
}

void QEInterruption::frequency(void) {
  if (encoding_ == TAC_DIRO_ENCODING) {
    frequency_ = double((this->getPulses() * (1.0 / tsample_)) / (1.0 * pulsesPerRev_));
  } else if (encoding_ == X1_ENCODING) {
    frequency_ = double((this->getPulses() * (1.0 / tsample_)) / (1.0 * pulsesPerRev_));
  } else if (encoding_ == X2_ENCODING) {
    frequency_ = double((this->getPulses() * (1.0 / tsample_)) / (2.0 * pulsesPerRev_));
  }
  // else if (encoding_ == X4_ENCODING)
  //     frequency_ = double((this->getPulses() * (1.0 / tsample_)) / (4.0 * pulsesPerRev_));
  // utils::pc.printf("Pulses %d\n", this->getPulses());
  this->resetPulses();
}

double QEInterruption::getFrequency(void) {
  return frequency_;
}

double QEInterruption::getSpeed() {
  return 2.0 * M_PI * (DRIBLER_BAR_GEAR / DRIBLER_GEAR) * this->getFrequency();
}

void QEInterruption::resetPulses(void) {
  pulses_ = 0;
}

int QEInterruption::getCurrentState(void) {

  return currState_;
}

int QEInterruption::getPulses(void) {
  return pulses_;
}

void QEInterruption::encode(void) {
  // int change = 0;
  int chanA = channelA_.read();
  int chanB = channelB_.read();
  currState_ = (chanA << 1) | (chanB);

  if (encoding_ == TAC_DIRO_ENCODING) {
    if (chanB)
      pulses_++; // 11->01->11->01 is counter clockwise rotation or "forward"
    else
      pulses_--; // 10->00->10->00 is clockwise rotation or "backward"
  } else if (encoding_ == X1_ENCODING) {
    if (chanB)
      pulses_--;
    else
      pulses_++;
  } else if (encoding_ == X2_ENCODING) {
    // 11->00->11->00 is counter clockwise rotation or "forward".
    if ((prevState_ == 0x3 && currState_ == 0x0) || (prevState_ == 0x0 && currState_ == 0x3)) {
      pulses_++;
    }
    // 10->01->10->01 is clockwise rotation or "backward".
    else if ((prevState_ == 0x2 && currState_ == 0x1) || (prevState_ == 0x1 && currState_ == 0x2)) {
      pulses_--;
    }
  }

  // else if (encoding_ == X4_ENCODING) {
  //     //Entered a new valid state.
  //     if (((currState_ ^ prevState_) != INVALID) && (currState_ != prevState_)) {
  //         //2 bit state. Right hand bit of prev XOR left hand bit of current
  //         //gives 0 if clockwise rotation and 1 if counter clockwise rotation.
  //         change = (prevState_ & PREV_MASK) ^ ((currState_ & CURR_MASK) >> 1);
  //         if (change == 0) {
  //             change = -1;
  //         }
  //         pulses_ -= change;
  //     }
  // }

  prevState_ = currState_;
  return;
}
