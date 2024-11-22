/*
  Sseg.cpp - mbed library for 2/4/8 digit seven segment LED driver.
  Copyright 2013,2014,2015 by morecat_lab

  base on Dots library.
  Copyright 2010 arms22. All right reserved.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "Sseg.h"
#include <Timer.h>

const int Sseg::numConv[] = {NUM_PAT_0,
                             NUM_PAT_1,
                             NUM_PAT_2,
                             NUM_PAT_3,
                             NUM_PAT_4,
                             NUM_PAT_5,
                             NUM_PAT_6,
                             NUM_PAT_7,
                             NUM_PAT_8,
                             NUM_PAT_9,
                             NUM_PAT_A,
                             NUM_PAT_B,
                             NUM_PAT_C,
                             NUM_PAT_D,
                             NUM_PAT_E,
                             NUM_PAT_F};

// 2 digit
Sseg::Sseg(PinName a,
           PinName b,
           PinName c,
           PinName d,
           PinName e,
           PinName f,
           PinName g,
           PinName dp,
           PinName d1,
           PinName d2) {
  _numOfDigs = 2;
  PinName segPins[8] = {a, b, c, d, e, f, g, dp};
  PinName digPins[2] = {d1, d2};

  for (int i = 0; i < 8; i++) {
    _segPins[i] = new DigitalOut(segPins[i]);
  }

  for (int i = 0; i < _numOfDigs; i++) {
    _digPins[i] = new DigitalOut(digPins[i]);
  }

  _updateInterval = (8333 / 2);
  _zeroSupress = true;
  _kcommon = false;
  _sinkDriver = false;
}

// 4 digit
Sseg::Sseg(PinName a,
           PinName b,
           PinName c,
           PinName d,
           PinName e,
           PinName f,
           PinName g,
           PinName dp,
           PinName d1,
           PinName d2,
           PinName d3,
           PinName d4) {
  _numOfDigs = 4;
  PinName segPins[8] = {a, b, c, d, e, f, g, dp};
  PinName digPins[4] = {d1, d2, d3, d4};

  for (int i = 0; i < 8; i++) {
    _segPins[i] = new DigitalOut(segPins[i]);
  }

  for (int i = 0; i < _numOfDigs; i++) {
    _digPins[i] = new DigitalOut(digPins[i]);
  }
  _updateInterval = (8333 / 4);
  _zeroSupress = true;
  _kcommon = false;
  _sinkDriver = false;
}

// 8 digit
Sseg::Sseg(PinName a,
           PinName b,
           PinName c,
           PinName d,
           PinName e,
           PinName f,
           PinName g,
           PinName dp,
           PinName d1,
           PinName d2,
           PinName d3,
           PinName d4,
           PinName d5,
           PinName d6,
           PinName d7,
           PinName d8) {
  _numOfDigs = 8;
  PinName segPins[8] = {a, b, c, d, e, f, g, dp};
  PinName digPins[8] = {d1, d2, d3, d4, d5, d6, d7, d8};

  for (int i = 0; i < 8; i++) {
    _segPins[i] = new DigitalOut(segPins[i]);
  }

  for (int i = 0; i < _numOfDigs; i++) {
    _digPins[i] = new DigitalOut(digPins[i]);
  }
  _updateInterval = (8333 / 8);
  _zeroSupress = true;
  _kcommon = false;
  _sinkDriver = false;
}

void Sseg::begin(void) {
  timer.start();
  clear();
}

void Sseg::setAcommon(void) {
  _kcommon = false;
}

void Sseg::setKcommon(void) {
  _kcommon = true;
}

void Sseg::setSinkDriver(void) {
  _sinkDriver = true;
}

char Sseg::segCh(char i) {
  return Sseg::numConv[i];
}

void Sseg::setDot(int d) {
  _buffer[d] |= 0x01;
}

void Sseg::clearDot(int d) {
  _buffer[d] &= 0xfe;
}

void Sseg::writeNum(int n) {
  if (_numOfDigs == 2) {
    writeNum2(n);
  } else if (_numOfDigs == 4) {
    writeNum4(n);
  } else if (_numOfDigs == 8) {
    writeNum8((long) n);
  }
}

void Sseg::writeNum2(int n) {
  if (n < 100) {
    _buffer[0] = segCh((n % 100) / 10);
    _buffer[1] = segCh(n % 10);
  } else {
    _buffer[0] = _buffer[1] = 0x02; // overflow
  }
}

void Sseg::writeNum4(int n) {
  if (n < 10000) {
    _buffer[0] = segCh((n % 10000) / 1000);
    _buffer[1] = segCh((n % 1000) / 100);
    _buffer[2] = segCh((n % 100) / 10);
    _buffer[3] = segCh(n % 10);
  } else {
    _buffer[0] = _buffer[1] = _buffer[2] = _buffer[3] = 0x02; // overflow
  }
}

void Sseg::writeNum8(int n) {
  _buffer[0] = segCh((n % 100000000) / 10000000);
  _buffer[1] = segCh((n % 10000000) / 1000000);
  _buffer[2] = segCh((n % 1000000) / 100000);
  _buffer[3] = segCh((n % 100000) / 10000);
  _buffer[4] = segCh((n % 10000) / 1000);
  _buffer[5] = segCh((n % 1000) / 100);
  _buffer[6] = segCh((n % 100) / 10);
  _buffer[7] = segCh(n % 10);
}

void Sseg::writeNum(char d1, char d2) {
  _buffer[0] = segCh(d1);
  _buffer[1] = segCh(d2);
}

void Sseg::writeNum(char d1, char d2, char d3, char d4) {
  _buffer[0] = segCh(d1);
  _buffer[1] = segCh(d2);
  _buffer[2] = segCh(d3);
  _buffer[3] = segCh(d4);
}

void Sseg::writeNum(char d1, char d2, char d3, char d4, char d5, char d6, char d7, char d8) {
  _buffer[0] = segCh(d1);
  _buffer[1] = segCh(d2);
  _buffer[2] = segCh(d3);
  _buffer[3] = segCh(d4);
  _buffer[4] = segCh(d5);
  _buffer[5] = segCh(d6);
  _buffer[6] = segCh(d7);
  _buffer[7] = segCh(d8);
}

void Sseg::writeHex(int n) {
  if (_numOfDigs == 2) {
    _buffer[0] = segCh((n >> 4) & 0xf);
    _buffer[1] = segCh(n & 0xf);
  } else if (_numOfDigs == 4) {
    _buffer[0] = segCh((n >> 12) & 0xf);
    _buffer[1] = segCh((n >> 8) & 0xf);
    _buffer[2] = segCh((n >> 4) & 0xf);
    _buffer[3] = segCh(n & 0xf);
  }
}

void Sseg::writeHex(long n) {
  _buffer[0] = segCh((n >> 28) & 0xf);
  _buffer[1] = segCh((n >> 24) & 0xf);
  _buffer[2] = segCh((n >> 20) & 0xf);
  _buffer[3] = segCh((n >> 16) & 0xf);
  _buffer[4] = segCh((n >> 12) & 0xf);
  _buffer[5] = segCh((n >> 8) & 0xf);
  _buffer[6] = segCh((n >> 4) & 0xf);
  _buffer[7] = segCh(n & 0xf);
}

void Sseg::setZeroSupress(bool t) {
  _zeroSupress = t;
}

void Sseg::supressZero() {
  int i;
  if (_zeroSupress) {
    for (i = 0; i < (_numOfDigs - 1); i++) {
      if (_buffer[i] == segCh(0)) {
        _buffer[i] = _buffer[i] & 0x1;
      } else {
        break;
      }
    }
  }
}

void Sseg::writeRawData(char d1, char d2) {
  _buffer[0] = d1;
  _buffer[1] = d2;
}

void Sseg::writeRawData(char d1, char d2, char d3, char d4) {
  _buffer[0] = d1;
  _buffer[1] = d2;
  _buffer[2] = d3;
  _buffer[3] = d4;
}

void Sseg::writeRawData(char d1, char d2, char d3, char d4, char d5, char d6, char d7, char d8) {
  _buffer[0] = d1;
  _buffer[1] = d2;
  _buffer[2] = d3;
  _buffer[3] = d4;
  _buffer[4] = d5;
  _buffer[5] = d6;
  _buffer[6] = d7;
  _buffer[7] = d8;
}

void Sseg::write(uint8_t x, uint8_t y, uint8_t value) {
  uint8_t tmp, msk;
  tmp = _buffer[y];
  msk = 0x80 >> x;
  tmp = tmp & ~msk;
  tmp = tmp | (value ? msk : 0);
  _buffer[y] = tmp;
}

void Sseg::write(uint8_t d, uint8_t value) {
  _buffer[d] = value;
}

void Sseg::write(uint8_t y, const uint8_t values[], uint8_t size) {
  uint8_t i;
  for (i = 0; i < size; i++)
    _buffer[(y++) & 0x07] = values[i];
}

void Sseg::clear(void) {
  int i;
  for (i = 0; i < 8; i++) {
    _buffer[i] = 0;
  }
  _dig = _numOfDigs - 1;
}

void Sseg::turnOff(void) {
  _digPins[_dig]->write((_kcommon) ? 1 : 0);
}

void Sseg::turnOn(void) {
  _digPins[_dig]->write((_kcommon) ? 0 : 1);
}

void Sseg::updateSeg(void) {
  char i, data, mask;
  if ((++_dig) >= _numOfDigs)
    _dig = 0;
  data = _buffer[_dig];
  mask = 0x80;
  for (i = 0; i < 8; i++) {
    if (data & mask) {
      _segPins[i]->write(((_kcommon) || (_sinkDriver)) ? 1 : 0);
    } else {
      _segPins[i]->write(((_kcommon) || (_sinkDriver)) ? 0 : 1);
    }
    mask >>= 1;
  }
}

bool Sseg::update(void) {
  int t = timer.read_us();
  bool sync = false;
  if ((t - _lastUpdateTime) > _updateInterval) {
    turnOff();
    updateSeg();
    turnOn();
    _lastUpdateTime = t;
    sync = (_dig == 0);
  }
  return sync;
}

void Sseg::updateWithDelay(int ms) {
  timer.reset(); // to avoid overflow 32bit counter  (~=30min)
  int start = timer.read_ms();
  do {
    bool sync = update();
    if (sync) {
      int t = timer.read_ms();
      if ((t - start) >= ms) {
        break;
      }
    }
  } while (1);
}

void Sseg::updateOnce(void) {
  uint8_t i;
  _dig = _numOfDigs - 1;
  turnOff();
  for (i = 0; i < _numOfDigs; i++) {
    updateSeg();
    turnOn();
    wait_us(1000); // wait 1ms
    turnOff();
  }
}

// EOF
