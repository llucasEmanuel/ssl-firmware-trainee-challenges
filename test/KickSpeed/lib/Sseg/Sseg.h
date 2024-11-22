/*
  SSeg.cpp - mbed library for 2/4/8 digit seven segment LED driver.
  Copyright 2013,2014,2015 by morecat_lab

  based on Dots library.
  Copyright 2010 arms22. All right reserved.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#ifndef SSEG_H
#define SSEG_H

#include "mbed.h"
#include <Timer.h>

#define NUM_PAT_0 0xfc
#define NUM_PAT_1 0x60
#define NUM_PAT_2 0xda
#define NUM_PAT_3 0xf2
#define NUM_PAT_4 0x66
#define NUM_PAT_5 0xb6
#define NUM_PAT_6 0xbe
#define NUM_PAT_7 0xe0
#define NUM_PAT_8 0xfe
#define NUM_PAT_9 0xf6
#define NUM_PAT_A 0xee
#define NUM_PAT_B 0x3e
#define NUM_PAT_C 0x9c
#define NUM_PAT_D 0x7a
#define NUM_PAT_E 0x9e
#define NUM_PAT_F 0x8e

/**
 * Seven segment LED driver library
 */
class Sseg {
 private:
  unsigned long _lastUpdateTime;
  int _updateInterval;
  DigitalOut* _segPins[8]; // changed to use array
  DigitalOut* _digPins[8]; // changed to use array
  char _buffer[8];
  int _numOfDigs;
  int _dig; // support 4 or 8
  bool _zeroSupress;
  bool _kcommon;    // Cathode-common flag
  bool _sinkDriver; // invert segment drive, if true
  void initConv(void);
  Timer timer;

 public:
  static const int numConv[16];
  /**
   * create an 2 digit seven segment driver
   *
   * @param PinName a Pin No for segment A
   * @param PinName b Pin No for segment B
   * @param PinName c Pin No for segment C
   * @param PinName d Pin No for segment D
   * @param PinName e Pin No for segment E
   * @param PinName f Pin No for segment F
   * @param PinName g Pin No for segment G
   * @param PinName dp Pin No for segment DP
   * @param PinName d1 Pin No for dight 1
   * @param PinName d2 Pin No for dight 2
     */
  Sseg(PinName a,
       PinName b,
       PinName c,
       PinName d,
       PinName e,
       PinName f,
       PinName g,
       PinName dp,
       PinName d1,
       PinName d2);

  /**
   * create an 4 digit seven segment driver
   *
   * @param PinName a Pin No for segment A
   * @param PinName b Pin No for segment B
   * @param PinName c Pin No for segment C
   * @param PinName d Pin No for segment D
   * @param PinName e Pin No for segment E
   * @param PinName f Pin No for segment F
   * @param PinName g Pin No for segment G
   * @param PinName dp Pin No for segment DP
   * @param PinName d1 Pin No for dight 1
   * @param PinName d2 Pin No for dight 2
   * @param PinName d3 Pin No for dight 3
   * @param PinName d4 Pin No for dight 4
   */
  Sseg(PinName a,
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
       PinName d4);

  /**
   * create an 8 digit seven segment driver
   *
   * @param PinName a Pin No for segment A
   * @param PinName b Pin No for segment B
   * @param PinName c Pin No for segment C
   * @param PinName d Pin No for segment D
   * @param PinName e Pin No for segment E
   * @param PinName f Pin No for segment F
   * @param PinName g Pin No for segment G
   * @param PinName dp Pin No for segment DP
   * @param PinName d1 Pin No for dight 1
   * @param PinName d2 Pin No for dight 2
   * @param PinName d3 Pin No for dight 3
   * @param PinName d4 Pin No for dight 4
   * @param PinName d5 Pin No for dight 5
   * @param PinName d6 Pin No for dight 6
   * @param PinName d7 Pin No for dight 7
   * @param PinName d8 Pin No for dight 8
   */
  Sseg(PinName a,
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
       PinName d8);

  /**
   * start driver
   */
  void begin(void);

  /**
   * use Kathode Common LED
   */
  void setKcommon(void);

  /**
   * use Anode Common LED (default)
   */
  void setAcommon(void);

  /**
   * sink driver
   */
  void setSinkDriver(void);

  /**
   * get a charcter pattern from a number
   *
   * @param i number
   *
   * @returns bit pattern of number i
   *
   */
  char segCh(char i);

  /**
   * turn on DP
   *
   * @param d dight
   *
   */
  void setDot(int d);

  /**
   * turn off DP
   *
   * @param d dight
   *
   */
  void clearDot(int d);

  /**
   * write a number to LED
   *
   * @param d number
   *
   */
  void writeNum(int n);

  /**
   * write a number to 2 dight LED
   *
   * @param n number
   *
   */
  void writeNum2(int n);

  /**
   * write a number to 4 dight LED
   *
   * @param n number
   *
   */
  void writeNum4(int n);

  /**
   * write a number to 8 dight LED
   *
   * @param n number
   *
   */
  void writeNum8(int n);

  /**
   * write numbers to each dight of 2 dight LED
   *
   * @param d1 digit 1 number
   * @param d2 digit 2 number
   *
   */
  void writeNum(char d1, char d2);

  /**
   * write numbers to each dight of 4 dight LED
   *
   * @param d1 digit 1 number
   * @param d2 digit 2 number
   * @param d3 digit 3 number
   * @param d4 digit 4 number
   *
   */
  void writeNum(char d1, char d2, char d3, char d4);

  /**
   * write numbers to each dight of 8 dight LED
   *
   * @param d1 digit 1 number
   * @param d2 digit 2 number
   * @param d3 digit 3 number
   * @param d4 digit 4 number
   * @param d5 digit 5 number
   * @param d6 digit 6 number
   * @param d7 digit 7 number
   * @param d8 digit 8 number
   *
   */
  void writeNum(char d1, char d2, char d3, char d4, char d5, char d6, char d7, char d8);

  /**
   * zero supress: tell driver not to display 0 in the left
   *
   */
  void supressZero();

  /**
   * control zero supress bit
   *
   * @param t, 1:supress on, 0:supress off
   *
   */
  void setZeroSupress(bool t);

  /**
   * write hex number to LED
   *
   * @param n number
   *
   */
  void writeHex(int n);

  /**
   * write hex number to LED
   *
   * @param n (long)number
   *
   */
  void writeHex(long n);

  /**
   * write patterns to each dight of 2 dight LED
   *
   * @param d1 digit 1 pattern
   * @param d2 digit 2 pattern
   *
   */
  void writeRawData(char d1, char d2);

  /**
   * write patterns to each dight of 4 dight LED
   *
   * @param d1 digit 1 pattern
   * @param d2 digit 2 pattern
   * @param d3 digit 3 pattern
   * @param d4 digit 4 pattern
   *
   */
  void writeRawData(char d1, char d2, char d3, char d4);

  /**
   * write patterns to each dight of 8 dight LED
   *
   * @param d1 digit 1 pattern
   * @param d2 digit 2 pattern
   * @param d3 digit 3 pattern
   * @param d4 digit 4 pattern
   * @param d5 digit 5 pattern
   * @param d6 digit 6 pattern
   * @param d7 digit 7 pattern
   * @param d8 digit 8 pattern
   *
   */
  void writeRawData(char d1, char d2, char d3, char d4, char d5, char d6, char d7, char d8);

  void write(uint8_t x, uint8_t y, uint8_t value);

  /**
   * write patterns to a dight
   *
   * @param d digit
   *
   * @param value pattern
   *
   */
  void write(uint8_t d, uint8_t value);
  void write(uint8_t y, const uint8_t values[], uint8_t size);

  /**
   * Clear LED buffer
   */
  void clear(void);

  /**
   * Turn off LED
   */
  void turnOff(void);

  /**
   * Turn on LED
   */
  void turnOn(void);

  /**
   * Update One dight of LED
   */
  void updateSeg(void);

  /**
   * Update LED (by internal clock)
   *
   * @returns sync = 1, if digit == 0
   *
   */
  bool update(void);

  /**
   * keep updating LED for specified period
   *
   * @param ms period (ms)
   *
   */
  void updateWithDelay(int ms);

  /**
   * Update LED Once with 1ms delay
   */
  void updateOnce(void);
};

#endif // SSEG.h
