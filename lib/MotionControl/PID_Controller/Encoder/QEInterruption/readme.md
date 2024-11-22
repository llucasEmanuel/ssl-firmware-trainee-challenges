
 
#Quadrature Encoder Interface.
 
  A quadrature encoder consists of two code tracks on a disc which are 90
  degrees out of phase. It can be used to determine how far a wheel has
  rotated, relative to a known starting position.
 
  Only one code track changes at a time leading to a more robust system than
  a single track, because any jitter around any edge won't cause a state
  change as the other track will remain constant.
 
  Encoders can be a homebrew affair, consisting of infrared emitters/receivers
  and paper code tracks consisting of alternating black and white sections;
  alternatively, complete disk and PCB emitter/receiver encoder systems can
  be bought, but the interface, regardless of implementation is the same.
 
                +-----+     +-----+     +-----+
  Channel A     |  ^  |     |     |     |     |
             ---+  ^  +-----+     +-----+     +-----
                ^  ^
                ^  +-----+     +-----+     +-----+
  Channel B     ^  |     |     |     |     |     |
             ------+     +-----+     +-----+     +-----
                ^  ^
                ^  ^
                90deg
 
  The interface uses X2 encoding by default which calculates the pulse count
  based on reading the current state after each rising and falling edge of
  channel A.
 
                +-----+     +-----+     +-----+
  Channel A     |     |     |     |     |     |
             ---+     +-----+     +-----+     +-----
                ^     ^     ^     ^     ^
                ^  +-----+  ^  +-----+  ^  +-----+
  Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
             ------+  ^  +-----+  ^  +-----+     +--
                ^     ^     ^     ^     ^
                ^     ^     ^     ^     ^
  Pulse count 0 1     2     3     4     5  ...
 
  This interface can also use X4 encoding which calculates the pulse count
  based on reading the current state after each rising and falling edge of
  either channel.
 
                +-----+     +-----+     +-----+
  Channel A     |     |     |     |     |     |
             ---+     +-----+     +-----+     +-----
                ^     ^     ^     ^     ^
                ^  +-----+  ^  +-----+  ^  +-----+
  Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
             ------+  ^  +-----+  ^  +-----+     +--
                ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
                ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
  Pulse count 0 1  2  3  4  5  6  7  8  9  ...
 
  It defaults
 
  An optional index channel can be used which determines when a full
  revolution has occured.
 
  If a 4 pules per revolution encoder was used, with X4 encoding,
  the following would be observed.
 
                +-----+     +-----+     +-----+
  Channel A     |     |     |     |     |     |
             ---+     +-----+     +-----+     +-----
                ^     ^     ^     ^     ^
                ^  +-----+  ^  +-----+  ^  +-----+
  Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
             ------+  ^  +-----+  ^  +-----+     +--
                ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
                ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
                ^  ^  ^  +--+  ^  ^  +--+  ^
                ^  ^  ^  |  |  ^  ^  |  |  ^
  Index      ------------+  +--------+  +-----------
                ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
  Pulse count 0 1  2  3  4  5  6  7  8  9  ...
  Rev.  count 0          1           2
 
 
  Linear position can be calculated by:
 
  (pulse count / X  N)  (1 / PPI)
 
  Where X is encoding type [e.g. X4 encoding => X=44], N is the number of
  pulses per revolution, and PPI is pulses per inch, or the equivalent for
  any other unit of displacement. PPI can be calculated by taking the
  circumference of the wheel or encoder disk and dividing it by the number
  of pulses per revolution.

#How it Works
 +-------------+
| TAC_DIRO Encoding |
+-------------+

Counter clockwise rotation:

11 -> 01 -> 11 -> 01  ->  ...

Clockwise rotation:

10 -> 00 -> 10 -> 00 ->...

 +-------------+
| X1 Encoding |
+-------------+

Counter clockwise rotation:

10 -> 10 -> ...

Clockwise rotation:

11 ->  11 ->...

  +-------------+
| X2 Encoding |
+-------------+

When observing states two patterns will appear:

Counter clockwise rotation:

10 -> 01 -> 10 -> 01 -> ...

Clockwise rotation:

11 -> 00 -> 11 -> 00 -> ...

We consider counter clockwise rotation to be "forward" and
counter clockwise to be "backward". Therefore pulse count will increase
during counter clockwise rotation and decrease during clockwise rotation.

+-------------+
| X4 Encoding |
+-------------+

There are four possible states for a quadrature encoder which correspond to
2-bit gray code.

A state change is only valid if of only one bit has changed.
A state change is invalid if both bits have changed.

Clockwise Rotation ->

   00 01 11 10 00

<- Counter Clockwise Rotation

If we observe any valid state changes going from left to right, we have
moved one pulse clockwise [we will consider this "backward" or "negative"].

If we observe any valid state changes going from right to left we have
moved one pulse counter clockwise [we will consider this "forward" or
"positive"].
