/*
  PwmOutput.h
  Created for the SAM_IO board, 2026.

  Where PwmDevices sends an output's power.

  Every board so far drives its outputs from its own GPIOs, so PwmDevices
  called analogWrite() directly. The SAM_IO board has no output stage of its
  own: it forwards every coil, flasher and lamp to a Stern SAM IO board over
  that board's parallel bus. This interface is the seam between the two. The
  pulse envelope, fast flips and stop switches stay in PwmDevices for every
  board; only the last step, "set this output to this power", changes.

  Play more pinball!
*/
#ifndef OUTPUT_PwmOutput_h
#define OUTPUT_PwmOutput_h

#include <Arduino.h>

class PwmOutput {
 public:
  virtual ~PwmOutput() {}
  // Called once when an output is registered, before any write.
  virtual void attach(byte type, byte port) = 0;
  // Power 0-255 for the output `port` of PWM_TYPE_* `type`. The type matters
  // where the same port number names two different outputs, as a SAM coil and
  // a SAM lamp do.
  virtual void write(byte type, byte port, byte power) = 0;
};

// The board's own GPIOs, through the Arduino core's PWM.
class GpioPwmOutput : public PwmOutput {
 public:
  void attach(byte type, byte port) override {
    (void)type;
    pinMode(port, OUTPUT);
    analogWrite(port, 0);
  }
  void write(byte type, byte port, byte power) override {
    (void)type;
    analogWrite(port, power);
  }
};

#endif
