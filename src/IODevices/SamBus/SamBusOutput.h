/*
  SamBusOutput.h
  Created for the SAM_IO board, 2026.

  PwmOutput for the SAM_IO board: coils and lamps go to the SAM IO board's
  registers through SamBusFrame instead of to GPIOs.

  Play more pinball!
*/
#ifndef SAMBUS_SamBusOutput_h
#define SAMBUS_SamBusOutput_h

#include "../../EventDispatcher/Event.h"
#include "../PwmOutput.h"
#include "SamBusFrame.h"

class SamBusOutput : public PwmOutput {
 public:
  explicit SamBusOutput(sambus::Frame& frame) : _frame(frame) {}

  void attach(byte type, byte port) override {
    if (type == PWM_TYPE_LAMP) _frame.attachLamp(port);
  }

  void write(byte type, byte port, byte power) override {
    if (type == PWM_TYPE_LAMP) {
      _frame.setLamp(port, power);
    } else {
      // Solenoids, flashers and motors are all SAM coil drivers.
      _frame.setCoil(port, power);
    }
  }

 private:
  sambus::Frame& _frame;
};

#endif
