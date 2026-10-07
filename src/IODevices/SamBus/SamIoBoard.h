/*
  SamIoBoard.h
  Created for the SAM_IO board, 2026.

  The SAM_IO board's single hook into IOBoardController. Everything SAM_IO
  adds lives in this folder; the rest of the firmware is the stock PPUC
  io-boards code. See README.md here for the full list of touch points.

  Play more pinball!
*/
#ifndef SAMBUS_SamIoBoard_h
#define SAMBUS_SamIoBoard_h

#include "../../EventDispatcher/Event.h"
#include "../PwmDevices.h"
#include "SamBusDriver.h"
#include "SamBusOutput.h"

class SamIoBoard {
 public:
  // Starts the SAM bus and points PwmDevices at it. Called before anything
  // else claims a PIO, so the lamp strobes (and the IO board's watchdog) run
  // from boot, with every output off until the host configures them.
  // Returns false if no PIO state machine or DMA channel is free.
  bool begin(PwmDevices* pwmDevices) {
    if (!pwmDevices || !_driver.begin()) return false;
    pwmDevices->setOutput(&_output);
    _running = true;
    return true;
  }

  // Whether an output of this PWM type can be registered. The port itself is
  // checked by Profile::allowsOutput(). The shaker is an effect on a GPIO,
  // which this board does not have.
  bool accepts(byte pwmType) const {
    return _running && pwmType != PWM_TYPE_SHAKER;
  }

  sambus::Driver& driver() { return _driver; }

 private:
  sambus::Driver _driver;
  SamBusOutput _output{_driver.frame()};
  bool _running = false;
};

#endif
