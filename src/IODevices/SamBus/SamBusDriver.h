/*
  SamBusDriver.h
  Created for the SAM_IO board, 2026.

  Runs SamBusFrame on the hardware: sam_bus.pio on one state machine, a DMA
  channel replaying the current frame into it, and a frame built per frame
  in the DMA interrupt.

  Fail-safe by construction: after every frame the DMA chain falls back to a
  frame with every output off (strobes still running). The interrupt has to
  queue a fresh frame each time, so if the firmware stops, coils and lamps
  drop within one frame while the IO board's watchdog stays fed.

  Bring-up: IO board held in reset, J1 buffers enabled, the safe frame
  running, then the reset released.

  Play more pinball!
*/
#ifndef SAMBUS_SamBusDriver_h
#define SAMBUS_SamBusDriver_h

#include <stdint.h>

#include "SamBusFrame.h"

namespace sambus {

class Driver {
 public:
  // Claims a state machine and three DMA channels and starts the bus.
  // Returns false if the PIO or DMA resources are not available.
  bool begin();
  Frame& frame() { return frame_; }
  // Last STATUS byte read from the IO board (interlocks, lamp faults).
  uint8_t status() const { return status_; }
  bool interlocksPresent() const {
    return (status_ & (kStatus20V | kStatus50V)) == (kStatus20V | kStatus50V);
  }
  uint32_t framesBuilt() const { return framesBuilt_; }

  // DMA interrupt: queue the next frame. Public for the IRQ trampoline.
  void onFrameDone();

 private:
  Frame frame_;
  volatile uint8_t status_ = 0;
  volatile uint32_t framesBuilt_ = 0;
  bool running_ = false;
};

}  // namespace sambus

#endif
