/*
  SamBusFrame.h
  Created for the SAM_IO board, 2026.

  Builds the bus traffic for one lamp frame of a Stern SAM IO board, as a list
  of commands for the sam_bus PIO program. A DMA channel replays these lists
  back to back, so the lamp scan, coil registers and aux latches keep running
  without the CPU; the CPU only builds the next list once per frame.

  One frame is the 10 lamp strobe lines, each `slotUs` long:

    blank      LMP_DRV = 0, strobe moves to the next line, the four coil
               registers, the aux coil latch, one strobed aux board, and on
               line 0 a STATUS read. Padded to a fixed length.
    window     LMP_DRV rewritten as lamps reach the end of their on-time
               (edge-sorted PWM: all lamps due on start together, each drops
               out at its own time, at most 9 writes).

  Every line has the same number of commands (kCmdsPerLine), so a list is
  always kFrameWords long; the idle field of each command spaces them in time.

  Lamp brightness (0-255, gamma 2.2) is the power PwmDevices writes for the
  lamp, so a lamp's `power` in the game config is its brightness. Lamps ramp
  up and down over their light-up and after-glow times, like the filament
  bulbs they replace. Coils below full power get a slow software PWM (ROM
  flipper hold: 1 ms in 12), switched on lamp-line boundaries.

  Pure logic, no hardware: host tests drive it directly.

  Play more pinball!
*/
#ifndef SAMBUS_SamBusFrame_h
#define SAMBUS_SamBusFrame_h

#include <stddef.h>
#include <stdint.h>

#include "SamBusMap.h"

namespace sambus {

// sam_bus.pio command word: data 7:0, register 11:8, read flag 12, idle
// cycles after the access 31:13.
constexpr uint32_t kMaxIdle = (1u << 19) - 1;
inline uint32_t writeCmd(uint8_t reg, uint8_t data, uint32_t idle) {
  return static_cast<uint32_t>(data) | (static_cast<uint32_t>(reg & 0xF) << 8) |
         ((idle > kMaxIdle ? kMaxIdle : idle) << 13);
}
inline uint32_t readCmd(uint8_t reg, uint32_t idle) {
  return (static_cast<uint32_t>(reg & 0xF) << 8) | (1u << 12) |
         ((idle > kMaxIdle ? kMaxIdle : idle) << 13);
}
inline bool cmdIsRead(uint32_t cmd) { return (cmd >> 12) & 1u; }
inline uint8_t cmdReg(uint32_t cmd) { return (cmd >> 8) & 0xF; }
inline uint8_t cmdData(uint32_t cmd) { return cmd & 0xFF; }
inline uint32_t cmdIdle(uint32_t cmd) { return cmd >> 13; }

// PIO cycles per access with no idle time (sam_bus.pio, clock divider 7 at
// 200 MHz = 35 ns per cycle).
constexpr uint32_t kCycleNs = 35;
constexpr uint32_t kWriteCycles = 18;
constexpr uint32_t kReadCycles = 23;
inline uint32_t cmdCycles(uint32_t cmd) {
  return (cmdIsRead(cmd) ? kReadCycles : kWriteCycles) + cmdIdle(cmd);
}

constexpr uint8_t kMaxSteps = 9;
constexpr uint8_t kBlankCmds = 14;
constexpr uint8_t kCmdsPerLine = kBlankCmds + kMaxSteps;
constexpr size_t kFrameWords = static_cast<size_t>(kLampLines) * kCmdsPerLine;

constexpr uint8_t kNumCoilPorts = kCoilLast + 1;
constexpr uint16_t kNumLampPorts = kAuxLampLast + 1;
constexpr uint16_t kUseDefault = 0xFFFF;

struct Settings {
  uint16_t slotUs = 250;           // per lamp line: 2.5 ms frame, 400 Hz
  uint16_t blankUs = 24;           // dark gap before each line (ROM value)
  uint16_t coilPeriodUs = 12000;   // software PWM period for coil power < 255
  uint16_t lightUpMs = 0;          // default lamp ramp up
  uint16_t afterGlowMs = 0;        // default lamp ramp down
};

class Frame {
 public:
  Frame();

  // Every output off and every lamp setting cleared. Settings stay.
  void reset();
  void setSettings(const Settings& s);
  const Settings& settings() const { return settings_; }

  void setCoil(uint8_t port, uint8_t power);
  void setLamp(uint8_t port, uint8_t brightness);
  // Ramp times for one lamp; kUseDefault takes the board setting.
  void setLampRamp(uint8_t port, uint16_t lightUpMs, uint16_t afterGlowMs);
  // Marks a lamp port as present, so its aux board gets latched.
  void attachLamp(uint8_t port);
  void setGi(bool on) { gi_ = on; }
  bool gi() const { return gi_; }
  // Drops every coil and lamp at once, without ramps.
  void allOff();

  // The next frame into `out` (kFrameWords commands). Moves the ramps and the
  // coil PWM on by one frame.
  void build(uint32_t* out);
  // A frame with every output off but the strobes running, for when nothing
  // builds in time: the IO board's watchdog stays fed, nothing stays on.
  void buildSafe(uint32_t* out) const;

  uint16_t lampLevel(uint8_t port) const { return level_[port]; }
  uint8_t coilPower(uint8_t port) const { return coil_[port]; }
  uint32_t frameUs() const { return static_cast<uint32_t>(slotUs()) * kLampLines; }

 private:
  struct Step {
    uint16_t tUs;
    uint8_t drive;
  };

  uint16_t slotUs() const;
  uint16_t blankUs() const;
  void updateLevels();
  uint8_t lineSteps(uint8_t line, Step* out) const;
  bool coilOn(uint8_t port, uint32_t lineIndex) const;
  void buildLine(uint8_t line, const uint8_t coils[5], uint8_t auxBit,
                 uint8_t auxValue, const Step* steps, uint8_t nSteps,
                 uint32_t* out) const;

  Settings settings_;
  uint8_t coil_[kNumCoilPorts];
  uint8_t target_[kNumLampPorts];
  uint16_t level_[kNumLampPorts];
  uint16_t lightUp_[kNumLampPorts];
  uint16_t afterGlow_[kNumLampPorts];
  uint32_t auxAcc_[kAuxLampLast - kAuxLampFirst + 1];
  uint8_t auxUsed_ = 0;  // bit = reg 0xB strobe bit with a lamp on it
  uint32_t lineCounter_ = 0;
  bool gi_ = true;
  uint16_t gamma_[256];
};

}  // namespace sambus

#endif
