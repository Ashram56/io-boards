/*
  SamBusMap.h
  Created for the SAM_IO board, 2026.

  The Stern SAM IO power driver board (520-5249-00) as seen from its J1 bus,
  and the PPUC ports the SAM_IO board gives its outputs.

  Register map and bit meanings come from the SAM ROM bus analysis
  (Ashram56/Tron-Legacy-LE-ROM-Decryption, io/bus/bus_register_map.csv and
  io/bus/README.md).

  Play more pinball!
*/
#ifndef SAMBUS_SamBusMap_h
#define SAMBUS_SamBusMap_h

#include <stdint.h>

namespace sambus {

// J1 registers, A3-A0.
enum Reg : uint8_t {
  kRegSolA = 0x0,     // W coils 9-16, bit 0 = coil 9
  kRegSolB = 0x1,     // W coils 1-8
  kRegSolC = 0x2,     // W coils 17-24
  kRegFlshLmp = 0x3,  // W coils 25-32 (flashers)
  kRegStatus = 0x5,   // R D0 20 V present, D1 50 V present, D2 zero cross,
                      //   D3 lamp driver 1 fault, D4 lamp driver 2 fault
  kRegAuxDrv = 0x6,   // W data for the strobed aux boards
  kRegAuxIn = 0x7,    // R J3 pins 1-8
  kRegLmpStb = 0x8,   // W lamp strobe lines 0-7. Line 0 (DRV0) feeds the IO
                      //   board's watchdog: it must keep toggling.
  kRegAuxLmp = 0x9,   // W lamp strobe lines 8-9
  kRegLmpDrv = 0xA,   // W lamp drive bits of the active line
  kRegAuxGi = 0xB,    // W bit 0 GI relay (0 = on), bits 3-7 aux strobes
                      //   (idle high, latch on the rising edge)
};

constexpr uint8_t kStatus20V = 0x01;
constexpr uint8_t kStatus50V = 0x02;

// Reg 0xB idle value with GI on; the ROM keeps bits 1-2 high too.
constexpr uint8_t kAuxGiIdle = 0xFE;
constexpr uint8_t kAuxGiRelayOff = 0x01;
// Aux coils 33-40 sit behind the reg 0xB bit 6 latch.
constexpr uint8_t kAuxCoilStrobeBit = 6;

constexpr uint8_t kLampLines = 10;

// ---------------------------------------------------------------------------
// PPUC ports. A port is the output's SAM number, so a game config can follow
// PinMAME's numbering. Which table a port belongs to comes from the output's
// PWM type: solenoid, flasher and motor are coils, lamp is a lamp.
//
//   coils   1-32 driver board, 33-40 aux coils (reg 0xB bit 6 latch)
//   lamps   1-80 lamp matrix: lamp n is line (n-1)/8, LMP_DRV bit 7-(n-1)%8
//           200 + (strobe bit - 3) * 8 + bit: strobed aux boards (reg 0xB
//           bits 3, 4, 5, 7), for example the Tron LE ramp tubes
// ---------------------------------------------------------------------------
constexpr uint8_t kCoilFirst = 1;
constexpr uint8_t kCoilLast = 40;
constexpr uint8_t kLampFirst = 1;
constexpr uint8_t kLampLast = 80;
constexpr uint8_t kAuxLampFirst = 200;
constexpr uint8_t kAuxLampLast = 239;

inline bool isAuxLampPort(uint8_t port) {
  return port >= kAuxLampFirst && port <= kAuxLampLast;
}
inline uint8_t auxLampStrobeBit(uint8_t port) {
  return static_cast<uint8_t>(3 + (port - kAuxLampFirst) / 8);
}

}  // namespace sambus

#endif
