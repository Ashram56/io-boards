// Host tests for the SAM_IO board's bus traffic (IODevices/SamBus).
//
// SamBusFrame is the whole of what reaches the SAM IO board: the DMA engine
// only replays its command lists. So the properties that keep a machine safe
// and its lamps right can all be checked here, by walking a list the way the
// PIO program executes it:
//   - the lamp strobes never stop and line 0 (the IO board's watchdog input)
//     is driven on every frame, including the fail-safe frame;
//   - every line lasts exactly its slot, so the lamp scan keeps its rate;
//   - coils and lamps land on the registers and bits of their SAM numbers.

#include <unity.h>

#include <vector>

#include "Arduino.h"
#include "EventDispatcher/Event.h"
#include "EventDispatcher/EventDispatcher.h"
#include "HardwareStubs.h"
#include "IODevices/PwmDevices.h"
#include "IODevices/SamBus/SamBusFrame.h"
#include "IODevices/SamBus/SamBusOutput.h"

using namespace sambus;

namespace {

struct Access {
  uint32_t cycle;  // start, from the frame start
  uint8_t line;
  bool read;
  uint8_t reg;
  uint8_t data;
};

std::vector<Access> Walk(const uint32_t* frame) {
  std::vector<Access> out;
  uint32_t t = 0;
  for (size_t i = 0; i < kFrameWords; ++i) {
    const uint32_t c = frame[i];
    out.push_back({t, static_cast<uint8_t>(i / kCmdsPerLine), cmdIsRead(c), cmdReg(c), cmdData(c)});
    t += cmdCycles(c);
  }
  return out;
}

uint32_t LineCycles(const uint32_t* frame, uint8_t line) {
  uint32_t t = 0;
  for (uint8_t i = 0; i < kCmdsPerLine; ++i) t += cmdCycles(frame[line * kCmdsPerLine + i]);
  return t;
}

uint32_t UsToCycles(uint32_t us) { return us * 1000u / kCycleNs; }

uint32_t g_frame[kFrameWords];

// Last value written to `reg` on `line` during the blank part of the line.
int RegOnLine(const std::vector<Access>& a, uint8_t line, uint8_t reg) {
  int v = -1;
  for (const auto& x : a) {
    if (x.line == line && !x.read && x.reg == reg) v = x.data;
  }
  return v;
}

}  // namespace

void setUp(void) { stubs::Reset(); }
void tearDown(void) {}

void test_every_line_lasts_its_slot(void) {
  Frame f;
  f.setLamp(1, 200);
  f.setLamp(12, 30);
  f.build(g_frame);
  for (uint8_t line = 0; line < kLampLines; ++line) {
    TEST_ASSERT_EQUAL_UINT32(UsToCycles(250), LineCycles(g_frame, line));
  }
}

void test_strobes_walk_all_ten_lines_and_line0_feeds_the_watchdog(void) {
  Frame f;
  uint32_t safe[kFrameWords];
  f.buildSafe(safe);
  for (const uint32_t* fr : {static_cast<const uint32_t*>(safe), static_cast<const uint32_t*>(g_frame)}) {
    if (fr == g_frame) f.build(g_frame);
    const auto a = Walk(fr);
    for (uint8_t line = 0; line < 8; ++line) {
      TEST_ASSERT_EQUAL_INT(1 << line, RegOnLine(a, line, kRegLmpStb));
    }
    TEST_ASSERT_EQUAL_INT(1, RegOnLine(a, 8, kRegAuxLmp));
    TEST_ASSERT_EQUAL_INT(2, RegOnLine(a, 9, kRegAuxLmp));
    // The previous line's strobe is cleared before the next one is set.
    TEST_ASSERT_EQUAL_INT(0, RegOnLine(a, 8, kRegLmpStb));
    TEST_ASSERT_EQUAL_INT(0, RegOnLine(a, 0, kRegAuxLmp));
  }
}

void test_status_is_read_once_per_frame(void) {
  Frame f;
  f.build(g_frame);
  int reads = 0;
  for (const auto& x : Walk(g_frame)) {
    if (x.read) {
      reads++;
      TEST_ASSERT_EQUAL_UINT8(kRegStatus, x.reg);
      TEST_ASSERT_EQUAL_UINT8(0, x.line);
    }
  }
  TEST_ASSERT_EQUAL_INT(1, reads);
}

void test_coils_land_on_their_registers(void) {
  Frame f;
  f.setCoil(1, 255);   // SOL_B bit 0
  f.setCoil(9, 255);   // SOL_A bit 0
  f.setCoil(20, 255);  // SOL_C bit 3
  f.setCoil(32, 255);  // FLSH_LMP bit 7
  f.setCoil(33, 255);  // aux coil, AUX_DRV bit 0 latched by reg 0xB bit 6
  f.build(g_frame);
  const auto a = Walk(g_frame);
  for (uint8_t line = 0; line < kLampLines; ++line) {
    TEST_ASSERT_EQUAL_INT(0x01, RegOnLine(a, line, kRegSolB));
    TEST_ASSERT_EQUAL_INT(0x01, RegOnLine(a, line, kRegSolA));
    TEST_ASSERT_EQUAL_INT(0x08, RegOnLine(a, line, kRegSolC));
    TEST_ASSERT_EQUAL_INT(0x80, RegOnLine(a, line, kRegFlshLmp));
  }
  // AUX_DRV = 1, then reg 0xB with bit 6 low, then back high.
  bool seen = false;
  for (size_t i = 0; i + 2 < a.size(); ++i) {
    if (a[i].reg == kRegAuxDrv && a[i].data == 0x01 && a[i + 1].reg == kRegAuxGi &&
        !(a[i + 1].data & (1u << kAuxCoilStrobeBit)) && a[i + 2].reg == kRegAuxGi &&
        (a[i + 2].data & (1u << kAuxCoilStrobeBit))) {
      seen = true;
    }
  }
  TEST_ASSERT_TRUE(seen);
}

void test_coil_hold_power_is_a_slow_pwm(void) {
  Frame f;
  f.setCoil(1, 21);  // 21/255 of a 12 ms period of 48 lines = 3 lines on
  int on = 0;
  for (int frame = 0; frame < 48 / 2; ++frame) {  // 24 frames = 240 lines = 5 periods
    f.build(g_frame);
    const auto a = Walk(g_frame);
    for (uint8_t line = 0; line < kLampLines; ++line) on += RegOnLine(a, line, kRegSolB) & 1;
  }
  TEST_ASSERT_EQUAL_INT(15, on);
}

void test_lamp_brightness_is_an_in_line_pwm(void) {
  Frame f;
  f.setLamp(1, 255);  // line 0, LMP_DRV bit 7, fully on
  f.setLamp(8, 128);  // line 0, bit 0, 21.8 % after gamma
  f.build(g_frame);
  const auto a = Walk(g_frame);
  uint32_t onAt = 0, dropAt = 0;
  for (const auto& x : a) {
    if (x.line != 0 || x.read || x.reg != kRegLmpDrv) continue;
    if (x.data == 0x81) onAt = x.cycle;
    if (x.data == 0x80) dropAt = x.cycle;
  }
  TEST_ASSERT_EQUAL_UINT32(UsToCycles(24), onAt);  // right after the blank
  const uint32_t onUs = (dropAt - onAt) * kCycleNs / 1000;
  TEST_ASSERT_UINT32_WITHIN(2, 49, onUs);  // 21.8 % of the 226 us window
  // Lamp 1 stays on to the end of the line; nothing else lights.
  TEST_ASSERT_EQUAL_INT(0, RegOnLine(a, 1, kRegLmpDrv));
}

void test_lamps_ramp_like_filaments(void) {
  Frame f;
  f.setLampRamp(5, /*lightUp*/ 100, /*afterGlow*/ 50);
  f.setLamp(5, 255);
  for (int i = 0; i < 4; ++i) f.build(g_frame);  // 10 ms
  TEST_ASSERT_UINT32_WITHIN(700, 6553, f.lampLevel(5));
  for (int i = 0; i < 40; ++i) f.build(g_frame);  // past 100 ms
  TEST_ASSERT_EQUAL_UINT16(65535, f.lampLevel(5));
  f.setLamp(5, 0);
  for (int i = 0; i < 10; ++i) f.build(g_frame);  // 25 ms of a 50 ms glow
  TEST_ASSERT_UINT32_WITHIN(1400, 32768, f.lampLevel(5));
  // Default ramps are instant.
  f.setLamp(6, 255);
  f.build(g_frame);
  TEST_ASSERT_EQUAL_UINT16(65535, f.lampLevel(6));
}

void test_aux_lamps_are_latched_on_their_board(void) {
  Frame f;
  f.attachLamp(213);  // 200 + (4 - 3) * 8 + 5: reg 0xB bit 4 board, AUX_DRV bit 5
  f.setLamp(213, 255);
  f.build(g_frame);
  const auto a = Walk(g_frame);
  bool latched = false;
  for (size_t i = 0; i + 1 < a.size(); ++i) {
    if (a[i].reg == kRegAuxDrv && a[i].data == (1u << 5) && a[i + 1].reg == kRegAuxGi &&
        !(a[i + 1].data & (1u << 4))) {
      latched = true;
    }
  }
  TEST_ASSERT_TRUE(latched);
}

void test_safe_frame_has_every_output_off(void) {
  Frame f;
  f.attachLamp(205);
  f.setCoil(1, 255);
  f.setLamp(1, 255);
  f.setLamp(205, 255);
  f.build(g_frame);
  f.buildSafe(g_frame);
  for (const auto& x : Walk(g_frame)) {
    if (x.read) continue;
    if (x.reg == kRegSolA || x.reg == kRegSolB || x.reg == kRegSolC || x.reg == kRegFlshLmp ||
        x.reg == kRegLmpDrv || x.reg == kRegAuxDrv) {
      TEST_ASSERT_EQUAL_UINT8(0, x.data);
    }
  }
}

void test_gi_relay_follows_the_setting(void) {
  Frame f;
  f.build(g_frame);
  TEST_ASSERT_EQUAL_INT(kAuxGiIdle, RegOnLine(Walk(g_frame), 3, kRegAuxGi));
  f.setGi(false);
  f.build(g_frame);
  TEST_ASSERT_EQUAL_INT(kAuxGiIdle | kAuxGiRelayOff, RegOnLine(Walk(g_frame), 3, kRegAuxGi));
}

void test_pwm_devices_route_coils_and_lamps_by_type(void) {
  // Coil 1 and lamp 1 share a port number but are different SAM outputs.
  EventDispatcher dispatcher;
  PwmDevices pwm(&dispatcher);
  Frame f;
  SamBusOutput out(f);
  pwm.setOutput(&out);
  pwm.registerSolenoid(/*port*/ 1, /*number*/ 10, 255, 0, 100, 0, 0, 0);
  pwm.registerLamp(/*port*/ 1, /*number*/ 20, /*brightness*/ 180);
  Event run(EVENT_RUN, 1, 1);
  pwm.handleEvent(&run);

  Event lamp(EVENT_SOURCE_LIGHT, 20, 1);
  pwm.handleEvent(&lamp);
  TEST_ASSERT_EQUAL_UINT8(0, f.coilPower(1));
  f.build(g_frame);
  TEST_ASSERT_EQUAL_UINT16(180 * 257, f.lampLevel(1));

  Event coil(EVENT_SOURCE_SOLENOID, 10, 1);
  pwm.handleEvent(&coil);
  TEST_ASSERT_EQUAL_UINT8(255, f.coilPower(1));
  pwm.off();
  TEST_ASSERT_EQUAL_UINT8(0, f.coilPower(1));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_every_line_lasts_its_slot);
  RUN_TEST(test_strobes_walk_all_ten_lines_and_line0_feeds_the_watchdog);
  RUN_TEST(test_status_is_read_once_per_frame);
  RUN_TEST(test_coils_land_on_their_registers);
  RUN_TEST(test_coil_hold_power_is_a_slow_pwm);
  RUN_TEST(test_lamp_brightness_is_an_in_line_pwm);
  RUN_TEST(test_lamps_ramp_like_filaments);
  RUN_TEST(test_aux_lamps_are_latched_on_their_board);
  RUN_TEST(test_safe_frame_has_every_output_off);
  RUN_TEST(test_gi_relay_follows_the_setting);
  RUN_TEST(test_pwm_devices_route_coils_and_lamps_by_type);
  return UNITY_END();
}
