#include "SamBusFrame.h"

#include <math.h>
#include <string.h>

namespace sambus {

namespace {

// Aux lamp boards in turn, one per lamp line (bit 6 is the aux coil latch).
constexpr uint8_t kAuxLampBoards[4] = {3, 4, 5, 7};

uint32_t usToCycles(uint32_t us) { return us * 1000u / kCycleNs; }

}  // namespace

Frame::Frame() {
  for (int i = 0; i < 256; ++i) {
    gamma_[i] = static_cast<uint16_t>(pow(i / 255.0, 2.2) * 65535.0 + 0.5);
  }
  // Anything above zero lights, however dimly.
  for (int i = 1; i < 256 && gamma_[i] == 0; ++i) gamma_[i] = 1;
  reset();
}

void Frame::reset() {
  memset(coil_, 0, sizeof(coil_));
  memset(target_, 0, sizeof(target_));
  memset(level_, 0, sizeof(level_));
  memset(auxAcc_, 0, sizeof(auxAcc_));
  for (uint16_t p = 0; p < kNumLampPorts; ++p) {
    lightUp_[p] = kUseDefault;
    afterGlow_[p] = kUseDefault;
  }
  auxUsed_ = 0;
}

void Frame::setSettings(const Settings& s) { settings_ = s; }

uint16_t Frame::slotUs() const {
  const uint16_t v = settings_.slotUs;
  return v < 100 ? 100 : (v > 2000 ? 2000 : v);
}

uint16_t Frame::blankUs() const {
  // The blank must hold the fixed commands (about 10 us) and leave a window.
  const uint16_t v = settings_.blankUs < 16 ? 16 : settings_.blankUs;
  return v > slotUs() / 2 ? static_cast<uint16_t>(slotUs() / 2) : v;
}

void Frame::setCoil(uint8_t port, uint8_t power) {
  if (port >= kCoilFirst && port <= kCoilLast) coil_[port] = power;
}

void Frame::setLamp(uint8_t port, uint8_t brightness) {
  if ((port >= kLampFirst && port <= kLampLast) || isAuxLampPort(port)) {
    target_[port] = brightness;
  }
}

void Frame::setLampRamp(uint8_t port, uint16_t lightUpMs, uint16_t afterGlowMs) {
  lightUp_[port] = lightUpMs;
  afterGlow_[port] = afterGlowMs;
}

void Frame::attachLamp(uint8_t port) {
  if (isAuxLampPort(port)) auxUsed_ |= static_cast<uint8_t>(1u << auxLampStrobeBit(port));
}

void Frame::allOff() {
  memset(coil_, 0, sizeof(coil_));
  memset(target_, 0, sizeof(target_));
  memset(level_, 0, sizeof(level_));
}

void Frame::updateLevels() {
  const uint32_t dtUs = frameUs();
  for (uint16_t p = 1; p < kNumLampPorts; ++p) {
    const uint32_t target = target_[p] * 257u;
    const uint32_t cur = level_[p];
    if (cur == target) continue;
    uint16_t rampMs = cur < target ? lightUp_[p] : afterGlow_[p];
    if (rampMs == kUseDefault) {
      rampMs = cur < target ? settings_.lightUpMs : settings_.afterGlowMs;
    }
    if (rampMs == 0) {
      level_[p] = static_cast<uint16_t>(target);
      continue;
    }
    uint32_t step = static_cast<uint32_t>((65535ull * dtUs) / (rampMs * 1000ull));
    if (step == 0) step = 1;
    if (cur < target) {
      level_[p] = static_cast<uint16_t>(target - cur <= step ? target : cur + step);
    } else {
      level_[p] = static_cast<uint16_t>(cur - target <= step ? target : cur - step);
    }
  }
}

uint8_t Frame::lineSteps(uint8_t line, Step* out) const {
  const uint16_t window = static_cast<uint16_t>(slotUs() - blankUs());
  uint16_t onUs[8];
  uint8_t all = 0;
  for (uint8_t k = 0; k < 8; ++k) {
    // Lamp line * 8 + k + 1 is LMP_DRV bit 7 - k.
    const uint8_t bit = static_cast<uint8_t>(7 - k);
    const uint8_t port = static_cast<uint8_t>(line * 8 + k + 1);
    onUs[bit] = 0;
    const uint16_t duty = gamma_[level_[port] >> 8];
    if (duty == 0) continue;
    uint32_t t = (static_cast<uint32_t>(duty) * window + 32768u) >> 16;
    if (t == 0) t = 1;
    onUs[bit] = static_cast<uint16_t>(t);
    all |= static_cast<uint8_t>(1u << bit);
  }
  if (!all) return 0;

  uint8_t n = 0;
  out[n++] = {0, all};
  uint8_t remaining = all;
  while (remaining) {
    uint16_t t = 0xFFFF;
    for (uint8_t b = 0; b < 8; ++b) {
      if (((remaining >> b) & 1u) && onUs[b] < t) t = onUs[b];
    }
    if (t >= window) break;  // the rest stay on to the end of the line
    for (uint8_t b = 0; b < 8; ++b) {
      if (((remaining >> b) & 1u) && onUs[b] == t) remaining &= static_cast<uint8_t>(~(1u << b));
    }
    out[n++] = {t, remaining};
  }
  return n;
}

bool Frame::coilOn(uint8_t port, uint32_t lineIndex) const {
  const uint8_t power = coil_[port];
  if (power == 0) return false;
  if (power == 255) return true;
  uint32_t period = settings_.coilPeriodUs / slotUs();
  if (period < 2) period = 2;
  uint32_t onLines = (power * period) / 255u;
  if (onLines == 0) onLines = 1;
  return (lineIndex % period) < onLines;
}

void Frame::buildLine(uint8_t line, const uint8_t coils[5], uint8_t auxBit,
                      uint8_t auxValue, const Step* steps, uint8_t nSteps,
                      uint32_t* out) const {
  const uint8_t giIdle = gi_ ? kAuxGiIdle : static_cast<uint8_t>(kAuxGiIdle | kAuxGiRelayOff);
  const uint8_t prev = static_cast<uint8_t>((line + kLampLines - 1) % kLampLines);
  uint32_t base[kCmdsPerLine];
  int32_t at[kCmdsPerLine];  // desired start in cycles from line start, -1 = asap
  uint8_t n = 0;
  auto add = [&](uint32_t cmd, int32_t when) {
    base[n] = cmd;
    at[n] = when;
    n++;
  };

  add(writeCmd(kRegLmpDrv, 0, 0), -1);
  add(prev < 8 ? writeCmd(kRegLmpStb, 0, 0) : writeCmd(kRegAuxLmp, 0, 0), -1);
  add(line < 8 ? writeCmd(kRegLmpStb, static_cast<uint8_t>(1u << line), 0)
               : writeCmd(kRegAuxLmp, static_cast<uint8_t>(1u << (line - 8)), 0),
      -1);
  add(writeCmd(kRegSolB, coils[0], 0), -1);
  add(writeCmd(kRegSolA, coils[1], 0), -1);
  add(writeCmd(kRegSolC, coils[2], 0), -1);
  add(writeCmd(kRegFlshLmp, coils[3], 0), -1);
  // Aux coils: data, then a low pulse on strobe bit 6 (latched on the rise).
  add(writeCmd(kRegAuxDrv, coils[4], 0), -1);
  add(writeCmd(kRegAuxGi, static_cast<uint8_t>(giIdle & ~(1u << kAuxCoilStrobeBit)), 0), -1);
  add(writeCmd(kRegAuxGi, giIdle, 0), -1);
  if (auxBit) {
    add(writeCmd(kRegAuxDrv, auxValue, 0), -1);
    add(writeCmd(kRegAuxGi, static_cast<uint8_t>(giIdle & ~(1u << auxBit)), 0), -1);
  } else {
    add(writeCmd(kRegLmpDrv, 0, 0), -1);
    add(writeCmd(kRegLmpDrv, 0, 0), -1);
  }
  // Reg 0xB rewritten on every line: also keeps the GI relay where it belongs.
  add(writeCmd(kRegAuxGi, giIdle, 0), -1);
  add(line == 0 ? readCmd(kRegStatus, 0) : writeCmd(kRegLmpDrv, 0, 0), -1);
  // Padding during the blank, so every line has the same length.
  for (uint8_t i = nSteps; i < kMaxSteps; ++i) add(writeCmd(kRegLmpDrv, 0, 0), -1);
  const uint32_t blank = usToCycles(blankUs());
  for (uint8_t i = 0; i < nSteps; ++i) {
    add(writeCmd(kRegLmpDrv, steps[i].drive, 0), static_cast<int32_t>(blank + usToCycles(steps[i].tUs)));
  }

  // Space the commands: each one's idle runs until the next one is due, the
  // last one's until the end of the line.
  const uint32_t slot = usToCycles(slotUs());
  uint32_t t = 0;
  for (uint8_t i = 0; i < n; ++i) {
    const uint32_t end = t + cmdCycles(base[i]);
    const int32_t next = i + 1 < n ? at[i + 1] : static_cast<int32_t>(slot);
    const uint32_t idle = next > static_cast<int32_t>(end) ? static_cast<uint32_t>(next) - end : 0;
    out[i] = base[i] | ((idle > kMaxIdle ? kMaxIdle : idle) << 13);
    t = end + idle;
  }
}

void Frame::build(uint32_t* out) {
  updateLevels();
  Step steps[kMaxSteps];
  for (uint8_t line = 0; line < kLampLines; ++line) {
    const uint32_t lineIndex = lineCounter_ + line;
    uint8_t coils[5] = {0, 0, 0, 0, 0};
    for (uint8_t port = kCoilFirst; port <= kCoilLast; ++port) {
      if (coilOn(port, lineIndex)) {
        coils[(port - 1) / 8] |= static_cast<uint8_t>(1u << ((port - 1) % 8));
      }
    }
    // One strobed aux board per line, its lamps dithered from frame to frame.
    const uint8_t auxBit = kAuxLampBoards[line % 4];
    uint8_t auxValue = 0;
    const bool auxOn = (auxUsed_ >> auxBit) & 1u;
    if (auxOn) {
      for (uint8_t b = 0; b < 8; ++b) {
        const uint8_t port = static_cast<uint8_t>(kAuxLampFirst + (auxBit - 3) * 8 + b);
        uint32_t& acc = auxAcc_[port - kAuxLampFirst];
        acc += gamma_[level_[port] >> 8];
        if (acc >= 65535u) {
          acc -= 65535u;
          auxValue |= static_cast<uint8_t>(1u << b);
        }
      }
    }
    const uint8_t n = lineSteps(line, steps);
    buildLine(line, coils, auxOn ? auxBit : 0, auxValue, steps, n, out + line * kCmdsPerLine);
  }
  lineCounter_ += kLampLines;
}

void Frame::buildSafe(uint32_t* out) const {
  const uint8_t coils[5] = {0, 0, 0, 0, 0};
  for (uint8_t line = 0; line < kLampLines; ++line) {
    const uint8_t auxBit = kAuxLampBoards[line % 4];
    const bool auxOn = (auxUsed_ >> auxBit) & 1u;
    buildLine(line, coils, auxOn ? auxBit : 0, 0, nullptr, 0, out + line * kCmdsPerLine);
  }
}

}  // namespace sambus
