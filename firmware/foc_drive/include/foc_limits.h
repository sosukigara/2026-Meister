#ifndef FOC_LIMITS_H
#define FOC_LIMITS_H

// Deliberately free of Arduino headers: the same header links into the native unit-test
// environment. Every command limit lives here so the console, the drive and the tests
// cannot drift apart.
//
// Limits are chosen for the LA8308 KV90 + MKS ESP32 FOC V2.0 bench:
//   - the board's current sense saturates near 6.6 A (10 mohm shunt, gain 50);
//   - 6 A/ch is a peak figure with cooling, not a measured continuous rating;
//   - 0.186 ohm (vendor figure) means 1.0 V already asks for ~5.4 A.
// Leave headroom for ripple and offset error; 4 A is the hard cap, 3 A the default.

#include <cmath>
#include <cstdint>

namespace foc {

constexpr float kCurrentLimitDefault = 3.0f;
constexpr float kCurrentLimitMin = 0.1f;
constexpr float kCurrentLimitMax = 4.0f;

constexpr float kAlignVoltageMin = 0.10f;
constexpr float kAlignVoltageMax = 0.60f;

constexpr float kOpenLoopVoltageMin = 0.10f;
constexpr float kOpenLoopVoltageMax = 0.60f;
constexpr float kOpenLoopVoltageWarn = 0.45f;

constexpr float kOpenLoopMaxRps = 5.0f;
constexpr float kClosedLoopMaxRps = 5.0f;
constexpr float kMaxAngleRad = 100.0f;

constexpr int kMinPolePairs = 1;
constexpr int kMaxPolePairs = 50;

inline bool validCurrentLimit(float amps) {
  return std::isfinite(amps) && amps >= kCurrentLimitMin && amps <= kCurrentLimitMax;
}

inline bool validAlignVoltage(float volts) {
  return std::isfinite(volts) && volts >= kAlignVoltageMin && volts <= kAlignVoltageMax;
}

inline float clampOpenLoopVoltage(float volts) {
  if (!std::isfinite(volts)) return kOpenLoopVoltageMin;
  if (volts < kOpenLoopVoltageMin) return kOpenLoopVoltageMin;
  if (volts > kOpenLoopVoltageMax) return kOpenLoopVoltageMax;
  return volts;
}

inline float clampSpeedRps(float rps, float limit) {
  if (!std::isfinite(rps)) return 0.0f;
  if (rps > limit) return limit;
  if (rps < -limit) return -limit;
  return rps;
}

inline float clampAngleRad(float rad) { return clampSpeedRps(rad, kMaxAngleRad); }

constexpr bool validPolePairs(int pairs) {
  return pairs >= kMinPolePairs && pairs <= kMaxPolePairs;
}

// Signed elapsed-time checks. millis() wraps after ~49.7 days; a plain `<` would either
// wedge the cooldown or let it be bypassed.
constexpr bool cooldownActive(uint32_t now_ms, uint32_t until_ms) {
  return static_cast<int32_t>(now_ms - until_ms) < 0;
}

constexpr bool timeoutElapsed(uint32_t now_ms, uint32_t since_ms, uint32_t timeout_ms) {
  return static_cast<uint32_t>(now_ms - since_ms) > timeout_ms;
}

}  // namespace foc

#endif  // FOC_LIMITS_H
