#ifndef FOC_LIMITS_H
#define FOC_LIMITS_H

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

// Validate the textual P argument before any int cast. A finite 1e10 passes the integer
// floor check but is outside int range; casting it is undefined behavior.
inline bool validPolePairsArg(float pairs) {
  return std::isfinite(pairs) && pairs == std::floor(pairs) &&
         pairs >= static_cast<float>(kMinPolePairs) &&
         pairs <= static_cast<float>(kMaxPolePairs);
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
