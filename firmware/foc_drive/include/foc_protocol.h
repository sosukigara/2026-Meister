#ifndef FOC_PROTOCOL_H
#define FOC_PROTOCOL_H

// Command grammar and telemetry row formatting. Deliberately free of Arduino headers so
// the same translation unit links into the native unit-test environment.

#include <cstddef>
#include <cstdint>

namespace foc {

enum class Command : uint8_t {
  kNone,                 // blank line, silently ignored
  kCalibrate,            // C
  kSetCurrentLimit,      // L <amps>
  kSetAlignVoltage,      // Z <volts>
  kTorque,               // T <torque>
  kVelocity,             // W <rev/s>
  kAngle,                // A <rad>
  kOpenLoop,             // O <rev/s>, sensorless velocity, spin check only
  kDiagnose,             // D, TEMP winding/driver current check, no arg
  kSetPolePairs,         // P <pairs>
  kSetOpenLoopVoltage,   // V <volts>
  kSelectChannel,        // M <0|1>
  kStop,                 // S, stops both channels
  kHelp,                 // ?
  kError                 // unknown verb, missing arg, trailing garbage, non-finite arg
};

struct Request {
  Command cmd = Command::kError;
  float arg = 0.0f;
  bool has_arg = false;
};

// A blank line is kNone rather than kError on purpose: readStringUntil() returns an empty
// string on every timeout, and the loop calls classify() on that empty result, so an error
// here would spam the console once per read timeout.
Request classify(const char* line);

// Sized so the widest row still fits: 9 fields with %.3f/%.4f/%.2f precision is ~75 bytes.
// Two rows at 50 Hz is ~7.5 kB/s, ~65 % of a 115200 baud link.
constexpr size_t kCsvRowCap = 104;

constexpr char kCsvHeader[] =
    "#t_s,ch,mech_rad,vel_rps,iq_a,mode,id_a,vq_v,vd_v";

// Returns the length snprintf would have produced, so a caller can detect truncation even
// when snprintf already NUL-terminated the buffer.
size_t formatCsvRow(char* out, size_t cap, float t_s, uint8_t ch, float mech_rad,
                    float vel_rps, float iq, uint8_t mode, float id, float vq, float vd);

size_t csvFieldCount(const char* row, size_t len);

}  // namespace foc

#endif  // FOC_PROTOCOL_H
