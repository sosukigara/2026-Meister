#ifndef FOC_PROTOCOL_H
#define FOC_PROTOCOL_H

// Command grammar and telemetry row formatting. Deliberately free of Arduino headers so
// the same translation unit links into the native unit-test environment.

#include <cstddef>
#include <cstdint>

namespace foc {

enum class Command : uint8_t {
  kNone,             // blank line, silently ignored
  kCalibrate,        // C
  kSetCurrentLimit,  // L <amps>
  kSetAlignVoltage,  // Z <volts>
  kTorque,           // T <torque>
  kVelocity,         // W <rev/s>
  kAngle,            // A <rad>
  kOpenLoop,         // O <rev/s>, sensorless velocity, spin check only
  kSine,             // N <amp_rps>, TEMP sensorless sine spin check
  kAngleOpenLoop,    // G <rad>, TEMP sensorless angle, spin check only
  kDiagnose,         // D, TEMP winding/driver current check, no arg
  kStop,             // S
  kHelp,             // ?
  kError             // unknown verb, missing arg, trailing garbage, non-finite arg
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

// Sized so the widest row still fits: 8 fields with %.3f/%.4f/%.2f precision is ~70 bytes.
// A row larger than 115200 baud / kTelemetryPeriodMs (~115 B at 100 Hz) would back up the
// UART and stall loop(), so this cap is the reason the telemetry cannot drift faster.
constexpr size_t kCsvRowCap = 96;

constexpr char kCsvHeader[] =
    "#t_s,mech_rad,vel_rps,iq_a,mode,id_a,vq_v,vd_v";

// Returns the length snprintf would have produced, so a caller can detect truncation even
// when snprintf already NUL-terminated the buffer.
size_t formatCsvRow(char* out, size_t cap, float t_s, float mech_rad, float vel_rps,
                    float iq, uint8_t mode, float id, float vq, float vd);

size_t csvFieldCount(const char* row, size_t len);

}  // namespace foc

#endif  // FOC_PROTOCOL_H
