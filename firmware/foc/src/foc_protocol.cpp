#include "foc_protocol.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace foc {
namespace {

bool isBlank(char c) {
  return std::isspace(static_cast<unsigned char>(c)) != 0;
}

// Returns the first non-blank character, or '\0' when the line carries nothing else.
const char* skipBlank(const char* p) {
  while (*p != '\0' && isBlank(*p)) ++p;
  return p;
}

Command verbOf(char c) {
  switch (c) {
    case 'C': return Command::kCalibrate;
    case 'L': return Command::kSetCurrentLimit;
    case 'Z': return Command::kSetAlignVoltage;
    case 'T': return Command::kTorque;
    case 'W': return Command::kVelocity;
    case 'A': return Command::kAngle;
    case 'O': return Command::kOpenLoop;
    case 'N': return Command::kSine;
    case 'G': return Command::kAngleOpenLoop;
    case 'D': return Command::kDiagnose;
    case 'S': return Command::kStop;
    case '?': return Command::kHelp;
    default: return Command::kError;
  }
}

bool takesArgument(Command c) {
  return c == Command::kSetCurrentLimit || c == Command::kSetAlignVoltage ||
         c == Command::kTorque || c == Command::kVelocity || c == Command::kAngle ||
         c == Command::kOpenLoop || c == Command::kSine || c == Command::kAngleOpenLoop;
}

}  // namespace

Request classify(const char* line) {
  Request req;
  if (line == nullptr) return req;

  const char* p = skipBlank(line);
  if (*p == '\0') {
    req.cmd = Command::kNone;
    return req;
  }

  const Command verb = verbOf(*p);
  if (verb == Command::kError) return req;
  req.cmd = verb;
  req.has_arg = takesArgument(verb);

  const char* argp = skipBlank(p + 1);
  if (!takesArgument(verb)) {
    // No argument is legal; trailing text on S/C/? is not.
    req.cmd = (*argp == '\0') ? verb : Command::kError;
    return req;
  }

  char* end = nullptr;
  const float value = std::strtof(argp, &end);
  if (end == argp) {
    req.cmd = Command::kError;
    req.has_arg = false;
    return req;
  }
  const char* tail = skipBlank(end);
  if (*tail != '\0' || !std::isfinite(value)) {
    req.cmd = Command::kError;
    req.has_arg = false;
    return req;
  }

  req.arg = value;
  return req;
}

size_t formatCsvRow(char* out, size_t cap, float t_s, float mech_rad, float vel_rps,
                    float iq, uint8_t mode, float id, float vq, float vd) {
  const int written =
      std::snprintf(out, cap, "%.3f,%.4f,%.2f,%.3f,%u,%.3f,%.3f,%.3f", static_cast<double>(t_s),
                    static_cast<double>(mech_rad), static_cast<double>(vel_rps),
                    static_cast<double>(iq), static_cast<unsigned>(mode),
                    static_cast<double>(id), static_cast<double>(vq), static_cast<double>(vd));
  if (written < 0) {
    if (cap > 0) out[0] = '\0';
    return 0;
  }
  return static_cast<size_t>(written);
}

size_t csvFieldCount(const char* row, size_t len) {
  if (row == nullptr || len == 0) return 0;
  size_t fields = 1;
  for (size_t i = 0; i < len; ++i) {
    if (row[i] == ',') ++fields;
  }
  return fields;
}

}  // namespace foc
