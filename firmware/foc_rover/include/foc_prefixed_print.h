#ifndef FOC_PREFIXED_PRINT_H
#define FOC_PREFIXED_PRINT_H

// SimpleFOC's monitor_port writes bare "MOT: ..." lines. The console contract reserves
// '#' for one-shot diagnostics and everything else for CSV rows, so route the library
// output through this wrapper. Every line starts with the prefix.

#include <Arduino.h>

class PrefixedPrint : public Print {
 public:
  PrefixedPrint(Print& out, const char* prefix) : out_(out), prefix_(prefix) {}

  size_t write(uint8_t c) override {
    if (c == '\n') {
      at_line_start_ = true;
      return out_.write(c);
    }
    if (c == '\r') return out_.write(c);
    if (at_line_start_) {
      for (const char* p = prefix_; *p != '\0'; ++p) {
        out_.write(static_cast<uint8_t>(*p));
      }
      at_line_start_ = false;
    }
    return out_.write(c);
  }

  using Print::write;

 private:
  Print& out_;
  const char* prefix_;
  bool at_line_start_ = true;
};

#endif  // FOC_PREFIXED_PRINT_H
