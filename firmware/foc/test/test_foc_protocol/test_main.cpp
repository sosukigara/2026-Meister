#include <unity.h>

#include <cstring>
#include <string>

#include "foc_protocol.h"

namespace {

constexpr size_t kExpectedCsvFields = 8;

void expect(const char* line, foc::Command want) {
  const foc::Request req = foc::classify(line);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(want), static_cast<uint8_t>(req.cmd));
}

void expectNoArg(const char* line, foc::Command want) {
  const foc::Request req = foc::classify(line);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(want), static_cast<uint8_t>(req.cmd));
  TEST_ASSERT_FALSE(req.has_arg);
}

void expectArg(const char* line, foc::Command want, float want_arg) {
  const foc::Request req = foc::classify(line);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(want), static_cast<uint8_t>(req.cmd));
  TEST_ASSERT_TRUE(req.has_arg);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, want_arg, req.arg);
}

void test_no_arg_commands() {
  expectNoArg("C", foc::Command::kCalibrate);
  expectNoArg("S", foc::Command::kStop);
  expectNoArg("?", foc::Command::kHelp);
  expectNoArg("c", foc::Command::kError);  // verbs are case sensitive
}

void test_arg_commands() {
  expectArg("L 1.5", foc::Command::kSetCurrentLimit, 1.5f);
  expectArg("Z 0.8", foc::Command::kSetAlignVoltage, 0.8f);
  expectArg("T 1", foc::Command::kTorque, 1.0f);
  expectArg("T -1.25", foc::Command::kTorque, -1.25f);
  expectArg("W 30", foc::Command::kVelocity, 30.0f);
  expectArg("A 1.5708", foc::Command::kAngle, 1.5708f);
  expectArg("O 3", foc::Command::kOpenLoop, 3.0f);
  expectArg("N 2", foc::Command::kSine, 2.0f);  // TEMP
  expectArg("  W  -12.5  \r", foc::Command::kVelocity, -12.5f);
  expectArg("T 1e2", foc::Command::kTorque, 100.0f);
}

void test_empty_line_is_not_an_error() {
  expect("", foc::Command::kNone);
  expect("   ", foc::Command::kNone);
  expect("\r\n", foc::Command::kNone);
}

void test_unknown_and_malformed() {
  expect("X", foc::Command::kError);
  expect("HELLO", foc::Command::kError);
  expect("T", foc::Command::kError);       // missing argument
  expect("L", foc::Command::kError);
  expect("O", foc::Command::kError);
  expect("N", foc::Command::kError);  // TEMP
  expect("W abc", foc::Command::kError);
  expect("T 1.0 junk", foc::Command::kError);  // trailing garbage
  expect("S now", foc::Command::kError);       // trailing garbage on a no-arg verb
  expect("? 5", foc::Command::kError);
  expect("T nan", foc::Command::kError);
  expect("T 1e999", foc::Command::kError);  // parses as infinity
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(foc::Command::kError),
                          static_cast<uint8_t>(foc::classify(nullptr).cmd));
}

void test_csv_row_shape() {
  char row[foc::kCsvRowCap];
  const size_t n = foc::formatCsvRow(row, sizeof(row), 1.234f, 5.4321f, -12.5f, 0.75f, 1, -0.25f,
                                    3.5f, -0.125f);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_UINT8('\0', static_cast<uint8_t>(row[n]));
  TEST_ASSERT_EQUAL_size_t(kExpectedCsvFields, foc::csvFieldCount(row, n));
}

void test_csv_row_truncates_without_overflow() {
  char guarded[8];
  std::memset(guarded, 0x5A, sizeof(guarded));
  const size_t n = foc::formatCsvRow(guarded, 5, 1.0f, 2.0f, 3.0f, 4.0f, 2, 5.0f, 6.0f, 7.0f);
  // snprintf reports the length it wanted, which lets the caller notice the truncation.
  TEST_ASSERT_TRUE(n > 5);
  TEST_ASSERT_EQUAL_UINT8('\0', static_cast<uint8_t>(guarded[4]));
  for (size_t i = 5; i < sizeof(guarded); ++i) {
    TEST_ASSERT_EQUAL_UINT8(0x5A, static_cast<uint8_t>(guarded[i]));
  }
}

void test_csv_header_field_count_matches_row() {
  char row[foc::kCsvRowCap];
  const size_t n = foc::formatCsvRow(row, sizeof(row), 0.0f, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 0.0f);
  const size_t hn = std::strlen(foc::kCsvHeader);
  TEST_ASSERT_EQUAL_UINT8('\0', static_cast<uint8_t>(foc::kCsvHeader[hn]));
  // The '#' on the header is not a separator, so the counts must line up.
  TEST_ASSERT_EQUAL_size_t(foc::csvFieldCount(foc::kCsvHeader, hn),
                           foc::csvFieldCount(row, n));
  TEST_ASSERT_EQUAL_size_t(kExpectedCsvFields, foc::csvFieldCount(foc::kCsvHeader, hn));
  TEST_ASSERT_EQUAL_size_t(0, foc::csvFieldCount(nullptr, 0));
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_no_arg_commands);
  RUN_TEST(test_arg_commands);
  RUN_TEST(test_empty_line_is_not_an_error);
  RUN_TEST(test_unknown_and_malformed);
  RUN_TEST(test_csv_row_shape);
  RUN_TEST(test_csv_row_truncates_without_overflow);
  RUN_TEST(test_csv_header_field_count_matches_row);
  return UNITY_END();
}
