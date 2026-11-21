// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "umeonigiri_serial_bridge/cli_number.hpp"

namespace {

using umeonigiri_serial_bridge::ParseDoubleArgument;
using umeonigiri_serial_bridge::ParseIntArgument;

}  // namespace

TEST(CliNumber, ParsesValidIntegers) {
  EXPECT_EQ(ParseIntArgument("115200", "--baud"), 115200);
  EXPECT_EQ(ParseIntArgument("-1", "--baud"), -1);
  EXPECT_EQ(ParseIntArgument("0", "--baud"), 0);
}

TEST(CliNumber, ParsesValidDoubles) {
  EXPECT_DOUBLE_EQ(ParseDoubleArgument("3.5", "--timeout"), 3.5);
  EXPECT_DOUBLE_EQ(ParseDoubleArgument("100", "--expect-hz"), 100.0);
  EXPECT_DOUBLE_EQ(ParseDoubleArgument("-0.25", "--duration"), -0.25);
}

TEST(CliNumber, RejectsNonNumeric) {
  // 以前は std::stoi / std::stod の例外が main を抜けて std::terminate になり、
  // "terminate called after throwing ... what(): stoi" だけを残して core を吐いた。
  EXPECT_EXIT(ParseIntArgument("abc", "--baud"), ::testing::ExitedWithCode(2), "");
  EXPECT_EXIT(ParseDoubleArgument("abc", "--timeout"), ::testing::ExitedWithCode(2), "");
}

TEST(CliNumber, RejectsTrailingGarbage) {
  EXPECT_EXIT(ParseIntArgument("115200abc", "--baud"), ::testing::ExitedWithCode(2), "");
  EXPECT_EXIT(ParseDoubleArgument("3.5s", "--timeout"), ::testing::ExitedWithCode(2), "");
}

TEST(CliNumber, RejectsNonFinite) {
  // std::stod は "nan" / "inf" を通す。周期計算にそのまま入ると測定値が壊れる。
  EXPECT_EXIT(ParseDoubleArgument("nan", "--expect-hz"), ::testing::ExitedWithCode(2), "");
  EXPECT_EXIT(ParseDoubleArgument("inf", "--expect-hz"), ::testing::ExitedWithCode(2), "");
  EXPECT_EXIT(ParseDoubleArgument("1e999", "--expect-hz"), ::testing::ExitedWithCode(2), "");
}
