#include "foc_board.h"

#include "foc_config.h"

FocBoard::FocBoard(int bus_index) : i2c_(bus_index) {}

void FocBoard::begin() {
  i2c_.begin(foc_cfg::kI2cSdaPin, foc_cfg::kI2cSclPin, foc_cfg::kI2cClockHz);
}

bool FocBoard::setAs5600FastMode() {
  i2c_.beginTransmission(foc_cfg::kAs5600Address);
  i2c_.write(foc_cfg::kAs5600ConfRegister);
  if (i2c_.endTransmission(false) != 0) return false;
  if (i2c_.requestFrom(static_cast<int>(foc_cfg::kAs5600Address), 1) != 1) return false;

  // Read-modify-write: the upper CONF bits hold the zero-rotation / direction settings and
  // must survive this write.
  const uint8_t conf = static_cast<uint8_t>(i2c_.read());
  const uint8_t updated =
      static_cast<uint8_t>((conf & ~foc_cfg::kAs5600ConfFastMask) | foc_cfg::kAs5600ConfFastMask);
  if (updated == conf) return true;

  i2c_.beginTransmission(foc_cfg::kAs5600Address);
  i2c_.write(foc_cfg::kAs5600ConfRegister);
  i2c_.write(updated);
  return i2c_.endTransmission(true) == 0;
}

float FocBoard::vinVolts() const {
  return analogReadMilliVolts(foc_cfg::kVinSensePin) * foc_cfg::kVinDividerRatio / 1000.0f;
}

bool FocBoard::undervoltage() const { return vinVolts() < foc_cfg::kUndervoltageVolts; }
