#pragma once

#include <stdint.h>

#include "imu_protocol.h"
#include "imu_sampling.h"

namespace meister_imu {

class ImuView {
 public:
  void Begin();
  void Draw(const protocol::Sample& sample, const RawImu& raw,
            uint32_t tx_hz, bool tx_ok);
};

}  // namespace meister_imu
