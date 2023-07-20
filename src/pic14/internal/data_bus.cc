#include "data_bus.h"

namespace sim::pic14::internal {

  void SRAM::reset() {
    cells_ = std::vector<uint8_t>(cells_.size(), reset_value_);
  }

}  // namespace sim::pic14::internal
