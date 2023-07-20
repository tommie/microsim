#include "data_bus.h"

namespace sim::pic14::internal {

  void SRAM::reset() {
    cells_ = std::u8string(cells_.size(), reset_value_);
  }

}  // namespace sim::pic14::internal
