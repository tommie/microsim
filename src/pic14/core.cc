#include "core.h"

namespace sim::pic14::internal {

  uint8_t Core::read_register(uint16_t addr) {
    switch (addr) {
    case 0x81: return option_reg_.read();
    default: return 0;
    }
  }

  void Core::write_register(uint16_t addr, uint8_t value) {
    switch (addr) {
    case 0x81:
      if (value != option_reg_.read()) {
        option_reg_.write(value);
        option_updated_();
      }
      break;
    }
  }

}  // namespace sim::pic14::internal
