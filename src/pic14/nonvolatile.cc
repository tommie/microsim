#include "nonvolatile.h"

namespace sim::pic14::internal {

  ICSP NonVolatile::enter_icsp() {
    return ICSP(this);
  }

}  // namespace sim::pic14::internal

namespace sim::pic14 {

  sim::core::Status ICSP::load_program(uint16_t addr, std::u8string_view data) {
    while (data.size() > 1) {
      if (addr < device->progmem.size()) {
        uint16_t end = std::min<uint16_t>(addr + data.size() / 2, device->progmem.size());
        for (uint16_t i = addr, j = 0; i < end; ++i, j += 2) {
          device->progmem[i] = (static_cast<uint16_t>(data[j + 1]) << 8) | data[j];
        }
        data = data.substr((end - addr) * 2);
        addr = end;
      } else if (addr >= 0x2000 && addr < 0x2000 + device->config.size()) {
        uint16_t end = std::min<uint16_t>(addr + data.size() / 2, 0x2000 + device->config.size());
        for (uint16_t i = addr, j = 0; i < end; ++i, j += 2) {
          device->config[i - 0x2000] = (static_cast<uint16_t>(data[j + 1]) << 8) | data[j];
        }
        data = data.substr((end - addr) * 2);
        addr = end;
      } else if (addr >= 0x2100 && addr < 0x2100 + device->eedata.size()) {
        uint16_t end = std::min<uint16_t>(addr + data.size() / 2, 0x2100 + device->eedata.size());
        for (uint16_t i = addr, j = 0; i < end; ++i, j += 2) {
          device->eedata[i - 0x2000] = (static_cast<uint16_t>(data[j + 1]) << 8) | data[j];
        }
        data = data.substr((end - addr) * 2);
        addr = end;
      } else {
        break;
      }
    }

    return std::error_code{};
  }

  sim::core::Status ICSP::load_data(uint16_t addr, std::u8string_view data) {
    while (!data.empty()) {
      if (addr >= 0x2100 && addr < 0x2100 + device->eedata.size()) {
        uint16_t end = std::min<uint16_t>(addr + data.size(), 0x2100 + device->eedata.size());
        device->eedata.replace(addr - 0x2100, end - addr, data);
        data = data.substr(end - addr);
        addr = end;
      } else {
        break;
      }
    }

    return std::error_code{};
  }

}  // namespace sim::pic14
