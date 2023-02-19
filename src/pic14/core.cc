#include "core.h"

#include <system_error>

namespace sim::pic14 {

  sim::util::Status ICSP::load_program(uint16_t addr, std::u8string_view data) {
    while (data.size() > 1) {
      if (addr < device_->progmem().size()) {
        uint16_t end = std::min<uint16_t>(addr + data.size() / 2, device_->progmem().size());
        for (uint16_t i = addr, j = 0; i < end; ++i, j += 2) {
          device_->progmem()[i] = (static_cast<uint16_t>(data[j + 1]) << 8) | data[j];
        }
        data = data.substr((end - addr) * 2);
        addr = end;
      } else if (addr >= 0x2000 && addr < 0x2000 + device_->config().size()) {
        uint16_t end = std::min<uint16_t>(addr + data.size() / 2, 0x2000 + device_->config().size());
        for (uint16_t i = addr, j = 0; i < end; ++i, j += 2) {
          device_->config()[i - 0x2000] = (static_cast<uint16_t>(data[j + 1]) << 8) | data[j];
        }
        data = data.substr((end - addr) * 2);
        addr = end;
      } else if (addr >= 0x2100 && addr < 0x2100 + device_->eedata().size()) {
        uint16_t end = std::min<uint16_t>(addr + data.size() / 2, 0x2100 + device_->eedata().size());
        for (uint16_t i = addr, j = 0; i < end; ++i, j += 2) {
          device_->eedata()[i - 0x2000] = (static_cast<uint16_t>(data[j + 1]) << 8) | data[j];
        }
        data = data.substr((end - addr) * 2);
        addr = end;
      } else {
        break;
      }
    }

    return std::error_code{};
  }

  sim::util::Status ICSP::load_data(uint16_t addr, std::u8string_view data) {
    while (!data.empty()) {
      if (addr >= 0x2100 && addr < 0x2100 + device_->eedata().size()) {
        uint16_t end = std::min<uint16_t>(addr + data.size(), 0x2100 + device_->eedata().size());
        device_->eedata().replace(addr - 0x2100, end - addr, data);
        data = data.substr(end - addr);
        addr = end;
      } else {
        break;
      }
    }

    return std::error_code{};
  }

}  // namespace sim::pic14

namespace sim::pic14::internal {

  Core::Core(sim::core::Clock *extosc, NonVolatile *nv, std::function<void(bool)> reset, std::function<void()> option_updated)
    : extosc_(extosc),
      nv_(nv),
      reset_(std::move(reset), 3),
      mclr_(reset_.make_signal()),
      mclr_pin_([this](bool value) {
        mclr_->set(!value);  // MCLR pin is active low.
      }),
      por_(reset_.make_signal(true)),
      icsp_reset_(reset_.make_signal()),
      option_updated_(std::move(option_updated)),
      option_reg_(SingleRegisterBackend<uint8_t>(0xFF)) {}

  ICSP Core::enter_icsp() {
    return ICSP(nv_, icsp_reset_);
  }

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

  sim::core::Advancement Core::advance_to(const sim::core::SimulationLimit &limit) {
    if (por_->value()) {
      por_->set(false);
    }

    return {
      .at_tick = fosc()->at(0),
      .next_tick = -1,
    };
  }

}  // namespace sim::pic14::internal
