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

  Core::Core(sim::core::Clock *extosc, NonVolatile *nv, std::function<void(bool)> reset, std::function<void()> option_updated, std::function<void()> fosc_changed)
    : extosc_(extosc),
      nv_(nv),
      reset_(std::move(reset), 4),
      option_updated_(std::move(option_updated)),
      mclr_(reset_.make_signal()),
      mclr_pin_([this](bool value) {
        mclr_->set(!value);  // MCLR pin is active low.
      }),
      por_(reset_.make_signal(true)),
      icsp_reset_(reset_.make_signal()),
      wdt_reset_(reset_.make_signal()),
      hfintosc_(sim::core::Nanoseconds(125)),    // 8 MHz
      lfintosc_(sim::core::Nanoseconds(32258)),  // 31 kHz
      fosc_(std::move(fosc_changed), &hfintosc_, 2),
      option_reg_(SingleRegisterBackend<uint8_t>(0xFF)),
      osccon_reg_(SingleRegisterBackend<uint8_t>(0x68 | 0x06)),
      config1_(SingleRegisterBackend<uint16_t>(0xFF)) {}

  void Core::reset() {
    option_reg_.reset();
    osccon_reg_.reset();
    config1_.write(nv_->config()[config1_.ADDR]);
    update_system_clock();
  }

  ICSP Core::enter_icsp() {
    return ICSP(nv_, icsp_reset_);
  }

  uint8_t Core::read_register(uint16_t addr) {
    switch (addr) {
    case 0x81: return option_reg_.read();
    case 0x8F: return osccon_reg_.read();
    default: return 0;
    }
  }

  void Core::write_register(uint16_t addr, uint8_t value) {
    switch (addr) {
    case 0x81: {
      if (value != option_reg_.read()) {
        option_reg_.write(value);
        option_updated_();
      }
      break;
    }
    case 0x8F: {
      osccon_reg_.write_masked(value);
      update_system_clock();
      break;
    }
    }
  }

  sim::core::Advancement Core::advance_to(const sim::core::AdvancementLimit &limit) {
    if (por_->value()) {
      por_->set(false);
    }

    return {};
  }

  void Core::update_system_clock() {
    auto sysclock = system_clock();
    fosc_.select(sysclock.first, sysclock.second);
  }

  std::pair<sim::core::Clock*, int> Core::system_clock() {
    auto fosc = config1_.fosc();

    if (osccon_reg_.scs()) {
      fosc = 5;
    }

    switch (fosc) {
    case 0:  // LP
    case 1:  // XT
    case 2:  // HS
    case 3:  // EC
    case 6:  // RCIO
    case 7:  // RC
      return {extosc_, 1};

    case 4:  // INTOSCIO
    case 5:  // INTOSC
      return {
        osccon_reg_.ircf() == 0 ? &lfintosc_ : &hfintosc_,
        osccon_reg_.ircf() == 0 ? 1 : (1 << (7 - osccon_reg_.ircf())),
      };

    default:
      std::abort();
    }
  }

}  // namespace sim::pic14::internal
