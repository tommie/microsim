#include "timer0.h"

namespace sim::pic14::internal {

  Timer0::Timer0(sim::core::ClockModifier *fosc, InterruptMux::MaskableIntconEdgeSignal &&interrupt, Core::OptionReg &&option_reg)
    : fosc_(fosc),
      interrupt_(std::move(interrupt)),
      option_reg_(std::move(option_reg)),
      pin_t0cki_(std::bind_front(&Timer0::t0cki_changed, this)),
      prescaler_(prescaled()) {}

  void Timer0::reset() {
    interrupt_.reset();
  }

  sim::core::Advancement Timer0::advance_to(const sim::core::AdvancementLimit &limit) {
    if (option_reg_.t0cs()) {
      return {};
    }

    auto delta = fosc_.delta();
    fosc_.reset((prescaler_ * TICKS_PER_COUNT - delta % (prescaler_ * TICKS_PER_COUNT)) % (prescaler_ * TICKS_PER_COUNT));
    long tmr0 = tmr0_reg_ + delta / (prescaler_ * TICKS_PER_COUNT);

    tmr0_reg_ = static_cast<uint8_t>(tmr0);
    prescaler_ = prescaled();

    if (tmr0 >= 256) {
      interrupt_.raise();
    }

    return { .next_time = fosc_.at((256 - (tmr0 & 0xFF)) * prescaler_ * TICKS_PER_COUNT) };
  }

  uint8_t Timer0::read_register(uint16_t addr) {
    switch (addr) {
    case 0x01: return tmr0_reg_ + (option_reg_.t0cs() ? 0 : fosc_.delta() / prescaler_ / TICKS_PER_COUNT);
    default: return 0;
    }
  }

  void Timer0::write_register(uint16_t addr, uint8_t value) {
    switch (addr) {
    case 0x01:
      tmr0_reg_ = value + 2;
      if (option_reg_.t0cs()) {
        prescaler_ = prescaled();
      } else {
        fosc_.reset();
        schedule_immediately();
      }
      break;
    }
  }

  void Timer0::t0cki_changed(bool rising) {
    if (option_reg_.t0cs() && rising == !option_reg_.t0se()) {
      if (!--prescaler_) {
        prescaler_ = prescaled();
        if (!++tmr0_reg_)
          schedule_immediately();
      }
    }
  }

  int Timer0::prescaled() const {
    return option_reg_.psa() ? 1 : (2 << option_reg_.ps());
  }

}  // namespace sim::pic14::internal
