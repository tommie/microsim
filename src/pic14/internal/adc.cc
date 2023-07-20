#include "adc.h"

#include <cstdlib>
#include <limits>

REGISTER_TRACE_ENTRY_TYPE(ADConversionDoneTraceEntry, sim::pic14::ADConversionDoneTraceEntry)

namespace sim::pic14::internal {

  ADConverter::ADConverter(sim::core::ClockModifier *fosc, InterruptMux::MaskablePeripheralEdgeSignal &&interrupt)
    : fosc_(fosc),
      interrupt_(std::move(interrupt)),
      input_pins_(13),
      avdd_pin_(1),
      frc_(sim::core::Microseconds(4)),
      frc_view_(&frc_),
      adcon0_reg_(SingleRegisterBackend<uint8_t>(0)),
      adcon1_reg_(SingleRegisterBackend<uint8_t>(0)),
      adresl_reg_(0),
      adresh_reg_(0) {}

  void ADConverter::reset() {
    adcon0_reg_.reset();
    adcon1_reg_.reset();
  }

  uint8_t ADConverter::read_register(uint16_t addr) {
    switch (static_cast<Register>(addr)) {
    case Register::ADCON0: return adcon0_reg_.read();
    case Register::ADCON1: return adcon1_reg_.read();
    case Register::ADRESL: return adresl_reg_.read();
    case Register::ADRESH: return adresh_reg_.read();
    default: std::abort();
    }
  }

  void ADConverter::write_register(uint16_t addr, uint8_t value) {
    switch (static_cast<Register>(addr)) {
    case Register::ADCON0:
      adcon0_reg_.write(value);
      if (adcon0_reg_.go() && adcon0_reg_.adon()) {
        if (adcon0_reg_.adcs() == 3)
          frc_view_.reset();
        else
          fosc_.reset();

        schedule_immediately();
      }
      break;

    case Register::ADCON1: adcon1_reg_.write_masked(value); break;
    case Register::ADRESL: adresl_reg_.write(value); break;
    case Register::ADRESH: adresh_reg_.write(value); break;
    }
  }

  static constexpr auto TICKS_PER_CONVERSION = sim::core::Clock::duration(11);
  static constexpr std::array<int, 4> FOSC_PRESCALER = { 2, 8, 32, 1 };

  sim::core::Advancement ADConverter::advance_to(const sim::core::AdvancementLimit &limit) {
    if (!adcon0_reg_.go() || !adcon0_reg_.adon()) {
      return {};
    }

    auto end_tick = TICKS_PER_CONVERSION * FOSC_PRESCALER[adcon0_reg_.adcs()];

    if ((adcon0_reg_.adcs() == 3 ? frc_view_.delta() : fosc_.delta()) >= end_tick) {
      double v = sample();
      double neg = adcon1_reg_.vcfg0() ? vref_neg_pin().external() : 0;
      double pos = adcon1_reg_.vcfg1() ? vref_pos_pin().external() : avdd_pin_.external();

      if (v < neg) v = neg;
      if (v > pos) v = pos;

      uint16_t vv = std::numeric_limits<uint16_t>::max() * (v - neg) / (pos - neg);

      if (adcon1_reg_.adfm()) vv >>= 6;
      else vv &= 0xFFC0;

      adresl_reg_.write(vv);
      adresh_reg_.write(vv >> 8);
      sim::core::trace_writer().emplace<ADConversionDoneTraceEntry>();
      adcon0_reg_.set_go(false);
      interrupt_.raise();

      return {};
    }

    return {
      .next_time = adcon0_reg_.adcs() == 3 ? frc_view_.at(end_tick) : fosc_.at(end_tick),
    };
  }

  double ADConverter::sample() const {
    switch (adcon0_reg_.chs()) {
    case 0x0E:
      std::abort();  // TODO: CVREF Not implemented.

    case 0x0F:
      return 0.6;  // 0.6 V fixed reference.

    default:
      return input_pins_[adcon0_reg_.chs()].external();
    }
  }

}  // namespace sim::pic14::internal
