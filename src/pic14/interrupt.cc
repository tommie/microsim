#include "interrupt.h"

namespace sim::pic14::internal {

  void InterruptMux::MaskableIntconEdgeSignal::raise() {
    if (!active_) {
      mux_->intcon_ |= flag_mask_;
      active_ = true;

      if (mux_->is_active()) mux_->interrupt_.emit();
    }
  }

  InterruptMux::MaskableIntconEdgeSignal InterruptMux::make_maskable_edge_signal_intcon(uint8_t en_bit, uint8_t flag_bit) {
    intcon_en_bits_[flag_bit] = en_bit;
    return MaskableIntconEdgeSignal(this, 1u << flag_bit);
  }

  bool InterruptMux::is_active() const {
    for (size_t i = 0; i < intcon_en_bits_.size(); ++i) {
      uint8_t en_bit = intcon_en_bits_[i];
      if (en_bit == 0xFF) continue;
      if (!(intcon_ & (1u << i))) continue;
      if (intcon_ & (1u << en_bit)) return true;
    }

    for (size_t i = 0; i < pie_.size(); ++i) {
      if (pir_[i] & pie_[i]) return true;
    }

    return false;
  }

  void InterruptMux::reset() {
    intcon_ = 0;
    for (size_t i = 0; i < pie_.size(); ++i) pie_[i] = 0;
    for (size_t i = 0; i < pir_.size(); ++i) pir_[i] = 0;
  }

  uint8_t InterruptMux::read_register(uint16_t addr) {
    if (addr == 0x0B) {
      return intcon_;
    } else if (addr >= 0x0C && addr < 0x0C + pir_.size()) {
      return pir_[addr - 0x0C];
    } else if (addr >= 0x8C && addr < 0x8C + pie_.size()) {
      return pie_[addr - 0x8C];
    } else {
      std::abort();
    }
  }

  void InterruptMux::write_register(uint16_t addr, uint8_t value) {
    if (addr == 0x0B) {
      intcon_ = value;
    } else if (addr >= 0x0C && addr < 0x0C + pir_.size()) {
      pir_[addr - 0x0C] = value;
    } else if (addr >= 0x8C && addr < 0x8C + pie_.size()) {
      pie_[addr - 0x8C] = value;
    }
  }

}  // namespace sim::pic14::internal
