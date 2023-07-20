#include "interrupt.h"

namespace sim::pic14::internal {

  void InterruptMux::MaskableIntconEdgeSignal::raise() {
    if (!active_) {
      mux_->intcon_.set_flags(flag_mask_);
      active_ = true;

      if (mux_->is_active()) mux_->interrupt_();
    }
  }

  void InterruptMux::MaskablePeripheralEdgeSignal::raise() {
    if (!active_) {
      mux_->pir_[reg_index_] |= flag_mask_;
      active_ = true;

      if (mux_->is_active()) mux_->interrupt_();
    }
  }

  void InterruptMux::MaskablePeripheralLevelSignal::reset() {
    mux_->pir_[reg_index_] &= ~flag_mask_;
    active_ = false;
  }

  void InterruptMux::MaskablePeripheralLevelSignal::raise() {
    if (!active_) {
      mux_->pir_[reg_index_] |= flag_mask_;
      active_ = true;

      if (mux_->is_active()) mux_->interrupt_();
    }
  }

  InterruptMux::MaskableIntconEdgeSignal InterruptMux::make_maskable_edge_signal_intcon(uint8_t en_bit, uint8_t flag_bit) {
    intcon_en_bits_[flag_bit] = en_bit;
    return MaskableIntconEdgeSignal(this, 1u << flag_bit);
  }

  InterruptMux::MaskablePeripheralEdgeSignal InterruptMux::make_maskable_edge_signal_peripheral(uint8_t bit) {
    return MaskablePeripheralEdgeSignal(this, bit / 8, 1u << (bit % 8));
  }

  InterruptMux::MaskablePeripheralLevelSignal InterruptMux::make_maskable_level_signal_peripheral(uint8_t bit) {
    pir_ro_mask_[bit / 8] |= (1u << (bit % 8));
    return MaskablePeripheralLevelSignal(this, bit / 8, 1u << (bit % 8));
  }

  bool InterruptMux::is_active() const {
    for (size_t i = 0; i < intcon_en_bits_.size(); ++i) {
      uint8_t en_bit = intcon_en_bits_[i];
      if (en_bit == 0xFF) continue;
      if (!intcon_.any_flag_set(1u << i)) continue;
      if (intcon_.any_flag_set(1u << en_bit)) return true;
    }

    if (intcon_.peie()) {
      for (size_t i = 0; i < pie_.size(); ++i) {
        if (pir_[i] & pie_[i]) return true;
      }
    }

    return false;
  }

  void InterruptMux::reset() {
    intcon_.reset();
    for (size_t i = 0; i < pie_.size(); ++i) pie_[i] = 0;
    for (size_t i = 0; i < pir_.size(); ++i) pir_[i] = 0;
  }

  uint8_t InterruptMux::read_register(uint16_t addr) {
    switch (static_cast<Register>(addr)) {
    case Register::INTCON: return intcon_.read();
    case Register::PIR1:
    case Register::PIR2: return pir_[addr - static_cast<uint16_t>(Register::PIR1)];
    case Register::PIE1:
    case Register::PIE2: return pie_[addr - static_cast<uint16_t>(Register::PIE1)];
    default: std::abort();
    }
  }

  void InterruptMux::write_register(uint16_t addr, uint8_t value) {
    switch (static_cast<Register>(addr)) {
    case Register::INTCON:
      intcon_.write(value);
      break;

    case Register::PIR1:
    case Register::PIR2: {
      uint8_t ro_mask = pir_ro_mask_[addr - static_cast<uint16_t>(Register::PIR1)];
      pir_[addr - static_cast<uint16_t>(Register::PIR1)] = (pir_[addr - static_cast<uint16_t>(Register::PIR1)] & ro_mask) | (value & ~ro_mask);
      break;
    }
    case Register::PIE1:
    case Register::PIE2:
      pie_[addr - static_cast<uint16_t>(Register::PIE1)] = value;
      break;
    }
  }

}  // namespace sim::pic14::internal
