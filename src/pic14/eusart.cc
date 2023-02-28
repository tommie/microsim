#include "eusart.h"

namespace sim::pic14::internal {

  void EUSART::ResetImpl::rc_pin_changed(bool v) {}

  void EUSART::ResetImpl::write_register(uint16_t addr, uint8_t value) {}

  sim::core::Advancement EUSART::ResetImpl::advance_to(const sim::core::AdvancementLimit &limit) {
    return {};
  }

  EUSART::AsyncImpl::AsyncImpl(EUSART *eusart)
    : eusart_(eusart) {
    eusart->set_tx_pin(true);
  }

  void EUSART::AsyncImpl::rc_pin_changed(bool v) {
  }

  void EUSART::AsyncImpl::write_register(uint16_t addr, uint8_t value) {
    switch (addr) {
    case 0x19:  // TXREG
      if (!eusart_->txsta_reg_.txen()) {
        break;
      }

      if (tsr_bits_ == 0) {
        load_tsr();
        eusart_->schedule_immediately();
      } else {
        tx_reg_valid_ = true;
        eusart_->tx_interrupt_.reset();
      }
      break;

    case 0x98:
      if (!eusart_->txsta_reg_.txen()) {
        eusart_->tx_interrupt_.reset();
        eusart_->set_tx_pin(true);
      }
      break;
    }
  }

  sim::core::Advancement EUSART::AsyncImpl::advance_to(const sim::core::AdvancementLimit &limit) {
    if (!eusart_->txsta_reg_.txen())
      return {};

    if (tsr_bits_ > 0 && eusart_->fosc_.delta() >= eusart_->bit_duration_) {
      eusart_->set_tx_pin((tsr_ & 1) != 0);

      tsr_ >>= 1;
      --tsr_bits_;

      if (tsr_bits_ == 0) {
        eusart_->txsta_reg_.update_tx_done();

        if (tx_reg_valid_) {
          load_tsr();
        }
      } else {
        eusart_->fosc_.reset();
      }
    }

    return {
      .next_time = (tsr_bits_ > 0 ? eusart_->fosc_.at(eusart_->bit_duration_) : sim::core::SimulationClock::NEVER),
    };
  }

  void EUSART::AsyncImpl::load_tsr() {
    if (eusart_->txsta_reg_.sendb()) {
      tsr_ = 1;
      tsr_bits_ = 14;
    } else {
      tsr_ = eusart_->tx_reg_.read() | (eusart_->txsta_reg_.tx9() ? (eusart_->txsta_reg_.tx9d() << 8) : 0);
      tsr_ |= 1u << (eusart_->txsta_reg_.tx9() ? 9 : 8);
      tsr_ <<= 1;
      tsr_bits_ = (eusart_->txsta_reg_.tx9() ? 11 : 10);
    }

    eusart_->txsta_reg_.set_trmt(false);

    tx_reg_valid_ = false;
    eusart_->tx_interrupt_.raise();

    eusart_->fosc_.reset();
  }

  EUSART::SyncImpl::SyncImpl(EUSART *eusart)
    : eusart_(eusart) {}

  void EUSART::SyncImpl::rc_pin_changed(bool v) {
  }

  void EUSART::SyncImpl::write_register(uint16_t addr, uint8_t value) {
  }

  sim::core::Advancement EUSART::SyncImpl::advance_to(const sim::core::AdvancementLimit &limit) {
    return {};
  }

  EUSART::EUSART(sim::core::DeviceListener *listener, sim::core::ClockModifier *fosc, InterruptMux::MaskablePeripheralEdgeSignal &&rc_interrupt, InterruptMux::MaskablePeripheralLevelSignal &&tx_interrupt)
    : listener_(listener),
      fosc_(fosc),
      rc_interrupt_(std::move(rc_interrupt)),
      tx_interrupt_(std::move(tx_interrupt)),
      rc_pin_([this](bool v) {
        std::visit([this, v](auto &impl) { impl.rc_pin_changed(v); }, impl_);
      }),
      tx_pin_(true),
      impl_(AsyncImpl(this)),
      rcsta_reg_(SingleRegisterBackend<uint8_t>(0)),
      txsta_reg_(SingleRegisterBackend<uint8_t>(0)),
      tx_reg_(0),
      rc_reg_(0),
      spbrg_reg_(0),
      spbrgh_reg_(0),
      baudctl_reg_(SingleRegisterBackend<uint8_t>(0)) {}

  void EUSART::reset() {
    tx_pin_.set_value(true);

    tx_interrupt_.reset();
    rc_interrupt_.reset();

    rcsta_reg_.reset();
    tx_reg_.write(0);
    rc_reg_.write(0);
    txsta_reg_.reset();
    spbrg_reg_.write(0);
    spbrgh_reg_.write(0);
    baudctl_reg_.reset();

    update_impl();
    update_bit_duration();
  }

  uint8_t EUSART::read_register(uint16_t addr) {
    switch (addr) {
    case 0x18: return rcsta_reg_.read();
    case 0x19: return tx_reg_.read();
    case 0x1A: return rc_reg_.read();
    case 0x98: return txsta_reg_.read();
    case 0x99: return spbrg_reg_.read();
    case 0x9A: return spbrgh_reg_.read();
    case 0x187: return baudctl_reg_.read();
    default: return 0;
    }
  }

  void EUSART::write_register(uint16_t addr, uint8_t value) {
    switch (addr) {
    case 0x18:
      rcsta_reg_.write_masked(value);
      update_impl();
      break;

    case 0x19: tx_reg_.write(value); break;
    case 0x1A: rc_reg_.write(value); break;

    case 0x98:
      txsta_reg_.write_masked(value);
      update_impl();
      update_bit_duration();
      break;

    case 0x99:
      spbrg_reg_.write(value);
      update_bit_duration();
      break;

    case 0x9A:
      spbrgh_reg_.write(value);
      update_bit_duration();
      break;

    case 0x187:
      baudctl_reg_.write_masked(value);
      update_bit_duration();
      break;
    }

    std::visit([addr, value](auto &impl) { impl.write_register(addr, value); }, impl_);
  }

  sim::core::Advancement EUSART::advance_to(const sim::core::AdvancementLimit &limit) {
    return std::visit([&limit](auto &impl) { return impl.advance_to(limit); }, impl_);
  }

  void EUSART::update_impl() {
    if (rcsta_reg_.spen()) {
      if (txsta_reg_.sync() && !std::holds_alternative<SyncImpl>(impl_))
        impl_.emplace<SyncImpl>(this);
      else if (!txsta_reg_.sync() && !std::holds_alternative<AsyncImpl>(impl_))
        impl_.emplace<AsyncImpl>(this);
    } else if (!std::holds_alternative<ResetImpl>(impl_)) {
      impl_.emplace<ResetImpl>();
      tx_interrupt_.reset();
    }
  }

  void EUSART::update_bit_duration() {
    unsigned long n = spbrg_reg_.read() + 1;

    if (baudctl_reg_.brg16()) {
      n += (static_cast<unsigned int>(spbrgh_reg_.read()) << 8);
    }

    n *= 4;

    if (!txsta_reg_.sync()) {
      if (!txsta_reg_.brgh()) {
        n *= 4;
      }
      if (!baudctl_reg_.brg16()) {
        n *= 4;
      }
    }

    auto dur = sim::core::Clock::duration(n);

    if (bit_duration_ != dur) {
      bit_duration_ = dur;
      fosc_.reset();
    }
  }

  void EUSART::set_tx_pin(bool v) {
    if ((tx_pin_.value() >= 0.5) == v)
      return;

    tx_pin_.set_value(v);
    listener_->pin_changed(&tx_pin_, sim::core::DeviceListener::VALUE);
  }

}  // namespace sim::pic14::internal
