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
    if (!eusart_->rcsta_reg_.spen() || !eusart_->rcsta_reg_.cren())
      return;

    if (eusart_->baudctl_reg_.wue()) {
      if (!v) {
        eusart_->push_rcreg(0);
      } else {
        eusart_->baudctl_reg_.set_wue(false);
      }
    } else if (eusart_->baudctl_reg_.abden()) {
      if (v) {
        if (rsr_bits_ == 0) {
          eusart_->rc_fosc_.reset();
          eusart_->baudctl_reg_.set_rcidl(false);
        }

        if (++rsr_bits_ == 5) {
          unsigned long brg = eusart_->rc_fosc_.delta().count() / 8;

          eusart_->spbrg_reg_.write(brg);
          eusart_->spbrgh_reg_.write(brg >> 8);
          eusart_->baudctl_reg_.update_abd_done(brg >= 0x10000);
          eusart_->push_rcreg(0);
          rsr_bits_ = 0;
        }
      }
    } else if (!v && rsr_bits_ == 0) {
      eusart_->rc_fosc_.reset();
      eusart_->baudctl_reg_.set_rcidl(false);
      eusart_->schedule_immediately();
    }
  }

  void EUSART::AsyncImpl::write_register(uint16_t addr, uint8_t value) {
    switch (addr) {
    case 0x98:
      if (!eusart_->txsta_reg_.txen()) {
        eusart_->tx_interrupt_.reset();
        eusart_->set_tx_pin(true);
      }
      break;
    }
  }

  sim::core::Advancement EUSART::AsyncImpl::advance_to(const sim::core::AdvancementLimit &limit) {
    auto next_time = advance_rc();

    if (auto nt = advance_tx(); !is_never(nt) && (is_never(next_time) || next_time > nt))
      next_time = nt;

    return { .next_time = next_time };
  }

  sim::core::TimePoint EUSART::AsyncImpl::advance_rc() {
    if (!eusart_->rcsta_reg_.cren())
      return sim::core::SimulationClock::NEVER;

    if (eusart_->baudctl_reg_.abden() || eusart_->baudctl_reg_.wue())
      return sim::core::SimulationClock::NEVER;

    if (rsr_bits_ == 0) {
      if (eusart_->rc_fosc_.delta() < eusart_->bit_duration_ / 2) {
        return eusart_->rc_fosc_.at((eusart_->bit_duration_ + sim::core::Clock::duration(1)) / 2);
      }

      if (eusart_->rc_pin_.external()) {
        // The start bit was compromised.
        return sim::core::SimulationClock::NEVER;
      }

      eusart_->rc_fosc_.reset();
      ++rsr_bits_;
      rsr_ = 0;
    } else if (eusart_->rc_fosc_.delta() >= eusart_->bit_duration_) {
      rsr_ >>= 1;
      rsr_ |= (eusart_->rc_pin_.external() ? 1u : 0u) << 15;

      if (rsr_bits_ == (eusart_->rcsta_reg_.rx9() ? 10 : 9)) {
        if (eusart_->rcsta_reg_.rx9()) {
          rsr_ >>= 16 - 8;
        } else {
          rsr_ >>= 16 - 9;
          rsr_ |= (rsr_ << 1) & 0x200;  // Promote the last bit to FERR.
        }

        eusart_->push_rcreg(rsr_);
        rsr_bits_ = 0;

        return sim::core::SimulationClock::NEVER;
      }

      eusart_->rc_fosc_.reset();
      ++rsr_bits_;
    }

    return eusart_->rc_fosc_.at(eusart_->bit_duration_);
  }

  sim::core::TimePoint EUSART::AsyncImpl::advance_tx() {
    if (!eusart_->txsta_reg_.txen())
      return sim::core::SimulationClock::NEVER;

    if (eusart_->tsr_empty())
      return sim::core::SimulationClock::NEVER;

    if (eusart_->tx_fosc_.delta() >= eusart_->bit_duration_)
      eusart_->set_pin_from_tsr();

    return eusart_->tx_fosc_.at(eusart_->bit_duration_);
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

  EUSART::EUSART(sim::core::DeviceListener *listener, sim::core::ClockModifier *fosc, InterruptMux::MaskablePeripheralLevelSignal &&rc_interrupt, InterruptMux::MaskablePeripheralLevelSignal &&tx_interrupt)
    : listener_(listener),
      rc_fosc_(fosc),
      tx_fosc_(fosc),
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
      spbrg_reg_(0),
      spbrgh_reg_(0),
      baudctl_reg_(SingleRegisterBackend<uint8_t>(0)) {}

  void EUSART::reset() {
    tx_pin_.set_value(true);

    tx_interrupt_.reset();
    rc_interrupt_.reset();

    rcsta_reg_.reset();
    tx_reg_.write(0);
    txsta_reg_.reset();
    spbrg_reg_.write(0);
    spbrgh_reg_.write(0);
    baudctl_reg_.reset();
    rcreg_fifo_head_ = 0;
    rcreg_fifo_tail_ = 0;

    tsr_bits_ = 0;
    tx_reg_valid_ = false;

    update_impl();
    update_bit_duration();
  }

  uint8_t EUSART::read_register(uint16_t addr) {
    switch (addr) {
    case 0x18: return rcsta_reg_.read();
    case 0x19: return tx_reg_.read();
    case 0x1A: return pop_rcreg();
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
      if (!rcsta_reg_.spen() || !rcsta_reg_.cren())
        rcsta_reg_.set_oerr(false);

      update_impl();
      break;

    case 0x19:
      tx_reg_.write(value);

      if (txsta_reg_.txen()) {
        if (tsr_empty()) {
          load_tsr();
          schedule_immediately();
        } else {
          tx_reg_valid_ = true;
          tx_interrupt_.reset();
        }
      }
      break;

    case 0x1A:
      rcreg_fifo_[rcreg_fifo_tail_] = value;
      break;

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
      rc_interrupt_.reset();
      tsr_bits_ = 0;
      tx_reg_valid_ = false;
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
      rc_fosc_.reset();
      tx_fosc_.reset();
    }
  }

  void EUSART::push_rcreg(uint16_t v) {
    baudctl_reg_.set_rcidl(true);

    if (rcsta_reg_.oerr())
      return;

    rcreg_fifo_[rcreg_fifo_head_] = v;
    if (rcreg_fifo_head_ == rcreg_fifo_tail_) {
      rcsta_reg_.set_rx9d((v & 0x100) != 0);
      rcsta_reg_.set_ferr((v & 0x200) == 0);  // Inverted.
    }

    if (++rcreg_fifo_head_ >= rcreg_fifo_.size())
      rcreg_fifo_head_ = 0;

    if (rcreg_fifo_head_ == rcreg_fifo_tail_)
      rcsta_reg_.set_oerr(true);

    rc_interrupt_.raise();
  }

  uint8_t EUSART::pop_rcreg() {
    if (rcsta_reg_.oerr())
      return 0;

    auto v = rcreg_fifo_[rcreg_fifo_tail_];

    if (++rcreg_fifo_tail_ >= rcreg_fifo_.size())
      rcreg_fifo_tail_ = 0;

    auto next = rcreg_fifo_[rcreg_fifo_tail_];
    rcsta_reg_.set_rx9d((next & 0x100) != 0);
    rcsta_reg_.set_ferr((next & 0x200) == 0);  // Inverted.
    if (rcreg_fifo_tail_ == rcreg_fifo_head_)
      rc_interrupt_.reset();

    return v;
  }

  void EUSART::load_tsr() {
    if (txsta_reg_.sendb()) {
      tsr_ = 1;
      tsr_bits_ = 14;
    } else {
      tsr_ = tx_reg_.read() | (txsta_reg_.tx9() ? (txsta_reg_.tx9d() << 8) : 0);
      tsr_ |= 1u << (txsta_reg_.tx9() ? 9 : 8);
      tsr_ <<= 1;
      tsr_bits_ = (txsta_reg_.tx9() ? 11 : 10);
    }

    txsta_reg_.set_trmt(false);

    tx_reg_valid_ = false;
    tx_interrupt_.raise();

    tx_fosc_.reset();
  }

  void EUSART::set_pin_from_tsr() {
    set_tx_pin((tsr_ & 1) != 0);

    tsr_ >>= 1;
    --tsr_bits_;

    if (tsr_empty()) {
      if (tx_reg_valid_) {
        load_tsr();
      } else {
        txsta_reg_.update_tx_done();
      }
    } else {
      tx_fosc_.reset();
    }
  }

  void EUSART::set_tx_pin(bool v) {
    auto *pin = &tx_pin_;

    if ((pin->value() >= 0.5) == v)
      return;

    pin->set_value(v);
    listener_->pin_changed(pin, sim::core::DeviceListener::VALUE);
  }

}  // namespace sim::pic14::internal
