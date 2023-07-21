#include "eusart.h"

REGISTER_TRACE_ENTRY_TYPE(EUSARTDataTraceEntry, sim::pic14::internal::EUSARTDataTraceEntry)

namespace sim::pic14::internal {

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

  void EUSART::AsyncImpl::tsr_loaded(uint16_t v, unsigned int num_bits) {
    sim::core::trace_writer().emplace<EUSARTDataTraceEntry>(EUSARTDataTraceEntry::Mode::ASYNC_TRANSMIT, v, num_bits);
  }

  void EUSART::AsyncImpl::write_register(uint16_t addr, uint8_t value) {
    switch (static_cast<EUSART::Register>(addr)) {
    case EUSART::Register::TXSTA:
      if (!eusart_->txsta_reg_.txen()) {
        eusart_->tx_interrupt_.reset();
        eusart_->set_tx_pin(true);
      }
      break;

    default:
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

        sim::core::trace_writer().emplace<EUSARTDataTraceEntry>(EUSARTDataTraceEntry::Mode::ASYNC_RECEIVED, rsr_ ^ 0x200, rsr_bits_ - 1);
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
    if (eusart_->tsr_empty()) {
      eusart_->update_tx_done();
      return sim::core::SimulationClock::NEVER;
    }

    if (!eusart_->txsta_reg_.txen())
      return sim::core::SimulationClock::NEVER;

    if (eusart_->tx_fosc_.delta() >= eusart_->bit_duration_)
      eusart_->set_pin_from_tsr();

    if (eusart_->tsr_empty()) {
      return eusart_->tx_fosc_.at(eusart_->bit_duration_ / 2);
    }

    return eusart_->tx_fosc_.at(eusart_->bit_duration_);
  }

  EUSART::SyncMasterImpl::SyncMasterImpl(EUSART *eusart)
    : eusart_(eusart) {
    eusart_->set_pin_value(&eusart_->ck_pin_, eusart_->baudctl_reg_.sckp());
    eusart_->set_pin_value(&eusart_->dt_pin_, false);
    eusart_->set_pin_input(&eusart_->ck_pin_, false);
    eusart_->set_pin_input(&eusart_->dt_pin_, !eusart_->txsta_reg_.txen());
  }

  void EUSART::SyncMasterImpl::tsr_loaded(uint16_t v, unsigned int num_bits) {
    sim::core::trace_writer().emplace<EUSARTDataTraceEntry>(EUSARTDataTraceEntry::Mode::SYNC_MASTER_TRANSMIT, v, num_bits);
  }

  void EUSART::SyncMasterImpl::write_register(uint16_t addr, uint8_t value) {
    switch (static_cast<EUSART::Register>(addr)) {
    case EUSART::Register::TXSTA:
      if (!eusart_->txsta_reg_.txen()) {
        eusart_->tx_interrupt_.reset();
        eusart_->set_pin_value(&eusart_->ck_pin_, eusart_->baudctl_reg_.sckp());
      }
      eusart_->set_pin_input(&eusart_->dt_pin_, !eusart_->txsta_reg_.txen());
      // The SYNC bit may have changed, so we also need to configure
      // what RCSTA says.

      // fall through

    case EUSART::Register::RCSTA:
      if (eusart_->rcsta_reg_.cren() || eusart_->rcsta_reg_.sren()) {
        eusart_->schedule_immediately();
      } else {
        eusart_->tx_interrupt_.reset();
      }
      break;

    default:
      break;
    }
  }

  sim::core::Advancement EUSART::SyncMasterImpl::advance_to(const sim::core::AdvancementLimit &limit) {
    if (eusart_->rcsta_reg_.cren() || eusart_->rcsta_reg_.sren())
      return advance_rc();
    else if (eusart_->txsta_reg_.txen())
      return advance_tx();
    else
      return { .next_time = sim::core::SimulationClock::NEVER };
  }

  sim::core::Advancement EUSART::SyncMasterImpl::advance_rc() {
    if (eusart_->rc_fosc_.delta() >= eusart_->bit_duration_ / 2) {
      if (rsr_half_bits_ % 2 == 0) {
        // Clock up: Data pin changes.
        eusart_->set_pin_value(&eusart_->ck_pin_, !eusart_->baudctl_reg_.sckp());
        ++rsr_half_bits_;
      } else {
        // Clock down: Data pin sampling.
        eusart_->set_pin_value(&eusart_->ck_pin_, eusart_->baudctl_reg_.sckp());

        rsr_ >>= 1;
        rsr_ |= (eusart_->dt_pin_.external() ? 0x100u : 0u);
        ++rsr_half_bits_;

        if (rsr_half_bits_ == (eusart_->rcsta_reg_.rx9() ? 2 * 9 : 2 * 8)) {
          if (!eusart_->rcsta_reg_.rx9()) {
            rsr_ >>= 9 - 8;
          }

          sim::core::trace_writer().emplace<EUSARTDataTraceEntry>(EUSARTDataTraceEntry::Mode::SYNC_MASTER_RECEIVED, rsr_, rsr_half_bits_ / 2);
          eusart_->push_rcreg(rsr_);
          rsr_half_bits_ = 0;
          rsr_ = 0;

          if (eusart_->rcsta_reg_.sren()) {
            eusart_->rcsta_reg_.set_sren(false);
            return { .next_time = sim::core::SimulationClock::NEVER };
          }
        }
      }

      eusart_->rc_fosc_.reset();
    }

    return { .next_time = eusart_->rc_fosc_.at((eusart_->bit_duration_ + sim::core::Clock::duration(1)) / 2) };
  }

  sim::core::Advancement EUSART::SyncMasterImpl::advance_tx() {
    if (eusart_->tsr_empty()) {
      if (eusart_->tx_fosc_.delta() >= eusart_->bit_duration_ / 2) {
        eusart_->set_pin_value(&eusart_->ck_pin_, !eusart_->baudctl_reg_.sckp());
        eusart_->update_tx_done();
        return { .next_time = sim::core::SimulationClock::NEVER };
      }
      return { .next_time = eusart_->tx_fosc_.at((eusart_->bit_duration_ + sim::core::Clock::duration(1)) / 2) };
    }

    if (eusart_->tx_fosc_.delta() >= eusart_->bit_duration_) {
      eusart_->set_pin_from_tsr();
      eusart_->set_pin_value(&eusart_->ck_pin_, !eusart_->baudctl_reg_.sckp());
    } else if (eusart_->tx_fosc_.delta() >= eusart_->bit_duration_ / 2) {
      // We don't reset tx_fosc_, so this may be called multiple times
      // per bit.
      eusart_->set_pin_value(&eusart_->ck_pin_, eusart_->baudctl_reg_.sckp());

      return { .next_time = eusart_->tx_fosc_.at(eusart_->bit_duration_) };
    }

    return { .next_time = eusart_->tx_fosc_.at((eusart_->bit_duration_ + sim::core::Clock::duration(1)) / 2) };
  }

  EUSART::SyncSlaveImpl::SyncSlaveImpl(EUSART *eusart)
    : eusart_(eusart) {
    eusart_->set_pin_input(&eusart_->ck_pin_, true);
    eusart_->set_pin_input(&eusart_->dt_pin_, !eusart_->txsta_reg_.txen());
  }

  void EUSART::SyncSlaveImpl::ck_pin_changed(bool v) {
    if (v != eusart_->baudctl_reg_.sckp()) {
      // Set data pin.
      if (eusart_->txsta_reg_.txen()) {
        eusart_->set_pin_from_tsr();
      }
    } else {
      if (eusart_->tsr_empty()) {
        eusart_->update_tx_done();
      }

      if (eusart_->rcsta_reg_.cren()) {
        // Sample data pin.
        rsr_ >>= 1;
        rsr_ |= (eusart_->dt_pin_.external() ? 0x100u : 0u);
        ++rsr_bits_;

        if (rsr_bits_ == (eusart_->rcsta_reg_.rx9() ? 9 : 8)) {
          if (!eusart_->rcsta_reg_.rx9()) {
            rsr_ >>= 9 - 8;
          }

          sim::core::trace_writer().emplace<EUSARTDataTraceEntry>(EUSARTDataTraceEntry::Mode::SYNC_SLAVE_RECEIVED, rsr_, rsr_bits_);
          eusart_->push_rcreg(rsr_);
          rsr_bits_ = 0;
          rsr_ = 0;
        }
      }
    }
  }

  void EUSART::SyncSlaveImpl::tsr_loaded(uint16_t v, unsigned int num_bits) {
    sim::core::trace_writer().emplace<EUSARTDataTraceEntry>(EUSARTDataTraceEntry::Mode::SYNC_SLAVE_TRANSMIT, v, num_bits);
  }

  void EUSART::SyncSlaveImpl::write_register(uint16_t addr, uint8_t value) {
    switch (static_cast<EUSART::Register>(addr)) {
    case EUSART::Register::TXSTA:
      if (!eusart_->txsta_reg_.txen()) {
        eusart_->tx_interrupt_.reset();
        eusart_->set_pin_value(&eusart_->ck_pin_, eusart_->baudctl_reg_.sckp());
      }
      eusart_->set_pin_input(&eusart_->dt_pin_, !eusart_->txsta_reg_.txen());
      // The SYNC bit may have changed, so we also need to configure
      // what RCSTA says.

      // fall through

    case EUSART::Register::RCSTA:
      if (eusart_->rcsta_reg_.cren() || eusart_->rcsta_reg_.sren()) {
        eusart_->schedule_immediately();
      } else {
        eusart_->tx_interrupt_.reset();
      }
      break;

    default:
      break;
    }
  }

  EUSART::EUSART(sim::core::DeviceListener *listener, sim::core::ClockModifier *fosc, InterruptMux::MaskablePeripheralLevelSignal &&rc_interrupt, InterruptMux::MaskablePeripheralLevelSignal &&tx_interrupt)
    : listener_(listener),
      rc_fosc_(fosc),
      tx_fosc_(fosc),
      rc_interrupt_(std::move(rc_interrupt)),
      tx_interrupt_(std::move(tx_interrupt)),
      rc_pin_([this](bool v) {
        std::visit([v](auto &impl) { impl.rc_pin_changed(v); }, impl_);
      }),
      tx_pin_(true),
      ck_pin_([this](bool v) {
        std::visit([v](auto &impl) { impl.ck_pin_changed(v); }, impl_);
      }),
      dt_pin_([](bool v) {}),
      rcsta_reg_(SingleRegisterBackend<uint8_t>(0)),
      txsta_reg_(SingleRegisterBackend<uint8_t>(0)),
      tx_reg_(0),
      spbrg_reg_(0),
      spbrgh_reg_(0),
      baudctl_reg_(SingleRegisterBackend<uint8_t>(0)),
      impl_(AsyncImpl(this)) {}

  void EUSART::reset() {
    set_pin_value(&tx_pin_, true);
    set_pin_input(&ck_pin_, true);
    set_pin_input(&dt_pin_, true);

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
    switch (static_cast<Register>(addr)) {
    case Register::RCSTA: return rcsta_reg_.read();
    case Register::TXREG: return tx_reg_.read();
    case Register::RCREG: return pop_rcreg();
    case Register::TXSTA: return txsta_reg_.read();
    case Register::SPBRG: return spbrg_reg_.read();
    case Register::SPBRGH: return spbrgh_reg_.read();
    case Register::BAUDCTL: return baudctl_reg_.read();
    default: std::abort();
    }
  }

  void EUSART::write_register(uint16_t addr, uint8_t value) {
    switch (static_cast<Register>(addr)) {
    case Register::RCSTA:
      rcsta_reg_.write_masked(value);
      if (!rcsta_reg_.spen() || !rcsta_reg_.cren())
        rcsta_reg_.set_oerr(false);

      update_impl();
      break;

    case Register::TXREG:
      tx_reg_.write(value);

      // Based on SYNC Tx, the TSR can be loaded even when TXEN is
      // off, and later the transmission can be started by setting
      // TXEN.
      if (tsr_empty()) {
        load_tsr();
        if (txsta_reg_.txen())
          schedule_immediately();
      } else {
        tx_reg_valid_ = true;
        tx_interrupt_.reset();
      }
      break;

    case Register::RCREG:
      rcreg_fifo_[rcreg_fifo_tail_] = value;
      break;

    case Register::TXSTA:
      txsta_reg_.write_masked(value);
      update_impl();
      update_bit_duration();

      if (txsta_reg_.txen())
        schedule_immediately();
      break;

    case Register::SPBRG:
      spbrg_reg_.write(value);
      update_bit_duration();
      break;

    case Register::SPBRGH:
      spbrgh_reg_.write(value);
      update_bit_duration();
      break;

    case Register::BAUDCTL:
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
      if (txsta_reg_.sync()) {
        if (txsta_reg_.csrc() && !std::holds_alternative<SyncMasterImpl>(impl_))
          impl_.emplace<SyncMasterImpl>(this);
        else if (!txsta_reg_.csrc() && !std::holds_alternative<SyncSlaveImpl>(impl_))
          impl_.emplace<SyncSlaveImpl>(this);
      } else if (!std::holds_alternative<AsyncImpl>(impl_)) {
        impl_.emplace<AsyncImpl>(this);
      }
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

  void EUSART::update_tx_done() {
    txsta_reg_.update_tx_done();
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
      std::visit([](auto &impl) { impl.tsr_loaded(0, 12); }, impl_);
    } else {
      tsr_ = tx_reg_.read() | (txsta_reg_.tx9() ? (txsta_reg_.tx9d() << 8) : 0);
      if (!txsta_reg_.sync() && baudctl_reg_.sckp())
        tsr_ ^= 0x1FF;

      tsr_bits_ = (txsta_reg_.tx9() ? 9 : 8);
      std::visit([this](auto &impl) { impl.tsr_loaded(tsr_, tsr_bits_); }, impl_);

      if (!txsta_reg_.sync()) {
        // Add start and stop bits.
        tsr_ |= 1u << tsr_bits_;
        tsr_ <<= 1;
        tsr_bits_ += 2;
      }
    }

    txsta_reg_.set_trmt(false);

    tx_reg_valid_ = false;
    tx_interrupt_.raise();

    tx_fosc_.reset();
  }

  void EUSART::set_pin_from_tsr() {
    if (tsr_empty()) {
      return;
    }

    set_tx_pin((tsr_ & 1) != 0);

    tsr_ >>= 1;
    --tsr_bits_;

    if (tsr_empty()) {
      if (tx_reg_valid_) {
        load_tsr();
      }
    } else {
      tx_fosc_.reset();
    }
  }

  void EUSART::set_tx_pin(bool v) {
    set_pin_value(txsta_reg_.sync() ? &dt_pin_ : &tx_pin_, v);
  }

  void EUSART::set_pin_value(OutputPin *pin, bool v) {
    if ((pin->value() >= 0.5) == v)
      return;

    pin->set_value(v);
    listener_->pin_changed(pin, sim::core::DeviceListener::VALUE);
  }

  void EUSART::set_pin_input(BiDiPin *pin, bool v) {
    if (pin->input() == v)
      return;

    pin->set_input(v);
    listener_->pin_changed(pin, sim::core::DeviceListener::RESISTANCE);
  }

}  // namespace sim::pic14::internal
