#include "eprom.h"

#include "execution.h"

REGISTER_TRACE_ENTRY_TYPE(WroteEEDATATraceEntry, sim::pic14::WroteEEDATATraceEntry)
REGISTER_TRACE_ENTRY_TYPE(WroteProgramFlashTraceEntry, sim::pic14::WroteProgramFlashTraceEntry)

namespace sim::pic14::internal {

  template<int PgmDatBufSize, const std::array<uint16_t, 4> SelfWriteCutoffs>
  EPROM<PgmDatBufSize, SelfWriteCutoffs>::EPROM(NonVolatile *nv, sim::core::Clock *lfintosc, const Core::Config2Reg *config2, InterruptMux::MaskablePeripheralEdgeSignal &&interrupt, Executor *exec)
    : nv_(nv),
      lfintosc_(lfintosc),
      config2_(config2),
      interrupt_(std::move(interrupt)),
      exec_(exec),
      eedat_reg_(0),
      eedath_reg_(0),
      eeadr_reg_(0),
      eeadrh_reg_(0),
      eecon1_reg_(SingleRegisterBackend<uint8_t>(0)) {}

  template<int PgmDatBufSize, const std::array<uint16_t, 4> SelfWriteCutoffs>
  void EPROM<PgmDatBufSize, SelfWriteCutoffs>::reset() {
    eedat_reg_.write(0);
    eedath_reg_.write(0);
    eeadr_reg_.write(0);
    eeadrh_reg_.write(0);
    eecon1_reg_.reset(eecon1_reg_.wr());
    eecon2_reg_ = 0;
    pgmdat_buf_head_ = 0;
    inhibitor_.reset();
  }

  template<int PgmDatBufSize, const std::array<uint16_t, 4> SelfWriteCutoffs>
  uint8_t EPROM<PgmDatBufSize, SelfWriteCutoffs>::read_register(uint16_t addr) {
    switch (static_cast<Register>(addr)) {
    case Register::EEDAT: return eedat_reg_.read();
    case Register::EEADR: return eeadr_reg_.read();
    case Register::EEDATH: return eedath_reg_.read();
    case Register::EEADRH: return eeadrh_reg_.read();
    case Register::EECON1: return eecon1_reg_.read();
    case Register::EECON2: return 0;
    default: std::abort();
    }
  }

  template<int PgmDatBufSize, const std::array<uint16_t, 4> SelfWriteCutoffs>
  void EPROM<PgmDatBufSize, SelfWriteCutoffs>::write_register(uint16_t addr, uint8_t value) {
    switch (static_cast<Register>(addr)) {
    case Register::EEDAT: eedat_reg_.write(value); break;
    case Register::EEADR: eeadr_reg_.write(value); break;
    case Register::EEDATH: {
      eedath_reg_.write(value);
      push_pgmdat((static_cast<uint16_t>(value) << 8) | eedat_reg_.read());
      break;
    }
    case Register::EEADRH: eeadrh_reg_.write(value); break;

    case Register::EECON1:
      if ((eecon1_reg_.rd() || eecon1_reg_.wr()) && (value & ((1u << EECon1Reg::RD) | (1u << EECon1Reg::WR))))
        break;

      // RD and WR cannot be cleared in software.
      eecon1_reg_.write_masked(value | (eecon1_reg_.read() & ((1u << EECon1Reg::RD) | (1u << EECon1Reg::WR))));

      if (eecon1_reg_.rd()) {
        if (eecon1_reg_.eepgd()) {
          uint16_t addr = (static_cast<uint16_t>(eeadrh_reg_.read()) << 8) | eeadr_reg_.read();
          uint16_t v = nv_->progmem()[addr % nv_->progmem().size()];

          push_pgmdat(v);
          eedat_reg_.write(v);
          eedath_reg_.write(v >> 8);
          exec_->inhibit(2);
          eecon1_reg_.set_rd(false);
        } else {
          eedat_reg_.write(nv_->eedata()[eeadr_reg_.read() % nv_->eedata().size()]);
          eecon1_reg_.set_rd(false);
        }
      } else if (eecon1_reg_.wr()) {
        if (!eecon1_reg_.wren() || eecon2_reg_ != EECON2_MAGIC) {
          eecon1_reg_.set_wr(false);
          return;
        }

        if (eecon1_reg_.eepgd()) {
          if ((eeadr_reg_.read() & (pgmdat_buf_.size() - 1)) == pgmdat_buf_.size() - 1) {
            // Perform erase/write.
            inhibitor_ = exec_->inhibit(2);
          } else {
            // Just load into the buffer.
            exec_->inhibit(2);
            eecon1_reg_.set_wr(false);
            break;
          }
        }

        lfintosc_.reset();
        schedule_immediately();
      }
      break;

    case Register::EECON2:
      eecon2_reg_ = (eecon2_reg_ << 8) | value;
      break;
    }
  }

  template<int PgmDatBufSize, const std::array<uint16_t, 4> SelfWriteCutoffs>
  sim::core::Advancement EPROM<PgmDatBufSize, SelfWriteCutoffs>::advance_to(const sim::core::AdvancementLimit &limit) {
    auto next_time = sim::core::SimulationClock::NEVER;

    if (eecon1_reg_.wr()) {
      if (eecon1_reg_.eepgd()) {
        if (lfintosc_.delta().count() * lfintosc_.interval() >= TPEW) {
          uint16_t addr = (static_cast<uint16_t>(eeadrh_reg_.read()) << 8) | eeadr_reg_.read();

          if (addr >= SelfWriteCutoffs[config2_->wrt()]) {
            addr -= pgmdat_buf_.size() - 1;

            if ((addr & 0x0F) == 0x00) {
              // Erase.
              nv_->progmem().replace(nv_->progmem().begin() + addr, nv_->progmem().begin() + addr + 0x10, 0x10, 0x3FFF);
            }

            // Write.
            for (unsigned int i = 0; i < pgmdat_buf_.size(); ++i) {
              nv_->progmem()[(addr + i) % nv_->progmem().size()] = pgmdat_buf_[pgmdat_buf_head_];
              if (++pgmdat_buf_head_ == pgmdat_buf_.size())
                pgmdat_buf_head_ = 0;
            }

            sim::core::trace_writer().emplace<WroteProgramFlashTraceEntry>(addr % nv_->progmem().size());
          }

          eecon1_reg_.set_wr(false);
          inhibitor_.reset();
        } else {
          next_time = lfintosc_.at(sim::core::Clock::duration((TPEW + lfintosc_.interval() - sim::core::Duration(1)) / lfintosc_.interval()));
        }
      } else {
        if (lfintosc_.delta().count() * lfintosc_.interval() >= TDEW) {
          nv_->eedata()[eeadr_reg_.read() % nv_->eedata().size()] = eedat_reg_.read();
          sim::core::trace_writer().emplace<WroteEEDATATraceEntry>(eeadr_reg_.read() % nv_->eedata().size());
          eecon1_reg_.set_wr(false);
          interrupt_.raise();
        } else {
          next_time = lfintosc_.at(sim::core::Clock::duration((TDEW + lfintosc_.interval() - sim::core::Duration(1)) / lfintosc_.interval()));
        }
      }
    }

    return { .next_time = next_time };
  }

  template<int PgmDatBufSize, const std::array<uint16_t, 4> SelfWriteCutoffs>
  void EPROM<PgmDatBufSize, SelfWriteCutoffs>::push_pgmdat(uint16_t v) {
    pgmdat_buf_[pgmdat_buf_head_] = v;
    if (++pgmdat_buf_head_ == pgmdat_buf_.size())
      pgmdat_buf_head_ = 0;
  }

  template class EPROM<4, self_write_cutoffs<2048>()>;
  template class EPROM<4, self_write_cutoffs<4096>()>;
  template class EPROM<8, self_write_cutoffs<8192>()>;

}  // namespace sim::pic14::internal
