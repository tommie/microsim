#ifndef sim_pic14_eprom_h
#define sim_pic14_eprom_h

#include <array>
#include <cstdint>
#include <optional>

#include "../core/clock.h"
#include "../core/scheduler.h"
#include "../core/trace.h"
#include "core.h"
#include "data_bus.h"
#include "execution.h"
#include "interrupt.h"
#include "nonvolatile.h"
#include "register.h"

namespace sim::pic14::internal {

  template<typename Backend>
  class EECon1RegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

    static constexpr uint8_t WRITE_MASK = 0x8F;

  public:
    enum Bits {
      RD, WR, WREN, WRERR, EEPGD = 7,
    };

    explicit EECon1RegBase(Backend backend) : BitRegister<Backend>(backend) {}

    void write_masked(uint8_t v) { Base::template set_masked<WRITE_MASK>(v); }

    bool rd() const { return Base::template bit<RD>(); }
    void set_rd(bool v) { Base::template set_bit<RD>(v); }
    bool wr() const { return Base::template bit<WR>(); }
    void set_wr(bool v) { Base::template set_bit<WR>(v); }
    bool wren() const { return Base::template bit<WREN>(); }
    bool wrerr() const { return Base::template bit<WRERR>(); }
    bool eepgd() const { return Base::template bit<EEPGD>(); }

    void reset(bool wrerr) { Base::write(wrerr ? (1u << WRERR) : 0); }
  };

  template<uint16_t ProgSize>
  constexpr const std::array<uint16_t, 4> self_write_cutoffs() {
    if (ProgSize <= 2048) return {0x0400, 0x0100, 0xFFFF, 0};
    else if (ProgSize <= 4096) return {0x0800, 0x0400, 0x0100, 0};
    else return {0x1000, 0x0800, 0x0100, 0};
  }

  /// The data EEPROM and Flash ROM read/write sequencer.
  template<int PgmDatBufSize, const std::array<uint16_t, 4> SelfWriteCutoffs>
  class EPROM : public RegisterBackend, public sim::core::Schedulable {
    // The actual clock EPROM uses is unknown. Using LFINTOSC because
    // of its constant frequency. The datasheet specifies wall-clock
    // times.
    static constexpr sim::core::Duration TPEW = sim::core::Milliseconds(2);
    static constexpr sim::core::Duration TDEW = sim::core::Milliseconds(5);
    static constexpr uint16_t EECON2_MAGIC = 0x55AA;

  public:
    using RegisterType = uint8_t;
    using RegisterAddressType = uint16_t;
    using EEDatReg = MultiRegisterBackend<EPROM, 0x10C>;
    using EEAdrReg = MultiRegisterBackend<EPROM, 0x10D>;
    using EEDatHReg = MultiRegisterBackend<EPROM, 0x10E>;
    using EEAdrHReg = MultiRegisterBackend<EPROM, 0x10F>;
    using EECon1Reg = EECon1RegBase<MultiRegisterBackend<EPROM, 0x18C>>;
    using EECon2Reg = MultiRegisterBackend<EPROM, 0x18D>;

    EPROM(NonVolatile *nv, sim::core::Clock *lfintosc, const Core::Config2Reg *config2, InterruptMux::MaskablePeripheralEdgeSignal &&interrupt, Executor *exec);

    void reset();

    EEDatReg eedat_reg() { return EEDatReg(this); }
    EEDatHReg eedath_reg() { return EEDatHReg(this); }
    EEAdrReg eeadr_reg() { return EEAdrReg(this); }
    EEAdrHReg eeadrh_reg() { return EEAdrHReg(this); }
    EECon1Reg eecon1_reg() { return EECon1Reg(MultiRegisterBackend<EPROM, 0x18C>(this)); }
    EECon2Reg eecon2_reg() { return EECon2Reg(this); }

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit) override;

  private:
    void push_pgmdat(uint16_t v);

  private:
    NonVolatile *nv_;
    sim::core::ClockView lfintosc_;
    const Core::Config2Reg *config2_;
    InterruptMux::MaskablePeripheralEdgeSignal interrupt_;
    Executor *exec_;

    SingleRegisterBackend<uint8_t> eedat_reg_;
    SingleRegisterBackend<uint8_t> eedath_reg_;
    SingleRegisterBackend<uint8_t> eeadr_reg_;
    SingleRegisterBackend<uint8_t> eeadrh_reg_;
    EECon1RegBase<SingleRegisterBackend<uint8_t>> eecon1_reg_;
    uint16_t eecon2_reg_ = 0;
    std::array<uint16_t, PgmDatBufSize> pgmdat_buf_;
    unsigned int pgmdat_buf_head_ = 0;
    std::optional<Executor::Inhibitor> inhibitor_;
  };

}  // namespace sim::pic14::internal

namespace sim::pic14 {

  class WroteEEDATATraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<WroteEEDATATraceEntry> TYPE;

    explicit WroteEEDATATraceEntry(uint8_t addr)
      : addr_(addr) {}

    uint8_t addr() const { return addr_; }

  private:
    uint8_t addr_;
  };

  class WroteProgramFlashTraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<WroteProgramFlashTraceEntry> TYPE;

    explicit WroteProgramFlashTraceEntry(uint16_t addr)
      : addr_(addr) {}

    uint16_t addr() const { return addr_; }

  private:
    uint16_t addr_;
  };

}  // namespace sim::pic14

#endif  // sim_pic14_eprom_h
