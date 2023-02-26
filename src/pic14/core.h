#ifndef sim_pic14_core_h
#define sim_pic14_core_h

#include <cstdint>
#include <functional>
#include <vector>

#include "../core/clock.h"
#include "../core/scheduler.h"
#include "../core/signal.h"
#include "../util/status.h"
#include "data_bus.h"
#include "nonvolatile.h"
#include "pin.h"
#include "register.h"

namespace sim::pic14 {

  /// A handle of an on-going ICSP.
  ///
  /// When destroyed, it informs the device it should reset itself.
  ///
  /// Instances can be moved, but not copied.
  class ICSP {
  public:
    ICSP(internal::NonVolatile *device, sim::core::Signal<bool> *reset)
      : device_(device), reset_(reset) {
      reset_->set(true);
    }

    ~ICSP() {
      reset_->set(false);
    }

    /// Programs memory at the specified address. For 14-bit data
    /// (program memory, configuration words), little-endian is used
    /// when decoding. This can also program EEPROM data presented as
    /// 16-bit values where the top byte is discarded.
    sim::util::Status load_program(uint16_t addr, std::u8string_view data);

    /// Program data at the specified address. Data is represented as
    /// 8-bit values.
    sim::util::Status load_data(uint16_t addr, std::u8string_view data);

    ICSP(ICSP&&) = default;
    ICSP(const ICSP&) = delete;
    ICSP& operator=(const ICSP&) = delete;

  private:
    internal::NonVolatile *device_;
    sim::core::Signal<bool> *reset_;
  };

}  // namespace sim::pic14

namespace sim::pic14::internal {

  template<typename Backend>
  class OptionRegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

  public:
    enum Bits {
      PS0, PS1, PS2, PSA, T0SE, T0CS, INTEDG, RBPU,
    };

    explicit OptionRegBase(Backend backend) : BitRegister<Backend>(backend) {}

    bool rbpu() const { return Base::template bit<RBPU>(); }
    bool intedg() const { return Base::template bit<INTEDG>(); }
    bool t0cs() const { return Base::template bit<T0CS>(); }
    bool t0se() const { return Base::template bit<T0SE>(); }
    bool psa() const { return Base::template bit<PSA>(); }
    bool ps() const { return Base::template bit_field<PS0, 3>(); }

    void reset() { Base::write(0xFF); }
  };

  template<typename Backend>
  class OscConRegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

    static constexpr uint8_t WRITE_MASK = 0x71;

  public:
    enum Bits {
      SCS, LTS, HTS, OSTS, IRCF0, IRCF1, IRCF2,
    };

    explicit OscConRegBase(Backend backend) : BitRegister<Backend>(backend) {}

    void write_masked(uint8_t v) { Base::template set_masked<WRITE_MASK>(v); }

    bool scs() const { return Base::template bit<SCS>(); }
    bool lts() const { return Base::template bit<LTS>(); }
    bool hts() const { return Base::template bit<HTS>(); }
    bool osts() const { return Base::template bit<OSTS>(); }
    uint8_t ircf() const { return Base::template bit_field<IRCF0, 3>(); }

    void reset() { Base::write(0x68 | 0x06); }
  };

  template<typename Backend>
  class Config1RegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

  public:
    static constexpr uint16_t ADDR = 7;

    enum Bits {
      FOSC0, FOSC1, FOSC2, WDTE, PWRTE, MCLRE, CP, CPD,
      BOREN0, BOREN1, IESO, FCMEN, LVP, DEBUG,
    };

    explicit Config1RegBase(Backend backend) : BitRegister<Backend>(backend) {}

    uint8_t fosc() const { return Base::template bit_field<FOSC0, 3>(); }
    bool wdte() const { return Base::template bit<WDTE>(); }
    bool pwrte() const { return !Base::template bit<PWRTE>(); }
    bool mclre() const { return Base::template bit<MCLRE>(); }
    bool cp() const { return !Base::template bit<CP>(); }
    bool cpd() const { return !Base::template bit<CPD>(); }
    uint8_t boren() const { return Base::template bit_field<BOREN0, 2>(); }
    bool ieso() const { return Base::template bit<IESO>(); }
    bool fcmen() const { return Base::template bit<FCMEN>(); }
    bool lvp() const { return Base::template bit<LVP>(); }
    bool debug() const { return !Base::template bit<DEBUG>(); }
  };

  /// Core parts that are shared by multiple other modules.
  class Core : public RegisterBackend, public sim::core::Schedulable {
  public:
    using RegisterType = uint8_t;
    using RegisterAddressType = uint16_t;
    using OptionReg = OptionRegBase<MultiRegisterBackend<Core, 0x81>>;
    using OscConReg = OscConRegBase<MultiRegisterBackend<Core, 0x8F>>;

    Core(sim::core::Clock *extosc, NonVolatile *nv, std::function<void(bool)> reset, std::function<void()> option_updated, std::function<void()> fosc_changed);

    void reset();

    OptionReg option_reg() { return OptionReg(MultiRegisterBackend<Core, 0x81>(this)); }

    sim::core::ClockModifier* fosc() { return &fosc_; }
    std::vector<sim::core::Clock*> clock_sources() { return { &hfintosc_, &lfintosc_ }; }

    bool in_reset() const { return reset_.value(); }
    InputPin* mclr_pin() { return &mclr_pin_; }

    /// Enters ICSP mode, allowing programming the non-volatile
    /// memory. When the returned ICSP object is destroyed, the device
    /// leaves ICSP mode.
    ICSP enter_icsp();

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit) override;

  private:
    void update_system_clock();
    std::pair<sim::core::Clock*, int> system_clock();

  private:
    sim::core::Clock *extosc_;
    NonVolatile *nv_;
    sim::core::CombinedSignal<sim::core::CombineOr<bool>> reset_;
    std::function<void()> option_updated_;

    sim::core::Signal<bool> *mclr_;
    InputPin mclr_pin_;
    sim::core::Signal<bool> *por_;
    sim::core::Signal<bool> *icsp_reset_;

    sim::core::Clock hfintosc_;
    sim::core::Clock lfintosc_;
    sim::core::ClockModifier fosc_;

    OptionRegBase<SingleRegisterBackend<uint8_t>> option_reg_;
    OscConRegBase<SingleRegisterBackend<uint8_t>> osccon_reg_;
    Config1RegBase<SingleRegisterBackend<uint16_t>> config1_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_core_h
