#ifndef sim_pic14_core_h
#define sim_pic14_core_h

#include <cstdint>
#include <functional>

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

  /// Core parts that are shared by multiple other modules.
  class Core : public RegisterBackend, public sim::core::Schedulable {
  public:
    using RegisterType = uint8_t;
    using RegisterAddressType = uint16_t;
    using OptionReg = OptionRegBase<MultiRegisterBackend<Core, 0x81>>;

    Core(sim::core::Clock *extosc, NonVolatile *nv, std::function<void(bool)> reset, std::function<void()> option_updated);

    void reset() {
      option_reg_.reset();
    }

    OptionReg option_reg() { return OptionReg(MultiRegisterBackend<Core, 0x81>(this)); }

    sim::core::Clock* fosc() { return extosc_; }

    bool in_reset() const { return reset_.value(); }
    InputPin* mclr_pin() { return &mclr_pin_; }

    /// Enters ICSP mode, allowing programming the non-volatile
    /// memory. When the returned ICSP object is destroyed, the device
    /// leaves ICSP mode.
    ICSP enter_icsp();

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    sim::core::Advancement advance_to(const sim::core::SimulationLimit &limit) override;

  private:
    sim::core::Clock *extosc_;
    NonVolatile *nv_;
    sim::core::CombinedSignal<sim::core::CombineOr<bool>> reset_;
    sim::core::Signal<bool> *mclr_;
    InputPin mclr_pin_;
    sim::core::Signal<bool> *por_;
    sim::core::Signal<bool> *icsp_reset_;

    std::function<void()> option_updated_;
    OptionRegBase<SingleRegisterBackend<uint8_t>> option_reg_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_core_h
