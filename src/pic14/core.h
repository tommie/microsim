#ifndef sim_pic14_core_h
#define sim_pic14_core_h

#include "data_bus.h"
#include "register.h"

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
  class Core : public RegisterBackend {
  public:
    using RegisterType = uint8_t;
    using RegisterAddressType = uint16_t;
    using OptionReg = OptionRegBase<MultiRegisterBackend<Core, 0x01>>;

    Core() : option_reg_(SingleRegisterBackend<uint8_t>(0xFF)) {}

    void reset() {
      option_reg_.reset();
    }

    OptionReg option_reg() { return OptionReg(MultiRegisterBackend<Core, 0x01>(this)); }

    uint8_t read_register(uint16_t addr) override {
      switch (addr) {
      case 0x01: return option_reg_.read();
      default: return 0;
      }
    }

    void write_register(uint16_t addr, uint8_t value) override {
      switch (addr) {
      case 0x01: option_reg_.write(value); break;
      }
    }

  private:
    OptionRegBase<SingleRegisterBackend<uint8_t>> option_reg_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_core_h
