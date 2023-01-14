#ifndef sim_pic14_register_h
#define sim_pic14_register_h

#include <cstdint>
#include <string>
#include <vector>

#include "data_bus.h"

namespace sim::pic14::internal {

  /// A register accessor and manipulation facade. This is intended as
  /// a base class for concrete register.
  template<uint16_t Addr>
  class BitRegister {
  protected:
    static const uint16_t ADDR = Addr;

    explicit BitRegister(DataBus *bus) : bus_(bus) {}

    uint8_t const_read() const { return bus_->const_read_register(ADDR); }
    uint8_t read() { return bus_->read_register(ADDR); }
    void write(uint8_t v) { bus_->write_register(ADDR, v); }

    template<uint8_t Lowest, int Size>
    uint8_t const_bit_field() const { return (const_read() >> Lowest) & ((1u << Size) - 1); }

    template<uint8_t Lowest, int Size>
    uint8_t bit_field() { return (read() >> Lowest) & ((1u << Size) - 1); }

    template<uint8_t Bit>
    bool const_bit() const { return (const_read() & (1u << Bit)) != 0; }

    template<uint8_t Bit>
    bool bit() { return (read() & (1u << Bit)) != 0; }

    template<uint8_t Bit>
    void set_bit(bool v) {
      uint8_t f = read();
      if (v) f |= 1u << Bit;
      else f &= ~(1u << Bit);
      write(f);
    }

    template<uint8_t Mask>
    void set_mask() { write(read() | Mask); }

    template<uint8_t Mask>
    void clear_mask() { write(read() & ~Mask); }

    template<uint8_t Mask>
    void toggle_mask() { write(read() ^ Mask); }

    template<uint8_t Set, uint8_t Clear, uint8_t Toggle>
    void update_masks() { write(((read() & ~Clear) | Set) ^ Toggle); }

    template<uint8_t Mask>
    void set_masked(uint8_t v) { write((read() & ~Mask) | v); }

  private:
    DataBus *bus_;
  };

  class OptionReg : public BitRegister<0x01> {
  public:
    enum Bits {
      PS0, PS1, PS2, PSA, T0SE, T0CS, INTEDG, RBPU,
    };

    explicit OptionReg(DataBus *bus) : BitRegister(bus) {}

    bool rbpu() const { return const_bit<RBPU>(); }
    bool intedg() const { return const_bit<INTEDG>(); }
    bool t0cs() const { return const_bit<T0CS>(); }
    bool t0se() const { return const_bit<T0SE>(); }
    bool psa() const { return const_bit<PSA>(); }
    bool ps() const { return const_bit_field<PS0, 3>(); }

    void reset() { write(0xFF); }
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_register_h
