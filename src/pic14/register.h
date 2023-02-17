#ifndef sim_pic14_register_h
#define sim_pic14_register_h

#include <cstdint>
#include <string>
#include <vector>

namespace sim::pic14::internal {

  /// A backend for a simple BitRegister using a trivial type. This
  /// can be useful as the underlying implementation of a
  /// `RegisterBackend` even if external access is handled through
  /// `MultiRegisterBackend`.
  template<typename Type>
  class SingleRegisterBackend {
  public:
    using RegisterType = Type;

    explicit SingleRegisterBackend(Type initial)
      : value_(initial) {}

    Type read() const { return value_; }
    void write(Type v) { value_ = v; }

  private:
    Type value_;
  };

  /// A backend for a BitRegister that updates a
  /// `RegisterBackend`. Use this for registers that may trigger
  /// changes in other modules, e.g. interrupts.
  template<typename Backend, typename Backend::RegisterAddressType Addr>
  class MultiRegisterBackend {
  public:
    using RegisterType = typename Backend::RegisterType;

    explicit MultiRegisterBackend(Backend *backend)
      : backend_(backend) {}

    RegisterType read() { return backend_->read_register(Addr); }
    RegisterType read() const { return backend_->read_register(Addr); }
    void write(const RegisterType &v) { backend_->write_register(Addr, v); }

  private:
    Backend *backend_;
  };

  /// A register accessor and manipulation facade. This is intended as
  /// a base class for concrete register.
  template<typename Backend>
  class BitRegister {
    using value_type = typename Backend::RegisterType;

  public:
    value_type read() { return backend_.read(); }
    value_type read() const { return backend_.read(); }
    void write(value_type v) { backend_.write(v); }

    bool all_flags_set(uint8_t mask) { return (read() & mask) == mask; }
    bool all_flags_set(uint8_t mask) const { return (read() & mask) == mask; }
    bool any_flag_set(uint8_t mask) { return (read() & mask) != 0; }
    bool any_flag_set(uint8_t mask) const { return (read() & mask) != 0; }

    void set_flags(uint8_t mask) { write(read() | mask); }

  protected:
    explicit BitRegister(Backend backend)
      : backend_(backend) {}

    template<int Lowest, int Size> value_type bit_field() { return (read() >> Lowest) & ((1u << Size) - 1); }
    template<int Lowest, int Size> value_type bit_field() const { return (read() >> Lowest) & ((1u << Size) - 1); }

    template<int Lowest, int Size>
    void clear_bitfield() { write(read() & ~(((1u << Size) - 1) << Lowest)); }

    template<int Bit> bool bit() { return (read() & (1u << Bit)) != 0; }
    template<int Bit> bool bit() const { return (read() & (1u << Bit)) != 0; }

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

  protected:
    Backend backend_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_register_h
