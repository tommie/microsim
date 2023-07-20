#ifndef sim_pic14_interrupt_h
#define sim_pic14_interrupt_h

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "data_bus.h"
#include "register.h"

namespace sim::pic14::internal {

  template<typename Backend>
  class IntConRegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

  public:
    enum Bits {
      RBIF, INTF, T0IF, RBIE, INTE, T0IE, PEIE, GIE,
    };

    explicit IntConRegBase(Backend backend) : BitRegister<Backend>(backend) {}

    bool rbif() const { return Base::template bit<RBIF>(); }
    bool intf() const { return Base::template bit<INTF>(); }
    bool t0if() const { return Base::template bit<T0IF>(); }
    bool rbie() const { return Base::template bit<RBIE>(); }
    bool inte() const { return Base::template bit<INTE>(); }
    bool t0ie() const { return Base::template bit<T0IE>(); }
    bool peie() const { return Base::template bit<PEIE>(); }

    bool gie() const { return Base::template bit<GIE>(); }
    void set_gie(bool v) { Base::template set_bit<GIE>(v); }

    void reset() { Base::template clear_bitfield<1, 7>(); }
  };

  /// An interrupt multiplexer that is active if any signal is
  /// raised.
  class InterruptMux : public RegisterBackend {
    using IntConRegImpl = IntConRegBase<SingleRegisterBackend<uint8_t>>;

  public:
    enum class Register : uint16_t {
      INTCON,
      PIR1,
      PIR2,
      PIE1,
      PIE2,
    };

    using RegisterType = uint8_t;
    using RegisterAddressType = uint16_t;
    using InterruptSignal = std::function<void()>;
    using IntConReg = IntConRegBase<MultiRegisterBackend<InterruptMux, Register::INTCON>>;

    class MaskableIntconEdgeSignal {
    public:
      MaskableIntconEdgeSignal(InterruptMux *mux, uint8_t flag_mask)
        : mux_(mux), flag_mask_(flag_mask) {}

      void reset() { active_ = false; }
      void raise();

      MaskableIntconEdgeSignal(MaskableIntconEdgeSignal&&) = default;
      MaskableIntconEdgeSignal& operator =(MaskableIntconEdgeSignal&&) = default;
      MaskableIntconEdgeSignal(const MaskableIntconEdgeSignal&) = delete;
      MaskableIntconEdgeSignal& operator =(const MaskableIntconEdgeSignal&) = delete;

    private:
      InterruptMux *mux_;
      uint8_t flag_mask_;
      bool active_ = false;
    };

    class MaskablePeripheralEdgeSignal {
    public:
      MaskablePeripheralEdgeSignal(InterruptMux *mux, int reg_index, uint8_t flag_mask)
        : mux_(mux), reg_index_(reg_index), flag_mask_(flag_mask) {}

      void reset() { active_ = false; }
      void raise();

      MaskablePeripheralEdgeSignal(MaskablePeripheralEdgeSignal&&) = default;
      MaskablePeripheralEdgeSignal& operator =(MaskablePeripheralEdgeSignal&&) = default;
      MaskablePeripheralEdgeSignal(const MaskablePeripheralEdgeSignal&) = delete;
      MaskablePeripheralEdgeSignal& operator =(const MaskablePeripheralEdgeSignal&) = delete;

    private:
      InterruptMux *mux_;
      int reg_index_;
      uint8_t flag_mask_;
      bool active_ = false;
    };

    class MaskablePeripheralLevelSignal {
    public:
      MaskablePeripheralLevelSignal(InterruptMux *mux, int reg_index, uint8_t flag_mask)
        : mux_(mux), reg_index_(reg_index), flag_mask_(flag_mask) {}

      void reset();
      void raise();

      MaskablePeripheralLevelSignal(MaskablePeripheralLevelSignal&&) = default;
      MaskablePeripheralLevelSignal& operator =(MaskablePeripheralLevelSignal&&) = default;
      MaskablePeripheralLevelSignal(const MaskablePeripheralLevelSignal&) = delete;
      MaskablePeripheralLevelSignal& operator =(const MaskablePeripheralLevelSignal&) = delete;

    private:
      InterruptMux *mux_;
      int reg_index_;
      uint8_t flag_mask_;
      bool active_ = false;
    };

    explicit InterruptMux(InterruptSignal interrupt)
      : interrupt_(std::move(interrupt)),
        intcon_(SingleRegisterBackend<uint8_t>(0)) {}

    /// Creates an interrupt signal masked by the INTCON `en_bit`,
    /// using `flag_bit` as the flag. Calling `raise()` multiple times
    /// does nothing to the flag until `reset()` is called.
    MaskableIntconEdgeSignal make_maskable_edge_signal_intcon(uint8_t en_bit, uint8_t flag_bit);

    /// Creates an interrupt signal masked by the PIEx and using PIRx
    /// as the flag. Calling `raise()` multiple times does nothing to
    /// the flag until `reset()` is called.
    MaskablePeripheralEdgeSignal make_maskable_edge_signal_peripheral(uint8_t bit);

    /// Creates an interrupt signal masked by the PIEx and using PIRx
    /// as the flag. Calling `raise()` multiple times does nothing to
    /// the flag until `reset()` is called. The flag cannot be cleared
    /// by software.
    MaskablePeripheralLevelSignal make_maskable_level_signal_peripheral(uint8_t bit);

    /// Returns whether any interrupt signal is raised. It does not
    /// check the GIE bit.
    bool is_active() const;

    /// Performs a device reset on the interrupt mux.
    void reset();

    IntConReg intcon_reg() { return IntConReg(MultiRegisterBackend<InterruptMux, Register::INTCON>(this)); }

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    InterruptMux(InterruptMux&&) = delete;
    InterruptMux& operator=(InterruptMux&&) = delete;
    InterruptMux(const InterruptMux&) = delete;
    InterruptMux& operator=(const InterruptMux&) = delete;

  private:
    InterruptSignal interrupt_;

    IntConRegImpl intcon_;
    std::array<uint8_t, 2> pie_ = {0, 0};
    std::array<uint8_t, 2> pir_ = {0, 0};
    std::array<uint8_t, 2> pir_ro_mask_ = {0, 0};

    std::array<uint8_t, 8> intcon_en_bits_ = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_interrupt_h
