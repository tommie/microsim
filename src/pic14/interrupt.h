#ifndef sim_pic14_interrupt_h
#define sim_pic14_interrupt_h

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "data_bus.h"
#include "register.h"

namespace sim::pic14::internal {

  class IntConReg : protected BitRegister<0x0B> {
  public:
    explicit IntConReg(DataBus *bus) : BitRegister(bus) {}

    bool gie() { return bit<7>(); }
    void set_gie(bool v) { set_bit<7>(v); }

    void reset() { write(0); }
  };

  /// An interrupt multiplexer that is active if any signal is
  /// raised.
  class InterruptMux : public RegisterBackend {
    static const int GIEBit = 7;

  public:
    using InterruptSignal = std::function<void()>;

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

    explicit InterruptMux(InterruptSignal interrupt) : interrupt_(std::move(interrupt)) {}

    /// Creates an interrupt signal masked by the INTCON `en_bit`,
    /// using `flag_bit` as the flag. Calling `raise()` multiple times
    /// does nothing to the flag until `reset()` is called.
    MaskableIntconEdgeSignal make_maskable_edge_signal_intcon(uint8_t en_bit, uint8_t flag_bit);

    /// Returns whether any interrupt signal is raised. It does not
    /// check the GIE bit.
    bool is_active() const;

    /// Performs a device reset on the interrupt mux.
    void reset();

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    InterruptMux(InterruptMux&&) = delete;
    InterruptMux& operator=(InterruptMux&&) = delete;
    InterruptMux(const InterruptMux&) = delete;
    InterruptMux& operator=(const InterruptMux&) = delete;

  private:
    InterruptSignal interrupt_;

    uint8_t intcon_ = 0;
    std::array<uint8_t, 2> pie_ = {0, 0};
    std::array<uint8_t, 2> pir_ = {0, 0};

    std::array<uint8_t, 8> intcon_en_bits_ = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_interrupt_h
