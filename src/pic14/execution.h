#ifndef sim_pic14_execution_h
#define sim_pic14_execution_h

#include "../core/clock.h"
#include "../core/device.h"
#include "../core/scheduler.h"
#include "data_bus.h"
#include "interrupt.h"
#include "nonvolatile.h"
#include "register.h"

#include <array>
#include <cstdint>
#include <string_view>

namespace sim::pic14::internal {

  template<typename Backend>
  class StatusRegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

  public:
    enum Bits {
      C, DC, Z, PD, TO, RP0, RP1, IRP,
    };

    explicit StatusRegBase(Backend backend) : BitRegister<Backend>(backend) {}

    uint16_t rp() const { return static_cast<uint16_t>(Base::template bit_field<RP0, 2>()) << 7; }

    bool c() const { return Base::template bit<C>(); }
    void set_c(bool v) { Base::template set_bit<C>(v); }

    void set_reset(uint8_t inv_v) {
      Base::template set_masked<(1 << PD) | (1 << TO)>(~inv_v);
    }

    void reset() { Base::template clear_bitfield<RP0, 3>(); }
  };

  class Executor : public sim::core::Schedulable, public RegisterBackend {
    static const int STACK_SIZE = 8;
    static const sim::core::Ticks TICKS_PER_INSN = 4;

    enum StackContext {
      INTERRUPT = 1,
      CALL,
      RETFIE,
      RETURN,
      RETLW,
    };

    class StatusRegImpl : public StatusRegBase<SingleRegisterBackend<uint8_t>> {
    public:
      using StatusRegBase<SingleRegisterBackend<uint8_t>>::StatusRegBase;

      void update_add(uint8_t a, uint8_t b);
      void update_sub(uint8_t a, uint8_t b);
      void update_logic(uint8_t v);
    };

  public:
    using RegisterType = uint8_t;
    using RegisterAddressType = uint16_t;
    using StatusReg = StatusRegBase<MultiRegisterBackend<Executor, 0x03>>;

    /// Constructs a new Executor with the given configuration for
    /// non-volatile memory, and data bus.
    Executor(sim::core::DeviceListener *listener, sim::core::Clock *fosc, NonVolatile *nv, DataBus &&data_bus, InterruptMux *interrupt_mux);

    /// Executes the next instruction and returns the number of ticks
    /// it took.
    sim::core::Ticks execute();

    /// Resets the execution unit, including its register values.
    void reset();

    /// Returns whether the execution unit is sleeping. It can only be
    /// awaken by interrupts, or a reset.
    bool is_sleeping() const { return in_sleep; }

    const DataBus& data_bus() const { return data_bus_; }
    DataBus& data_bus() { return data_bus_; }

    StatusReg status_reg() { return StatusReg(MultiRegisterBackend<Executor, 0x03>(this)); }

    /// Informs the executor of an active interrupt condition.
    void interrupted();

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

  protected:
    /// Implements Schedulable.
    sim::core::Advancement advance_to(const sim::core::SimulationLimit &limit) override;

  private:
    uint8_t get_register(uint16_t addr) { return data_bus_.read_register(addr); }
    void set_register(uint16_t addr, uint8_t v) { data_bus_.write_register(addr, v); }
    uint16_t get_pc() const { return ((uint16_t) data_bus_.const_read_register(0x0A) << 8) | data_bus_.const_read_register(0x02); }
    void set_pc(uint16_t v) { v &= nv_->progmem().size() - 1; set_register(0x0A, v >> 8); set_register(0x02, v & 0xFF); }
    uint8_t get_w() const { return w_reg_; }
    void set_w(uint8_t v) { w_reg_ = v; }
    uint16_t pop_stack(StackContext context);
    void push_stack(uint16_t pc, StackContext context);

    uint16_t banked_data_addr(uint8_t addr) const { return status_reg_.rp() | addr; }

    static std::string_view stack_context_text(StackContext context);

  private:
    sim::core::DeviceListener *listener_;
    sim::core::Clock *fosc_;
    NonVolatile *nv_;
    DataBus data_bus_;
    InterruptMux *interrupt_mux_;

    std::array<uint16_t, STACK_SIZE> stack;
    uint8_t sp_reg_;
    uint8_t w_reg_;
    StatusRegImpl status_reg_;

    bool in_sleep;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_execution_h
