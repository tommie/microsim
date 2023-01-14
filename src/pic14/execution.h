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

  class StatusReg : public BitRegister<0x03> {
  public:
    enum Bits {
      C, DC, Z, PD, TO, RP0, RP1, IRP,
    };

    explicit StatusReg(DataBus *bus) : BitRegister(bus) {}

    uint16_t rp() const { return static_cast<uint16_t>(const_bit_field<RP0, 2>()) << 7; }

    bool c() const { return const_bit<C>(); }
    void set_c(bool v) { set_bit<C>(v); }

    /// Updates the status register for a reset. Only PD and TO bits
    /// in `inv_v` are preserved, and they are first inverted.
    void reset(uint8_t inv_v);

    void update_reset(uint8_t inv_v);
    void update_add(uint8_t a, uint8_t b);
    void update_sub(uint8_t a, uint8_t b);
    void update_logic(uint8_t v);
  };

  class Execution : public sim::core::Device, public NonVolatile, public Interruption {
    friend class Executor;

    static const int STACK_SIZE = 8;

    enum StackContext {
      INTERRUPT = 1,
      CALL,
      RETFIE,
      RETURN,
      RETLW,
    };

  public:
    /// Constructs a new Execution with the given configuration for
    /// non-volatile memory, and data bus.
    Execution(sim::core::DeviceListener *listener, sim::core::Clock *clock, const NonVolatile::Config &nv_config, DataBus &&data_bus);

  protected:
    sim::core::Advancement execute_to(const sim::core::SimulationLimit &limit);

    /// Executes the next instruction and returns the number of ticks
    /// it took.
    sim::core::Ticks execute();

    /// Resets the execution unit, including register values. `status`
    /// is a mask of TO and PD bits to set.
    void reset(uint8_t status);

    /// Returns whether the execution unit is sleeping. It can only be
    /// awaken by interrupts, or a reset.
    bool is_sleeping() const { return in_sleep; }

    const DataBus& data_bus() const { return data_bus_; }
    DataBus& data_bus() { return data_bus_; }

    const StatusReg status_reg() const { return StatusReg(const_cast<DataBus*>(&data_bus_)); }
    StatusReg status_reg() { return StatusReg(&data_bus_); }

    const IntConReg intcon_reg() const { return IntConReg(const_cast<DataBus*>(&data_bus_)); }
    IntConReg intcon_reg() { return IntConReg(&data_bus_); }

    /// Called by NonVolatile when leaving ICSP mode.
    void icsp_reset() override { reset(0); }

  private:
    uint8_t get_register(uint16_t addr) { return data_bus_.read_register(addr); }
    void set_register(uint16_t addr, uint8_t v) { data_bus_.write_register(addr, v); }
    uint16_t get_pc() const { return ((uint16_t) data_bus_.const_read_register(0x0A) << 8) | data_bus_.const_read_register(0x02); }
    void set_pc(uint16_t v) { v &= progmem.size() - 1; set_register(0x0A, v >> 8); set_register(0x02, v & 0xFF); }
    uint8_t get_w() const { return w_reg_; }
    void set_w(uint8_t v) { w_reg_ = v; }
    uint16_t pop_stack(StackContext context);
    void push_stack(uint16_t pc, StackContext context);

    uint16_t banked_data_addr(uint8_t addr) const { return status_reg().rp() | addr; }

    static std::string_view stack_context_text(StackContext context);

  private:
    sim::core::Clock *clock_;
    DataBus data_bus_;

    std::array<uint16_t, STACK_SIZE> stack;
    uint8_t sp_reg_;
    uint8_t w_reg_;

    bool in_sleep;
  };

  class Executor : public sim::core::Schedulable {
  public:
    explicit Executor(Execution *ex) : execution_(ex) {}

    sim::core::Advancement advance_to(const sim::core::SimulationLimit &limit) override;

  private:
    Execution *execution_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_execution_h
