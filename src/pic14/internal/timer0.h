#ifndef sim_pic14_internal_timer0_h
#define sim_pic14_internal_timer0_h

#include <functional>

#include "../../core/clock.h"
#include "../../core/scheduler.h"
#include "data_bus.h"
#include "pin.h"
#include "port.h"
#include "register.h"

namespace sim::pic14::internal {

  class Timer0 : public sim::core::Schedulable, public RegisterBackend {
  public:
    static constexpr sim::core::Clock::duration TICKS_PER_COUNT = sim::core::Clock::duration(4);

    enum class Register : uint16_t {
      TMR0,
    };

    Timer0(sim::core::ClockModifier *fosc, InterruptMux::MaskableIntconEdgeSignal &&interrupt, Core::OptionReg &&option_reg);

    const sim::core::Pin& pin_t0cki() const { return pin_t0cki_; }
    sim::core::Pin& pin_t0cki() { return pin_t0cki_; }

    void reset();

    sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit) override;

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    /// Notifies the module that fosc has changed.
    void fosc_changed() { schedule_immediately(); }

    /// Notifies the module that OPTION_REG has changed.
    void option_reg_updated() { schedule_immediately(); }

  private:
    void t0cki_changed(bool rising);

    int prescaled() const;

  private:
    sim::core::ClockModifierView fosc_;
    InterruptMux::MaskableIntconEdgeSignal interrupt_;
    Core::OptionReg option_reg_;
    InputPin pin_t0cki_;
    uint8_t tmr0_reg_ = 0x7F;  // TMR0 is actually uninitialized at POR.
    int prescaler_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_internal_timer0_h
