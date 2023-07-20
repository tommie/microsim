#ifndef sim_pic14_watchdog_h
#define sim_pic14_watchdog_h

#include "../core/clock.h"
#include "../core/scheduler.h"
#include "../core/signal.h"
#include "../core/trace.h"
#include "core.h"
#include "data_bus.h"
#include "execution.h"
#include "register.h"

namespace sim::pic14 {

  namespace internal {

  template<typename Backend>
  class WDTConRegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

    static constexpr uint8_t WRITE_MASK = 0x1F;

  public:
    enum Bits {
      SWDTEN, WDTPS0, WDTPS1, WDTPS2, WDTPS3,
    };

    explicit WDTConRegBase(Backend backend) : BitRegister<Backend>(backend) {}

    void write_masked(uint8_t v) { Base::template set_masked<WRITE_MASK>(v); }

    bool swdten() const { return Base::template bit<SWDTEN>(); }
    uint8_t wdtps() const { return Base::template bit_field<WDTPS0, 4>(); }

    void reset() { Base::write(0x08); }
  };

  class WatchDogTimer : public sim::core::Schedulable, public RegisterBackend {
  public:
    enum class Register : uint16_t {
      WDTCON,
    };

    WatchDogTimer(sim::core::Clock *lfintosc, sim::core::Signal<bool> *reset, Core::OptionReg &&option_reg, const Core::Config1Reg *config1, Executor::StatusReg &&status_reg);

    void reset();

    void clear();

    sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit) override;

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    /// Notifies the module that OPTION_REG has changed.
    void option_reg_updated() { schedule_immediately(); }

  private:
    int prescaled() const;

  private:
    sim::core::ClockView lfintosc_;
    sim::core::Signal<bool> *reset_;
    Core::OptionReg option_reg_;
    const Core::Config1Reg *config1_;
    Executor::StatusReg status_reg_;

    WDTConRegBase<SingleRegisterBackend<uint8_t>> wdtcon_reg_;
    int prescaler_ = 0;
  };

  }  // namespace internal

  class WatchDogClearedTraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<WatchDogClearedTraceEntry> TYPE;

    WatchDogClearedTraceEntry() {}
  };

  class WatchDogTimedOutTraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<WatchDogTimedOutTraceEntry> TYPE;

    WatchDogTimedOutTraceEntry() {}
  };

}  // namespace sim::pic14

#endif  // sim_pic14_watchdog_h
