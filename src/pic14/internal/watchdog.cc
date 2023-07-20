#include "watchdog.h"

REGISTER_TRACE_ENTRY_TYPE(WatchDogClearedTraceEntry, sim::pic14::WatchDogClearedTraceEntry)
REGISTER_TRACE_ENTRY_TYPE(WatchDogTimedOutTraceEntry, sim::pic14::WatchDogTimedOutTraceEntry)

namespace sim::pic14::internal {

  WatchDogTimer::WatchDogTimer(sim::core::Clock *lfintosc, sim::core::Signal<bool> *reset, Core::OptionReg &&option_reg, const Core::Config1Reg *config1, Executor::StatusReg &&status_reg)
    : lfintosc_(lfintosc),
      reset_(reset),
      option_reg_(std::move(option_reg)),
      config1_(config1),
      status_reg_(std::move(status_reg)),
      wdtcon_reg_(SingleRegisterBackend<uint8_t>(0x08)) {}

  void WatchDogTimer::reset() {
    lfintosc_.reset();
  }

  void WatchDogTimer::clear() {
    lfintosc_.reset();
    sim::core::trace_writer().emplace<WatchDogClearedTraceEntry>();
    // Clearing only moves the timeout forward, so there is no need to
    // schedule immediately.
  }

  sim::core::Advancement WatchDogTimer::advance_to(const sim::core::AdvancementLimit &limit) {
    if (prescaler_ > 0 && lfintosc_.delta() >= sim::core::Clock::duration(prescaler_)) {
      sim::core::trace_writer().emplace<WatchDogTimedOutTraceEntry>();
      status_reg_.set_to(true);
      lfintosc_.reset();
      reset_->set(true);
      reset_->set(false);
    }

    prescaler_ = prescaled();

    return {
      .next_time = prescaler_ == 0 ? sim::core::SimulationClock::NEVER : lfintosc_.at(sim::core::Clock::duration(prescaler_)),
    };
  }

  uint8_t WatchDogTimer::read_register(uint16_t addr) {
    switch (static_cast<Register>(addr)) {
    case Register::WDTCON: return wdtcon_reg_.read();
    default: std::abort();
    }
  }

  void WatchDogTimer::write_register(uint16_t addr, uint8_t value) {
    switch (static_cast<Register>(addr)) {
    case Register::WDTCON:
      if (value != wdtcon_reg_.read()) {
        wdtcon_reg_.write_masked(value);
        schedule_immediately();
      }
      break;
    }
  }

  int WatchDogTimer::prescaled() const {
    if (!wdtcon_reg_.swdten() && !config1_->wdte())
      return 0;

    return (32 << wdtcon_reg_.wdtps()) * (option_reg_.psa() ? (2 << option_reg_.ps()) : 1);
  }

}  // namespace sim::pic14::internal
