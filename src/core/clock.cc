#include "clock.h"

#include "trace.h"

namespace sim::core {

  Clock::Clock(Duration interval)
    : interval_(interval) {}

  void Clock::advance_to(TimePoint end_time) {
    Duration d = end_time - sim_now_;

    if (d <= Duration()) return;

    now_ += duration(d / interval_);
    sim_now_ = TimePoint(Duration(now_.time_since_epoch().count() * interval_));

    trace_writer().emplace<ClockAdvancedTraceEntry>(this);
  }

  void ClockScheduler::advance_to(TimePoint end_time) {
    sim_clock_->advance_to(end_time);
    for (auto *clock : clocks_) {
      clock->advance_to(end_time);
    }

    trace_writer().emplace<SimulationClockAdvancedTraceEntry>(sim_clock_);
  }

  ClockModifier::ClockModifier(std::function<void()> changed, Clock *selected, int prescaler)
    : changed_(changed),
      selected_(selected),
      prescaler_(prescaler) {}

  void ClockModifier::select(Clock *selected, int prescaler) {
    if (selected == &selected_.clock() && prescaler == prescaler_) return;

    at_ += selected_.delta() / prescaler_;
    adj_ += selected_.clock().at({}) - selected->at({});
    selected_ = ClockView(selected);
    prescaler_ = prescaler;

    changed_();
  }

}  // namespace sim::core
