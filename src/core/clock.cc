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

}  // namespace sim::core
