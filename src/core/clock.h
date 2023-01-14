#ifndef sim_core_clock_domain_h
#define sim_core_clock_domain_h

#include <vector>

#include "simulation.h"

namespace sim::core {

  /// A clock quantizes time and allows querying about time in
  /// multiples of an interval. The interpretation of a tick is up to
  /// the caller.
  class Clock {
  public:
    /// Constructs a new clock with a known interval.
    explicit Clock(Ticks interval) : interval_(interval), next_tick_(interval) {}

    /// Returns the tick at the given future interval. `at(0)` is the
    /// next clock tick, and so on. A negative number works as
    /// expected.
    Ticks at(Ticks n) const { return next_tick_ + n * interval_; }

    /// Returns the configured interval of this clock.
    Ticks interval() const { return interval_; }

    /// Advances the clock. This should be called as simulation
    /// progresses so that `at(0)` always represents the next clock
    /// tick, rather than some time in the past.
    void advance_to(Ticks end_tick) {
      Ticks d = end_tick - (next_tick_ + interval_);

      if (d < 0) return;

      next_tick_ += d % interval_ + interval_ * d / interval_;
    }

  private:
    const Ticks interval_;
    Ticks next_tick_;
  };

  /// A scheduler that advances multiple clocks. This is similar to
  /// `Scheduler`, but only works with clocks. (This way, it does not
  /// require virtual calls.)
  class ClockScheduler {
  public:
    template<typename C>
    explicit ClockScheduler(C clocks) : clocks_(clocks) {}

    /// Advances the clocks of the scheduler so that `at(0)` returns
    /// some time at or after `end_tick`.
    void advance_to(Ticks end_tick) {
      for (auto *clock : clocks_) {
        clock->advance_to(end_tick);
      }
    }

  private:
    std::vector<Clock*> clocks_;
  };

}  // namespace sim::core

#endif  // sim_core_clock_domain_h
