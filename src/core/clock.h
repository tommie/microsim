#ifndef sim_core_clock_h
#define sim_core_clock_h

#include <vector>

#include "simtime.h"

namespace sim::core {

  /// A clock quantizes time and allows querying about time in
  /// multiples of an interval. The interpretation of a tick is up to
  /// the caller.
  class Clock {
  public:
    using duration = std::chrono::duration<long long>;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<Clock, duration>;

    static constexpr bool is_steady() { return true; }

    /// Constructs a new clock with a known interval.
    explicit Clock(Duration interval);

    /// Returns the configured interval of this clock.
    Duration interval() const { return interval_; }

    time_point now() const { return now_; }

    TimePoint at(duration d) const { return sim_now_ + d.count() * interval_; }

    /// Advances the clock. This should be called as simulation
    /// progresses so that `at(0)` always represents the next clock
    /// tick, rather than some time in the past.
    void advance_to(TimePoint end_time);

  private:
    const Duration interval_;
    time_point now_ = {};
    SimulationClock::time_point sim_now_ = {};
  };

  /// A scheduler that advances multiple clocks. This is similar to
  /// `Scheduler`, but only works with clocks. (This way, it does not
  /// require virtual calls.)
  class ClockScheduler {
  public:
    template<typename I>
    ClockScheduler(SimulationClock *sim_clock, I begin, I end)
      : sim_clock_(sim_clock), clocks_(begin, end) {}

    /// Advances the clocks of the scheduler so that `at(0)` returns
    /// some time at or after `end_time`.
    void advance_to(TimePoint end_time);

  private:
    SimulationClock *sim_clock_;
    std::vector<Clock*> clocks_;
  };

  /// A handle on a `Clock` that keeps tracks of deltas to some
  /// "current" time. E.g. an executor that needs to catch up to the
  /// clock's `now` would have a current time in the past. An executor
  /// that is executing an instruction, but needs to skip a clock
  /// cycle will have a current time in the future.
  class ClockView {
  public:
    explicit ClockView(Clock *clock)
      : clock_(clock), at_(clock->now()) {}

    Duration interval() const { return clock_->interval(); }

    /// Returns the simulated point-in-time for `d` ticks in the
    /// future, relative the view's current time.
    TimePoint at(Clock::duration d) const { return clock_->at(d - delta()); }

    /// Resets the view's current time to `now - rem`. Invariant:
    /// `reset(delta)` is a no-op.
    void reset(Clock::duration rem = Clock::duration()) { at_ = clock_->now() - rem; }

    /// Returns the current delta to the clock's now. If positive, the
    /// view's current time is in the past.
    Clock::duration delta() const { return clock_->now() - at_; }

  private:
    Clock *clock_;
    Clock::time_point at_;
  };

}  // namespace sim::core

#endif  // sim_core_clock_h
