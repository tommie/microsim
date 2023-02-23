// This file is not named time.h, because Automake's -I. and <chrono>
// including <time.h> breaks.
#ifndef sim_core_simtime_h
#define sim_core_simtime_h

#include <chrono>

namespace sim::core {

  class SimulationClock {
  public:
    using duration = std::chrono::nanoseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<SimulationClock, duration>;

    static constexpr time_point NEVER = time_point(duration(-1));

    static constexpr bool is_steady() { return true; }

    time_point now() const { return now_; }

    void advance_to(time_point end_time);

  private:
    time_point now_;
  };

  using Duration = SimulationClock::duration;
  using TimePoint = SimulationClock::time_point;

  inline bool operator <(TimePoint a, TimePoint b) { return a - b < Duration(); }
  inline bool operator >(TimePoint a, TimePoint b) { return a - b > Duration(); }
  inline bool operator <=(TimePoint a, TimePoint b) { return a - b <= Duration(); }
  inline bool operator >=(TimePoint a, TimePoint b) { return a - b >= Duration(); }

  inline bool is_never(TimePoint t) { return t== SimulationClock::NEVER; }

}  // namespace sim::core

#endif  // sim_core_simtime_h
