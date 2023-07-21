#ifndef sim_core_trace_h
#define sim_core_trace_h

#include "../util/trace.h"
#include "clock.h"

namespace sim::core {

  class Schedulable;
  class SimulationClock;
  class Simulator;

  class ClockAdvancedTraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<ClockAdvancedTraceEntry> TYPE;

    explicit ClockAdvancedTraceEntry(const Clock *clock)
      : clock_(clock), now_(clock->now()) {}

    const Clock* clock() const { return clock_; }
    Clock::time_point now() const { return now_; }

  private:
    const Clock *clock_;
    Clock::time_point now_;
  };

  class SchedulableTraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<SchedulableTraceEntry> TYPE;

    explicit SchedulableTraceEntry(const Schedulable *sch)
      : sch_(sch) {}

    const Schedulable* schedulable() const { return sch_; }

  private:
    const Schedulable *sch_;
  };

  class SimulationClockAdvancedTraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<SimulationClockAdvancedTraceEntry> TYPE;

    explicit SimulationClockAdvancedTraceEntry(const SimulationClock *clock)
      : now_(clock->now()) {}

    TimePoint now() const { return now_; }

  private:
    TimePoint now_;
  };

  class SimulatorTraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<SimulatorTraceEntry> TYPE;

    explicit SimulatorTraceEntry(const Simulator *sim)
      : sim_(sim) {}

    const Simulator* simulator() const { return sim_; }

  private:
    const Simulator *sim_;
  };

  /// Changes the trace buffer capacity, in bytes. This only affects
  /// the construction of the global trace buffer, and it must be
  /// called before the first time the global trace buffer is used.
  ///
  /// Defaults to the value of environment variable
  /// `MICROSIM_TRACE_BUFFER_SIZE`, falling back to zero.
  void set_trace_buffer_capacity(std::size_t cap);
  sim::util::TraceBuffer& trace_buffer();
  sim::util::TraceWriter& trace_writer();

}  // namespace sim::core

#endif  // sim_core_trace_h
