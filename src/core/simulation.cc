#include "simulation.h"

#include "trace.h"

namespace sim::core {

  Advancement Simulator::advance_to(const AdvancementLimit &limit) {
    AdvancementLimit sublimit = limit;

    sublimit.advanced = [this, &limit](TimePoint at_time) {
      cs_.advance_to(at_time);
      if (limit.advanced) limit.advanced(at_time);
    };

    trace_writer().emplace<SimulatorTraceEntry>(this);

    return s_.advance_to(sublimit);
  }

  void SimulationContext::add_clock(Clock *c) {
    clocks_.push_back(c);
  }

  void SimulationContext::add_clock(std::unique_ptr<Clock> c) {
    add_clock(c.get());
    owned_clocks_.push_back(std::move(c));
  }

  void SimulationContext::add_object(SimulationObject *o) {
    objects_.push_back(o);

    for (auto *c : o->clock_sources()) {
      add_clock(c);
    }
  }

  void SimulationContext::add_object(std::unique_ptr<SimulationObject> o) {
    add_object(o.get());
    owned_objects_.push_back(std::move(o));
  }

}  // namespace sim::core
