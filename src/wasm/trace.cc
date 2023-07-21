#include <string>

#include <emscripten/bind.h>

#include "../core/trace.h"
#include "../util/trace.h"
#include "core.h"
#include "util.h"

using namespace emscripten;

namespace sim::wasm {

  sim::util::TraceBuffer* TraceBuffer_getGlobal() {
    return &sim::core::trace_buffer();
  }

  val TraceEntry_getDecoded(sim::util::TraceEntry &self) {
    return self.visit<
      sim::core::ClockAdvancedTraceEntry,
      sim::core::SimulationClockAdvancedTraceEntry
      >([](const auto &e) {
        using T = std::decay_t<decltype(e)>;
        if constexpr (std::is_same_v<T, sim::core::ClockAdvancedTraceEntry>) {
          return val(std::make_unique<sim::core::ClockAdvancedTraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::core::SimulationClockAdvancedTraceEntry>) {
          return val(std::make_unique<sim::core::SimulationClockAdvancedTraceEntry>(e));
        } else {
          return val::null();
        }
      });
  }

  double ClockAdvancedTraceEntry_now(const sim::core::ClockAdvancedTraceEntry &self) {
    return to_jstime(sim::core::TimePoint(self.now().time_since_epoch().count() * self.clock()->interval()));
  }

  double SimulationClockAdvancedTraceEntry_now(const sim::core::SimulationClockAdvancedTraceEntry &self) {
    return to_jstime(self.now());
  }

  EMSCRIPTEN_BINDINGS(trace) {

    class_<sim::util::TraceEntryBase>("TraceEntryBase");

    class_<sim::util::TraceBuffer>("TraceBuffer")
      .class_function("setGlobalCapacity", &sim::core::set_trace_buffer_capacity)
      .class_function("getGlobal", &TraceBuffer_getGlobal, allow_raw_pointers())
      .property("capacity", &sim::util::TraceBuffer::capacity)
      .property("discarded", &sim::util::TraceBuffer::discarded)
      .property("length", &sim::util::TraceBuffer::entries)
      .property("top", &sim::util::TraceBuffer::top)
      .function("pop", &sim::util::TraceBuffer::pop);

    class_<sim::util::TraceEntry>("TraceEntry")
      .property("kind", &sim::util::TraceEntry::kind)
      .function("getDecoded", &TraceEntry_getDecoded);

    // --- Core Trace Entries ---

    class_<sim::core::ClockAdvancedTraceEntry, base<sim::util::TraceEntryBase>>("ClockAdvancedTraceEntry")
      .property("now", &ClockAdvancedTraceEntry_now);

    class_<sim::core::SimulationClockAdvancedTraceEntry, base<sim::util::TraceEntryBase>>("SimulationClockAdvancedTraceEntry")
      .property("now", &SimulationClockAdvancedTraceEntry_now);

  }

}  // namespace sim::wasm
