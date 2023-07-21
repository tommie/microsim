#include <string>

#include <emscripten/bind.h>

#include "../core/trace.h"
#include "../pic14/trace.h"
#include "../util/trace.h"
#include "core.h"
#include "util.h"

using namespace emscripten;
using namespace sim::wasm;

namespace {

  sim::util::TraceBuffer* TraceBuffer_getGlobal() {
    return &sim::core::trace_buffer();
  }

  val TraceEntry_getDecoded(sim::util::TraceEntry &self) {
    return self.visit<
      sim::core::ClockAdvancedTraceEntry,
      sim::core::SimulationClockAdvancedTraceEntry,
      sim::pic14::ADConversionDoneTraceEntry,
      sim::pic14::WroteEEDATATraceEntry,
      sim::pic14::WroteProgramFlashTraceEntry,
      sim::pic14::EUSARTDataTraceEntry,
      sim::pic14::ExecutedTraceEntry,
      sim::pic14::WatchDogClearedTraceEntry,
      sim::pic14::WatchDogTimedOutTraceEntry
      >([](const auto &e) {
        using T = std::decay_t<decltype(e)>;
        if constexpr (std::is_same_v<T, sim::core::ClockAdvancedTraceEntry>) {
          return val(std::make_unique<sim::core::ClockAdvancedTraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::core::SimulationClockAdvancedTraceEntry>) {
          return val(std::make_unique<sim::core::SimulationClockAdvancedTraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::pic14::ADConversionDoneTraceEntry>) {
          return val(std::make_unique<sim::pic14::ADConversionDoneTraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::pic14::WroteEEDATATraceEntry>) {
          return val(std::make_unique<sim::pic14::WroteEEDATATraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::pic14::WroteProgramFlashTraceEntry>) {
          return val(std::make_unique<sim::pic14::WroteProgramFlashTraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::pic14::EUSARTDataTraceEntry>) {
          return val(std::make_unique<sim::pic14::EUSARTDataTraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::pic14::ExecutedTraceEntry>) {
          return val(std::make_unique<sim::pic14::ExecutedTraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::pic14::WatchDogClearedTraceEntry>) {
          return val(std::make_unique<sim::pic14::WatchDogClearedTraceEntry>(e));
        } else if constexpr (std::is_same_v<T, sim::pic14::WatchDogTimedOutTraceEntry>) {
          return val(std::make_unique<sim::pic14::WatchDogTimedOutTraceEntry>(e));
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

  template<typename Subject>
  auto Some_address(const Subject &self) {
    return self.addr();
  }

  unsigned int EUSARTDataTraceEntry_data(const sim::pic14::EUSARTDataTraceEntry &self) {
    return self.data();
  }

  bool EUSARTDataTraceEntry_framingError(const sim::pic14::EUSARTDataTraceEntry &self) {
    return self.ferr();
  }

  auto EUSARTDataTraceEntry_mode(const sim::pic14::EUSARTDataTraceEntry &self) {
    return self.mode();
  }

  unsigned int EUSARTDataTraceEntry_numBits(const sim::pic14::EUSARTDataTraceEntry &self) {
    return self.num_bits();
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

    // --- PIC14 Trace Entries ---

    class_<sim::pic14::ADConversionDoneTraceEntry, base<sim::util::TraceEntryBase>>("ADConversionDoneTraceEntry");
    class_<sim::pic14::WroteEEDATATraceEntry, base<sim::util::TraceEntryBase>>("WroteEEDATATraceEntry")
      .property("address", &Some_address<sim::pic14::WroteEEDATATraceEntry>);
    class_<sim::pic14::WroteProgramFlashTraceEntry, base<sim::util::TraceEntryBase>>("WroteProgramFlashTraceEntry")
      .property("address", &Some_address<sim::pic14::WroteProgramFlashTraceEntry>);
    class_<sim::pic14::EUSARTDataTraceEntry, base<sim::util::TraceEntryBase>>("EUSARTDataTraceEntry")
      .property("data", &EUSARTDataTraceEntry_data)
      .property("framingError", &EUSARTDataTraceEntry_framingError)
      .property("mode", &EUSARTDataTraceEntry_mode)
      .property("numBits", &EUSARTDataTraceEntry_numBits);
    class_<sim::pic14::ExecutedTraceEntry, base<sim::util::TraceEntryBase>>("ExecutedTraceEntry")
      .property("address", &Some_address<sim::pic14::ExecutedTraceEntry>);
    class_<sim::pic14::WatchDogClearedTraceEntry, base<sim::util::TraceEntryBase>>("WatchDogClearedTraceEntry");
    class_<sim::pic14::WatchDogTimedOutTraceEntry, base<sim::util::TraceEntryBase>>("WatchDogTimedOutTraceEntry");

  }

}  // namespace
