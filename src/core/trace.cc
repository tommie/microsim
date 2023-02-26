#include "trace.h"

#include <cstdlib>

REGISTER_TRACE_ENTRY_TYPE(ClockAdvancedTraceEntry, sim::core::ClockAdvancedTraceEntry)
REGISTER_TRACE_ENTRY_TYPE(SchedulableTraceEntry, sim::core::SchedulableTraceEntry)
REGISTER_TRACE_ENTRY_TYPE(SimulationClockAdvancedTraceEntry, sim::core::SimulationClockAdvancedTraceEntry)
REGISTER_TRACE_ENTRY_TYPE(SimulatorTraceEntry, sim::core::SimulatorTraceEntry)

namespace sim::core {

  namespace {

    class TraceBufferFactory {
    public:
      TraceBufferFactory() : cap_(env_buffer_capacity()) {}

      void set_capacity(std::size_t cap) {
        cap_ = cap;
      }

      sim::util::TraceBuffer build() const {
        return sim::util::TraceBuffer(cap_);
      }

    private:
      static std::size_t env_buffer_capacity() {
        const char *str = std::getenv("MICROSIM_TRACE_BUFFER_SIZE");

        if (str == nullptr) return 0;

        return std::strtoul(str, nullptr, 0);
      }

    private:
      std::size_t cap_;
    };

    TraceBufferFactory& factory() {
      static TraceBufferFactory factory;
      return factory;
    }

  }

  void set_trace_buffer_capacity(std::size_t cap) {
    factory().set_capacity(cap);
  }

  sim::util::TraceBuffer& trace_buffer() {
    static sim::util::TraceBuffer buf = factory().build();
    return buf;
  }

  sim::util::TraceWriter& trace_writer() {
    static sim::util::TraceWriter writer(&trace_buffer());
    return writer;
  }

}  // namespace sim::core
