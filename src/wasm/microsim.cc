#include <emscripten/bind.h>

#include "../core/device.h"
#include "../core/scheduler.h"
#include "../core/simulation.h"
#include "../core/trace.h"
#include "../pic14/p16f88x.h"
#include "../util/trace.h"

using namespace emscripten;

namespace {

  class DeviceListenerWrapper : public wrapper<sim::core::DeviceListener> {
  public:
    EMSCRIPTEN_WRAPPER(DeviceListenerWrapper);

    void invalid_internal_state(const sim::util::Status &status) override {
      call<void>("invalidInternalState", status);
    }

    void pin_changed(sim::core::Pin *pin, PinChange change) override {
      call<void>("pinChanged", val(pin), change);
    }
  };

EMSCRIPTEN_BINDINGS(util) {

  class_<sim::util::TraceEntryBase>("TraceEntryBase");

}

EMSCRIPTEN_BINDINGS(core) {

#if 0
  enum_<sim::core::DeviceListener::PinChange>("PinChange")
    .value("VALUE", sim::core::DeviceListener::VALUE)
    .value("RESISTANCE", sim::core::DeviceListener::RESISTANCE);

  class_<sim::core::Advancement>("Advancement")
    .constructor()
    .property("nextTime", &sim::core::Advancement::next_time);

  class_<sim::core::AdvancementLimit>("AdvancementLimit")
    .constructor()
    .property("canAdvanceTo", &sim::core::AdvancementLimit::can_advance_to)
    .property("advanced", &sim::core::AdvancementLimit::advanced);

  class_<sim::core::Clock>("Clock")
    .constructor()
    .property("interval", &sim::core::Advancement::interval)
    .property("now", &sim::core::Advancement::now);
#endif

  class_<sim::core::DeviceListener>("DeviceListener")
    .allow_subclass<DeviceListenerWrapper>("DeviceListener", constructor<>())
    .function("invalidInternalState", &sim::core::DeviceListener::invalid_internal_state)
    .function("pinChanged", &sim::core::DeviceListener::pin_changed, allow_raw_pointers());

  class_<sim::core::Device, base<sim::core::SimulationObject>>("Device");

#if 0
  class_<sim::core::Pin>("Pin")
    .property("value", &sim::core::Pim::value)
    .property("resistance", &sim::core::Pim::resistance)
    .function("setExternal", &sim::core::Pim::setExternal);

  class_<sim::core::PinDescriptor>("PinDescriptor")
    .property("pin", &sim::core::PinDescriptor::pin, allow_raw_pointers())
    .property("name", &sim::core::PinDescriptor::name)
    .property("path", &sim::core::PinDescriptor::path);
#endif

  class_<sim::core::Schedulable>("Schedulable");

#if 0
  class_<sim::core::SimulationContext>("SimulationContext")
    .constructor()
    .function("addObject", &sim::core::SimulationContext::add_object)
    .function("makeSimulator", &sim::core::SimulationContext::make_simulator);
#endif

  class_<sim::core::SimulationObject, base<sim::core::Schedulable>>("SimulationObject");
  //.allow_subclass<SimulationObjectWrapper>("SimulationObject", constructor<>());

#if 0
  class_<sim::core::Simulator>("Simulator")
    .function("advanceTo", &sim::core::Simulator::advance_to);

  class_<sim::core::TraceBuffer>("TraceBuffer")
    .class_property("setGlobalTraceBufferCapacity", &sim::core::setTraceBufferCapacity)
    .class_property("global", &sim::core::trace_buffer)
    .property("empty", &sim::core::TraceBuffer::empty)
    .function("pop", &sim::core::TraceBuffer::pop)
    .property("top", &sim::core::TraceBuffer::top)
    .property("length", &sim::core::TraceBuffer::entries)
    .property("discarded", &sim::core::TraceBuffer::discarded);

  class_<sim::core::TraceEntry>("TraceEntry")
    .property("kind", &sim::core::TraceEntry::kind)
    .function("getDecoded", [](sim::core::TraceEntry &self){
      self.visit<
        sim::core::ClockAdvancedTraceEntry,
        >([](const auto &e) {
          using T = std::decay_t<decltype(e)>;
          if constexpr (std::is_same_v<T, sim::core::ClockAdvancedTraceEntry>) {
            return val(std::make_unique<sim::core::ClockAdvancedTraceEntry>(e));
          } else {
            return val::null();
          }
        });
    });

  // --- Core Trace Entries ---

  class_<sim::core::ClockAdvancedTraceEntry, base<sim::util::TraceEntryBase>>("ClockAdvancedTraceEntry")
    .property("now", &sim::core::ClockAdvancedTraceEntry::now);
#endif

}

EMSCRIPTEN_BINDINGS(pic14) {

  class_<sim::pic14::ICSP>("Pic14ICSP")
    .function("loadProgram", &sim::pic14::ICSP::load_program)
    .function("loadData", &sim::pic14::ICSP::load_data);

  class_<sim::pic14::P16F887, base<sim::core::Device>>("P16F887")
    .constructor<sim::core::DeviceListener*, sim::core::Clock*>()
    .function("enterICSP", &sim::pic14::P16F887::enter_icsp)
    .property("sleeping", &sim::pic14::P16F887::is_sleeping);

}

}  // namespace
