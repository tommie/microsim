#include <chrono>
#include <memory>
#include <vector>

#include <emscripten/bind.h>

#include "../core/clock.h"
#include "../core/device.h"
#include "../core/scheduler.h"
#include "../core/simulation.h"
#include "core.h"
#include "util.h"

using namespace emscripten;
using namespace sim::wasm;

namespace {

  class DeviceListenerWrapper : public wrapper<sim::core::DeviceListener> {
  public:
    EMSCRIPTEN_WRAPPER(DeviceListenerWrapper);

    void invalid_internal_state(const sim::util::Status &status) override {
      call<void>("invalidInternalState", StatusWrapper(status));
    }

    void pin_changed(sim::core::Pin *pin, PinChange change) override {
      call<void>("pinChanged", val(pin), change);
    }
  };

  class SimulationObjectWrapper : public wrapper<sim::core::SimulationObject> {
  public:
    EMSCRIPTEN_WRAPPER(SimulationObjectWrapper);

    sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit) override {
      return call<sim::core::Advancement>("advanceTo", limit);
    }

    std::vector<sim::core::Clock*> clock_sources() override {
      std::vector<sim::core::Clock*> clocks;
      for (const val &v : vecFromJSArray<val>(call<val>("getClockSources"))) {
        clocks.push_back(v.as<sim::core::Clock*>(allow_raw_pointers()));
      }

      return clocks;
    }
  };

  double Advancement_nextTime(const sim::core::Advancement &self) {
    return to_jstime(self.next_time);
  }

  void Advancement_nextTime_setter(sim::core::Advancement &self, double at) {
    self.next_time = sim::core::TimePoint(std::chrono::duration_cast<sim::core::Duration>(jsms(at)));
  }

  std::unique_ptr<sim::core::AdvancementLimit> newAdvancementLimit(val init) {
    sim::core::AdvancementLimit limit = {};

    val can_advance_to = init["canAdvanceTo"];
    if (can_advance_to.as<bool>()) {
      limit.can_advance_to = [can_advance_to = std::move(can_advance_to)](sim::core::TimePoint at) -> bool {
        return can_advance_to(to_jstime(at)).as<bool>();
      };
    }

    val advanced = init["advanced"];
    if (advanced.as<bool>()) {
      limit.advanced = [advanced = std::move(advanced)](sim::core::TimePoint at) -> bool {
        return advanced(to_jstime(at)).as<bool>();
      };
    }

    return std::make_unique<sim::core::AdvancementLimit>(limit);
  }

  bool AdvancementLimit_canAdvanceTo(const sim::core::AdvancementLimit &self, double at) {
    if (!self.can_advance_to) return true;
    return self.can_advance_to(sim::core::TimePoint(std::chrono::duration_cast<sim::core::Duration>(jsms(at))));
  }

  void AdvancementLimit_advanced(const sim::core::AdvancementLimit &self, double at) {
    if (!self.advanced) return;
    self.advanced(sim::core::TimePoint(std::chrono::duration_cast<sim::core::Duration>(jsms(at))));
  }

  std::unique_ptr<sim::core::Clock> newClock(double ival) {
    return std::make_unique<sim::core::Clock>(std::chrono::duration_cast<sim::core::Duration>(jsms(ival)));
  }

  double Clock_at(const sim::core::Clock &self, long at) {
    auto tp = self.at(sim::core::Clock::duration(at));
    return std::chrono::time_point_cast<jsms>(tp).time_since_epoch().count();
  }

  double Clock_interval(const sim::core::Clock &self) {
    return std::chrono::duration_cast<jsms>(self.interval()).count();
  }

  double Clock_now(const sim::core::Clock &self) {
    return std::chrono::time_point_cast<jsms>(self.now()).time_since_epoch().count();
  }

  void Clock_advanceTo(sim::core::Clock &self, double at) {
    self.advance_to(sim::core::TimePoint(std::chrono::duration_cast<sim::core::Duration>(jsms(at))));
  }

  uintptr_t Pin_ptr(const sim::core::Pin &self) {
    return reinterpret_cast<uintptr_t>(&self);
  }

  sim::core::Pin* PinDescriptor_getPin(const sim::core::PinDescriptor &self) {
    return self.pin;
  }

  EMSCRIPTEN_BINDINGS(core) {

    enum_<sim::core::DeviceListener::PinChange>("PinChange")
      .value("VALUE", sim::core::DeviceListener::VALUE)
      .value("RESISTANCE", sim::core::DeviceListener::RESISTANCE);

    value_object<sim::core::Advancement>("Advancement")
      .field("nextTime", &Advancement_nextTime, &Advancement_nextTime_setter);

    class_<sim::core::AdvancementLimit>("AdvancementLimit")
      .constructor(&newAdvancementLimit)
      .function("canAdvanceTo", &AdvancementLimit_canAdvanceTo)
      .function("advanced", &AdvancementLimit_advanced);

    class_<sim::core::Clock>("Clock")
      .constructor(&newClock)
      .property("interval", &Clock_interval)
      .property("now", &Clock_now)
      .function("at", &Clock_at)
      .function("advanceTo", &Clock_advanceTo);

    class_<sim::core::DeviceListener>("DeviceListener")
      .allow_subclass<DeviceListenerWrapper>("DeviceListenerWrapper", constructor<>())
      .function("invalidInternalState", &sim::core::DeviceListener::invalid_internal_state)
      .function("pinChanged", &sim::core::DeviceListener::pin_changed, allow_raw_pointers());

    class_<sim::core::Device, base<sim::core::SimulationObject>>("Device")
      .property("pins", &sim::core::Device::pins);

    class_<sim::core::Pin>("Pin")
      .property("$ptr", &Pin_ptr)  // For comparing pin objects to one another.
      .property("value", &sim::core::Pin::value)
      .property("resistance", &sim::core::Pin::resistance)
      .function("setExternal", &sim::core::Pin::set_external);

    // TODO: property doesn't support raw pointers.
    class_<sim::core::PinDescriptor>("PinDescriptor")
      .function("getPin", &PinDescriptor_getPin, allow_raw_pointers())
      .property("name", &sim::core::PinDescriptor::name)
      .property("path", &sim::core::PinDescriptor::path);

    class_<sim::core::Schedulable>("Schedulable")
      .function("advanceTo", &sim::core::Schedulable::advance_to);

    class_<sim::core::SimulationContext>("SimulationContext")
      .constructor()
      .function("addClock", select_overload<void(sim::core::Clock*)>(&sim::core::SimulationContext::add_clock), allow_raw_pointers())
      .function("addObject", select_overload<void(sim::core::SimulationObject*)>(&sim::core::SimulationContext::add_object), allow_raw_pointers())
      .function("makeSimulator", &sim::core::SimulationContext::make_simulator);

    class_<sim::core::SimulationObject, base<sim::core::Schedulable>>("SimulationObject")
      .allow_subclass<SimulationObjectWrapper>("SimulationObjectWrapper", constructor<>())
      .function("getClockSources", &sim::core::SimulationObject::clock_sources);

    class_<sim::core::Simulator>("Simulator")
      .function("advanceTo", &sim::core::Simulator::advance_to);

    register_vector<sim::core::PinDescriptor>("PinDescriptors");

  }

}  // namespace
