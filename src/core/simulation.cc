#include "simulation.h"

namespace sim::core {

  void SimulationContext::add_object(SimulationObject *o) {
    objects_.push_back(o);
  }

  void SimulationContext::add_object(std::unique_ptr<SimulationObject> o) {
    objects_.push_back(o.get());
    owned_objects_.push_back(std::move(o));
  }

  Simulator::Simulator(const std::vector<SimulationObject*> &objects)
    : Scheduler(std::begin(objects), std::end(objects)) {}

}  // namespace sim::core
