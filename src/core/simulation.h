#ifndef sim_core_simulation_h
#define sim_core_simulation_h

#include <memory>
#include <vector>

#include "../util/status.h"
#include "scheduler.h"

namespace sim::core {

  /// An object that can be simulated.
  class SimulationObject : public Schedulable {
  public:
    virtual ~SimulationObject() = default;
  };

  /// A scheduler can advance the state of objects.
  class Simulator : public Scheduler {
  public:
    explicit Simulator(const std::vector<SimulationObject*> &objects);
  };

  /// A collection of objects to simulate. This is a helper class to
  /// manage the lifetime of objects.
  class SimulationContext {
  public:
    /// Adds an object to be part of the simulation, as a borrowed
    /// pointer.
    void add_object(SimulationObject *o);

    /// Adds an object to be part of the simulation, transferring
    /// ownership.
    void add_object(std::unique_ptr<SimulationObject> o);

    /// Creates a simulator, borrowing object pointers from this
    /// context.
    Simulator make_simulator() { return Simulator(objects_); }

  private:
    std::vector<SimulationObject*> objects_;
    std::vector<std::unique_ptr<SimulationObject>> owned_objects_;
  };

}  // namespace sim::core

#endif  // sim_core_simulation_h
