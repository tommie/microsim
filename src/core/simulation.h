#ifndef sim_core_simulation_h
#define sim_core_simulation_h

#include <memory>
#include <vector>

#include "../util/status.h"
#include "clock.h"
#include "scheduler.h"

namespace sim::core {

  /// An object that can be simulated.
  class SimulationObject : public Schedulable {
  public:
    virtual ~SimulationObject() = default;

    /// Returns any clocks this device drives internally. They will
    /// be added to the `Simulator`'s `ClockScheduler`.
    virtual std::vector<Clock*> clock_sources() { return {}; }
  };

  /// A scheduler can advance the state of clocks and objects.
  class Simulator {
  public:
    template<typename CC = std::initializer_list<Clock*>, typename OC = std::initializer_list<SimulationObject*>>
    explicit Simulator(CC clocks, OC objects)
      : cs_(&sim_clock_, std::begin(clocks), std::end(clocks)),
        s_(std::begin(objects), std::end(objects)) {}

    Advancement advance_to(const AdvancementLimit &limit);

  private:
    SimulationClock sim_clock_;
    ClockScheduler cs_;
    Scheduler s_;
  };

  /// A collection of objects to simulate. This is a helper class to
  /// build a `Simulator` and manage the lifetime of objects.
  class SimulationContext {
  public:
    SimulationContext() = default;

    template<typename CC = std::initializer_list<Clock*>, typename OC = std::initializer_list<SimulationObject*>>
    SimulationContext(CC clocks, OC objects)
      : clocks_(std::begin(clocks), std::end(clocks)),
        objects_(std::begin(objects), std::end(objects)) {}

    /// Adds a clock to be part of the simulation, as a borrowed
    /// pointer.
    void add_clock(Clock *c);

    /// Adds a clock to be part of the simulation, as a transferring
    /// ownership.
    void add_clock(std::unique_ptr<Clock> c);

    /// Adds an object to be part of the simulation, as a borrowed
    /// pointer.
    void add_object(SimulationObject *o);

    /// Adds an object to be part of the simulation, transferring
    /// ownership.
    void add_object(std::unique_ptr<SimulationObject> o);

    /// Creates a simulator, borrowing object pointers from this
    /// context.
    Simulator make_simulator() { return Simulator(clocks_, objects_); }

  private:
    std::vector<Clock*> clocks_;
    std::vector<std::unique_ptr<Clock>> owned_clocks_;

    std::vector<SimulationObject*> objects_;
    std::vector<std::unique_ptr<SimulationObject>> owned_objects_;
  };

}  // namespace sim::core

#endif  // sim_core_simulation_h
