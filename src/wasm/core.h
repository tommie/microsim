#ifndef sim_wasm_core_h
#define sim_wasm_core_h

#include <chrono>

#include "../core/simtime.h"

namespace sim::wasm {

  using jsms = std::chrono::duration<double, std::milli>;
  using jstime = std::chrono::time_point<sim::core::SimulationClock, std::chrono::duration<double, std::milli>>;

  static double to_jstime(sim::core::TimePoint tp) {
    if (tp == sim::core::SimulationClock::NEVER)
      return INFINITY;

    return std::chrono::time_point_cast<jsms>(tp).time_since_epoch().count();
  }

}  // namespace sim::wasm

#endif  // sim_wasm_core_h
