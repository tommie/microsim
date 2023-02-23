#include "simtime.h"

namespace sim::core {

  void SimulationClock::advance_to(time_point end_time) {
    if (end_time < now_) {
      return;
    }
    now_ = end_time;
  }

}  // namespace sim::core
