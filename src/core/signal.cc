#include "signal.h"

namespace sim::core {

  bool SignalQueue::execute_front() {
    if (queue_.empty()) {
      return false;
    }

    auto *s = queue_.front();
    queue_.pop();

    s->execute();

    return !queue_.empty();
  }

}  // namespace sim::core
