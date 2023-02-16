#include "scheduler.h"

namespace sim::core {

  Scheduler::~Scheduler() {
    for (auto *s : all_) {
      s->task_queue_ = nullptr;
    }
  }

  Advancement Scheduler::advance_to(const SimulationLimit &limit) {
    while ((!tick_queue_.empty() || !queue_.empty()) && (limit.end_tick < 0 || next_tick() < 0 || limit.end_tick - next_tick() >= 0)) {
      if (!queue_.empty()) {
        while (!queue_.empty()) {
          std::unordered_set<Schedulable*> immediates;
          std::swap(queue_, immediates);

          for (auto *s : immediates) {
            SimulationLimit sublimit = limit;
            sublimit.end_tick = at_tick_;

            run_one(s, std::move(sublimit));
          }
        }

        if (limit.cond && !limit.cond(at_tick_))
          break;

        continue;
      }

      std::pair<Ticks, Schedulable*> top = tick_queue_.top();
      auto *s = top.second;

      tick_queue_.pop();

      if (top.second->next_tick_ - top.first > 0) {
        // The scheduling was postponed.
        continue;
      }

      SimulationLimit sublimit = limit;
      sublimit.cond = [this, &limit](Ticks at_tick) {
        // Always run limit.cond, since it may have side effects.
        return (!limit.cond || limit.cond(at_tick)) && queue_.empty();
      };
      run_one(s, std::move(sublimit));

      if (limit.cond && !limit.cond(at_tick_))
        break;
    }

    return {
      .at_tick = at_tick_,
      .next_tick = next_tick(),
    };
  }

  inline void Scheduler::run_one(Schedulable *s, SimulationLimit &&limit) {
    SimulationLimit sublimit = std::move(limit);
    sublimit.end_tick = (limit.end_tick >= 0 ? std::min(limit.end_tick, next_tick()) : next_tick());

    Advancement adv = s->advance_to(sublimit);

    if (adv.at_tick - at_tick_ < 0)
      std::abort();  // An object going back in time is a programming error.

    if (adv.next_tick >= 0 && s->next_tick_ >= 0 && adv.next_tick - s->next_tick_ < 0)
      std::abort();  // An object going back in time is a programming error.

    s->next_tick_ = adv.next_tick;
    if (adv.next_tick >= 0) {
      tick_queue_.emplace(adv.next_tick, s);
    }
    at_tick_ = std::max(at_tick_, adv.at_tick);
  }

}  // namespace sim::core
