#include "scheduler.h"

namespace sim::core {

  Scheduler::~Scheduler() {
    for (auto *s : all_) {
      s->task_queue_ = nullptr;
    }
  }

  Advancement Scheduler::advance_to(const AdvancementLimit &limit) {
    TimePoint at_time = SimulationClock::NEVER;

    for (;;) {
      if (!queue_.empty()) {
        AdvancementLimit sublimit = {
          .end_time = !time_queue_.empty() && (is_never(limit.end_time) || time_queue_.top().first < limit.end_time) ? time_queue_.top().first - Duration(1) : limit.end_time,
          .cond = limit.cond,
          .advanced = limit.advanced,
        };

        while (!queue_.empty()) {
          std::unordered_set<Schedulable*> immediates;
          std::swap(queue_, immediates);

          for (auto *s : immediates) {
            run_one(s, sublimit);
          }

          if (limit.cond && !limit.cond())
            return { .next_time = next_time() };
        }

        continue;
      }

      TimePoint prev_time = SimulationClock::NEVER;
      while (!time_queue_.empty()) {
        std::pair<TimePoint, Schedulable*> top = time_queue_.top();

        if (!is_never(limit.end_time) && top.first > limit.end_time)
          break;

        if (!is_never(prev_time) && top.first > prev_time) {
          break;
        }

        time_queue_.pop();

        if (top.second->next_time_ > top.first) {
          // The scheduling was postponed.
          continue;
        }

        if (is_never(prev_time) && !is_never(at_time) && at_time == top.first) {
          std::abort();  // The scheduler is not making progress.
        }

        enqueue(top.second);
        prev_time = top.first;
        at_time = top.first;

        if (limit.advanced) limit.advanced(top.first);
      }

      if (queue_.empty())
        return { .next_time = next_time() };
    }
  }

  inline TimePoint Scheduler::next_time() const {
    if (!queue_.empty()) return TimePoint(Duration(0));

    return time_queue_.empty() ? SimulationClock::NEVER : time_queue_.top().first;
  }

  inline void Scheduler::run_one(Schedulable *s, const AdvancementLimit &limit) {
    Advancement adv = s->advance_to(limit);

    s->next_time_ = adv.next_time;
    if (!is_never(adv.next_time)) {
      time_queue_.emplace(adv.next_time, s);
    }
  }

}  // namespace sim::core
