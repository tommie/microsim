#include "scheduler.h"

#include "trace.h"

namespace sim::core {

  /// An AdvanceMarker sets and clears the TaskQueue::advancing_
  /// flag. This is used to avoid propagating immediate schedulings
  /// further than necessary.
  class AdvanceMarker {
  public:
    explicit AdvanceMarker(Scheduler *sch)
      : sch_(sch) {
      if (sch->advancing_) std::abort();  // Scheduler is not re-entrant.
      sch->advancing_ = true;
    }
    ~AdvanceMarker() { sch_->advancing_ = false; }

  private:
    Scheduler *sch_;
  };

  Scheduler::~Scheduler() {
    for (auto *s : all_) {
      s->scheduler_ = nullptr;
    }
  }

  Advancement Scheduler::advance_to(const AdvancementLimit &limit) {
    AdvanceMarker marker(this);

    for (;;) {
      if (immediates_.empty() && !advance_next(limit)) {
        return {
          .next_time = time_queue_.empty() ? SimulationClock::NEVER : time_queue_.top().first,
        };
      }

      run_immediates(limit);
    }
  }

  bool Scheduler::advance_next(const AdvancementLimit &limit) {
    TimePoint at_time = SimulationClock::NEVER;

    while (!time_queue_.empty()) {
      std::pair<TimePoint, Schedulable*> top = time_queue_.top();

      if (top.second->next_time_ != top.first) {
        // The scheduling was postponed.
        time_queue_.pop();
        continue;
      }

      if (limit.can_advance_to && !limit.can_advance_to(top.first)) {
        break;
      }

      if (!is_never(at_time) && top.first > at_time) {
        // We are beyond the leading edge in the queue.
        break;
      }

      time_queue_.pop();
      immediates_.insert(top.second);
      at_time = top.first;
    }

    if (is_never(at_time)) {
      return false;
    }

    if (limit.advanced) limit.advanced(at_time);

    return true;
  }

  void Scheduler::run_immediates(const AdvancementLimit &limit) {
    AdvancementLimit sublimit = {
      .can_advance_to = [this, &limit](TimePoint at) {
        return immediates_.empty() &&
          (time_queue_.empty() || time_queue_.top().first >= at) &&
          (!limit.can_advance_to || limit.can_advance_to(at));
      },
      .advanced = limit.advanced,
    };

    std::unordered_set<Schedulable*> immediates;
    std::swap(immediates_, immediates);

    for (auto *s : immediates) {
      trace_writer().emplace<SchedulableTraceEntry>(s);

      Advancement adv = s->advance_to(sublimit);

      s->next_time_ = adv.next_time;
      if (!is_never(adv.next_time)) {
        time_queue_.emplace(adv.next_time, s);
      }
    }
  }

}  // namespace sim::core
