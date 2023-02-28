#ifndef sim_core_scheduler_h
#define sim_core_scheduler_h

#include <functional>
#include <queue>
#include <unordered_set>
#include <vector>

#include "simtime.h"

namespace sim::core {

  class Scheduler;

  struct AdvancementLimit {
    /// If provided, simulation stops when it returns false.
    ///
    /// This function may be invoked any number of times during
    /// `advance_to`; do not count invocations. Any immediate
    /// schedulables will have been run before this is called.
    std::function<bool(TimePoint)> can_advance_to;

    /// This function must be invoked after every time advancement. It
    /// is provided as an alternative to returning from
    /// `Schedulable::advance_to`, to avoid having to pop the call
    /// stack all the way to the root after every instruction cycle.
    std::function<void(TimePoint at_time)> advanced;
  };

  /// The result of a call to `advance_to`.
  struct Advancement {
    /// The time when the schedulable needs to be advanced next. This
    /// may be before what the `Schedulable` returned previously, but
    /// cannot be before the latest time provided to
    /// `AdvancementLimit::advanced`.
    TimePoint next_time = SimulationClock::NEVER;
  };

  /// An object driven by some clock.
  class Schedulable {
    friend class Scheduler;

  public:
    /// Advances the object within the given limits.
    virtual Advancement advance_to(const AdvancementLimit &limit) = 0;

  protected:
    /// Makes the scheduler advance this schedulable ahead of the
    /// normal queue. Use this if the inputs have changed, requiring
    /// urgent recomputation.
    void schedule_immediately();

  private:
    Scheduler *scheduler_ = nullptr;
    TimePoint next_time_ = {};
  };

  /// A scheduler running objects in lockstep.
  class Scheduler {
    friend class AdvanceMarker;
    friend class Schedulable;

  public:
    template<typename C = std::initializer_list<Schedulable*>>
    explicit Scheduler(C objects, Schedulable *parent = nullptr)
      : Scheduler(std::begin(objects), std::end(objects), parent) {}

    template<typename I>
    Scheduler(I begin, I end, Schedulable *parent = nullptr)
      : all_(begin, end), immediates_(begin, end), parent_(parent) {
      for (auto *o : all_) {
        o->scheduler_ = this;
      }
    }

    ~Scheduler();

    /// Invokes advance_to on the schedulables. This may run
    /// indefinitely if there keeps being things to do, and no limits
    /// have been set.
    Advancement advance_to(const AdvancementLimit &limit);

  private:
    /// Adds a schedulable for execution. It is a no-op if the
    /// schedulable is already in the queue.
    void enqueue(Schedulable *s) {
      immediates_.insert(s);
      if (parent_ && !advancing_) {
        parent_->schedule_immediately();
      }
    }

    bool advance_next(const AdvancementLimit &limit);
    void run_immediates(const AdvancementLimit &limit);

  private:
    std::vector<Schedulable*> all_;
    std::priority_queue<std::pair<TimePoint, Schedulable*>,
                        std::vector<std::pair<TimePoint, Schedulable*>>,
                        std::greater<>> time_queue_;
    std::unordered_set<Schedulable*> immediates_;
    Schedulable *parent_;
    bool advancing_ = false;
  };

  inline void Schedulable::schedule_immediately() {
    if (scheduler_) {
      scheduler_->enqueue(this);
    }
  }

}  // namespace sim::core

#endif  // sim_core_scheduler_h
