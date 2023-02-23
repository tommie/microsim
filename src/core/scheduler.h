#ifndef sim_core_scheduler_h
#define sim_core_scheduler_h

#include <functional>
#include <queue>
#include <unordered_set>
#include <vector>

#include "simtime.h"

namespace sim::core {

  class TaskQueue;

  struct AdvancementLimit {
    /// The time at which to stop the simulation, or NEVER.
    TimePoint end_time = SimulationClock::NEVER;

    /// If provided, simulation stops when it returns false. It is
    /// always valid when used as argument to
    /// `Schedulable::advance_to`.
    ///
    /// This function may be invoked any number of times during
    /// `advance_to`; do not count invocations, except that before the
    /// first invocation, some Schedulable will have been given a
    /// chance to advance.
    std::function<bool()> cond;

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
    friend class TaskQueue;

  public:
    /// Advances the object within the given limits.
    virtual Advancement advance_to(const AdvancementLimit &limit) = 0;

  protected:
    /// Makes the scheduler advance this schedulable ahead of the
    /// normal queue. Use this if the inputs have changed, requiring
    /// urgent recomputation.
    void schedule_immediately();

  private:
    TaskQueue *task_queue_ = nullptr;
    TimePoint next_time_ = {};
  };

  /// A set of tasks to run. Order is not guaranteed.
  class TaskQueue {
  public:
    template<typename I>
    TaskQueue(I begin, I end, Schedulable *parent = nullptr)
      : queue_(begin, end), parent_(parent) {}

    /// Adds a schedulable for execution. It is a no-op if the
    /// schedulable is already in the queue.
    void enqueue(Schedulable *s) {
      queue_.insert(s);
      if (parent_) {
        parent_->schedule_immediately();
      }
    }

  protected:
    std::unordered_set<Schedulable*> queue_;
    Schedulable *parent_;
  };

  /// A scheduler running objects in lockstep.
  class Scheduler : protected TaskQueue {
  public:
    template<typename C = std::initializer_list<Schedulable*>>
    explicit Scheduler(C objects, Schedulable *parent = nullptr)
      : Scheduler(std::begin(objects), std::end(objects), parent) {}

    template<typename I>
    Scheduler(I begin, I end, Schedulable *parent = nullptr)
      : TaskQueue(begin, end, parent), all_(begin, end) {
      for (auto *o : all_) {
        o->task_queue_ = this;
      }
    }

    ~Scheduler();

    /// Invokes advance_to on the schedulables. This may run
    /// indefinitely if there keeps being things to do, and no limits
    /// have been set.
    Advancement advance_to(const AdvancementLimit &limit);

  private:
    TimePoint next_time() const;
    void run_one(Schedulable *s, const AdvancementLimit &limit);

  private:
    std::vector<Schedulable*> all_;
    std::priority_queue<std::pair<TimePoint, Schedulable*>,
                        std::vector<std::pair<TimePoint, Schedulable*>>,
                        std::greater<>> time_queue_;
  };

  inline void Schedulable::schedule_immediately() {
    if (task_queue_) {
      task_queue_->enqueue(this);
    }
  }

}  // namespace sim::core

#endif  // sim_core_scheduler_h
