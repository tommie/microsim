#ifndef sim_core_scheduler_h
#define sim_core_scheduler_h

#include <functional>
#include <queue>
#include <unordered_set>
#include <vector>

namespace sim::core {

  class TaskQueue;

  /// A measurement of wall-clock time in unspecified units.
  typedef long long Ticks;

  struct SimulationLimit {
    /// The tick at which to stop simulation. -1 means no limit.
    Ticks end_tick = -1;

    /// If provided, simulation stops when it returns false. The
    /// argument is the number of ticks simulated so far. This
    /// function may be invoked any number of times during
    /// `advance_to`; do not count invocations.
    std::function<bool(Ticks)> cond;
  };

  /// The result of a call to `advance_to`.
  struct Advancement {
    /// The tick the schedulable is currently at. This is not used by
    /// Scheduler, but is returned by it. It provides a lower bound on
    /// all future possible ticks.
    Ticks at_tick;

    /// The tick when the schedulable needs to be advanced next. A
    /// value of -1 means "never."
    Ticks next_tick;
  };

  /// An object driven by some clock.
  class Schedulable {
    friend class Scheduler;
    friend class TaskQueue;

  public:
    /// Advances the object within the given limits.
    virtual Advancement advance_to(const SimulationLimit &limit) = 0;

  protected:
    /// Makes the scheduler advance this schedulable ahead of the
    /// normal queue. Use this if the inputs have changed, requiring
    /// urgent recomputation.
    void schedule_immediately();

  private:
    TaskQueue *task_queue_ = nullptr;
    Ticks next_tick_ = 0;
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
    Advancement advance_to(const SimulationLimit &limit);

  private:
    Ticks next_tick() const {
      if (!queue_.empty()) return at_tick_;

      return tick_queue_.empty() ? -1 : tick_queue_.top().first;
    }

    void run_one(Schedulable *s, SimulationLimit &&limit);

  private:
    std::vector<Schedulable*> all_;
    std::priority_queue<std::pair<Ticks, Schedulable*>,
                        std::vector<std::pair<Ticks, Schedulable*>>,
                        std::greater<>> tick_queue_;
    Ticks at_tick_ = 0;
  };

  inline void Schedulable::schedule_immediately() {
    if (task_queue_) {
      task_queue_->enqueue(this);
    }
  }

}  // namespace sim::core

#endif  // sim_core_scheduler_h
