#ifndef sim_core_scheduler_h
#define sim_core_scheduler_h

#include <functional>
#include <queue>
#include <unordered_set>
#include <vector>

namespace sim::core {

  class Schedulable;

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

  /// A set of tasks to run. Order is not guaranteed.
  class TaskQueue {
  public:
    template<typename I>
    TaskQueue(I begin, I end) : queue_(begin, end) {}

    /// Adds a schedulable for execution. It is a no-op if the
    /// schedulable is already in the queue.
    void enqueue(Schedulable *s) { queue_.insert(s); }

  protected:
    std::unordered_set<Schedulable*> queue_;
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

  public:
    /// Advances the object within the given limits.
    virtual Advancement advance_to(const SimulationLimit &limit) = 0;

  protected:
    /// Makes the scheduler advance this schedulable ahead of the
    /// normal queue. Use this if the inputs have changed, requiring
    /// urgent recomputation.
    void schedule_immediately() { task_queue_->enqueue(this); }

  private:
    TaskQueue *task_queue_;
    Ticks next_tick_ = 0;
  };

  /// A scheduler running objects in lockstep.
  class Scheduler : protected TaskQueue {
  public:
    template<typename C>
    explicit Scheduler(C objects)
      : TaskQueue(std::begin(objects), std::end(objects)), all_(std::begin(objects), std::end(objects)) {
      for (auto *o : all_) {
        o->task_queue_ = this;
      }
    }

    template<typename I>
    Scheduler(I begin, I end)
      : TaskQueue(begin, end), all_(begin, end) {
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

}  // namespace sim::core

#endif  // sim_core_scheduler_h
