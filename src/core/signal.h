#ifndef sim_core_signal_h
#define sim_core_signal_h

#include <queue>
#include <vector>

namespace sim::core {

  namespace internal {

    class SignalBase {
    public:
      virtual void execute() = 0;
    };

  }  // namespace internal

  /// A queue of emitted signals. The scheduler will use
  /// `execute_front` in a loop to execute the handlers and reset the
  /// signal to unemitted state.
  class SignalQueue {
    template<typename H, void(H::*F)()>
    friend class Signal;

  public:
    SignalQueue() = default;

    /// Executes all handlers of the front signal, if any. Returns
    /// false if there are no more signals queued.
    bool execute_front();

  private:
    /// Called by `Signal::emit`.
    void enqueue(internal::SignalBase *s) { queue_.push(s); }

  private:
    std::queue<internal::SignalBase*> queue_;
  };

  /// A signal is like a signal wire that can be asserted by some
  /// module, and responded to by others. Like simple wires, it can
  /// say that something has happened, but with no further details.
  ///
  /// The set of handlers is static after construction.
  template<typename H, void(H::*F)()>
  class Signal : private internal::SignalBase {
    friend class SignalQueue;

  public:
    /// Constructs a new signal attached to the given queue, with the
    /// given handlers. All pointers are borrowed and must remain
    /// valid until the signal is destroyed.
    template<typename C>
    Signal(SignalQueue *queue, C handlers, bool emitting = false)
      : queue_(queue),
        handlers_(std::begin(handlers), std::end(handlers)) {
      if (emitting) emit();
    }

    /// Asserts the signal. If it was already asserted, this does
    /// nothing. Calling `SignalQueue::execute_front` resets the
    /// signal.
    void emit() {
      if (!enqueued_) {
        enqueued_ = true;
        queue_->enqueue(this);
      }
    }

  private:
    /// Called by `SignalQueue` to run all handlers.
    void execute() override {
      enqueued_ = false;
      for (auto *h : handlers_) {
        (h->*F)();
      }
    }

  private:
    SignalQueue *queue_;
    std::vector<H*> handlers_;
    bool enqueued_ = false;
  };

}  // namespace sim::core

#endif  // sim_core_signal_h
