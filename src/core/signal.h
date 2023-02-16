#ifndef sim_core_signal_h
#define sim_core_signal_h

#include <algorithm>
#include <functional>
#include <vector>

namespace sim::core {

  /// A signal is like a bunch of signal wires that can be set by some
  /// module, and responded to by others.
  template<typename T>
  class Signal {
  public:
    using Handler = std::function<void(const T&)>;

    /// Constructs a new signal. The handler is not invoked with the
    /// initial value.
    Signal(Handler &&handler, const T &initial = T())
      : handler_(std::move(handler)),
        value_(initial) {}

    T value() const { return value_; }

    /// Updates the signal. If there was no change, this does
    /// nothing.
    void set(const T &v) {
      if (v != value_) {
        value_ = v;
        handler_(value_);
      }
    }

    Signal(Signal&&) = default;
    Signal& operator =(Signal&&) = default;

    // Signals are "singletons" and attempting a copy is probably an error.
    Signal(const Signal&) = delete;
    Signal& operator =(const Signal&) = delete;

  private:
    Handler handler_;
    T value_;
  };

  /// A set of signals combined into one. This emulates a wire with
  /// multiple sources/sinks (depending on the combiner used.)
  ///
  /// To use it, instantiate it with the appropriate number of
  /// initials, and then give a `make_signal()` to each emitter.
  ///
  /// The combiner must have the property that
  /// `C::combine(x, C::initial) == x` for all `Input x`.
  template<typename C>
  class CombinedSignal : protected Signal<typename C::OutputType> {
  public:
    using OutputType = typename C::OutputType;
    using InputType = typename C::InputType;
    static constexpr InputType initial = C::initial;

    using typename Signal<OutputType>::Handler;

    /// Constructs a new combined signal with using the `handler` as
    /// output. `max_signals` sets the size of the maximum number of
    /// `make_signal` invocations.
    CombinedSignal(typename Signal<OutputType>::Handler &&handler, std::size_t max_signals)
      : Signal<OutputType>(std::move(handler), C::postprocess(initial)) {
      // Because we return pointers to signals, we cannot let
      // std::vector relocate them. At the same time, we don't want
      // `combined()` to have to traverse pointers.
      signals_.reserve(max_signals);
    }

    using Signal<OutputType>::value;

    /// Returns a borrowed pointer to a new signal that is combined to
    /// form the output signal.
    Signal<OutputType>* make_signal(const InputType &value = initial) {
      if (signals_.size() == signals_.capacity())
        std::abort();

      signals_.emplace_back(std::bind_front(&CombinedSignal::handle, this), value);
      return &signals_.back();
    }

    /// Returns a new sub-combined signal. This can be used to chain
    /// different combiners. E.g. tying a NAND to an OR. It has a
    /// static assertion to ensure the initial value of the new signal
    /// is not disrupting `this`; adding the sub-signal should not
    /// change the output value.
    template<typename C2>
    CombinedSignal<C2> make_combined_signal(std::size_t max_signals) {
      static_assert(std::is_assignable_v<InputType&, typename C2::OutputType>, "New output type is not assianble to the existing input type");
      static_assert(C2::postprocess(C2::initial) == initial, "Initial values don't match up; simply attaching this signal would change the value");

      if (signals_.size() == signals_.capacity())
        std::abort();

      return CombinedSignal<C2>(std::bind_front(&Signal<InputType>::set, make_signal()), max_signals);
    }

    // Since `this` is tied up in signals_ handlers, moving is not an
    // option.
    CombinedSignal(CombinedSignal&&) = delete;
    CombinedSignal& operator =(CombinedSignal&&) = delete;

    // Signals are "singletons" and attempting a copy is probably an error.
    CombinedSignal(const CombinedSignal&) = delete;
    CombinedSignal& operator =(const CombinedSignal&) = delete;

  private:
    /// Handles an updated emitter.
    void handle(const OutputType&) {
      OutputType v = combined();
      if (v != Signal<OutputType>::value()) {
        Signal<OutputType>::set(std::move(v));
      }
    }

    OutputType combined() const {
      OutputType ret = initial;
      for (const auto &s : signals_) {
        ret = C::combine(std::move(ret), s.value());
      }
      return C::postprocess(std::move(ret));
    }

  private:
    std::vector<Signal<InputType>> signals_;
  };

  template<typename T>
  struct CombineOr {
    using OutputType = T;
    using InputType = T;
    static constexpr T initial = T();

    static constexpr T combine(const T &acc, const T &value) { return acc | value; }
    static constexpr T postprocess(const T &value) { return value; }
  };

  template<>
  struct CombineOr<bool> {
    using OutputType = bool;
    using InputType = bool;
    static constexpr bool initial = false;

    static constexpr bool combine(const bool &acc, const bool &value) { return acc || value; }
    static constexpr bool postprocess(const bool &value) { return value; }
  };

  template<typename T>
  struct CombineAnd {
    using OutputType = T;
    using InputType = T;
    static constexpr T initial = T();

    static constexpr T combine(const T &acc, const T &value) { return acc & value; }
    static constexpr T postprocess(const T &value) { return value; }
  };

  template<>
  struct CombineAnd<bool> {
    using OutputType = bool;
    using InputType = bool;
    static constexpr bool initial = true;

    static constexpr bool combine(const bool &acc, const bool &value) { return acc && value; }
    static constexpr bool postprocess(const bool &value) { return value; }
  };

  template<typename C, typename Enable = void>
  struct Inverted : C {
    using typename C::OutputType;
    using typename C::InputType;
    static constexpr InputType initial = C::initial;

    static constexpr OutputType postprocess(const InputType &v) { return ~v; }
  };

  template<typename C>
  struct Inverted<C, typename std::enable_if<std::is_same_v<typename C::OutputType, bool>>::type> : C {
    using typename C::OutputType;
    using typename C::InputType;
    static constexpr InputType initial = C::initial;

    static constexpr bool postprocess(const bool &v) { return !v; }
  };

}  // namespace sim::core

#endif  // sim_core_signal_h
