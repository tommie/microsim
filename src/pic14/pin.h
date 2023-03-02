#ifndef sim_pic14_pin_h
#define sim_pic14_pin_h

#include "../core/device.h"

namespace sim::pic14 {

  /// A simple digital input pin.
  class InputPin : public virtual sim::core::Pin {
  public:
    explicit InputPin(std::function<void(bool)> changed)
      : changed_(std::move(changed)) {}

    double value() const override { return value_ ? 1 : 0; }
    double resistance() const override { return 1; }

    void set_external(double v) override {
      bool old = value_;
      value_ = v >= 0.5;

      if (value_ != old) {
        changed_(value_);
      }
    }

    bool external() const { return value_; }

  private:
    std::function<void(bool)> changed_;
    bool value_ = false;
  };

  /// A simple digital output pin.
  class OutputPin : public virtual sim::core::Pin {
  public:
    explicit OutputPin(bool value = false)
      : value_(value) {}

    double value() const override { return value_ ? 1 : 0; }
    double resistance() const override { return 0; }

    void set_external(double v) override {}

    void set_value(bool v) { value_ = v; }

  private:
    bool value_;
  };

  /// A pin that can switch between being a digital input and output.
  class BiDiPin : public virtual InputPin, public virtual OutputPin {
  public:
    BiDiPin(std::function<void(bool)> changed, bool input = true, bool value = false)
      : InputPin(std::move(changed)),
        OutputPin(value),
        input_(input) {}

    double value() const override { return OutputPin::value(); }
    double resistance() const override { return input_ ? InputPin::resistance() : OutputPin::resistance(); }
    void set_external(double v) override { InputPin::set_external(v); }

    bool input() const { return input_; }
    void set_input(bool v) { input_ = v; }

  private:
    bool input_;
  };

}  // namespace sim::pic14

#endif  // sim_pic14_pin_h
