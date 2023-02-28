#ifndef sim_pic14_pin_h
#define sim_pic14_pin_h

#include "../core/device.h"

namespace sim::pic14 {

  /// A simple digital input pin.
  class InputPin : public sim::core::Pin {
  public:
    explicit InputPin(std::function<void(bool)> changed)
      : changed_(changed) {}

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
  class OutputPin : public sim::core::Pin {
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

}  // namespace sim::pic14

#endif  // sim_pic14_pin_h
