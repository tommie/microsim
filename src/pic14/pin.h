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
      value_ = v;

      if (value_ != old) {
        changed_(value_);
      }
    }

  private:
    std::function<void(bool)> changed_;
    bool value_ = false;
  };

}  // namespace sim::pic14

#endif  // sim_pic14_pin_h
