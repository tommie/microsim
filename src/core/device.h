#ifndef sim_core_device_h
#define sim_core_device_h

#include <string>
#include <vector>

#include "../util/status.h"
#include "scheduler.h"
#include "simulation.h"

namespace sim::core {

  class Pin {
  public:
    virtual ~Pin() = default;

    virtual double value() const = 0;
    virtual double resistance() const = 0;

    virtual void set_external(double v) = 0;
  };

  class DeviceListener {
  public:
    enum PinChange {
      UNSPECIFIED_CHANGE,
      VALUE,
      RESISTANCE,
    };

    /// Invoked when the device has encountered an unexpected invalid
    /// internal state, e.g. if the stack pointer wraps around, or
    /// writing to a register that doesn't exist. The device should
    /// continue working as specified, but it is likely not working as
    /// the user intended.
    virtual void invalid_internal_state(const sim::util::Status &status) {}

    /// Invoked when a pin has changed state on the device.
    virtual void pin_changed(Pin *pin, PinChange change) {}
  };

  struct PinDescriptor {
    sim::core::Pin *pin;
    std::string name;
    std::vector<std::string> path;
  };

  class Device : public Schedulable {
  public:
    Device(DeviceListener *listener);
    virtual ~Device() = default;

    virtual const std::vector<PinDescriptor>& pins() const = 0;

  protected:
    DeviceListener& device_listener() { return *listener_; }

  private:
    DeviceListener *listener_;
  };

}  // namespace sim::core

#endif  // sim_core_device_h
