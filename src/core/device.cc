#include "device.h"

namespace sim::core {

  namespace {

    DeviceListener* canonical_listener(DeviceListener *listener) {
      static DeviceListener default_listener;

      return listener ? listener : &default_listener;
    }

  }

  Device::Device(DeviceListener *listener)
    : listener_(canonical_listener(listener)) {}

}  // namespace sim::core
