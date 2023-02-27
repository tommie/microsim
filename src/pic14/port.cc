#include "port.h"

#include <cmath>

namespace sim::pic14::internal {

  double PortPin::value() const {
    return tris_ ? NAN : (output_ ? 1 : 0);
  }

  double PortPin::resistance() const {
    return tris_ ? NAN : 0;
  }

  template<typename Pin>
  PortBase<Pin>::PortBase(uint8_t index, sim::core::DeviceListener *listener, std::array<Pin, 8> &&pins)
    : index_(index), listener_(listener), pins_(std::move(pins)) {}

  template<typename Pin>
  inline uint8_t PortBase<Pin>::read(uint16_t addr) {
    if (addr == 0x05 + index_) {
      return (output_ & ~tris_) | (input_ & tris_);
    } else {
      return tris_;
    }
  }

  template<typename Pin>
  inline void PortBase<Pin>::write(uint16_t addr, uint8_t value) {
    if (addr == 0x05 + index_) {
      // PORTx
      for (int i = 0; i < 8; ++i) {
        uint8_t mask = 1u << i;
        bool v = ((value & mask) != 0);
        if (!(tris_ & mask) && v != ((output_ & mask) != 0)) {
          pins_[i].update_output(v);
          listener_->pin_changed(&pins_[i], core::DeviceListener::VALUE);
        }
      }

      output_ = value;
    } else {
      // TRISx
      for (int i = 0; i < 8; ++i) {
        uint8_t mask = 1u << i;
        bool v = (value & mask) != 0;
        if (v != ((tris_ & mask) != 0)) {
          pins_[i].update_output((output_ & mask) != 0);
          pins_[i].update_tris(v);
          listener_->pin_changed(&pins_[i], core::DeviceListener::RESISTANCE);
        }
      }

      tris_ = value;
    }
  }

  template class PortBase<PortPin>;
  template class PortBase<InterruptiblePortPin>;

  double InterruptiblePortPin::resistance() const {
    return wpu_ ? 1 : PortPin::resistance();
  }

  Port::Port(uint8_t index, core::DeviceListener *listener)
    : PortBase(index, listener, {}) {}

  InterruptiblePort::InterruptiblePort(uint8_t index,
                                       sim::core::DeviceListener *listener,
                                       InterruptMux::MaskableIntconEdgeSignal &&change)
    : PortBase(index, listener, std::array<InterruptiblePortPin, 8>{
        InterruptiblePortPin(std::bind_front(&InterruptiblePort::pin_changed, this, 0)),
        InterruptiblePortPin(std::bind_front(&InterruptiblePort::pin_changed, this, 1)),
        InterruptiblePortPin(std::bind_front(&InterruptiblePort::pin_changed, this, 2)),
        InterruptiblePortPin(std::bind_front(&InterruptiblePort::pin_changed, this, 3)),
        InterruptiblePortPin(std::bind_front(&InterruptiblePort::pin_changed, this, 4)),
        InterruptiblePortPin(std::bind_front(&InterruptiblePort::pin_changed, this, 5)),
        InterruptiblePortPin(std::bind_front(&InterruptiblePort::pin_changed, this, 6)),
        InterruptiblePortPin(std::bind_front(&InterruptiblePort::pin_changed, this, 7)),
      }),
      change_(std::move(change)) {}

  uint8_t InterruptiblePort::read_register(uint16_t addr) {
    if (addr == 0x95) {
      return wpu_;
    } else if (addr == 0x96) {
      return ioc_;
    } else {
      return PortBase::read(addr);
    }
  }

  void InterruptiblePort::write_register(uint16_t addr, uint8_t value) {
    if (addr == 0x95) {
      // WPUx

      for (int i = 0; i < 8; ++i) {
        uint8_t mask = 1u << i;
        bool wpu = (value & mask) != 0;
        if (wpu != ((wpu_ & mask) != 0)) {
          pins_[i].update_wpu(wpu);
          if (tris_ & mask) {
            listener_->pin_changed(&pins_[i], core::DeviceListener::RESISTANCE);
          }
        }
      }
      wpu_ = value;
    } else if (addr == 0x96) {
      ioc_ = value;
    } else {
      PortBase::write(addr, value);
    }
  }

  void InterruptiblePort::pin_changed(int index, bool rising) {
    input_ = (input_ & ~(1u << index)) | (rising ? 1u << index : 0u);

    if (ioc_ & (1u << index)) {
      change_.raise();
    }
  }

}  // namespace sim::pic14::internal
