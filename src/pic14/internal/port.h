#ifndef sim_pic14_internal_port_h
#define sim_pic14_internal_port_h

#include "../../core/device.h"
#include "core.h"
#include "execution.h"
#include "interrupt.h"
#include "register.h"

#include <array>
#include <cstdint>
#include <string_view>

namespace sim::pic14::internal {

  class PortPin : public sim::core::Pin {
  public:
    PortPin() = default;

    double value() const override;
    double resistance() const override;

    void set_external(double v) override {
      input_ = v >= 0.5;
    }

    void update_output(bool output) {
      output_ = output;
    }

    void update_tris(bool tris) {
      tris_ = tris;
    }

    PortPin(PortPin&&) = default;
    PortPin(const PortPin&) = delete;
    PortPin& operator=(const PortPin&) = delete;

  protected:
    bool input_ = false;
    bool output_ = false;
    bool tris_ = true;
  };

  template<typename Pin>
  class PortBase : public RegisterBackend {
  public:
    enum class Register : uint16_t {
      PORT,
      TRIS,
      NUM_REGISTERS,
    };

    uint8_t read_register(uint16_t addr) override { return read(addr); }
    void write_register(uint16_t addr, uint8_t value) override { write(addr, value); }

    const std::array<Pin, 8>& pins() const { return pins_; }
    std::array<Pin, 8>& pins() { return pins_; }

  protected:
    PortBase(uint8_t index, sim::core::DeviceListener *listener, std::array<Pin, 8> &&pins);

    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t value);

  protected:
    uint8_t index_;
    sim::core::DeviceListener *listener_;
    std::array<Pin, 8> pins_;

    uint8_t input_ = 0;
    uint8_t output_ = 0;
    uint8_t tris_ = 0xFF;
  };

  class Port : public PortBase<PortPin> {
  public:
    Port(uint8_t index, sim::core::DeviceListener *listener);
  };

  class InterruptiblePortPin : public PortPin {
    friend class InterruptiblePort;

  public:
    explicit InterruptiblePortPin(std::function<void(bool)> changed)
      : changed_(changed) {}

    double resistance() const override;

    void set_external(double v) override {
      bool old = input_;
      PortPin::set_external(v);

      if (tris_ && input_ != old) {
        changed_(input_);
      }
    }

  private:
    void update_wpu(bool v) { wpu_ = v; }

  private:
    std::function<void(bool)> changed_;
    bool wpu_ = true;
  };

  class InterruptiblePort : public PortBase<InterruptiblePortPin> {
  public:
    enum class Register : uint16_t {
      PORT = static_cast<uint16_t>(PortBase<InterruptiblePortPin>::Register::PORT),
      TRIS = static_cast<uint16_t>(PortBase<InterruptiblePortPin>::Register::TRIS),
      WPU = static_cast<uint16_t>(PortBase<InterruptiblePortPin>::Register::NUM_REGISTERS),
      IOC,
    };

    InterruptiblePort(uint8_t index,
                      sim::core::DeviceListener *listener,
                      InterruptMux::MaskableIntconEdgeSignal &&change);

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

  private:
    void pin_changed(int index, bool rising);

  private:
    InterruptMux::MaskableIntconEdgeSignal change_;
    uint8_t ioc_ = 0;
    uint8_t wpu_ = 0xFF;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_internal_port_h
