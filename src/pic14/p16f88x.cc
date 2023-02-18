#include "p16f88x.h"

namespace sim::pic14::internal {

  std::u16string build_addrmap(uint16_t size) {
    std::u16string addrmap(size, 0);

    for (uint16_t i = 0; i < addrmap.size(); ++i) {
      addrmap[i] = i;
    }
    addrmap[0x80] = addrmap[0x100] = addrmap[0x180] = 0x00;
    addrmap[0x82] = addrmap[0x102] = addrmap[0x182] = 0x02;
    addrmap[0x83] = addrmap[0x103] = addrmap[0x183] = 0x03;
    addrmap[0x84] = addrmap[0x104] = addrmap[0x184] = 0x04;
    addrmap[0x106] = 0x06;
    addrmap[0x186] = 0x86;
    addrmap[0x8A] = addrmap[0x10A] = addrmap[0x18A] = 0x0A;
    addrmap[0x8B] = addrmap[0x10B] = addrmap[0x18B] = 0x0B;
    for (uint16_t i = 0x70; i < 0x80; ++i) {
      addrmap[0x80 + i] = addrmap[0x100 + i] = addrmap[0x180 + i] = i;
    }

    return addrmap;
  }

  template<uint16_t ProgSize, uint16_t EEDataSize, int NumPorts>
  const std::u16string_view P16F88X<ProgSize, EEDataSize, NumPorts>::address_map() {
    static const std::u16string addrmap = build_addrmap(P16F88X::FILE_BUS_SIZE);
    return addrmap;
  }

  template<uint16_t ProgSize, uint16_t EEDataSize, int NumPorts>
  P16F88X<ProgSize, EEDataSize, NumPorts>::P16F88X(core::DeviceListener *listener, sim::core::Clock *fosc)
    : Device(listener),
      fosc_(fosc),
      reset_([this](bool raised) {
        if (!raised) {
          reset();
        }
      }, 3),
      mclr_(reset_.make_signal()),
      por_(reset_.make_signal(true)),
      core_([this]() {}),
      interrupt_mux_([this]() {
        executor_.interrupted();
      }),
      nv_(NonVolatile::Config{PROG_SIZE, CONFIG_SIZE, EEDATA_SIZE}, reset_.make_signal()),
      executor_(listener,
                fosc_,
                &nv_,
                build_data_bus(),
                &interrupt_mux_),
      ports_{
        internal::Port(0, listener),
        // PORTB is handled separately.
        internal::Port(2, listener),
        internal::Port(3, listener),
        internal::Port(4, listener),
      },
      portb_(1, listener, interrupt_mux_.make_maskable_edge_signal_intcon(3, 0), interrupt_mux_.make_maskable_edge_signal_intcon(4, 1), core_.option_reg()),
      pin_descrs_(build_pin_descrs()),
      scheduler_({
        &executor_,
      }, this) {}

  template<uint16_t ProgSize, uint16_t EEDataSize, int NumPorts>
  internal::DataBus P16F88X<ProgSize, EEDataSize, NumPorts>::build_data_bus() {
    std::vector<internal::RegisterBackend*> backs;
    std::u8string backmap(FILE_BUS_SIZE, 0xFF);

    // When this code runs, the backends have not yet been initialized. Only taking pointers.
    backmap[0x05 + 0] = backmap[0x85 + 0] = backs.size(); backs.push_back(&ports_[0]);
    backmap[0x05 + 1] = backmap[0x85 + 1] = backmap[0x95] = backmap[0x96] = backs.size(); backs.push_back(&portb_);
    backmap[0x05 + 2] = backmap[0x85 + 2] = backs.size(); backs.push_back(&ports_[1]);
    backmap[0x05 + 3] = backmap[0x85 + 3] = backs.size(); backs.push_back(&ports_[2]);
    backmap[0x03] = backs.size(); backs.push_back(&executor_);
    backmap[0x0B] = backmap[0x0C] = backmap[0x0D] = backmap[0x8C] = backmap[0x8D] = backs.size(); backs.push_back(&interrupt_mux_);
    backmap[0x81] = backs.size(); backs.push_back(&core_);

    return internal::DataBus(FILE_BUS_SIZE, 0, std::move(backs), std::move(backmap), address_map());
  }

  template<uint16_t ProgSize, uint16_t EEDataSize, int NumPorts>
  std::vector<sim::core::PinDescriptor> P16F88X<ProgSize, EEDataSize, NumPorts>::build_pin_descrs() {
    std::vector<sim::core::PinDescriptor> descrs;

    std::string name_buf("RA0");
    for (auto &port : ports_) {
      for (auto &pin : port.pins()) {
        descrs.push_back({.pin = &pin, .name = name_buf});
        ++name_buf[2];
      }
      ++name_buf[1];
      if (name_buf[1] == 'B') ++name_buf[1];  // PORTB is handled separately.
      name_buf[2] = '0';
    }

    name_buf = "RB0";
    for (auto &pin : portb_.pins()) {
      descrs.push_back({.pin = &pin, .name = name_buf});
      ++name_buf[2];
    }

    return descrs;
  }

  template<uint16_t ProgSize, uint16_t EEDataSize, int NumPorts>
  void P16F88X<ProgSize, EEDataSize, NumPorts>::reset() {
    core_.reset();
    interrupt_mux_.reset();
    executor_.reset();
  }

  template<uint16_t ProgSize, uint16_t EEDataSize, int NumPorts>
  sim::core::Advancement P16F88X<ProgSize, EEDataSize, NumPorts>::advance_to(const sim::core::SimulationLimit &limit) {
    if (por_->value()) {
      por_->set(false);
    }

    if (reset_.value()) return {};

    return scheduler_.advance_to(limit);
  }

  // Template instantiations for specific processors.
  template class P16F88X<4096, 256, 5>;
  template class P16F88X<8192, 256, 5>;

} // namespace sim::pic14::internal
