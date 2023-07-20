#include "p16f88x.h"

#include <sstream>

namespace sim::pic14::internal {

  std::vector<uint16_t> build_addrmap(uint16_t size) {
    std::vector<uint16_t> addrmap(size, 0xFFFF);

    addrmap[0x82] = addrmap[0x102] = addrmap[0x182] = 0x02;  // PCL
    addrmap[0x8A] = addrmap[0x10A] = addrmap[0x18A] = 0x0A;  // PCLATH

    for (uint16_t i = 0x70; i < 0x80; ++i) {
      addrmap[0x80 + i] = addrmap[0x100 + i] = addrmap[0x180 + i] = i;
    }

    return addrmap;
  }

  template<typename Config>
  const std::vector<uint16_t>& P16F88X<Config>::address_map() {
    static const std::vector<uint16_t> addrmap = build_addrmap(P16F88X::FILE_BUS_SIZE);
    return addrmap;
  }

  template<typename Config>
  P16F88X<Config>::P16F88X(sim::core::DeviceListener *listener, sim::core::Clock *extosc)
    : Device(listener),
      nv_(NonVolatile::Config{PROG_SIZE, CONFIG_SIZE, EEDATA_SIZE}),
      core_(extosc, &nv_, [this](bool raised) {
        if (!raised) {
          reset();
        }
      }, [this]() {
        wdt_.option_reg_updated();
        timer0_.option_reg_updated();
      }, [this]() {
        ulpwu_.pcon_updated();
      }, [this]() {
        executor_.fosc_changed();
        timer0_.fosc_changed();
        adc_.fosc_changed();
        eusart_.fosc_changed();
      }),
      interrupt_mux_([this]() {
        core_.sleep_signal()->set(false);
        executor_.interrupted();
      }),
      executor_(listener,
                core_.fosc(),
                &nv_,
                build_data_bus(),
                core_.sleep_signal(),
                std::bind_front(&WatchDogTimer::clear, &wdt_),
                &interrupt_mux_),
      wdt_(core_.lfintosc(),
           core_.wdt_reset_signal(),
           core_.option_reg(),
           &core_.config1(),
           executor_.status_reg()),
      timer0_(core_.fosc(), interrupt_mux_.make_maskable_edge_signal_intcon(5, 2), core_.option_reg()),
      ports_{
        internal::Port(0, listener),
        // PORTB is handled separately.
        internal::Port(2, listener),
        internal::Port(3, listener),
        internal::Port(4, listener),
      },
      portb_(1, listener, interrupt_mux_.make_maskable_edge_signal_intcon(3, 0)),
      extint_(core_.option_reg(), interrupt_mux_.make_maskable_edge_signal_intcon(4, 1)),
      adc_(core_.fosc(), interrupt_mux_.make_maskable_edge_signal_peripheral(6)),
      ulpwu_(listener, core_.pcon_reg(), interrupt_mux_.make_maskable_edge_signal_peripheral(10)),
      eprom_(&nv_, core_.lfintosc(), &core_.config2(), interrupt_mux_.make_maskable_edge_signal_peripheral(12), &executor_),
      eusart_(listener, core_.fosc(),  interrupt_mux_.make_maskable_level_signal_peripheral(5),  interrupt_mux_.make_maskable_level_signal_peripheral(4)),
      pin_descrs_(build_pin_descrs()),
      scheduler_({
        &core_,
        &wdt_,
        &executor_,
        &timer0_,
        &adc_,
        &eprom_,
        &eusart_,
      }, this) {}

  template<typename Config>
  internal::DataBus P16F88X<Config>::build_data_bus() {
    DataBusBuilder<FILE_BUS_SIZE> builder(address_map());

    // When this code runs, the backends have not yet been initialized. Only taking pointers.
    builder.indirect({0x00, 0x80, 0x100, 0x180}, {0x04, 0x84, 0x104, 0x184});

    builder.backend(&timer0_, {{Timer0::Register::TMR0, {0x01, 0x101}}});
    builder.backend(&executor_, {{Executor::Register::STATUS, {0x03, 0x83, 0x103, 0x183}}});
    builder.backend(&ports_[0], {{Port::Register::PORT, {0x05 + 0}}, {Port::Register::TRIS, {0x85 + 0}}});
    builder.backend(&portb_, {{InterruptiblePort::Register::PORT, {0x05 + 1, 0x105 + 1}}, {InterruptiblePort::Register::TRIS, {0x85 + 1, 0x185 + 1}}, {InterruptiblePort::Register::WPU, {0x95}}, {InterruptiblePort::Register::IOC, {0x96}}});
    builder.backend(&ports_[1], {{Port::Register::PORT, {0x05 + 2}}, {Port::Register::TRIS, {0x85 + 2}}});
    builder.backend(&ports_[2], {{Port::Register::PORT, {0x05 + 3}}, {Port::Register::TRIS, {0x85 + 3}}});
    builder.backend(&interrupt_mux_, {{InterruptMux::Register::INTCON, {0x0B, 0x8B, 0x10B, 0x18B}}, {InterruptMux::Register::PIR1, {0x0C}}, {InterruptMux::Register::PIR2, {0x0D}}, {InterruptMux::Register::PIE1, {0x8C}}, {InterruptMux::Register::PIE2, {0x8D}}});
    builder.backend(&eusart_, {{EUSART::Register::RCSTA, {0x18}}, {EUSART::Register::TXREG, {0x19}}, {EUSART::Register::RCREG, {0x1A}}, {EUSART::Register::TXSTA, {0x98}}, {EUSART::Register::SPBRG, {0x99}}, {EUSART::Register::SPBRGH, {0x9A}}, {EUSART::Register::BAUDCTL, {0x187}}});
    builder.backend(&adc_, {{ADConverter::Register::ADRESH, {0x1E}}, {ADConverter::Register::ADCON0, {0x1F}}, {ADConverter::Register::ADRESL, {0x9E}}, {ADConverter::Register::ADCON1, {0x9F}}});
    builder.backend(&core_, {{Core::Register::OPTION, {0x81, 0x181}}, {Core::Register::PCON, {0x8E}}, {Core::Register::OSCCON, {0x8F}}});
    builder.backend(&wdt_, {{WatchDogTimer::Register::WDTCON, {0x105}}});
    builder.backend(&eprom_, {{EPROMType::Register::EEDAT, {0x10C}}, {EPROMType::Register::EEADR, {0x10D}}, {EPROMType::Register::EEDATH, {0x10E}}, {EPROMType::Register::EEADRH, {0x10F}}, {EPROMType::Register::EECON1, {0x18C}}, {EPROMType::Register::EECON2, {0x18D}}});

    return builder.build(0);
  }

  template<typename Config>
  std::vector<sim::core::PinDescriptor> P16F88X<Config>::build_pin_descrs() {
    std::vector<sim::core::PinDescriptor> descrs;

    descrs.push_back({.pin = core_.mclr_pin(), .name = "MCLR"});
    descrs.push_back({.pin = extint_.pin(), .name = "INT"});
    descrs.push_back({.pin = ulpwu_.pin(), .name = "ULPWU"});

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

    descrs.push_back({.pin = &adc_.avdd_pin(), .name = "AVDD"});
    int i = 0;
    for (auto &pin : adc_.input_pins()) {
      std::ostringstream ss;
      ss << "AN" << i;
      ++i;
      descrs.push_back({.pin = &pin, .name = ss.str()});
    }

    descrs.push_back({.pin = &eusart_.rc_pin(), .name = "RC"});
    descrs.push_back({.pin = &eusart_.tx_pin(), .name = "TX"});
    descrs.push_back({.pin = &eusart_.ck_pin(), .name = "CK"});
    descrs.push_back({.pin = &eusart_.dt_pin(), .name = "DT"});

    return descrs;
  }

  template<typename Config>
  void P16F88X<Config>::reset() {
    core_.reset();
    wdt_.reset();
    interrupt_mux_.reset();
    executor_.reset();
    adc_.reset();
    eprom_.reset();
    eusart_.reset();
  }

  template<typename Config>
  std::vector<sim::core::Clock*> P16F88X<Config>::clock_sources() {
    std::vector<sim::core::Clock*> sources = core_.clock_sources();
    auto adc_sources = adc_.clock_sources();

    sources.insert(sources.end(), adc_sources.begin(), adc_sources.end());

    return sources;
  }

  template<typename Config>
  sim::core::Advancement P16F88X<Config>::advance_to(const sim::core::AdvancementLimit &limit) {
    if (core_.in_reset()) {
      return core_.advance_to(limit);
    }

    return scheduler_.advance_to(limit);
  }

  // Template instantiations for specific processors.
  template class P16F88X<P16F88XConfig<4096, 256, 5>>;
  template class P16F88X<P16F88XConfig<8192, 256, 5>>;

} // namespace sim::pic14::internal
