#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <unordered_map>
#include <sstream>

#include "../core/ihex.h"
#include "../core/trace.h"
#include "../testing/nrz.h"
#include "../testing/spi.h"
#include "../testing/testing.h"
#include "../util/status.h"
#include "p16f88x.h"


template<typename Proc>
sim::util::Status load_testdata_ihex(Proc *proc, std::string_view path) {
  auto icsp = proc->enter_icsp();
  std::ifstream ihex(static_cast<std::string>(path));

  if (!ihex) {
    return sim::util::Status(std::error_code(errno, std::system_category()), path);
  }

  return sim::core::load_ihex(ihex, std::bind_front(&sim::pic14::ICSP::load_program, &icsp));
}

class NRZReceiver {
public:
  NRZReceiver(const sim::core::SimulationClock *sim_clock, sim::core::Pin *pin, unsigned int num_data_bits, sim::core::Duration bit_duration)
    : sim_clock_(sim_clock), rcvr_(num_data_bits, bit_duration.count()), pin_(pin) {}

  std::u16string_view received() const { return rcvr_.received(); }

  void pin_changed(sim::core::Pin *pin) {
    if (pin != pin_) return;

    rcvr_.signal_changed(sim_clock_->now().time_since_epoch().count(), pin->value());
  }

private:
  const sim::core::SimulationClock *sim_clock_;
  sim::testing::NRZReceiver rcvr_;
  sim::core::Pin *pin_;
};

class NRZTransmitter : public sim::core::SimulationObject {
public:
  NRZTransmitter(const sim::core::SimulationClock *sim_clock, sim::core::Pin *pin)
    : sim_clock_(sim_clock), pin_(pin) {
    pin->set_external(true);
  }

  bool empty() const { return !tmtr_ || tmtr_->empty(); }

  void transmit(std::u16string_view data, unsigned int num_data_bits, sim::core::Duration bit_duration, sim::core::Duration delay = {}) {
    if (tmtr_) std::abort();  // Already transmitting.

    bit_duration_ = bit_duration;
    delay_ = delay;
    tmtr_.emplace(num_data_bits, 1, data);
    schedule_immediately();
  }

  sim::core::Advancement advance_to(const sim::core::AdvancementLimit&) override {
    if (!tmtr_) return {};

    if (sim::core::is_never(tmtr_start_)) {
      tmtr_start_ = sim_clock_->now() + delay_;
      return { .next_time = tmtr_start_ };
    }

    auto [next_tick, v] = tmtr_->next_signal_change();
    pin_->set_external(v);

    if (tmtr_->empty()) {
      tmtr_start_ = sim::core::SimulationClock::NEVER;
      tmtr_.reset();
    }

    if (next_tick == 0)
      return {};

    return {
      .next_time = tmtr_start_ + next_tick * bit_duration_,
    };
  }

private:
  const sim::core::SimulationClock *sim_clock_;
  sim::core::Pin *pin_;

  sim::core::Duration bit_duration_;
  sim::core::Duration delay_;
  sim::core::TimePoint tmtr_start_ = sim::core::SimulationClock::NEVER;
  std::optional<sim::testing::NRZTransmitter> tmtr_;
};

class SPISlave {
public:
  SPISlave(sim::core::Pin *ck_pin, sim::core::Pin *dt_pin, unsigned int num_data_bits, std::u16string_view data = {})
    : spi_(num_data_bits, data),
      ck_pin_(ck_pin), dt_pin_(dt_pin) {}

  std::u16string_view data() const { return spi_.data(); }

  void pin_changed(sim::core::Pin *pin) {
    if (pin != ck_pin_ && pin != dt_pin_) return;

    double dt = spi_.signal_changed(ck_pin_->resistance() >= 0.5 ? 0.5 : ck_pin_->value(),
                                    dt_pin_->resistance() >= 0.5 ? 0.5 : dt_pin_->value());
    if (dt < 0.4 || dt >= 0.6) {
      dt_pin_->set_external(dt);
    }
  }

private:
  sim::testing::SPISlave spi_;
  sim::core::Pin *ck_pin_;
  sim::core::Pin *dt_pin_;
};

class SPIMaster : public sim::core::SimulationObject {
public:
  SPIMaster(const sim::core::SimulationClock *sim_clock, sim::core::Pin *ck_pin, sim::core::Pin *di_pin, sim::core::Pin *do_pin)
    : sim_clock_(sim_clock),
      ck_pin_(ck_pin), di_pin_(di_pin), do_pin_(do_pin) {
  }

  bool empty() const { return spi_->empty(); }
  std::u16string_view data() const { return spi_->data(); }

  void transact(std::u16string_view data, unsigned int num_data_bits, sim::core::Duration bit_duration, sim::core::Duration delay = {}) {
    ck_pin_->set_external(false);
    do_pin_->set_external(false);

    spi_ = sim::testing::SPIMaster(num_data_bits, bit_duration.count(), data);
    start_time_ = sim_clock_->now() + delay;
    schedule_immediately();
  }

  sim::core::Advancement advance_to(const sim::core::AdvancementLimit&) override {
    if (!spi_) return {};

    if (sim_clock_->now() < start_time_)
      return { .next_time = start_time_ };

    auto [next_tick, ck, dt] = spi_->next_signal_change((sim_clock_->now() - start_time_).count(),
                                                        di_pin_->resistance() >= 0.5 ? 0.5 : di_pin_->value());
    ck_pin_->set_external(ck);
    do_pin_->set_external(dt);

    if (next_tick == 0) {
      spi_.reset();
      return {};
    }

    return {
      .next_time = start_time_ + sim::core::Duration(next_tick),
    };
  }

private:
  const sim::core::SimulationClock *sim_clock_;
  sim::core::Pin *ck_pin_;
  sim::core::Pin *di_pin_;
  sim::core::Pin *do_pin_;
  std::optional<sim::testing::SPIMaster> spi_;
  sim::core::TimePoint start_time_ = sim::core::SimulationClock::NEVER;
};

class DeviceListener : public sim::core::DeviceListener {
public:
  void pin_changed(sim::core::Pin *pin, PinChange change) override {
    std::cout << "  " << std::setw(6) << sim_clock->now().time_since_epoch().count() << " pin_changed " << pin_descrs[pin].name << " " << (change == PinChange::VALUE ? "value" : "res") << " " << pin->value() << ":" << (pin->resistance() < 0.5 ? "o" : "i") << std::endl;

    if (nrz_rcvr_) nrz_rcvr_->pin_changed(pin);
    if (spi_slave_) spi_slave_->pin_changed(pin);
  }

  std::unordered_map<sim::core::Pin*, sim::core::PinDescriptor> pin_descrs;
  const sim::core::SimulationClock *sim_clock;

  NRZReceiver *nrz_rcvr_ = nullptr;
  SPISlave *spi_slave_ = nullptr;
};

void dump_trace_buffer(sim::util::TraceBuffer &buf = sim::core::trace_buffer()) {
  while (!buf.empty()) {
    buf.top().visit<sim::core::ClockAdvancedTraceEntry,
                    sim::core::SchedulableTraceEntry,
                    sim::core::SimulationClockAdvancedTraceEntry,
                    sim::core::SimulatorTraceEntry,
                    sim::pic14::ADConversionDoneTraceEntry,
                    sim::pic14::EUSARTDataTraceEntry,
                    sim::pic14::ExecutedTraceEntry,
                    sim::pic14::WatchDogClearedTraceEntry,
                    sim::pic14::WatchDogTimedOutTraceEntry,
                    sim::pic14::WroteEEDATATraceEntry,
                    sim::pic14::WroteProgramFlashTraceEntry>([](const auto &e) {
      using T = std::decay_t<decltype(e)>;
      if constexpr (std::is_same_v<T, sim::core::ClockAdvancedTraceEntry>) {
        std::cout << "Clock       " << e.clock() << " now=" << e.now().time_since_epoch().count() << std::endl;
      } else if constexpr (std::is_same_v<T, sim::core::SchedulableTraceEntry>) {
        std::cout << "Schedulable " << e.schedulable() << std::endl;
      } else if constexpr (std::is_same_v<T, sim::core::SimulationClockAdvancedTraceEntry>) {
        std::cout << "SimClock    now=" << e.now().time_since_epoch().count() << std::endl;
      } else if constexpr (std::is_same_v<T, sim::core::SimulatorTraceEntry>) {
        std::cout << "Simulator   " << e.simulator() << std::endl;
      } else if constexpr (std::is_same_v<T, sim::pic14::ADConversionDoneTraceEntry>) {
        std::cout << "ADC Done" << std::endl;
      } else if constexpr (std::is_same_v<T, sim::pic14::EUSARTDataTraceEntry>) {
        std::cout << "EUSART      0x" << std::hex << e.data() << std::dec << " ";

        switch (e.mode()) {
        case sim::pic14::EUSARTDataTraceEntry::Mode::ASYNC_TRANSMIT: std::cout << "A-Tx"; break;
        case sim::pic14::EUSARTDataTraceEntry::Mode::ASYNC_RECEIVED: std::cout << "A-Rc"; break;
        case sim::pic14::EUSARTDataTraceEntry::Mode::SYNC_MASTER_TRANSMIT: std::cout << "M-Tx"; break;
        case sim::pic14::EUSARTDataTraceEntry::Mode::SYNC_MASTER_RECEIVED: std::cout << "M-Rc"; break;
        case sim::pic14::EUSARTDataTraceEntry::Mode::SYNC_SLAVE_TRANSMIT: std::cout << "S-Tx"; break;
        case sim::pic14::EUSARTDataTraceEntry::Mode::SYNC_SLAVE_RECEIVED: std::cout << "S-Rc"; break;
        }

        if (e.num_bits() != 8) {
          std::cout << ", " << e.num_bits() << " bits";
        }

        if (e.ferr()) {
          std::cout << ", framing error";
        }
        std::cout << std::endl;
      } else if constexpr (std::is_same_v<T, sim::pic14::ExecutedTraceEntry>) {
        std::cout << "Executed    0x" << std::hex << e.addr() << std::dec << std::endl;
      } else if constexpr (std::is_same_v<T, sim::pic14::WatchDogClearedTraceEntry>) {
        std::cout << "WDT Clear" << std::endl;
      } else if constexpr (std::is_same_v<T, sim::pic14::WatchDogTimedOutTraceEntry>) {
        std::cout << "WDT Reset" << std::endl;
      } else if constexpr (std::is_same_v<T, sim::pic14::WroteEEDATATraceEntry>) {
        std::cout << "WR EEDATA   0x" << std::hex << (unsigned long) e.addr() << std::dec << std::endl;
      } else if constexpr (std::is_same_v<T, sim::pic14::WroteProgramFlashTraceEntry>) {
        std::cout << "WR Program  0x" << std::hex << (unsigned long) e.addr() << std::dec << std::endl;
      } else {
        std::cout << "Other(" << e.kind() << ")" << std::endl;
      }
    });

    buf.pop();
  }
}

template<typename Proc>
class ProcessorTestCase : public sim::testing::TestCase {
  static std::unordered_map<std::string, sim::core::Pin*> build_pin_descrs(const std::vector<sim::core::PinDescriptor> &pins) {
    std::unordered_map<std::string, sim::core::Pin*> descrs;
    for (const auto &descr : pins) {
      descrs[descr.name] = descr.pin;
    }
    return descrs;
  }

public:
  ProcessorTestCase(std::string_view firmware)
    : firmware_(firmware), fosc_(std::chrono::seconds(1)), proc(&listener_, &fosc_),
      pins(build_pin_descrs(proc.pins())),
      tmtr(&sim_clock(), pins["RC"]),
      spi_master(&sim_clock(), pins["CK"], pins["DT"], pins["DT"]),
      sim_(sim::core::SimulationContext({&fosc_}, {&proc, &tmtr, &spi_master}).make_simulator()) {
    listener_.sim_clock = &sim_.sim_clock();
    for (const auto &descr : proc.pins()) {
      listener_.pin_descrs[descr.pin] = descr;
    }
  }

  const sim::core::SimulationClock& sim_clock() const { return sim_.sim_clock(); }

protected:
  void advance_until_sleep() {
    if (proc.is_sleeping())
      advance_during_sleep();

    advance_while([this]() {
        return !proc.is_sleeping();
    });
  }

  void advance_during_sleep() {
    advance_while([this]() {
        return proc.is_sleeping();
    });
  }

  void advance_while(std::function<bool()> pred) {
    sim::core::AdvancementLimit limit = {
      .can_advance_to = [&pred](sim::core::TimePoint) {
        return pred();
      },
    };

    while (pred()) {
      auto next_time = sim_.advance_to(limit).next_time;

      dump_trace_buffer();

      if (!pred()) {
        return;
      }

      if (sim::core::is_never(next_time)) {
        throw std::runtime_error("the simulation is stalled and cannot advance");
      }
    }
  }

  template<typename T>
  void advance_until_empty(const T &subject) {
    advance_while([&subject]() {
      return !subject.empty();
    });
  }

  void set_nrz_receiver(NRZReceiver *rcvr) {
    listener_.nrz_rcvr_ = rcvr;
  }

  void set_spi_slave(SPISlave *slave) {
    listener_.spi_slave_ = slave;
  }

  sim::util::Status setUp() override {
    if (auto status = load_testdata_ihex(&proc, firmware_); !status.ok()) {
      return status;
    }
    return sim::util::Status();
  }

private:
  std::string_view firmware_;
  DeviceListener listener_;
  sim::core::Clock fosc_;

protected:
  Proc proc;
  std::unordered_map<std::string, sim::core::Pin*> pins;
  NRZTransmitter tmtr;
  SPIMaster spi_master;

private:
  sim::core::Simulator sim_;
};

#define PROCESSOR_TEST(Name, Proc, Firmware) \
  SIM_TEST_BASE(Name, ProcessorTestCase<sim::pic14::Proc>, Firmware)


PROCESSOR_TEST(PortTest, P16F887, "testdata/port.hex") {
  pins["RB0"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1");
}

PROCESSOR_TEST(ExtInterruptTest, P16F887, "testdata/extint.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0 before pin change");

  pins["INT"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after pin change");
}

PROCESSOR_TEST(UltraLowPowerWakeUpTest, P16F887, "testdata/ulpwu.hex") {
  pins["ULPWU"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0 before pin change");
  if (pins["ULPWU"]->resistance() > 0.55) fail("ULPWU resistance should be 0.5 before pin change");

  pins["ULPWU"]->set_external(0);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after pin change");
}

PROCESSOR_TEST(GlobalInterruptTest, P16F887, "testdata/gie.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0 before pin change");

  pins["INT"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after pin change");
}

PROCESSOR_TEST(Timer0Test, P16F887, "testdata/timer0.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1");
}

PROCESSOR_TEST(Timer0InterruptTest, P16F887, "testdata/t0if.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1");
}

PROCESSOR_TEST(WatchdogTest, P16F887, "testdata/watchdog.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1");
}

PROCESSOR_TEST(EEDATATest, P16F887, "testdata/eedata.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0");

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after read");
}

PROCESSOR_TEST(ProgramFlashTest, P16F887, "testdata/eeprog.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0");

  pins["INT"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after read");
}

PROCESSOR_TEST(ADConverterTest, P16F887, "testdata/adc.hex") {
  pins["AN1"]->set_external(0.5);

  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0");

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after ADC");
}

PROCESSOR_TEST(EUSARTAsyncTransmitTest, P16F887, "testdata/eusart_async_tx.hex") {
  NRZReceiver rcvr(&sim_clock(), pins["TX"], 8, sim::core::Microseconds(16));
  set_nrz_receiver(&rcvr);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after transmit");

  if (rcvr.received().empty()) fail("No byte transmitted");
  if (rcvr.received()[0] != 42) fail("Byte transmitted should be 42");
}

PROCESSOR_TEST(EUSARTAsyncReceiveTest, P16F887, "testdata/eusart_async_rc.hex") {
  tmtr.transmit(std::u16string{42}, 8, sim::core::Microseconds(16), sim::core::Microseconds(16));

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after transmit");
}

PROCESSOR_TEST(EUSARTAutoBaudRateDetectTest, P16F887, "testdata/eusart_abd.hex") {
  tmtr.transmit(std::u16string{0x55}, 8, sim::core::Microseconds(16), sim::core::Microseconds(16));

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after transmit");
}

PROCESSOR_TEST(EUSARTWakeUpTest, P16F887, "testdata/eusart_wakeup.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0");

  tmtr.transmit(std::u16string{0}, 8, sim::core::Microseconds(16));

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after transmit");
}

PROCESSOR_TEST(EUSARTMasterTransmitTest, P16F887, "testdata/eusart_master_tx.hex") {
  SPISlave spi(pins["CK"], pins["DT"], 8);
  set_spi_slave(&spi);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after transmit");

  if (spi.data().empty()) fail("No byte transmitted");
  if (spi.data()[0] != 42) fail("Byte transmitted should be 42");
}

PROCESSOR_TEST(EUSARTMasterReceiveTest, P16F887, "testdata/eusart_master_rc.hex") {
  SPISlave spi(pins["CK"], pins["DT"], 8, std::u16string{42});
  set_spi_slave(&spi);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after receive");
}

PROCESSOR_TEST(EUSARTSlaveTransmitTest, P16F887, "testdata/eusart_slave_tx.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0");

  spi_master.transact(std::u16string{0x8000, 0x8000}, 8, sim::core::Microseconds(16));

  advance_until_empty(spi_master);

  if (spi_master.data()[0] != 42) fail("Byte transmitted should be 42");
  if (spi_master.data()[1] != 43) fail("Byte transmitted should be 43");

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after transmit");
}

PROCESSOR_TEST(EUSARTSlaveReceiveTest, P16F887, "testdata/eusart_slave_rc.hex") {
  spi_master.transact(std::u16string{42, 43}, 8, sim::core::Microseconds(16), sim::core::Microseconds(16));

  advance_until_empty(spi_master);
  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after transmit");
}

int main() {
  return sim::testing::TestSuite::global().run();
}
