#include <array>
#include <fstream>
#include <iostream>
#include <list>
#include <unordered_map>
#include <sstream>

#include "../core/ihex.h"
#include "../core/trace.h"
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

class DeviceListener : public sim::core::DeviceListener {
public:
  void pin_changed(sim::core::Pin* pin, PinChange change) override {
    std::cout << "  pin_changed " << pin_descrs[pin].name << " " << (change == PinChange::VALUE ? "value" : "res") << " " << pin->value() << ":" << (pin->resistance() < 0.5 ? "o" : "i") << std::endl;
  }

  std::unordered_map<sim::core::Pin*, sim::core::PinDescriptor> pin_descrs;
};

void dump_trace_buffer(sim::util::TraceBuffer &buf = sim::core::trace_buffer()) {
  while (!buf.empty()) {
    buf.top().visit<sim::core::ClockAdvancedTraceEntry,
                    sim::core::SchedulableTraceEntry,
                    sim::core::SimulationClockAdvancedTraceEntry,
                    sim::core::SimulatorTraceEntry,
                    sim::pic14::ADConversionDoneTraceEntry,
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
public:
  ProcessorTestCase(std::string_view firmware)
    : firmware_(firmware), fosc_(std::chrono::seconds(1)), proc(&listener_, &fosc_),
      sim_(sim::core::SimulationContext({&fosc_}, {&proc}).make_simulator()) {
    for (const auto &descr : proc.pins()) {
      listener_.pin_descrs[descr.pin] = descr;
      pins[descr.name] = descr.pin;
    }
  }

protected:
  void advance_until_sleep() {
    if (proc.is_sleeping())
      advance_during_sleep();

    sim::core::AdvancementLimit limit = {
      .cond = [this]() {
        return !proc.is_sleeping();
      },
    };

    do {
      sim_.advance_to(limit);
      dump_trace_buffer();
    } while (!proc.is_sleeping());
  }

  void advance_during_sleep() {
    sim::core::AdvancementLimit limit = {
      .cond = [this]() {
        return proc.is_sleeping();
      },
    };

    while (proc.is_sleeping()) {
      if (sim::core::is_never(sim_.advance_to(limit).next_time))
        std::abort();

      dump_trace_buffer();
    }
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

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0");

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

int main() {
  return sim::testing::TestSuite::global().run();
}
