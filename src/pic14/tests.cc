#include <array>
#include <fstream>
#include <iostream>
#include <list>
#include <unordered_map>
#include <sstream>

#include "../core/ihex.h"
#include "../testing/testing.h"
#include "../util/status.h"
#include "p16f88x.h"

template<typename Proc>
sim::util::Status load_testdata_ihex(Proc *proc, std::string_view path) {
  auto icsp = proc->enter_icsp();
  std::ifstream ihex(path);

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

template<typename Proc>
class ProcessorTestCase : public sim::testing::TestCase {
public:
  ProcessorTestCase(std::string_view firmware)
    : firmware_(firmware), proc(&listener) {
    for (const auto &descr : proc.pins()) {
      listener.pin_descrs[descr.pin] = descr;
      pins[descr.name] = descr.pin;
    }
  }

protected:
  void advance_until_sleep() {
    sim::core::SimulationLimit limit = {
      .cond = [this](sim::core::Ticks at_tick) { return !proc.is_sleeping(); },
    };
    for (;;) {
      proc.advance_to(limit);
      if (proc.is_sleeping()) return;
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
  DeviceListener listener;

protected:
  Proc proc;
  std::unordered_map<std::string, sim::core::Pin*> pins;
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

  pins["RB0"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after pin change");
}

PROCESSOR_TEST(GlobalInterruptTest, P16F887, "testdata/gie.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0 before pin change");

  pins["RB0"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after pin change");
}

int main() {
  return sim::testing::TestSuite::global().run();
}
