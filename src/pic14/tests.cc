#include <array>
#include <fstream>
#include <iostream>
#include <list>
#include <unordered_map>
#include <sstream>

#include "../core/status.h"
#include "../testing/testing.h"
#include "p16f88x.h"

int parse_hex(char c) {
  if (c < '0') return -1;
  if (c <= '9') return c - '0';
  if (c < 'A') return -1;
  if (c <= 'F') return c - 'A' + 10;
  if (c < 'a') return -1;
  if (c <= 'f') return c - 'a' + 10;
  return -1;
}

int parse_hex(char c1, char c2) {
  int i1 = parse_hex(c1);
  if (i1 < 0) return -1;
  return (i1 << 4) | parse_hex(c2);
}

sim::core::Status load_ihex(std::istream &in, std::function<sim::core::Status(uint16_t, std::u8string_view)> load) {
  for (std::array<char, 256> buf; in.getline(&buf[0], buf.size());) {
    std::string line(&buf[0]);

    if (line.size() == 0) {
      continue;
    }

    if (line.size() < 1 + 2 + 4 + 2 + 2) {
      return std::make_error_code(std::errc::invalid_argument);
    }

    if (line[0] != ':') {
      return std::make_error_code(std::errc::invalid_argument);
    }

    int count = parse_hex(line[1], line[2]);
    if (count < 0 || line.size() < 1 + 2 + 4 + 2 + 2 * count + 2)
      return std::make_error_code(std::errc::invalid_argument);

    int addr = parse_hex(line[3], line[4]);
    if (addr < 0) {
      return std::make_error_code(std::errc::invalid_argument);
    }

    addr = (addr << 8) | parse_hex(line[5], line[6]);
    int rtype = parse_hex(line[7], line[8]);
    unsigned int csum = parse_hex(line[1 + 2 + 4 + 2 + 2 * count], line[1 + 2 + 4 + 2 + 2 * count + 1]);
    if (addr < 0 || rtype < 0 || csum < 0) {
      return std::make_error_code(std::errc::invalid_argument);
    }

    std::u8string data(count, 0);
    for (int i = 0, j = 1 + 2 + 4 + 2; i < count; ++i, j += 2) {
      data[i] = parse_hex(line[j], line[j + 1]);
    }

    unsigned int csum_real = 0;
    for (int i = 1; i < line.size() - 2; i += 2) {
      csum_real += parse_hex(line[i], line[i + 1]);
    }

    if (static_cast<uint8_t>(csum + csum_real) != 0) {
      return std::make_error_code(std::errc::invalid_argument);
    }

    switch (rtype) {
    case 0:
      if (auto status = load(static_cast<uint16_t>(addr / 2), data); !status.ok()) {
        return status;
      }
      break;

    case 1:
      return sim::core::Status();

    case 4:
      // Ignored.
      break;

    default:
      return std::make_error_code(std::errc::invalid_argument);
    }
  }

  return sim::core::Status(std::make_error_code(std::errc::invalid_argument), "missing EOF record");
}

template<typename Proc>
sim::core::Status load_testdata_ihex(Proc *proc, std::string_view path) {
  auto icsp = proc->enter_icsp();
  std::ifstream ihex(path);

  if (!ihex) {
    return sim::core::Status(std::error_code(errno, std::system_category()), path);
  }

  return load_ihex(ihex, std::bind_front(&sim::pic14::ICSP::load_program, &icsp));
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
  sim::core::Ticks advance_until_sleep() {
    sim::core::Ticks ticks = 0;
    for (;;) {
      ticks += proc.advance();
      if (proc.is_sleeping()) return ticks;
    }
  }

  sim::core::Status setUp() override {
    if (auto status = load_testdata_ihex(&proc, firmware_); !status.ok()) {
      return status;
    }
    return sim::core::Status();
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


PROCESSOR_TEST(PortTest, P16F88X, "testdata/port.hex") {
  pins["RB0"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1");
}

PROCESSOR_TEST(ExtInterruptTest, P16F88X, "testdata/extint.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0 before pin change");

  pins["RB0"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after pin change");
}

PROCESSOR_TEST(GlobalInterruptTest, P16F88X, "testdata/gie.hex") {
  advance_until_sleep();

  if (pins["RA0"]->value() != 0) fail("RA0 should be 0 before pin change");

  pins["RB0"]->set_external(1);

  advance_until_sleep();

  if (pins["RA0"]->value() != 1) fail("RA0 should be 1 after pin change");
}

int main() {
  return sim::testing::TestSuite::global().run();
}
