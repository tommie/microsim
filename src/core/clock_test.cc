#include "clock.h"

#include "../testing/testing.h"

namespace sim::core {

  SIM_TEST(ClockTest) {
    Clock c(Duration(10));

    if (c.interval() != Duration(10)) fail("c.interval() not 10");
    if (c.at(Clock::duration(0)) != TimePoint(Duration(0))) fail("c.at(0) not 0");
    if (c.at(Clock::duration(1)) != TimePoint(Duration(10))) fail("c.at(1) not 10");

    c.advance_to(TimePoint());

    if (c.at(Clock::duration(0)) != TimePoint(Duration(0))) fail("c.at(0) not 0 after advance_to(0)");

    c.advance_to(TimePoint(Duration(5)));

    if (c.at(Clock::duration(0)) != TimePoint(Duration(0))) fail("c.at(0) not 0 after advance_to(5)");

    c.advance_to(TimePoint(Duration(10)));

    if (c.at(Clock::duration(0)) != TimePoint(Duration(10))) fail("c.at(0) not 10 after advance_to(10)");

    c.advance_to(TimePoint(Duration(25)));

    if (c.at(Clock::duration(0)) != TimePoint(Duration(20))) fail("c.at(0) not 20 after advance_to(25)");
  }

  SIM_TEST(ClockModifierNoPrescalerTest) {
    Clock c(Duration(10));

    c.advance_to(TimePoint(Duration(20)));

    int nchanges = 0;
    ClockModifier mod([&nchanges]() { ++nchanges; }, &c);

    if (mod.interval() != Duration(10)) fail("mod.interval() not 10");
    if (mod.now() != Clock::time_point(Clock::duration(0))) fail("mod.now() not 0");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(20))) fail("mod.at(0) not 20");
    if (mod.at(Clock::duration(1)) != TimePoint(Duration(30))) fail("mod.at(1) not 30");

    c.advance_to(TimePoint(Duration(30)));

    if (mod.now() != Clock::time_point(Clock::duration(1))) fail("mod.now() not 1 after advance_to(30)");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(30))) fail("mod.at(0) not 30 after advance_to(30)");

    Clock c2(Duration(12));

    c2.advance_to(TimePoint(Duration(30)));

    mod.select(&c2);

    if (nchanges != 1) fail("nchanges not 1 after select(c2)");

    if (mod.interval() != Duration(12)) fail("mod.interval() not 12 after select(c2)");
    if (mod.now() != Clock::time_point(Clock::duration(1))) fail("mod.now() not 1 after select(c2)");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(30))) fail("mod.at(0) not 30 after select(c2)");
    if (mod.at(Clock::duration(1)) != TimePoint(Duration(42))) fail("mod.at(1) not 42 after select(c2)");

    mod.select(&c);

    if (nchanges != 2) fail("nchanges not 2 after select(c)");

    if (mod.now() != Clock::time_point(Clock::duration(1))) fail("mod.now() not 1 after select(c)");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(30))) fail("mod.at(0) not 30 after select(c)");
    if (mod.at(Clock::duration(1)) != TimePoint(Duration(40))) fail("mod.at(1) not 40 after select(c)");
  }

  SIM_TEST(ClockModifierWithPrescalerTest) {
    Clock c(Duration(10));

    c.advance_to(TimePoint(Duration(20)));

    int nchanges = 0;
    ClockModifier mod([&nchanges]() { ++nchanges; }, &c, 4);

    if (mod.interval() != Duration(40)) fail("mod.interval() not 40");
    if (mod.now() != Clock::time_point(Clock::duration(0))) fail("mod.now() not 0");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(20))) fail("mod.at(0) not 20");
    if (mod.at(Clock::duration(1)) != TimePoint(Duration(60))) fail("mod.at(1) not 60");

    c.advance_to(TimePoint(Duration(60)));

    if (mod.now() != Clock::time_point(Clock::duration(1))) fail("mod.now() not 1 after advance_to(60)");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(60))) fail("mod.at(0) not 60 after advance_to(60)");

    Clock c2(Duration(12));

    c2.advance_to(TimePoint(Duration(60)));

    mod.select(&c2, 3);

    if (mod.interval() != Duration(36)) fail("mod.interval() not 36 after select(c2, 3)");
    if (mod.now() != Clock::time_point(Clock::duration(1))) fail("mod.now() not 1 after select(c2, 3)");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(60))) fail("mod.at(0) not 60 after select(c2, 3)");
    if (mod.at(Clock::duration(1)) != TimePoint(Duration(96))) fail("mod.at(1) not 96 after select(c2, 3)");

    mod.select(&c, 2);

    if (mod.now() != Clock::time_point(Clock::duration(1))) fail("mod.now() not 1 after select(c, 2)");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(60))) fail("mod.at(0) not 20 after select(c, 2)");
    if (mod.at(Clock::duration(1)) != TimePoint(Duration(80))) fail("mod.at(1) not 80 after select(c, 2)");
  }

  SIM_TEST(ClockModifierZeroTest) {
    Clock c(Duration(10));

    int nchanges = 0;
    ClockModifier mod([&nchanges]() { ++nchanges; }, &c);

    c.advance_to(TimePoint(Duration(20)));

    if (mod.now() != Clock::time_point(Clock::duration(2))) fail("mod.now() not 0");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(20))) fail("mod.at(0) not 20");

    mod.select(&c, 0);

    if (nchanges != 1) fail("nchanges not 1 after select(c2)");

    if (mod.interval() != Duration()) fail("mod.interval() not 0 after select(c2)");
    if (mod.now() != Clock::time_point(Clock::duration(2))) fail("mod.now() not 2 after select(0)");
    if (mod.at(Clock::duration(0)) != SimulationClock::NEVER) fail("mod.at(0) not NEVER after select(0)");
    if (mod.at(Clock::duration(1)) != SimulationClock::NEVER) fail("mod.at(1) not NEVER after select(0)");

    c.advance_to(TimePoint(Duration(50)));

    if (mod.now() != Clock::time_point(Clock::duration(2))) fail("mod.now() not 2 after advance_to(50)");
    if (mod.at(Clock::duration(0)) != SimulationClock::NEVER) fail("mod.at(0) not NEVER after advance_to(50)");

    mod.select(&c, 1);

    if (mod.now() != Clock::time_point(Clock::duration(2))) fail("mod.now() not 2 after select(1)");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(50))) fail("mod.at(0) not 50 after select(1)");

    c.advance_to(TimePoint(Duration(60)));

    if (mod.now() != Clock::time_point(Clock::duration(3))) fail("mod.now() not 2 after advance_to(60)");
    if (mod.at(Clock::duration(0)) != TimePoint(Duration(60))) fail("mod.at(0) not 60 after advance_to(60)");
  }

}  // namespace sim::core

int main() {
  return sim::testing::TestSuite::global().run();
}
