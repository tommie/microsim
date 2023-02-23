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

}  // namespace sim::core

int main() {
  return sim::testing::TestSuite::global().run();
}
