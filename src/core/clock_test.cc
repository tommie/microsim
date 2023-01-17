#include "clock.h"

#include "../testing/testing.h"

namespace sim::core {

  SIM_TEST(ClockTest) {
    Clock c(10);

    if (c.interval() != 10) fail("c.interval() not 10");
    if (c.at(0) != 10) fail("c.at(0) not 0");
    if (c.at(1) != 20) fail("c.at(1) not 20");

    c.advance_to(0);

    if (c.at(0) != 10) fail("c.at(0) not 10 after advance_to(0)");

    c.advance_to(5);

    if (c.at(0) != 10) fail("c.at(0) not 10 after advance_to(5)");

    c.advance_to(10);

    if (c.at(0) != 10) fail("c.at(0) not 10 after advance_to(10)");

    c.advance_to(25);

    if (c.at(0) != 20) fail("c.at(0) not 20 after advance_to(25)");
  }

}  // namespace sim::core

int main() {
  return sim::testing::TestSuite::global().run();
}
