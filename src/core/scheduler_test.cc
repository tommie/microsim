#include "scheduler.h"

#include <vector>

#include "../testing/testing.h"

namespace sim::core {

  class TestSchedulable : public Schedulable {
  public:
    Advancement advance_to(const SimulationLimit &limit) override {
      ++advance_calls;
      return {.at_tick = at, .next_tick = next};
    }

    using Schedulable::schedule_immediately;

    Ticks at = 0;
    Ticks next = 0;
    int advance_calls = 0;
  };

  SIM_TEST(SchedulerSingleTest) {
    TestSchedulable s;
    s.next = 1;
    Scheduler scheduler(std::vector<Schedulable*>{&s});

    auto got = scheduler.advance_to(SimulationLimit{.end_tick = 0});
    if (got.at_tick != 0) fail("at_tick not 0");
    if (got.next_tick != 1) fail("next_tick not 1");
    if (s.advance_calls != 1) fail("advance_calls not 1");

    s.at = 1;
    s.next = 2;

    got = scheduler.advance_to(SimulationLimit{.end_tick = 1});
    if (got.at_tick != 1) fail("at_tick not 1");
    if (got.next_tick != 2) fail("next_tick not 2");

    s.at = 2;
    s.next = -1;

    got = scheduler.advance_to(SimulationLimit{.end_tick = 2});
    if (got.at_tick != 2) fail("at_tick not 2");
    if (got.next_tick != -1) fail("next_tick not -1");

    s.at = 3;
    s.next = 4;

    s.schedule_immediately();

    got = scheduler.advance_to(SimulationLimit{.end_tick = 3});
    if (got.at_tick != 3) fail("at_tick not 3");
    if (got.next_tick != 4) fail("next_tick not 4");

    if (s.advance_calls != 4) fail("advance_calls not 4");

    s.at = 4;
    s.next = -1;

    got = scheduler.advance_to(SimulationLimit{.end_tick = 10, .cond = [](Ticks) { return false; }});
    if (got.at_tick != 4) fail("at_tick not 4");
    if (s.advance_calls != 5) fail("advance_calls not 5");
  }

  SIM_TEST(SchedulerParentTest) {
    TestSchedulable parent;
    parent.next = -1;
    Scheduler pscheduler(std::vector<Schedulable*>{&parent});

    TestSchedulable s;
    s.next = -1;
    Scheduler scheduler(std::vector<Schedulable*>{&s}, &parent);

    s.schedule_immediately();

    pscheduler.advance_to({});
    if (parent.advance_calls != 1) fail("advance_calls not 1");
  }

  SIM_TEST(SchedulerDoubleTest) {
    TestSchedulable s1, s2;
    s1.next = 1;
    s2.next = 2;
    Scheduler scheduler(std::vector<Schedulable*>{&s1, &s2});

    auto got = scheduler.advance_to(SimulationLimit{.end_tick = 0});
    if (got.at_tick != 0) fail("at_tick not 0");
    if (got.next_tick != 1) fail("next_tick not 1");
    if (s1.advance_calls != 1) fail("s1.advance_calls not 1");
    if (s2.advance_calls != 1) fail("s2.advance_calls not 1");

    s1.at = s2.at = 1;
    s1.next = 3;

    got = scheduler.advance_to(SimulationLimit{.end_tick = 1});
    if (got.at_tick != 1) fail("at_tick not 1");
    if (got.next_tick != 2) fail("next_tick not 2");
    if (s1.advance_calls != 2) fail("s1.advance_calls not 2");
    if (s2.advance_calls != 1) fail("s2.advance_calls not 1");

    s1.at = s2.at = 2;
    s2.next = 4;

    got = scheduler.advance_to(SimulationLimit{.end_tick = 2});
    if (got.at_tick != 2) fail("at_tick not 2");
    if (got.next_tick != 3) fail("next_tick not 3");
    if (s1.advance_calls != 2) fail("s1.advance_calls not 2");
    if (s2.advance_calls != 2) fail("s2.advance_calls not 2");

    s1.at = s2.at = 3;
    s1.next = s2.next = -1;

    got = scheduler.advance_to(SimulationLimit{.end_tick = 10});
    if (got.at_tick != 3) fail("at_tick not 3");
    if (got.next_tick != -1) fail("next_tick not -1");
    if (s1.advance_calls != 3) fail("s1.advance_calls not 3");
    if (s2.advance_calls != 3) fail("s2.advance_calls not 3");
  }

}  // namespace sim::core

int main() {
  return sim::testing::TestSuite::global().run();
}
