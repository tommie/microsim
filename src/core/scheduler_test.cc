#include "scheduler.h"

#include <vector>

#include "../testing/testing.h"

namespace sim::core {

  class TestSchedulable : public Schedulable {
  public:
    Advancement advance_to(const AdvancementLimit &limit) override {
      ++advance_calls;
      return { .next_time = TimePoint(Duration(next)) };
    }

    using Schedulable::schedule_immediately;

    int next = 0;
    int advance_calls = 0;
  };

  AdvancementLimit advance_to_time(int limit) {
    return {
      .can_advance_to = [limit = TimePoint(Duration(limit))](TimePoint at) { return at <= limit; },
    };
  }

  SIM_TEST(SchedulerSingleTest) {
    TestSchedulable s;
    s.next = 1;
    Scheduler scheduler(std::vector<Schedulable*>{&s});

    auto got = scheduler.advance_to(advance_to_time(0));
    if (got.next_time != TimePoint(Duration(1))) fail("next_time not 1");
    if (s.advance_calls != 1) fail("advance_calls not 1");

    s.next = 2;

    got = scheduler.advance_to(advance_to_time(1));
    if (got.next_time != TimePoint(Duration(2))) fail("next_time not 2");

    s.next = -1;

    got = scheduler.advance_to(advance_to_time(2));
    if (!is_never(got.next_time)) fail("next_time not NEVER");

    s.next = 4;

    s.schedule_immediately();

    got = scheduler.advance_to(advance_to_time(3));
    if (got.next_time != TimePoint(Duration(4))) fail("next_time not 4");

    if (s.advance_calls != 4) fail("advance_calls not 4");
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

    auto got = scheduler.advance_to(advance_to_time(0));
    if (got.next_time != TimePoint(Duration(1))) fail("next_time not 1");
    if (s1.advance_calls != 1) fail("s1.advance_calls not 1");
    if (s2.advance_calls != 1) fail("s2.advance_calls not 1");

    s1.next = 3;

    got = scheduler.advance_to(advance_to_time(1));
    if (got.next_time != TimePoint(Duration(2))) fail("next_time not 2");
    if (s1.advance_calls != 2) fail("s1.advance_calls not 2");
    if (s2.advance_calls != 1) fail("s2.advance_calls not 1");

    s2.next = 4;

    got = scheduler.advance_to(advance_to_time(2));
    if (got.next_time != TimePoint(Duration(3))) fail("next_time not 3");
    if (s1.advance_calls != 2) fail("s1.advance_calls not 2");
    if (s2.advance_calls != 2) fail("s2.advance_calls not 2");

    s1.next = s2.next = -1;

    got = scheduler.advance_to(advance_to_time(10));
    if (!is_never(got.next_time)) fail("next_time not -1");
    if (s1.advance_calls != 3) fail("s1.advance_calls not 3");
    if (s2.advance_calls != 3) fail("s2.advance_calls not 3");
  }

}  // namespace sim::core

int main() {
  return sim::testing::TestSuite::global().run();
}
