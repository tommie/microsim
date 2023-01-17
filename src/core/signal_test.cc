#include "signal.h"

#include <array>

#include "../testing/testing.h"

namespace sim::core {

  class TestHandler {
  public:
    virtual void test() { ++n; }

    int n = 0;
  };

  SIM_TEST(SignalTest) {
    SignalQueue queue;

    if (queue.execute_front()) fail("queue.execute_front() returned true");

    TestHandler hdlr;
    Signal<TestHandler, &TestHandler::test> sig(&queue, std::to_array({&hdlr}));

    if (queue.execute_front()) fail("queue.execute_front() returned true");

    if (hdlr.n != 0) fail("hdlr.n not 0 without emit");

    sig.emit();

    if (queue.execute_front()) fail("queue.execute_front() returned true");

    if (hdlr.n != 1) fail("hdlr.n not 1 after emit");
  }

}  // namespace sim::core

int main() {
  return sim::testing::TestSuite::global().run();
}
