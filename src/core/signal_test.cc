#include "signal.h"

#include "../testing/testing.h"

namespace sim::core {

  class TestHandler {
  public:
    void test(const bool&) { ++n; }

    int n = 0;
  };

  SIM_TEST(BoolSignalTest) {
    TestHandler hdlr;
    Signal<bool> sig(std::bind_front(&TestHandler::test, &hdlr));

    if (hdlr.n != 0) fail("hdlr.n not 0 without set");
    if (sig.value() != false) fail("value not false without set");

    sig.set(true);

    if (hdlr.n != 1) fail("hdlr.n not 1 after set");
    if (sig.value() != true) fail("value not true after set");

    sig.set(true);

    if (hdlr.n != 1) fail("hdlr.n not 1 after second set");
    if (sig.value() != true) fail("value not true after second set");

    sig.set(false);

    if (hdlr.n != 2) fail("hdlr.n not 2 after reset");
    if (sig.value() != false) fail("value not false after reset");
  }

  SIM_TEST(ByteSignalTest) {
    TestHandler hdlr;
    Signal<uint8_t> sig(std::bind_front(&TestHandler::test, &hdlr));

    if (hdlr.n != 0) fail("hdlr.n not 0 without set");
    if (sig.value() != 0) fail("value not false without set");

    sig.set(42);

    if (hdlr.n != 1) fail("hdlr.n not 1 after set");
    if (sig.value() != 42) fail("value not 42 after set");

    sig.set(42);

    if (hdlr.n != 1) fail("hdlr.n not 1 after second set");
    if (sig.value() != 42) fail("value not 42 after second set");

    sig.set(43);

    if (hdlr.n != 2) fail("hdlr.n not 2 after third set");
    if (sig.value() != 43) fail("value not 43 after third set");
  }

  SIM_TEST(BoolCombinedSignalTest) {
    TestHandler hdlr;
    CombinedSignal<CombineOr<bool>> sig(std::bind_front(&TestHandler::test, &hdlr), 2);

    if (hdlr.n != 0) fail("hdlr.n not 0 without set");
    if (sig.value() != false) fail("value not false without set");

    auto *sig0 = sig.make_signal();
    auto *sig1 = sig.make_signal();
    sig0->set(true);

    if (hdlr.n != 1) fail("hdlr.n not 1 after set");
    if (sig.value() != true) fail("value not true after set");

    sig0->set(true);

    if (hdlr.n != 1) fail("hdlr.n not 1 after second set");
    if (sig.value() != true) fail("value not true after second set");

    sig0->set(false);

    if (hdlr.n != 2) fail("hdlr.n not 2 after reset");
    if (sig.value() != false) fail("value not false after reset");

    sig1->set(true);

    if (hdlr.n != 3) fail("hdlr.n not 3 after sig1->set");
    if (sig.value() != true) fail("value not true after sig1->set");
  }

  SIM_TEST(SubCombinedSignalTest) {
    TestHandler hdlr;
    CombinedSignal<CombineOr<bool>> sig(std::bind_front(&TestHandler::test, &hdlr), 1);
    auto sig0 = sig.make_combined_signal<Inverted<CombineAnd<bool>>>(1);

    if (hdlr.n != 0) fail("hdlr.n not 0 after make_combined_signal");
    if (sig.value() != false) fail("value not false after make_combined_signal");

    auto sig0_0 = sig0.make_signal();
    sig0_0->set(false);

    if (hdlr.n != 1) fail("hdlr.n not 1 after set");
    if (sig.value() != true) fail("value not true after set");
  }

  SIM_TEST(BoolInvertedTest) {
    Inverted<CombineOr<bool>> inv;

    if (inv.postprocess(true)) fail("postprocess(true) was true");
    if (!inv.postprocess(false)) fail("postprocess(false) was false");
  }

  template struct Inverted<CombineOr<uint8_t>>;
  template class CombinedSignal<CombineOr<uint8_t>>;
  template class Signal<uint8_t>;

}  // namespace sim::core

int main() {
  return sim::testing::TestSuite::global().run();
}
