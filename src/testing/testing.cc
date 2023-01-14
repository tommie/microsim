#include "testing.h"

namespace sim::testing {

  namespace {

    static StreamTestReporter& default_test_reporter() {
      static StreamTestReporter reporter(&std::cout);
      return reporter;
    }

    static TestSuite& default_test_suite() {
      static TestSuite suite(&default_test_reporter());
      return suite;
    }

  }

  int TestSuite::run() {
    int ret = 0;
    for (auto *cas : cases_) {
      auto status = (*cas)();
      reporter_->test_case_done(cas->name(), status);
      if (!status.ok()) {
        ret = 1;
      }
    }
    return ret;
  }

  TestSuite& TestSuite::global() {
    return default_test_suite();
  }

  TestCase::TestCase() {
    default_test_suite().register_test_case(this);
  }

  sim::util::Status TestCase::operator()() {
    if (auto status = setUp(); !status.ok()) return status;
    run();
    return result_;
  }

}  // namespace sim::testing
