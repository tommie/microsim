#ifndef sim_testing_testing_h
#define sim_testing_testing_h

#include <iostream>
#include <list>
#include <string>
#include <string_view>

#include "../util/status.h"

namespace sim::testing {

  class TestCase;

  class TestReporter {
  public:
    virtual void test_case_done(std::string_view name, const sim::util::Status &status) {}
  };

  class StreamTestReporter : public TestReporter {
  public:
    explicit StreamTestReporter(std::ostream *out) : out_(out) {}

    void test_case_done(std::string_view name, const sim::util::Status &status) override {
      if (status.ok()) {
        *out_ << "ok TEST " << name << std::endl;
      } else {
        *out_ << "!! TEST " << name << ": " << status << std::endl;
      }
    }

  private:
    std::ostream *out_;
  };

  class TestSuite {
  public:
    explicit TestSuite(TestReporter *reporter) : reporter_(reporter) {}

    int run();

    void register_test_case(TestCase *cas) { cases_.push_back(cas); }

    static TestSuite& global();

  private:
    TestReporter *reporter_;
    std::list<TestCase*> cases_;
  };

  class TestCase {
  public:
    TestCase();

    virtual std::string_view name() const = 0;

    sim::util::Status operator()();

  protected:
    virtual sim::util::Status setUp() { return sim::util::Status(); }
    virtual void run() = 0;

    void fail(const sim::util::Status &status) {
      if (result_.ok()) result_ = status;
    }

    void fail(std::string_view msg) {
      if (result_.ok()) result_ = sim::util::Status(std::make_error_code(std::errc::broken_pipe), msg);
    }

  private:
    sim::util::Status result_;
  };

#define SIM_TEST_BASE(Name, TestCase, ...)                              \
  struct Name : public TestCase {                                       \
    Name() : TestCase(__VA_ARGS__) {}                                   \
    std::string_view name() const override { return #Name; }            \
    void run() override;                                                \
  } Name;                                                               \
  void Name::run()

#define SIM_TEST(Name) SIM_TEST_BASE(Name, ::sim::testing::TestCase)

}  // namespace sim::testing

#endif  // sim_testing_testing_h
