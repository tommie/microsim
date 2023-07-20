#include "trace.h"

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "../testing/testing.h"

namespace sim::util {

  class PointerEntry : public TraceEntryBase {
  public:
    static const TraceEntryType<PointerEntry> TYPE;

    explicit PointerEntry(const void* v) : v_(v) {}

    const void* value() const { return v_; }

  private:
    const void* v_;
  };

  REGISTER_TRACE_ENTRY_TYPE(PointerEntry, PointerEntry)

  class Uint16Entry : public TraceEntryBase {
  public:
    static const TraceEntryType<Uint16Entry> TYPE;

    explicit Uint16Entry(uint16_t v) : v_(v) {}

    uint16_t value() const { return v_; }

  private:
    uint16_t v_;
  };

  static const uintptr_t BLANK[] = {0, 0, 0};

  REGISTER_TRACE_ENTRY_TYPE(Uint16Entry, Uint16Entry)

  SIM_TEST(TraceEmplacePointerAlignedTest) {
    TraceBuffer buf(4 * sizeof(uintptr_t));

    if (sizeof(PointerEntry) != sizeof(uintptr_t)) fail("unexpected PointerEntry size");

    if (buf.capacity() != 4 * sizeof(uintptr_t)) fail("capacity() is not 4 words");

    if (!buf.empty()) fail("empty() is not true");
    if (buf.size() > 0) fail("size() is not 0");
    if (buf.entries() != 0) fail("entries() is not 0");
    if (buf.discarded() != 0) fail("discarded() is not 0");

    buf.emplace<PointerEntry>(BLANK);

    if (buf.empty()) fail("empty() is not false after first");
    if (buf.size() != 2 * sizeof(uintptr_t)) fail("size() is not one entry after first");
    if (buf.entries() != 1) fail("entries() is not 1 after first");
    if (buf.discarded() != 0) fail("discarded() is not 0 after first");

    buf.emplace<PointerEntry>(BLANK);

    if (buf.empty()) fail("empty() is not false after second");
    if (buf.size() != 4 * sizeof(uintptr_t)) fail("size() is not two entries after second");
    if (buf.entries() != 2) fail("entries() is not 2 after second");
    if (buf.discarded() != 0) fail("discarded() is not 0 after second");

    buf.emplace<PointerEntry>(BLANK);

    if (buf.empty()) fail("empty() is not false after third");
    if (buf.size() != 4 * sizeof(uintptr_t)) fail("size() is not two entries after third");
    if (buf.entries() != 2) fail("entries() is not 2 after third");
    if (buf.discarded() != 1) fail("discarded() is not 1 after third");
  }

  SIM_TEST(TraceEmplacePointerUnalignedTest) {
    TraceBuffer buf(5 * sizeof(uintptr_t));

    if (sizeof(PointerEntry) != sizeof(uintptr_t)) fail("unexpected PointerEntry size");

    buf.emplace<PointerEntry>(BLANK);

    if (buf.empty()) fail("empty() is not false after first");
    if (buf.size() != 2 * sizeof(uintptr_t)) fail("size() is not one entry after first");
    if (buf.entries() != 1) fail("entries() is not 1 after first");
    if (buf.discarded() != 0) fail("discarded() is not 0 after first");

    buf.emplace<PointerEntry>(BLANK);

    if (buf.empty()) fail("empty() is not false after second");
    if (buf.size() != 4 * sizeof(uintptr_t)) fail("size() is not two entries after second");
    if (buf.entries() != 2) fail("entries() is not 1 after second");
    if (buf.discarded() != 0) fail("discarded() is not 0 after second");

    buf.emplace<PointerEntry>(BLANK);

    if (buf.empty()) fail("empty() is not false after third");
    if (buf.size() != 5 * sizeof(uintptr_t)) fail("size() is not two entries after third");
    if (buf.entries() != 2) fail("entries() is not 2 after third");

    // There wasn't enough room for the third, so we discard the
    // first, and then discard the second to disambiguate the
    // fullness.
    if (buf.discarded() != 1) fail("discarded() is not 1 after third");
  }

  SIM_TEST(TraceEmplaceUint16Test) {
    TraceBuffer buf(2 * sizeof(uintptr_t));

    if (sizeof(Uint16Entry) != sizeof(uint16_t)) fail("unexpected PointerEntry size");

    buf.emplace<Uint16Entry>(0x11);

    if (buf.empty()) fail("empty() is not false after first");
    if (buf.size() != 1 * sizeof(uintptr_t)) fail("size() is not one entry after first");
    if (buf.entries() != 1) fail("entries() is not 1 after first");
    if (buf.discarded() != 0) fail("discarded() is not 0 after first");

    buf.emplace<Uint16Entry>(0x22);

    if (buf.empty()) fail("empty() is not false after second");
    if (buf.size() != 2 * sizeof(uintptr_t)) fail("size() is not two entries after second");
    if (buf.entries() != 2) fail("entries() is not 1 after second");
    if (buf.discarded() != 0) fail("discarded() is not 0 after second");

    buf.emplace<Uint16Entry>(0x33);

    if (buf.empty()) fail("empty() is not false after third");
    if (buf.size() != 2 * sizeof(uintptr_t)) fail("size() is not two entries after third");
    if (buf.entries() != 2) fail("entries() is not 1 after second");
    if (buf.discarded() != 1) fail("discarded() is not 1 after third");
  }

  SIM_TEST(TracePopPointerAlignedTest) {
    TraceBuffer buf(4 * sizeof(uintptr_t));

    buf.emplace<PointerEntry>(BLANK);
    buf.emplace<PointerEntry>(BLANK + 1);
    buf.emplace<PointerEntry>(BLANK + 2);

    if (buf.empty()) fail("empty() is not false before first");

    auto e = buf.top();
    if (e.kind() != 1) fail("kind() is not 1 for first");
    if (e.as<PointerEntry>()->value() != BLANK + 1) fail("value() is not BLANK+1 for first");

    buf.pop();
    if (buf.empty()) fail("empty() is not false after first");

    e = buf.top();
    if (e.kind() != 1) fail("kind() is not 1 for second");
    if (e.as<PointerEntry>()->value() != BLANK + 2) fail("value() is not BLANK+2 for second");

    buf.pop();
    if (!buf.empty()) fail("empty() is not true after second");
  }

  SIM_TEST(TracePopPointerUnalignedTest) {
    TraceBuffer buf(5 * sizeof(uintptr_t));

    buf.emplace<PointerEntry>(BLANK);
    buf.emplace<PointerEntry>(BLANK + 1);
    buf.emplace<PointerEntry>(BLANK + 2);

    if (buf.empty()) fail("empty() is not false before first");

    auto e = buf.top();
    if (e.kind() != 1) fail("kind() is not 1 for first");
    if (e.as<PointerEntry>()->value() != BLANK + 1) fail("value() is not BLANK+1 for first");

    buf.pop();
    if (buf.empty()) fail("empty() is not false after first");

    e = buf.top();
    if (e.kind() != 1) fail("kind() is not 1 for second");
    if (e.as<PointerEntry>()->value() != BLANK + 2) fail("value() is not BLANK+2 for second");

    buf.pop();
    if (!buf.empty()) fail("empty() is not true after second");
  }

  SIM_TEST(TracePopUint16Test) {
    TraceBuffer buf(2 * sizeof(uintptr_t));

    buf.emplace<Uint16Entry>(0x0110);
    buf.emplace<Uint16Entry>(0x0220);
    buf.emplace<Uint16Entry>(0x0330);

    if (buf.empty()) fail("empty() is not false before first");

    auto e = buf.top();
    if (e.kind() != 2) fail("kind() is not 2 for first");
    if (e.as<Uint16Entry>()->value() != 0x0220) fail("value() is not 0x0220 for first");

    buf.pop();
    if (buf.empty()) fail("empty() is not false after first");

    e = buf.top();
    if (e.kind() != 2) fail("kind() is not 2 for second");
    if (e.as<Uint16Entry>()->value() != 0x0330) fail("value() is not 0x0330 for second");

    buf.pop();
    if (!buf.empty()) fail("empty() is not true after second");
  }

  SIM_TEST(TraceVisitTest) {
    TraceBuffer buf(4 * sizeof(uintptr_t));

    buf.emplace<Uint16Entry>(0x0110);
    buf.emplace<PointerEntry>(BLANK);

    const void *ptr = nullptr;
    uint16_t uint16 = 0;
    int nunknowns = 0;

    while (!buf.empty()) {
      buf.top().visit<PointerEntry, Uint16Entry>([&uint16, &ptr, &nunknowns](const auto &e) {
        using T = std::decay_t<decltype(e)>;
        if constexpr (std::is_same_v<T, PointerEntry>) {
          ptr = e.value();
        } else if constexpr (std::is_same_v<T, Uint16Entry>) {
          uint16 = e.value();
        } else {
          ++nunknowns;
        }
      });

      buf.pop();
    }

    if (ptr != BLANK) fail("PointerEntry not BLANK");
    if (uint16 != 0x0110) fail("Uint16Entry not 0x0110");
    if (nunknowns > 0) fail("got unknown entries");
  }

  SIM_TEST(TracePopAtEndMarkerTest) {
    TraceBuffer buf(3 * sizeof(uintptr_t));

    buf.emplace<PointerEntry>(BLANK);
    buf.pop();

    buf.emplace<PointerEntry>(BLANK + 1);
    buf.pop();
  }

  SIM_TEST(TraceFuzzTest) {
    unsigned int seed = std::time(0);
    if (const char *env = std::getenv("MICROSIM_TEST_SEED"); env != nullptr)
      std::istringstream(env) >> seed;
    else
      std::cout << "Using seed " << seed << std::endl;
    std::srand(seed);

    TraceBuffer buf((4 + std::rand() % 4) * sizeof(uintptr_t));

    for (int i = 0; i < 256; ++i) {
      switch (std::rand() % 4) {
      case 0:
        std::cout << "+ptr " << buf.capacity() << ", " << buf.size() << ", " << buf.entries() << ", " << buf.discarded() << std::endl;
        buf.emplace<PointerEntry>(BLANK);
        break;

      case 1:
        std::cout << "+uint16 " << buf.capacity() << ", " << buf.size() << ", " << buf.entries() << ", " << buf.discarded() << std::endl;
        buf.emplace<Uint16Entry>(0x0110);
        break;

      case 2:
      case 3:
        if (buf.empty()) break;

        std::cout << "-entry " << buf.capacity() << ", " << buf.size() << ", " << buf.entries() << ", " << buf.discarded() << std::endl;
        buf.top().visit<PointerEntry, Uint16Entry>([this](const auto &e) {
          using T = std::decay_t<decltype(e)>;
          if constexpr (std::is_same_v<T, PointerEntry>) {
            if (e.value() != BLANK) fail("invalid PointerEntry");
          } else if constexpr (std::is_same_v<T, Uint16Entry>) {
            if (e.value() != 0x0110) fail("invalid Uint16Entry");
          } else {
            fail("invalid entry");
          }
        });

        buf.pop();
        break;
      }
    }
  }

}  // namespace sim::util

int main() {
  return sim::testing::TestSuite::global().run();
}
