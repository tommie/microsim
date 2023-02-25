#include "trace.h"

DEFINE_REGISTRY(TraceEntries, ::sim::util::internal::TraceEntryType)

namespace sim::util {

  namespace internal {

    int trace_kind(const TraceEntryType *type) {
      for (auto it = TraceEntries::begin(); it != TraceEntries::end(); ++it) {
        if (*it == type) {
          return 1 + it - TraceEntries::begin();
        }
      }

      std::abort();
    }

  }  // namespace internal

  TraceBuffer::TraceBuffer(std::size_t capacity)
    : entries_((capacity + sizeof(uintptr_t) - 1) / sizeof(uintptr_t), 0) {}

  void TraceBuffer::pop() {
    uint8_t kind = *type_ptr(front_);

    front_ += (type(kind)->size() + sizeof(uintptr_t)) / sizeof(uintptr_t);

    if (front_ == entries_.size() || (back_ < front_ && *type_ptr(front_) == 0))
      front_ = 0;

    --num_entries_;
  }

  TraceEntry TraceBuffer::top() const {
    return TraceEntry(*type_ptr(front_), reinterpret_cast<const TraceEntryBase*>(&entries_[front_]));
  }

  inline const uint8_t* TraceBuffer::type_ptr(std::size_t i) const {
    return &reinterpret_cast<const uint8_t*>(&entries_[(i + entries_.size() - 1) % entries_.size()])[sizeof(uintptr_t) - 1];
  }

  inline uint8_t* TraceBuffer::type_ptr(std::size_t i) {
    return &reinterpret_cast<uint8_t*>(&entries_[(i + entries_.size() - 1) % entries_.size()])[sizeof(uintptr_t) - 1];
  }

  const internal::TraceEntryType* TraceBuffer::type(std::size_t kind) {
    return &internal::TraceEntries::get(kind - 1);
  }

  void* TraceBuffer::allocate(std::size_t n, uint8_t kind) {
    auto nelems = (n + sizeof(uintptr_t)) / sizeof(uintptr_t);
    n = nelems * sizeof(uintptr_t);

    if (n > capacity())
      std::abort();  // The buffer is not large enough for even one entry.

    // We need a contiguous memory area, so if we are too close to
    // the end, mark the next entry as "end" and start at the
    // beginning.
    if (back_ + nelems > entries_.size()) {
      while (!empty() && front_ >= back_)
        discard();

      if (empty())
        front_ = 0;

      // Insert an end marker.
      *type_ptr(back_) = 0;
      back_ = 0;
    }

    while (capacity() - size() < n)
      discard();

    auto *p = &entries_[back_];

    *type_ptr(back_) = kind;
    back_ += nelems;

    if (back_ == entries_.size())
      back_ = 0;

    ++num_entries_;

    return p;
  }

  inline void TraceBuffer::discard() {
    ++discarded_;
    pop();
  }

}  // namespace sim::util
