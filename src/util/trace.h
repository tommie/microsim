#ifndef sim_util_trace_h
#define sim_util_trace_h

#include <cstdint>
#include <vector>

#include "registry.h"

/// Registers a trace entry type. This is used to allocate a unique
/// integer for the type. This must be called in the global
/// namespace. The name must be unique in the current scope.
#define REGISTER_TRACE_ENTRY_TYPE(name, type)                           \
  const ::sim::util::TraceEntryType<type> type::TYPE;                   \
  template int ::sim::util::internal::trace_kind<type>();               \
  REGISTRY_ADD(::sim::util::internal::TraceEntries, name, ::sim::util::TraceEntryType<type>, &type::TYPE);

namespace sim::util {

  namespace internal {

    class TraceEntryType {
    public:
      virtual std::size_t size() const = 0;
    };

    DECLARE_REGISTRY(TraceEntries, TraceEntryType);

    int trace_kind(const TraceEntryType *type);

    /// Returns the integer kind corresponding to the entry type.
    template<typename T>
    extern int trace_kind() {
      static int kind = []() { return trace_kind(&T::TYPE); }();
      return kind;
    }

  }  // namespace internal

  /// The meta-type for an entry base.
  template<typename T>
  class TraceEntryType : public internal::TraceEntryType {
    static_assert(std::is_trivially_copyable_v<T>, "Trace entry types must be trivially copyable");

  public:
    std::size_t size() const override { return sizeof(T); }
  };

  /// A base class for entry types.
  ///
  /// Subclasses must have a `static const TraceEntryType<T> TYPE` member.
  class TraceEntryBase {};

  /// A view of a parsed trace entry. The view uses a borrowed pointer
  /// to actual storage. Be aware that both `TraceBuffer::emplace()`
  /// and `pop()` may invalidate the pointer.
  class TraceEntry {
  public:
    TraceEntry(uint8_t kind, const TraceEntryBase *entry)
      : kind_(kind), entry_(entry) {}

    int kind() const { return kind_; }
    const internal::TraceEntryType& type() const { return internal::TraceEntries::get(kind_ - 1); }

    /// Returns a cast entry, if the type and kind match. Returns
    /// `nullptr` otherwise.
    template<typename Type>
    const Type* as() const {
      return &type() == &Type::TYPE ? static_cast<const Type*>(entry_) : nullptr;
    }

    /// Invokes a function with the entry type-cast.
    ///
    /// See the unit-test for a usage example.
    template<typename Type, typename... Types, typename Visitor>
    auto visit(Visitor &&vis) {
      // A helper because we need to have the parameter pack last.
      return visit2<Visitor, Type, Types...>(std::move(vis));
    }

  private:
    template<typename Visitor, typename Type, typename... Types>
    auto visit2(Visitor &&vis) {
      if (&type() == &Type::TYPE)
        return vis(static_cast<const Type&>(*entry_));

      if constexpr (sizeof...(Types) == 0)
        return vis(*this);
      else
        return visit2<Visitor, Types...>(std::move(vis));
    }

  private:
    uint8_t kind_;
    const TraceEntryBase *entry_;
  };

  /// A circular buffer of trace entries. Subsystems can define their
  /// own trace entry types and efficiently store them in the buffer.
  ///
  /// Entries have at most sizeof(uintptr_t) - 1 bytes overhead.
  class TraceBuffer {
  public:
    /// Constructs a new buffer with the given capacity in bytes.
    explicit TraceBuffer(std::size_t capacity);

    /// Appends an entry to the buffer.
    template<typename T, typename... Args>
    void emplace(Args... args) { new (allocate(sizeof(T), internal::trace_kind<T>())) T(args...); }

    /// Pops the top entry. The buffer must not be empty.
    void pop();

    /// Returns true if the buffer is empty.
    bool empty() const { return entries() == 0; }

    /// Returns the configured capacity, in bytes.
    std::size_t capacity() const { return entries_.size() * sizeof(uintptr_t); }

    /// Returns the currently used space, in bytes.
    std::size_t size() const {
      if (!empty() && back_ == front_) return entries_.size() * sizeof(uintptr_t);
      return (back_ - front_ + entries_.size()) % entries_.size() * sizeof(uintptr_t);
    }

    /// Returns the currently used space, in entries.
    std::size_t entries() const { return num_entries_; }

    /// Returns the number of entries that have been discarded by
    /// `emplace()`, since creation of the buffer.
    std::size_t discarded() const { return discarded_; }

    /// Returns the top entry. The buffer must not be empty.
    TraceEntry top() const;

  private:
    const uint8_t* type_ptr(std::size_t i) const;
    uint8_t* type_ptr(std::size_t i);

    static const internal::TraceEntryType* type(std::size_t kind);

    void* allocate(std::size_t n, uint8_t kind);

    void discard();

  private:
    std::vector<uintptr_t> entries_;
    std::size_t front_ = 0, back_ = 0;
    std::size_t num_entries_ = 0;
    std::size_t discarded_ = 0;
  };

  /// A facade that only allows writing to a trace buffer. If the
  /// buffer's capacity is zero, tracing is disabled.
  class TraceWriter {
  public:
    explicit TraceWriter(TraceBuffer *buf)
      : buf_(buf->capacity() == 0 ? nullptr : buf) {}

    template<typename T, typename... Args>
    void emplace(Args... args) {
      if (buf_ != nullptr)
        buf_->emplace<T>(args...);
    }

  private:
    TraceBuffer *buf_;
  };

}  // namespace sim::util

#endif  // sim_util_trace_h
