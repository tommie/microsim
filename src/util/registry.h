#ifndef sim_util_registry_h
#define sim_util_registry_h

#include <cstdint>
#include <vector>

namespace sim::util {

  /// A registry populated at initialization time. Entries can be
  /// added, but not removed.
  template<typename T>
  class Registry {
  public:
    using const_iterator = typename std::vector<const T*>::const_iterator;

    static std::size_t size() { return entries().size(); }
    static const T& get(std::size_t i) { return *begin()[i]; }
    static const_iterator begin() { return entries().begin(); }
    static const_iterator end() { return entries().end(); }

    static void add(const T *v) { entries().push_back(v); }

  private:
    static std::vector<const T*>& entries();
  };

/// Declares a registry type of the given name and value type.
#define DECLARE_REGISTRY(name, type) typedef Registry<type> name

/// Defines the registry. This must be invoked in global scope.
#define DEFINE_REGISTRY(name, type)                                     \
  template<> std::vector<const type*>& ::sim::util::Registry<type>::entries() { \
    static std::vector<const type*> entries;                            \
    return entries;                                                     \
  }                                                                     \
  template class ::sim::util::Registry<type>;

/// Adds a value to the named registry. The name must be unique in the
/// scope. This may be called in any namespace.
#define REGISTRY_ADD(reg, name, type, value)                    \
  static const class REG_ ## name {                             \
  public:                                                       \
    REG_ ## name(const type *v) { reg::add(v); }                \
  } REG_ ## name = {value}

}  // namespace sim::util

#endif  // sim_util_registry_h
