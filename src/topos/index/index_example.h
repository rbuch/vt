
#if !defined INCLUDED_TOPOS_INDEX_EXAMPLE
#define INCLUDED_TOPOS_INDEX_EXAMPLE

#include "config.h"
#include "serialization/traits/byte_copy_trait.h"

#include <cstdint>
#include <functional>

namespace vt { namespace index {

/*
 * This index (ExampleIndex) exists for pedagogical purposes only: to demonstrate
 * an example vt::index that conforms to the correct interface. If the detector
 * is enabled, this can be checked at compile time.
 */

struct ExampleIndex {
  using IndexSizeType = size_t;
  using ApplyType = std::function<void(ExampleIndex)>;
  using IsByteCopyable = serialization::ByteCopyTrait;

  // An index must have a default constructor
  ExampleIndex() = default;

  // An index must have a copy constructor
  ExampleIndex(ExampleIndex const&) = default;

  // An index must have an operator=
  ExampleIndex& operator=(ExampleIndex const&) = default;

  // An index must have equality defined
  bool operator==(ExampleIndex const& other) const;

  // An index must inform the runtime of its packed size
  IndexSizeType packedSize() const;

  // An index must inform the runtime if it is byte copyable
  bool indexIsByteCopyable() const;

  // Generate unique bit sequence for element index
  UniqueIndexBitType uniqueBits() const;

  // Iterator for every element in a index used as a range
  void foreach(ExampleIndex const& max, ApplyType fn) const;
};

}} // end namespace vt::index

#if backend_check_enabled(detector)
  #include "topos/index/traits/traits.h"

  namespace vt { namespace index {

  static_assert(
    IndexTraits<ExampleIndex>::is_index,
    "ExampleIndex must follow the Index concept"
  );

  }} // end namespace vt::index
#endif

#endif  /*INCLUDED_TOPOS_INDEX_EXAMPLE*/
