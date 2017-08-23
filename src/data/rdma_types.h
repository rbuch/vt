
#if ! defined __RUNTIME_TRANSPORT_RDMA_TYPES__
#define __RUNTIME_TRANSPORT_RDMA_TYPES__

#include "common.h"

#include <sstream>
#include <iostream>

namespace runtime { namespace rdma {

using underlying_node_t = node_t;

struct Endpoint {
  Endpoint(
    bool const& in_is_target, underlying_node_t const& in_node
  ) : is_target(in_is_target), value(in_node)
  { }

  operator underlying_node_t() const { return get(); }

  underlying_node_t get() const { return value; }

  bool target() const { return is_target; }

private:
  bool is_target = false;
  underlying_node_t value = uninitialized_destination;
};

struct Target : Endpoint {
  explicit Target(underlying_node_t const& in_node)
    : Endpoint(true, in_node)
  { }
};

struct NonTarget : Endpoint {
  explicit NonTarget(underlying_node_t const& in_node)
    : Endpoint(false, in_node)
  { }
};

}} //end namespace runtime::rdma

namespace runtime {

using rdma_target_t = rdma::Target;
using rdma_nontarget_t = rdma::NonTarget;

struct from_s {
  rdma_target_t operator=(rdma::underlying_node_t val) {
    return rdma_target_t(val);
  }
};

struct to_s {
  rdma_nontarget_t operator=(rdma::underlying_node_t val) {
    return rdma_nontarget_t(val);
  }
};

extern from_s rdma_from;
extern to_s rdma_to;

} //end namespace runtime

#endif /*__RUNTIME_TRANSPORT_RDMA_TYPES__*/
