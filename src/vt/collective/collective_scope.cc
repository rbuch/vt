/*
//@HEADER
// *****************************************************************************
//
//                             collective_scope.cc
//                           DARMA Toolkit v. 1.0.0
//                       DARMA/vt => Virtual Transport
//
// Copyright 2019 National Technology & Engineering Solutions of Sandia, LLC
// (NTESS). Under the terms of Contract DE-NA0003525 with NTESS, the U.S.
// Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact darma@sandia.gov
//
// *****************************************************************************
//@HEADER
*/

#if !defined INCLUDED_VT_COLLECTIVE_COLLECTIVE_SCOPE_CC
#define INCLUDED_VT_COLLECTIVE_COLLECTIVE_SCOPE_CC

#include "vt/config.h"
#include "vt/collective/collective_scope.h"
#include "vt/collective/collective_alg.h"

namespace vt { namespace collective {

TagType CollectiveScope::mpiCollectiveAsync(ActionType action) {
  auto impl = getScope();
  auto tag = impl->next_seq_++;

  // Create a new collective action with the next tag
  detail::ScopeImpl::CollectiveInfo info(tag, action);

  impl->planned_collective_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(tag),
    std::forward_as_tuple(info)
  );

  debug_print(
    gen, node,
    "mpiCollectiveAsync: scope={}: new MPI collective with tag={}\n",
    scope_, tag
  );

  // Do a reduction followed by a broadcast to trigger a collective
  // operation. Note that in VT reductions and broadcasts can be executed out of
  // order. This implies that runCollective might be called with different tags
  // on different nodes. Thus, in runCollective, we will use a consensus
  // protocol to agree on a consistent tag across all the nodes.
  NodeType collective_root = 0;

  using CollectiveMsg = CollectiveAlg::CollectiveMsg;
  auto cb = theCB()->makeBcast<CollectiveMsg,&CollectiveAlg::runCollective>();

  // Put this in a separate namespace
  // @todo: broader issue: need to implement a better way to scope reductions
  auto ident = 0xEFFFFFFFFFFFFFFF;
  auto msg = makeMessage<CollectiveMsg>(is_user_tag_, scope_, tag, collective_root);

  // The tag for the reduce is a combination of the scope and seq tag.
  theCollective()->reduce<collective::None>(
    collective_root, msg.get(), cb, scope_, no_seq_id, 1, ident, tag
  );

  return tag;
}

bool CollectiveScope::isCollectiveDone(TagType tag) {
  auto impl = getScope();
  return impl->planned_collective_.find(tag) == impl->planned_collective_.end();
}

void CollectiveScope::waitCollective(TagType tag) {
  while (not isCollectiveDone(tag)) {
    runScheduler();
  }
}

void CollectiveScope::mpiCollectiveWait(ActionType action) {
  waitCollective(mpiCollectiveAsync(action));
}

detail::ScopeImpl* CollectiveScope::getScope() {
  auto& scopes = is_user_tag_ ?
    theCollective()->user_scope_ :
    theCollective()->system_scope_;

  auto scope_iter = scopes.find(scope_);
  vtAssert(scope_iter != scopes.end(), "Scope must exist");
  return scope_iter->second.get();
}

CollectiveScope::~CollectiveScope() {
  auto impl = getScope();
  impl->live_ = false;
}

}} /* end namespace vt::collective */

#endif /*INCLUDED_VT_COLLECTIVE_COLLECTIVE_SCOPE_CC*/
