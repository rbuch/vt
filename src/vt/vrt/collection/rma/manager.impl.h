/*
//@HEADER
// *****************************************************************************
//
//                                manager.impl.h
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

#if !defined INCLUDED_VT_VRT_COLLECTION_RMA_MANAGER_IMPL_H
#define INCLUDED_VT_VRT_COLLECTION_RMA_MANAGER_IMPL_H

#include "vt/config.h"
#include "vt/vrt/collection/rma/count_msg.h"

namespace vt { namespace vrt { namespace collection { namespace rma {

template <typename ColT>
/*static*/ void Manager::connect(ConnectMsg<ColT>* msg, ColT* col) {
  using IndexType = typename ColT::IndexType;
  remote_accessors<IndexType>[msg->handle()][col->getIndex()].push_back(msg->from());
  handle_to_proxy[msg->handle()] = col->getProxy();

  debug_print(
    gen, node,
    "Manager::connect: handle={}, idx={}, from={}\n",
    msg->handle(), col->getIndex(), msg->from()
  );
}

template <typename ColT>
/*static*/ void Manager::connected(ConnectedMsg<ColT>* msg, ColT* col) {
  col->addHandleConnection(msg->handle(), msg->from(), msg->slot(), msg->rank());

  debug_print(
    gen, node,
    "Manager::connected: handle={}, from={}, slot={}, rank={}\n",
    msg->handle(), msg->from(), msg->slot(), msg->rank()
  );
}

template <typename ColT>
/*static*/ void Manager::finish(HandleType handle) {
  using IndexType = typename ColT::IndexType;

  debug_print(
    gen, node,
    "Manager::finish: handle={}, num={}\n",
    handle, local_handles<IndexType>[handle].size()
  );

  int& slot = handle_slot[handle];
  for (auto&& idx : local_handles<IndexType>[handle]) {
    local_offsets<IndexType>[handle][idx] = slot++;
  }
  //auto epoch = theTerm()->makeCollectiveEpoch();
  //theMsg()->pushEpoch(epoch);

  auto rank = theContext()->getNode();
  auto proxy_bits = handle_to_proxy[handle];
  auto& remotes = remote_accessors<IndexType>[handle];
  for (auto&& idx : local_handles<IndexType>[handle]) {
    auto cur_slot = local_offsets<IndexType>[handle][idx];
    for (auto&& connector : remotes[idx]) {
      CollectionProxy<ColT, IndexType> col_proxy(proxy_bits);
      col_proxy[connector].template send<ConnectedMsg<ColT>,Manager::connected<ColT>>(
        handle, idx, cur_slot, rank
      );
    }
  }

  //theMsg()->pop(epoch);

  int const num_local_handles = local_offsets<IndexType>[handle].size();
  auto msg = makeMessage<CountMsg<ColT>>(handle, num_local_handles);
  auto cb = theCB()->makeBcast<CountMsg<ColT>,Manager::finishLocalCount<ColT>>();
  theCollective()->reduce<collective::MaxOp<int>>(0,msg.get(),cb);

  do {
    vt::runScheduler();
  } while (payload_<ColT>.find(handle) == payload_<ColT>.end());
}

template <typename ColT>
/*static*/ void Manager::finishLocalCount(CountMsg<ColT>* msg) {
  auto const handle = msg->handle();
  auto const count = msg->getVal();

// int MPI_Win_allocate
// (MPI_Aint size, int disp_unit, MPI_Info info,
// MPI_Comm comm, void *baseptr, MPI_Win *win)

  int ret = 0;
  void* ptr = nullptr;
  MPI_Win window;

  auto comm = theContext()->getComm();
  auto bytes = count * sizeof(int);
  auto elm = sizeof(int);

  ret = MPI_Win_allocate(bytes, elm, MPI_INFO_NULL, comm, &ptr, &window);
  vtAssert(ret == MPI_SUCCESS, "MPI_Win_allocate: Should be successful");

  auto payload = std::make_unique<Payload>(window, count, ptr);
  payload_<ColT>[handle] = std::move(payload);
}

template <typename ColT>
/*static*/ int Manager::atomicGetAccum(
  HandleType handle, int rank, int slot, int val
) {
  vtAssertExpr(payload_<ColT>.find(handle) != payload_<ColT>.end());
  auto payload = payload_<ColT>[handle].get();
  auto window = payload->window;
  auto count = payload->count;
  int res = -1;
  MPI_Win_lock(MPI_LOCK_SHARED, rank, 0, window);
  MPI_Get_accumulate(
    &val, 1, MPI_INT,
    &res, 1, MPI_INT,
    rank, slot, count, MPI_INT, MPI_SUM, window
  );
  MPI_Win_unlock(rank, window);
  return res;
}

template <typename ColT>
/*static*/ void Manager::addLocalHandle(
  HandleType handle, typename ColT::IndexType idx
) {
  using IndexType = typename ColT::IndexType;
  local_handles<IndexType>[handle].insert(idx);
}

template <typename IndexT>
/*static*/ std::unordered_map<HandleType, std::map<IndexT, int>>
  Manager::local_offsets = {};

template <typename IndexT>
/*static*/ std::unordered_map<
  HandleType, std::unordered_map<IndexT, std::vector<IndexT>>
> Manager::remote_accessors = {};

template <typename IndexT>
/*static*/ std::unordered_map<HandleType, std::unordered_set<IndexT>>
  Manager::local_handles = {};

template <typename ColT>
/*static*/ std::unordered_map<HandleType, std::unique_ptr<Payload>>
  Manager::payload_ = {};


}}}} /* end namespace vt::vrt::collection::rma */

#endif /*INCLUDED_VT_VRT_COLLECTION_RMA_MANAGER_IMPL_H*/
