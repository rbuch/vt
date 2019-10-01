/*
//@HEADER
// *****************************************************************************
//
//                           base_collection_proxy.h
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

#if !defined INCLUDED_VRT_PROXY_BASE_COLLECTION_PROXY_H
#define INCLUDED_VRT_PROXY_BASE_COLLECTION_PROXY_H

#include "vt/config.h"
#include "vt/vrt/vrt_common.h"

namespace vt { namespace vrt { namespace collection {

template <typename ColT, typename IndexT>
struct BaseCollectionProxy {
  using CollectionType = ColT;
  using IndexType = IndexT;

  BaseCollectionProxy() = default;
  BaseCollectionProxy(BaseCollectionProxy const&) = default;
  BaseCollectionProxy(BaseCollectionProxy&&) = default;
  explicit BaseCollectionProxy(VirtualProxyType const in_proxy);
  BaseCollectionProxy& operator=(
    BaseCollectionProxy const&
  ) = default;

  VirtualProxyType getProxy() const;

  template <typename SerializerT>
  void serialize(SerializerT& s) {
    s | proxy_;
  }

protected:
  VirtualProxyType proxy_ = no_vrt_proxy;
};

}}} /* end namespace vt::vrt::collection */

#include "vt/vrt/proxy/base_collection_proxy.impl.h"

#endif /*INCLUDED_VRT_PROXY_BASE_COLLECTION_PROXY_H*/
