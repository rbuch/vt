/*
//@HEADER
// *****************************************************************************
//
//                               auto_registry.cc
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

#include "vt/config.h"
#include "vt/registry/auto/auto_registry_common.h"
#include "vt/registry/auto/auto_registry_interface.h"
#include "vt/registry/registry.h"

#include <cassert>

namespace vt { namespace auto_registry {

#if backend_check_enabled(trace_enabled)
trace::TraceEntryIDType theTraceID(
  HandlerType const& handler, RegistryTypeEnum reg_type
) {
  switch (reg_type) {
  case RegistryTypeEnum::RegGeneral: {
    bool const& is_functor = HandlerManagerType::isHandlerFunctor(handler);
    auto const& han_id = HandlerManagerType::getHandlerIdentifier(handler);
    if (not is_functor) {
      using ContType = AutoActiveContainerType;
      return getAutoRegistryGen<ContType>().at(han_id).theTraceID();
    } else {
      using ContType = AutoActiveFunctorContainerType;
      return getAutoRegistryGen<ContType>().at(han_id).theTraceID();
    }
    break;
  }
  case RegistryTypeEnum::RegMap: {
    bool const& is_functor = HandlerManagerType::isHandlerFunctor(handler);
    auto const& han_id = HandlerManagerType::getHandlerIdentifier(handler);
    if (not is_functor) {
      using ContType = AutoActiveMapContainerType;
      return getAutoRegistryGen<ContType>().at(han_id).theTraceID();
    } else {
      using ContType = AutoActiveMapFunctorContainerType;
      return getAutoRegistryGen<ContType>().at(han_id).theTraceID();
    }
    break;
  }
  case RegistryTypeEnum::RegVrt: {
    using ContType = AutoActiveVCContainerType;
    return getAutoRegistryGen<ContType>().at(handler).theTraceID();
    break;
  }
  case RegistryTypeEnum::RegObjGroup: {
    using ContType = AutoActiveObjGroupContainerType;
    return getAutoRegistryGen<ContType>().at(handler).theTraceID();
    break;
  }
  case RegistryTypeEnum::RegVrtCollection: {
    using ContType = AutoActiveCollectionContainerType;
    return getAutoRegistryGen<ContType>().at(handler).theTraceID();
    break;
  }
  case RegistryTypeEnum::RegVrtCollectionMember: {
    using ContType = AutoActiveCollectionMemContainerType;
    return getAutoRegistryGen<ContType>().at(handler).theTraceID();
    break;
  }
  case RegistryTypeEnum::RegVrtCollectionHandle: {
    using ContType = AutoActiveCollectionHanContainerType;
    return getAutoRegistryGen<ContType>().at(handler).theTraceID();
    break;
  }
  case RegistryTypeEnum::RegRDMAGet: {
    using ContType = AutoActiveRDMAGetContainerType;
    return getAutoRegistryGen<ContType>().at(handler).theTraceID();
    break;
  }
  case RegistryTypeEnum::RegRDMAPut: {
    using ContType = AutoActiveRDMAPutContainerType;
    return getAutoRegistryGen<ContType>().at(handler).theTraceID();
    break;
  }
  case RegistryTypeEnum::RegSeed: {
    using ContType = AutoActiveSeedMapContainerType;
    auto const& han_id = HandlerManagerType::getHandlerIdentifier(handler);
    return getAutoRegistryGen<ContType>().at(han_id).theTraceID();
    break;
  }
  default:
    assert(0 && "Should not be reachable");
    break;
  }
}
#endif

}} // end namespace vt::auto_registry
