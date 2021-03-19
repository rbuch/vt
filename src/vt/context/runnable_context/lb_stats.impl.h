/*
//@HEADER
// *****************************************************************************
//
//                               lb_stats.impl.h
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

#if !defined INCLUDED_VT_CONTEXT_RUNNABLE_CONTEXT_LB_STATS_IMPL_H
#define INCLUDED_VT_CONTEXT_RUNNABLE_CONTEXT_LB_STATS_IMPL_H

#include "vt/context/runnable_context/lb_stats.h"
#include "vt/messaging/active.h"
#include "vt/vrt/collection/manager.h"
#include "vt/vrt/collection/balance/lb_listener.h"

#include <memory>

namespace vt { namespace ctx {

template <typename ElmT>
template <typename MsgT>
LBStats<ElmT>::LBStats(ElmT* in_elm, MsgT* msg)
  : elm_(in_elm),
    should_instrument_(msg->lbLiteInstrument())
{
  // record the communication stats right away!
  theCollection()->recordStats(elm_, msg);
}

template <typename ElmT>
void LBStats<ElmT>::begin() {
  // Save current element ID context
  prev_elm_id_ = theCollection()->getCurrentContext();

  // Set the current element context for communication stats
  auto const elm_id = elm_->getElmID();
  theCollection()->setCurrentContext(elm_id);

  // Add the listener for the active messenger
  std::unique_ptr<messaging::Listener> listener =
    std::make_unique<vrt::collection::balance::LBListener>(
      [&](NodeType dest, MsgSizeType size, bool bcast){
        auto& stats = elm_->getStats();
        stats.recvToNode(dest, elm_id, size, bcast);
      }
    );
  theMsg()->addSendListener(std::move(listener));

  // record start time
  if (should_instrument_) {
    auto& stats = elm_->getStats();
    stats.startTime();
  }
}

template <typename ElmT>
void LBStats<ElmT>::end() {
  // Set the element ID context back the previous element
  theCollection()->setCurrentContext(prev_elm_id_);

  // Clear the listener now that we are done
  theMsg()->clearListeners();

  // record end time
  if (should_instrument_) {
    auto& stats = elm_->getStats();
    stats.stopTime();
  }
}

template <typename ElmT>
void LBStats<ElmT>::suspend() {
  end();
}

template <typename ElmT>
void LBStats<ElmT>::resume() {
  begin();
}

}} /* end namespace vt::ctx */

#endif /*INCLUDED_VT_CONTEXT_RUNNABLE_CONTEXT_LB_STATS_IMPL_H*/