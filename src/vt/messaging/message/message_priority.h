/*
//@HEADER
// *****************************************************************************
//
//                              message_priority.h
//                       DARMA/vt => Virtual Transport
//
// Copyright 2019-2021 National Technology & Engineering Solutions of Sandia, LLC
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

#if !defined INCLUDED_VT_MESSAGING_MESSAGE_MESSAGE_PRIORITY_H
#define INCLUDED_VT_MESSAGING_MESSAGE_MESSAGE_PRIORITY_H

#include "vt/config.h"
#include "vt/scheduler/priority.h"
#include "vt/scheduler/priority_manip.h"
#include "vt/messaging/envelope.h"

namespace vt { namespace messaging {

template <typename MsgT>
void msgSetPriorityLevel(MsgT ptr, PriorityLevelType level);

template <typename MsgT>
void msgSetPriorityAllLevels(MsgT ptr, PriorityType priority);

template <typename MsgT, typename MsgU>
bool msgIncPriorityLevel(MsgT old_msg, MsgU new_msg);

template <typename MsgU>
void msgSetPriority(MsgU new_msg, PriorityType priority, bool increment_level = false);

template <typename MsgU>
void msgSetPriorityImpl(
  MsgU new_msg, PriorityType new_priority, PriorityType old_priority,
  PriorityLevelType level
);

template <typename MsgT, typename MsgU>
void msgSetPriorityFrom(
  MsgT old_msg, MsgU new_msg, PriorityType priority, bool increment_level = false
);

template <typename MsgT>
void msgSystemSetPriority(MsgT ptr, PriorityType priority);

}} /* end namespace vt::messaging */

#endif /*INCLUDED_VT_MESSAGING_MESSAGE_MESSAGE_PRIORITY_H*/
