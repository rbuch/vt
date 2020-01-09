/*
//@HEADER
// *****************************************************************************
//
//                               envelope_type.h
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

#if !defined INCLUDED_MESSAGING_ENVELOPE_ENVELOPE_TYPE_H
#define INCLUDED_MESSAGING_ENVELOPE_ENVELOPE_TYPE_H

#include "vt/config.h"

namespace vt { namespace messaging {

/** \file */

/*
 *  Envelope Type Bits:
 *    001 -> Pipe Message
 *    010 -> Put Message
 *    100 -> Term Message
 *    ...
 */

/// Enum for envelope type bits, used to cast to sub-types and interpret bits
enum eEnvelopeType {
  EnvPipe      = 0,             /**< Whether to interpret group field as pipe */
  EnvPut       = 1,             /**< Whether the envelope has a PUT payload */
  EnvTerm      = 2,             /**< Whether the message is a term control msg */
  EnvBroadcast = 3,             /**< Whether the message is being broadcast */
  EnvEpochType = 4,             /**< Whether the envelope can hold an epoch */
  EnvTagType   = 5,             /**< Whether the envelope can hold a tag */
  EnvPackedPut = 6              /**< Whether the message is packed with data */
};

/// Number of bits allocated for \c eEnvelopeType
static constexpr BitCountType const envelope_num_bits = 8;

}} /* end namespace vt::messaging */

namespace vt {

using eEnvType = messaging::eEnvelopeType;

} /* end namespace vt */

#endif /*INCLUDED_MESSAGING_ENVELOPE_ENVELOPE_TYPE_H*/
