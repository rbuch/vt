/*
//@HEADER
// *****************************************************************************
//
//                                   epoch.h
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

#if !defined INCLUDED_VT_TERMINATION_EPOCH_GUARD_H
#define INCLUDED_VT_TERMINATION_EPOCH_GUARD_H

#include "vt/config.h"

namespace vt {
struct epoch_guard {
public:

  explicit epoch_guard( EpochType ep, bool finish_on_release = false )
    : guarded_epoch_( ep ), finish_on_release_( finish_on_release )
  {
    vtAssert( guarded_epoch_ != no_epoch, "epoch guard cannot take no_epoch" );
    theMsg()->pushEpoch( guarded_epoch_ );
  }

  epoch_guard( const epoch_guard & ) = delete;
  epoch_guard( epoch_guard && ) noexcept = default;

  ~epoch_guard()
  {
    theMsg()->popEpoch( guarded_epoch_ );
    if ( finish_on_release_ )
      finish();
  }

  epoch_guard &operator=( const epoch_guard & ) = delete;
  epoch_guard &operator=( epoch_guard && ) noexcept = default;

  void finish()
  {
    theTerm()->finishedEpoch( guarded_epoch_ );
  }

private:

  EpochType guarded_epoch_ = no_epoch;
  bool finish_on_release_ = false;
};
}

#endif /*INCLUDED_VT_TERMINATION_EPOCH_GUARD_H*/