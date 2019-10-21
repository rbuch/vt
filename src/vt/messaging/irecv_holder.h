/*
//@HEADER
// *****************************************************************************
//
//                                irecv_holder.h
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

#if !defined INCLUDED_VT_MESSAGING_IRECV_HOLDER_H
#define INCLUDED_VT_MESSAGING_IRECV_HOLDER_H

#include "vt/config.h"

#include <vector>

namespace vt { namespace messaging {

template <typename T>
struct IRecvHolder {
  IRecvHolder() = default;

  template <typename U>
  void emplace(U&& u) {
    if (holes_.size() > 0) {
      auto const slot = holes_.back();
      holes_.pop_back();
      holder_.emplace(holder_.begin() + slot, std::forward<U>(u));
    } else {
      holder_.emplace_back(std::forward<U>(u));
    }
  }

  template <typename Callable>
  bool testAll(Callable c) {
    bool progress_made = false;
    for (int i = 0; i < holder_.size(); i++) {
      auto& e = holder_[i];
      if (e.valid) {
        int flag = 0;
        MPI_Status stat;
        MPI_Test(&e.req, &flag, &stat);
        if (flag == 1) {
          c(&e);
          progress_made = true;
          e.valid = false;
          holes_.push_back(i);
        }
      }
    }
    return progress_made;
  }

private:
  std::vector<T> holder_;
  std::vector<int> holes_;
};

}} /* end namespace vt::messaging */

#endif /*INCLUDED_VT_MESSAGING_IRECV_HOLDER_H*/
