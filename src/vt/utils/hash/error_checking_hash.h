/*
//@HEADER
// *****************************************************************************
//
//                            error_checking_hash.h
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

#if !defined INCLUDED_VT_UTILS_HASH_ERROR_CHECKING_HASH_H
#define INCLUDED_VT_UTILS_HASH_ERROR_CHECKING_HASH_H

#include "vt/config.h"

namespace vt { namespace util { namespace hash {

/**
 * \struct ErrorCheckingHash
 *
 * \brief Error checking hash functions
 */
struct ErrorCheckingHash {

  /**
   * \brief PJW hash function used for elf
   *
   * Reference: https://en.wikipedia.org/wiki/PJW_hash_function
   *
   * \param[in] s char array
   * \param[in] len length of array
   *
   * \return the hash value
   */
  static uint64_t elf(char const* s, std::size_t len);

};

} /* end namespace hash */

/**
 * \brief Invoke the default hash function for error checking
 *
 * \param[in] s char array
 * \param[in] len length of array
 *
 * \return the hash value
 */
inline uint64_t defaultHash(char const* s, std::size_t len) {
  return hash::ErrorCheckingHash::elf(s, len);
}

}} /* end namespace vt::util */

#endif /*INCLUDED_VT_UTILS_HASH_ERROR_CHECKING_HASH_H*/