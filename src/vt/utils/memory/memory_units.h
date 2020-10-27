/*
//@HEADER
// *****************************************************************************
//
//                                memory_units.h
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

#if !defined INCLUDED_VT_UTILS_MEMORY_MEMORY_UNITS_H
#define INCLUDED_VT_UTILS_MEMORY_MEMORY_UNITS_H

#include "vt/config.h"

#include <string>

namespace vt { namespace util { namespace memory {

enum struct MemoryUnitEnum : int8_t {
  Bytes      = 0,
  Kilobytes  = 1,
  Megabytes  = 2,
  Gigabytes  = 3,
  Terabytes  = 4,
  Petabytes  = 5,
  Exabytes   = 6,
  Zettabytes = 7,
  Yottabytes = 8
};

std::string getMemoryUnitName(MemoryUnitEnum unit);
MemoryUnitEnum getUnitFromString(std::string const& unit);
std::tuple<std::string, double> getBestMemoryUnit(std::size_t bytes);

}}} /* end namespace vt::util::memory */

namespace std {

using MemoryUnitType = vt::util::memory::MemoryUnitEnum;

template <>
struct hash<MemoryUnitType> {
  size_t operator()(MemoryUnitType const& in) const {
    using MemoryUnitUnderType = typename std::underlying_type<MemoryUnitType>::type;
    auto const val = static_cast<MemoryUnitUnderType>(in);
    return std::hash<MemoryUnitUnderType>()(val);
  }
};

} /* end namespace std */

#endif /*INCLUDED_VT_UTILS_MEMORY_MEMORY_UNITS_H*/
