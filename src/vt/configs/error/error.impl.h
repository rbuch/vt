/*
//@HEADER
// ************************************************************************
//
//                          error.impl.h
//                     vt (Virtual Transport)
//                  Copyright (C) 2018 NTESS, LLC
//
// Under the terms of Contract DE-NA-0003525 with NTESS, LLC,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact darma@sandia.gov
//
// ************************************************************************
//@HEADER
*/

#if !defined INCLUDED_CONFIGS_ERROR_ERROR_IMPL_H
#define INCLUDED_CONFIGS_ERROR_ERROR_IMPL_H

#include "vt/configs/debug/debug_config.h"
#include "vt/configs/types/types_type.h"
#include "vt/configs/error/common.h"
#include "vt/configs/error/error.h"
#include "vt/configs/error/pretty_print_message.h"

#include <string>
#include <tuple>
#include <type_traits>

#include "fmt/format.h"

namespace vt { namespace error {

template <typename... Args>
inline
std::enable_if_t<std::tuple_size<std::tuple<Args...>>::value == 0>
display(std::string const& str, ErrorCodeType error, Args&&... args) {
  std::string const inf = ::fmt::format("FATAL ERROR: {}\n",str);
  return ::vt::abort(inf,error);
}

template <typename... Args>
inline
std::enable_if_t<std::tuple_size<std::tuple<Args...>>::value != 0>
display(std::string const& str, ErrorCodeType error, Args&&... args) {
  std::string const buf = ::fmt::format(str,std::forward<Args>(args)...);
  return ::vt::error::display(buf,error);
}

template <typename... Args>
inline
std::enable_if_t<std::tuple_size<std::tuple<Args...>>::value == 0>
displayLoc(
  std::string const& str, ErrorCodeType error,
  std::string const& file, int const line, std::string const& func,
  Args&&... args
) {
  auto msg = "vtAbort() Invoked";
  auto const inf = debug::stringizeMessage(msg,str,"",file,line,func,error);
  return ::vt::output(inf,error,true,true,true,true);
}

template <typename... Args>
inline
std::enable_if_t<std::tuple_size<std::tuple<Args...>>::value != 0>
displayLoc(
  std::string const& str, ErrorCodeType error,
  std::string const& file, int const line, std::string const& func,
  Args&&... args
) {
  auto msg = "vtAbort() Invoked";
  std::string const buf = ::fmt::format(str,std::forward<Args>(args)...);
  auto const inf = debug::stringizeMessage(msg,buf,"",file,line,func,error);
  return ::vt::output(inf,error,true,true,true,true);
}

}} /* end namespace vt::error */

#endif /*INCLUDED_CONFIGS_ERROR_ERROR_IMPL_H*/
