/*
//@HEADER
// *****************************************************************************
//
//                                jump_state.cc
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

#include "context/fcontext.h"
#include "util.h"

#include <cstdio>
#include <cassert>

using namespace fcontext::examples;

static fcontext_t ctx1;
static int* data_ptr = nullptr;

static void fn1(fcontext_transfer_t t) {
  printf("fn1: t.data=%p\n", t.data);

  assert(t.data == data_ptr);

  puts("fn1 1");
  sleep(100);
  auto t1 = jump_fcontext(t.ctx, nullptr);
  puts("fn1 2");
  sleep(100);
  jump_fcontext(t1.ctx, nullptr);
}

int main(int argc, char** argv) {
  fcontext_stack_t s1 = create_fcontext_stack();

  ctx1 = make_fcontext_stack(s1, fn1);

  int data = 10;
  data_ptr = &data;

  printf("main 1: data=%p\n", static_cast<void*>(&data));
  jump_fcontext(ctx1, &data);
  puts("END");

  destroy_fcontext_stack(s1);

  return 0;
}
