/*
//@HEADER
// *****************************************************************************
//
//                           proposed_reassignment.h
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

#if !defined INCLUDED_VT_VRT_COLLECTION_BALANCE_MODEL_PROPOSED_REASSIGNMENT_H
#define INCLUDED_VT_VRT_COLLECTION_BALANCE_MODEL_PROPOSED_REASSIGNMENT_H

#include "vt/config.h"
#include "vt/vrt/collection/balance/model/composed_model.h"

namespace vt { namespace vrt { namespace collection { namespace balance {

class ProposedReassignment;

class ReassignmentIterator : public ObjectIteratorImpl
{
public:
  void operator++() override;
  value_type operator*() const override;
  bool operator==(EndObjectIterator rhs) const override;
  bool operator!=(EndObjectIterator rhs) const override;

  ReassignmentIterator(ObjectIterator &&present,
		       LoadMapObjectIterator arriving,
		       ProposedReassignment *p_in);

private:
  ObjectIterator it_present;
  LoadMapObjectIterator it_arriving;
  ProposedReassignment *p;
};

class ProposedReassignment : public ComposedModel
{
  ProposedReassignment(std::shared_ptr<balance::LoadModel> base,
                       Reassignment reassignment);

  ObjectIterator begin() override;
  int getNumObjects() override;
  TimeType getWork(ElementIDStruct object, PhaseOffset when) override;

 private:
  Reassignment reassignment_;
  friend class ReassignmentIterator;
};

}}}}

#endif