/*
//@HEADER
// *****************************************************************************
//
//                               charmlb_types.h
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

#if !defined INCLUDED_VT_VRT_COLLECTION_BALANCE_CHARMLB_CHARMLB_TYPES_H
#define INCLUDED_VT_VRT_COLLECTION_BALANCE_CHARMLB_CHARMLB_TYPES_H

#include "vt/config.h"
#include "vt/vrt/collection/balance/lb_common.h"

#include <cstdlib>
#include <list>
#include <unordered_map>
#include <map>

namespace vt { namespace vrt { namespace collection { namespace lb {

struct CharmLBTypes {
  using ObjIDType = balance::ElementIDStruct;
  using ObjBinType = int32_t;
  using ObjBinListType = std::list<ObjIDType>;
  using ObjSampleType = std::map<ObjBinType, ObjBinListType>;
  using LoadType = double;
  using LoadProfileType = std::unordered_map<NodeType,LoadType>;
};

struct CharmRecord {
  using ObjType = CharmLBTypes::ObjIDType;
  using LoadType = CharmLBTypes::LoadType;

  CharmRecord(ObjType const& in_obj, LoadType const& in_load)
    : obj_(in_obj), load_(in_load)
  { }

  LoadType getLoad() const { return load_; }
  LoadType getLoad(const int dim) const { return load_; }
  LoadType operator[](const size_t dim) const { return load_; }
  ObjType getObj() const { return obj_; }

private:
  CharmLBTypes::ObjIDType obj_ = { elm::no_element_id, uninitialized_destination };
  LoadType load_ = 0.0f;
};

struct CharmProc {
  CharmProc() = default;
  CharmProc(
    NodeType const& in_node, CharmLBTypes::LoadType const& in_load
  ) : node_(in_node), load_(in_load) {}

  CharmLBTypes::LoadType getLoad() const { return load_; }
  CharmLBTypes::LoadType getLoad(const int d) const { return load_; }
  CharmLBTypes::LoadType operator[](const size_t d) const { return load_; }

  void assign(CharmRecord rec) {
    objs_.push_back(rec.getObj());
    load_ += rec.getLoad();
  }

  NodeType node_ = uninitialized_destination;
  CharmLBTypes::LoadType load_ = 0.0f;
  std::vector<CharmLBTypes::ObjIDType> objs_;
  static constexpr int dimension = 1;
};

template <typename T>
struct CharmCompareLoadMin {
  bool operator()(T const& p1, T const& p2) const {
    return p1.getLoad() > p2.getLoad();
  }
};

template <typename T>
struct CharmCompareLoadMax {
  bool operator()(T const& p1, T const& p2) const {
    return p1.getLoad() < p2.getLoad();
  }
};

}}}} /* end namespace vt::vrt::collection::lb */

#endif /*INCLUDED_VT_VRT_COLLECTION_BALANCE_CHARMLB_CHARMLB_TYPES_H*/
