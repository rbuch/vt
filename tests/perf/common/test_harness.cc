/*
//@HEADER
// *****************************************************************************
//
//                               test_harness.cc
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

#include "test_harness.h"

#include <vt/collective/collective_ops.h>
#include <vt/scheduler/scheduler.h>
#include <vt/phase/phase_manager.h>
#include <vt/utils/memory/memory_usage.h>

#include <numeric>
#include <fstream>

namespace vt { namespace tests { namespace perf { namespace common {

/////////////////////////////////////////////////
///////////////      HELPERS      ///////////////
/////////////////////////////////////////////////
static void CopyTestData(
  PerfTestHarness::TestResults const& source_time,
  PerfTestHarness::CombinedResults& dest_time,
  PerfTestHarness::MemoryUsage const& source_memory,
  PerfTestHarness::CombinedMemoryUse& dest_memory, NodeType node) {
  for (auto const& test_result : source_time) {
    auto const& test_name = test_result.first;
    auto& dst = dest_time[test_name][node];
    auto const& src = test_result.second;

    std::copy(src.begin(), src.end(), std::back_inserter(dst));
  }

  std::copy(
    source_memory.begin(), source_memory.end(),
    std::back_inserter(dest_memory[node])
  );
}

struct TestMsg : Message {
  vt_msg_serialize_required();
  TestMsg() = default;

  TestMsg(
    PerfTestHarness::TestResults const& results,
    PerfTestHarness::MemoryUsage const& memory, NodeType from_node)
    : results_(results),
      memory_load_(memory),
      from_node_(from_node) { }

  PerfTestHarness::TestResults results_ = {};
  PerfTestHarness::MemoryUsage memory_load_ = {};
  NodeType from_node_ = {};

  template <typename SerializerT>
  void serialize(SerializerT& s) {
    Message::serialize(s);
    s | results_;
    s | memory_load_;
    s | from_node_;
  }
};

void OutputToFile(std::string const& name, std::string const& content) {
  std::ofstream file(fmt::format("{}.csv", name));
  file << content;
}

/////////////////////////////////////////////////
///////////////   TEST HARNESS    ///////////////
/////////////////////////////////////////////////

PerfTestHarness::CombinedResults PerfTestHarness::combined_timings_ = {};
PerfTestHarness::CombinedMemoryUse PerfTestHarness::combined_mem_use_ = {};
std::unordered_map<std::string, StopWatch> PerfTestHarness::timers_ = {};
PerfTestHarness::MemoryUsage PerfTestHarness::memory_use_ = {};
PerfTestHarness::TestResults PerfTestHarness::timings_ = {};
NodeType PerfTestHarness::my_node_ = {};
std::string PerfTestHarness::name_ = {};
vt::util::memory::StatM PerfTestHarness::mem_tracker_ = {};

void PerfTestHarness::SetUp(int argc, char** argv) {
  // pre-parsed args, that may contain performance-test specific args
  std::vector<char*> args_before(argv, argv + argc);

  // original args, minus performance-test specific args
  std::vector<char*> args_after;

  for (auto& arg : args_before) {
    std::string arg_s{arg};
    if (arg_s == "--vt_perf_gen_file") {
      gen_file_ = true;
    } else if (arg_s == "--vt_perf_verbose") {
      verbose_ = true;
    } else {
      args_after.push_back(arg);
    }
  }

  auto custom_argv = args_after.data();
  auto custom_argc = static_cast<int32_t>(args_after.size());
  CollectiveOps::initialize(custom_argc, custom_argv, no_workers, true);
  my_node_ = theContext()->getNode();

  // DumpResults (which uses colorized output) will be called
  // after vt finalizes, so we use preconfig here
  vt::debug::preConfigRef()->colorize_output = true;
}

void PerfTestHarness::TearDown() {
  SpinScheduler();
  CollectiveOps::finalize();

  timings_.clear();
  memory_use_.clear();
}

std::string PerfTestHarness::GetName() const {
  return name_;
}

void PerfTestHarness::DumpResults() const {
  // Only dump results on root node
  if (my_node_ == 0) {
    std::string file_content = "name,node,mean\n";

    for (auto const& per_node_mem : combined_mem_use_) {
      auto const node = per_node_mem.first;
      auto const& memory_use = per_node_mem.second;

      for (auto const mem : memory_use) {
        auto const best_mem = util::memory::getBestMemoryUnit(mem);
        fmt::print(
          "{} Memory usage: {}\n", debug::proc(node),
          debug::emph(fmt::format(
            "{:.6g}{}", std::get<1>(best_mem), std::get<0>(best_mem))
          )
        );
      }
    }

    for (auto& test_run : combined_timings_) {
      auto const name = test_run.first;

      for (auto& per_node_result : test_run.second) {
        auto const node = per_node_result.first;
        auto& timings = per_node_result.second;

        auto const num_timings = timings.size();
        auto const mean =
          std::accumulate(timings.begin(), timings.end(), 0.0) / num_timings;
        std::sort(timings.begin(), timings.end());
        auto const& min = timings.front();
        auto const& max = timings.back();

        file_content.append(fmt::format("{},{},{:.3f}\n", name, node, mean));

        fmt::print(
          "{} Timings for {} (mean value: {} min: {} max: {})\n",
          debug::proc(node), debug::reg(name),
          debug::emph(fmt::format("{:.3f}ms", mean)),
          debug::emph(fmt::format("{:.3f}ms", min)),
          debug::emph(fmt::format("{:.3f}ms", max)));

        if (verbose_) {
          for (uint32_t run_num = 0; run_num < num_timings; ++run_num) {
            fmt::print(
              "{} Run {} -> {}\n", debug::proc(node), run_num,
              debug::emph(fmt::format("{:.3f}ms", timings[run_num])));
          }
        }
      }
    }

    if (gen_file_) {
      OutputToFile(name_, file_content);
    }
  }
}

void PerfTestHarness::AddResult(TestResult const& test_result) {
  timings_[test_result.first].push_back(test_result.second);
}

void PerfTestHarness::SyncResults() {
  // Root node will be responsible for generating the final output
  // so every other node sends its results to it (at the end of test iteration)
  runInEpochCollective([] {
    constexpr auto root_node = 0;
    if (my_node_ != root_node) {
      auto msg = makeMessage<TestMsg>(timings_, memory_use_, my_node_);
      theMsg()->sendMsg<TestMsg, &PerfTestHarness::RecvTestResult>(
        root_node, msg);
    }else{
      // Copy the root node's data to 'combined_timings_'
      CopyTestData(
        timings_, combined_timings_, memory_use_, combined_mem_use_, my_node_);
    }
  });
}

void PerfTestHarness::RecvTestResult(TestMsg* msg) {
  CopyTestData(
    msg->results_, combined_timings_, msg->memory_load_, combined_mem_use_,
    msg->from_node_
  );
}

void PerfTestHarness::SpinScheduler() {
  theSched()->runSchedulerWhile([] { return !rt->isTerminated(); });
}

void PerfTestHarness::BenchmarkPhase(std::string const& prefix)
{
  auto GetName = [prefix] {
    return fmt::format("{}{}", prefix, thePhase()->getCurrentPhase());
  };

  thePhase()->registerHookCollective(phase::PhaseHook::Start, [GetName] {
    timers_[GetName()].Start();
  });

  thePhase()->registerHookCollective(phase::PhaseHook::End, [GetName] {
    auto const name = GetName();
    AddResult({name, timers_[name].Stop()});
    GetMemoryUsage();
  });
}

void PerfTestHarness::GetMemoryUsage() {
  memory_use_.push_back(mem_tracker_.getUsage());
}

}}}} // namespace vt::tests::perf::common