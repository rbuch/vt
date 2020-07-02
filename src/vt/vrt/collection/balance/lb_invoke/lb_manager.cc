/*
//@HEADER
// *****************************************************************************
//
//                                  lb_manager.cc
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

#include "vt/config.h"
#include "vt/context/context.h"
#include "vt/vrt/collection/balance/lb_invoke/lb_manager.h"
#include "vt/vrt/collection/balance/lb_invoke/start_lb_msg.h"
#include "vt/vrt/collection/balance/read_lb.h"
#include "vt/vrt/collection/balance/lb_type.h"
#include "vt/vrt/collection/balance/hierarchicallb/hierlb.h"
#include "vt/vrt/collection/balance/greedylb/greedylb.h"
#include "vt/vrt/collection/balance/rotatelb/rotatelb.h"
#include "vt/vrt/collection/balance/gossiplb/gossiplb.h"
#include "vt/vrt/collection/balance/statsmaplb/statsmaplb.h"
#include "vt/vrt/collection/balance/stats_restart_reader.h"
#include "vt/vrt/collection/balance/zoltanlb/zoltanlb.h"
#include "vt/vrt/collection/balance/randomlb/randomlb.h"
#include "vt/vrt/collection/messages/system_create.h"
#include "vt/vrt/collection/manager.fwd.h"
#include "vt/utils/memory/memory_usage.h"

namespace vt { namespace vrt { namespace collection { namespace balance {

/*static*/ std::unique_ptr<LBManager> LBManager::construct() {
  auto ptr = std::make_unique<LBManager>();
  auto proxy = theObjGroup()->makeCollective<LBManager>(ptr.get());
  proxy.get()->setProxy(proxy);
  return ptr;
}

LBType LBManager::decideLBToRun(PhaseType phase, bool try_file) {
  debug_print(
    lb, node,
    "LBManager::decideLBToRun: phase={}, try_file={}, cached_phase_={}, lb={}\n",
    phase, try_file, cached_phase_, lb_names_[cached_lb_]
  );

  if (phase == cached_phase_) {
    return cached_lb_;
  } else {
    cached_phase_ = phase;
  }

  LBType the_lb = LBType::NoLB;

  // --vt_lb is not enabled, thus do not run the load balancer
  if (not ArgType::vt_lb) {
    return the_lb;
  }

  //--- User-specified map without any change, thus do not run
  if ((ArgType::vt_lb_name == lb_names_[LBType::StatsMapLB]) and
      not theStatsReader()->needsLB(phase)) {
    return LBType::NoLB;
  }

  if (ArgType::vt_lb_file and try_file) {
    bool const has_spec = ReadLBSpec::hasSpec();
    if (has_spec) {
      the_lb = ReadLBSpec::getLB(phase);
    }
  } else {
    vtAssert(ArgType::vt_lb_interval != 0, "LB Interval must not be 0");
    if (phase % ArgType::vt_lb_interval == 0) {
      for (auto&& elm : lb_names_) {
        if (elm.second == ArgType::vt_lb_name) {
          the_lb = elm.first;
          break;
        }
      }
    }
  }

  debug_print(
    lb, node,
    "LBManager::decidedLBToRun: phase={}, return lb_={}\n",
    phase, lb_names_[the_lb]
  );

  cached_lb_ = the_lb;
  return the_lb;
}

template <typename LB>
objgroup::proxy::Proxy<LB>
LBManager::makeLB(MsgSharedPtr<StartLBMsg> msg) {
  auto proxy = theObjGroup()->makeCollective<LB>();
  auto strat = proxy.get();
  strat->init(proxy);
  auto base_proxy = proxy.template registerBaseCollective<lb::BaseLB>();
  auto phase = msg->getPhase();

  EpochType balance_epoch = theTerm()->makeEpochCollective("LBManager::balance_epoch");
  theMsg()->pushEpoch(balance_epoch);
  strat->startLB(phase, base_proxy, theProcStats()->getProcLoad(phase), theProcStats()->getProcComm(phase));
  theMsg()->popEpoch(balance_epoch);
  theTerm()->finishedEpoch(balance_epoch);

  EpochType migrate_epoch = theTerm()->makeEpochCollective("LBManager::migrate_epoch");

  theTerm()->addAction(balance_epoch, [=] {
    debug_print(
      lb, node,
      "LBManager: starting migrations\n"
    );
    theMsg()->pushEpoch(migrate_epoch);
    strat->applyMigrations(strat->getTransfers());
    theMsg()->popEpoch(migrate_epoch);
    theTerm()->finishedEpoch(migrate_epoch);
  });

  theTerm()->addAction(migrate_epoch, [=] {
    debug_print(
      lb, node,
      "LBManager: finished migrations\n"
    );
    theProcStats()->startIterCleanup();
    this->finishedRunningLB(phase);
  });

  destroy_lb_ = [proxy]{ proxy.destroyCollective(); };

  runSchedulerThrough(migrate_epoch);

  return proxy;
}

void LBManager::collectiveImpl(
  PhaseType phase, LBType lb, bool manual, std::size_t num_calls
) {
  debug_print(
    lb, node,
    "collectiveImpl: phase={}, manual={}, num_invocations_={}, num_calls={}, "
    "num_release={}\n",
    phase, manual, num_invocations_, num_calls, num_release_
  );

  num_invocations_++;

  if (num_invocations_ == num_calls) {
    auto const& this_node = theContext()->getNode();

    if (this_node == 0 and not ArgType::vt_lb_quiet) {
      debug_print(
        lb, node,
        "LBManager::collectiveImpl: phase={}, balancer={}, name={}\n",
        phase,
        static_cast<typename std::underlying_type<LBType>::type>(lb),
        lb_names_[lb]
      );
    }

    auto msg = makeMessage<StartLBMsg>(phase);
    switch (lb) {
    case LBType::HierarchicalLB: makeLB<lb::HierarchicalLB>(msg); break;
    case LBType::GreedyLB:       makeLB<lb::GreedyLB>(msg);       break;
    case LBType::RotateLB:       makeLB<lb::RotateLB>(msg);       break;
    case LBType::GossipLB:       makeLB<lb::GossipLB>(msg);       break;
    case LBType::StatsMapLB:     makeLB<lb::StatsMapLB>(msg);     break;
    case LBType::RandomLB:       makeLB<lb::RandomLB>(msg);       break;
#   if backend_check_enabled(zoltan)
    case LBType::ZoltanLB:       makeLB<lb::ZoltanLB>(msg);       break;
#   endif
    case LBType::NoLB:
      vtAssert(false, "LBType::NoLB is not a valid LB for collectiveImpl");
      break;
    default:
      vtAssert(false, "A valid LB must be passed to collectiveImpl");
      break;
    }
  }
}

void LBManager::waitLBCollective() {
  debug_print(
    lb, node,
    "waitLBCollective (begin)\n"
  );

  //
  // The invocation should only happen collectively across the whole all nodes.
  //
  theTerm()->produce();
  theSched()->runSchedulerWhile([this]{ return synced_in_lb_; });
  synced_in_lb_ = true;
  theTerm()->consume();

  debug_print(
    lb, node,
    "waitLBCollective (end)\n"
  );
}

void LBManager::finishedRunningLB(PhaseType phase) {
  debug_print(
    lb, node,
    "finishedRunningLB\n"
  );
  releaseImpl(phase);
}

void LBManager::releaseImpl(PhaseType phase, std::size_t num_calls) {
  debug_print(
    lb, node,
    "releaseImpl: phase={}, num_invocations_={}, num_calls={}, num_release={}\n",
    phase, num_invocations_, num_calls, num_release_
  );

  vtAssert(
    num_calls != 0 or
    num_invocations_ > 0, "Must be automatically invoked to releaseImpl"
  );
  num_release_++;
  if (num_release_ == num_calls or num_release_ == num_invocations_) {
    releaseNow(phase);
  }
}

void LBManager::releaseNow(PhaseType phase) {
  debug_print(lb, node, "releaseNow\n");

  auto this_node = theContext()->getNode();

  if (this_node == 0) {
    vt_print(
      lb,
      "LBManager::releaseNow: finished LB, phase={}, invocations={}\n",
      phase, num_invocations_
    );
  }

  auto msg = makeMessage<CollectionPhaseMsg>();

  // Destruct the objgroup that was used for LB
  if (destroy_lb_ != nullptr) {
    triggerListeners(phase);
    destroy_lb_();
    printMemoryUsage(phase);
    destroy_lb_ = nullptr;
  }
  releaseLBPhase(msg.get());
  synced_in_lb_ = false;
  num_invocations_ = num_release_ = 0;
}

void LBManager::setTraceEnabledNextPhase(PhaseType phase) {
  // Set if tracing is enabled for this next phase. Do this immediately before
  // LB runs so LB is always instrumented as the beginning of the next phase
#if backend_check_enabled(trace_enabled)
  theTrace()->setTraceEnabledCurrentPhase(phase + 1);
# endif
}

void LBManager::flushTraceNextPhase() {
#if backend_check_enabled(trace_enabled)
  theTrace()->flushTracesFile(false);
# endif
}

int LBManager::registerListenerAfterLB(ListenerFnType fn) {
  listeners_.push_back(fn);
  return static_cast<int>(listeners_.size() - 1);
}

void LBManager::unregisterListenerAfterLB(int element) {
  vtAssert(
    listeners_.size() > static_cast<std::size_t>(element), "Listener must exist"
  );
  listeners_[element] = nullptr;
}

void LBManager::printMemoryUsage(PhaseType phase) {
  if (arguments::ArgConfig::vt_print_memory_each_phase) {
    auto this_node = theContext()->getNode();
    if (
      "all" == arguments::ArgConfig::vt_print_memory_node or
      std::to_string(this_node) == arguments::ArgConfig::vt_print_memory_node
    ) {
      if (theMemUsage()->hasWorkingReporter()) {
        auto memory_usage_str = fmt::format(
          "Memory Usage: phase={}: {}\n", phase, theMemUsage()->getUsageAll()
        );
        vt_print(gen, memory_usage_str);
      }
    }
  }
}

void LBManager::triggerListeners(PhaseType phase) {
  for (auto&& l : listeners_) {
    if (l) {
      l(phase);
    }
  }
}

}}}} /* end namespace vt::vrt::collection::balance */