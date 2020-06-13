/*
//@HEADER
// *****************************************************************************
//
//                                 scheduler.h
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

#if !defined INCLUDED_SCHEDULER_SCHEDULER_H
#define INCLUDED_SCHEDULER_SCHEDULER_H

#include "vt/config.h"
#include "vt/scheduler/queue.h"
#include "vt/scheduler/priority_queue.h"
#include "vt/scheduler/prioritized_work_unit.h"
#include "vt/scheduler/work_unit.h"
#include "vt/messaging/message/smart_ptr.h"
#include "vt/timing/timing.h"
#include "vt/trace/trace.h"
#include "vt/trace/trace_user.h"

#include <cassert>
#include <vector>
#include <list>
#include <functional>
#include <memory>

namespace vt {
  void runScheduler();
  void runSchedulerThrough(EpochType epoch);

  void runInEpochRooted(ActionType&& fn);
  void runInEpochCollective(ActionType&& fn);
}

namespace vt { namespace sched {

enum SchedulerEvent {
  BeginIdle          = 0,
  EndIdle            = 1,
  BeginIdleMinusTerm = 2,
  EndIdleMinusTerm   = 3,
  BeginSchedulerLoop = 4,
  EndSchedulerLoop   = 5,

  LastSchedulerEvent = 5,
};

struct Scheduler {
  using SchedulerEventType   = SchedulerEvent;
  using TriggerType          = std::function<void()>;
  using TriggerContainerType = std::list<TriggerType>;
  using EventTriggerContType = std::vector<TriggerContainerType>;

# if backend_check_enabled(priorities)
  using UnitType             = PriorityUnit;
# else
  using UnitType             = Unit;
# endif

  Scheduler();

  virtual void startup() override;
  virtual void finalize() override;

  static void checkTermSingleNode();

  bool shouldCallProgress(
    int32_t processed_since_last_progress, TimeType time_since_last_progress
  ) const;

  void scheduler(bool msg_only = false);
  void runProgress(bool msg_only = false);

  /**
   * \brief Runs the scheduler until a condition is met.
   *
   * Runs the scheduler until a condition is met.
   * This form SHOULD be used instead of "while (..) { runScheduler(..) }"
   * in all cases of nested scheduler loops, such as during a barrier,
   * in order to ensure proper event unwinding and idle time tracking.
   */
  void runSchedulerWhile(std::function<bool()> cond);

  void registerTrigger(SchedulerEventType const& event, TriggerType trigger);
  void registerTriggerOnce(
    SchedulerEventType const& event, TriggerType trigger
  );
  void triggerEvent(SchedulerEventType const& event);

  bool hasSchedRun() const { return has_executed_; }

  void enqueue(ActionType action);
  void enqueue(PriorityType priority, ActionType action);

  void printMemoryUsage();

  template <typename MsgT>
  void enqueue(MsgT* msg, ActionType action);
  template <typename MsgT>
  void enqueue(MsgSharedPtr<MsgT> msg, ActionType action);

  std::size_t workQueueSize() const { return work_queue_.size(); }
  bool workQueueEmpty() const { return work_queue_.empty(); }

  bool isIdle() const { return work_queue_.empty(); }
  bool isIdleMinusTerm() const { return work_queue_.size() == num_term_msgs_; }

private:

  /**
   * \brief Executes a specific work unit.
   */
  void runWorkUnit(UnitType& work);
  bool progressMsgOnlyImpl();
  bool progressImpl();

private:

# if backend_check_enabled(priorities)
  PriorityQueue<UnitType> work_queue_;
# else
  Queue<UnitType> work_queue_;
# endif

  trace::UserEventIDType between_sched_event_type_ = trace::no_user_event_id;

  bool has_executed_      = false;
  bool is_idle            = true;
  bool is_idle_minus_term = true;
  // The depth of work action currently executing.
  unsigned int action_depth_ = 0;

  // User event for tracking between top-level VT scheduler loops.
  std::unique_ptr<trace::TraceScopedEvent> between_sched_event_ = nullptr;

  // The number of termination messages currently in the queue---they weakly
  // imply idleness for the stake of termination
  std::size_t num_term_msgs_ = 0;

  EventTriggerContType event_triggers;
  EventTriggerContType event_triggers_once;

  TimeType last_progress_time_ = 0.0;
  bool progress_time_enabled_ = false;
  int32_t processed_after_last_progress_ = 0;

  std::size_t last_threshold_memory_usage_ = 0;
  std::size_t threshold_memory_usage_ = 0;
  std::size_t last_memory_usage_poll_ = 0;

  // Access to between_sched_event_
  friend void vt::runInEpochRooted(ActionType&& fn);
  friend void vt::runInEpochCollective(ActionType&& fn);
};

}} //end namespace vt::sched

#include "vt/scheduler/scheduler.impl.h"

namespace vt {

extern sched::Scheduler* theSched();

}  //end namespace vt

#endif /*INCLUDED_SCHEDULER_SCHEDULER_H*/
