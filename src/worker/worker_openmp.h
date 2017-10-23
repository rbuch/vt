
#if !defined INCLUDED_WORKER_WORKER_OPENMP_H
#define INCLUDED_WORKER_WORKER_OPENMP_H

#include "config.h"
#include "worker/worker_common.h"
#include "worker/worker_types.h"
#include "utils/container/concurrent_deque.h"

#include <thread>
#include <functional>
#include <memory>
#include <atomic>

namespace vt { namespace worker {

struct OMPWorker {
  using WorkerFunType = std::function<void()>;
  using WorkUnitContainerType = util::container::ConcurrentDeque<WorkUnitType>;

  OMPWorker(WorkerIDType const& in_worker_id_, WorkerCountType const& in_num_thds);
  OMPWorker(OMPWorker const&) = delete;

  void spawn();
  void join();
  void dispatch(WorkerFunType fun);
  void enqueue(WorkUnitType const& work_unit);
  void sendTerminateSignal();

private:
  void scheduler();

private:
  bool should_terminate_= false;
  WorkerIDType worker_id_ = no_worker_id;
  WorkerCountType num_thds_ = no_workers;
  WorkUnitContainerType work_queue_;
};

}} /* end namespace vt::worker */

#if backend_check_enabled(detector)
  #include "worker/worker_traits.h"

  namespace vt { namespace worker {

  static_assert(
    WorkerTraits<OMPWorker>::is_worker,
    "vt::worker::Worker must follow the Worker concept"
  );

  }} /* end namespace vt::worker */
#endif

#endif /*INCLUDED_WORKER_WORKER_OPENMP_H*/
