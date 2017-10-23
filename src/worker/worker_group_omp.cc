
#include "config.h"
#include "context/context.h"
#include "collective/collective.h"

#if backend_check_enabled(openmp)

#include "worker/worker_common.h"
#include "worker/worker_group_omp.h"

#include <omp.h>

namespace vt { namespace worker {

WorkerGroupOMP::WorkerGroupOMP()
  : WorkerGroupOMP(num_default_workers)
{ }

WorkerGroupOMP::WorkerGroupOMP(WorkerCountType const& in_num_workers)
  : num_workers_(in_num_workers)
{
  initialize();
}

void WorkerGroupOMP::initialize() {
  worker_state_.resize(num_workers_);
}

/*virtual*/ WorkerGroupOMP::~WorkerGroupOMP() {
  worker_state_.clear();
}

void WorkerGroupOMP::spawnWorkersBlock(WorkerCommFnType comm_fn) {
  debug_print(
    worker, node,
    "Worker group OMP: launching num worker threads=%d, num comm threads=%d\n",
    num_workers_, num_default_comm
  );

  initialized_ = true;

  debug_print(
    worker, node,
    "worker group OMP spawning=%d\n", num_workers_ + 1
  );

  #pragma omp parallel num_threads(num_workers_ + 1)
  {
    WorkerIDType const thd = omp_get_thread_num();
    WorkerIDType const nthds = omp_get_num_threads();

    debug_print(
      worker, node,
      "Worker group OMP: thd=%d, num threads=%d\n", thd, nthds
    );

    if (thd < num_workers_) {
      // For now, all workers to have direct access to the runtime
      // TODO: this needs to change
      CollectiveOps::setCurrentRuntimeTLS();
      //ctx::ContextAttorney::setWorker(thd);

      worker_state_[thd] = std::make_unique<WorkerStateType>(thd, nthds);
      worker_state_[thd]->spawn();
    } else {
      CollectiveOps::setCurrentRuntimeTLS();

      // launch comm function on the main communication thread
      comm_fn();

      // once the comm function exits the program is terminated
      for (auto thd = 0; thd < num_workers_; thd++) {
        debug_print( worker, node, "comm: calling join thd=%d\n", thd );
        worker_state_[thd]->join();
      }
    }
  }
}

void WorkerGroupOMP::spawnWorkers() {
  assert(0 and "Not supported on OMP workers");
}

void WorkerGroupOMP::joinWorkers() {
  for (int i = 0; i < num_workers_; i++) {
    worker_state_[i]->sendTerminateSignal();
  }
}

void WorkerGroupOMP::enqueueAnyWorker(WorkUnitType const& work_unit) {
  assert(initialized_ and "Must be initialized to enqueue");

  worker_state_[0]->enqueue(work_unit);
}

void WorkerGroupOMP::enqueueForWorker(
  WorkerIDType const& worker_id, WorkUnitType const& work_unit
) {
  assert(initialized_ and "Must be initialized to enqueue");
  assert(worker_id < worker_state_.size() and "Worker ID must be valid");

  worker_state_[worker_id]->enqueue(work_unit);
}

void WorkerGroupOMP::enqueueAllWorkers(WorkUnitType const& work_unit) {
  assert(initialized_ and "Must be initialized to enqueue");

  for (auto&& elm : worker_state_) {
    elm->enqueue(work_unit);
  }
}

}} /* end namespace vt::worker */

#endif
