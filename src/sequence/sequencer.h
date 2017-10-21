
#if !defined INCLUDED_SEQUENCE_SEQUENCER_H
#define INCLUDED_SEQUENCE_SEQUENCER_H

#include "config.h"
#include "messaging/message.h"
#include "messaging/active.h"
#include "termination/termination.h"
#include "utils/container/concurrent_deque.h"

#include "sequencer_manager.h"
#include "seq_common.h"
#include "seq_context.h"
#include "seq_node.h"
#include "seq_list.h"
#include "seq_state.h"
#include "seq_matcher.h"
#include "seq_action.h"
#include "seq_parallel.h"

#include <unordered_map>
#include <list>
#include <vector>
#include <memory>
#include <cassert>

namespace vt { namespace seq {

template <typename SeqTag, template <typename> class SeqTrigger>
struct TaggedSequencer {
  using SeqType = SeqTag;
  using SeqListType = SeqList;
  using SeqContextType = SeqContext;
  using SeqParallelType = SeqParallel;
  using SeqFunType = SeqListType::SeqFunType;
  using SeqContextPtrType = SeqContextType*;
  using SeqContextContainerType = std::unordered_map<SeqType, SeqNodePtrType>;
  using SeqFuncContainerType = std::vector<FuncType>;
  using SeqCtxFunctionType = std::function<void()>;

  template <typename MessageT>
  using SeqActionType = Action<MessageT>;

  template <typename MessageT>
  using SeqTriggerType = SeqTrigger<MessageT>;

  template <typename T>
  using SeqIDContainerType = std::unordered_map<SeqType, T>;

  template <typename MessageT, ActiveTypedFnType<MessageT>* f>
  using SeqStateMatcherType = SeqMatcher<MessageT, f>;

  using SeqManagerType = SeqManager<SeqTag, SeqTrigger>;

  static std::unique_ptr<SeqManagerType> seq_manager;

  TaggedSequencer() = default;

  // Get the correct ID based on the type
  virtual SeqType getNextID();

  SeqType nextSeq();
  SeqType createSeq();

  static SeqFunType convertSeqFun(SeqType const& id, UserSeqFunType fn);

  void assertValidContext() const;
  bool hasContext() const;

  void sequenced(FuncType const& fn);
  void sequenced(SeqType const& seq_id, FuncIDType const& fn);
  void sequenced(SeqType const& seq_id, FuncType const& fn);

  void for_loop(
    ForIndex const& begin, ForIndex const& end, ForIndex const& stride,
    FuncIndexType fn
  );

  // void forall_loop(
  //   ForIndex const& begin, ForIndex const& end, ForIndex const& stride,
  //   FuncIndexType fn
  // );

  // compile-time list of parallel functions
  template <typename... FnT>
  void parallel(FnT&&... fns);
  template <typename... FnT>
  void parallel(SeqType const& seq_id, FnT&&... fns);

  // runtime dynamic list of parallel functions
  void parallel_lst(SeqFuncContainerType const& fn_list);
  void parallel_lst(SeqType const& seq_id, SeqFuncContainerType const& fn_list);

  void dispatch_parallel(
    bool const& has_context, SeqType const& seq_id, SeqNodePtrType par_node
  );

  void enqueueSeqList(SeqType const& seq_id);
  SeqType getCurrentSeq() const;
  bool scheduler();
  bool isLocalTerm();

  SeqNodePtrType getNode(SeqType const& id) const;
  SeqType getSeqID() const;

  // the general wait function
  template <typename MessageT, ActiveTypedFnType<MessageT>* f>
  void wait_on_trigger(TagType const& tag, SeqActionType<MessageT> action);

  // Wait functions that do not have state (they can be easily migrated if they
  // are registered)
  template <typename MessageT, ActiveTypedFnType<MessageT>* f>
  void wait(SeqTriggerType<MessageT> trigger);
  template <typename MessageT, ActiveTypedFnType<MessageT>* f>
  void wait(TagType const& tag, SeqTriggerType<MessageT> trigger);

  // Closure-based wait functions that have state and cannot be migrated easily
  template <typename MessageT, ActiveTypedFnType<MessageT>* f>
  void wait_closure(TagType const& tag, SeqNonMigratableTriggerType<MessageT> trigger);
  template <typename MessageT, ActiveTypedFnType<MessageT>* f>
  void wait_closure(SeqNonMigratableTriggerType<MessageT> trigger);

  // @todo: should be made thread-safe and thread-local
  bool lookupContextExecute(SeqType const& id, SeqCtxFunctionType c);

  void storeNodeContext(SeqType const& id, SeqNodePtrType node);

  bool executeInNodeContext(
    SeqType const& id, SeqNodePtrType node, SeqCtxFunctionType c,
    bool const suspendable = false
  );
  bool executeSuspendableContext(
    SeqType const& id, SeqNodePtrType node, SeqCtxFunctionType c
  );

public:
  void enqueue(ActionType const& action);

  template <typename MessageT, ActiveTypedFnType<MessageT>* f>
  void sequenceMsg(MessageT* msg);

private:
  SeqListType& getSeqList(SeqType const& seq_id);

protected:
  SeqContext* context_ = nullptr;

private:
  SeqContextContainerType node_lookup_;

  SeqIDContainerType<SeqListType> seq_lookup_;

  util::container::ConcurrentDeque<ActionType> work_deque_;
};

template <typename Fn>
bool executeSeqExpandContext(SeqType const& id, SeqNodePtrType node, Fn&& fn);

using Sequencer = TaggedSequencer<SeqType, SeqMigratableTriggerType>;

#define SEQUENCE_REGISTER_HANDLER(message, handler)                     \
  static void handler(message* m) {                                     \
    theSeq->sequenceMsg<message, handler>(m);                           \
  }

}} //end namespace vt::seq

namespace vt {

extern std::unique_ptr<seq::Sequencer> theSeq;

} //end namespace vt

#include "sequencer.impl.h"

#endif /*INCLUDED_SEQUENCE_SEQUENCER_H*/

