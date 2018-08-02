
#if !defined INCLUDED_PIPE_CALLBACK_CB_UNION_CB_RAW_BASE_H
#define INCLUDED_PIPE_CALLBACK_CB_UNION_CB_RAW_BASE_H

#include "config.h"
#include "pipe/callback/cb_union/cb_raw.h"

#include <cassert>

namespace vt { namespace pipe { namespace callback { namespace cbunion {

static struct RawAnonTagType        { } RawAnonTag        { };
static struct RawSendMsgTagType     { } RawSendMsgTag     { };
static struct RawBcastMsgTagType    { } RawBcastMsgTag    { };
static struct RawSendColMsgTagType  { } RawSendColMsgTag  { };
static struct RawBcastColMsgTagType { } RawBcastColMsgTag { };

struct CallbackRawBaseSingle {

  CallbackRawBaseSingle() = default;
  CallbackRawBaseSingle(
    RawSendMsgTagType, PipeType const& in_pipe, HandlerType const& in_handler,
    NodeType const& in_node
  );
  CallbackRawBaseSingle(
    RawBcastMsgTagType, PipeType const& in_pipe, HandlerType const& in_handler,
    bool const& in_inc
  );
  CallbackRawBaseSingle(RawAnonTagType, PipeType const& in_pipe);

  template <typename MsgT>
  void send(MsgT* msg);

  void send();

  template <typename SerializerT>
  void serialize(SerializerT& s);

  PipeType getPipe() const { return pipe_; }

private:
  PipeType pipe_ = no_pipe;
  GeneralCallback cb_;
};

template <typename MsgT>
struct CallbackRawBaseTyped : CallbackRawBaseSingle {

  CallbackRawBaseTyped() = default;

  template <typename SerializerT>
  void serialize(SerializerT& s) {
    CallbackRawBaseSingle::serialize(s);
  }

};

}}}} /* end namespace vt::pipe::callback::cbunion */

#include "pipe/callback/cb_union/cb_raw_base.impl.h"

#endif /*INCLUDED_PIPE_CALLBACK_CB_UNION_CB_RAW_BASE_H*/
