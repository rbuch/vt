
#include "common.h"
#include "trace.h"
#include "scheduler.h"

#include <zlib.h>

namespace runtime { namespace trace {

Trace::Trace(std::string const& in_prog_name, std::string const& in_trace_name)
  : prog_name(in_prog_name), trace_name(in_trace_name),
    start_time(get_current_time())
{
  initialize();
}

Trace::Trace() {
  initialize();
}

/*static*/ void
Trace::trace_begin_idle_trigger() {
  backend_enable_if(
    trace_enabled, {
      if (not the_trace->in_idle_event()) {
        the_trace->begin_idle();
      }
    }
  );
}

void
Trace::initialize() {
  traces.reserve(trace_reserve_count);

  the_sched->register_trigger(
    sched::SchedulerEvent::BeginIdle, trace_begin_idle_trigger
  );
}

bool
Trace::in_idle_event() const {
  return idle_begun;
}

void
Trace::setup_names(
  std::string const& in_prog_name, std::string const& in_trace_name
) {
  prog_name = in_prog_name;
  trace_name = in_trace_name;
  start_time = get_current_time();
}

/*virtual*/
Trace::~Trace() {
  write_traces_file();
}

void
Trace::begin_processing(
  TraceEntryIDType const& ep, TraceMsgLenType const& len,
  TraceEventIDType const& event, NodeType const& from_node, double const& time
) {
  auto const& type = trace_type_t::BeginProcessing;
  log_ptr_t log = new log_t(time, ep, type);

  debug_print(
    trace, node,
    "event_start: ep=%lu, event=%d, time=%f\n", ep, event, time
  );

  log->node = from_node;
  log->msg_len = len;
  log->event = event;

  log_event(log);
}

void
Trace::end_processing(
  TraceEntryIDType const& ep, TraceMsgLenType const& len,
  TraceEventIDType const& event, NodeType const& from_node, double const& time
) {
  auto const& type = trace_type_t::EndProcessing;
  log_ptr_t log = new log_t(time, ep, type);

  debug_print(
    trace, node,
    "event_stop: ep=%lu, event=%d, time=%f\n", ep, event, time
  );

  log->node = from_node;
  log->msg_len = len;
  log->event = event;

  log_event(log);
}

void
Trace::begin_idle(double const& time) {
  auto const& type = trace_type_t::BeginIdle;
  log_ptr_t log = new log_t(time, no_trace_entry_id, type);

  debug_print(
    trace, node, "begin_idle: time=%f\n", time
  );

  log->node = the_context->get_node();

  log_event(log);

  idle_begun = true;
}

void
Trace::end_idle(double const& time) {
  auto const& type = trace_type_t::EndIdle;
  log_ptr_t log = new log_t(time, no_trace_entry_id, type);

  debug_print(
    trace, node, "end_idle: time=%f\n", time
  );

  log->node = the_context->get_node();

  log_event(log);

  idle_begun = false;
}

TraceEventIDType
Trace::message_creation(
  TraceEntryIDType const& ep, TraceMsgLenType const& len, double const& time
) {
  auto const& type = trace_type_t::Creation;
  log_ptr_t log = new log_t(time, ep, type);

  log->node = the_context->get_node();
  log->msg_len = len;

  return log_event(log);
}

TraceEventIDType
Trace::message_creation_bcast(
  TraceEntryIDType const& ep, TraceMsgLenType const& len, double const& time
) {
  auto const& type = trace_type_t::CreationBcast;
  log_ptr_t log = new log_t(time, ep, type);

  log->node = the_context->get_node();
  log->msg_len = len;

  return log_event(log);
}

TraceEventIDType
Trace::message_recv(
  TraceEntryIDType const& ep, TraceMsgLenType const& len,
  NodeType const& from_node, double const& time
) {
  auto const& type = trace_type_t::MessageRecv;
  log_ptr_t log = new log_t(time, ep, type);

  log->node = from_node;

  return log_event(log);
}

TraceEventIDType
Trace::log_event(log_ptr_t log) {
  if (not enabled) {
    return 0;
  }

  // close any idle event as soon as we encounter any other type of event
  if (idle_begun and
      log->type != trace_type_t::BeginIdle and
      log->type != trace_type_t::EndIdle) {
    end_idle();
  }

  auto grouped_begin = [&]() -> TraceEventIDType {
    if (not open_events.empty()) {
      traces.push_back(
        new log_t(
          log->time, open_events.top()->ep, trace_type_t::EndProcessing
        )
      );
    }

    // push on open stack.
    open_events.push(log);
    traces.push_back(log);

    return log->event;
  };

  auto grouped_end = [&]() -> TraceEventIDType {
    assert(
      not open_events.empty() and "Stack should be empty"
    );

    assert(
      open_events.top()->ep == log->ep and
      open_events.top()->type == trace_type_t::BeginProcessing and
      "Top event should be correct type and event"
    );

    // match event with the one that this ends
    log->event = open_events.top()->event;

    // set up begin/end links
    open_events.pop();

    traces.push_back(log);

    if (not open_events.empty()) {
      traces.push_back(
        new log_t(
          log->time, open_events.top()->ep, trace_type_t::BeginProcessing
        )
      );
    }

    return log->event;
  };

  auto basic_new_event_create = [&]() -> TraceEventIDType {
    traces.push_back(log);

    log->event = cur_event++;

    return log->event;
  };

  auto basic_no_event_create = [&]() -> TraceEventIDType {
    traces.push_back(log);

    log->event = no_trace_event;

    return log->event;
  };

  switch (log->type) {
  case trace_type_t::BeginProcessing:
    return grouped_begin();
    break;
  case trace_type_t::EndProcessing:
    return grouped_end();
    break;
  case trace_type_t::Creation:
  case trace_type_t::CreationBcast:
  case trace_type_t::MessageRecv:
    return basic_new_event_create();
    break;
  case trace_type_t::BeginIdle:
  case trace_type_t::EndIdle:
    return basic_no_event_create();
    break;
  default:
    assert(0 and "Not implemented");
    return 0;
    break;
  }
}

void
Trace::enable_tracing() {
  enabled = true;
};

void
Trace::disable_tracing() {
  enabled = false;
};

void
Trace::write_traces_file() {
  auto const& node = the_context->get_node();
  auto const& num_nodes = the_context->get_num_nodes();

  debug_print(
    trace, node,
    "write_traces_file: traces.size=%ld, "
    "event_type_container.size=%ld, event_container.size=%ld\n",
    traces.size(),
    trace_cont_t::event_type_container.size(),
    trace_cont_t::event_container.size()
  );

  gzFile file = gzopen(trace_name.c_str(), "wb");
  output_header(node, start_time, file);
  write_log_file(file, traces);
  output_footer(node, start_time, file);
  gzclose(file);

  if (node == designated_root_node) {
    std::ofstream file;
    file.open(prog_name + ".sts");
    output_control_file(file);
    file.flush();
    file.close();
  }
}

void
Trace::write_log_file(gzFile file, trace_container_t const& traces) {
  for (auto&& log : traces) {
    auto const& converted_time = time_to_int(log->time - start_time);

    auto const& type = static_cast<
      std::underlying_type<decltype(log->type)>::type
        >(log->type);

    auto event_iter = trace_cont_t::event_container.find(log->ep);

    assert(
      log->ep == no_trace_entry_id or
      event_iter != trace_cont_t::event_container.end() and
      "Event must exist that was logged"
    );

    auto const& event_seq_id = log->ep == no_trace_entry_id ?
      no_trace_entry_id : event_iter->second.get_event_seq();

    auto const& num_nodes = the_context->get_num_nodes();

    switch (log->type) {
    case trace_type_t::BeginProcessing:
      gzprintf(
        file,
        "%d %d %lu %lld %d %d 0 0 0 0 0 0 0\n",
        type,
        TraceEnvelopeTypes::ForChareMsg,
        event_seq_id,
        converted_time,
        log->event,
        log->node
      );
      break;
    case trace_type_t::EndProcessing:
      gzprintf(
        file,
        "%d %d %lu %lld %d %d 0 0 0 0 0 0 0\n",
        type,
        TraceEnvelopeTypes::ForChareMsg,
        event_seq_id,
        converted_time,
        log->event,
        log->node
      );
      break;
    case trace_type_t::BeginIdle:
      gzprintf(
        file,
        "%d %lld %d\n",
        type,
        converted_time,
        log->node
      );
      break;
    case trace_type_t::EndIdle:
      gzprintf(
        file,
        "%d %lld %d\n",
        type,
        converted_time,
        log->node
      );
      break;
    case trace_type_t::CreationBcast:
      gzprintf(
        file,
        "%d %d %lu %lld %d %d %d %d %d\n",
        type,
        TraceEnvelopeTypes::ForChareMsg,
        event_seq_id,
        converted_time,
        log->event,
        log->node,
        log->msg_len,
        0,
        num_nodes
      );
      break;
    case trace_type_t::Creation:
      gzprintf(
        file,
        "%d %d %lu %lld %d %d %d 0\n",
        type,
        TraceEnvelopeTypes::ForChareMsg,
        event_seq_id,
        converted_time,
        log->event,
        log->node,
        log->msg_len
      );
      break;
    case trace_type_t::MessageRecv:
      assert(0);
      break;
    default:
      assert(0);
    }

    delete log;
  }

  traces.empty();
}

/*static*/ double
Trace::get_current_time() {
  return MPI_Wtime();
}

/*static*/ void
Trace::output_control_file(std::ofstream& file) {
  auto const& node = the_context->get_node();
  auto const& num_nodes = the_context->get_num_nodes();

  file << "PROJECTIONS_ID\n"
       << "VERSION 7.0\n"
       << "TOTAL_PHASES 1\n"
       << "MACHINE unknown\n"
       << "PROCESSORS " << num_nodes << "\n"
       << "TOTAL_CHARES " << trace_cont_t::event_type_container.size() << "\n"
       << "TOTAL_EPS " << trace_cont_t::event_container.size() << "\n"
       << "TOTAL_MSGS 0\n"
       << "TOTAL_PSEUDOS 0\n"
       << "TOTAL_EVENTS 0"
       << std::endl;

  container_event_sorted_t sorted_event;
  container_event_type_sorted_t sorted_event_type;

  for (auto&& elem : trace_cont_t::event_container) {
    sorted_event.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(&elem.second),
      std::forward_as_tuple(true)
    );
  }

  for (auto&& elem : trace_cont_t::event_type_container) {
    sorted_event_type.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(&elem.second),
      std::forward_as_tuple(true)
    );
  }

  for (auto&& event : sorted_event_type) {
    auto const& name = event.first->get_event_name();
    auto const& id = event.first->get_event_seq();

    auto const& out_name = std::string("::" + name);

    file << "CHARE "
         << id << " "
         << out_name << " "
         << std::endl;
  }

  for (auto&& event : sorted_event) {
    auto const& name = event.first->get_event_name();
    auto const& type = event.first->get_event_type_seq();
    auto const& id = event.first->get_event_seq();

    file << "ENTRY CHARE "
         << id << " "
         << name << " "
         << type << " "
         << id << " "
         << std::endl;
  }

  file << "TOTAL_FUNCTIONS 0\n"
       << "END\n"
       << std::endl;
}

/*static*/ void
Trace::output_header(
  NodeType const& node, double const& start, gzFile file
) {
  // Output header for projections file
  gzprintf(file, "PROJECTIONS-RECORD 0\n");
  // '6' means COMPUTATION_BEGIN to Projections: this starts a trace
  gzprintf(file, "6 0\n");
}

/*static*/ void
Trace::output_footer(
  NodeType const& node, double const& start, gzFile file
) {
  // Output footer for projections file, '7' means COMPUTATION_END to
  // Projections
  gzprintf(file, "7 %lld\n", time_to_int(get_current_time() - start));
}

/*static*/ Trace::time_int_t
Trace::time_to_int(double const& time) {
  return static_cast<time_int_t>(time * 1e6);
}

}} //end namespace runtime::trace
