
#include "transport.h"

#include <iostream>

using namespace runtime;

node_t this_node = -1;
node_t num_nodes = -1;
handler_t test_msg_han = 0;

struct TestMsg : runtime::Message {
  int val;
  int val2;

  TestMsg(int const& in_val, int const& in_val2)
    : val(in_val), val2(in_val2)
  { }
};

void handle_test_msg(runtime::Message* in_msg) {
}

void send_to_neighbor() {
  TestMsg* msg = new TestMsg(this_node, num_nodes);

  int const next = this_node+1 < num_nodes ? this_node+1 : 0;

  event_t evt = the_msg->send_msg(next, test_msg_han, msg, [=]{
    std::cout << "deleting msg" << std::endl;
    delete msg;
  });
}

int main(int argc, char** argv) {
  CollectiveOps::initialize_context(argc, argv);

  this_node = the_context->get_node();
  num_nodes = the_context->get_num_nodes();

  std::cout << "this_node=" << this_node << std::endl;

  //test_msg_han = CollectiveOps::register_handler(handle_test_msg);

  test_msg_han = CollectiveOps::register_handler([](runtime::Message* in_msg){
    TestMsg& msg = *static_cast<TestMsg*>(in_msg);

    std::cout
      << "this_node = " << this_node << ", "
      << "from_node = " << msg.val << ", "
      << "num_nodes "  << msg.val2
      << std::endl;

    if (this_node != 0) {
      send_to_neighbor();
    }
  });

  if (this_node == 0) {
    send_to_neighbor();
  }

  while (1) {
    the_msg->scheduler();
  }

  CollectiveOps::finalize_context();
}
