
#include <cstdlib>
#include <iostream>

#include "transport.h"
#include "context/context_vrt.h"
#include "context/context_vrtmanager.h"

using namespace vt;
using namespace vt::vrt;

struct HelloMsg : vt::Message {
  int from;

  explicit HelloMsg(int const& in_from)
      : Message(), from(in_from) {}
};

struct HelloVrtContext : VrtContext {
  int from;

  explicit HelloVrtContext(int const& in_from)
      : VrtContext(), from(in_from) {}
};

static void hello_world(HelloMsg *msg) {
  printf("%d: Hello from node %d\n", theContext->getNode(), msg->from);
}

int main(int argc, char **argv) {
  CollectiveOps::initialize(argc, argv);

  auto const& my_node = theContext->getNode();
  auto const& num_nodes = theContext->getNumNodes();

  if (num_nodes == 1) {
    fprintf(stderr, "Please run with at least two ranks!\n");
    fprintf(stderr, "\t mpirun-mpich-clang -n 2 %s\n", argv[0]);
    exit(1);
  }



  if (my_node == 1) {

    auto vrtc1 = theVrtCManager->constructVrtContext<HelloVrtContext>(10);
    auto vrtc2 = theVrtCManager->constructVrtContext<HelloVrtContext>(20);

    // auto temp1 = theVrtCManager->getVrtContextByID(vrtc1);
    // auto hello1 = static_cast<HelloVrtContext*>(temp1);

    // theVrtCManager->destroyVrtContextByID(vrtc1);
    // auto temp1_ = theVrtCManager->getVrtContextByID(vrtc1);
    // auto hello1_ = static_cast<HelloVrtContext*>(temp1);

    // if (temp1_ != nullptr) {
    //   std::cout << "Boom "<<std::endl;
    // }

    // auto temp2 = theVrtCManager->getVrtContextByID(vrtc2);
    // auto hello2 = static_cast<HelloVrtContext*>(temp2);

    // if (temp2 != nullptr) {
    //   std::cout << "Boom "<<std::endl;
    // }


//    std::cout <<
//
//    std::cout << vrtc1 << std::endl;
//
//    VrtContext_IdType id1 = 5;
//    VrtContext_IdType id2 = 0;
//    auto temp1 = theVrtCManager->getVrtContextByID(id1);
////    auto temp2 = reinterpret_cast<HelloVrtContext*>(theVrtCManager->find(id2));
//
//    if (temp1 == nullptr) {
//      std::cout << "Cool" << std::endl;
//    }
//
//    auto temping = theVrtCManager->getVrtContextByID(id2);
//
//    auto temp2 = static_cast<HelloVrtContext*>(temping);
//
////    if (temp2 != nullptr) {
//      std::cout << temp2->getVrtContextNode() << std::endl;
////    }
//
//    auto vrtc1 = theVrtCManager->newVrtContext();

//    vrtc.printVrtContext();
//    vrtc1.printVrtContext();
//
//    HelloVrtContext my_vrtc(20);
//
//    my_vrtc.printVrtContext();
//
//    auto myHelloVrtC_proxy =
//        theVrtCManager->newVrtContext<HelloVrtContext>(&my_vrtc);
//
//    my_vrtc.printVrtContext();
//    std::cout << my_vrtc.from << std::endl;


//    my_vrtc.printVrtContext();
//    theVrtCManager->newVrtContext(my_vrtc);
//    my_vrtc.printVrtContext();
//    vrtc.printVrtContext();
//    vrtc1.printVrtContext();
////    std::cout << "My node: " << vrtc.getVrtContextNode() << std::endl;
////    std::cout << theVrtCManager_->newVrtContext() << std::endl;
//
//    std::cout << my_vrtc.getVrtContextUId() << std::endl;
//    std::cout << my_vrtc.isCollection() << std::endl;

    HelloMsg *msg = new HelloMsg(my_node);
    theMsg->broadcastMsg<HelloMsg, hello_world>(msg, [=] { delete msg; });
  }

  while (vtIsWorking) {
    runScheduler();
  }

  CollectiveOps::finalize();

  return 0;
}
