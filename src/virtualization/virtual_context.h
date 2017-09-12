
#if ! defined __RUNTIME_TRANSPORT_VIRTUAL_CONTEXT__
#define __RUNTIME_TRANSPORT_VIRTUAL_CONTEXT__

#include "common.h"
#include "message.h"

namespace runtime {

using CollectionSizeType = uint32_t;
using CollectionIDType = uint32_t;
using CollectionProxyType = uint64_t;

struct VirtualLocMessage : Message {
  CollectionIDType col_id = 0;
};

struct VirutalLoc {
  VirtualContext() = default;
};

struct VirtualLocCollection {
  CollectionSizeType col_size;
};

struct VirtualLocCollectionManager {
  static CollectionProxyType create_virtual_loc(CollectionSizeType col_size) {
    auto const& node = the_context->get_node();
    return (CollectionProxyType)node << (64 - (sizeof() * 8)) | cur_col_id;
  }

private:
  CollectionIDType cur_col_id;
};


} //end namespace runtime

#endif /*__RUNTIME_TRANSPORT_CONTEXT__*/
