
#if ! defined __RUNTIME_TRANSPORT_FUNCTION__
#define __RUNTIME_TRANSPORT_FUNCTION__

#include "common.h"
#include "envelope.h"
#include "message.h"

namespace runtime {

using active_function_t = std::function<void(BaseMessage*)>;

} //end namespace runtime

#endif /*__RUNTIME_TRANSPORT_FUNCTION__*/
