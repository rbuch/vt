
#if ! defined __RUNTIME_TRANSPORT_LOCATION_IMPL__
#define __RUNTIME_TRANSPORT_LOCATION_IMPL__

#include "config.h"
#include "context.h"
#include "active.h"
#include "location_common.h"
#include "location_entity.h"
#include "location.h"

#include <cstdint>
#include <memory>
#include <unordered_map>

namespace vt { namespace location {

template <typename EntityID>
void EntityLocationCoord<EntityID>::registerEntity(
  EntityID const& id, LocMsgActionType msg_action
) {
  auto const& this_node = theContext->getNode();
  auto reg_iter = local_registered_.find(id);

  assert(
    reg_iter == local_registered_.end() and
    "EntityLocationCoord entity should not already be registered"
  );

  debug_print(
    location, node,
    "EntityLocationCoord: registerEntity: id=%d\n", id
  );

  local_registered_.insert(id);

  recs_.insert(id, LocRecType{id, eLocState::Local, this_node});

  if (msg_action != nullptr) {
    assert(
      local_registered_msg_han_.find(id) == local_registered_msg_han_.end() and
      "Entitiy should not exist in local registered msg handler"
    );
    local_registered_msg_han_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(id),
      std::forward_as_tuple(LocEntityMsg{id,msg_action})
    );
  }

  // trigger any pending actions upon registration
  auto pending_lookup_iter =  pending_lookups_.find(id);

  if (pending_lookup_iter != pending_lookups_.end()) {
    for (auto&& pending_action : pending_lookup_iter->second) {
      pending_action(id);
    }
    pending_lookups_.erase(pending_lookup_iter);
  }
}

template <typename EntityID>
void EntityLocationCoord<EntityID>::unregisterEntity(EntityID const& id) {
  auto reg_iter = local_registered_.find(id);

  assert(
    reg_iter != local_registered_.end() and
    "EntityLocationCoord entity must be registered"
  );

  debug_print(
    location, node,
    "EntityLocationCoord: unregisterEntity: id=%d\n", id
  );

  local_registered_.erase(reg_iter);

  bool const& rec_exists = recs_.exists(id);
  if (rec_exists) {
    recs_.remove(id);
  }

  auto reg_msg_han_iter = local_registered_msg_han_.find(id);
  if (reg_msg_han_iter != local_registered_msg_han_.end()) {
    local_registered_msg_han_.erase(reg_msg_han_iter);
  }
}

template <typename EntityID>
void EntityLocationCoord<EntityID>::entityMigrated(
  EntityID const& id, NodeType const& new_node
) {
  auto reg_iter = local_registered_.find(id);

  if (reg_iter != local_registered_.end()) {
    local_registered_.erase(reg_iter);
  }

  recs_.insert(id, LocRecType{id, eLocState::Remote, new_node});
}

template <typename EntityID>
void EntityLocationCoord<EntityID>::insertPendingEntityAction(
  EntityID const& id, NodeActionType action
) {
  // this is the home node and there is no record on this entity
  auto pending_iter = pending_lookups_.find(id);
  if (pending_iter != pending_lookups_.end()) {
    pending_iter->second.push_back(action);
  } else {
    pending_lookups_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(id),
      std::forward_as_tuple(ActionListType{action})
    );
  }
}

template <typename EntityID>
template <typename MessageT>
void EntityLocationCoord<EntityID>::routeMsgEager(
  EntityID const& id, NodeType const& home_node, MessageT* msg
) {
  auto const& this_node = theContext->getNode();

  NodeType route_to_node = uninitialized_destination;

  auto reg_iter = local_registered_.find(id);

  if (reg_iter != local_registered_.end()) {
    recs_.insert(id, LocRecType{id, eLocState::Local, this_node});
    route_to_node = this_node;
  } else {
    bool const& rec_exists = recs_.exists(id);

    if (not rec_exists) {
      if (home_node != this_node) {
        route_to_node = home_node;
      } else {
        route_to_node = this_node;
      }
    } else {
      auto const& rec = recs_.get(id);

      if (rec.isLocal()) {
        route_to_node = this_node;
      } else if (rec.isRemote()) {
        route_to_node = rec.getRemoteNode();
      }
    }
  }

  assert(
    route_to_node != uninitialized_destination and
    "Node to route to must be set by this point"
  );

  return routeMsgNode<MessageT>(id, home_node, route_to_node, msg);
}

template <typename EntityID>
void EntityLocationCoord<EntityID>::getLocation(
  EntityID const& id, NodeType const& home_node, NodeActionType const& action
) {
  auto const& this_node = theContext->getNode();

  auto reg_iter = local_registered_.find(id);

  if (reg_iter != local_registered_.end()) {
    debug_print(
      location, node,
      "EntityLocationCoord: getLocation: id=%d, entity is local\n", id
    );

    action(this_node);
    recs_.insert(id, LocRecType{id, eLocState::Local, this_node});
    return;
  } else {
    bool const& rec_exists = recs_.exists(id);

    debug_print(
      location, node,
      "EntityLocationCoord: getLocation: id=%d, home_node=%d, rec_exists=%s\n",
      id, home_node, print_bool(rec_exists)
    );

    if (not rec_exists) {
      if (home_node != this_node) {
        auto const& event_id = fst_location_event_id++;
        auto msg = new LocMsgType(
          LocManInstType::VirtualLocManInst, id, event_id, this_node, home_node
        );
        theMsg->sendMsg<LocMsgType, getLocationHandler>(
          home_node, msg, [=]{ delete msg; }
        );
        // save a pending action when information about location arrives
        pending_actions_.emplace(
          std::piecewise_construct,
          std::forward_as_tuple(event_id),
          std::forward_as_tuple(PendingType{id,action})
        );
      } else {
        // this is the home node and there is no record on this entity
        insertPendingEntityAction(id, action);
      }
    } else {
      auto const& rec = recs_.get(id);

      if (rec.isLocal()) {
        assert(0 and "Should be registered if this is the case!");
        action(this_node);
      } else if (rec.isRemote()) {
        debug_print(
          location, node,
          "EntityLocationCoord: getLocation: id=%d, entity is remote\n", id
        );

        action(rec.getRemoteNode());
      }
    }
  }
}

template <typename EntityID>
template <typename MessageT>
void EntityLocationCoord<EntityID>::routeMsgNode(
  EntityID const& id, NodeType const& home_node, NodeType const& to_node,
  MessageT* msg
) {
  auto const& this_node = theContext->getNode();
  if (to_node != this_node) {
    auto entity_msg = static_cast<EntityMsgType<MessageT>*>(msg);
    // send to the node discovered by the location manager
    theMsg->sendMsg<MessageT, msgHandler>(to_node, entity_msg);
  } else {
    auto trigger_msg_handler_action = [=](EntityID const& id){
      auto reg_han_iter = local_registered_msg_han_.find(id);
      assert(
        reg_han_iter != local_registered_msg_han_.end() and
        "Message handler must exist for location manager routed msg"
      );
      reg_han_iter->second.applyRegisteredActionMsg(msg);
    };

    auto reg_iter = local_registered_.find(id);
    if (reg_iter != local_registered_.end()) {
      trigger_msg_handler_action(id);
    } else {
      // buffer the message here, the entity will be registered in the future
      insertPendingEntityAction(id, [=](NodeType) {
        trigger_msg_handler_action(id);
      });
    }
  }
}

template <typename EntityID>
template <typename MessageT>
void EntityLocationCoord<EntityID>::routeMsg(
  EntityID const& id, NodeType const& home_node, MessageT* msg
) {
  // set field for location routed message
  msg->entity_id = id;
  msg->home_node = home_node;

  auto const& msg_size = sizeof(*msg);
  bool const& is_large_msg = msg_size > small_msg_max_size;
  bool const& use_eager = not is_large_msg;

  debug_print(
    location, node,
    "routeMsg: id=%d, home=%d, msg_size=%ld, is_large_msg=%s, eager=%s\n",
    id, home_node, msg_size, print_bool(is_large_msg), print_bool(use_eager)
  );

  if (use_eager) {
    routeMsgEager<MessageT>(id, home_node, msg);
  } else {
    // non-eager protocol: get location first then send message after resolution
    getLocation(id, home_node, [=](NodeType node) {
      routeMsgNode<MessageT>(id, home_node, node, msg);
    });
  }
}

template <typename EntityID>
void EntityLocationCoord<EntityID>::updatePendingRequest(
  LocEventID const& event_id, NodeType const& node
) {
  auto pending_iter = pending_actions_.find(event_id);

  assert(
    pending_iter != pending_actions_.end() and "Event must exist in pending"
  );

  auto const& entity = pending_iter->second.entity;

  debug_print(
    location, node,
    "EntityLocationCoord: updatePendingRequest: event_id=%lld, entity=%d, "
    "node=%d\n",
    event_id, entity, node
  );

  recs_.insert(entity, LocRecType{entity, eLocState::Remote, node});

  pending_iter->second.applyNodeAction(node);

  pending_actions_.erase(pending_iter);
}

template <typename EntityID>
void EntityLocationCoord<EntityID>::printCurrentCache() const {
  recs_.printCache();
}

template <typename EntityID>
template <typename MessageT>
/*static*/ void EntityLocationCoord<EntityID>::msgHandler(MessageT* msg) {
  auto const& entity_id = msg->entity_id;
  auto const& home_node = msg->home_node;

  // increase the reference count
  messageRef(msg);

  auto const& loc = theLocMan->virtual_loc;
  loc->routeMsg(entity_id, home_node, msg);
}

template <typename EntityID>
/*static*/ void EntityLocationCoord<EntityID>::getLocationHandler(LocMsgType* msg) {
  auto const& event_id = msg->loc_event;
  auto const& inst = msg->loc_man_inst;
  auto const& entity = msg->entity;
  auto const& home_node = msg->home_node;
  auto const& ask_node = msg->ask_node;

  if (inst == LocManInstType::VirtualLocManInst) {
    auto const& loc = theLocMan->virtual_loc;
    loc->getLocation(entity, home_node, [=](NodeType node) {
      auto msg = new LocMsgType(
        LocManInstType::VirtualLocManInst, entity, event_id, ask_node, home_node
      );
      msg->setResolvedNode(node);
      theMsg->sendMsg<LocMsgType, updateLocation>(
        ask_node, msg, [=]{ delete msg; }
      );
    });
  }
}

template <typename EntityID>
/*static*/ void EntityLocationCoord<EntityID>::updateLocation(LocMsgType* msg) {
  auto const& event_id = msg->loc_event;
  auto const& inst = msg->loc_man_inst;
  auto const& entity = msg->entity;

  if (inst == LocManInstType::VirtualLocManInst) {
    auto const& loc = theLocMan->virtual_loc;
    loc->updatePendingRequest(event_id, msg->resolved_node);
  }
}

}} // end namespace vt::location

#endif /*__RUNTIME_TRANSPORT_LOCATION_IMPL__*/
