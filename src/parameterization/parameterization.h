
#if ! defined __RUNTIME_TRANSPORT_PARAMETERIZATION__
#define __RUNTIME_TRANSPORT_PARAMETERIZATION__

#include "common.h"
#include "message.h"
#include "active.h"
#include "auto_registry_interface.h"

namespace runtime { namespace param {

using handler_manager_t = runtime::HandlerManager;

template <typename... Args>
using multi_param_t = void(*)(Args...);

template<typename T, T value>
struct NonType {};

#define param_function_rhs(value) runtime::param::NonType<decltype(value),(value)>()
#define param_function(value) decltype(value),(value)

template <typename Tuple>
struct DataMsg : runtime::Message {
  Tuple tup;
  handler_t sub_han = uninitialized_handler;

  DataMsg(handler_t const& in_sub_han, Tuple&& a)
    : Message(), tup(std::forward<Tuple>(a)), sub_han(in_sub_han)
  { }

  template <typename... Args>
  DataMsg(handler_t const& in_sub_han, Args&&... a)
    : Message(), tup(std::forward<Args>(a)...), sub_han(in_sub_han)
  { }
};

template <typename Function, typename Tuple, size_t... I>
auto call(Function f, Tuple&& t, std::index_sequence<I...>) {
  return f(
    std::forward<typename std::tuple_element<I,Tuple>::type>(
      std::get<I>(t)
    )...
  );
}

template <typename FnT, typename... Args>
void get_fn_sig(std::tuple<Args...>&& tup, FnT fn) {
  using tuple_type_t = typename std::decay<decltype(tup)>::type;
  static constexpr auto size = std::tuple_size<tuple_type_t>::value;
  auto typed_fn = reinterpret_cast<multi_param_t<Args...>>(fn);
  call(typed_fn, std::forward<std::tuple<Args...>>(tup), std::make_index_sequence<size>{});
}

template <typename Tuple>
static void data_message_handler(DataMsg<Tuple>* msg) {
  if (handler_manager_t::is_handler_functor(msg->sub_han)) {
    auto fn = auto_registry::get_auto_handler_functor(msg->sub_han);
    get_fn_sig(std::forward<Tuple>(msg->tup), fn);
  } else {
    // regular active function
    auto fn = auto_registry::get_auto_handler(msg->sub_han);
    debug_print(
      param, node,
      "data_message_handler: id=%d, fn=%p\n", msg->sub_han, fn
    );
    get_fn_sig(std::forward<Tuple>(msg->tup), fn);
  }
}

template <bool...> struct bool_pack;
template <bool... bs>
using all_true = std::is_same<bool_pack<bs..., true>, bool_pack<true, bs...>>;

struct Param {

  template <typename... Args>
  event_t send_data_tuple(
    node_t const& dest, handler_t const& han, std::tuple<Args...>&& tup
  ) {
    static_check_copyable<Args...>();

    using tuple_type_t = typename std::decay<decltype(tup)>::type;

    DataMsg<tuple_type_t>* m = new DataMsg<tuple_type_t>(
      han, std::forward<std::tuple<Args...>>(tup)
    );

    return the_msg->send_msg<DataMsg<tuple_type_t>, data_message_handler>(
      dest, m, [=]{ delete m; }
    );
  }

  template <typename... Args>
  void static_check_copyable() {
    using cond = all_true<std::is_trivially_copyable<Args>::value...>;

    // static_assert(
    //   std::is_same<typename cond::type,std::true_type>::value == true,
    //   "All types passed for parameterization must be trivially copyable"
    // );
  }

  template <typename DataMsg>
  event_t send_data_msg(node_t const& dest, handler_t const& han, DataMsg* m) {
    return the_msg->send_msg<DataMsg, data_message_handler>(
      dest, m, [=]{ delete m; }
    );
  }

  template <typename T, T value, typename Tuple>
  event_t send_data(
    node_t const& dest, Tuple tup, NonType<T, value> non = NonType<T,value>()
  ) {
    auto const& han = auto_registry::make_auto_handler<T,value>();
    return send_data_tuple(dest, han, std::forward<Tuple>(tup));
  }

  template <typename T, T value, typename... Args>
  event_t send_data(node_t const& dest, NonType<T, value> non, Args&&... a) {
    auto const& han = auto_registry::make_auto_handler<T,value>();

    static_check_copyable<Args...>();

    using tuple_type_t = std::tuple<Args...>;

    DataMsg<tuple_type_t>* m = new DataMsg<tuple_type_t>(
      han, std::forward<Args>(a)...
    );

    return send_data_msg(dest, han, m);
  }

  template <typename T, T value, typename... Args>
  event_t send_data(node_t const& dest, Args&&... a) {
    return send_data(dest, NonType<T,value>(), std::forward<Args>(a)...);
  }

  /*
   * Functor variants
   */

  template <typename FunctorT, typename... Args>
  event_t send_data_helper_functor(
    node_t const& dest, std::tuple<Args...>&& tup
  ) {
    using message_t = runtime::BaseMessage;
    auto const& han = auto_registry::make_auto_handler_functor<FunctorT,Args...>();
    return send_data_tuple(dest, han, std::forward<std::tuple<Args...>>(tup));
  }

  template <typename FunctorT, typename Tuple>
  event_t send_data(node_t const& dest, Tuple tup) {
    return send_data_helper_functor<FunctorT>(dest, std::forward<Tuple>(tup));
  }

  template <typename FunctorT, typename... Args>
  event_t send_data(node_t const& dest, Args&&... a) {
    static_check_copyable<Args...>();

    auto const& han = auto_registry::make_auto_handler_functor<FunctorT,Args...>();

    using tuple_type_t = std::tuple<Args...>;

    DataMsg<tuple_type_t>* m = new DataMsg<tuple_type_t>(
      han, std::forward<Args>(a)...
    );

    return send_data_msg(dest, han, m);
  }
};

}} //end namespace runtime::param

namespace runtime {

extern std::unique_ptr<param::Param> the_param;

template <typename... Args>
std::tuple<Args...> build_data(Args&&... a) {
  return std::tuple<Args...>(std::forward<Args>(a)...);
}

} //end namespace runtime

#endif /*__RUNTIME_TRANSPORT_PARAMETERIZATION__*/
