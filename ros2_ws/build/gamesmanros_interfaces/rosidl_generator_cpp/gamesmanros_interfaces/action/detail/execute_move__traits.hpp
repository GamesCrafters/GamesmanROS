// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gamesmanros_interfaces:action/ExecuteMove.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__TRAITS_HPP_
#define GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gamesmanros_interfaces/action/detail/execute_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace gamesmanros_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMove_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: game_id
  {
    out << "game_id: ";
    rosidl_generator_traits::value_to_yaml(msg.game_id, out);
    out << ", ";
  }

  // member: move_string
  {
    out << "move_string: ";
    rosidl_generator_traits::value_to_yaml(msg.move_string, out);
    out << ", ";
  }

  // member: current_position
  {
    out << "current_position: ";
    rosidl_generator_traits::value_to_yaml(msg.current_position, out);
    out << ", ";
  }

  // member: new_position
  {
    out << "new_position: ";
    rosidl_generator_traits::value_to_yaml(msg.new_position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: game_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "game_id: ";
    rosidl_generator_traits::value_to_yaml(msg.game_id, out);
    out << "\n";
  }

  // member: move_string
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_string: ";
    rosidl_generator_traits::value_to_yaml(msg.move_string, out);
    out << "\n";
  }

  // member: current_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_position: ";
    rosidl_generator_traits::value_to_yaml(msg.current_position, out);
    out << "\n";
  }

  // member: new_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "new_position: ";
    rosidl_generator_traits::value_to_yaml(msg.new_position, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::action::ExecuteMove_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::action::ExecuteMove_Goal & msg)
{
  return gamesmanros_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_Goal>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_Goal";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_Goal>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_Goal";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<gamesmanros_interfaces::action::ExecuteMove_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace gamesmanros_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMove_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::action::ExecuteMove_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::action::ExecuteMove_Result & msg)
{
  return gamesmanros_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_Result>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_Result";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_Result>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_Result";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gamesmanros_interfaces::action::ExecuteMove_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace gamesmanros_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMove_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::action::ExecuteMove_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::action::ExecuteMove_Feedback & msg)
{
  return gamesmanros_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_Feedback>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_Feedback";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_Feedback>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_Feedback";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<gamesmanros_interfaces::action::ExecuteMove_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "gamesmanros_interfaces/action/detail/execute_move__traits.hpp"

namespace gamesmanros_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMove_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request & msg)
{
  return gamesmanros_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_SendGoal_Request";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace gamesmanros_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMove_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response & msg)
{
  return gamesmanros_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_SendGoal_Response";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_SendGoal>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_SendGoal";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_SendGoal>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_SendGoal";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>::value &&
    has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>::value &&
    has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<gamesmanros_interfaces::action::ExecuteMove_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace gamesmanros_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMove_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::action::ExecuteMove_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::action::ExecuteMove_GetResult_Request & msg)
{
  return gamesmanros_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_GetResult_Request";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_GetResult_Request";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "gamesmanros_interfaces/action/detail/execute_move__traits.hpp"

namespace gamesmanros_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMove_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::action::ExecuteMove_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::action::ExecuteMove_GetResult_Response & msg)
{
  return gamesmanros_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_GetResult_Response";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_GetResult_Response";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_Result>::value> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_Result>::value> {};

template<>
struct is_message<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_GetResult>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_GetResult";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_GetResult>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_GetResult";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>::value &&
    has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>::value &&
    has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>::value
  >
{
};

template<>
struct is_service<gamesmanros_interfaces::action::ExecuteMove_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "gamesmanros_interfaces/action/detail/execute_move__traits.hpp"

namespace gamesmanros_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteMove_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage & msg)
{
  return gamesmanros_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage>()
{
  return "gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage";
}

template<>
inline const char * name<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage>()
{
  return "gamesmanros_interfaces/action/ExecuteMove_FeedbackMessage";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<gamesmanros_interfaces::action::ExecuteMove_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<gamesmanros_interfaces::action::ExecuteMove_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<gamesmanros_interfaces::action::ExecuteMove>
  : std::true_type
{
};

template<>
struct is_action_goal<gamesmanros_interfaces::action::ExecuteMove_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<gamesmanros_interfaces::action::ExecuteMove_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<gamesmanros_interfaces::action::ExecuteMove_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__TRAITS_HPP_
