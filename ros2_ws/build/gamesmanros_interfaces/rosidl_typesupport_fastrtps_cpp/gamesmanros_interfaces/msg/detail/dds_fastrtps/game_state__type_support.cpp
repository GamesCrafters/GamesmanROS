// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from gamesmanros_interfaces:msg/GameState.idl
// generated code does not contain a copyright notice
#include "gamesmanros_interfaces/msg/detail/game_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "gamesmanros_interfaces/msg/detail/game_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace gamesmanros_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gamesmanros_interfaces
cdr_serialize(
  const gamesmanros_interfaces::msg::GameState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: position
  cdr << ros_message.position;
  // Member: available_moves
  {
    cdr << ros_message.available_moves;
  }
  // Member: is_robot_turn
  cdr << (ros_message.is_robot_turn ? true : false);
  // Member: game_over
  cdr << (ros_message.game_over ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gamesmanros_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  gamesmanros_interfaces::msg::GameState & ros_message)
{
  // Member: position
  cdr >> ros_message.position;

  // Member: available_moves
  {
    cdr >> ros_message.available_moves;
  }

  // Member: is_robot_turn
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_robot_turn = tmp ? true : false;
  }

  // Member: game_over
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.game_over = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gamesmanros_interfaces
get_serialized_size(
  const gamesmanros_interfaces::msg::GameState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: position
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.position.size() + 1);
  // Member: available_moves
  {
    size_t array_size = ros_message.available_moves.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.available_moves[index].size() + 1);
    }
  }
  // Member: is_robot_turn
  {
    size_t item_size = sizeof(ros_message.is_robot_turn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: game_over
  {
    size_t item_size = sizeof(ros_message.game_over);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gamesmanros_interfaces
max_serialized_size_GameState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: position
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: available_moves
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: is_robot_turn
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: game_over
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = gamesmanros_interfaces::msg::GameState;
    is_plain =
      (
      offsetof(DataType, game_over) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GameState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const gamesmanros_interfaces::msg::GameState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GameState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<gamesmanros_interfaces::msg::GameState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GameState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const gamesmanros_interfaces::msg::GameState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GameState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GameState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GameState__callbacks = {
  "gamesmanros_interfaces::msg",
  "GameState",
  _GameState__cdr_serialize,
  _GameState__cdr_deserialize,
  _GameState__get_serialized_size,
  _GameState__max_serialized_size
};

static rosidl_message_type_support_t _GameState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GameState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace gamesmanros_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_gamesmanros_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<gamesmanros_interfaces::msg::GameState>()
{
  return &gamesmanros_interfaces::msg::typesupport_fastrtps_cpp::_GameState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, gamesmanros_interfaces, msg, GameState)() {
  return &gamesmanros_interfaces::msg::typesupport_fastrtps_cpp::_GameState__handle;
}

#ifdef __cplusplus
}
#endif
