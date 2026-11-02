// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from gamesmanros_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "gamesmanros_interfaces/msg/detail/board_state__rosidl_typesupport_introspection_c.h"
#include "gamesmanros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "gamesmanros_interfaces/msg/detail/board_state__functions.h"
#include "gamesmanros_interfaces/msg/detail/board_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `occupancy`
#include "rosidl_runtime_c/string_functions.h"
// Member `pieces`
#include "gamesmanros_interfaces/msg/piece_detection.h"
// Member `pieces`
#include "gamesmanros_interfaces/msg/detail/piece_detection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gamesmanros_interfaces__msg__BoardState__init(message_memory);
}

void gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_fini_function(void * message_memory)
{
  gamesmanros_interfaces__msg__BoardState__fini(message_memory);
}

size_t gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__size_function__BoardState__pieces(
  const void * untyped_member)
{
  const gamesmanros_interfaces__msg__PieceDetection__Sequence * member =
    (const gamesmanros_interfaces__msg__PieceDetection__Sequence *)(untyped_member);
  return member->size;
}

const void * gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__pieces(
  const void * untyped_member, size_t index)
{
  const gamesmanros_interfaces__msg__PieceDetection__Sequence * member =
    (const gamesmanros_interfaces__msg__PieceDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__pieces(
  void * untyped_member, size_t index)
{
  gamesmanros_interfaces__msg__PieceDetection__Sequence * member =
    (gamesmanros_interfaces__msg__PieceDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__fetch_function__BoardState__pieces(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const gamesmanros_interfaces__msg__PieceDetection * item =
    ((const gamesmanros_interfaces__msg__PieceDetection *)
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__pieces(untyped_member, index));
  gamesmanros_interfaces__msg__PieceDetection * value =
    (gamesmanros_interfaces__msg__PieceDetection *)(untyped_value);
  *value = *item;
}

void gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__assign_function__BoardState__pieces(
  void * untyped_member, size_t index, const void * untyped_value)
{
  gamesmanros_interfaces__msg__PieceDetection * item =
    ((gamesmanros_interfaces__msg__PieceDetection *)
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__pieces(untyped_member, index));
  const gamesmanros_interfaces__msg__PieceDetection * value =
    (const gamesmanros_interfaces__msg__PieceDetection *)(untyped_value);
  *item = *value;
}

bool gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__resize_function__BoardState__pieces(
  void * untyped_member, size_t size)
{
  gamesmanros_interfaces__msg__PieceDetection__Sequence * member =
    (gamesmanros_interfaces__msg__PieceDetection__Sequence *)(untyped_member);
  gamesmanros_interfaces__msg__PieceDetection__Sequence__fini(member);
  return gamesmanros_interfaces__msg__PieceDetection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__BoardState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "occupancy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__BoardState, occupancy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pieces",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__BoardState, pieces),  // bytes offset in struct
    NULL,  // default value
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__size_function__BoardState__pieces,  // size() function pointer
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__pieces,  // get_const(index) function pointer
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__pieces,  // get(index) function pointer
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__fetch_function__BoardState__pieces,  // fetch(index, &value) function pointer
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__assign_function__BoardState__pieces,  // assign(index, value) function pointer
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__resize_function__BoardState__pieces  // resize(index) function pointer
  },
  {
    "board_located",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__BoardState, board_located),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frames_voted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__BoardState, frames_voted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_members = {
  "gamesmanros_interfaces__msg",  // message namespace
  "BoardState",  // message name
  5,  // number of fields
  sizeof(gamesmanros_interfaces__msg__BoardState),
  gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_member_array,  // message members
  gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_init_function,  // function to initialize message memory (memory has to be allocated)
  gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_type_support_handle = {
  0,
  &gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gamesmanros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gamesmanros_interfaces, msg, BoardState)() {
  gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gamesmanros_interfaces, msg, PieceDetection)();
  if (!gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_type_support_handle.typesupport_identifier) {
    gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gamesmanros_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
