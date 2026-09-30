// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from gamesmanros_interfaces:msg/GameState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "gamesmanros_interfaces/msg/detail/game_state__rosidl_typesupport_introspection_c.h"
#include "gamesmanros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "gamesmanros_interfaces/msg/detail/game_state__functions.h"
#include "gamesmanros_interfaces/msg/detail/game_state__struct.h"


// Include directives for member types
// Member `position`
// Member `available_moves`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gamesmanros_interfaces__msg__GameState__init(message_memory);
}

void gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_fini_function(void * message_memory)
{
  gamesmanros_interfaces__msg__GameState__fini(message_memory);
}

size_t gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__size_function__GameState__available_moves(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__get_const_function__GameState__available_moves(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__get_function__GameState__available_moves(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__fetch_function__GameState__available_moves(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__get_const_function__GameState__available_moves(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__assign_function__GameState__available_moves(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__get_function__GameState__available_moves(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__resize_function__GameState__available_moves(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_message_member_array[4] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__GameState, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "available_moves",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__GameState, available_moves),  // bytes offset in struct
    NULL,  // default value
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__size_function__GameState__available_moves,  // size() function pointer
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__get_const_function__GameState__available_moves,  // get_const(index) function pointer
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__get_function__GameState__available_moves,  // get(index) function pointer
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__fetch_function__GameState__available_moves,  // fetch(index, &value) function pointer
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__assign_function__GameState__available_moves,  // assign(index, value) function pointer
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__resize_function__GameState__available_moves  // resize(index) function pointer
  },
  {
    "is_robot_turn",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__GameState, is_robot_turn),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "game_over",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gamesmanros_interfaces__msg__GameState, game_over),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_message_members = {
  "gamesmanros_interfaces__msg",  // message namespace
  "GameState",  // message name
  4,  // number of fields
  sizeof(gamesmanros_interfaces__msg__GameState),
  gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_message_member_array,  // message members
  gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_init_function,  // function to initialize message memory (memory has to be allocated)
  gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_message_type_support_handle = {
  0,
  &gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gamesmanros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gamesmanros_interfaces, msg, GameState)() {
  if (!gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_message_type_support_handle.typesupport_identifier) {
    gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gamesmanros_interfaces__msg__GameState__rosidl_typesupport_introspection_c__GameState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
