// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gamesmanros_interfaces:msg/GameState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__STRUCT_H_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
// Member 'available_moves'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GameState in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__msg__GameState
{
  rosidl_runtime_c__String position;
  rosidl_runtime_c__String__Sequence available_moves;
  bool is_robot_turn;
  bool game_over;
} gamesmanros_interfaces__msg__GameState;

// Struct for a sequence of gamesmanros_interfaces__msg__GameState.
typedef struct gamesmanros_interfaces__msg__GameState__Sequence
{
  gamesmanros_interfaces__msg__GameState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__msg__GameState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__STRUCT_H_
