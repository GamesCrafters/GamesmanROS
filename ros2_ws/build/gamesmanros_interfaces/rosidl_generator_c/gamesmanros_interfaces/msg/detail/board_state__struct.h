// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gamesmanros_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_H_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'occupancy'
#include "rosidl_runtime_c/string.h"
// Member 'pieces'
#include "gamesmanros_interfaces/msg/detail/piece_detection__struct.h"

/// Struct defined in msg/BoardState in the package gamesmanros_interfaces.
/**
  * Full board observation from vision_node.
 */
typedef struct gamesmanros_interfaces__msg__BoardState
{
  std_msgs__msg__Header header;
  /// One character per cell: '1' player one, '2' player two, '-' empty.
  /// Kept for consumers that only need occupancy.
  rosidl_runtime_c__String occupancy;
  /// Per-piece detections, which occupancy cannot express: piece identity and
  /// true position. Used to correct pick coordinates.
  gamesmanros_interfaces__msg__PieceDetection__Sequence pieces;
  /// were all four corner markers visible this frame
  bool board_located;
  /// how many frames this state was agreed across
  int32_t frames_voted;
} gamesmanros_interfaces__msg__BoardState;

// Struct for a sequence of gamesmanros_interfaces__msg__BoardState.
typedef struct gamesmanros_interfaces__msg__BoardState__Sequence
{
  gamesmanros_interfaces__msg__BoardState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__msg__BoardState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_H_
