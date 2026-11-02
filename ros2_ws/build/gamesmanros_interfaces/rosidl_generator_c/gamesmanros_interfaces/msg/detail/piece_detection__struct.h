// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gamesmanros_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_H_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PieceDetection in the package gamesmanros_interfaces.
/**
  * One AR-tagged game piece seen on the board.
 */
typedef struct gamesmanros_interfaces__msg__PieceDetection
{
  /// ArUco marker ID — identifies the specific physical piece
  int32_t tag_id;
  /// index into centers.py for this game, or -1 if off-board
  int32_t cell_index;
  /// position in board cell units (same space as centers.py)
  double board_x;
  double board_y;
  /// distance from the ideal centre of cell_index, in mm
  double offset_mm;
} gamesmanros_interfaces__msg__PieceDetection;

// Struct for a sequence of gamesmanros_interfaces__msg__PieceDetection.
typedef struct gamesmanros_interfaces__msg__PieceDetection__Sequence
{
  gamesmanros_interfaces__msg__PieceDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__msg__PieceDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_H_
