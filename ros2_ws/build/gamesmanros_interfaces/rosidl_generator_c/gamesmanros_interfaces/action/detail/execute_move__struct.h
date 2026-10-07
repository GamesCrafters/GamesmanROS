// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gamesmanros_interfaces:action/ExecuteMove.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__STRUCT_H_
#define GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'game_id'
// Member 'move_string'
// Member 'current_position'
// Member 'new_position'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/ExecuteMove in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__action__ExecuteMove_Goal
{
  rosidl_runtime_c__String game_id;
  rosidl_runtime_c__String move_string;
  rosidl_runtime_c__String current_position;
  rosidl_runtime_c__String new_position;
} gamesmanros_interfaces__action__ExecuteMove_Goal;

// Struct for a sequence of gamesmanros_interfaces__action__ExecuteMove_Goal.
typedef struct gamesmanros_interfaces__action__ExecuteMove_Goal__Sequence
{
  gamesmanros_interfaces__action__ExecuteMove_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__action__ExecuteMove_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/ExecuteMove in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__action__ExecuteMove_Result
{
  bool success;
} gamesmanros_interfaces__action__ExecuteMove_Result;

// Struct for a sequence of gamesmanros_interfaces__action__ExecuteMove_Result.
typedef struct gamesmanros_interfaces__action__ExecuteMove_Result__Sequence
{
  gamesmanros_interfaces__action__ExecuteMove_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__action__ExecuteMove_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/ExecuteMove in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__action__ExecuteMove_Feedback
{
  rosidl_runtime_c__String status;
} gamesmanros_interfaces__action__ExecuteMove_Feedback;

// Struct for a sequence of gamesmanros_interfaces__action__ExecuteMove_Feedback.
typedef struct gamesmanros_interfaces__action__ExecuteMove_Feedback__Sequence
{
  gamesmanros_interfaces__action__ExecuteMove_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__action__ExecuteMove_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "gamesmanros_interfaces/action/detail/execute_move__struct.h"

/// Struct defined in action/ExecuteMove in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  gamesmanros_interfaces__action__ExecuteMove_Goal goal;
} gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request;

// Struct for a sequence of gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request.
typedef struct gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__Sequence
{
  gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ExecuteMove in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response;

// Struct for a sequence of gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response.
typedef struct gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__Sequence
{
  gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ExecuteMove in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__action__ExecuteMove_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} gamesmanros_interfaces__action__ExecuteMove_GetResult_Request;

// Struct for a sequence of gamesmanros_interfaces__action__ExecuteMove_GetResult_Request.
typedef struct gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__Sequence
{
  gamesmanros_interfaces__action__ExecuteMove_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "gamesmanros_interfaces/action/detail/execute_move__struct.h"

/// Struct defined in action/ExecuteMove in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__action__ExecuteMove_GetResult_Response
{
  int8_t status;
  gamesmanros_interfaces__action__ExecuteMove_Result result;
} gamesmanros_interfaces__action__ExecuteMove_GetResult_Response;

// Struct for a sequence of gamesmanros_interfaces__action__ExecuteMove_GetResult_Response.
typedef struct gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__Sequence
{
  gamesmanros_interfaces__action__ExecuteMove_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "gamesmanros_interfaces/action/detail/execute_move__struct.h"

/// Struct defined in action/ExecuteMove in the package gamesmanros_interfaces.
typedef struct gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  gamesmanros_interfaces__action__ExecuteMove_Feedback feedback;
} gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage;

// Struct for a sequence of gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage.
typedef struct gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__Sequence
{
  gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__STRUCT_H_
