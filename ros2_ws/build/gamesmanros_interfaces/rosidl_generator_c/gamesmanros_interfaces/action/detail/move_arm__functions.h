// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from gamesmanros_interfaces:action/MoveArm.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_ARM__FUNCTIONS_H_
#define GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_ARM__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "gamesmanros_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "gamesmanros_interfaces/action/detail/move_arm__struct.h"

/// Initialize action/MoveArm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__action__MoveArm_Goal
 * )) before or use
 * gamesmanros_interfaces__action__MoveArm_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Goal__init(gamesmanros_interfaces__action__MoveArm_Goal * msg);

/// Finalize action/MoveArm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Goal__fini(gamesmanros_interfaces__action__MoveArm_Goal * msg);

/// Create action/MoveArm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__action__MoveArm_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_Goal *
gamesmanros_interfaces__action__MoveArm_Goal__create();

/// Destroy action/MoveArm message.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Goal__destroy(gamesmanros_interfaces__action__MoveArm_Goal * msg);

/// Check for action/MoveArm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Goal__are_equal(const gamesmanros_interfaces__action__MoveArm_Goal * lhs, const gamesmanros_interfaces__action__MoveArm_Goal * rhs);

/// Copy a action/MoveArm message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Goal__copy(
  const gamesmanros_interfaces__action__MoveArm_Goal * input,
  gamesmanros_interfaces__action__MoveArm_Goal * output);

/// Initialize array of action/MoveArm messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__action__MoveArm_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Goal__Sequence__init(gamesmanros_interfaces__action__MoveArm_Goal__Sequence * array, size_t size);

/// Finalize array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Goal__Sequence__fini(gamesmanros_interfaces__action__MoveArm_Goal__Sequence * array);

/// Create array of action/MoveArm messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__action__MoveArm_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_Goal__Sequence *
gamesmanros_interfaces__action__MoveArm_Goal__Sequence__create(size_t size);

/// Destroy array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Goal__Sequence__destroy(gamesmanros_interfaces__action__MoveArm_Goal__Sequence * array);

/// Check for action/MoveArm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Goal__Sequence__are_equal(const gamesmanros_interfaces__action__MoveArm_Goal__Sequence * lhs, const gamesmanros_interfaces__action__MoveArm_Goal__Sequence * rhs);

/// Copy an array of action/MoveArm messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Goal__Sequence__copy(
  const gamesmanros_interfaces__action__MoveArm_Goal__Sequence * input,
  gamesmanros_interfaces__action__MoveArm_Goal__Sequence * output);

/// Initialize action/MoveArm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__action__MoveArm_Result
 * )) before or use
 * gamesmanros_interfaces__action__MoveArm_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Result__init(gamesmanros_interfaces__action__MoveArm_Result * msg);

/// Finalize action/MoveArm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Result__fini(gamesmanros_interfaces__action__MoveArm_Result * msg);

/// Create action/MoveArm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__action__MoveArm_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_Result *
gamesmanros_interfaces__action__MoveArm_Result__create();

/// Destroy action/MoveArm message.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Result__destroy(gamesmanros_interfaces__action__MoveArm_Result * msg);

/// Check for action/MoveArm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Result__are_equal(const gamesmanros_interfaces__action__MoveArm_Result * lhs, const gamesmanros_interfaces__action__MoveArm_Result * rhs);

/// Copy a action/MoveArm message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Result__copy(
  const gamesmanros_interfaces__action__MoveArm_Result * input,
  gamesmanros_interfaces__action__MoveArm_Result * output);

/// Initialize array of action/MoveArm messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__action__MoveArm_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Result__Sequence__init(gamesmanros_interfaces__action__MoveArm_Result__Sequence * array, size_t size);

/// Finalize array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Result__Sequence__fini(gamesmanros_interfaces__action__MoveArm_Result__Sequence * array);

/// Create array of action/MoveArm messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__action__MoveArm_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_Result__Sequence *
gamesmanros_interfaces__action__MoveArm_Result__Sequence__create(size_t size);

/// Destroy array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Result__Sequence__destroy(gamesmanros_interfaces__action__MoveArm_Result__Sequence * array);

/// Check for action/MoveArm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Result__Sequence__are_equal(const gamesmanros_interfaces__action__MoveArm_Result__Sequence * lhs, const gamesmanros_interfaces__action__MoveArm_Result__Sequence * rhs);

/// Copy an array of action/MoveArm messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Result__Sequence__copy(
  const gamesmanros_interfaces__action__MoveArm_Result__Sequence * input,
  gamesmanros_interfaces__action__MoveArm_Result__Sequence * output);

/// Initialize action/MoveArm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__action__MoveArm_Feedback
 * )) before or use
 * gamesmanros_interfaces__action__MoveArm_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Feedback__init(gamesmanros_interfaces__action__MoveArm_Feedback * msg);

/// Finalize action/MoveArm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Feedback__fini(gamesmanros_interfaces__action__MoveArm_Feedback * msg);

/// Create action/MoveArm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__action__MoveArm_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_Feedback *
gamesmanros_interfaces__action__MoveArm_Feedback__create();

/// Destroy action/MoveArm message.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Feedback__destroy(gamesmanros_interfaces__action__MoveArm_Feedback * msg);

/// Check for action/MoveArm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Feedback__are_equal(const gamesmanros_interfaces__action__MoveArm_Feedback * lhs, const gamesmanros_interfaces__action__MoveArm_Feedback * rhs);

/// Copy a action/MoveArm message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Feedback__copy(
  const gamesmanros_interfaces__action__MoveArm_Feedback * input,
  gamesmanros_interfaces__action__MoveArm_Feedback * output);

/// Initialize array of action/MoveArm messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__action__MoveArm_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__init(gamesmanros_interfaces__action__MoveArm_Feedback__Sequence * array, size_t size);

/// Finalize array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__fini(gamesmanros_interfaces__action__MoveArm_Feedback__Sequence * array);

/// Create array of action/MoveArm messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_Feedback__Sequence *
gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__create(size_t size);

/// Destroy array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__destroy(gamesmanros_interfaces__action__MoveArm_Feedback__Sequence * array);

/// Check for action/MoveArm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__are_equal(const gamesmanros_interfaces__action__MoveArm_Feedback__Sequence * lhs, const gamesmanros_interfaces__action__MoveArm_Feedback__Sequence * rhs);

/// Copy an array of action/MoveArm messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__copy(
  const gamesmanros_interfaces__action__MoveArm_Feedback__Sequence * input,
  gamesmanros_interfaces__action__MoveArm_Feedback__Sequence * output);

/// Initialize action/MoveArm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Request
 * )) before or use
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__init(gamesmanros_interfaces__action__MoveArm_SendGoal_Request * msg);

/// Finalize action/MoveArm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__fini(gamesmanros_interfaces__action__MoveArm_SendGoal_Request * msg);

/// Create action/MoveArm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_SendGoal_Request *
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__create();

/// Destroy action/MoveArm message.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__destroy(gamesmanros_interfaces__action__MoveArm_SendGoal_Request * msg);

/// Check for action/MoveArm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__are_equal(const gamesmanros_interfaces__action__MoveArm_SendGoal_Request * lhs, const gamesmanros_interfaces__action__MoveArm_SendGoal_Request * rhs);

/// Copy a action/MoveArm message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__copy(
  const gamesmanros_interfaces__action__MoveArm_SendGoal_Request * input,
  gamesmanros_interfaces__action__MoveArm_SendGoal_Request * output);

/// Initialize array of action/MoveArm messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__init(gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__fini(gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence * array);

/// Create array of action/MoveArm messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence *
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__destroy(gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence * array);

/// Check for action/MoveArm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__are_equal(const gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence * lhs, const gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/MoveArm messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__copy(
  const gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence * input,
  gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence * output);

/// Initialize action/MoveArm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Response
 * )) before or use
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__init(gamesmanros_interfaces__action__MoveArm_SendGoal_Response * msg);

/// Finalize action/MoveArm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__fini(gamesmanros_interfaces__action__MoveArm_SendGoal_Response * msg);

/// Create action/MoveArm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_SendGoal_Response *
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__create();

/// Destroy action/MoveArm message.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__destroy(gamesmanros_interfaces__action__MoveArm_SendGoal_Response * msg);

/// Check for action/MoveArm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__are_equal(const gamesmanros_interfaces__action__MoveArm_SendGoal_Response * lhs, const gamesmanros_interfaces__action__MoveArm_SendGoal_Response * rhs);

/// Copy a action/MoveArm message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__copy(
  const gamesmanros_interfaces__action__MoveArm_SendGoal_Response * input,
  gamesmanros_interfaces__action__MoveArm_SendGoal_Response * output);

/// Initialize array of action/MoveArm messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__init(gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__fini(gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence * array);

/// Create array of action/MoveArm messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence *
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__destroy(gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence * array);

/// Check for action/MoveArm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__are_equal(const gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence * lhs, const gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/MoveArm messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__copy(
  const gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence * input,
  gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence * output);

/// Initialize action/MoveArm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__action__MoveArm_GetResult_Request
 * )) before or use
 * gamesmanros_interfaces__action__MoveArm_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Request__init(gamesmanros_interfaces__action__MoveArm_GetResult_Request * msg);

/// Finalize action/MoveArm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_GetResult_Request__fini(gamesmanros_interfaces__action__MoveArm_GetResult_Request * msg);

/// Create action/MoveArm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_GetResult_Request *
gamesmanros_interfaces__action__MoveArm_GetResult_Request__create();

/// Destroy action/MoveArm message.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_GetResult_Request__destroy(gamesmanros_interfaces__action__MoveArm_GetResult_Request * msg);

/// Check for action/MoveArm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Request__are_equal(const gamesmanros_interfaces__action__MoveArm_GetResult_Request * lhs, const gamesmanros_interfaces__action__MoveArm_GetResult_Request * rhs);

/// Copy a action/MoveArm message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Request__copy(
  const gamesmanros_interfaces__action__MoveArm_GetResult_Request * input,
  gamesmanros_interfaces__action__MoveArm_GetResult_Request * output);

/// Initialize array of action/MoveArm messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__init(gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__fini(gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence * array);

/// Create array of action/MoveArm messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence *
gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__destroy(gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence * array);

/// Check for action/MoveArm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__are_equal(const gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence * lhs, const gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence * rhs);

/// Copy an array of action/MoveArm messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__copy(
  const gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence * input,
  gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence * output);

/// Initialize action/MoveArm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__action__MoveArm_GetResult_Response
 * )) before or use
 * gamesmanros_interfaces__action__MoveArm_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Response__init(gamesmanros_interfaces__action__MoveArm_GetResult_Response * msg);

/// Finalize action/MoveArm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_GetResult_Response__fini(gamesmanros_interfaces__action__MoveArm_GetResult_Response * msg);

/// Create action/MoveArm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_GetResult_Response *
gamesmanros_interfaces__action__MoveArm_GetResult_Response__create();

/// Destroy action/MoveArm message.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_GetResult_Response__destroy(gamesmanros_interfaces__action__MoveArm_GetResult_Response * msg);

/// Check for action/MoveArm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Response__are_equal(const gamesmanros_interfaces__action__MoveArm_GetResult_Response * lhs, const gamesmanros_interfaces__action__MoveArm_GetResult_Response * rhs);

/// Copy a action/MoveArm message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Response__copy(
  const gamesmanros_interfaces__action__MoveArm_GetResult_Response * input,
  gamesmanros_interfaces__action__MoveArm_GetResult_Response * output);

/// Initialize array of action/MoveArm messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__init(gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__fini(gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence * array);

/// Create array of action/MoveArm messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence *
gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__destroy(gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence * array);

/// Check for action/MoveArm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__are_equal(const gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence * lhs, const gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence * rhs);

/// Copy an array of action/MoveArm messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__copy(
  const gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence * input,
  gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence * output);

/// Initialize action/MoveArm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__action__MoveArm_FeedbackMessage
 * )) before or use
 * gamesmanros_interfaces__action__MoveArm_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__init(gamesmanros_interfaces__action__MoveArm_FeedbackMessage * msg);

/// Finalize action/MoveArm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__fini(gamesmanros_interfaces__action__MoveArm_FeedbackMessage * msg);

/// Create action/MoveArm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__action__MoveArm_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_FeedbackMessage *
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__create();

/// Destroy action/MoveArm message.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__destroy(gamesmanros_interfaces__action__MoveArm_FeedbackMessage * msg);

/// Check for action/MoveArm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__are_equal(const gamesmanros_interfaces__action__MoveArm_FeedbackMessage * lhs, const gamesmanros_interfaces__action__MoveArm_FeedbackMessage * rhs);

/// Copy a action/MoveArm message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__copy(
  const gamesmanros_interfaces__action__MoveArm_FeedbackMessage * input,
  gamesmanros_interfaces__action__MoveArm_FeedbackMessage * output);

/// Initialize array of action/MoveArm messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__action__MoveArm_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__init(gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__fini(gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence * array);

/// Create array of action/MoveArm messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence *
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/MoveArm messages.
/**
 * It calls
 * gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__destroy(gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence * array);

/// Check for action/MoveArm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__are_equal(const gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence * lhs, const gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/MoveArm messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__copy(
  const gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence * input,
  gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_ARM__FUNCTIONS_H_
