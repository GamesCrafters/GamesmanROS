// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from gamesmanros_interfaces:msg/GameState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__FUNCTIONS_H_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "gamesmanros_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "gamesmanros_interfaces/msg/detail/game_state__struct.h"

/// Initialize msg/GameState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gamesmanros_interfaces__msg__GameState
 * )) before or use
 * gamesmanros_interfaces__msg__GameState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__msg__GameState__init(gamesmanros_interfaces__msg__GameState * msg);

/// Finalize msg/GameState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__msg__GameState__fini(gamesmanros_interfaces__msg__GameState * msg);

/// Create msg/GameState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gamesmanros_interfaces__msg__GameState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__msg__GameState *
gamesmanros_interfaces__msg__GameState__create();

/// Destroy msg/GameState message.
/**
 * It calls
 * gamesmanros_interfaces__msg__GameState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__msg__GameState__destroy(gamesmanros_interfaces__msg__GameState * msg);

/// Check for msg/GameState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__msg__GameState__are_equal(const gamesmanros_interfaces__msg__GameState * lhs, const gamesmanros_interfaces__msg__GameState * rhs);

/// Copy a msg/GameState message.
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
gamesmanros_interfaces__msg__GameState__copy(
  const gamesmanros_interfaces__msg__GameState * input,
  gamesmanros_interfaces__msg__GameState * output);

/// Initialize array of msg/GameState messages.
/**
 * It allocates the memory for the number of elements and calls
 * gamesmanros_interfaces__msg__GameState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__msg__GameState__Sequence__init(gamesmanros_interfaces__msg__GameState__Sequence * array, size_t size);

/// Finalize array of msg/GameState messages.
/**
 * It calls
 * gamesmanros_interfaces__msg__GameState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__msg__GameState__Sequence__fini(gamesmanros_interfaces__msg__GameState__Sequence * array);

/// Create array of msg/GameState messages.
/**
 * It allocates the memory for the array and calls
 * gamesmanros_interfaces__msg__GameState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
gamesmanros_interfaces__msg__GameState__Sequence *
gamesmanros_interfaces__msg__GameState__Sequence__create(size_t size);

/// Destroy array of msg/GameState messages.
/**
 * It calls
 * gamesmanros_interfaces__msg__GameState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
void
gamesmanros_interfaces__msg__GameState__Sequence__destroy(gamesmanros_interfaces__msg__GameState__Sequence * array);

/// Check for msg/GameState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gamesmanros_interfaces
bool
gamesmanros_interfaces__msg__GameState__Sequence__are_equal(const gamesmanros_interfaces__msg__GameState__Sequence * lhs, const gamesmanros_interfaces__msg__GameState__Sequence * rhs);

/// Copy an array of msg/GameState messages.
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
gamesmanros_interfaces__msg__GameState__Sequence__copy(
  const gamesmanros_interfaces__msg__GameState__Sequence * input,
  gamesmanros_interfaces__msg__GameState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__FUNCTIONS_H_
