// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from gamesmanros_interfaces:msg/GameState.idl
// generated code does not contain a copyright notice
#include "gamesmanros_interfaces/msg/detail/game_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
// Member `available_moves`
#include "rosidl_runtime_c/string_functions.h"

bool
gamesmanros_interfaces__msg__GameState__init(gamesmanros_interfaces__msg__GameState * msg)
{
  if (!msg) {
    return false;
  }
  // position
  if (!rosidl_runtime_c__String__init(&msg->position)) {
    gamesmanros_interfaces__msg__GameState__fini(msg);
    return false;
  }
  // available_moves
  if (!rosidl_runtime_c__String__Sequence__init(&msg->available_moves, 0)) {
    gamesmanros_interfaces__msg__GameState__fini(msg);
    return false;
  }
  // is_robot_turn
  // game_over
  return true;
}

void
gamesmanros_interfaces__msg__GameState__fini(gamesmanros_interfaces__msg__GameState * msg)
{
  if (!msg) {
    return;
  }
  // position
  rosidl_runtime_c__String__fini(&msg->position);
  // available_moves
  rosidl_runtime_c__String__Sequence__fini(&msg->available_moves);
  // is_robot_turn
  // game_over
}

bool
gamesmanros_interfaces__msg__GameState__are_equal(const gamesmanros_interfaces__msg__GameState * lhs, const gamesmanros_interfaces__msg__GameState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // available_moves
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->available_moves), &(rhs->available_moves)))
  {
    return false;
  }
  // is_robot_turn
  if (lhs->is_robot_turn != rhs->is_robot_turn) {
    return false;
  }
  // game_over
  if (lhs->game_over != rhs->game_over) {
    return false;
  }
  return true;
}

bool
gamesmanros_interfaces__msg__GameState__copy(
  const gamesmanros_interfaces__msg__GameState * input,
  gamesmanros_interfaces__msg__GameState * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  if (!rosidl_runtime_c__String__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // available_moves
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->available_moves), &(output->available_moves)))
  {
    return false;
  }
  // is_robot_turn
  output->is_robot_turn = input->is_robot_turn;
  // game_over
  output->game_over = input->game_over;
  return true;
}

gamesmanros_interfaces__msg__GameState *
gamesmanros_interfaces__msg__GameState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__GameState * msg = (gamesmanros_interfaces__msg__GameState *)allocator.allocate(sizeof(gamesmanros_interfaces__msg__GameState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gamesmanros_interfaces__msg__GameState));
  bool success = gamesmanros_interfaces__msg__GameState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gamesmanros_interfaces__msg__GameState__destroy(gamesmanros_interfaces__msg__GameState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gamesmanros_interfaces__msg__GameState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gamesmanros_interfaces__msg__GameState__Sequence__init(gamesmanros_interfaces__msg__GameState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__GameState * data = NULL;

  if (size) {
    data = (gamesmanros_interfaces__msg__GameState *)allocator.zero_allocate(size, sizeof(gamesmanros_interfaces__msg__GameState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gamesmanros_interfaces__msg__GameState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gamesmanros_interfaces__msg__GameState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
gamesmanros_interfaces__msg__GameState__Sequence__fini(gamesmanros_interfaces__msg__GameState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      gamesmanros_interfaces__msg__GameState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

gamesmanros_interfaces__msg__GameState__Sequence *
gamesmanros_interfaces__msg__GameState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__GameState__Sequence * array = (gamesmanros_interfaces__msg__GameState__Sequence *)allocator.allocate(sizeof(gamesmanros_interfaces__msg__GameState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gamesmanros_interfaces__msg__GameState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gamesmanros_interfaces__msg__GameState__Sequence__destroy(gamesmanros_interfaces__msg__GameState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gamesmanros_interfaces__msg__GameState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gamesmanros_interfaces__msg__GameState__Sequence__are_equal(const gamesmanros_interfaces__msg__GameState__Sequence * lhs, const gamesmanros_interfaces__msg__GameState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gamesmanros_interfaces__msg__GameState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gamesmanros_interfaces__msg__GameState__Sequence__copy(
  const gamesmanros_interfaces__msg__GameState__Sequence * input,
  gamesmanros_interfaces__msg__GameState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gamesmanros_interfaces__msg__GameState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    gamesmanros_interfaces__msg__GameState * data =
      (gamesmanros_interfaces__msg__GameState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gamesmanros_interfaces__msg__GameState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          gamesmanros_interfaces__msg__GameState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gamesmanros_interfaces__msg__GameState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
