// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from gamesmanros_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice
#include "gamesmanros_interfaces/msg/detail/board_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `occupancy`
#include "rosidl_runtime_c/string_functions.h"
// Member `pieces`
#include "gamesmanros_interfaces/msg/detail/piece_detection__functions.h"

bool
gamesmanros_interfaces__msg__BoardState__init(gamesmanros_interfaces__msg__BoardState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    gamesmanros_interfaces__msg__BoardState__fini(msg);
    return false;
  }
  // occupancy
  if (!rosidl_runtime_c__String__init(&msg->occupancy)) {
    gamesmanros_interfaces__msg__BoardState__fini(msg);
    return false;
  }
  // pieces
  if (!gamesmanros_interfaces__msg__PieceDetection__Sequence__init(&msg->pieces, 0)) {
    gamesmanros_interfaces__msg__BoardState__fini(msg);
    return false;
  }
  // board_located
  // frames_voted
  return true;
}

void
gamesmanros_interfaces__msg__BoardState__fini(gamesmanros_interfaces__msg__BoardState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // occupancy
  rosidl_runtime_c__String__fini(&msg->occupancy);
  // pieces
  gamesmanros_interfaces__msg__PieceDetection__Sequence__fini(&msg->pieces);
  // board_located
  // frames_voted
}

bool
gamesmanros_interfaces__msg__BoardState__are_equal(const gamesmanros_interfaces__msg__BoardState * lhs, const gamesmanros_interfaces__msg__BoardState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // occupancy
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->occupancy), &(rhs->occupancy)))
  {
    return false;
  }
  // pieces
  if (!gamesmanros_interfaces__msg__PieceDetection__Sequence__are_equal(
      &(lhs->pieces), &(rhs->pieces)))
  {
    return false;
  }
  // board_located
  if (lhs->board_located != rhs->board_located) {
    return false;
  }
  // frames_voted
  if (lhs->frames_voted != rhs->frames_voted) {
    return false;
  }
  return true;
}

bool
gamesmanros_interfaces__msg__BoardState__copy(
  const gamesmanros_interfaces__msg__BoardState * input,
  gamesmanros_interfaces__msg__BoardState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // occupancy
  if (!rosidl_runtime_c__String__copy(
      &(input->occupancy), &(output->occupancy)))
  {
    return false;
  }
  // pieces
  if (!gamesmanros_interfaces__msg__PieceDetection__Sequence__copy(
      &(input->pieces), &(output->pieces)))
  {
    return false;
  }
  // board_located
  output->board_located = input->board_located;
  // frames_voted
  output->frames_voted = input->frames_voted;
  return true;
}

gamesmanros_interfaces__msg__BoardState *
gamesmanros_interfaces__msg__BoardState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__BoardState * msg = (gamesmanros_interfaces__msg__BoardState *)allocator.allocate(sizeof(gamesmanros_interfaces__msg__BoardState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gamesmanros_interfaces__msg__BoardState));
  bool success = gamesmanros_interfaces__msg__BoardState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gamesmanros_interfaces__msg__BoardState__destroy(gamesmanros_interfaces__msg__BoardState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gamesmanros_interfaces__msg__BoardState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gamesmanros_interfaces__msg__BoardState__Sequence__init(gamesmanros_interfaces__msg__BoardState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__BoardState * data = NULL;

  if (size) {
    data = (gamesmanros_interfaces__msg__BoardState *)allocator.zero_allocate(size, sizeof(gamesmanros_interfaces__msg__BoardState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gamesmanros_interfaces__msg__BoardState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gamesmanros_interfaces__msg__BoardState__fini(&data[i - 1]);
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
gamesmanros_interfaces__msg__BoardState__Sequence__fini(gamesmanros_interfaces__msg__BoardState__Sequence * array)
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
      gamesmanros_interfaces__msg__BoardState__fini(&array->data[i]);
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

gamesmanros_interfaces__msg__BoardState__Sequence *
gamesmanros_interfaces__msg__BoardState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__BoardState__Sequence * array = (gamesmanros_interfaces__msg__BoardState__Sequence *)allocator.allocate(sizeof(gamesmanros_interfaces__msg__BoardState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gamesmanros_interfaces__msg__BoardState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gamesmanros_interfaces__msg__BoardState__Sequence__destroy(gamesmanros_interfaces__msg__BoardState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gamesmanros_interfaces__msg__BoardState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gamesmanros_interfaces__msg__BoardState__Sequence__are_equal(const gamesmanros_interfaces__msg__BoardState__Sequence * lhs, const gamesmanros_interfaces__msg__BoardState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gamesmanros_interfaces__msg__BoardState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gamesmanros_interfaces__msg__BoardState__Sequence__copy(
  const gamesmanros_interfaces__msg__BoardState__Sequence * input,
  gamesmanros_interfaces__msg__BoardState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gamesmanros_interfaces__msg__BoardState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    gamesmanros_interfaces__msg__BoardState * data =
      (gamesmanros_interfaces__msg__BoardState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gamesmanros_interfaces__msg__BoardState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          gamesmanros_interfaces__msg__BoardState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gamesmanros_interfaces__msg__BoardState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
