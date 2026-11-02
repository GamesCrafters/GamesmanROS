// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from gamesmanros_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice
#include "gamesmanros_interfaces/msg/detail/piece_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
gamesmanros_interfaces__msg__PieceDetection__init(gamesmanros_interfaces__msg__PieceDetection * msg)
{
  if (!msg) {
    return false;
  }
  // tag_id
  // cell_index
  // board_x
  // board_y
  // offset_mm
  return true;
}

void
gamesmanros_interfaces__msg__PieceDetection__fini(gamesmanros_interfaces__msg__PieceDetection * msg)
{
  if (!msg) {
    return;
  }
  // tag_id
  // cell_index
  // board_x
  // board_y
  // offset_mm
}

bool
gamesmanros_interfaces__msg__PieceDetection__are_equal(const gamesmanros_interfaces__msg__PieceDetection * lhs, const gamesmanros_interfaces__msg__PieceDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tag_id
  if (lhs->tag_id != rhs->tag_id) {
    return false;
  }
  // cell_index
  if (lhs->cell_index != rhs->cell_index) {
    return false;
  }
  // board_x
  if (lhs->board_x != rhs->board_x) {
    return false;
  }
  // board_y
  if (lhs->board_y != rhs->board_y) {
    return false;
  }
  // offset_mm
  if (lhs->offset_mm != rhs->offset_mm) {
    return false;
  }
  return true;
}

bool
gamesmanros_interfaces__msg__PieceDetection__copy(
  const gamesmanros_interfaces__msg__PieceDetection * input,
  gamesmanros_interfaces__msg__PieceDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // tag_id
  output->tag_id = input->tag_id;
  // cell_index
  output->cell_index = input->cell_index;
  // board_x
  output->board_x = input->board_x;
  // board_y
  output->board_y = input->board_y;
  // offset_mm
  output->offset_mm = input->offset_mm;
  return true;
}

gamesmanros_interfaces__msg__PieceDetection *
gamesmanros_interfaces__msg__PieceDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__PieceDetection * msg = (gamesmanros_interfaces__msg__PieceDetection *)allocator.allocate(sizeof(gamesmanros_interfaces__msg__PieceDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gamesmanros_interfaces__msg__PieceDetection));
  bool success = gamesmanros_interfaces__msg__PieceDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gamesmanros_interfaces__msg__PieceDetection__destroy(gamesmanros_interfaces__msg__PieceDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gamesmanros_interfaces__msg__PieceDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gamesmanros_interfaces__msg__PieceDetection__Sequence__init(gamesmanros_interfaces__msg__PieceDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__PieceDetection * data = NULL;

  if (size) {
    data = (gamesmanros_interfaces__msg__PieceDetection *)allocator.zero_allocate(size, sizeof(gamesmanros_interfaces__msg__PieceDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gamesmanros_interfaces__msg__PieceDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gamesmanros_interfaces__msg__PieceDetection__fini(&data[i - 1]);
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
gamesmanros_interfaces__msg__PieceDetection__Sequence__fini(gamesmanros_interfaces__msg__PieceDetection__Sequence * array)
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
      gamesmanros_interfaces__msg__PieceDetection__fini(&array->data[i]);
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

gamesmanros_interfaces__msg__PieceDetection__Sequence *
gamesmanros_interfaces__msg__PieceDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gamesmanros_interfaces__msg__PieceDetection__Sequence * array = (gamesmanros_interfaces__msg__PieceDetection__Sequence *)allocator.allocate(sizeof(gamesmanros_interfaces__msg__PieceDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gamesmanros_interfaces__msg__PieceDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gamesmanros_interfaces__msg__PieceDetection__Sequence__destroy(gamesmanros_interfaces__msg__PieceDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gamesmanros_interfaces__msg__PieceDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gamesmanros_interfaces__msg__PieceDetection__Sequence__are_equal(const gamesmanros_interfaces__msg__PieceDetection__Sequence * lhs, const gamesmanros_interfaces__msg__PieceDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gamesmanros_interfaces__msg__PieceDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gamesmanros_interfaces__msg__PieceDetection__Sequence__copy(
  const gamesmanros_interfaces__msg__PieceDetection__Sequence * input,
  gamesmanros_interfaces__msg__PieceDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gamesmanros_interfaces__msg__PieceDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    gamesmanros_interfaces__msg__PieceDetection * data =
      (gamesmanros_interfaces__msg__PieceDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gamesmanros_interfaces__msg__PieceDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          gamesmanros_interfaces__msg__PieceDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gamesmanros_interfaces__msg__PieceDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
