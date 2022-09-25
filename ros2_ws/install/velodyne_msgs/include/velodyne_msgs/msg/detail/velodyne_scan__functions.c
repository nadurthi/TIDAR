// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from velodyne_msgs:msg/VelodyneScan.idl
// generated code does not contain a copyright notice
#include "velodyne_msgs/msg/detail/velodyne_scan__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `packets`
#include "velodyne_msgs/msg/detail/velodyne_packet__functions.h"

bool
velodyne_msgs__msg__VelodyneScan__init(velodyne_msgs__msg__VelodyneScan * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    velodyne_msgs__msg__VelodyneScan__fini(msg);
    return false;
  }
  // packets
  if (!velodyne_msgs__msg__VelodynePacket__Sequence__init(&msg->packets, 0)) {
    velodyne_msgs__msg__VelodyneScan__fini(msg);
    return false;
  }
  return true;
}

void
velodyne_msgs__msg__VelodyneScan__fini(velodyne_msgs__msg__VelodyneScan * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // packets
  velodyne_msgs__msg__VelodynePacket__Sequence__fini(&msg->packets);
}

velodyne_msgs__msg__VelodyneScan *
velodyne_msgs__msg__VelodyneScan__create()
{
  velodyne_msgs__msg__VelodyneScan * msg = (velodyne_msgs__msg__VelodyneScan *)malloc(sizeof(velodyne_msgs__msg__VelodyneScan));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(velodyne_msgs__msg__VelodyneScan));
  bool success = velodyne_msgs__msg__VelodyneScan__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
velodyne_msgs__msg__VelodyneScan__destroy(velodyne_msgs__msg__VelodyneScan * msg)
{
  if (msg) {
    velodyne_msgs__msg__VelodyneScan__fini(msg);
  }
  free(msg);
}


bool
velodyne_msgs__msg__VelodyneScan__Sequence__init(velodyne_msgs__msg__VelodyneScan__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  velodyne_msgs__msg__VelodyneScan * data = NULL;
  if (size) {
    data = (velodyne_msgs__msg__VelodyneScan *)calloc(size, sizeof(velodyne_msgs__msg__VelodyneScan));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = velodyne_msgs__msg__VelodyneScan__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        velodyne_msgs__msg__VelodyneScan__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
velodyne_msgs__msg__VelodyneScan__Sequence__fini(velodyne_msgs__msg__VelodyneScan__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      velodyne_msgs__msg__VelodyneScan__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

velodyne_msgs__msg__VelodyneScan__Sequence *
velodyne_msgs__msg__VelodyneScan__Sequence__create(size_t size)
{
  velodyne_msgs__msg__VelodyneScan__Sequence * array = (velodyne_msgs__msg__VelodyneScan__Sequence *)malloc(sizeof(velodyne_msgs__msg__VelodyneScan__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = velodyne_msgs__msg__VelodyneScan__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
velodyne_msgs__msg__VelodyneScan__Sequence__destroy(velodyne_msgs__msg__VelodyneScan__Sequence * array)
{
  if (array) {
    velodyne_msgs__msg__VelodyneScan__Sequence__fini(array);
  }
  free(array);
}
