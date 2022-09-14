// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from velodyne_msgs:msg/VelodyneScan.idl
// generated code does not contain a copyright notice

#ifndef VELODYNE_MSGS__MSG__DETAIL__VELODYNE_SCAN__FUNCTIONS_H_
#define VELODYNE_MSGS__MSG__DETAIL__VELODYNE_SCAN__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "velodyne_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "velodyne_msgs/msg/detail/velodyne_scan__struct.h"

/// Initialize msg/VelodyneScan message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * velodyne_msgs__msg__VelodyneScan
 * )) before or use
 * velodyne_msgs__msg__VelodyneScan__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
bool
velodyne_msgs__msg__VelodyneScan__init(velodyne_msgs__msg__VelodyneScan * msg);

/// Finalize msg/VelodyneScan message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
void
velodyne_msgs__msg__VelodyneScan__fini(velodyne_msgs__msg__VelodyneScan * msg);

/// Create msg/VelodyneScan message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * velodyne_msgs__msg__VelodyneScan__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
velodyne_msgs__msg__VelodyneScan *
velodyne_msgs__msg__VelodyneScan__create();

/// Destroy msg/VelodyneScan message.
/**
 * It calls
 * velodyne_msgs__msg__VelodyneScan__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
void
velodyne_msgs__msg__VelodyneScan__destroy(velodyne_msgs__msg__VelodyneScan * msg);


/// Initialize array of msg/VelodyneScan messages.
/**
 * It allocates the memory for the number of elements and calls
 * velodyne_msgs__msg__VelodyneScan__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
bool
velodyne_msgs__msg__VelodyneScan__Sequence__init(velodyne_msgs__msg__VelodyneScan__Sequence * array, size_t size);

/// Finalize array of msg/VelodyneScan messages.
/**
 * It calls
 * velodyne_msgs__msg__VelodyneScan__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
void
velodyne_msgs__msg__VelodyneScan__Sequence__fini(velodyne_msgs__msg__VelodyneScan__Sequence * array);

/// Create array of msg/VelodyneScan messages.
/**
 * It allocates the memory for the array and calls
 * velodyne_msgs__msg__VelodyneScan__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
velodyne_msgs__msg__VelodyneScan__Sequence *
velodyne_msgs__msg__VelodyneScan__Sequence__create(size_t size);

/// Destroy array of msg/VelodyneScan messages.
/**
 * It calls
 * velodyne_msgs__msg__VelodyneScan__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
void
velodyne_msgs__msg__VelodyneScan__Sequence__destroy(velodyne_msgs__msg__VelodyneScan__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // VELODYNE_MSGS__MSG__DETAIL__VELODYNE_SCAN__FUNCTIONS_H_
