// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from velodyne_msgs:msg/VelodynePacket.idl
// generated code does not contain a copyright notice

#ifndef VELODYNE_MSGS__MSG__DETAIL__VELODYNE_PACKET__FUNCTIONS_H_
#define VELODYNE_MSGS__MSG__DETAIL__VELODYNE_PACKET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "velodyne_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "velodyne_msgs/msg/detail/velodyne_packet__struct.h"

/// Initialize msg/VelodynePacket message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * velodyne_msgs__msg__VelodynePacket
 * )) before or use
 * velodyne_msgs__msg__VelodynePacket__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
bool
velodyne_msgs__msg__VelodynePacket__init(velodyne_msgs__msg__VelodynePacket * msg);

/// Finalize msg/VelodynePacket message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
void
velodyne_msgs__msg__VelodynePacket__fini(velodyne_msgs__msg__VelodynePacket * msg);

/// Create msg/VelodynePacket message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * velodyne_msgs__msg__VelodynePacket__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
velodyne_msgs__msg__VelodynePacket *
velodyne_msgs__msg__VelodynePacket__create();

/// Destroy msg/VelodynePacket message.
/**
 * It calls
 * velodyne_msgs__msg__VelodynePacket__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
void
velodyne_msgs__msg__VelodynePacket__destroy(velodyne_msgs__msg__VelodynePacket * msg);


/// Initialize array of msg/VelodynePacket messages.
/**
 * It allocates the memory for the number of elements and calls
 * velodyne_msgs__msg__VelodynePacket__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
bool
velodyne_msgs__msg__VelodynePacket__Sequence__init(velodyne_msgs__msg__VelodynePacket__Sequence * array, size_t size);

/// Finalize array of msg/VelodynePacket messages.
/**
 * It calls
 * velodyne_msgs__msg__VelodynePacket__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
void
velodyne_msgs__msg__VelodynePacket__Sequence__fini(velodyne_msgs__msg__VelodynePacket__Sequence * array);

/// Create array of msg/VelodynePacket messages.
/**
 * It allocates the memory for the array and calls
 * velodyne_msgs__msg__VelodynePacket__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
velodyne_msgs__msg__VelodynePacket__Sequence *
velodyne_msgs__msg__VelodynePacket__Sequence__create(size_t size);

/// Destroy array of msg/VelodynePacket messages.
/**
 * It calls
 * velodyne_msgs__msg__VelodynePacket__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
void
velodyne_msgs__msg__VelodynePacket__Sequence__destroy(velodyne_msgs__msg__VelodynePacket__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // VELODYNE_MSGS__MSG__DETAIL__VELODYNE_PACKET__FUNCTIONS_H_
