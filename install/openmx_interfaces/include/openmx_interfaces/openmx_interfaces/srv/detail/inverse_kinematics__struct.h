// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from openmx_interfaces:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_H_
#define OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/InverseKinematics in the package openmx_interfaces.
typedef struct openmx_interfaces__srv__InverseKinematics_Request
{
  geometry_msgs__msg__Pose pose;
} openmx_interfaces__srv__InverseKinematics_Request;

// Struct for a sequence of openmx_interfaces__srv__InverseKinematics_Request.
typedef struct openmx_interfaces__srv__InverseKinematics_Request__Sequence
{
  openmx_interfaces__srv__InverseKinematics_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openmx_interfaces__srv__InverseKinematics_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/InverseKinematics in the package openmx_interfaces.
typedef struct openmx_interfaces__srv__InverseKinematics_Response
{
  /// Response: joint angles (in degrees) and success flag
  bool success;
  rosidl_runtime_c__String message;
  double theta1;
  double theta2;
  double theta3;
  double theta4;
} openmx_interfaces__srv__InverseKinematics_Response;

// Struct for a sequence of openmx_interfaces__srv__InverseKinematics_Response.
typedef struct openmx_interfaces__srv__InverseKinematics_Response__Sequence
{
  openmx_interfaces__srv__InverseKinematics_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openmx_interfaces__srv__InverseKinematics_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_H_
