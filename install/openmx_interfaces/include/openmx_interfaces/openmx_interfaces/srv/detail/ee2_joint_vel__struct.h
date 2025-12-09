// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from openmx_interfaces:srv/EE2JointVel.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__EE2_JOINT_VEL__STRUCT_H_
#define OPENMX_INTERFACES__SRV__DETAIL__EE2_JOINT_VEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/EE2JointVel in the package openmx_interfaces.
typedef struct openmx_interfaces__srv__EE2JointVel_Request
{
  /// Request: Joint configuration and desired end-effector velocity
  /// Joint 1 position (rad)
  double q1;
  /// Joint 2 position (rad)
  double q2;
  /// Joint 3 position (rad)
  double q3;
  /// Joint 4 position (rad)
  double q4;
  /// Desired EE linear velocity x-component (m/s)
  double vx;
  /// Desired EE linear velocity y-component (m/s)
  double vy;
  /// Desired EE linear velocity z-component (m/s)
  double vz;
  /// Desired EE angular velocity x-component (rad/s)
  double wx;
  /// Desired EE angular velocity y-component (rad/s)
  double wy;
  /// Desired EE angular velocity z-component (rad/s)
  double wz;
} openmx_interfaces__srv__EE2JointVel_Request;

// Struct for a sequence of openmx_interfaces__srv__EE2JointVel_Request.
typedef struct openmx_interfaces__srv__EE2JointVel_Request__Sequence
{
  openmx_interfaces__srv__EE2JointVel_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openmx_interfaces__srv__EE2JointVel_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/EE2JointVel in the package openmx_interfaces.
typedef struct openmx_interfaces__srv__EE2JointVel_Response
{
  /// Response: Joint velocities
  /// Joint 1 velocity (rad/s)
  double q1_dot;
  /// Joint 2 velocity (rad/s)
  double q2_dot;
  /// Joint 3 velocity (rad/s)
  double q3_dot;
  /// Joint 4 velocity (rad/s)
  double q4_dot;
  /// True if computation succeeded
  bool success;
  /// Status message or error description
  rosidl_runtime_c__String message;
} openmx_interfaces__srv__EE2JointVel_Response;

// Struct for a sequence of openmx_interfaces__srv__EE2JointVel_Response.
typedef struct openmx_interfaces__srv__EE2JointVel_Response__Sequence
{
  openmx_interfaces__srv__EE2JointVel_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openmx_interfaces__srv__EE2JointVel_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OPENMX_INTERFACES__SRV__DETAIL__EE2_JOINT_VEL__STRUCT_H_
