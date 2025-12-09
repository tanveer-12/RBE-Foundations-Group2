// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from openmx_interfaces:srv/JointVel2EE.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__STRUCT_H_
#define OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/JointVel2EE in the package openmx_interfaces.
typedef struct openmx_interfaces__srv__JointVel2EE_Request
{
  /// Request: Joint configuration and joint velocities
  /// Joint 1 position (rad)
  double q1;
  /// Joint 2 position (rad)
  double q2;
  /// Joint 3 position (rad)
  double q3;
  /// Joint 4 position (rad)
  double q4;
  /// Joint 1 velocity (rad/s)
  double q1_dot;
  /// Joint 2 velocity (rad/s)
  double q2_dot;
  /// Joint 3 velocity (rad/s)
  double q3_dot;
  /// Joint 4 velocity (rad/s)
  double q4_dot;
} openmx_interfaces__srv__JointVel2EE_Request;

// Struct for a sequence of openmx_interfaces__srv__JointVel2EE_Request.
typedef struct openmx_interfaces__srv__JointVel2EE_Request__Sequence
{
  openmx_interfaces__srv__JointVel2EE_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openmx_interfaces__srv__JointVel2EE_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/JointVel2EE in the package openmx_interfaces.
typedef struct openmx_interfaces__srv__JointVel2EE_Response
{
  /// Response: End-effector linear and angular velocities
  /// EE linear velocity x-component (m/s)
  double vx;
  /// EE linear velocity y-component (m/s)
  double vy;
  /// EE linear velocity z-component (m/s)
  double vz;
  /// EE angular velocity x-component (rad/s)
  double wx;
  /// EE angular velocity y-component (rad/s)
  double wy;
  /// EE angular velocity z-component (rad/s)
  double wz;
  /// True if computation succeeded
  bool success;
  /// Status message or error description
  rosidl_runtime_c__String message;
} openmx_interfaces__srv__JointVel2EE_Response;

// Struct for a sequence of openmx_interfaces__srv__JointVel2EE_Response.
typedef struct openmx_interfaces__srv__JointVel2EE_Response__Sequence
{
  openmx_interfaces__srv__JointVel2EE_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openmx_interfaces__srv__JointVel2EE_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__STRUCT_H_
