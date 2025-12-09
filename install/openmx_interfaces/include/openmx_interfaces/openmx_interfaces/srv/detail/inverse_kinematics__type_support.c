// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from openmx_interfaces:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "openmx_interfaces/srv/detail/inverse_kinematics__rosidl_typesupport_introspection_c.h"
#include "openmx_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "openmx_interfaces/srv/detail/inverse_kinematics__functions.h"
#include "openmx_interfaces/srv/detail/inverse_kinematics__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  openmx_interfaces__srv__InverseKinematics_Request__init(message_memory);
}

void openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_fini_function(void * message_memory)
{
  openmx_interfaces__srv__InverseKinematics_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_member_array[1] = {
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(openmx_interfaces__srv__InverseKinematics_Request, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_members = {
  "openmx_interfaces__srv",  // message namespace
  "InverseKinematics_Request",  // message name
  1,  // number of fields
  sizeof(openmx_interfaces__srv__InverseKinematics_Request),
  openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_member_array,  // message members
  openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_type_support_handle = {
  0,
  &openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_openmx_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, openmx_interfaces, srv, InverseKinematics_Request)() {
  openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_type_support_handle.typesupport_identifier) {
    openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &openmx_interfaces__srv__InverseKinematics_Request__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "openmx_interfaces/srv/detail/inverse_kinematics__rosidl_typesupport_introspection_c.h"
// already included above
// #include "openmx_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "openmx_interfaces/srv/detail/inverse_kinematics__functions.h"
// already included above
// #include "openmx_interfaces/srv/detail/inverse_kinematics__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  openmx_interfaces__srv__InverseKinematics_Response__init(message_memory);
}

void openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_fini_function(void * message_memory)
{
  openmx_interfaces__srv__InverseKinematics_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_member_array[6] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(openmx_interfaces__srv__InverseKinematics_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(openmx_interfaces__srv__InverseKinematics_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(openmx_interfaces__srv__InverseKinematics_Response, theta1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(openmx_interfaces__srv__InverseKinematics_Response, theta2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(openmx_interfaces__srv__InverseKinematics_Response, theta3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(openmx_interfaces__srv__InverseKinematics_Response, theta4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_members = {
  "openmx_interfaces__srv",  // message namespace
  "InverseKinematics_Response",  // message name
  6,  // number of fields
  sizeof(openmx_interfaces__srv__InverseKinematics_Response),
  openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_member_array,  // message members
  openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_type_support_handle = {
  0,
  &openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_openmx_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, openmx_interfaces, srv, InverseKinematics_Response)() {
  if (!openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_type_support_handle.typesupport_identifier) {
    openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &openmx_interfaces__srv__InverseKinematics_Response__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "openmx_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "openmx_interfaces/srv/detail/inverse_kinematics__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_service_members = {
  "openmx_interfaces__srv",  // service namespace
  "InverseKinematics",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_Request_message_type_support_handle,
  NULL  // response message
  // openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_Response_message_type_support_handle
};

static rosidl_service_type_support_t openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_service_type_support_handle = {
  0,
  &openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, openmx_interfaces, srv, InverseKinematics_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, openmx_interfaces, srv, InverseKinematics_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_openmx_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, openmx_interfaces, srv, InverseKinematics)() {
  if (!openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_service_type_support_handle.typesupport_identifier) {
    openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, openmx_interfaces, srv, InverseKinematics_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, openmx_interfaces, srv, InverseKinematics_Response)()->data;
  }

  return &openmx_interfaces__srv__detail__inverse_kinematics__rosidl_typesupport_introspection_c__InverseKinematics_service_type_support_handle;
}
