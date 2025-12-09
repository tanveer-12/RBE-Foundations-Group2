// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from openmx_interfaces:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__TRAITS_HPP_
#define OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "openmx_interfaces/srv/detail/inverse_kinematics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace openmx_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const InverseKinematics_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InverseKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InverseKinematics_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace openmx_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use openmx_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const openmx_interfaces::srv::InverseKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  openmx_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use openmx_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const openmx_interfaces::srv::InverseKinematics_Request & msg)
{
  return openmx_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<openmx_interfaces::srv::InverseKinematics_Request>()
{
  return "openmx_interfaces::srv::InverseKinematics_Request";
}

template<>
inline const char * name<openmx_interfaces::srv::InverseKinematics_Request>()
{
  return "openmx_interfaces/srv/InverseKinematics_Request";
}

template<>
struct has_fixed_size<openmx_interfaces::srv::InverseKinematics_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<openmx_interfaces::srv::InverseKinematics_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<openmx_interfaces::srv::InverseKinematics_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace openmx_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const InverseKinematics_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: theta1
  {
    out << "theta1: ";
    rosidl_generator_traits::value_to_yaml(msg.theta1, out);
    out << ", ";
  }

  // member: theta2
  {
    out << "theta2: ";
    rosidl_generator_traits::value_to_yaml(msg.theta2, out);
    out << ", ";
  }

  // member: theta3
  {
    out << "theta3: ";
    rosidl_generator_traits::value_to_yaml(msg.theta3, out);
    out << ", ";
  }

  // member: theta4
  {
    out << "theta4: ";
    rosidl_generator_traits::value_to_yaml(msg.theta4, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InverseKinematics_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: theta1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta1: ";
    rosidl_generator_traits::value_to_yaml(msg.theta1, out);
    out << "\n";
  }

  // member: theta2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta2: ";
    rosidl_generator_traits::value_to_yaml(msg.theta2, out);
    out << "\n";
  }

  // member: theta3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta3: ";
    rosidl_generator_traits::value_to_yaml(msg.theta3, out);
    out << "\n";
  }

  // member: theta4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta4: ";
    rosidl_generator_traits::value_to_yaml(msg.theta4, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InverseKinematics_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace openmx_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use openmx_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const openmx_interfaces::srv::InverseKinematics_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  openmx_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use openmx_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const openmx_interfaces::srv::InverseKinematics_Response & msg)
{
  return openmx_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<openmx_interfaces::srv::InverseKinematics_Response>()
{
  return "openmx_interfaces::srv::InverseKinematics_Response";
}

template<>
inline const char * name<openmx_interfaces::srv::InverseKinematics_Response>()
{
  return "openmx_interfaces/srv/InverseKinematics_Response";
}

template<>
struct has_fixed_size<openmx_interfaces::srv::InverseKinematics_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<openmx_interfaces::srv::InverseKinematics_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<openmx_interfaces::srv::InverseKinematics_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<openmx_interfaces::srv::InverseKinematics>()
{
  return "openmx_interfaces::srv::InverseKinematics";
}

template<>
inline const char * name<openmx_interfaces::srv::InverseKinematics>()
{
  return "openmx_interfaces/srv/InverseKinematics";
}

template<>
struct has_fixed_size<openmx_interfaces::srv::InverseKinematics>
  : std::integral_constant<
    bool,
    has_fixed_size<openmx_interfaces::srv::InverseKinematics_Request>::value &&
    has_fixed_size<openmx_interfaces::srv::InverseKinematics_Response>::value
  >
{
};

template<>
struct has_bounded_size<openmx_interfaces::srv::InverseKinematics>
  : std::integral_constant<
    bool,
    has_bounded_size<openmx_interfaces::srv::InverseKinematics_Request>::value &&
    has_bounded_size<openmx_interfaces::srv::InverseKinematics_Response>::value
  >
{
};

template<>
struct is_service<openmx_interfaces::srv::InverseKinematics>
  : std::true_type
{
};

template<>
struct is_service_request<openmx_interfaces::srv::InverseKinematics_Request>
  : std::true_type
{
};

template<>
struct is_service_response<openmx_interfaces::srv::InverseKinematics_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__TRAITS_HPP_
