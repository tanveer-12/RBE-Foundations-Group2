// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from openmx_interfaces:srv/JointVel2EE.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__TRAITS_HPP_
#define OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "openmx_interfaces/srv/detail/joint_vel2_ee__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace openmx_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const JointVel2EE_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: q1
  {
    out << "q1: ";
    rosidl_generator_traits::value_to_yaml(msg.q1, out);
    out << ", ";
  }

  // member: q2
  {
    out << "q2: ";
    rosidl_generator_traits::value_to_yaml(msg.q2, out);
    out << ", ";
  }

  // member: q3
  {
    out << "q3: ";
    rosidl_generator_traits::value_to_yaml(msg.q3, out);
    out << ", ";
  }

  // member: q4
  {
    out << "q4: ";
    rosidl_generator_traits::value_to_yaml(msg.q4, out);
    out << ", ";
  }

  // member: q1_dot
  {
    out << "q1_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q1_dot, out);
    out << ", ";
  }

  // member: q2_dot
  {
    out << "q2_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q2_dot, out);
    out << ", ";
  }

  // member: q3_dot
  {
    out << "q3_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q3_dot, out);
    out << ", ";
  }

  // member: q4_dot
  {
    out << "q4_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q4_dot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointVel2EE_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: q1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q1: ";
    rosidl_generator_traits::value_to_yaml(msg.q1, out);
    out << "\n";
  }

  // member: q2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q2: ";
    rosidl_generator_traits::value_to_yaml(msg.q2, out);
    out << "\n";
  }

  // member: q3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q3: ";
    rosidl_generator_traits::value_to_yaml(msg.q3, out);
    out << "\n";
  }

  // member: q4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q4: ";
    rosidl_generator_traits::value_to_yaml(msg.q4, out);
    out << "\n";
  }

  // member: q1_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q1_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q1_dot, out);
    out << "\n";
  }

  // member: q2_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q2_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q2_dot, out);
    out << "\n";
  }

  // member: q3_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q3_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q3_dot, out);
    out << "\n";
  }

  // member: q4_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q4_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q4_dot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointVel2EE_Request & msg, bool use_flow_style = false)
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
  const openmx_interfaces::srv::JointVel2EE_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  openmx_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use openmx_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const openmx_interfaces::srv::JointVel2EE_Request & msg)
{
  return openmx_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<openmx_interfaces::srv::JointVel2EE_Request>()
{
  return "openmx_interfaces::srv::JointVel2EE_Request";
}

template<>
inline const char * name<openmx_interfaces::srv::JointVel2EE_Request>()
{
  return "openmx_interfaces/srv/JointVel2EE_Request";
}

template<>
struct has_fixed_size<openmx_interfaces::srv::JointVel2EE_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<openmx_interfaces::srv::JointVel2EE_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<openmx_interfaces::srv::JointVel2EE_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace openmx_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const JointVel2EE_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << ", ";
  }

  // member: vz
  {
    out << "vz: ";
    rosidl_generator_traits::value_to_yaml(msg.vz, out);
    out << ", ";
  }

  // member: wx
  {
    out << "wx: ";
    rosidl_generator_traits::value_to_yaml(msg.wx, out);
    out << ", ";
  }

  // member: wy
  {
    out << "wy: ";
    rosidl_generator_traits::value_to_yaml(msg.wy, out);
    out << ", ";
  }

  // member: wz
  {
    out << "wz: ";
    rosidl_generator_traits::value_to_yaml(msg.wz, out);
    out << ", ";
  }

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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointVel2EE_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }

  // member: vz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vz: ";
    rosidl_generator_traits::value_to_yaml(msg.vz, out);
    out << "\n";
  }

  // member: wx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wx: ";
    rosidl_generator_traits::value_to_yaml(msg.wx, out);
    out << "\n";
  }

  // member: wy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wy: ";
    rosidl_generator_traits::value_to_yaml(msg.wy, out);
    out << "\n";
  }

  // member: wz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wz: ";
    rosidl_generator_traits::value_to_yaml(msg.wz, out);
    out << "\n";
  }

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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointVel2EE_Response & msg, bool use_flow_style = false)
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
  const openmx_interfaces::srv::JointVel2EE_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  openmx_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use openmx_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const openmx_interfaces::srv::JointVel2EE_Response & msg)
{
  return openmx_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<openmx_interfaces::srv::JointVel2EE_Response>()
{
  return "openmx_interfaces::srv::JointVel2EE_Response";
}

template<>
inline const char * name<openmx_interfaces::srv::JointVel2EE_Response>()
{
  return "openmx_interfaces/srv/JointVel2EE_Response";
}

template<>
struct has_fixed_size<openmx_interfaces::srv::JointVel2EE_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<openmx_interfaces::srv::JointVel2EE_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<openmx_interfaces::srv::JointVel2EE_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<openmx_interfaces::srv::JointVel2EE>()
{
  return "openmx_interfaces::srv::JointVel2EE";
}

template<>
inline const char * name<openmx_interfaces::srv::JointVel2EE>()
{
  return "openmx_interfaces/srv/JointVel2EE";
}

template<>
struct has_fixed_size<openmx_interfaces::srv::JointVel2EE>
  : std::integral_constant<
    bool,
    has_fixed_size<openmx_interfaces::srv::JointVel2EE_Request>::value &&
    has_fixed_size<openmx_interfaces::srv::JointVel2EE_Response>::value
  >
{
};

template<>
struct has_bounded_size<openmx_interfaces::srv::JointVel2EE>
  : std::integral_constant<
    bool,
    has_bounded_size<openmx_interfaces::srv::JointVel2EE_Request>::value &&
    has_bounded_size<openmx_interfaces::srv::JointVel2EE_Response>::value
  >
{
};

template<>
struct is_service<openmx_interfaces::srv::JointVel2EE>
  : std::true_type
{
};

template<>
struct is_service_request<openmx_interfaces::srv::JointVel2EE_Request>
  : std::true_type
{
};

template<>
struct is_service_response<openmx_interfaces::srv::JointVel2EE_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__TRAITS_HPP_
