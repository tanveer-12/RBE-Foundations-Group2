// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from openmx_interfaces:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__BUILDER_HPP_
#define OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "openmx_interfaces/srv/detail/inverse_kinematics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace openmx_interfaces
{

namespace srv
{

namespace builder
{

class Init_InverseKinematics_Request_pose
{
public:
  Init_InverseKinematics_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::openmx_interfaces::srv::InverseKinematics_Request pose(::openmx_interfaces::srv::InverseKinematics_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::openmx_interfaces::srv::InverseKinematics_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openmx_interfaces::srv::InverseKinematics_Request>()
{
  return openmx_interfaces::srv::builder::Init_InverseKinematics_Request_pose();
}

}  // namespace openmx_interfaces


namespace openmx_interfaces
{

namespace srv
{

namespace builder
{

class Init_InverseKinematics_Response_theta4
{
public:
  explicit Init_InverseKinematics_Response_theta4(::openmx_interfaces::srv::InverseKinematics_Response & msg)
  : msg_(msg)
  {}
  ::openmx_interfaces::srv::InverseKinematics_Response theta4(::openmx_interfaces::srv::InverseKinematics_Response::_theta4_type arg)
  {
    msg_.theta4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::openmx_interfaces::srv::InverseKinematics_Response msg_;
};

class Init_InverseKinematics_Response_theta3
{
public:
  explicit Init_InverseKinematics_Response_theta3(::openmx_interfaces::srv::InverseKinematics_Response & msg)
  : msg_(msg)
  {}
  Init_InverseKinematics_Response_theta4 theta3(::openmx_interfaces::srv::InverseKinematics_Response::_theta3_type arg)
  {
    msg_.theta3 = std::move(arg);
    return Init_InverseKinematics_Response_theta4(msg_);
  }

private:
  ::openmx_interfaces::srv::InverseKinematics_Response msg_;
};

class Init_InverseKinematics_Response_theta2
{
public:
  explicit Init_InverseKinematics_Response_theta2(::openmx_interfaces::srv::InverseKinematics_Response & msg)
  : msg_(msg)
  {}
  Init_InverseKinematics_Response_theta3 theta2(::openmx_interfaces::srv::InverseKinematics_Response::_theta2_type arg)
  {
    msg_.theta2 = std::move(arg);
    return Init_InverseKinematics_Response_theta3(msg_);
  }

private:
  ::openmx_interfaces::srv::InverseKinematics_Response msg_;
};

class Init_InverseKinematics_Response_theta1
{
public:
  explicit Init_InverseKinematics_Response_theta1(::openmx_interfaces::srv::InverseKinematics_Response & msg)
  : msg_(msg)
  {}
  Init_InverseKinematics_Response_theta2 theta1(::openmx_interfaces::srv::InverseKinematics_Response::_theta1_type arg)
  {
    msg_.theta1 = std::move(arg);
    return Init_InverseKinematics_Response_theta2(msg_);
  }

private:
  ::openmx_interfaces::srv::InverseKinematics_Response msg_;
};

class Init_InverseKinematics_Response_message
{
public:
  explicit Init_InverseKinematics_Response_message(::openmx_interfaces::srv::InverseKinematics_Response & msg)
  : msg_(msg)
  {}
  Init_InverseKinematics_Response_theta1 message(::openmx_interfaces::srv::InverseKinematics_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_InverseKinematics_Response_theta1(msg_);
  }

private:
  ::openmx_interfaces::srv::InverseKinematics_Response msg_;
};

class Init_InverseKinematics_Response_success
{
public:
  Init_InverseKinematics_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InverseKinematics_Response_message success(::openmx_interfaces::srv::InverseKinematics_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_InverseKinematics_Response_message(msg_);
  }

private:
  ::openmx_interfaces::srv::InverseKinematics_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openmx_interfaces::srv::InverseKinematics_Response>()
{
  return openmx_interfaces::srv::builder::Init_InverseKinematics_Response_success();
}

}  // namespace openmx_interfaces

#endif  // OPENMX_INTERFACES__SRV__DETAIL__INVERSE_KINEMATICS__BUILDER_HPP_
