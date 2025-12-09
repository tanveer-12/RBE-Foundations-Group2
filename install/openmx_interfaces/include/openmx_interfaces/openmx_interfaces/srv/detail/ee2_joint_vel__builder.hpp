// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from openmx_interfaces:srv/EE2JointVel.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__EE2_JOINT_VEL__BUILDER_HPP_
#define OPENMX_INTERFACES__SRV__DETAIL__EE2_JOINT_VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "openmx_interfaces/srv/detail/ee2_joint_vel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace openmx_interfaces
{

namespace srv
{

namespace builder
{

class Init_EE2JointVel_Request_wz
{
public:
  explicit Init_EE2JointVel_Request_wz(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  ::openmx_interfaces::srv::EE2JointVel_Request wz(::openmx_interfaces::srv::EE2JointVel_Request::_wz_type arg)
  {
    msg_.wz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_wy
{
public:
  explicit Init_EE2JointVel_Request_wy(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Request_wz wy(::openmx_interfaces::srv::EE2JointVel_Request::_wy_type arg)
  {
    msg_.wy = std::move(arg);
    return Init_EE2JointVel_Request_wz(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_wx
{
public:
  explicit Init_EE2JointVel_Request_wx(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Request_wy wx(::openmx_interfaces::srv::EE2JointVel_Request::_wx_type arg)
  {
    msg_.wx = std::move(arg);
    return Init_EE2JointVel_Request_wy(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_vz
{
public:
  explicit Init_EE2JointVel_Request_vz(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Request_wx vz(::openmx_interfaces::srv::EE2JointVel_Request::_vz_type arg)
  {
    msg_.vz = std::move(arg);
    return Init_EE2JointVel_Request_wx(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_vy
{
public:
  explicit Init_EE2JointVel_Request_vy(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Request_vz vy(::openmx_interfaces::srv::EE2JointVel_Request::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_EE2JointVel_Request_vz(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_vx
{
public:
  explicit Init_EE2JointVel_Request_vx(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Request_vy vx(::openmx_interfaces::srv::EE2JointVel_Request::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_EE2JointVel_Request_vy(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_q4
{
public:
  explicit Init_EE2JointVel_Request_q4(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Request_vx q4(::openmx_interfaces::srv::EE2JointVel_Request::_q4_type arg)
  {
    msg_.q4 = std::move(arg);
    return Init_EE2JointVel_Request_vx(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_q3
{
public:
  explicit Init_EE2JointVel_Request_q3(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Request_q4 q3(::openmx_interfaces::srv::EE2JointVel_Request::_q3_type arg)
  {
    msg_.q3 = std::move(arg);
    return Init_EE2JointVel_Request_q4(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_q2
{
public:
  explicit Init_EE2JointVel_Request_q2(::openmx_interfaces::srv::EE2JointVel_Request & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Request_q3 q2(::openmx_interfaces::srv::EE2JointVel_Request::_q2_type arg)
  {
    msg_.q2 = std::move(arg);
    return Init_EE2JointVel_Request_q3(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

class Init_EE2JointVel_Request_q1
{
public:
  Init_EE2JointVel_Request_q1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EE2JointVel_Request_q2 q1(::openmx_interfaces::srv::EE2JointVel_Request::_q1_type arg)
  {
    msg_.q1 = std::move(arg);
    return Init_EE2JointVel_Request_q2(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openmx_interfaces::srv::EE2JointVel_Request>()
{
  return openmx_interfaces::srv::builder::Init_EE2JointVel_Request_q1();
}

}  // namespace openmx_interfaces


namespace openmx_interfaces
{

namespace srv
{

namespace builder
{

class Init_EE2JointVel_Response_message
{
public:
  explicit Init_EE2JointVel_Response_message(::openmx_interfaces::srv::EE2JointVel_Response & msg)
  : msg_(msg)
  {}
  ::openmx_interfaces::srv::EE2JointVel_Response message(::openmx_interfaces::srv::EE2JointVel_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Response msg_;
};

class Init_EE2JointVel_Response_success
{
public:
  explicit Init_EE2JointVel_Response_success(::openmx_interfaces::srv::EE2JointVel_Response & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Response_message success(::openmx_interfaces::srv::EE2JointVel_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_EE2JointVel_Response_message(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Response msg_;
};

class Init_EE2JointVel_Response_q4_dot
{
public:
  explicit Init_EE2JointVel_Response_q4_dot(::openmx_interfaces::srv::EE2JointVel_Response & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Response_success q4_dot(::openmx_interfaces::srv::EE2JointVel_Response::_q4_dot_type arg)
  {
    msg_.q4_dot = std::move(arg);
    return Init_EE2JointVel_Response_success(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Response msg_;
};

class Init_EE2JointVel_Response_q3_dot
{
public:
  explicit Init_EE2JointVel_Response_q3_dot(::openmx_interfaces::srv::EE2JointVel_Response & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Response_q4_dot q3_dot(::openmx_interfaces::srv::EE2JointVel_Response::_q3_dot_type arg)
  {
    msg_.q3_dot = std::move(arg);
    return Init_EE2JointVel_Response_q4_dot(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Response msg_;
};

class Init_EE2JointVel_Response_q2_dot
{
public:
  explicit Init_EE2JointVel_Response_q2_dot(::openmx_interfaces::srv::EE2JointVel_Response & msg)
  : msg_(msg)
  {}
  Init_EE2JointVel_Response_q3_dot q2_dot(::openmx_interfaces::srv::EE2JointVel_Response::_q2_dot_type arg)
  {
    msg_.q2_dot = std::move(arg);
    return Init_EE2JointVel_Response_q3_dot(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Response msg_;
};

class Init_EE2JointVel_Response_q1_dot
{
public:
  Init_EE2JointVel_Response_q1_dot()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EE2JointVel_Response_q2_dot q1_dot(::openmx_interfaces::srv::EE2JointVel_Response::_q1_dot_type arg)
  {
    msg_.q1_dot = std::move(arg);
    return Init_EE2JointVel_Response_q2_dot(msg_);
  }

private:
  ::openmx_interfaces::srv::EE2JointVel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openmx_interfaces::srv::EE2JointVel_Response>()
{
  return openmx_interfaces::srv::builder::Init_EE2JointVel_Response_q1_dot();
}

}  // namespace openmx_interfaces

#endif  // OPENMX_INTERFACES__SRV__DETAIL__EE2_JOINT_VEL__BUILDER_HPP_
