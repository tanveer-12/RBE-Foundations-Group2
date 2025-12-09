// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from openmx_interfaces:srv/JointVel2EE.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__BUILDER_HPP_
#define OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "openmx_interfaces/srv/detail/joint_vel2_ee__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace openmx_interfaces
{

namespace srv
{

namespace builder
{

class Init_JointVel2EE_Request_q4_dot
{
public:
  explicit Init_JointVel2EE_Request_q4_dot(::openmx_interfaces::srv::JointVel2EE_Request & msg)
  : msg_(msg)
  {}
  ::openmx_interfaces::srv::JointVel2EE_Request q4_dot(::openmx_interfaces::srv::JointVel2EE_Request::_q4_dot_type arg)
  {
    msg_.q4_dot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Request msg_;
};

class Init_JointVel2EE_Request_q3_dot
{
public:
  explicit Init_JointVel2EE_Request_q3_dot(::openmx_interfaces::srv::JointVel2EE_Request & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Request_q4_dot q3_dot(::openmx_interfaces::srv::JointVel2EE_Request::_q3_dot_type arg)
  {
    msg_.q3_dot = std::move(arg);
    return Init_JointVel2EE_Request_q4_dot(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Request msg_;
};

class Init_JointVel2EE_Request_q2_dot
{
public:
  explicit Init_JointVel2EE_Request_q2_dot(::openmx_interfaces::srv::JointVel2EE_Request & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Request_q3_dot q2_dot(::openmx_interfaces::srv::JointVel2EE_Request::_q2_dot_type arg)
  {
    msg_.q2_dot = std::move(arg);
    return Init_JointVel2EE_Request_q3_dot(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Request msg_;
};

class Init_JointVel2EE_Request_q1_dot
{
public:
  explicit Init_JointVel2EE_Request_q1_dot(::openmx_interfaces::srv::JointVel2EE_Request & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Request_q2_dot q1_dot(::openmx_interfaces::srv::JointVel2EE_Request::_q1_dot_type arg)
  {
    msg_.q1_dot = std::move(arg);
    return Init_JointVel2EE_Request_q2_dot(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Request msg_;
};

class Init_JointVel2EE_Request_q4
{
public:
  explicit Init_JointVel2EE_Request_q4(::openmx_interfaces::srv::JointVel2EE_Request & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Request_q1_dot q4(::openmx_interfaces::srv::JointVel2EE_Request::_q4_type arg)
  {
    msg_.q4 = std::move(arg);
    return Init_JointVel2EE_Request_q1_dot(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Request msg_;
};

class Init_JointVel2EE_Request_q3
{
public:
  explicit Init_JointVel2EE_Request_q3(::openmx_interfaces::srv::JointVel2EE_Request & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Request_q4 q3(::openmx_interfaces::srv::JointVel2EE_Request::_q3_type arg)
  {
    msg_.q3 = std::move(arg);
    return Init_JointVel2EE_Request_q4(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Request msg_;
};

class Init_JointVel2EE_Request_q2
{
public:
  explicit Init_JointVel2EE_Request_q2(::openmx_interfaces::srv::JointVel2EE_Request & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Request_q3 q2(::openmx_interfaces::srv::JointVel2EE_Request::_q2_type arg)
  {
    msg_.q2 = std::move(arg);
    return Init_JointVel2EE_Request_q3(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Request msg_;
};

class Init_JointVel2EE_Request_q1
{
public:
  Init_JointVel2EE_Request_q1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointVel2EE_Request_q2 q1(::openmx_interfaces::srv::JointVel2EE_Request::_q1_type arg)
  {
    msg_.q1 = std::move(arg);
    return Init_JointVel2EE_Request_q2(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openmx_interfaces::srv::JointVel2EE_Request>()
{
  return openmx_interfaces::srv::builder::Init_JointVel2EE_Request_q1();
}

}  // namespace openmx_interfaces


namespace openmx_interfaces
{

namespace srv
{

namespace builder
{

class Init_JointVel2EE_Response_message
{
public:
  explicit Init_JointVel2EE_Response_message(::openmx_interfaces::srv::JointVel2EE_Response & msg)
  : msg_(msg)
  {}
  ::openmx_interfaces::srv::JointVel2EE_Response message(::openmx_interfaces::srv::JointVel2EE_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Response msg_;
};

class Init_JointVel2EE_Response_success
{
public:
  explicit Init_JointVel2EE_Response_success(::openmx_interfaces::srv::JointVel2EE_Response & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Response_message success(::openmx_interfaces::srv::JointVel2EE_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_JointVel2EE_Response_message(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Response msg_;
};

class Init_JointVel2EE_Response_wz
{
public:
  explicit Init_JointVel2EE_Response_wz(::openmx_interfaces::srv::JointVel2EE_Response & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Response_success wz(::openmx_interfaces::srv::JointVel2EE_Response::_wz_type arg)
  {
    msg_.wz = std::move(arg);
    return Init_JointVel2EE_Response_success(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Response msg_;
};

class Init_JointVel2EE_Response_wy
{
public:
  explicit Init_JointVel2EE_Response_wy(::openmx_interfaces::srv::JointVel2EE_Response & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Response_wz wy(::openmx_interfaces::srv::JointVel2EE_Response::_wy_type arg)
  {
    msg_.wy = std::move(arg);
    return Init_JointVel2EE_Response_wz(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Response msg_;
};

class Init_JointVel2EE_Response_wx
{
public:
  explicit Init_JointVel2EE_Response_wx(::openmx_interfaces::srv::JointVel2EE_Response & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Response_wy wx(::openmx_interfaces::srv::JointVel2EE_Response::_wx_type arg)
  {
    msg_.wx = std::move(arg);
    return Init_JointVel2EE_Response_wy(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Response msg_;
};

class Init_JointVel2EE_Response_vz
{
public:
  explicit Init_JointVel2EE_Response_vz(::openmx_interfaces::srv::JointVel2EE_Response & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Response_wx vz(::openmx_interfaces::srv::JointVel2EE_Response::_vz_type arg)
  {
    msg_.vz = std::move(arg);
    return Init_JointVel2EE_Response_wx(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Response msg_;
};

class Init_JointVel2EE_Response_vy
{
public:
  explicit Init_JointVel2EE_Response_vy(::openmx_interfaces::srv::JointVel2EE_Response & msg)
  : msg_(msg)
  {}
  Init_JointVel2EE_Response_vz vy(::openmx_interfaces::srv::JointVel2EE_Response::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_JointVel2EE_Response_vz(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Response msg_;
};

class Init_JointVel2EE_Response_vx
{
public:
  Init_JointVel2EE_Response_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointVel2EE_Response_vy vx(::openmx_interfaces::srv::JointVel2EE_Response::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_JointVel2EE_Response_vy(msg_);
  }

private:
  ::openmx_interfaces::srv::JointVel2EE_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openmx_interfaces::srv::JointVel2EE_Response>()
{
  return openmx_interfaces::srv::builder::Init_JointVel2EE_Response_vx();
}

}  // namespace openmx_interfaces

#endif  // OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__BUILDER_HPP_
