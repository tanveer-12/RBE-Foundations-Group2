// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from openmx_interfaces:srv/JointVel2EE.idl
// generated code does not contain a copyright notice

#ifndef OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__STRUCT_HPP_
#define OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__openmx_interfaces__srv__JointVel2EE_Request __attribute__((deprecated))
#else
# define DEPRECATED__openmx_interfaces__srv__JointVel2EE_Request __declspec(deprecated)
#endif

namespace openmx_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct JointVel2EE_Request_
{
  using Type = JointVel2EE_Request_<ContainerAllocator>;

  explicit JointVel2EE_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q1 = 0.0;
      this->q2 = 0.0;
      this->q3 = 0.0;
      this->q4 = 0.0;
      this->q1_dot = 0.0;
      this->q2_dot = 0.0;
      this->q3_dot = 0.0;
      this->q4_dot = 0.0;
    }
  }

  explicit JointVel2EE_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q1 = 0.0;
      this->q2 = 0.0;
      this->q3 = 0.0;
      this->q4 = 0.0;
      this->q1_dot = 0.0;
      this->q2_dot = 0.0;
      this->q3_dot = 0.0;
      this->q4_dot = 0.0;
    }
  }

  // field types and members
  using _q1_type =
    double;
  _q1_type q1;
  using _q2_type =
    double;
  _q2_type q2;
  using _q3_type =
    double;
  _q3_type q3;
  using _q4_type =
    double;
  _q4_type q4;
  using _q1_dot_type =
    double;
  _q1_dot_type q1_dot;
  using _q2_dot_type =
    double;
  _q2_dot_type q2_dot;
  using _q3_dot_type =
    double;
  _q3_dot_type q3_dot;
  using _q4_dot_type =
    double;
  _q4_dot_type q4_dot;

  // setters for named parameter idiom
  Type & set__q1(
    const double & _arg)
  {
    this->q1 = _arg;
    return *this;
  }
  Type & set__q2(
    const double & _arg)
  {
    this->q2 = _arg;
    return *this;
  }
  Type & set__q3(
    const double & _arg)
  {
    this->q3 = _arg;
    return *this;
  }
  Type & set__q4(
    const double & _arg)
  {
    this->q4 = _arg;
    return *this;
  }
  Type & set__q1_dot(
    const double & _arg)
  {
    this->q1_dot = _arg;
    return *this;
  }
  Type & set__q2_dot(
    const double & _arg)
  {
    this->q2_dot = _arg;
    return *this;
  }
  Type & set__q3_dot(
    const double & _arg)
  {
    this->q3_dot = _arg;
    return *this;
  }
  Type & set__q4_dot(
    const double & _arg)
  {
    this->q4_dot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__openmx_interfaces__srv__JointVel2EE_Request
    std::shared_ptr<openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__openmx_interfaces__srv__JointVel2EE_Request
    std::shared_ptr<openmx_interfaces::srv::JointVel2EE_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointVel2EE_Request_ & other) const
  {
    if (this->q1 != other.q1) {
      return false;
    }
    if (this->q2 != other.q2) {
      return false;
    }
    if (this->q3 != other.q3) {
      return false;
    }
    if (this->q4 != other.q4) {
      return false;
    }
    if (this->q1_dot != other.q1_dot) {
      return false;
    }
    if (this->q2_dot != other.q2_dot) {
      return false;
    }
    if (this->q3_dot != other.q3_dot) {
      return false;
    }
    if (this->q4_dot != other.q4_dot) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointVel2EE_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointVel2EE_Request_

// alias to use template instance with default allocator
using JointVel2EE_Request =
  openmx_interfaces::srv::JointVel2EE_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace openmx_interfaces


#ifndef _WIN32
# define DEPRECATED__openmx_interfaces__srv__JointVel2EE_Response __attribute__((deprecated))
#else
# define DEPRECATED__openmx_interfaces__srv__JointVel2EE_Response __declspec(deprecated)
#endif

namespace openmx_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct JointVel2EE_Response_
{
  using Type = JointVel2EE_Response_<ContainerAllocator>;

  explicit JointVel2EE_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0.0;
      this->vy = 0.0;
      this->vz = 0.0;
      this->wx = 0.0;
      this->wy = 0.0;
      this->wz = 0.0;
      this->success = false;
      this->message = "";
    }
  }

  explicit JointVel2EE_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0.0;
      this->vy = 0.0;
      this->vz = 0.0;
      this->wx = 0.0;
      this->wy = 0.0;
      this->wz = 0.0;
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _vx_type =
    double;
  _vx_type vx;
  using _vy_type =
    double;
  _vy_type vy;
  using _vz_type =
    double;
  _vz_type vz;
  using _wx_type =
    double;
  _wx_type wx;
  using _wy_type =
    double;
  _wy_type wy;
  using _wz_type =
    double;
  _wz_type wz;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__vx(
    const double & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const double & _arg)
  {
    this->vy = _arg;
    return *this;
  }
  Type & set__vz(
    const double & _arg)
  {
    this->vz = _arg;
    return *this;
  }
  Type & set__wx(
    const double & _arg)
  {
    this->wx = _arg;
    return *this;
  }
  Type & set__wy(
    const double & _arg)
  {
    this->wy = _arg;
    return *this;
  }
  Type & set__wz(
    const double & _arg)
  {
    this->wz = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__openmx_interfaces__srv__JointVel2EE_Response
    std::shared_ptr<openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__openmx_interfaces__srv__JointVel2EE_Response
    std::shared_ptr<openmx_interfaces::srv::JointVel2EE_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointVel2EE_Response_ & other) const
  {
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    if (this->vz != other.vz) {
      return false;
    }
    if (this->wx != other.wx) {
      return false;
    }
    if (this->wy != other.wy) {
      return false;
    }
    if (this->wz != other.wz) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointVel2EE_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointVel2EE_Response_

// alias to use template instance with default allocator
using JointVel2EE_Response =
  openmx_interfaces::srv::JointVel2EE_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace openmx_interfaces

namespace openmx_interfaces
{

namespace srv
{

struct JointVel2EE
{
  using Request = openmx_interfaces::srv::JointVel2EE_Request;
  using Response = openmx_interfaces::srv::JointVel2EE_Response;
};

}  // namespace srv

}  // namespace openmx_interfaces

#endif  // OPENMX_INTERFACES__SRV__DETAIL__JOINT_VEL2_EE__STRUCT_HPP_
