// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tutorial_interfaces:srv/Command.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__STRUCT_HPP_
#define TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tutorial_interfaces__srv__Command_Request __attribute__((deprecated))
#else
# define DEPRECATED__tutorial_interfaces__srv__Command_Request __declspec(deprecated)
#endif

namespace tutorial_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Command_Request_
{
  using Type = Command_Request_<ContainerAllocator>;

  explicit Command_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  explicit Command_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tutorial_interfaces::srv::Command_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const tutorial_interfaces::srv::Command_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tutorial_interfaces::srv::Command_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tutorial_interfaces::srv::Command_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tutorial_interfaces::srv::Command_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tutorial_interfaces::srv::Command_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tutorial_interfaces::srv::Command_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tutorial_interfaces::srv::Command_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tutorial_interfaces::srv::Command_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tutorial_interfaces::srv::Command_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tutorial_interfaces__srv__Command_Request
    std::shared_ptr<tutorial_interfaces::srv::Command_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tutorial_interfaces__srv__Command_Request
    std::shared_ptr<tutorial_interfaces::srv::Command_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_Request_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_Request_

// alias to use template instance with default allocator
using Command_Request =
  tutorial_interfaces::srv::Command_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tutorial_interfaces


#ifndef _WIN32
# define DEPRECATED__tutorial_interfaces__srv__Command_Response __attribute__((deprecated))
#else
# define DEPRECATED__tutorial_interfaces__srv__Command_Response __declspec(deprecated)
#endif

namespace tutorial_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Command_Response_
{
  using Type = Command_Response_<ContainerAllocator>;

  explicit Command_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rta = "";
    }
  }

  explicit Command_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rta(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rta = "";
    }
  }

  // field types and members
  using _rta_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _rta_type rta;

  // setters for named parameter idiom
  Type & set__rta(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->rta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tutorial_interfaces::srv::Command_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const tutorial_interfaces::srv::Command_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tutorial_interfaces::srv::Command_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tutorial_interfaces::srv::Command_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tutorial_interfaces::srv::Command_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tutorial_interfaces::srv::Command_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tutorial_interfaces::srv::Command_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tutorial_interfaces::srv::Command_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tutorial_interfaces::srv::Command_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tutorial_interfaces::srv::Command_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tutorial_interfaces__srv__Command_Response
    std::shared_ptr<tutorial_interfaces::srv::Command_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tutorial_interfaces__srv__Command_Response
    std::shared_ptr<tutorial_interfaces::srv::Command_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_Response_ & other) const
  {
    if (this->rta != other.rta) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_Response_

// alias to use template instance with default allocator
using Command_Response =
  tutorial_interfaces::srv::Command_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tutorial_interfaces

namespace tutorial_interfaces
{

namespace srv
{

struct Command
{
  using Request = tutorial_interfaces::srv::Command_Request;
  using Response = tutorial_interfaces::srv::Command_Response;
};

}  // namespace srv

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__STRUCT_HPP_
