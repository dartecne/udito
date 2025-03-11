// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interfaces:srv/Command.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__BUILDER_HPP_
#define TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_interfaces/srv/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_interfaces
{

namespace srv
{

namespace builder
{

class Init_Command_Request_data
{
public:
  Init_Command_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tutorial_interfaces::srv::Command_Request data(::tutorial_interfaces::srv::Command_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::srv::Command_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::srv::Command_Request>()
{
  return tutorial_interfaces::srv::builder::Init_Command_Request_data();
}

}  // namespace tutorial_interfaces


namespace tutorial_interfaces
{

namespace srv
{

namespace builder
{

class Init_Command_Response_rta
{
public:
  Init_Command_Response_rta()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tutorial_interfaces::srv::Command_Response rta(::tutorial_interfaces::srv::Command_Response::_rta_type arg)
  {
    msg_.rta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::srv::Command_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::srv::Command_Response>()
{
  return tutorial_interfaces::srv::builder::Init_Command_Response_rta();
}

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__BUILDER_HPP_
