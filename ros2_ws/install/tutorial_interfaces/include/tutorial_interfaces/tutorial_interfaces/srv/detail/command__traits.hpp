// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tutorial_interfaces:srv/Command.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__TRAITS_HPP_
#define TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tutorial_interfaces/srv/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tutorial_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Command_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_Request & msg, bool use_flow_style = false)
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

}  // namespace tutorial_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use tutorial_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tutorial_interfaces::srv::Command_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  tutorial_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tutorial_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const tutorial_interfaces::srv::Command_Request & msg)
{
  return tutorial_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tutorial_interfaces::srv::Command_Request>()
{
  return "tutorial_interfaces::srv::Command_Request";
}

template<>
inline const char * name<tutorial_interfaces::srv::Command_Request>()
{
  return "tutorial_interfaces/srv/Command_Request";
}

template<>
struct has_fixed_size<tutorial_interfaces::srv::Command_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tutorial_interfaces::srv::Command_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tutorial_interfaces::srv::Command_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace tutorial_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Command_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: rta
  {
    out << "rta: ";
    rosidl_generator_traits::value_to_yaml(msg.rta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rta: ";
    rosidl_generator_traits::value_to_yaml(msg.rta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_Response & msg, bool use_flow_style = false)
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

}  // namespace tutorial_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use tutorial_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tutorial_interfaces::srv::Command_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  tutorial_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tutorial_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const tutorial_interfaces::srv::Command_Response & msg)
{
  return tutorial_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tutorial_interfaces::srv::Command_Response>()
{
  return "tutorial_interfaces::srv::Command_Response";
}

template<>
inline const char * name<tutorial_interfaces::srv::Command_Response>()
{
  return "tutorial_interfaces/srv/Command_Response";
}

template<>
struct has_fixed_size<tutorial_interfaces::srv::Command_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tutorial_interfaces::srv::Command_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tutorial_interfaces::srv::Command_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tutorial_interfaces::srv::Command>()
{
  return "tutorial_interfaces::srv::Command";
}

template<>
inline const char * name<tutorial_interfaces::srv::Command>()
{
  return "tutorial_interfaces/srv/Command";
}

template<>
struct has_fixed_size<tutorial_interfaces::srv::Command>
  : std::integral_constant<
    bool,
    has_fixed_size<tutorial_interfaces::srv::Command_Request>::value &&
    has_fixed_size<tutorial_interfaces::srv::Command_Response>::value
  >
{
};

template<>
struct has_bounded_size<tutorial_interfaces::srv::Command>
  : std::integral_constant<
    bool,
    has_bounded_size<tutorial_interfaces::srv::Command_Request>::value &&
    has_bounded_size<tutorial_interfaces::srv::Command_Response>::value
  >
{
};

template<>
struct is_service<tutorial_interfaces::srv::Command>
  : std::true_type
{
};

template<>
struct is_service_request<tutorial_interfaces::srv::Command_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tutorial_interfaces::srv::Command_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TUTORIAL_INTERFACES__SRV__DETAIL__COMMAND__TRAITS_HPP_
