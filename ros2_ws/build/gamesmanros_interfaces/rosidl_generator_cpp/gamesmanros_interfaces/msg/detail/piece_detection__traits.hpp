// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gamesmanros_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__TRAITS_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gamesmanros_interfaces/msg/detail/piece_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace gamesmanros_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PieceDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: tag_id
  {
    out << "tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_id, out);
    out << ", ";
  }

  // member: cell_index
  {
    out << "cell_index: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_index, out);
    out << ", ";
  }

  // member: board_x
  {
    out << "board_x: ";
    rosidl_generator_traits::value_to_yaml(msg.board_x, out);
    out << ", ";
  }

  // member: board_y
  {
    out << "board_y: ";
    rosidl_generator_traits::value_to_yaml(msg.board_y, out);
    out << ", ";
  }

  // member: offset_mm
  {
    out << "offset_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_mm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PieceDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_id, out);
    out << "\n";
  }

  // member: cell_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_index: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_index, out);
    out << "\n";
  }

  // member: board_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "board_x: ";
    rosidl_generator_traits::value_to_yaml(msg.board_x, out);
    out << "\n";
  }

  // member: board_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "board_y: ";
    rosidl_generator_traits::value_to_yaml(msg.board_y, out);
    out << "\n";
  }

  // member: offset_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_mm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PieceDetection & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace gamesmanros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gamesmanros_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gamesmanros_interfaces::msg::PieceDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::msg::PieceDetection & msg)
{
  return gamesmanros_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::msg::PieceDetection>()
{
  return "gamesmanros_interfaces::msg::PieceDetection";
}

template<>
inline const char * name<gamesmanros_interfaces::msg::PieceDetection>()
{
  return "gamesmanros_interfaces/msg/PieceDetection";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::msg::PieceDetection>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::msg::PieceDetection>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gamesmanros_interfaces::msg::PieceDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__TRAITS_HPP_
