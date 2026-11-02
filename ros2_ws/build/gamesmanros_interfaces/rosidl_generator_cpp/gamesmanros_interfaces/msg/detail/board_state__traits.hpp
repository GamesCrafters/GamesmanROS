// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gamesmanros_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__TRAITS_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gamesmanros_interfaces/msg/detail/board_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pieces'
#include "gamesmanros_interfaces/msg/detail/piece_detection__traits.hpp"

namespace gamesmanros_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoardState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: occupancy
  {
    out << "occupancy: ";
    rosidl_generator_traits::value_to_yaml(msg.occupancy, out);
    out << ", ";
  }

  // member: pieces
  {
    if (msg.pieces.size() == 0) {
      out << "pieces: []";
    } else {
      out << "pieces: [";
      size_t pending_items = msg.pieces.size();
      for (auto item : msg.pieces) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: board_located
  {
    out << "board_located: ";
    rosidl_generator_traits::value_to_yaml(msg.board_located, out);
    out << ", ";
  }

  // member: frames_voted
  {
    out << "frames_voted: ";
    rosidl_generator_traits::value_to_yaml(msg.frames_voted, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BoardState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: occupancy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "occupancy: ";
    rosidl_generator_traits::value_to_yaml(msg.occupancy, out);
    out << "\n";
  }

  // member: pieces
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pieces.size() == 0) {
      out << "pieces: []\n";
    } else {
      out << "pieces:\n";
      for (auto item : msg.pieces) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: board_located
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "board_located: ";
    rosidl_generator_traits::value_to_yaml(msg.board_located, out);
    out << "\n";
  }

  // member: frames_voted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frames_voted: ";
    rosidl_generator_traits::value_to_yaml(msg.frames_voted, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoardState & msg, bool use_flow_style = false)
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
  const gamesmanros_interfaces::msg::BoardState & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::msg::BoardState & msg)
{
  return gamesmanros_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::msg::BoardState>()
{
  return "gamesmanros_interfaces::msg::BoardState";
}

template<>
inline const char * name<gamesmanros_interfaces::msg::BoardState>()
{
  return "gamesmanros_interfaces/msg/BoardState";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::msg::BoardState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::msg::BoardState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<gamesmanros_interfaces::msg::BoardState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__TRAITS_HPP_
