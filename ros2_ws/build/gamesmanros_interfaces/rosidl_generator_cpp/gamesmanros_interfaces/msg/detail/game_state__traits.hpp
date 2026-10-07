// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gamesmanros_interfaces:msg/GameState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__TRAITS_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gamesmanros_interfaces/msg/detail/game_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace gamesmanros_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GameState & msg,
  std::ostream & out)
{
  out << "{";
  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: available_moves
  {
    if (msg.available_moves.size() == 0) {
      out << "available_moves: []";
    } else {
      out << "available_moves: [";
      size_t pending_items = msg.available_moves.size();
      for (auto item : msg.available_moves) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: is_robot_turn
  {
    out << "is_robot_turn: ";
    rosidl_generator_traits::value_to_yaml(msg.is_robot_turn, out);
    out << ", ";
  }

  // member: game_over
  {
    out << "game_over: ";
    rosidl_generator_traits::value_to_yaml(msg.game_over, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GameState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: available_moves
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.available_moves.size() == 0) {
      out << "available_moves: []\n";
    } else {
      out << "available_moves:\n";
      for (auto item : msg.available_moves) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: is_robot_turn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_robot_turn: ";
    rosidl_generator_traits::value_to_yaml(msg.is_robot_turn, out);
    out << "\n";
  }

  // member: game_over
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "game_over: ";
    rosidl_generator_traits::value_to_yaml(msg.game_over, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GameState & msg, bool use_flow_style = false)
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
  const gamesmanros_interfaces::msg::GameState & msg,
  std::ostream & out, size_t indentation = 0)
{
  gamesmanros_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gamesmanros_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const gamesmanros_interfaces::msg::GameState & msg)
{
  return gamesmanros_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<gamesmanros_interfaces::msg::GameState>()
{
  return "gamesmanros_interfaces::msg::GameState";
}

template<>
inline const char * name<gamesmanros_interfaces::msg::GameState>()
{
  return "gamesmanros_interfaces/msg/GameState";
}

template<>
struct has_fixed_size<gamesmanros_interfaces::msg::GameState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<gamesmanros_interfaces::msg::GameState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<gamesmanros_interfaces::msg::GameState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__TRAITS_HPP_
