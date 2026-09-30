// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gamesmanros_interfaces:msg/GameState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__BUILDER_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gamesmanros_interfaces/msg/detail/game_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gamesmanros_interfaces
{

namespace msg
{

namespace builder
{

class Init_GameState_game_over
{
public:
  explicit Init_GameState_game_over(::gamesmanros_interfaces::msg::GameState & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::msg::GameState game_over(::gamesmanros_interfaces::msg::GameState::_game_over_type arg)
  {
    msg_.game_over = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::GameState msg_;
};

class Init_GameState_is_robot_turn
{
public:
  explicit Init_GameState_is_robot_turn(::gamesmanros_interfaces::msg::GameState & msg)
  : msg_(msg)
  {}
  Init_GameState_game_over is_robot_turn(::gamesmanros_interfaces::msg::GameState::_is_robot_turn_type arg)
  {
    msg_.is_robot_turn = std::move(arg);
    return Init_GameState_game_over(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::GameState msg_;
};

class Init_GameState_available_moves
{
public:
  explicit Init_GameState_available_moves(::gamesmanros_interfaces::msg::GameState & msg)
  : msg_(msg)
  {}
  Init_GameState_is_robot_turn available_moves(::gamesmanros_interfaces::msg::GameState::_available_moves_type arg)
  {
    msg_.available_moves = std::move(arg);
    return Init_GameState_is_robot_turn(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::GameState msg_;
};

class Init_GameState_position
{
public:
  Init_GameState_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GameState_available_moves position(::gamesmanros_interfaces::msg::GameState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_GameState_available_moves(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::GameState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::msg::GameState>()
{
  return gamesmanros_interfaces::msg::builder::Init_GameState_position();
}

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__BUILDER_HPP_
