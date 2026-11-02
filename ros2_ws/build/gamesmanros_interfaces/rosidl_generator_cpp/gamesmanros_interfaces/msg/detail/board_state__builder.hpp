// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gamesmanros_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__BUILDER_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gamesmanros_interfaces/msg/detail/board_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gamesmanros_interfaces
{

namespace msg
{

namespace builder
{

class Init_BoardState_frames_voted
{
public:
  explicit Init_BoardState_frames_voted(::gamesmanros_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::msg::BoardState frames_voted(::gamesmanros_interfaces::msg::BoardState::_frames_voted_type arg)
  {
    msg_.frames_voted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::BoardState msg_;
};

class Init_BoardState_board_located
{
public:
  explicit Init_BoardState_board_located(::gamesmanros_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_frames_voted board_located(::gamesmanros_interfaces::msg::BoardState::_board_located_type arg)
  {
    msg_.board_located = std::move(arg);
    return Init_BoardState_frames_voted(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::BoardState msg_;
};

class Init_BoardState_pieces
{
public:
  explicit Init_BoardState_pieces(::gamesmanros_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_board_located pieces(::gamesmanros_interfaces::msg::BoardState::_pieces_type arg)
  {
    msg_.pieces = std::move(arg);
    return Init_BoardState_board_located(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::BoardState msg_;
};

class Init_BoardState_occupancy
{
public:
  explicit Init_BoardState_occupancy(::gamesmanros_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_pieces occupancy(::gamesmanros_interfaces::msg::BoardState::_occupancy_type arg)
  {
    msg_.occupancy = std::move(arg);
    return Init_BoardState_pieces(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::BoardState msg_;
};

class Init_BoardState_header
{
public:
  Init_BoardState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoardState_occupancy header(::gamesmanros_interfaces::msg::BoardState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BoardState_occupancy(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::BoardState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::msg::BoardState>()
{
  return gamesmanros_interfaces::msg::builder::Init_BoardState_header();
}

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__BUILDER_HPP_
