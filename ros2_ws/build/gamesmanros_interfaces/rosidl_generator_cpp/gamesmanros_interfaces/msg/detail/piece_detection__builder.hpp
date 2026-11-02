// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gamesmanros_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__BUILDER_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gamesmanros_interfaces/msg/detail/piece_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gamesmanros_interfaces
{

namespace msg
{

namespace builder
{

class Init_PieceDetection_offset_mm
{
public:
  explicit Init_PieceDetection_offset_mm(::gamesmanros_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::msg::PieceDetection offset_mm(::gamesmanros_interfaces::msg::PieceDetection::_offset_mm_type arg)
  {
    msg_.offset_mm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_board_y
{
public:
  explicit Init_PieceDetection_board_y(::gamesmanros_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  Init_PieceDetection_offset_mm board_y(::gamesmanros_interfaces::msg::PieceDetection::_board_y_type arg)
  {
    msg_.board_y = std::move(arg);
    return Init_PieceDetection_offset_mm(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_board_x
{
public:
  explicit Init_PieceDetection_board_x(::gamesmanros_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  Init_PieceDetection_board_y board_x(::gamesmanros_interfaces::msg::PieceDetection::_board_x_type arg)
  {
    msg_.board_x = std::move(arg);
    return Init_PieceDetection_board_y(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_cell_index
{
public:
  explicit Init_PieceDetection_cell_index(::gamesmanros_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  Init_PieceDetection_board_x cell_index(::gamesmanros_interfaces::msg::PieceDetection::_cell_index_type arg)
  {
    msg_.cell_index = std::move(arg);
    return Init_PieceDetection_board_x(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_tag_id
{
public:
  Init_PieceDetection_tag_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PieceDetection_cell_index tag_id(::gamesmanros_interfaces::msg::PieceDetection::_tag_id_type arg)
  {
    msg_.tag_id = std::move(arg);
    return Init_PieceDetection_cell_index(msg_);
  }

private:
  ::gamesmanros_interfaces::msg::PieceDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::msg::PieceDetection>()
{
  return gamesmanros_interfaces::msg::builder::Init_PieceDetection_tag_id();
}

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__BUILDER_HPP_
