// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gamesmanros_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pieces'
#include "gamesmanros_interfaces/msg/detail/piece_detection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__msg__BoardState __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__msg__BoardState __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoardState_
{
  using Type = BoardState_<ContainerAllocator>;

  explicit BoardState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->occupancy = "";
      this->board_located = false;
      this->frames_voted = 0l;
    }
  }

  explicit BoardState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    occupancy(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->occupancy = "";
      this->board_located = false;
      this->frames_voted = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _occupancy_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _occupancy_type occupancy;
  using _pieces_type =
    std::vector<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>>>;
  _pieces_type pieces;
  using _board_located_type =
    bool;
  _board_located_type board_located;
  using _frames_voted_type =
    int32_t;
  _frames_voted_type frames_voted;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__occupancy(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->occupancy = _arg;
    return *this;
  }
  Type & set__pieces(
    const std::vector<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>>> & _arg)
  {
    this->pieces = _arg;
    return *this;
  }
  Type & set__board_located(
    const bool & _arg)
  {
    this->board_located = _arg;
    return *this;
  }
  Type & set__frames_voted(
    const int32_t & _arg)
  {
    this->frames_voted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::msg::BoardState_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::msg::BoardState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::msg::BoardState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::msg::BoardState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::msg::BoardState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::msg::BoardState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::msg::BoardState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::msg::BoardState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::msg::BoardState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::msg::BoardState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__msg__BoardState
    std::shared_ptr<gamesmanros_interfaces::msg::BoardState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__msg__BoardState
    std::shared_ptr<gamesmanros_interfaces::msg::BoardState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoardState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->occupancy != other.occupancy) {
      return false;
    }
    if (this->pieces != other.pieces) {
      return false;
    }
    if (this->board_located != other.board_located) {
      return false;
    }
    if (this->frames_voted != other.frames_voted) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoardState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoardState_

// alias to use template instance with default allocator
using BoardState =
  gamesmanros_interfaces::msg::BoardState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_HPP_
