// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gamesmanros_interfaces:msg/GameState.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__STRUCT_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__msg__GameState __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__msg__GameState __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GameState_
{
  using Type = GameState_<ContainerAllocator>;

  explicit GameState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position = "";
      this->is_robot_turn = false;
      this->game_over = false;
    }
  }

  explicit GameState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position = "";
      this->is_robot_turn = false;
      this->game_over = false;
    }
  }

  // field types and members
  using _position_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _position_type position;
  using _available_moves_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _available_moves_type available_moves;
  using _is_robot_turn_type =
    bool;
  _is_robot_turn_type is_robot_turn;
  using _game_over_type =
    bool;
  _game_over_type game_over;

  // setters for named parameter idiom
  Type & set__position(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__available_moves(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->available_moves = _arg;
    return *this;
  }
  Type & set__is_robot_turn(
    const bool & _arg)
  {
    this->is_robot_turn = _arg;
    return *this;
  }
  Type & set__game_over(
    const bool & _arg)
  {
    this->game_over = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::msg::GameState_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::msg::GameState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::msg::GameState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::msg::GameState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::msg::GameState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::msg::GameState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::msg::GameState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::msg::GameState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::msg::GameState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::msg::GameState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__msg__GameState
    std::shared_ptr<gamesmanros_interfaces::msg::GameState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__msg__GameState
    std::shared_ptr<gamesmanros_interfaces::msg::GameState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GameState_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->available_moves != other.available_moves) {
      return false;
    }
    if (this->is_robot_turn != other.is_robot_turn) {
      return false;
    }
    if (this->game_over != other.game_over) {
      return false;
    }
    return true;
  }
  bool operator!=(const GameState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GameState_

// alias to use template instance with default allocator
using GameState =
  gamesmanros_interfaces::msg::GameState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__GAME_STATE__STRUCT_HPP_
