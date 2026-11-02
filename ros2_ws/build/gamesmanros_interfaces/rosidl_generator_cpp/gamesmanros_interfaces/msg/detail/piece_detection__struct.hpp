// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gamesmanros_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_HPP_
#define GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__msg__PieceDetection __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__msg__PieceDetection __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PieceDetection_
{
  using Type = PieceDetection_<ContainerAllocator>;

  explicit PieceDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tag_id = 0l;
      this->cell_index = 0l;
      this->board_x = 0.0;
      this->board_y = 0.0;
      this->offset_mm = 0.0;
    }
  }

  explicit PieceDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tag_id = 0l;
      this->cell_index = 0l;
      this->board_x = 0.0;
      this->board_y = 0.0;
      this->offset_mm = 0.0;
    }
  }

  // field types and members
  using _tag_id_type =
    int32_t;
  _tag_id_type tag_id;
  using _cell_index_type =
    int32_t;
  _cell_index_type cell_index;
  using _board_x_type =
    double;
  _board_x_type board_x;
  using _board_y_type =
    double;
  _board_y_type board_y;
  using _offset_mm_type =
    double;
  _offset_mm_type offset_mm;

  // setters for named parameter idiom
  Type & set__tag_id(
    const int32_t & _arg)
  {
    this->tag_id = _arg;
    return *this;
  }
  Type & set__cell_index(
    const int32_t & _arg)
  {
    this->cell_index = _arg;
    return *this;
  }
  Type & set__board_x(
    const double & _arg)
  {
    this->board_x = _arg;
    return *this;
  }
  Type & set__board_y(
    const double & _arg)
  {
    this->board_y = _arg;
    return *this;
  }
  Type & set__offset_mm(
    const double & _arg)
  {
    this->offset_mm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__msg__PieceDetection
    std::shared_ptr<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__msg__PieceDetection
    std::shared_ptr<gamesmanros_interfaces::msg::PieceDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PieceDetection_ & other) const
  {
    if (this->tag_id != other.tag_id) {
      return false;
    }
    if (this->cell_index != other.cell_index) {
      return false;
    }
    if (this->board_x != other.board_x) {
      return false;
    }
    if (this->board_y != other.board_y) {
      return false;
    }
    if (this->offset_mm != other.offset_mm) {
      return false;
    }
    return true;
  }
  bool operator!=(const PieceDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PieceDetection_

// alias to use template instance with default allocator
using PieceDetection =
  gamesmanros_interfaces::msg::PieceDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_HPP_
