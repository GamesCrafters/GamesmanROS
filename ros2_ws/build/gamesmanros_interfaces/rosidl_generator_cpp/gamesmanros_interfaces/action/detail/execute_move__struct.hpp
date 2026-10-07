// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gamesmanros_interfaces:action/ExecuteMove.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__STRUCT_HPP_
#define GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Goal __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Goal __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_Goal_
{
  using Type = ExecuteMove_Goal_<ContainerAllocator>;

  explicit ExecuteMove_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->game_id = "";
      this->move_string = "";
      this->current_position = "";
      this->new_position = "";
    }
  }

  explicit ExecuteMove_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : game_id(_alloc),
    move_string(_alloc),
    current_position(_alloc),
    new_position(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->game_id = "";
      this->move_string = "";
      this->current_position = "";
      this->new_position = "";
    }
  }

  // field types and members
  using _game_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _game_id_type game_id;
  using _move_string_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _move_string_type move_string;
  using _current_position_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_position_type current_position;
  using _new_position_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _new_position_type new_position;

  // setters for named parameter idiom
  Type & set__game_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->game_id = _arg;
    return *this;
  }
  Type & set__move_string(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->move_string = _arg;
    return *this;
  }
  Type & set__current_position(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_position = _arg;
    return *this;
  }
  Type & set__new_position(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->new_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Goal
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Goal
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_Goal_ & other) const
  {
    if (this->game_id != other.game_id) {
      return false;
    }
    if (this->move_string != other.move_string) {
      return false;
    }
    if (this->current_position != other.current_position) {
      return false;
    }
    if (this->new_position != other.new_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_Goal_

// alias to use template instance with default allocator
using ExecuteMove_Goal =
  gamesmanros_interfaces::action::ExecuteMove_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace gamesmanros_interfaces


#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Result __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Result __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_Result_
{
  using Type = ExecuteMove_Result_<ContainerAllocator>;

  explicit ExecuteMove_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ExecuteMove_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Result
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Result
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_Result_

// alias to use template instance with default allocator
using ExecuteMove_Result =
  gamesmanros_interfaces::action::ExecuteMove_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace gamesmanros_interfaces


#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Feedback __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_Feedback_
{
  using Type = ExecuteMove_Feedback_<ContainerAllocator>;

  explicit ExecuteMove_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  explicit ExecuteMove_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  // field types and members
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Feedback
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_Feedback
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_Feedback_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_Feedback_

// alias to use template instance with default allocator
using ExecuteMove_Feedback =
  gamesmanros_interfaces::action::ExecuteMove_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace gamesmanros_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "gamesmanros_interfaces/action/detail/execute_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_SendGoal_Request_
{
  using Type = ExecuteMove_SendGoal_Request_<ContainerAllocator>;

  explicit ExecuteMove_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit ExecuteMove_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const gamesmanros_interfaces::action::ExecuteMove_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_SendGoal_Request_

// alias to use template instance with default allocator
using ExecuteMove_SendGoal_Request =
  gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace gamesmanros_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_SendGoal_Response_
{
  using Type = ExecuteMove_SendGoal_Response_<ContainerAllocator>;

  explicit ExecuteMove_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit ExecuteMove_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_SendGoal_Response_

// alias to use template instance with default allocator
using ExecuteMove_SendGoal_Response =
  gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace gamesmanros_interfaces
{

namespace action
{

struct ExecuteMove_SendGoal
{
  using Request = gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request;
  using Response = gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response;
};

}  // namespace action

}  // namespace gamesmanros_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_GetResult_Request __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_GetResult_Request_
{
  using Type = ExecuteMove_GetResult_Request_<ContainerAllocator>;

  explicit ExecuteMove_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit ExecuteMove_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_GetResult_Request
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_GetResult_Request
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_GetResult_Request_

// alias to use template instance with default allocator
using ExecuteMove_GetResult_Request =
  gamesmanros_interfaces::action::ExecuteMove_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace gamesmanros_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "gamesmanros_interfaces/action/detail/execute_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_GetResult_Response __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_GetResult_Response_
{
  using Type = ExecuteMove_GetResult_Response_<ContainerAllocator>;

  explicit ExecuteMove_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit ExecuteMove_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const gamesmanros_interfaces::action::ExecuteMove_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_GetResult_Response
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_GetResult_Response
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_GetResult_Response_

// alias to use template instance with default allocator
using ExecuteMove_GetResult_Response =
  gamesmanros_interfaces::action::ExecuteMove_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace gamesmanros_interfaces

namespace gamesmanros_interfaces
{

namespace action
{

struct ExecuteMove_GetResult
{
  using Request = gamesmanros_interfaces::action::ExecuteMove_GetResult_Request;
  using Response = gamesmanros_interfaces::action::ExecuteMove_GetResult_Response;
};

}  // namespace action

}  // namespace gamesmanros_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "gamesmanros_interfaces/action/detail/execute_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage __declspec(deprecated)
#endif

namespace gamesmanros_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_FeedbackMessage_
{
  using Type = ExecuteMove_FeedbackMessage_<ContainerAllocator>;

  explicit ExecuteMove_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit ExecuteMove_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const gamesmanros_interfaces::action::ExecuteMove_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage
    std::shared_ptr<gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_FeedbackMessage_

// alias to use template instance with default allocator
using ExecuteMove_FeedbackMessage =
  gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace gamesmanros_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace gamesmanros_interfaces
{

namespace action
{

struct ExecuteMove
{
  /// The goal message defined in the action definition.
  using Goal = gamesmanros_interfaces::action::ExecuteMove_Goal;
  /// The result message defined in the action definition.
  using Result = gamesmanros_interfaces::action::ExecuteMove_Result;
  /// The feedback message defined in the action definition.
  using Feedback = gamesmanros_interfaces::action::ExecuteMove_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = gamesmanros_interfaces::action::ExecuteMove_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = gamesmanros_interfaces::action::ExecuteMove_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct ExecuteMove ExecuteMove;

}  // namespace action

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__STRUCT_HPP_
