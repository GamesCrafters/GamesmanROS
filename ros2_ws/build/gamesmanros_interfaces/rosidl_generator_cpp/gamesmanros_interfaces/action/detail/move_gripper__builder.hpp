// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gamesmanros_interfaces:action/MoveGripper.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_GRIPPER__BUILDER_HPP_
#define GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_GRIPPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gamesmanros_interfaces/action/detail/move_gripper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveGripper_Goal_open
{
public:
  Init_MoveGripper_Goal_open()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::MoveGripper_Goal open(::gamesmanros_interfaces::action::MoveGripper_Goal::_open_type arg)
  {
    msg_.open = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveGripper_Goal>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveGripper_Goal_open();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveGripper_Result_success
{
public:
  Init_MoveGripper_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::MoveGripper_Result success(::gamesmanros_interfaces::action::MoveGripper_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveGripper_Result>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveGripper_Result_success();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveGripper_Feedback_status
{
public:
  Init_MoveGripper_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::MoveGripper_Feedback status(::gamesmanros_interfaces::action::MoveGripper_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveGripper_Feedback>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveGripper_Feedback_status();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveGripper_SendGoal_Request_goal
{
public:
  explicit Init_MoveGripper_SendGoal_Request_goal(::gamesmanros_interfaces::action::MoveGripper_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveGripper_SendGoal_Request goal(::gamesmanros_interfaces::action::MoveGripper_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_SendGoal_Request msg_;
};

class Init_MoveGripper_SendGoal_Request_goal_id
{
public:
  Init_MoveGripper_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveGripper_SendGoal_Request_goal goal_id(::gamesmanros_interfaces::action::MoveGripper_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveGripper_SendGoal_Request_goal(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveGripper_SendGoal_Request>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveGripper_SendGoal_Request_goal_id();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveGripper_SendGoal_Response_stamp
{
public:
  explicit Init_MoveGripper_SendGoal_Response_stamp(::gamesmanros_interfaces::action::MoveGripper_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveGripper_SendGoal_Response stamp(::gamesmanros_interfaces::action::MoveGripper_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_SendGoal_Response msg_;
};

class Init_MoveGripper_SendGoal_Response_accepted
{
public:
  Init_MoveGripper_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveGripper_SendGoal_Response_stamp accepted(::gamesmanros_interfaces::action::MoveGripper_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MoveGripper_SendGoal_Response_stamp(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveGripper_SendGoal_Response>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveGripper_SendGoal_Response_accepted();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveGripper_GetResult_Request_goal_id
{
public:
  Init_MoveGripper_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::MoveGripper_GetResult_Request goal_id(::gamesmanros_interfaces::action::MoveGripper_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveGripper_GetResult_Request>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveGripper_GetResult_Request_goal_id();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveGripper_GetResult_Response_result
{
public:
  explicit Init_MoveGripper_GetResult_Response_result(::gamesmanros_interfaces::action::MoveGripper_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveGripper_GetResult_Response result(::gamesmanros_interfaces::action::MoveGripper_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_GetResult_Response msg_;
};

class Init_MoveGripper_GetResult_Response_status
{
public:
  Init_MoveGripper_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveGripper_GetResult_Response_result status(::gamesmanros_interfaces::action::MoveGripper_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveGripper_GetResult_Response_result(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveGripper_GetResult_Response>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveGripper_GetResult_Response_status();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveGripper_FeedbackMessage_feedback
{
public:
  explicit Init_MoveGripper_FeedbackMessage_feedback(::gamesmanros_interfaces::action::MoveGripper_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveGripper_FeedbackMessage feedback(::gamesmanros_interfaces::action::MoveGripper_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_FeedbackMessage msg_;
};

class Init_MoveGripper_FeedbackMessage_goal_id
{
public:
  Init_MoveGripper_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveGripper_FeedbackMessage_feedback goal_id(::gamesmanros_interfaces::action::MoveGripper_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveGripper_FeedbackMessage_feedback(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveGripper_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveGripper_FeedbackMessage>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveGripper_FeedbackMessage_goal_id();
}

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_GRIPPER__BUILDER_HPP_
