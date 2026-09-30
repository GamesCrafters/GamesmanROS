// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gamesmanros_interfaces:action/MoveArm.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_ARM__BUILDER_HPP_
#define GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_ARM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gamesmanros_interfaces/action/detail/move_arm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveArm_Goal_z
{
public:
  explicit Init_MoveArm_Goal_z(::gamesmanros_interfaces::action::MoveArm_Goal & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveArm_Goal z(::gamesmanros_interfaces::action::MoveArm_Goal::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_Goal msg_;
};

class Init_MoveArm_Goal_y
{
public:
  explicit Init_MoveArm_Goal_y(::gamesmanros_interfaces::action::MoveArm_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveArm_Goal_z y(::gamesmanros_interfaces::action::MoveArm_Goal::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MoveArm_Goal_z(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_Goal msg_;
};

class Init_MoveArm_Goal_x
{
public:
  Init_MoveArm_Goal_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveArm_Goal_y x(::gamesmanros_interfaces::action::MoveArm_Goal::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MoveArm_Goal_y(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveArm_Goal>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveArm_Goal_x();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveArm_Result_message
{
public:
  explicit Init_MoveArm_Result_message(::gamesmanros_interfaces::action::MoveArm_Result & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveArm_Result message(::gamesmanros_interfaces::action::MoveArm_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_Result msg_;
};

class Init_MoveArm_Result_success
{
public:
  Init_MoveArm_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveArm_Result_message success(::gamesmanros_interfaces::action::MoveArm_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveArm_Result_message(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveArm_Result>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveArm_Result_success();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveArm_Feedback_status
{
public:
  Init_MoveArm_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::MoveArm_Feedback status(::gamesmanros_interfaces::action::MoveArm_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveArm_Feedback>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveArm_Feedback_status();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveArm_SendGoal_Request_goal
{
public:
  explicit Init_MoveArm_SendGoal_Request_goal(::gamesmanros_interfaces::action::MoveArm_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveArm_SendGoal_Request goal(::gamesmanros_interfaces::action::MoveArm_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_SendGoal_Request msg_;
};

class Init_MoveArm_SendGoal_Request_goal_id
{
public:
  Init_MoveArm_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveArm_SendGoal_Request_goal goal_id(::gamesmanros_interfaces::action::MoveArm_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveArm_SendGoal_Request_goal(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveArm_SendGoal_Request>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveArm_SendGoal_Request_goal_id();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveArm_SendGoal_Response_stamp
{
public:
  explicit Init_MoveArm_SendGoal_Response_stamp(::gamesmanros_interfaces::action::MoveArm_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveArm_SendGoal_Response stamp(::gamesmanros_interfaces::action::MoveArm_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_SendGoal_Response msg_;
};

class Init_MoveArm_SendGoal_Response_accepted
{
public:
  Init_MoveArm_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveArm_SendGoal_Response_stamp accepted(::gamesmanros_interfaces::action::MoveArm_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MoveArm_SendGoal_Response_stamp(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveArm_SendGoal_Response>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveArm_SendGoal_Response_accepted();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveArm_GetResult_Request_goal_id
{
public:
  Init_MoveArm_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::MoveArm_GetResult_Request goal_id(::gamesmanros_interfaces::action::MoveArm_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveArm_GetResult_Request>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveArm_GetResult_Request_goal_id();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveArm_GetResult_Response_result
{
public:
  explicit Init_MoveArm_GetResult_Response_result(::gamesmanros_interfaces::action::MoveArm_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveArm_GetResult_Response result(::gamesmanros_interfaces::action::MoveArm_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_GetResult_Response msg_;
};

class Init_MoveArm_GetResult_Response_status
{
public:
  Init_MoveArm_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveArm_GetResult_Response_result status(::gamesmanros_interfaces::action::MoveArm_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveArm_GetResult_Response_result(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveArm_GetResult_Response>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveArm_GetResult_Response_status();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveArm_FeedbackMessage_feedback
{
public:
  explicit Init_MoveArm_FeedbackMessage_feedback(::gamesmanros_interfaces::action::MoveArm_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::MoveArm_FeedbackMessage feedback(::gamesmanros_interfaces::action::MoveArm_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_FeedbackMessage msg_;
};

class Init_MoveArm_FeedbackMessage_goal_id
{
public:
  Init_MoveArm_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveArm_FeedbackMessage_feedback goal_id(::gamesmanros_interfaces::action::MoveArm_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveArm_FeedbackMessage_feedback(msg_);
  }

private:
  ::gamesmanros_interfaces::action::MoveArm_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::MoveArm_FeedbackMessage>()
{
  return gamesmanros_interfaces::action::builder::Init_MoveArm_FeedbackMessage_goal_id();
}

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__ACTION__DETAIL__MOVE_ARM__BUILDER_HPP_
