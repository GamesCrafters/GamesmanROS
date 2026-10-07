// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gamesmanros_interfaces:action/ExecuteMove.idl
// generated code does not contain a copyright notice

#ifndef GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__BUILDER_HPP_
#define GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gamesmanros_interfaces/action/detail/execute_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMove_Goal_new_position
{
public:
  explicit Init_ExecuteMove_Goal_new_position(::gamesmanros_interfaces::action::ExecuteMove_Goal & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::ExecuteMove_Goal new_position(::gamesmanros_interfaces::action::ExecuteMove_Goal::_new_position_type arg)
  {
    msg_.new_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_Goal msg_;
};

class Init_ExecuteMove_Goal_current_position
{
public:
  explicit Init_ExecuteMove_Goal_current_position(::gamesmanros_interfaces::action::ExecuteMove_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteMove_Goal_new_position current_position(::gamesmanros_interfaces::action::ExecuteMove_Goal::_current_position_type arg)
  {
    msg_.current_position = std::move(arg);
    return Init_ExecuteMove_Goal_new_position(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_Goal msg_;
};

class Init_ExecuteMove_Goal_move_string
{
public:
  explicit Init_ExecuteMove_Goal_move_string(::gamesmanros_interfaces::action::ExecuteMove_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteMove_Goal_current_position move_string(::gamesmanros_interfaces::action::ExecuteMove_Goal::_move_string_type arg)
  {
    msg_.move_string = std::move(arg);
    return Init_ExecuteMove_Goal_current_position(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_Goal msg_;
};

class Init_ExecuteMove_Goal_game_id
{
public:
  Init_ExecuteMove_Goal_game_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMove_Goal_move_string game_id(::gamesmanros_interfaces::action::ExecuteMove_Goal::_game_id_type arg)
  {
    msg_.game_id = std::move(arg);
    return Init_ExecuteMove_Goal_move_string(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::ExecuteMove_Goal>()
{
  return gamesmanros_interfaces::action::builder::Init_ExecuteMove_Goal_game_id();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMove_Result_success
{
public:
  Init_ExecuteMove_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::ExecuteMove_Result success(::gamesmanros_interfaces::action::ExecuteMove_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::ExecuteMove_Result>()
{
  return gamesmanros_interfaces::action::builder::Init_ExecuteMove_Result_success();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMove_Feedback_status
{
public:
  Init_ExecuteMove_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::ExecuteMove_Feedback status(::gamesmanros_interfaces::action::ExecuteMove_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::ExecuteMove_Feedback>()
{
  return gamesmanros_interfaces::action::builder::Init_ExecuteMove_Feedback_status();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMove_SendGoal_Request_goal
{
public:
  explicit Init_ExecuteMove_SendGoal_Request_goal(::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request goal(::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request msg_;
};

class Init_ExecuteMove_SendGoal_Request_goal_id
{
public:
  Init_ExecuteMove_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMove_SendGoal_Request_goal goal_id(::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteMove_SendGoal_Request_goal(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Request>()
{
  return gamesmanros_interfaces::action::builder::Init_ExecuteMove_SendGoal_Request_goal_id();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMove_SendGoal_Response_stamp
{
public:
  explicit Init_ExecuteMove_SendGoal_Response_stamp(::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response stamp(::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response msg_;
};

class Init_ExecuteMove_SendGoal_Response_accepted
{
public:
  Init_ExecuteMove_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMove_SendGoal_Response_stamp accepted(::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ExecuteMove_SendGoal_Response_stamp(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::ExecuteMove_SendGoal_Response>()
{
  return gamesmanros_interfaces::action::builder::Init_ExecuteMove_SendGoal_Response_accepted();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMove_GetResult_Request_goal_id
{
public:
  Init_ExecuteMove_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gamesmanros_interfaces::action::ExecuteMove_GetResult_Request goal_id(::gamesmanros_interfaces::action::ExecuteMove_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::ExecuteMove_GetResult_Request>()
{
  return gamesmanros_interfaces::action::builder::Init_ExecuteMove_GetResult_Request_goal_id();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMove_GetResult_Response_result
{
public:
  explicit Init_ExecuteMove_GetResult_Response_result(::gamesmanros_interfaces::action::ExecuteMove_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::ExecuteMove_GetResult_Response result(::gamesmanros_interfaces::action::ExecuteMove_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_GetResult_Response msg_;
};

class Init_ExecuteMove_GetResult_Response_status
{
public:
  Init_ExecuteMove_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMove_GetResult_Response_result status(::gamesmanros_interfaces::action::ExecuteMove_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ExecuteMove_GetResult_Response_result(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::ExecuteMove_GetResult_Response>()
{
  return gamesmanros_interfaces::action::builder::Init_ExecuteMove_GetResult_Response_status();
}

}  // namespace gamesmanros_interfaces


namespace gamesmanros_interfaces
{

namespace action
{

namespace builder
{

class Init_ExecuteMove_FeedbackMessage_feedback
{
public:
  explicit Init_ExecuteMove_FeedbackMessage_feedback(::gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage feedback(::gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage msg_;
};

class Init_ExecuteMove_FeedbackMessage_goal_id
{
public:
  Init_ExecuteMove_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMove_FeedbackMessage_feedback goal_id(::gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteMove_FeedbackMessage_feedback(msg_);
  }

private:
  ::gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::gamesmanros_interfaces::action::ExecuteMove_FeedbackMessage>()
{
  return gamesmanros_interfaces::action::builder::Init_ExecuteMove_FeedbackMessage_goal_id();
}

}  // namespace gamesmanros_interfaces

#endif  // GAMESMANROS_INTERFACES__ACTION__DETAIL__EXECUTE_MOVE__BUILDER_HPP_
