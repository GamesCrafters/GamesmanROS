#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to gamesmanros_interfaces__msg__GameState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GameState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub position: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub available_moves: Vec<std::string::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_robot_turn: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub game_over: bool,

}



impl Default for GameState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GameState::default())
  }
}

impl rosidl_runtime_rs::Message for GameState {
  type RmwMsg = super::msg::rmw::GameState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: msg.position.as_str().into(),
        available_moves: msg.available_moves
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        is_robot_turn: msg.is_robot_turn,
        game_over: msg.game_over,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: msg.position.as_str().into(),
        available_moves: msg.available_moves
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      is_robot_turn: msg.is_robot_turn,
      game_over: msg.game_over,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      position: msg.position.to_string(),
      available_moves: msg.available_moves
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      is_robot_turn: msg.is_robot_turn,
      game_over: msg.game_over,
    }
  }
}


