#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__msg__GameState() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__msg__GameState__init(msg: *mut GameState) -> bool;
    fn gamesmanros_interfaces__msg__GameState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GameState>, size: usize) -> bool;
    fn gamesmanros_interfaces__msg__GameState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GameState>);
    fn gamesmanros_interfaces__msg__GameState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GameState>, out_seq: *mut rosidl_runtime_rs::Sequence<GameState>) -> bool;
}

// Corresponds to gamesmanros_interfaces__msg__GameState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GameState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub position: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub available_moves: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_robot_turn: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub game_over: bool,

}



impl Default for GameState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__msg__GameState__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__msg__GameState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GameState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__msg__GameState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__msg__GameState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__msg__GameState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GameState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GameState where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/msg/GameState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__msg__GameState() }
  }
}


