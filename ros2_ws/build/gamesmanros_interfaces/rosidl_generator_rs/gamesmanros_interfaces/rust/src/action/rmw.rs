
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_Goal() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveArm_Goal__init(msg: *mut MoveArm_Goal) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Goal>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Goal>);
    fn gamesmanros_interfaces__action__MoveArm_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveArm_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Goal>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveArm_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f64,

}



impl Default for MoveArm_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveArm_Goal__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveArm_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveArm_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveArm_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveArm_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveArm_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_Goal() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_Result() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveArm_Result__init(msg: *mut MoveArm_Result) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Result>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Result>);
    fn gamesmanros_interfaces__action__MoveArm_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveArm_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Result>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveArm_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for MoveArm_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveArm_Result__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveArm_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveArm_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveArm_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveArm_Result where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveArm_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_Result() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveArm_Feedback__init(msg: *mut MoveArm_Feedback) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Feedback>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Feedback>);
    fn gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveArm_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveArm_Feedback>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveArm_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: rosidl_runtime_rs::String,

}



impl Default for MoveArm_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveArm_Feedback__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveArm_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveArm_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveArm_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveArm_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveArm_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_Feedback() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveArm_FeedbackMessage__init(msg: *mut MoveArm_FeedbackMessage) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_FeedbackMessage>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_FeedbackMessage>);
    fn gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveArm_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveArm_FeedbackMessage>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveArm_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::MoveArm_Feedback,

}



impl Default for MoveArm_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveArm_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveArm_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveArm_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveArm_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveArm_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveArm_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_FeedbackMessage() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_Goal() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveGripper_Goal__init(msg: *mut MoveGripper_Goal) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Goal>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Goal>);
    fn gamesmanros_interfaces__action__MoveGripper_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveGripper_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Goal>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveGripper_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub open: bool,

}



impl Default for MoveGripper_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveGripper_Goal__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveGripper_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveGripper_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveGripper_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveGripper_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveGripper_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_Goal() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_Result() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveGripper_Result__init(msg: *mut MoveGripper_Result) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Result>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Result>);
    fn gamesmanros_interfaces__action__MoveGripper_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveGripper_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Result>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveGripper_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for MoveGripper_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveGripper_Result__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveGripper_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveGripper_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveGripper_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveGripper_Result where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveGripper_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_Result() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveGripper_Feedback__init(msg: *mut MoveGripper_Feedback) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Feedback>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Feedback>);
    fn gamesmanros_interfaces__action__MoveGripper_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveGripper_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_Feedback>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveGripper_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: rosidl_runtime_rs::String,

}



impl Default for MoveGripper_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveGripper_Feedback__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveGripper_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveGripper_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveGripper_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveGripper_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveGripper_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_Feedback() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__init(msg: *mut MoveGripper_FeedbackMessage) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_FeedbackMessage>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_FeedbackMessage>);
    fn gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveGripper_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_FeedbackMessage>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveGripper_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::MoveGripper_Feedback,

}



impl Default for MoveGripper_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveGripper_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveGripper_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveGripper_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveGripper_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_FeedbackMessage() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_Goal() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__ExecuteMove_Goal__init(msg: *mut ExecuteMove_Goal) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Goal>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Goal>);
    fn gamesmanros_interfaces__action__ExecuteMove_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteMove_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Goal>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteMove_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub game_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub move_string: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_position: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub new_position: rosidl_runtime_rs::String,

}



impl Default for ExecuteMove_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__ExecuteMove_Goal__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__ExecuteMove_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteMove_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteMove_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteMove_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/ExecuteMove_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_Goal() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_Result() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__ExecuteMove_Result__init(msg: *mut ExecuteMove_Result) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Result>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Result>);
    fn gamesmanros_interfaces__action__ExecuteMove_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteMove_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Result>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteMove_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for ExecuteMove_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__ExecuteMove_Result__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__ExecuteMove_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteMove_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteMove_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteMove_Result where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/ExecuteMove_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_Result() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__ExecuteMove_Feedback__init(msg: *mut ExecuteMove_Feedback) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Feedback>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Feedback>);
    fn gamesmanros_interfaces__action__ExecuteMove_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteMove_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_Feedback>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteMove_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: rosidl_runtime_rs::String,

}



impl Default for ExecuteMove_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__ExecuteMove_Feedback__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__ExecuteMove_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteMove_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteMove_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteMove_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/ExecuteMove_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_Feedback() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__init(msg: *mut ExecuteMove_FeedbackMessage) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_FeedbackMessage>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_FeedbackMessage>);
    fn gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteMove_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_FeedbackMessage>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteMove_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::ExecuteMove_Feedback,

}



impl Default for ExecuteMove_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteMove_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteMove_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteMove_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/ExecuteMove_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_FeedbackMessage() }
  }
}




#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveArm_SendGoal_Request__init(msg: *mut MoveArm_SendGoal_Request) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_SendGoal_Request>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_SendGoal_Request>);
    fn gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveArm_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveArm_SendGoal_Request>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveArm_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::MoveArm_Goal,

}



impl Default for MoveArm_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveArm_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveArm_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveArm_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveArm_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveArm_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveArm_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_SendGoal_Request() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveArm_SendGoal_Response__init(msg: *mut MoveArm_SendGoal_Response) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_SendGoal_Response>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_SendGoal_Response>);
    fn gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveArm_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveArm_SendGoal_Response>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveArm_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for MoveArm_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveArm_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveArm_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveArm_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveArm_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveArm_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveArm_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_SendGoal_Response() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveArm_GetResult_Request__init(msg: *mut MoveArm_GetResult_Request) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_GetResult_Request>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_GetResult_Request>);
    fn gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveArm_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveArm_GetResult_Request>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveArm_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for MoveArm_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveArm_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveArm_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveArm_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveArm_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveArm_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveArm_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_GetResult_Request() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveArm_GetResult_Response__init(msg: *mut MoveArm_GetResult_Response) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_GetResult_Response>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveArm_GetResult_Response>);
    fn gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveArm_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveArm_GetResult_Response>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveArm_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::MoveArm_Result,

}



impl Default for MoveArm_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveArm_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveArm_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveArm_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveArm_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveArm_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveArm_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveArm_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveArm_GetResult_Response() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__init(msg: *mut MoveGripper_SendGoal_Request) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_SendGoal_Request>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_SendGoal_Request>);
    fn gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveGripper_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_SendGoal_Request>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveGripper_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::MoveGripper_Goal,

}



impl Default for MoveGripper_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveGripper_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveGripper_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveGripper_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveGripper_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_SendGoal_Request() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__init(msg: *mut MoveGripper_SendGoal_Response) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_SendGoal_Response>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_SendGoal_Response>);
    fn gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveGripper_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_SendGoal_Response>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveGripper_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for MoveGripper_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveGripper_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveGripper_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveGripper_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveGripper_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_SendGoal_Response() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveGripper_GetResult_Request__init(msg: *mut MoveGripper_GetResult_Request) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_GetResult_Request>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_GetResult_Request>);
    fn gamesmanros_interfaces__action__MoveGripper_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveGripper_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_GetResult_Request>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveGripper_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for MoveGripper_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveGripper_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveGripper_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveGripper_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveGripper_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveGripper_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveGripper_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_GetResult_Request() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__MoveGripper_GetResult_Response__init(msg: *mut MoveGripper_GetResult_Response) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_GetResult_Response>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__MoveGripper_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_GetResult_Response>);
    fn gamesmanros_interfaces__action__MoveGripper_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveGripper_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveGripper_GetResult_Response>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveGripper_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::MoveGripper_Result,

}



impl Default for MoveGripper_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__MoveGripper_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__MoveGripper_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveGripper_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__MoveGripper_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveGripper_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveGripper_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/MoveGripper_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__MoveGripper_GetResult_Response() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__init(msg: *mut ExecuteMove_SendGoal_Request) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_SendGoal_Request>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_SendGoal_Request>);
    fn gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteMove_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_SendGoal_Request>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteMove_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::ExecuteMove_Goal,

}



impl Default for ExecuteMove_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteMove_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteMove_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteMove_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/ExecuteMove_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Request() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__init(msg: *mut ExecuteMove_SendGoal_Response) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_SendGoal_Response>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_SendGoal_Response>);
    fn gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteMove_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_SendGoal_Response>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteMove_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for ExecuteMove_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteMove_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteMove_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteMove_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/ExecuteMove_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_SendGoal_Response() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__init(msg: *mut ExecuteMove_GetResult_Request) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_GetResult_Request>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_GetResult_Request>);
    fn gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteMove_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_GetResult_Request>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteMove_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for ExecuteMove_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteMove_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteMove_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteMove_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/ExecuteMove_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_GetResult_Request() }
  }
}


#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "gamesmanros_interfaces__rosidl_generator_c")]
extern "C" {
    fn gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__init(msg: *mut ExecuteMove_GetResult_Response) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_GetResult_Response>, size: usize) -> bool;
    fn gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_GetResult_Response>);
    fn gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteMove_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteMove_GetResult_Response>) -> bool;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteMove_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::ExecuteMove_Result,

}



impl Default for ExecuteMove_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteMove_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { gamesmanros_interfaces__action__ExecuteMove_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteMove_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteMove_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "gamesmanros_interfaces/action/ExecuteMove_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_GetResult_Response() }
  }
}






#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__MoveArm_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct MoveArm_SendGoal;

impl rosidl_runtime_rs::Service for MoveArm_SendGoal {
    type Request = MoveArm_SendGoal_Request;
    type Response = MoveArm_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__MoveArm_SendGoal() }
    }
}




#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__MoveArm_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to gamesmanros_interfaces__action__MoveArm_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct MoveArm_GetResult;

impl rosidl_runtime_rs::Service for MoveArm_GetResult {
    type Request = MoveArm_GetResult_Request;
    type Response = MoveArm_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__MoveArm_GetResult() }
    }
}




#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__MoveGripper_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct MoveGripper_SendGoal;

impl rosidl_runtime_rs::Service for MoveGripper_SendGoal {
    type Request = MoveGripper_SendGoal_Request;
    type Response = MoveGripper_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__MoveGripper_SendGoal() }
    }
}




#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__MoveGripper_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to gamesmanros_interfaces__action__MoveGripper_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct MoveGripper_GetResult;

impl rosidl_runtime_rs::Service for MoveGripper_GetResult {
    type Request = MoveGripper_GetResult_Request;
    type Response = MoveGripper_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__MoveGripper_GetResult() }
    }
}




#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ExecuteMove_SendGoal;

impl rosidl_runtime_rs::Service for ExecuteMove_SendGoal {
    type Request = ExecuteMove_SendGoal_Request;
    type Response = ExecuteMove_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_SendGoal() }
    }
}




#[link(name = "gamesmanros_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to gamesmanros_interfaces__action__ExecuteMove_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ExecuteMove_GetResult;

impl rosidl_runtime_rs::Service for ExecuteMove_GetResult {
    type Request = ExecuteMove_GetResult_Request;
    type Response = ExecuteMove_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__gamesmanros_interfaces__action__ExecuteMove_GetResult() }
    }
}


