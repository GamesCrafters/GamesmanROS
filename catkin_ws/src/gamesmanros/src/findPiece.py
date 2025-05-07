#!/usr/bin/env python3

import rospy
import tf2_ros
import sys
import quaternions
import numpy as np
import tf

from geometry_msgs.msg import Point, PointStamped, Quaternion, QuaternionStamped, Pose
from std_msgs.msg import Header, String

class PieceFinder():
  def __init__(self, name):
    self.name = name
    #Create a publisher and a tf buffer, which is primed with a tf listener
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
    self.position = [0.0, 0.15, 0.25] #Some valid default


    self.tf_listener = tf.TransformListener()
    self.tfbr = tf.TransformBroadcaster()

    try:
      self.findPiece(self.name)
      rospy.spin()
    except rospy.ROSInterruptException:
      pass

  def get_position(self):
    return self.position

  #Define the method which contains the main functionality of the node.
  def findPiece(self, ar_frame):
    """
    Locates the position of a provided piece

    Inputs:
    - ar_frame: the tf frame of the AR tag on a given piece
    """

    ################################### YOUR CODE HERE ##############

    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    r = rospy.Rate(10) # 10hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
      try:
        cam_to_base_trans = self.tfBuffer.lookup_transform("joint1", "usb_cam", rospy.Time())
        ar_tag_trans = self.tfBuffer.lookup_transform("usb_cam", ar_frame, rospy.Time())

        #TODO MODIFY THIS OFFSET
        # Process trans to get your state error
        cam_trans_x = cam_to_base_trans.transform.translation.x
        cam_trans_y = cam_to_base_trans.transform.translation.y
        cam_trans_z = cam_to_base_trans.transform.translation.z

        input_x = ar_tag_trans.transform.translation.x
        input_y = ar_tag_trans.transform.translation.y 
        input_z = ar_tag_trans.transform.translation.z

        self.tfbr.sendTransform((cam_trans_x, cam_trans_y, cam_trans_z),
                          (0, 0, 0, 1),
                          rospy.Time.now(),
                          "aligned_usb_cam",
                          "joint1")
        self.tfbr.sendTransform((input_x, -input_y+0.05, 0),
                          (0, 0, 0, 1),
                          rospy.Time.now(),
                          "aligned_" + ar_frame,
                          "aligned_usb_cam")
        
        ar_tag_trans = self.tfBuffer.lookup_transform("aligned_usb_cam", "aligned_" + ar_frame, rospy.Time())

        #TODO MODIFY THIS OFFSET
        # Process trans to get your state error

        input_x = ar_tag_trans.transform.translation.x
        input_y = ar_tag_trans.transform.translation.y 
        input_z = ar_tag_trans.transform.translation.z


        piece = Point()
        piece.x = input_x
        piece.y = input_y
        piece.z = input_z

        orientation = Quaternion()
        orientation.x = ar_tag_trans.transform.rotation.x
        orientation.y = ar_tag_trans.transform.rotation.y
        orientation.z = ar_tag_trans.transform.rotation.z
        orientation.w = ar_tag_trans.transform.rotation.w


        self.tf_listener.waitForTransform("joint1", "aligned_usb_cam", rospy.Time(), rospy.Duration(10.0))
        center_in_base = self.tf_listener.transformPoint("joint1", PointStamped(header=Header(stamp=rospy.Time(), frame_id="aligned_usb_cam"), point=piece))

        x, y, z = center_in_base.point.x, center_in_base.point.y, center_in_base.point.z
        self.position = [x, y, z]
        #################################### end your code ###############
        
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        pass
        # Use our rate object to sleep until it is time to publish again



