#!/usr/bin/env python3

import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, -0.04, -0.0),
                        (0.70710678, -0.70710678, 0, -0),
                        rospy.Time.now(),
                        "usb_cam",
                        "joint6_flange")
        rate.sleep()