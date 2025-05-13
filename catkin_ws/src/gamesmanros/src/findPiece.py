#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import Point, PointStamped, Quaternion, Pose
from std_msgs.msg import Header

class PieceFinder:
    def __init__(self):
        rospy.init_node('piece_finder')
        self.name = rospy.get_param("~frame_id", "ar_marker_0")
        self.position = [0.0, 0.15, 0.25]  # Default fallback position

        # TF2
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfbr = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # Publisher for pose
        self.pose_pub = rospy.Publisher(f"/piece_position/{self.name}", Pose, queue_size=10)

        rospy.loginfo(f"[PieceFinder] Tracking {self.name}")
        self.find_piece_loop()

    def find_piece_loop(self):
        rate = rospy.Rate(10)  # 10 Hz loop

        while not rospy.is_shutdown():
            try:
                # Get transforms
                cam_to_base = self.tfBuffer.lookup_transform("joint1", "usb_cam", rospy.Time(0), rospy.Duration(1.0))
                tag_in_cam = self.tfBuffer.lookup_transform("usb_cam", self.name, rospy.Time(0), rospy.Duration(1.0))

                # Broadcast aligned frames
                self.tfbr.sendTransform(
                    (cam_to_base.transform.translation.x,
                     cam_to_base.transform.translation.y,
                     cam_to_base.transform.translation.z),
                    (0, 0, 0, 1),
                    rospy.Time.now(),
                    "aligned_usb_cam",
                    "joint1"
                )

                self.tfbr.sendTransform(
                    (tag_in_cam.transform.translation.x,
                     -tag_in_cam.transform.translation.y + 0.05,
                     0),
                    (0, 0, 0, 1),
                    rospy.Time.now(),
                    f"aligned_{self.name}",
                    "aligned_usb_cam"
                )

                # Get final transform in base frame
                aligned_tf = self.tfBuffer.lookup_transform("aligned_usb_cam", f"aligned_{self.name}", rospy.Time(0), rospy.Duration(1.0))

                point = Point(
                    x=aligned_tf.transform.translation.x,
                    y=aligned_tf.transform.translation.y,
                    z=aligned_tf.transform.translation.z
                )

                quat = Quaternion(
                    x=aligned_tf.transform.rotation.x,
                    y=aligned_tf.transform.rotation.y,
                    z=aligned_tf.transform.rotation.z,
                    w=aligned_tf.transform.rotation.w
                )

                # Convert to joint1 frame
                self.tf_listener.waitForTransform("joint1", "aligned_usb_cam", rospy.Time(), rospy.Duration(1.0))
                point_in_base = self.tf_listener.transformPoint(
                    "joint1",
                    PointStamped(header=Header(stamp=rospy.Time(), frame_id="aligned_usb_cam"), point=point)
                )

                self.position = [point_in_base.point.x, point_in_base.point.y, point_in_base.point.z]

                # Publish
                pose_msg = Pose()
                pose_msg.position = point_in_base.point
                pose_msg.orientation = quat
                self.pose_pub.publish(pose_msg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException,
                    tf.Exception) as e:
                #rospy.logwarn_throttle(5, f"[{self.name}] TF lookup failed: {e}")
                continue

            rate.sleep()

if __name__ == '__main__':
    try:
        PieceFinder()
    except rospy.ROSInterruptException:
        pass