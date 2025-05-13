#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import threading

class ARTagListener:
    def __init__(self):
        self.positions = {}
        self.lock = threading.Lock()

        # Subscribe to all AR tag topics
        for i in range(18):
            tag_id = f"ar_marker_{i}"
            rospy.Subscriber(f"/piece_position/{tag_id}", Pose, self.make_callback(tag_id))

    def make_callback(self, tag_id):
        def callback(msg):
            with self.lock:
                self.positions[tag_id] = [msg.position.x, msg.position.y]
        return callback

    def get_pose(self, tag_id):
        with self.lock:
            return self.positions.get(tag_id, None)

    def get_all(self):
        with self.lock:
            return dict(self.positions)  # Shallow copy
