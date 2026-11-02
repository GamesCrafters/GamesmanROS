#!/usr/bin/env python3
"""
Real-arm command path: mycobot_topics and joint_trajectory.

These run with no arm attached: pymycobot is replaced by a recording fake, and
the executor's publisher is replaced by a list. What they pin down is everything
that is easy to get silently wrong on hardware — units (radians vs degrees),
joint order, the gripper's 0/1 polarity, point timing, and that a missing arm
cannot hang a trajectory forever.

    cd src/gamesmanros && python3 -m pytest test/test_hardware_path.py -v
"""

import math
import os
import re
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mycobot_interfaces')
pytest.importorskip('control_msgs')

from gamesmanros import mycobot_topics as drv                       # noqa: E402
from gamesmanros import joint_trajectory as tex                  # noqa: E402

MSG_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'mycobot_interfaces', 'msg')


@pytest.fixture(scope='module', autouse=True)
def ros():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Message definitions
# ---------------------------------------------------------------------------

def test_msg_field_names_are_valid_ros2_identifiers():
    """rosidl rejects capitals — the ROS 1 `Status` field cannot be copied over."""
    valid = re.compile(r'^(?!.*__)(?!.*_$)[a-z]([a-z0-9_]?[a-z0-9]+)*$')
    for fn in os.listdir(MSG_DIR):
        for line in open(os.path.join(MSG_DIR, fn)):
            line = line.split('#')[0].strip()
            if line:
                name = line.split()[1]
                assert valid.match(name), f'{fn}: invalid field name {name!r}'


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def test_joint_order_matches_the_urdf_and_the_old_bridge():
    assert tex.JOINT_NAMES == [
        'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
        'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']


def test_positions_are_reordered_into_driver_order():
    shuffled = list(reversed(tex.JOINT_NAMES))
    assert tex.order_positions(shuffled, [6, 5, 4, 3, 2, 1]) == [1, 2, 3, 4, 5, 6]


def test_wrong_joints_are_rejected():
    with pytest.raises(ValueError):
        tex.order_positions(['a', 'b', 'c', 'd', 'e', 'f'], [0] * 6)


def test_radians_become_degrees():
    assert tex.to_degrees([math.pi, math.pi / 2, 0, -math.pi, 1, 0]) == \
        pytest.approx([180, 90, 0, -180, 57.29578, 0], abs=1e-4)


def test_arrival_check():
    assert tex.arrived([10, 20, 30, 40, 50, 60], [11, 19, 30, 40, 50, 60], 3.0)
    assert not tex.arrived([10, 20, 30, 40, 50, 60], [10, 20, 30, 40, 50, 70], 3.0)


def test_gripper_polarity_matches_ros1():
    """ROS 1: Status True -> set_gripper_state(0, 80) opens; False -> (1, 80) closes."""
    assert drv.gripper_flag(True) == 0
    assert drv.gripper_flag(False) == 1


def test_speed_is_clamped_to_the_arms_range():
    assert (drv.clamp_speed(0), drv.clamp_speed(40), drv.clamp_speed(500)) == (1, 40, 100)


# ---------------------------------------------------------------------------
# mycobot_topics, with a fake pymycobot
# ---------------------------------------------------------------------------

class FakeMyCobot:
    def __init__(self, port, baud):
        self.port, self.baud = port, baud
        self.sent, self.gripper, self.angles = [], [], [1, 2, 3, 4, 5, 6]

    def get_angles(self):
        return self.angles

    def send_angles(self, angles, speed):
        self.sent.append((list(angles), speed))

    def set_gripper_state(self, flag, speed):
        self.gripper.append((flag, speed))


@pytest.fixture
def driver(monkeypatch):
    pkg = types.ModuleType('pymycobot')
    mod = types.ModuleType('pymycobot.mycobot')
    mod.MyCobot = FakeMyCobot
    monkeypatch.setitem(sys.modules, 'pymycobot', pkg)
    monkeypatch.setitem(sys.modules, 'pymycobot.mycobot', mod)
    node = drv.MycobotTopics()
    node.published = []
    node._pub = types.SimpleNamespace(publish=node.published.append)
    yield node
    node.destroy_node()


def test_driver_forwards_goals_to_the_arm(driver):
    from mycobot_interfaces.msg import MycobotSetAngles
    m = MycobotSetAngles()
    m.joint_1, m.joint_2, m.joint_3, m.joint_4, m.joint_5, m.joint_6 = 10, 20, 30, 40, 50, 60
    m.speed = 40
    driver._on_goal(m)
    assert driver._mc.sent == [([10, 20, 30, 40, 50, 60], 40)]


def test_driver_opens_and_closes_the_gripper(driver):
    from mycobot_interfaces.msg import MycobotGripperStatus
    m = MycobotGripperStatus()
    m.status = True
    driver._on_gripper(m)
    m.status = False
    driver._on_gripper(m)
    assert driver._mc.gripper == [(0, 80), (1, 80)]


def test_driver_publishes_real_angles_and_skips_bad_reads(driver):
    driver._publish_angles()
    assert len(driver.published) == 1
    assert driver.published[0].joint_3 == 3
    driver._mc.angles = []                 # serial hiccup: nothing published
    driver._publish_angles()
    driver._mc.angles = None
    driver._publish_angles()
    assert len(driver.published) == 1


# ---------------------------------------------------------------------------
# joint_trajectory
# ---------------------------------------------------------------------------

def make_point(positions, t):
    return types.SimpleNamespace(
        positions=positions,
        time_from_start=types.SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9)))


def make_goal(points, names=None):
    traj = types.SimpleNamespace(joint_names=names or list(tex.JOINT_NAMES), points=points)
    handle = types.SimpleNamespace(request=types.SimpleNamespace(trajectory=traj),
                                   succeeded=False)
    handle.succeed = lambda: setattr(handle, 'succeeded', True)
    return handle


@pytest.fixture
def executor():
    node = tex.JointTrajectoryActionServer()
    node.sent = []
    node._pub = types.SimpleNamespace(publish=node.sent.append)
    node._wait_for_arrival = lambda target: True
    yield node
    node.destroy_node()


def test_executor_streams_every_point_in_degrees_at_the_set_speed(executor):
    pts = [make_point([0.0] * 6, 0.0),
           make_point([math.pi / 2, 0, 0, 0, 0, -math.pi / 2], 0.01)]
    goal = make_goal(pts)
    result = executor._execute(goal)
    assert goal.succeeded
    assert result.error_code == result.SUCCESSFUL
    assert len(executor.sent) == 2
    last = executor.sent[-1]
    assert (last.joint_1, last.joint_6) == pytest.approx((90.0, -90.0))
    assert last.speed == 40


def test_executor_honours_planned_timing(executor):
    import time
    goal = make_goal([make_point([0.0] * 6, 0.0), make_point([0.1] * 6, 0.3)])
    start = time.monotonic()
    executor._execute(goal)
    assert time.monotonic() - start >= 0.28      # waited for time_from_start


def test_executor_still_finishes_if_the_arm_never_reports_arrival(executor):
    """No driver running must not hang the planner forever (the wait is bounded)."""
    executor._wait_for_arrival = lambda target: False
    goal = make_goal([make_point([0.0] * 6, 0.0)])
    executor._execute(goal)
    assert goal.succeeded                        # warns, but does not hang or crash


def test_executor_rejects_empty_and_misnamed_trajectories(executor):
    from rclpy.action import GoalResponse
    empty = types.SimpleNamespace(trajectory=types.SimpleNamespace(joint_names=[], points=[]))
    assert executor._on_goal(empty) == GoalResponse.REJECT
    bad = types.SimpleNamespace(trajectory=types.SimpleNamespace(
        joint_names=['a'] * 6, points=[make_point([0.0] * 6, 0.0)]))
    assert executor._on_goal(bad) == GoalResponse.REJECT
    good = types.SimpleNamespace(trajectory=types.SimpleNamespace(
        joint_names=list(tex.JOINT_NAMES), points=[make_point([0.0] * 6, 0.0)]))
    assert executor._on_goal(good) == GoalResponse.ACCEPT
