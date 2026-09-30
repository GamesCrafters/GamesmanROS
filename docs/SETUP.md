# GamesmanROS 2 — Development Setup

Target platform is **ROS 2 Humble on Ubuntu 22.04**. Nobody needs to install
that on their laptop — we use a Docker image instead, so everyone builds against
an identical environment regardless of host OS.

Everything below has been verified on an Apple Silicon Mac (M3, macOS 14.6).

---

## The decision

**Docker is the standard environment.** One image, one build command, same result
on macOS / Linux / Windows. If it builds in the container, it builds for everyone.

Base image is **`ros:humble`**, not `osrf/ros:humble-desktop`. The `osrf` desktop
image is published for amd64 only, so on an Apple Silicon Mac it runs under
emulation and is painfully slow. `ros:humble` is multi-arch and runs natively on
arm64 — which also matches the Raspberry Pi's architecture.

Docker has two limits worth knowing up front, both only affecting macOS:

| Need | Works in Docker? | What to do instead |
|---|---|---|
| Build, run nodes, unit tests | Yes, everywhere | — |
| RViz / MoveIt GUI | Linux yes; macOS needs XQuartz | See *Track A* below |
| USB camera | Linux yes; **macOS no** | Run the camera code natively — see *Track B* |
| DDS to the robot over WiFi | Linux yes; macOS unreliable | SSH into the Pi and run nodes there |

Only Track A needs a GUI and only Track B needs a camera, so most work is
unaffected. **If anyone on the team has a native Linux machine, they should take
Track A** — it removes the only genuinely awkward setup on the project.

---

## Quick start

```bash
git clone https://github.com/egale800/GamesmanROS.git
cd GamesmanROS
git checkout meow

# Build the image (first time only, ~5 min)
docker build -f docker/Dockerfile -t gamesmanros:humble docker/

# Open a shell in the container
docker compose -f docker/docker-compose.yml run --rm dev
```

Then, **inside the container**:

```bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means Python edits on your host take effect immediately —
you do not need to rebuild after every change (you do after editing
`.action` / `.msg` files).

---

## Verify your setup

Run this inside the container. All four checks should pass before the next meeting.

```bash
# 1. Both packages build
colcon build --symlink-install

# 2. Generated interfaces are importable
source install/setup.bash
python3 -c "from gamesmanros_interfaces.action import ExecuteMove; print('interfaces OK')"

# 3. A node starts and exposes its action server
ros2 run gamesmanros robot_control &
sleep 5 && ros2 action list        # expect /execute_move, /move_arm, /move_gripper
kill %1

# 4. OpenCV has the ArUco API we need
python3 -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco,'ArucoDetector'))"
```

Expected from #4: version **>= 4.7** and `True`.

---

## Track-specific setup

### Track A — MoveIt 2 / RViz

**You do not need the robot.** MoveIt runs against mock hardware, so you can build
the config and validate OMPL planning in RViz with nothing plugged in.

On Linux, RViz works directly. On macOS you need XQuartz:

```bash
brew install --cask xquartz
open -a XQuartz
# XQuartz → Settings → Security → check "Allow connections from network clients"
# then log out and back in
xhost + 127.0.0.1
```

Then launch the container as normal — `DISPLAY` is already wired up in the
compose file. Expect software rendering; it is usable but not fast.

### Track B — Vision

Docker Desktop on macOS **cannot see USB cameras**, so do camera work natively on
your host:

```bash
pip3 install "numpy<2" "opencv-contrib-python>=4.7"
```

The HSV calibration tooling needs no ROS at all — it is plain OpenCV. Write and
tune it on the host, then move the finished bounds into the ROS node and test that
part in the container.

### Track C — Game data and tests

No special setup. The Docker quick start is all you need. `centers.py` has no ROS
dependency, so much of this work is testable as plain Python.

---

## Known issues

**`joint_state_bridge` and `gripper_controller` do not currently import.** Both
depend on `mycobot_interfaces` (`MycobotAngles`, `MycobotGripperStatus`), which is
not vendored in this repo and is not a stock ROS 2 package — it comes from
Elephant Robotics' `mycobot_ros2`. We need to either vendor it or define our own
equivalent messages. **This blocks any hardware bring-up and is unassigned.**

**NumPy is pinned below 2.0** in the Dockerfile. `tf_transformations` (used by
`arm_controller`) calls `np.maximum_sctype`, which NumPy 2.0 removed. Do not
unpin it without testing `arm_controller` imports.

**OpenCV must come from pip, not apt.** Ubuntu 22.04 ships `python3-opencv` 4.5.4,
which predates `cv2.aruco.ArucoDetector`. `vision_node.py` will not run against it.

---

## Troubleshooting

**`colcon: command not found`** — you are on the host, not in the container.

**Changes to `.action` / `.msg` not taking effect** — interface changes need a
real rebuild: `rm -rf build install && colcon build --symlink-install`.

**Nodes cannot see each other** — check `ROS_DOMAIN_ID` matches across all
terminals. Set it in your shell or in the compose file.

**RViz shows a blank window on macOS** — XQuartz is not accepting connections.
Re-run `xhost + 127.0.0.1` and confirm the "network clients" setting above.

**Image build fails partway** — rerun it. Occasional apt mirror timeouts are
normal; Docker resumes from the last good layer.
