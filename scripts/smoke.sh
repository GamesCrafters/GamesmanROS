#!/usr/bin/env bash
# Full-stack regression. Run before you push, and after you pull.
#
#   ./scripts/smoke.sh
#
# Builds, imports every node, runs the test suite, then plays real UWAPI moves
# through the whole pipeline against mock hardware. No robot required.
set -o pipefail   # NOT set -u: ROS setup.bash references unset vars

PASS=0; FAIL=0
ok()   { echo "  PASS  $1"; PASS=$((PASS+1)); }
bad()  { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }

cd /ws
source /opt/ros/humble/setup.bash

echo
echo "1. build"
if colcon build --symlink-install >/tmp/build.log 2>&1; then ok "colcon build"
else bad "colcon build"; tail -20 /tmp/build.log; exit 1; fi
source install/setup.bash

echo
echo "2. interfaces"
for t in "action ExecuteMove" "action MoveArm" "action MoveGripper" \
         "msg GameState" "msg BoardState" "msg PieceDetection"; do
  set -- $t
  python3 -c "from gamesmanros_interfaces.$1 import $2" 2>/dev/null \
    && ok "$1/$2" || bad "$1/$2"
done

echo
echo "3. node imports"
for m in centers robot_control game_manager vision_node \
         mock_hardware reach_sweep arm_controller scene \
         joint_state_bridge gripper_controller mycobot_topics joint_trajectory; do
  python3 -c "import gamesmanros.$m" 2>/dev/null && ok "$m" || bad "$m"
done

echo
echo "4. unit tests"
( cd src/gamesmanros && python3 -m pytest test/ -q >/tmp/pytest.log 2>&1 ) \
  && ok "pytest" || { bad "pytest"; tail -15 /tmp/pytest.log; }

echo
echo "5. integration — real UWAPI move through the full stack"
ros2 run gamesmanros mock_hardware --ros-args -p arm_delay:=0.0 \
    -p gripper_delay:=0.0 >/tmp/mock.log 2>&1 &
MOCK=$!
ros2 run gamesmanros robot_control >/tmp/rc.log 2>&1 &
RC=$!
sleep 6

OUT=$(ros2 action send_goal /execute_move \
  gamesmanros_interfaces/action/ExecuteMove \
  "{game_id: dodgem, move_string: M_4_5_x, current_position: \"1_----o---o----xx-\", new_position: \"2_-----o--o----xx-\"}" 2>&1)

echo "$OUT" | grep -q "success: true" && ok "ExecuteMove succeeded" || bad "ExecuteMove"
N=$(grep -c "arm #"  /tmp/mock.log 2>/dev/null); N=${N:-0}
G=$(grep -c "grip #" /tmp/mock.log 2>/dev/null); G=${G:-0}
[ "$N" = "6" ] && ok "6 arm goals"     || bad "arm goals: got $N expected 6"
[ "$G" = "3" ] && ok "3 gripper goals" || bad "gripper goals: got $G expected 3"

kill $MOCK $RC 2>/dev/null; wait 2>/dev/null

echo
echo "6. reachability sweep"
ros2 run gamesmanros mock_hardware --ros-args -p arm_delay:=0.0 \
    -p verbose:=false >/tmp/mock2.log 2>&1 &
MOCK2=$!
sleep 5
ros2 run gamesmanros reach_sweep --ros-args -p game:=dodgem -p mode:=cells \
    >/tmp/sweep.log 2>&1
grep -q "32/32 goals planned" /tmp/sweep.log \
  && ok "all 32 dodgem goals planned" \
  || bad "sweep: $(grep 'goals planned' /tmp/sweep.log || echo 'no result')"
kill $MOCK2 2>/dev/null; wait 2>/dev/null

echo
echo "────────────────────────────────"
echo "  $PASS passed, $FAIL failed"
echo "────────────────────────────────"
[ "$FAIL" -eq 0 ] || exit 1
