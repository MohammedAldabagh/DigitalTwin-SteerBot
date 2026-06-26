#!/usr/bin/env bash
set -e

fake_hardware=false
use_sim_time=false
mode="real"   # real | isaac | fake

echo "Passed arguments: $@"

# -r real, -i isaac, -f fake without simulator
while getopts "rif" opt; do
  case $opt in
    r)
      echo "Real hardware selected (-r)"
      mode="real"
      fake_hardware=false
      use_sim_time=false
      ;;
    i)
      echo "Simulation with Isaac (/clock) selected (-i)"
      mode="isaac"
      fake_hardware=true
      use_sim_time=true
      ;;
    f)
      echo "Fake simulation without simulator selected (-f)"
      mode="fake"
      fake_hardware=true
      use_sim_time=false   # IMPORTANT: no /clock required
      ;;
    *)
      echo "Invalid argument. Use -r (real), -i (isaac), or -f (fake without simulator)."
      exit 1
      ;;
  esac
done

echo "MODE=${mode}"
echo "fake_hardware=${fake_hardware}"
echo "use_sim_time=${use_sim_time}"
echo ""

start_terminal() {
  gnome-terminal -- bash -c "$1; exec bash" &
  terminal_pids+=($!)
}

kill_all() {
  echo ""
  echo "Shutting down all started ROS 2 terminals..."
  for pid in "${terminal_pids[@]}"; do
    kill -9 "$pid" 2>/dev/null || true
  done
  pkill -9 gnome-terminal || true
  echo "All terminals closed."
}

terminal_pids=()

echo "Starting controller bringup..."
start_terminal "cd ~/Steeringwheel-Workspace/ros2_ws; \
  source /opt/ros/humble/setup.bash; \
  source install/setup.bash; \
  ros2 launch piper_with_gripper_moveit controller_bringup_gripper.launch.py \
    fake_hardware:=${fake_hardware} use_sim_time:=${use_sim_time}"

sleep 6

echo "Starting MoveIt DT..."
start_terminal "cd ~/Steeringwheel-Workspace/ros2_ws; \
  source /opt/ros/humble/setup.bash; \
  source install/setup.bash; \
  ros2 launch piper_with_gripper_moveit moveit_dt_gripper.launch.py \
    use_sim_time:=${use_sim_time}"

sleep 3

echo ""
echo "========================================="
echo "Press [ENTER] to close **ALL terminals**."
echo "========================================="
read -r

kill_all
