# Overview
The piper_ros package provides the complete ROS2 (Humble) integration for the Piper robot arm. It contains:
- Hardware bringup (CAN-Bus tools + control nodes)
- URDF/Xacro robot descriptions
- MoveIt2 configurations
- Simulation (Gazebo & MuJoCo)
- Helper scripts
This repository allows you to use the Piper robot in real hardware mode or simulation mode with a unified interface.

# Paketstruktur
## piper — Hardware Control Nodes
ROS2 nodes to control real Piper hardware over CAN-Bus.

Enthält u.a.:
- piper_ctrl_single_node.py — sendet Commands an einen echten Roboter
- piper_ctrl_single_node_new.py — aktualisierte Version
- piper_read_slave_joint.py — liest echte Gelenkwerte

Launchfiles:
```
start_single_piper.launch.py
start_single_piper_rviz.launch.py
start_two_piper.launch.py
```
Zusätzlich:
```can_activate.sh, can_config.sh, find_all_can_port.sh ``` wichtige Tools zur CAN-Bus-Konfiguration.

## piper_description — URDF & Meshes
Hier liegt das komplette geometrische Modell:
- URDF/Xacro: Varianten mit / ohne Gripper / mit Teach Pendant
- Meshes (STL)
- RViz configs
- Mujoco Models
Wird von MoveIt2, Gazebo und MuJoCo genutzt.

## piper_moveit — MoveIt2 Integration
MoveIt-Configs für:
- Piper ohne Gripper
- Piper mit Gripper

Wichtige Launchfiles:
```
controller_bringup.launch.py — startet ros2_control
moveit_dt.launch.py — startet MoveIt2 mit Demo/Teleop
move_group.launch.py
moveit_rviz.launch.py
rsp.launch.py — Robot State Publisher
spawn_controllers.launch.py
```

## piper_sim — Simulation
Unterstützt:
 - Gazebo (Ignition)
```
piper_no_gripper_gazebo.launch.py
piper_gazebo.launch.py
```

- MuJoCo
```
piper_mujoco_ctrl.py
piper_no_gripper_mujoco_ctrl.py
```

## piper_msgs — Custom ROS2 Messages
Eigene Nachrichten:
- PiperStatusMsg.msg — Status, Fehler etc.
- PosCmd.msg — Positionsbefehle

Services:
- Enable.srv — Roboter aktivieren/deaktivieren

# How to Start the System
## Workspace builden
```
cd ~/Steerbot/ros2_ws
colcon build
source install/setup.bash
```

## Start Real Hardware

### 1. CAN-Bus initialisieren
Beispiel: ```./can_activate.sh```

### 2. Piper starten
ros2 launch piper start_single_piper.launch.py

## Start Simulation
Gazebo
```ros2 launch piper_gazebo piper_no_gripper_gazebo.launch.py```
MuJoCo
```ros2 run piper_mujoco piper_no_gripper_mujoco_ctrl.py```

## Start MoveIt2
Fall - Fake Hardware + MoveIt2
```
ros2 launch piper_no_gripper_moveit controller_bringup.launch.py fake_hardware:=true
ros2 launch piper_no_gripper_moveit moveit_dt.launch.py use_sim_time:=false
```