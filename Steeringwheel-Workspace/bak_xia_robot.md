bash find_all_can_port.sh
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-3:1.0

Not this here
bash can_activate.sh can0 1000000 "3-3:1.0"
Activate can:
(2) Activate the CAN device. Assuming the USB port value is 3-1.4:1.0, run:

```bash
bash can_activate.sh can_piper 1000000 "3-3:1.0"

-------------------START-----------------------
Both ethtool and can-utils are installed.
Detected USB hardware address parameter: 3-3:1.0
Found the interface corresponding to USB hardware address 3-3:1.0: can0.
Interface can0 is already activated with a bitrate of 1000000.
Rename interface can0 to can_piper.
The interface has been renamed to can_piper and reactivated.
-------------------OVER------------------------
```

activate can:

bash can_activate.sh can_piper 1000000 "3-3:1.0"
-------------------START-----------------------
Both ethtool and can-utils are installed.
Detected USB hardware address parameter: 3-3:1.0
Found the interface corresponding to USB hardware address 3-3:1.0: can_piper.
Interface can_piper is already activated with a bitrate of 1000000.
The interface name is already can_piper.
-------------------OVER------------------------



## start the node
Example:
```bash
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=false -p gripper_val_mutiple:=2
```
Our version:
```bash
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can_piper -p auto_enable:=false -p gripper_exist:=false
```

Parameter:
- auto_enable: Whether to automatically enable the system. If True, the system will automatically enable upon starting the program. Set this to False if you want to manually control the enable state. If the program is interrupted and then restarted, the robotic arm will maintain the state it had during the last run.
 - If the arm was enabled, it will remain enabled after restarting.
 - If the arm was disabled, it will remain disabled after restarting.
- gripper_exist: Whether there is an end gripper. If True, it means there is an end gripper and the gripper control will be turned on.
- rviz_ctrl_flag: Whether to use RViz to send joint angle messages. If True, the system will receive joint angle messages sent by rViz.
- gripper_val_mutiple: Set the gripper control multiplier.  

Notice: Since the range of joint7 in RViz is [0, 0.04], while the actual gripper travel is 0.08m, the gripper needs to be set to twice the value in RViz to control the real gripper correctly.



ros2 run piper piper_single_ctrl --ros-args -p can_port:=can_piper -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2


ros2 run piper piper_single_ctrl --ros-args -p can_port:=can_piper -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2




bash can_activate.sh can0 1000000 "3-11:1.0"






bash can_activate.sh can0 1000000
-------------------START-----------------------
Both ethtool and can-utils are installed.
Expected to configure a single CAN module, detected interface can0 with corresponding USB address 3-3:1.0.
Interface can0 is not activated or bitrate is not set.
Interface can0 has been reset to bitrate 1000000 and activated.
-------------------OVER------------------------









