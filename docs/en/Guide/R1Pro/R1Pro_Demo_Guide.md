# R1 Pro Demo Guide

## Preparation Before Use
Before you start, please ensure that:

1. Two arms are correctly installed. At this time, the elbow bend (the black rubber) should face directly forward; the gripper points vertically downward towards the ground, and the gripper cable faces outward.
2. The torso is in the standing pose.
3. No personnel and obstacles within a radius of 1.5 meters. (As shown in the figure below.)

![R1Pro_6_2ndcheck](./assets/unbox/R1Pro_6_2ndcheck.png)

## Script Execution

**Only after the above preparation work is completed can the robot be used to start the demo.**

<span style="color:red">**Note: If any errors occur, please contact us promptly for technical support. If there are no abnormalities, press `Ctrl + C` to close.**</span>

1. Kill all TMUX running and close all ROS programs.
   ```Bash
   sudo tmux kill-server
   tmux kill-server
   pkill -9 ros
   ```

2. Enter TMUX.
   ```Bash
   tmux
   ```

3. Start FDCAN Communication
   ```Bash
   sudo ip link set dev can0 type can bitrate 1000000 dbitrate 5000000 fd on
   sudo ip link set up can0  
   # You may need to enter the password again: nvidia
   # If the error message "RTNETLINK answers: Device or resource busy" appears, it usually means that the device you are trying to configure (such as a network interface or CAN transceiver) has already been configured and is currently in operation.
   ```

4. Start roscore.f
   ```Bash
   roscore
   ```

5. Press `Ctrl + B` then `C` to create a new terminal, and execute launch file.  
   ```Bash
   source {your_download_path}/install/setup.bash
   roslaunch HDAS r1pro.launch
   ```

6. Press `Ctrl + B` then `C` to create a new terminal. Then, start self-check.
   ```Bash
   source {your_download_path}/install/setup.bash
   rosrun HDAS check_node 
   # Press 1 (0 indicates self-check with robotic arm installed; 1 indicates self-check without robot arm)
   ```

7. Press `Ctrl + B` then `C` to create a new terminal. Then, start chassis control.
   ```Bash
   source {your_download_path}/install/setup.bash
   roslaunch mobiman r1_pro_chassis_control.launch
   ```

8. Press `Ctrl + B` then `C` to create a new terminal. Then, start arm and torso control.
   ```Bash
   source {your_download_path}/install/setup.bash
   roslaunch mobiman r1_pro_jointTrackerdemo_pid.launch
   ```

9. Press `Ctrl + B` then `C` to create a new terminal. Then, start the demo script.
   ```Bash
   source {your_download_path}/install/setup.bash
   cd {your_download_path}/install/share/mobiman/script
   python3 r1pro_test_open_box.py
   ```

10. Enter the number corresponding to each demo action and press Enter. Then, R1 Pro will start to perform.

    ![R1Pro_7_demo](./assets/unbox/R1Pro_7_demo.png)

11. When you have completed all actions, press `Q` to quit. R1 Pro will be back to the zero-point pose.