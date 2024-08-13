# Software Overview
## Software Dependency

1. [Ubuntu](https://en.wikipedia.org/wiki/Ubuntu) 20.04 LTS
2. ROS Noetic

## Installation

This SDK does not require recompilation. Please refer to the contents below.

## First Move

### Chassis - Spins in place

```Bash
#Switch SWB to the bottom position and switch SWC to the middle position to enter upper computer chassis control mode.
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1" 
 
  ### Send This Command to make R1 Spin with Controller 
  
  
  ###Two Way to make it stop
  rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
  
  ### Change SWB / SWC to quit Upper Computer Control mode of chassis
```

### Torso - Squat & Stand

Controller teleoperation primarily involves using a Python script to map messages from the topic `/controller` to the topic `/target_torso_speed`. (`/controller` is sent by Chassis Control Unit to Control Unit via CAN.) Therefore, the torso is actually controlled by the topic `/target_torso_speed`. 

Please follow the following commands:

1. **Quit Torso Control Mode:** Switch SWB and SWC to positions other than the bottom to ensure that the topic `/controller` is NOT published.
2.  **Move the torso:**

```Bash
##############
rostopic pub /target_torso_speed geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0   
  z: 0.0
angular:
  x: 0.0
  y: 0.1
  z: 0.0"
  
  ###linear.y > 0 means raise the torso 
  ###linear.y < 0 means down the torso
  ###angular.x > 0 means increase the pitch and <0 means decrease the pitch
  ###angular.y > 0 means anti-clockwise the wrist and < 0 means clockwise the wrist
```

3. **Stop the torso:**

```Bash
##############
rostopic pub /target_torso_speed geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0   
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
  
  ###linear.y > 0 means raise the torso 
  ###linear.y < 0 means down the torso
  ###angular.x > 0 means increase the pitch and <0 means decrease the pitch
  ###angular.y > 0 means anti-clockwise the wrist and < 0 means clockwise the wrist
```

### Arm - Wave & Salute

Since the arm control is complex and cannot be managed with just two joysticks, we have stored a preset trajectory in the Galaxea R1's Control Unit in the format of rosbag.

Before operating arms, ensure that the pose is as shown below, especially for joints 4, 5, and 6 of both arms. Since the arm remains stationary when powered on, you can turn it off to manually adjust the arm's pose.

  ![R1_arm_wave](assets/R1_arm_wave.png)

Use the command below to first make the arm wave and then perform a salute.

```Bash
rosbag play ~/Downloads/test_wave_salute.bag
```

