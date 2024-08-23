# Galaxea R1 Software Guide

## Software Dependency

1. [Ubuntu](https://en.wikipedia.org/wiki/Ubuntu) 20.04 LTS
2. ROS Noetic

## Installation

The SDK does not require recompilation. Please refer to the contents below.

## First Move

### Arm - Wave & Salute

Since the arm control is complex and cannot be managed with just two joysticks, we have stored a preset trajectory in the Galaxea R1's Control Unit in the format of rosbag.

Before operating arms, ensure that the pose is as shown below, especially for joints 4, 5, and 6 of both arms. Since the arm remains stationary when powered on, you can turn it off to manually adjust the arm's pose.

![R1_arm_wave](assets/R1_arm_wave.png)



Use the command below to make the arm wave first and salute next.

```Bash
rosbag play ~/Downloads/test_wave_salute.bag
```

### Torso - Squat & Stand

Controller teleoperation primarily involves using a Python script to map messages from the topic `/controller` to the topic `/target_torso_speed`.   (`/controller` is sent by Chassis Control Unit to Control Unit via CAN.) Therefore, the torso is actually controlled by the topic `/target_torso_speed`.

Switch SWB and SWC to the top to quit the Torso Controller Mode. This will ensure that the topic `/controller` is NOT published. And use the upper computer to control the torso.

**1. Move the Torso:**

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

**2. Stop the Torso:**

```Bash
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

### Chassis - Spins in place

Switch SWB to the bottom and switch SWC to the middle to enter the Chassis Upper Computer Mode.

**1. Spin the chassis:**

```Bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1" 
```

**2. Stop the chassis:**

You can stop the chassis by sending the command below:

```Bash
  rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

Or by switching SWB and SWC to the top to quit the Chassis Upper Computer Mode.

## Torso Re-calibration

**1. Disable the Auto-start Torso Control Interface**

Before performing zero-point calibration, please first disable the auto-start torso control algorithm.

- ```Bash
    ps aux | grep torso_control 
    ### Find the Torso Control Node PID First
    kill -9 (PID)
    ### PID is the found PID
    ```

**2. Adjust Single-joint Position Control Interface**

Please proceed with caution during the adjustment and ensure that the delta position of each joint is less than 10 degrees each time. Adjust each Torso joint one at a time, and support the Galaxea R1 during the adjustment to prevent any potential tipping.

```Bash
rostopic echo /torso_feedback 
## Find Current Torso Feedback Position First, p1, p2, p3, p4 for the current degree
source ${SDK_PATH}/install/setup.bash
rostopic pub /torso_control signal_arm/arm_control "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
p_des: [${p1+delta_p1}, ${p2+delta_p2}, ${p3+delta_p3}, ${p4+delta_p4}]
v_des: [0.1, 0.1, 0.1, 0.1]
kp: [0]
kd: [0]
t_ff: [0]
mode: 0"

### delta_pi is the delta degree wanna each joint to move.
### delta_p1 > 0 -> T1 Moves Forward 
### delta_p2 > 0 -> T2 Moves Forward 
### delta_p3 > 0 -> T3 Moves Backward 
### delta_p4 > 0 -> T4 Moves anticlockwise 
```

**3. Rosservice Call**

<u>Important: Hold the robot by hand.</u>

Once near the zero-point, you can use the following interface for calibration. The zero-point posture is as shown. When issuing commands, the motor brakes will briefly release, so please hold the robot to prevent calibration errors.

![R1_torso_recalibration](assets/R1_torso_recalibration.png)

```Bash
rosservice call /torso_node/function_frame "0x03"
```

Repower the system and check if it is at zero.

```Bash
rostopic echo /torso_feedback 
### Check Current Torso Feedback Position Check whether calibration succeed 
```

## Software Interface

In this chapter, we describe the various control and status feedback interfaces for Galaxea R1, to help users better understand how to communicate and control the arm through the ROS package.

### Driver Interface

The current Galaxea R1 driver is mainly composed of three parts, including four independent ROS nodes: the drivers of the chassis, the left and right arms, and the torso. The interfaces provided by these drivers are in the form of ROS topics, as described below.

#### Chassis Driver Interface

This interface is used for the chassis status feedback ROS package. The package defines multiple topics for posting the status of multiple motors in the chassis. The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr style="background-color: black; color: white; text-align: left;">
      <th style="padding: 8px; border: 1px solid #ddd; width: 200px;">Topic Name</th>
      <th style="padding: 8px; border: 1px solid #ddd; width: 300px;">Description</th>
      <th style="padding: 8px; border: 1px solid #ddd;">Message Type</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white; text-align: left;">
      <td style="padding: 8px; border: 1px solid #ddd; width: 200px;">/hdas/motor_state</td>
      <td style="padding: 8px; border: 1px solid #ddd; width: 300px;">Chassis motion state feedback</td>
      <td style="padding: 8px; border: 1px solid #ddd;">chassis_msg/DrivetrainStamped</td>
    </tr>
  </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="13">/hdas/motor_state</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.steering_angle_fl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel left-wheel steering angle degrees</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.steering_angle_fr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel right-wheel steering angle degrees</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.steering_angle_rl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel left-wheel steering angle degrees</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.steering_angle_rr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel right-wheel steering angle degrees</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_speed_fl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel left-wheel linear speed m/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_speed_fr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel right-wheel linear speed m/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_speed_rl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel left-wheel linear speed m/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_speed_rr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel right-wheel linear speed m/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_angular_speed_fl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel left-wheel angular speed rad/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_angular_speed_fr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel right-wheel angular speed rad/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_angular_speed_rl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel left-wheel angular speed rad/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_angular_speed_rr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel right-wheel angular speed rad/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.motion_mode</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Motion Mode 0: Dual Ackerman Mode <br /> Motion Mode 1: Parallel Mode <br /> Motion Mode 2: Spinning Mode</td>
  </tr>
</table>


#### Arms Driver Interface

This interface is used for robot arm control and status feedback ROS package. The package defines multiple  topics for publishing and subscribing to the status of the robot arm, control commands, and associated error code information. The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/left_arm/joint_states<br>/right_arm/joint_states</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Robot arm joint status feedback</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">sensor_msgs/JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/left_arm/arm_status
<br>/right_arm/arm_status</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Arm motor status feedback</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">signal_arm/status_stamped</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/left_arm/arm_joint_command<br>/right_arm/arm_joint_command</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Arm joint control interface</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">signal_arm/arm_control</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/left_arm/gripper_joint_command<br>/right_arm/gripper_joint_command</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Gripper force control interface</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">signal_arm/gripper_joint_command</td>
        </tr>        
    </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Field</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="5">/left_arm/joint_states<br>/right_arm/joint_states</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Name of each arm joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Position of each arm joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Velocity of each arm joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Torque of each arm joint</td>            
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="3">/left_arm/arm_status<br>/right_arm/arm_status</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">status.name</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Name of each status</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">status.motor_errors</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Errors of each status</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="7">/left_arm/arm_joint_command<br>/right_arm/arm_joint_command</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Position command of robot arm</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Velocity command of robot arm</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Torque command of robot arm</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kp</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Proportion coefficient of position</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kd</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Differential coefficient of position</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Control mode</td>
 </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="7">/left_arm/gripper_joint_command<br>/right_arm/gripper_joint_command</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">gripper_force</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Gripper force</td>
        </tr>
    </tbody>
</table>


##### Diagnostic Trouble Code

DTC is used to feedback the error information of the MCU and the drive, and can be used to view the real-time status of each motor and the running status of the drive. The following is a detailed description of each fault code and its corresponding status.

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 100px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">DTC</th>
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Status</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">0</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Disabled</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Disabled</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">2</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Motor Disconnected</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">3</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Position Jump</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">4</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Continuous High Current</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">5</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Excessive Torque</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">6</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">ECU -> MCU Timeout</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">7</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Position Limit Exceeded</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">8</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Speed Limit Exceeded</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">9</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Torque Limit Exceeded</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">10</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Overpressure</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">11</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Low pressure</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">12</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Overcurrent</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">13</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">MOS Overtemperature</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">14</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Motor Winding Overtemperature</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">15</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Communication loss</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">16</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Overload</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">17</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Serial Connection Disconnected (No Device File)</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">18</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Device File Connected, No Messages</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">19</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Serial Read/Write Failure</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">20</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Feedback Reception Overflow</td>
        </tr>
    </tbody>
</table>


#### Torso Driver Interface

This interface is used for torso control and status feedback ROS package. The package defines multiple  topics for publishing and subscribing to the status of torso motors and control commands. The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/torso_feedback</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Torso motor joint status feedback</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">sensor_msgs/JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/torso_control</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Torso motor status feedback</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">signal_arm/arm_control</td>
        </tr>
    </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Field</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="5">/torso_feedback</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Name of each torso joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Position of each torso joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Velocity of each torso joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">N/A</td>            
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="6">/torso_control</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Position command of torso</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Velocity command of torso</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">N/A</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kp</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">N/A</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kd</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">N/A</td>
        </tr>
    </tbody>
</table>


### Control Interface

The current Galaxea R1 operation and control interface is mainly composed of three parts: the chassis, the left and right arms, and the torso. Each drive has different motion interface exposure. The interfaces provided by these drivers are in the form of ROS topics as described below and available to users.

#### Chassis Control Interface

The chassis control interface can be used to command the chassis to move at the target speed, which includes the combination of X, Y, and Yaw Rate:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/cmd_vel</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Issuing the chassis control signal </td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">geometry_msgs/Twist</td>
        </tr>
    </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Field</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="6">/cmd_vel</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard ROS header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">linear</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">control of linear speed</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">linear_speed of x, range (-1 to 1) m/s</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.y</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">linear_speed of y, range (-1 to 1) m/s</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">angular</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">control of yaw rate</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.z</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">control of angular speed, -1 - 1 rad/s</td>
        </tr>
    </tbody>
</table>


#### Arms Control Interface

##### Joint Position Movement Interface

`joint_move` is a ROS package for single-joint control of robot arms and is used to move each joint from the current position to the target position. The maximum speed and maximum acceleration can be specified; if not, the default maximum speed, 20 rad/s, and the default maximum acceleration, 20 rad/s², will be used for planning. <br>

The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/left_arm/arm_joint_target_position <br>/right_arm/arm_joint_target_position</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Target position and pose of each arm joint </td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Sensor_msgs/JointState</td>
        </tr>
    </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Field</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="5">/left_arm/arm_joint_target_position<br>/right_arm/arm_joint_target_position</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Name of each arm  joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Position of each arm  joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Velocity of each arm  joint</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Torque of each arm joint</td>            
        </tr>
    </tbody>
</table>


##### End-Effector Pose Movement Interface

`eef_move`is a ROS package for end control of robot arms and is used to move the end-effector to the target position with the target pose.

**Note:** If the given position and pose are unreachable, the robot arm will be as close to the target position and pose as possible through the optimization function. When setting the targets, the weight of the position gap is twice the weight of the pose gap).

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/left_arm/arm_target_pose <br>/right_arm/arm_target_pose</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Target position and pose of the end-effector </td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Geometry_msgs::PoseStamped</td>
        </tr>
    </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Field</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="8">/left_arm/arm_target_pose <br>/right_arm/arm_target_pose</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard ROS header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Shift in the X direction</td>
        </tr>
 		<tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.y</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Shift in the Y direction</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.z</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Shift in the Z direction</td>
        </tr>  
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr> 
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.y</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr>         
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.z</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr> 
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.w</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr>        
    </tbody>
</table>


##### End-Effector Trajectory Movement interface

`eef_follow` is a ROS package for the end-effector control of robot arms and is used to move the end following the target trajectory to the target position.

**Note:** If the given position and pose are unreachable, the robot arm will be as close to the target position and pose as possible through the optimization function. When setting the targets, the weight of the position gap is twice the weight of the pose gap.

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/left_arm/arm_target_trajectory <br>/right_arm/arm_target_trajectory</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Target trajectory of the end-effetor</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Geometry_msgs::PoseArray</td>
        </tr>
    </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Field</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="9">/left_arm/arm_target_trajectory <br>/right_arm/arm_target_trajectory</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard ROS header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">poses<br>geometry_msgs/Pose[]</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description of pose in the BaseLink coordinate system, where each pose has a fixed time interval of 0.1s.</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Shift in the X direction</td>
        </tr>        
 		<tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.y</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Shift in the Y direction</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.z</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Shift in the Z direction</td>
        </tr>  
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr> 
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.y</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr>         
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.z</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr> 
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.w</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr>        
    </tbody>
</table>


#### Torso Control Interface

The coordinate system is defined as follows:

- The coordinate axes represent the end of the torso.
- The red, green, and blue lines represent the X, Y, and Z axes, respectively.

![img](assets/R1_torso_control_interface.png)

##### Torso Speed Control Interface

`torso_control` is a ROS package for the end control of the torso and is used to move the end of the torso at a target speed.

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/target_torso_speed</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Issuing the torso control signal</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">geometry_msgs/Twist</td>
        </tr>
    </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Field</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="6">/cmd_vel</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard ROS header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">linear</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">control of linear speed</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.z</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">linear_speed of z, range (-0.2 to 0.2 ) m/s</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">angular</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">control of yaw rate</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">control of angular speed in pitch direction, ±0.5 rad/s</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.y</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">control of angular speed in pitch direction, ±0.5 rad/s</td>
        </tr>  
    </tbody>
</table>


##### Torso Pose Control Interface

`torso_control` is a ROS package for the end control of the torso and is used to move the pose of the torso to the target height and to the corresponding pitch and yaw.

This pose is subject to certain constraints, as described below.

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/torso_target_pose</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Target pose of arm end joint</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Geometry_msgs::PoseStamped</td>
        </tr>
    </tbody>
</table>


<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Field</th>
            <th style="width: 300px; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="8">/torso_target_pose</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Standard ROS header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Shift in x-direction in the range of (0,0.25)</td>
        </tr>        
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.z</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Shift in z-direction in the range of (0,1)</td>
        </tr>  
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">The constraint of -Pitch satisfies sin(pitch) (-x/0.32, (0.25-x)/0.32)<br>The constraint of -Yaw is satisfied with ±3.05.</td>
        </tr> 
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.x</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr> 
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.y</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr>         
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.z</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr> 
        <tr style="background-color: white;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.w</td>
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Orientation quaternion</td>
        </tr>        
    </tbody>
</table>


## Use Case