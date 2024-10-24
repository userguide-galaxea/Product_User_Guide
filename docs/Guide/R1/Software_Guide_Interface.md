

# Software Interface

In this chapter, we describe the various control and status feedback interfaces for Galaxea R1, to help users better understand how to communicate and control the arm through the ROS package.

## Driver Interface

The current Galaxea R1 driver is mainly composed of three parts, including four independent Ros nodes: the drivers of the chassis, the left and right arms, and the torso. The interfaces provided by these drivers are in the form of ROS topics, as described below.

### Chassis Driver Interface

This interface is used for the chassis status feedback ROS package. The package defines multiple topics for posting the status of multiple motors in the chassis. The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/hdas/motor_state</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Chassis motion status feedback</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">chassis_msg/DrivetrainStamped</td>
  </tr>
</table>

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="15">/hdas/motor_state</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard ROS header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.steering_angle_fl</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Front wheel left wheel steering angle degrees</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.steering_angle_fr</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Front wheel right wheel steering angle degrees</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.steering_angle_rl</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Rear wheel left wheel steering angle degrees</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.steering_angle_rr</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Rear wheel right wheel steering angle degrees</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.drive_speed_fl</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Front wheel left wheel linear speed m/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.drive_speed_fr</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Front wheel right wheel linear speed m/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.drive_speed_rl</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Rear wheel left wheel linear speed m/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.drive_speed_rr</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Rear wheel right wheel linear speed m/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.drive_angular_speed_fl</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Front wheel left wheel angular speed rad/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.drive_angular_speed_fr</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Front wheel right wheel angular speed rad/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.drive_angular_speed_rl</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Rear wheel left wheel angular speed rad/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.drive_angular_speed_rr</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Rear wheel right wheel angular speed rad/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.motion_mode</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">N/A</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">mode</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Control mode</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="2">/gripper_joint_command</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">gripper_force</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Gripper force</td>
  </tr>
</table>

### Arms Driver Interface

This interface is used for arm control and status feedback ROS package. The package defines multiple topics for publishing and subscribing to the status of the robot arm, control commands, and associated error code information. The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/left_arm/joint_states/right_arm/joint_states</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Arm joint status feedback</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">sensor_msgs/JointState</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/left_arm/arm_status/right_arm/arm_status</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Arm motor status feedback</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">signal_arm/status_stamped</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/left_arm/arm_joint_command/right_arm/arm_joint_command</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Arm control interface</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">signal_arm/arm_control</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/left_arm/gripper_joint_command/right_arm/gripper_joint_command</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Gripper force control interface</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">signal_arm/gripper_joint_command</td>
  </tr>
</table>

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="4">/left_arm/joint_states/right_arm/joint_states</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">name</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Name of each arm joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">position</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Position of each arm joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">velocity</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Velocity of each arm joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="3">/left_arm/arm_status/right_arm/arm_status</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">status.name</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Name of each status</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">status.motor_errors</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Errors of each status</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="6">/left_arm/arm_joint_command/right_arm/arm_joint_command</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">p_des</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Position command of the arm</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">v_des</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Velocity command of the arm</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">t_ff</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Torque command of the arm</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">kp</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Proportion coefficient of position</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">kd</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Differential coefficient of position</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="2">/left_arm/gripper_joint_command/right_arm/gripper_joint_command</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">gripper_force</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Gripper force</td>
  </tr>
</table>

#### Diagnostic Trouble Code

DTC is used to feedback the error information of the MCU and the driver, and can be used to view the real-time status of each motor and the running status of the driver. The following is a detailed description of each fault code and its corresponding status.

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">DTC</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Status</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">0</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Disabled</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">1</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Disabled</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">2</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Motor Disconnected</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">3</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Position Jump</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">4</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Continuous High Current</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">5</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Excessive Torque</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">6</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">ECU -> ACU Timeout</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">7</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Position Limit Exceeded</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">8</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Speed Limit Exceeded</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">9</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Torque Limit Exceeded</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">10</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Overpressure</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">11</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Low pressure</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">12</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Overcurrent</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">13</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">MOS Overtemperature</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">14</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Motor Winding Overtemperature</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">15</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Communication loss</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">16</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Overload</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">17</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Serial Connection Disconnected (No Device File)</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">18</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Device File Connected, No Messages</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">19</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Serial Read/Write Failure</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">20</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Feedback Reception Overflow</td>
  </tr>
</table>

### Torso Driver Interface

This interface is used for torso control and status feedback ROS package. The package defines multiple  topics for publishing and subscribing to the status of torso motors and control commands. The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/torso_feedback</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Joint status feedback of the torso motor</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">sensor_msgs/JointState</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/torso_control</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Torso motor status feedback</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">signal_arm/arm_control</td>
  </tr>
</table>


<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="5">/torso_feedback</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">name</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Name of each torso joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">position</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Position of each torso joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">velocity</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Velocity of each torso joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">effort</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">N/A</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="6">/torso_control</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">p_des</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Position command of torso</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">v_des</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Velocity command of torso</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">t_ff</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">N/A</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">kp</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">N/A</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">kd</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">N/A</td>
  </tr>
</table>

## Operation Control Interface

The current Galaxea R1 operation and control interface is mainly composed of three parts: the chassis, the left and right arms, and the torso. Each drive has different motion interface exposure. The interfaces provided by these drivers are in the form of ROS topics as described below and available to users.

### Chassis Control Interface

The chassis control interface can be used to command the chassis to move at the target speed, which includes the combination of X, Y, and Yaw Rate:

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/cmd_vel</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Issuing the chassis control signal </td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">geometry_msgs/Twist</td>
  </tr>
</table>


<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="6">/cmd_vel</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard ROS header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">linear</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Control of linear speed</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.x</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Linear speed of x, range (-1 to 1) m/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.y</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Linear speed of y, range (-1 to 1) m/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">angular</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Control of yaw rate</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.z</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Control of angular speed, -1 to 1 rad/s</td>
  </tr>
</table>

### Arms Control Interface

#### Joint Position Movement Interface

`joint_move` is a ROS package for the single-joint control of robot arms and is used to move each joint from the current position to the target position. The maximum speed and maximum acceleration can be specified; if not, the default maximum speed, 20 rad/s, and the default maximum acceleration, 20 rad/s², will be used for planning. 
The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/left_arm/arm_joint_target_position/right_arm/arm_joint_target_position</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Target position and pose of each arm joint</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Sensor_msgs/JointState</td>
  </tr>
</table>


<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="5">/left_arm/arm_joint_target_position/right_arm/arm_joint_target_position</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">name</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Name of each arm joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">position</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Position of each arm joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">velocity</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Velocity of each arm joint</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">effort</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Torque of each arm joints</td>
  </tr>
</table>

#### End Pose Movement Interface

`eef_move`is a ROS package for the end control of robot arms and is used to move the end of the robot arm to the target position with the target pose. 

**Note:** If the given position and pose are unreachable, the robot arm will be as close to the target position and pose as possible through the optimization function. When setting the targets, the weight of the position gap is twice the weight of the pose gap.

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/left_arm/arm_target_pose/right_arm/arm_target_pose</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Target positions of the end of the robot arm</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Geometry_msgs::PoseStamped</td>
  </tr>
</table>


<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="8">/left_arm/arm_target_pose/right_arm/arm_target_pose</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.position.x</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Shift in the X direction</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.position.y</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Shift in the Y direction</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.position.z</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Shift in the Z direction</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.x</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.y</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.z</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.w</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
</table>


#### End Trajectory Movement interface

`eef_follow` is a ROS package for the end control of robot arms and is used to move the end following the target trajectory to the target position.

**Note:** If the given position and pose are unreachable, the robot arm will be as close to the target position and pose as possible through the optimization function. When setting the targets, the weight of the position gap is twice the weight of the pose gap.

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/left_arm/arm_target_trajectory/right_arm/arm_target_trajectory</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Target trajectory of the end of arms</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Geometry_msgs::PoseArray</td>
  </tr>
</table>


<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="9">/left_arm/arm_target_trajectory/right_arm/arm_target_trajectory</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">posesgeometry_msgs/Pose[]</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description of pose in the BaseLink coordinate system, where each pose has a fixed time interval of 0.1s.</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.position.x</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Shift in the X direction</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.position.y</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Shift in the Y direction</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.position.z</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Shift in the Z direction</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.x</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.y</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.z</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.w</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
</table>

### Torso Control Interface
The coordinate system is defined as follows:

- The coordinate axes represent the end of the torso.

- The red, green, and blue lines represent the X, Y, and Z axes, respectively.

![R1_torso_control_interface](assets/R1_torso_control_interface.png)

#### Torso Speed Control Interface

`torso_control` is a ROS package for the end control of the torso and is used to move the end of the torso at a target speed.

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/target_torso_speed</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Issuing the torso control signal</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">geometry_msgs/Twist</td>
  </tr>
</table>


<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="6">/cmd_vel</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard ROS header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">linear</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">control of linear speed</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.z</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">linear_speed of z, range (-0.2 to 0.2) m/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">angular</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">control of yaw rate</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.x</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">control of angular speed in pitch direction, -0.5 - 0.5 rad/s</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">.y</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">control of angular speed in pitch direction, -0.5 - 0.5 rad/s</td>
  </tr>
</table>

#### Torso Pose Control Interface

`torso_control` is a ROS package for the end control of the torso and is used to move the pose of the torso to the target height and to the corresponding pitch and yaw.
This pose is subject to certain constraints, as described below.

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Message Type</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">/torso_target_pose</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Target pose of end arm joint</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Geometry_msgs::PoseStamped</td>
  </tr>
</table>


<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="8">/torso_target_pose</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Standard Header</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.position.x</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Shift in x-direction in the range of (0,0.25)</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.position.z</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Shift in z-direction in the range of (0,1)</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">The constraint of -Pitch satisfies sin(pitch) (-x/0.32, (0.25-x)/0.32). The constraint of -Yaw is satisfied with (-3.05-3.05)</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.x</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.y</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.z</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">pose.orientation.w</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">Orientation quaternion</td>
  </tr>
</table>

## Torso Re-calibration

### Disable the Auto-start Torso Control Interface

Before performing zero-point calibration, please first disable the auto-start torso control algorithm.

 ```Bash
    ps aux | grep torso_control 
    ### Find the Torso Control Node PID First
    kill -9 (PID)
    ### PID is the found PID
 ```

### Adjust Single-joint Position Control Interface

Please proceed with caution during the adjustment and ensure that the delta position of each joint is less than 10 degrees each time. Adjust each Torso joint one at a time, and hold the robot during the adjustment to prevent any potential tipping.

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

### Rosservice Call

<u>Importance: Hold the robot by hand.</u>

Once near the zero-point, you can use the following interface for calibration. The zero-point posture is as shown. When issuing commands, the motor brakes will briefly release, so please hold the robot to prevent calibration errors.

![R1_torso_recalibration](assets/R1_torso_recalibration.png)

```Bash
rosservice call /torso_node/function_frame "0x03"
```

* Repower the system and check if it is at zero.

```Bash
rostopic echo /torso_feedback 
### Check Current Torso Feedback Position Check whether calibration succeed 
```

