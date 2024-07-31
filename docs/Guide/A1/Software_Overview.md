# Software Overview
We developed an efficient driver for converting the serial signals through the slave computer and finally released it as a ROS (Robot Operating System) topics. 
This driver not only enables the control of the slave computer, 
but also obtains the feedback information and error codes of the device, 
thus enabling two-way communication and real-time control. 
The tutorials will introduce how to use the programme to develop and operate A1. 

## Software Dependency
1. Ubuntu 20.04LTS
2. ROS Noetic

## Installation

## Developing and Operating Tutorials
### A1 Driver Kit
```shell
sudo chmod 777 /dev/ttyACM0
```

```shell
cd a1_driver_sdk/install
source setup.bash
roslaunch signal_arm single_arm_node.launch
```

The interface section describes the various control and status feedback interfaces for A1 robot arm, 
to help users better understand how to communicate and control the arm thriugh the ROS package.
### Driver Interface
The interface is a ROS package for manipulator control and status feedback. 
This package defines several topics for Posting and subscribing to the status of the robot arm, 
control commands, and associated error code information. 
Below are detailed descriptions of each topic and its related message types:

| Topic Name             | Description                     | Message Type                     |
|------------------------|---------------------------------|----------------------------------|
| /joint_states          | Robot arm joint state feedback  | sensor_msgs/JointState           |
| /arm_status            | Robot arm motor status feedback | signal_arm/arm_status            |
| /arm_joint_command     | Robot arm control interface     | signal_arm/arm_control           |
| /gripper_joint_command | Gripper force control interface | signal_arm/gripper_joint_command |



| Topic Name             | Field         | Description                                          | Data Type | Unit      | Notes                  |
|------------------------|---------------|------------------------------------------------------|-----------|-----------|------------------------|
| /joint_states          | header        | Standard ROS message header                          | -         | -         | -                      |
|                        | name          | Names of the robot arm joints                        | string[]  | -         | -                      |
|                        | position      | Positions of the robot arm joints                    | float64[] | $rad$     | -                      |
|                        | velocity      | Velocities of the robot arm joints                   | float64[] | $rad/s$   | -                      |
|                        | effort        | Torques of the robot arm joints                      | float64[] | $Nm$      | -                      |
| /arm_status            | header        | Standard ROS message header                          | -         | -         | -                      |
|                        | name          | Names of the robot arm joints                        | string[]  | -         | -                      |
|                        | error_code    | Error codes for the robot arm joints                 | float32[] | -         | -                      |
|                        | t_mos         | MOS chip temperature of the robot arm joints         | float32[] | $Celsius$ | -                      |
|                        | t_rotor       | Internal encoder temperature of the robot arm joints | float32[] | $Celsius$ | -                      |
| /arm_joint_command     | header        | Standard ROS message header                          | -         | -         | -                      |
|                        | p_des         | Desired position of the robot arm                    | float32[] | $rad$     | -                      |
|                        | v_des         | Desired velocity of the robot arm                    | float32[] | $rad/s$   | -                      |
|                        | t_ff          | Desired torque of the robot arm                      | float32[] | $Nm$      | -                      |
|                        | kp            | Proportional gain for position                       | float32[] | -         | -                      |
|                        | kd            | Derivative gain for position                         | float32[] | -         | -                      |
|                        | mode          | Control mode                                         | uint8     | -         | Default 0, MIT control |
| /gripper_force_control | header        | Standard ROS message header                          | -         | -         | -                      |
|                        | gripper_force | Gripper support force                                | float32   | $N$       | -                      |


### Diagnostic Trouble Code
DTC is used to feedback the error information of the ACU and the drive, 
and can be used to view the real-time status of each motor and the running status of the drive. 
The following is a detailed description of each fault code and its corresponding status.

| Fault Code Position | Corresponding Status                                             |
|---------------------|------------------------------------------------------------------|
| 0                   | ACU Feedback: Disabled                                           |
| 1                   | ACU Feedback: Enabled                                            |
| 2                   | ACU Feedback: Motor Disconnected                                 |
| 3                   | ACU Feedback: Position Jump                                      |
| 4                   | ACU Feedback: Persistent High Current                            |
| 5                   | ACU Feedback: Excessive Torque                                   |
| 6                   | ACU Feedback: ECU -> ACU Timeout                                 |
| 7                   | ACU Feedback: Position Limit Exceeded                            |
| 8                   | ACU Feedback: Speed Limit Exceeded                               |
| 9                   | ACU Feedback: Torque Limit Exceeded                              |
| 10                  | ACU Feedback: Overpressure                                       |
| 11                  | ACU Feedback: Low Pressure                                       |
| 12                  | ACU Feedback: Overcurrent                                        |
| 13                  | ACU Feedback: MOS Overtemperature                                |
| 14                  | ACU Feedback: Motor Winding Overtemperature                      |
| 15                  | ACU Feedback: Communication Loss                                 |
| 16                  | ACU Feedback: Overload                                           |
| 17                  | Driver Feedback: Serial Connection Disconnected (No Device File) |
| 18                  | Driver Feedback: Device File Connected, No Messages              |
| 19                  | Driver Feedback: Serial Read/Write Failure                       |
| 20                  | Driver Feedback: Feedback Reception Overflow                     |

