# Software Guide
We developed an efficient driver for converting the serial signals through the slave computer and finally released it as a ROS (Robot Operating System) topics. 
This driver not only enables the control of the slave computer, 
but also obtains the feedback information and error codes of the device, 
thus enabling two-way communication and real-time control. 
The tutorials will introduce how to use the programme to develop and operate A1. 

## Software Dependency
1. Ubuntu 20.04 LTS
2. ROS Noetic

## Installation
This SDK does not need to be recompiled, please refer to the Developing and Operating Tutorial to use directly.

## Developing and Operating Tutorials
### A1 Driver Kit
1. For the first time, after confirming the power supply and USB connection, 
run the following command to modify the read and write permission of serial port files:
```shell
sudo chmod 777 /dev/ttyACM0
```

2. After confirming the modification, you can initialize the SDK:
```shell
cd a1_driver_sdk/install
source setup.bash
roslaunch signal_arm single_arm_node.launch
```

The interface section describes the various control and status feedback interfaces for A1 robot arm, 
to help users better understand how to communicate and control the arm thriugh the ROS package.

### Driver Interface
The interface is a ROS package for manipulator control and status feedback. 
This package defines several topics for publishing and subscribing to the status of the robot arm, 
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

## Joint and End-Effector Movement Control
We provide the joint and end-effector movement control interfaces for A1 robotic arm, 
enabling efficient control of the arm through the ROS (Robot Operating System) framework. 
Whether you are performing end-effector movement or joint movement, 
you must first activate the `signal_arm` interface; 
detailed operation instructions can be found in the `signal_arm` documentation. 
This project encompasses several primary functions:

1. **End-Effector Pose Movement**: Enables users to control the position and orientation of the robotic arm's 
end-effector by publishing target pose messages, suitable for applications requiring precise positioning.

2. **End-Effector Trajectory Movement**: Achieves movement of the robotic arm's end-effector along a specified trajectory 
by publishing a series of pose messages, ideal for complex path planning and execution.

3. **Joint Angle Movement**: Offers a joint-level control interface where users can set the target positions for each individual joint, 
facilitating coordinated whole-arm movements.

### End-Effector Pose Movement
1. First, initiate the end-effector pose movement script, which will launch an RViz visualization for A1, with the default joint positions at zero.
```shell
cd release/install
source setup.bash
roslaunch mobiman eeTrackerdemo.launch
```
2. In File *eeTrackerdemo.launch*：
```shell
<param name="joint_states_topic" value="/joint_states" /> #topic /joint_states topic the channel for acquiring simulated values, specifically the states of the robot's joints, in a simulation environment.
<param name="joint_command" value="/a1_robot_right/arm_joint_command" /> #topic /a1_robot_right/arm_joint_command represents the topic of the issuing motor.
```
3. Send messages to the end-effector movement, 
specifically named `/a1_mpc_target`. This operation is non-blocking, 
allowing for continuous sending. It enables the end of the robotic arm to move seamlessly. 
However, it's critical that the target endpoint should not be too far from the current position of the robotic arm's end, 
to prevent overstraining the mechanics or risking a collision.
```shell
rostopic pub /a1_mpc_target geometry_msgs/PoseStamped "{
header: {
seq: 0,
stamp: {secs: 0, nsecs: 0},
frame_id: 'world'
},
pose: {
position: {x: 0.08, y: 0.0, z: 0.5},
orientation: {x: 0.5, y: 0.5, z: 0.5, w: 0.5}
}
}"
```
```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
def publish_pose():
rospy.init_node('pose_stamped_publisher', anonymous=True)
pose_pub = rospy.Publisher('/a1_mpc_target', PoseStamped, queue_size=10)
# Create message with type PoseStamped
pose_msg = PoseStamped()
pose_msg.header.seq = 0
pose_msg.header.stamp = rospy.Time.now()
pose_msg.header.frame_id = 'world'
pose_msg.pose.position.x = 0.
pose_msg.pose.position.y = 0.
pose_msg.pose.position.z = 0.
pose_msg.pose.orientation.x = 0.
pose_msg.pose.orientation.y = 0.
pose_msg.pose.orientation.z = 0.
pose_msg.pose.orientation.w = 0.
# Wait for subscribers to connect
rospy.sleep(1)
# Pulish message
pose_pub.publish(pose_msg)
rospy.loginfo("Published PoseStamped message to /a1_mpc_target")
if __name__ == '__main__':
try:
publish_pose()
except rospy.ROSInterruptException:
pass
```
4. Usage Example:
<div style="display: flex; justify-content: center; align-items: center;">
<video width="1920" height="1080" controls>
  <source src="../assets/A1_End-Effector_Motion.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</div>


### End-Effector Trajectory Movement
1. Firstly, initiate the end-effector trajectory movement script, 
which will launch an RViz visualization for A1, with the default joint positions set at zero.
```shell
cd release/install
source setup.bash
roslaunch mobiman eeTrajTrackerdemo.launch
```
2. in file *eeTrajTrackerdemo.launch* :
```shell
<param name="joint_states_topic" value="/joint_states" /> #/joint_states topic represents the channel for acquiring simulated values, specifically the states of the robot's joints, in a simulation environment.
<param name="joint_command" value="/a1_robot_right/arm_joint_command" /> #/a1_robot_right/arm_joint_command topic represents the topic of the issuing motor.
```
3. Publish messages to specify a trajectory for the end-effector movement, specially named  `/arm_target_trajectory`. 
This operation is non-blocking, allowing for continuous publishing. 
Note that the trajectory should not deviate significantly from the current end-effector position. However, 
it's recommended to wait until the current trajectory is 
completed before sending the next one to avoid inaccuracies in tracking the desired path.
```c++
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_array_publisher");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseArray>
        ("/arm_target_trajectory", 10);
    // Wait for subscribers to connect
    ros::Rate wait_rate(10);
    while (pose_pub.getNumSubscribers() == 0){
        wait_rate.sleep();
        }
    geometry_msgs::PoseArray poseArrayMsg;
    geometry_msgs::Pose pose1;
    pose1.position.x = 0.08;
    pose1.position.y = 0.0;
    pose1.position.z = 0.3;
    pose1.orientation.w = 0.5;
    pose1.orientation.x = 0.5;
    pose1.orientation.y = 0.5;
    pose1.orientation.z = 0.5;
    geometry_msgs::Pose pose2;
    pose2.position.x = 0.08;
    pose2.position.y = 0.0;
    pose2.position.z = 0.4;
    pose2.orientation.w = 0.5;
    pose2.orientation.x = 0.5;
    pose2.orientation.y = 0.5;
    pose2.orientation.z = 0.5;
    geometry_msgs::Pose pose3;
    pose3.position.x = 0.08;
    pose3.position.y = 0.0;
    pose3.position.z = 0.54;
    pose3.orientation.w = 0.5;
    pose3.orientation.x = 0.5;
    pose3.orientation.y = 0.5;
    pose3.orientation.z = 0.5;
    poseArrayMsg.poses.push_back(pose1);
    poseArrayMsg.poses.push_back(pose2);
    poseArrayMsg.poses.push_back(pose3);
    pose_pub.publish(poseArrayMsg);
    ROS_INFO("Published PoseArray with 3 poses");
    return 0;
}
```
4. 使用如影片所示:
<div style="display: flex; justify-content: center; align-items: center;">
<video width="1920" height="1080" controls>
  <source src="../assets/A1_End-Effector_Trajectory_Motion.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</div>

#### End-Effector Pose Movement Interface
| Topic Name     | Description                  | Message Type               |
|----------------|------------------------------|----------------------------|
| /a1_mpc_target | Target pose of end arm joint | Geometry_msgs::PoseStamped |

| Topic Name     | Field              | Description            |
|----------------|--------------------|------------------------|
| /a1_mpc_target | header             | Standard Header        |
| /a1_mpc_target | pose.position.x    | Shift in x direction   |
| /a1_mpc_target | pose.position.y    | Shift in y direction   |
| /a1_mpc_target | pose.position.z    | Shift in z direction   |
| /a1_mpc_target | pose.orientation.x | Orientation quaternion |
| /a1_mpc_target | pose.orientation.y | Orientation quaternion |
| /a1_mpc_target | pose.orientation.z | Orientation quaternion |
| /a1_mpc_target | pose.orientation.w | Orientation quaternion |

### Joint Angle Movement
Start the joint movement script first. 
This will launch an RViz instance for the A1 robotic arm, with joints initially at zero position.

```shell
cd release/install
source setup.bash
roslaunch mobiman jointTrackerdemo.launch
```

In the launch file:
```shell
<param name="joint_states_sub_topic" value="/joint_states" />

```
`/joint_states` represents the topic for obtaining simulation values.

```shell
<param name="arm_joint_command_topic" value="/arm_joint_command_host" />
```
`/arm_joint_command_host` represents the topic to publish motor commands.

Publish messages for joint movement. 
The message name is `/arm_joint_target_position`, which is non-blocking and can be continuously published. 
The robotic arm joints can move continuously.

```python
import rospy
from sensor_msgs.msg import JointState

def publish_joint_state():
    rospy.init_node('joint_state_publisher', anonymous=True)
    pub = rospy.Publisher('/arm_joint_target_position', JointState, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz
    joint_state = JointState()
    joint_state.header.seq = 0
    joint_state.header.stamp = rospy.Time.now()
    joint_state.header.frame_id = 'world'
    joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_state.velocity = []
    joint_state.effort = []
    # Initialize positions to zeros
    joint_state.position = [0, 0, 0, 0, 0, 0]
    steps = 100 # Number of steps

 #to reach the target position
    target_position = [0.5, 0, 0, 0, 0, 0]
    step_increment = [(target - current) / steps for target, current in zip(target_position, joint_state.position)]
    for step in range(steps):
        joint_state.header.stamp = rospy.Time.now() # Update the timestamp
        joint_state.position = [current + increment for current, increment in zip(joint_state.position, step_increment)]
        pub.publish(joint_state)
        rate.sleep()
    rospy.loginfo("Published JointState message to /arm_joint_target_position")

if __name__ == '__main__':
    try:
        publish_joint_state()
    except rospy.ROSInterruptException:
        pass
```




#### Joint Position Movement Interface
joint_move is a ROS package for single-joint control of A1 arms. 
This package is used to specify the movement of each joint from the current position to the specified position, 
the maximum speed and maximum acceleration during the movement can be specified, 
if not specified, the default speed and acceleration will be planned. 
The default maximum speed is 20 $rad/s$, and the maximum acceleration is $20 rad/s^2$. 
The topic names and fields of the movement interface are shown in the following table.

| Topic Name                  | Description                         | Message Type             |
|-----------------------------|-------------------------------------|--------------------------|
| /arm_joint_target_position  | Target position of each arm joint   | Sensor_msgs/JointState   |

| Topic Name                  | Field      | Description                                      |
|-----------------------------|------------|--------------------------------------------------|
| /arm_joint_target_position  | header     | Standard Header                                  |
| /arm_joint_target_position  | name       | Name of each arm joint                           |
| /arm_joint_target_position  | position   | Target position of each arm joint                |
| /arm_joint_target_position  | velocity   | Maximum velocity of each arm joint               |
| /arm_joint_target_position  | effort     | Maximum acceleration velocity of each arm joint  |
