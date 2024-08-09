# Software Guide
We developed an efficient driver for converting serial signals through the slave computer, which has been released as a ROS (Robot Operating System) topic. This driver not only enables control of the slave computer but also retrieves feedback information and error codes from the device, facilitating two-way communication and real-time control. The tutorials will guide you on how to use this program to develop and operate A1.

## Software Dependency
1. Ubuntu 20.04 LTS
2. ROS Noetic

## Installation
This SDK does not require recompilation. Please refer to the Developing and Operating Tutorials for direct usage instructions.

## First Move
```shell
cd release/install
source setup.bash
roslaunch mobiman eeTrackerdemo.launch
```
```shell
rostopic pub /a1_ee_target geometry_msgs/PoseStamped "{
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


## Developing and Operating Tutorials
### A1 Driver Kit
1. For the first use, after confirming the power supply and USB connection, run the following command to modify the read and write permissions of the serial port files:
```shell
sudo chmod 777 /dev/ttyACM0
```

2. After confirming the modification, you can initialize the SDK:
```shell
cd a1_driver_sdk/install
source setup.bash
roslaunch signal_arm single_arm_node.launch
```

The interface section describes the various control and status feedback interfaces for A1 robot arm, helping users understand how to communicate with and control the arm through the ROS package.

#### Driver Interface
The interface is a ROS package designed for manipulator control and status feedback. This package defines several topics for publishing and subscribing to the robot arm’s status, control commands, and associated error codes. Below are detailed descriptions of each topic and its related message types:

<table style="border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white;text-align: left;">
            <th>Topic Name </th>
            <th>Description </th>
	    <th>Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;text-align: left;">
            <td>/joint_states</td>
            <td>Robot arm joint status feedback</td>
			<td>sensor_msgs/JointState</td>
        </tr>
        <tr style="background-color: white;text-align: left;">
            <td>/arm_status</td>
            <td>Robot arm motor status feedback</td>
			<td>signal_arm/status_stamped</td>
        </tr>
        <tr style="background-color: white;text-align: left;">
            <td>/arm_joint_command</td>
            <td>Robot arm joint control interface</td>
			<td>signal_arm/arm_control</td>
        </tr>
    </tbody>
</table>

<table style="border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white;text-align: left">
            <th>Topic Name</th>
            <th>Field</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>/joint_states</td>
            <td>Header</td>
            <td>Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>name</td>
            <td>Name of each arm joint</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>position</td>
            <td>Position of each arm joint</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>velocity</td>
            <td>Velocity of each arm joint</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>effort</td>
            <td>Torque of each arm joint</td>
        </tr>
        <tr style="background-color: white;">
            <td>/arm_status</td>
            <td>header</td>
            <td>Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>status.name</td>
            <td>Name of each status</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>status.motor_errors</td>
            <td>Errors of each status</td>
        </tr>
        <tr style="background-color: white;">
            <td>/arm_joint_command</td>
            <td>header</td>
            <td>Standard header</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>p_des</td>
            <td>Position command of robot arm</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>v_des</td>
            <td>Velocity command of robot arm</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>t_ff</td>
            <td>Torque command of robot arm</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>kp</td>
            <td>Proportion coefficient of position</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>kd</td>
            <td>Differential coefficient of position</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>mode</td>
            <td>Control mode</td>
        </tr>
    </tbody>
</table>

#### Diagnostic Trouble Code

DTC is used to feedback the error information of the MCU and the drive, 
and can be used to view the real-time status of each motor and the running status of the drive. The following is a detailed description of each fault code and its corresponding status.

<table style="border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white;text-align:left">
            <th>DTC</th>
            <th>Status</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>0</td>
            <td>Disabled</td>
        </tr>
        <tr style="background-color: white;">
            <td>1</td>
            <td>Disabled</td>
        </tr>
        <tr style="background-color: white;">
            <td>2</td>
            <td>Motor Disconnected</td>
        </tr>
        <tr style="background-color: white;">
            <td>3</td>
            <td>Position Jump</td>
        </tr>
        <tr style="background-color: white;">
            <td>4</td>
            <td>Continuous High Current</td>
        </tr>
        <tr style="background-color: white;">
            <td>5</td>
            <td>Excessive Torque</td>
        </tr>
        <tr style="background-color: white;">
            <td>6</td>
            <td>ECU -> MCU Timeout</td>
        </tr>
        <tr style="background-color: white;">
            <td>7</td>
            <td>Position Limit Exceeded</td>
        </tr>
        <tr style="background-color: white;">
            <td>8</td>
            <td>Speed Limit Exceeded</td>
        </tr>
        <tr style="background-color: white;">
            <td>9</td>
            <td>Torque Limit Exceeded</td>
        </tr>
        <tr style="background-color: white;">
            <td>10</td>
            <td>Overpressure</td>
        </tr>
        <tr style="background-color: white;">
            <td>11</td>
            <td>Low pressure</td>
        </tr>
        <tr style="background-color: white;">
            <td>12</td>
            <td>Overcurrent</td>
        </tr>
        <tr style="background-color: white;">
            <td>13</td>
            <td>MOS Overtemperature</td>
        </tr>
        <tr style="background-color: white;">
            <td>14</td>
            <td>Motor Winding Overtemperature</td>
        </tr>
        <tr style="background-color: white;">
            <td>15</td>
            <td>Communication loss</td>
        </tr>
        <tr style="background-color: white;">
            <td>16</td>
            <td>Overload</td>
        </tr>
        <tr style="background-color: white;">
            <td>17</td>
            <td>Serial Connection Disconnected (No Device File)</td>
        </tr>
        <tr style="background-color: white;">
            <td>18</td>
            <td>Device File Connected, No Messages</td>
        </tr>
        <tr style="background-color: white;">
            <td>19</td>
            <td>Serial Read/Write Failure</td>
        </tr>
        <tr style="background-color: white;">
            <td>20</td>
            <td>Feedback Reception Overflow</td>
        </tr>
    </tbody>
</table>

### Joint and End-Effector Movement Control

We provide joint and end-effector movement control interfaces for A1 robot arm, enabling efficient control through the ROS (Robot Operating System) framework. Before performing end-effector or joint movement, you must first activate the `signal_arm` interface; detailed operation instructions can be found in the  `signal_arm` documentation. This project includes several primary functions:

- **End-Effector Pose Movement**: Allows users to control the position and orientation of the robot arm's end-effector by publishing target pose messages. This function is suitable for applications requiring precise positioning.

- **End-Effector Trajectory Movement**: Facilitates the movement of the robot arm's end-effector along a specified trajectory by publishing a series of pose messages. This function is ideal for complex path planning and execution.

- **Joint Angle Movement**: Provides a joint-level control interface where users can set the target positions for each individual joint, enabling coordinated whole-arm movements.

#### End-Effector Pose Movement

1. First, initiate the end-effector pose movement script. This will launch an RViz visualization for A1 robot arm, with the default joint positions set to zero.
```shell
cd release/install
source setup.bash
roslaunch mobiman eeTrackerdemo.launch
```
2. In the file *eeTrackerdemo.launch*：
```shell
<param name="joint_states_topic" value="/joint_states" /> # the topic /joint_states  represents the channel for acquiring simulated values, specifically the states of the robot's joints, within a simulation environment.
<param name="joint_command" value="/a1_robot_right/arm_joint_command" /> # the topic /a1_robot_right/arm_joint_command topic represents the channel for issuing commands to the motors.
```
3. Publish messages to the end-effector movement topic, specifically named `/a1_ee_target`. This operation is non-blocking, allowing for continuous message sending and enabling seamless movement of the robot arm's end-effector. However, it's critical that the target endpoint is not too far from the current position of the end-effector to avoid overstraining the mechanics or risking a collision.
```shell
rostopic pub /a1_ee_target geometry_msgs/PoseStamped "{
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
pose_pub = rospy.Publisher('/a1_ee_target', PoseStamped, queue_size=10)
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
rospy.loginfo("Published PoseStamped message to /a1_ee_target")
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


#### End-Effector Trajectory Movement

1. Firstly, initiate the end-effector trajectory movement script. This will launch an RViz visualization for A1 robot arm, with the default joint positions set to zero.
```shell
cd release/install
source setup.bash
roslaunch mobiman eeTrajTrackerdemo.launch
```
2. In the file *eeTrajTrackerdemo.launch* :
```shell
<param name="joint_states_topic" value="/joint_states" /> # the /joint_states topic represents the channel for acquiring simulated values, specifically the states of the robot's joints, within a simulation environment.
<param name="joint_command" value="/a1_robot_right/arm_joint_command" /> #the /a1_robot_right/arm_joint_command topic represents the channel for issuing commands to the motors.
```
3. PPublish messages to specify a trajectory for the end-effector movement on the   `/arm_target_trajectory` topic. This operation is non-blocking, allowing for continuous publishing. Ensure that the trajectory does not deviate significantly from the current end-effector position. It is recommended to wait until the current trajectory is completed before sending the next one to avoid inaccuracies in tracking the desired path.
```c++
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

geometry_msgs::Pose createPose(double x, double y, double z, double w, double ox, double oy, double oz) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = w;
    pose.orientation.x = ox;
    pose.orientation.y = oy;
    pose.orientation.z = oz;
    return pose;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_array_publisher");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseArray>("/arm_target_trajectory", 10);
    
    // Wait for subscribers to connect
    ros::Rate wait_rate(10);
    while (pose_pub.getNumSubscribers() == 0) {
        wait_rate.sleep();
    }

    geometry_msgs::PoseArray poseArrayMsg;

    poseArrayMsg.poses.push_back(createPose(0.08, 0.0, 0.3, 0.5, 0.5, 0.5, 0.5));
    poseArrayMsg.poses.push_back(createPose(0.08, 0.0, 0.4, 0.5, 0.5, 0.5, 0.5));
    poseArrayMsg.poses.push_back(createPose(0.08, 0.0, 0.54, 0.5, 0.5, 0.5, 0.5));

    pose_pub.publish(poseArrayMsg);
    ROS_INFO("Published PoseArray with 3 poses");

    return 0;
}

```
4. Usage Example:
<div style="display: flex; justify-content: center; align-items: center;">
<video width="1920" height="1080" controls>
  <source src="../assets/A1_End-Effector_Trajectory_Motion.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</div>


##### End-Effector Pose Movement Interface

<table style="border-collapse: collapse;text-align:left">
    <thead>
        <tr style="background-color: black; color: white;">
            <th>Topic Name</th>
            <th>Description</th>
            <th>Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>/a1_ee_target</td>
            <td>Target pose of end arm joint</td>
            <td>Geometry_msgs::PoseStamped</td>
        </tr>
    </tbody>
</table>


<table style="border-collapse: collapse;text-align:left">
    <thead>
        <tr style="background-color: black; color: white;">
            <th>Topic Name</th>
            <th>Field</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>/a1_ee_target</td>
            <td>header</td>
            <td>Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>pose.position.x</td>
            <td>Shift in x direction</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>pose.position.y</td>
            <td>Shift in y direction</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>pose.position.z</td>
            <td>Shift in z direction</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>pose.orientation.x</td>
            <td>Orientation quaternion</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>pose.orientation.y</td>
            <td>Orientation quaternion</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>pose.orientation.z</td>
            <td>Orientation quaternion</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>pose.orientation.w</td>
            <td>Orientation quaternion</td>
        </tr>
    </tbody>
</table>


#### Joint Angle Movement

1. Firstly, initiate the joint angle movement script. This will launch an RViz visualization for A1 robot arm, with the default joint positions set to zero.
```shell
cd release/install
source setup.bash
roslaunch mobiman jointTrackerdemo.launch
```

2. In the file *jointTrackerdemo.launch* :
```shell
<param name="joint_states_sub_topic" value="/joint_states" /> # the /joint_states topic represents the channel for acquiring simulated values, specifically the states of the robot's joints, within a simulation environment.
<param name="joint_command" value="/a1_robot_right/arm_joint_command" /> #the /a1_robot_right/arm_joint_command topic represents the channel for issuing commands to the motors.
```
Publish messages for joint movement on the  `/arm_joint_target_position` topic. This operation is non-blocking, allowing for continuous publishing and enabling uninterrupted movement of the robot arm's joints.
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


##### Joint Position Movement Interface

The `/joint_move` is a ROS package for single-joint control of A1 arms. This package allows you to specify the movement of each joint from its current position to a target position, with configurable maximum speed and acceleration. If these parameters are not specified, default values will be used. The default maximum speed is 20 rad/s, and the default maximum acceleration is 20 rad/s². The topic names and fields of the movement interface are detailed in the following table.

<table style="border-collapse: collapse;text-align:left">
    <thead>
        <tr style="background-color: black; color: white;">
            <th>Topic Name</th>
            <th>Description</th>
            <th>Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>/arm_joint_target_position</td>
            <td>Target position of each joint</td>
            <td>Sensor_msgs/JointState</td>
        </tr>
    </tbody>
</table>


<table style="border-collapse: collapse;text-align:left">
    <thead>
        <tr style="background-color: black; color: white;">
            <th>Topic Name</th>
            <th>Field</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>/arm_joint_target_position</td>
            <td>header</td>
            <td>Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>name</td>
            <td>Name of each joint </td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>position</td>
            <td>Target position of each joint</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>velocity</td>
            <td>Maximum velocity of each joint</td>
        </tr>
        <tr style="background-color: white;">
            <td></td>
            <td>effort</td>
            <td>Maximum acceleration of each joint</td>
        </tr>
    </tbody>
</table>
