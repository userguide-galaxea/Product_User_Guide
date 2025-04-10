# A1XY Software Introduction
## Software Dependency

1. OS Dependency: Ubuntu 20.04 LTS 
2. Middleware Dependency: ROS Noetic

## Obtain Resources

Click any of the following links to obtain A1XY SDK. 

- Baidu Cloud: [https://pan.baidu.com/s/1jVEqjL-r_Ll7bKFB1XxKIw?pwd=a1xy](https://pan.baidu.com/s/1jVEqjL-r_Ll7bKFB1XxKIw?pwd=a1xy)
- Google Drive：[https://drive.google.com/drive/folders/180qSZTc7bZgwklVuwcuhq5O2DPteI2nR?usp=sharing](https://drive.google.com/drive/folders/180qSZTc7bZgwklVuwcuhq5O2DPteI2nR?usp=sharing)

This is the initial software release for the A1XY robot arm. For subsequent version updates, you can refer to the A1XY Software Version Cahngelog to obtain the latest SDK package and update information.

Click the following links to obtain A1XY URDF.

- [A1X URDF](https://github.com/userguide-galaxea/URDF/tree/galaxea/main/A1X)
- [A1Y URDF](https://github.com/userguide-galaxea/URDF/tree/galaxea/main/A1Y)

## Start SDK

Visit the [A1XY Startup and Demo Guide](./A1XY_Startup_Demo_Guide.md).

## Demo

Visit the [A1XY Demo Guide](./A1XY_Startup_Demo_Guide.md/#23-demo).

## Software Interface

### Driver Interface

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">I/O</th>          
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Desciption</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Position, speed, and torque feedback for each joint</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">End-effector gripper stroke feedback</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_status</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                                   
            <td style="padding: 8px; border: 1px solid #ddd;">Status feedback for each joint</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::feedback_status</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                   
            <td style="padding: 8px; border: 1px solid #ddd;">Motor control interface for each joint</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                 
            <td style="padding: 8px; border: 1px solid #ddd;">End-effector gripper motor control interface</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/position_control_gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">End-effector gripper stroke control interface</td>
            <td style="padding: 8px; border: 1px solid #ddd;">std_msgs::Float32</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/arm_node/function_frame_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Robot arm function frame</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg/FunctionFrame</td>
        </tr>        
    </tbody>
</table>

The specific fields and their detailed descriptions for the above topic are shown in the table below:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">Field</th>          
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">Desciption</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td rowspan="4" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/hdas/feedback_arm</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">position</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[Joint1_position, Joint2_position, Joint3_position, Joint4_position, Joint5_position, Joint6_position, gripper_position]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">velocity</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[Joint1_velocity, Joint2_velocity, Joint3_velocity, Joint4_velocity, Joint5_velocity, Joint6_velocity, gripper_velocity]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">effort</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[Joint1_effort, Joint2_effort, Joint3_effort, Joint4_effort, Joint5_effort, Joint6_effort, gripper_effort]</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="4" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/hdas/feedback_gripper</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">position</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[gripper_stroke]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">velocity</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Not used</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">effort</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Not used</td>
        </tr>
                <tr style="background-color: white;">
            <td rowspan="3" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/hdas/feedback_status_arm</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">name_id</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Joint name</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">errors</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Contains error code and desciption</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="8" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/motion_control/control_arm</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">name</td>
            <td style="padding: 10px; border: 1px solid #ddd;">-</td>
        </tr>        
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">p_des</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[Joint1_position, Joint2_position, Joint3_position, Joint4_position, Joint5_position, Joint6_position]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">v_des</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[Joint1_velocity, Joint2_velocity, Joint3_velocity, Joint4_velocity, Joint5_velocity, Joint6_velocity]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">kp</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[Joint1_kp, Joint2_kp, Joint3_kp, Joint4_kp, Joint5_kp, Joint6_kp]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">kd</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[Joint1_kd, Joint2_kd, Joint3_kd, Joint4_kd, Joint5_kd, Joint6_kd]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">t_ff</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[Joint1_effort, Joint2_effort, Joint3_effort, Joint4_effort, Joint5_effort, Joint6_effort]</td>
        </tr>        
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">mode</td>
            <td style="padding: 10px; border: 1px solid #ddd;">-</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="8" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/motion_control/control_gripper</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">name</td>
            <td style="padding: 10px; border: 1px solid #ddd;">-</td>
        </tr>        
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">p_des</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[gripper_position]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">v_des</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[gripper_velocity]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">kp</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[gripper_kp]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">kd</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[gripper_kd]</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">t_ff</td>
            <td style="padding: 10px; border: 1px solid #ddd;">[gripper_effort]</td>
        </tr>        
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">mode</td>
            <td style="padding: 10px; border: 1px solid #ddd;">-</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="2" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/motion_control/position_control_gripper</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">data</td>
            <td style="padding: 10px; border: 1px solid #ddd;">desired_gripper_stroke, range (0 to 100) mm</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/arm_node/function_frame_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">command</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">1: Enable<br>2: Disable<br>3: Whole arm calibration<br>4: Clear error</td>
        </tr>
    </tbody>
</table>


### Motion Control Interface

#### Joint Control

```Bash
source A1XY_workspace/install/setup.bash

# Depending on the product model, select one of the following startup methods:
roslaunch mobiman a1x_jointTrackerdemo.launch
roslaunch mobiman a1y_jointTrackerdemo.launch
```

This launch file will start with the `a1_xy_jointTracker_demo_node`, which is the main node responsible for controlling each joint.

The interface is shown below:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">I/O</th>          
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Desciption</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                                   
            <td style="padding: 8px; border: 1px solid #ddd;">Arm joint feedback</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Hdas_msg::motor_control</td>
        </tr>        
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Arm motor control</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_target/target_joint_state_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Target position of each joint</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
        </tr>
    </tbody>
</table>

The specific fields and their detailed descriptions for the above topic are shown in the table below:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Field</th>          
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">Desciption</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td rowspan="2" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/motion_target/target_joint_state_arm</td>
            <td style="padding: 10px; border: 1px solid #ddd;">position</td>
            <td style="padding: 10px; border: 1px solid #ddd;">This is a vector with six elements representing the target positions of each joint.This is a vector with six elements representing the target positions of each joint.</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">velocity</td>
            <td style="padding: 10px; border: 1px solid #ddd;">This is a vector with six elements representing the maximum velocity of each joint during motion. The maximum velocities are as follows: {3, 3, 3, 5, 5, 5, 5}. The acceleration and jerk limits are set to 1.5 times the velocity limits.</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">-</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Please refer to the arm diver interface.</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">-</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Please refer to the arm diver interface.</td>
        </tr>
    </tbody>
</table>

#### Arm Pose Control

A1XY Arm Pose Control is a ROS package designed to control the movement of the robotic arm to the target end-effector (ee) coordinate frame. It primarily includes a launch file that can be started using the following command:

```Bash
source A1XY_workspace/install/setup.bash

# Depending on the product model, select one of the following startup methods:
roslaunch mobiman a1x_arm_relaxed_ik.launch
roslaunch mobiman a1y_arm_relaxed_ik.launch
```

Note: 

- When the dual-arm pose controller is started, the joint control node must also be launched. This is because the pose controller continuously calculates the target joint angles based on the desired end-effector (ee) pose and sends these target angles to the `/motion_target/target_joint_state_arm` topic.


    ```Bash
    source A1XY_workspace/install/setup.bash

    # Depending on the product model, select one of the following startup methods:
    roslaunch mobiman a1x_jointTrackerdemo.launch
    roslaunch mobiman a1y_jointTrackerdemo.launch
    ```

- The relative pose of the current end-effector pose control is the transformation of the gripper_link relative to the base_link in the URDF. Taking the A1X as an example, the following figure shows the relative relationship of the gripper_link coordinate system to the base_link coordinate system, including the offsets in x, y, and z, as well as the rotational offsets corresponding to the orientation.

  ![A1XY_arm_pose_cn](./assets/A1XY_arm_pose.png)

The interface is shown below:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">I/O</th>          
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Desciption</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/pose_ee_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                                   
            <td style="padding: 8px; border: 1px solid #ddd;">Actual pose of ee</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Hdas_msg::motor_control</td>
        </tr> 
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                                   
            <td style="padding: 8px; border: 1px solid #ddd;">Arm joint feedback</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Hdas_msg::motor_control</td>
        </tr>        
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_target/target_joint_state_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Target of Arm joint </td>
            <td style="padding: 8px; border: 1px solid #ddd;">Sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_target/pose_ee_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Target pose of ee</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Geometry_msgs::PoseStamped</td>
        </tr>
    </tbody>
</table>

The specific fields and their detailed descriptions for the above topic are shown in the table below:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Field</th>          
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">Desciption</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td rowspan="8" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/motion_target/pose_ee_arm</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">Standard Header</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">pose.position.x</td>
            <td style="padding: 10px; border: 1px solid #ddd;">X-axis offset</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">pose.position.y</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Y-axis offset</td>                    
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">pose.position.z</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Z-axis offset</td>                    
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">pose.orientation.x</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Rotation quaternion</td>                    
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">pose.orientation.y</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Rotation quaternion</td>                    
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">pose.orientation.z</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Rotation quaternion</td>                    
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">pose.orientation.w</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Rotation quaternion</td>                    
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">-</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Please refer to the arm diver interface.</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_target/target_joint_state_arm</td>
            <td style="padding: 8px; border: 1px solid #ddd;">-</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Please refer to the joint control interface</td>
        </tr>
    </tbody>
</table>

#### Gripper Control

A1XY Gripper Control is a ROS node designed to control the end-effector gripper. It primarily includes a launch file that can be started using the following command:

```Bash
source A1XY_workspace/install/setup.bash
roslaunch mobiman a1xy_gripperController.launch
```

This launch file will start `a1_xy_jointTracker_demo_node`, which is the main node responsible for controlling each joint.

The interface is shown below:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">I/O</th>          
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Desciption</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_target/target_position_gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                                   
            <td style="padding: 8px; border: 1px solid #ddd;">Target position of gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Sensor_msgs::JointState</td>
        </tr>        
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Motor control of gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Sensor_msgs::JointState</td>
        </tr>
    </tbody>
</table>

The specific fields and their detailed descriptions for the above topic are shown in the table below:

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">Topic Name</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Field</th>          
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">Desciption</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_target/target_position_gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">position</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Indicates the target position of the gripper, ranging from [0, 100], where 0 represents fully closed and 100 represents fully open.</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_gripper</td>
            <td style="padding: 8px; border: 1px solid #ddd;">-</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">Please refer to the driver interface.</td>
        </tr>
    </tbody>
</table>