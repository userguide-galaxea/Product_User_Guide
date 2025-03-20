# Galaxea A1 Software Guide
>本教程将指导您如何开发和操作Galaxea A1。

## 软件依赖
1. Ubuntu 20.04 LTS
2. ROS Noetic

## 获取SDK
点击[此处](https://github.com/userguide-galaxea/A1_SDK)可在我们的GitHub社区获取SDK及其他信息。SDK无需重新编译。请参考开发与操作教程直接使用。或者，使用以下任一方式获取SDK：

- 百度云盘：[https://pan.baidu.com/s/1w-zctmpHBfk_Sqm2uihAaA?pwd=arm1](https://pan.baidu.com/s/1w-zctmpHBfk_Sqm2uihAaA?pwd=arm1)

- Google Drive: [https://drive.google.com/drive/folders/12oYeylWJWcaRDKeD2qG7xQNI7rFpBbel?usp=sharing](https://drive.google.com/drive/folders/12oYeylWJWcaRDKeD2qG7xQNI7rFpBbel?usp=sharing)

## 开发与操作教程
### 初次使用设置
1. 确认电源和USB连接后，运行以下命令修改串口文件的读写权限：
    ```Bash
    sudo chmod 777 /dev/ttyACM0
    ```
2. 确认修改后，初始化SDK：
    ```Bash
    cd A1_SDK/install
    source setup.bash
    roslaunch signal_arm single_arm_node.launch
    ```
    接口部分描述了A1机械臂的各种控制和状态反馈接口，帮助用户了解如何通过ROS包连接和控制机械臂。

### Demo演示
点击[此处](https://github.com/userguide-galaxea/A1_SDK/tree/galaxea/main/resource)获取A1 Demo脚本。

```Bash
cd A1_SDK/install
source setup.bash
roslaunch mobiman eeTrackerdemo.launch
```
运行以下命令以控制末端执行器运动：
```Bash
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
## 软件接口
### 驱动接口
该接口是一个用于机械臂控制和状态反馈的ROS包，定义了多个话题用于发布和订阅臂的状态、控制命令和相关错误代码。以下是每个话题及其相关消息类型的详细描述：
<table style="border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white;text-align: left;">
            <th>话题名称</th>
            <th>描述</th>
	    <th>消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;text-align: left;">
            <td>/joint_states_host</td>
            <td>机械臂关节状态反馈</td>
			<td>sensor_msgs/JointState</td>
        </tr>
        <tr style="background-color: white;text-align: left;">
            <td>/arm_status_host</td>
            <td>机械臂电机状态反馈</td>
			<td>signal_arm/arm_status</td>
        </tr>
        <tr style="background-color: white;text-align: left;">
            <td>/arm_joint_command_host</td>
            <td>机械臂控制接口</td>
			<td>signal_arm/arm_control</td>
        </tr>
        <tr style="background-color: white;text-align: left;">
            <td>/gripper_force_control_host</td>
            <td>夹爪力控制接口</td>
			<td>signal_arm/gripper_joint_command</td>
        </tr>
        <tr style="background-color: white;text-align: left;">
            <td>/gripper_position_control_host</td>
            <td>夹爪位置控制接口</td>
			<td>signal_arm/gripper_position_control</td>
        </tr>
        <tr style="background-color: white;text-align: left;">
            <td>/gripper_stroke_host</td>
            <td>夹爪行程反馈接口</td>
			<td>sensor_msgs/JointState</td>
        </tr>
    </tbody>
</table>
<table style="border-collapse: collapse; width: 100%;">
    <thead>
        <tr tr style="background-color: black; color: white;text-align: left;">
            <th style="padding: 10px; border: 1px solid #ddd;">话题名称</th>
            <th style="padding: 10px; border: 1px solid #ddd;">字段</th>
            <th style="padding: 10px; border: 1px solid #ddd;">描述</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td rowspan="5" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/joint_states_host</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">name</td>
            <td style="padding: 10px; border: 1px solid #ddd;">关节名称</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">position</td>
            <td style="padding: 10px; border: 1px solid #ddd;">关节位置</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">velocity</td>
            <td style="padding: 10px; border: 1px solid #ddd;">关节速度</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">effort</td>
            <td style="padding: 10px; border: 1px solid #ddd;">关节力矩</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="4" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/arm_status_host</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">data.name</td>
            <td style="padding: 10px; border: 1px solid #ddd;">关节名称</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">data.motor_errors.error_code</td>
            <td style="padding: 10px; border: 1px solid #ddd;">关节错误码</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">data.motor_errors.error_description</td>
            <td style="padding: 10px; border: 1px solid #ddd;">关节错误描述</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="7" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/arm_joint_command_host</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">p_des</td>
            <td style="padding: 10px; border: 1px solid #ddd;">目标关节位置</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">v_des</td>
            <td style="padding: 10px; border: 1px solid #ddd;">目标关节速度</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">t_ff</td>
            <td style="padding: 10px; border: 1px solid #ddd;">目标关节力矩</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">kp</td>
            <td style="padding: 10px; border: 1px solid #ddd;">目标关节kp</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">kd</td>
            <td style="padding: 10px; border: 1px solid #ddd;">目标关节kd</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">mode</td>
            <td style="padding: 10px; border: 1px solid #ddd;">控制模式</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="2" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/gripper_force_control_host</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">gripper_force</td>
            <td style="padding: 10px; border: 1px solid #ddd;">夹持力</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="2" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/gripper_position_control_host</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">gripper_stroke</td>
            <td style="padding: 10px; border: 1px solid #ddd;">目标行程</td>
        </tr>
        <tr style="background-color: white;">
            <td rowspan="5" style="padding: 10px; border: 1px solid #ddd; vertical-align: middle;">/gripper_stroke_host</td>
            <td style="padding: 10px; border: 1px solid #ddd;">header</td>
            <td style="padding: 10px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">name</td>
            <td style="padding: 10px; border: 1px solid #ddd;">夹爪名称</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">position</td>
            <td style="padding: 10px; border: 1px solid #ddd;">夹爪行程</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">velocity</td>
            <td style="padding: 10px; border: 1px solid #ddd;">夹爪速度</td>
        </tr>
        <tr style="background-color: white;">
            <td style="padding: 10px; border: 1px solid #ddd;">effort</td>
            <td style="padding: 10px; border: 1px solid #ddd;">夹爪力矩</td>
        </tr>
    </tbody>
</table>

#### 夹爪控制示例
1. **夹爪力控制接口**
    ```Bash
    # 控制夹爪到指定的力:
        # gripper_force为正值：闭合夹爪；
        # gripper_force为负值：打开夹爪。
    rostopic pub /gripper_force_control_host signal_arm/gripper_joint_command "header:
        seq: 0
        stamp:
        secs: 0
        nsecs: 0
        frame_id: ''
    gripper_force: 10.0"
    ```
2. **夹爪位置控制接口**
    ```Bash
    # 控制夹爪到指定位置:
        # 60为打开夹爪；
        # 0为闭合夹爪。
    rostopic pub /gripper_position_control_host signal_arm/gripper_position_control "header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    gripper_stroke: 40.0"
    ```
#### 诊断故障代码
DTC用于反馈MCU和驱动器的错误信息，可用于查看每个电机的实时状态和驱动器的运行状态。以下是每个故障代码及其对应状态的详细描述：

<table style="border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white;text-align:left">
            <th>DTC</th>
            <th>状态</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>0</td>
            <td>禁用</td>
        </tr>
        <tr style="background-color: white;">
            <td>1</td>
            <td>禁用</td>
        </tr>
        <tr style="background-color: white;">
            <td>2</td>
            <td>电机断开</td>
        </tr>
        <tr style="background-color: white;">
            <td>3</td>
            <td>位置跳跃</td>
        </tr>
        <tr style="background-color: white;">
            <td>4</td>
            <td>持续高电流</td>
        </tr>
        <tr style="background-color: white;">
            <td>5</td>
            <td>扭矩过大</td>
        </tr>
        <tr style="background-color: white;">
            <td>6</td>
            <td>ECU -> MCU 超时</td>
        </tr>
        <tr style="background-color: white;">
            <td>7</td>
            <td>超出位置限制</td>
        </tr>
        <tr style="background-color: white;">
            <td>8</td>
            <td>超出速度限制</td>
        </tr>
        <tr style="background-color: white;">
            <td>9</td>
            <td>扭矩限制超出</td>
        </tr>
        <tr style="background-color: white;">
            <td>10</td>
            <td>过压</td>
        </tr>
        <tr style="background-color: white;">
            <td>11</td>
            <td>低压</td>
        </tr>
        <tr style="background-color: white;">
            <td>12</td>
            <td>过电流</td>
        </tr>
        <tr style="background-color: white;">
            <td>13</td>
            <td>MOS 过温</td>
        </tr>
        <tr style="background-color: white;">
            <td>14</td>
            <td>电机绕组过温</td>
        </tr>
        <tr style="background-color: white;">
            <td>15</td>
            <td>通信丢失</td>
        </tr>
        <tr style="background-color: white;">
            <td>16</td>
            <td>过载</td>
        </tr>
        <tr style="background-color: white;">
            <td>17</td>
            <td>串行连接断开（无设备文件）</td>
        </tr>
        <tr style="background-color: white;">
            <td>18</td>
            <td>设备文件已连接，无消息</td>
        </tr>
        <tr style="background-color: white;">
            <td>19</td>
            <td>串行读写失败</td>
        </tr>
        <tr style="background-color: white;">
            <td>20</td>
            <td>反馈接收溢出</td>
        </tr>
    </tbody>
</table>


### 运动控制接口
Galaxea A1 提供关节和末端执行器运动控制接口，通过ROS框架实现高效控制。在执行末端执行器或关节运动之前，激活`signal_arm`接口。

- **末端执行器姿态运动**：允许通过发布目标姿态消息来控制Galaxea A1的末端执行器的位置和方向。该功能适用于需要精确定位的应用。
- **末端执行器轨迹运动**：通过发布一系列姿态消息，实现Galaxea A1末端执行器沿指定轨迹的运动。该功能适用于复杂的路径规划和执行。
- **关节角度运动**：提供一个关节级别的控制接口，可为每个关节设置目标位置，实现协调的整个手臂运动。

#### 末端执行器姿态运动

1. 启动末端执行器姿态运动脚本。启动RViz可视化，默认关节位置设置为零。
    ```Bash
    cd A1_SDK/install
    source setup.bash
    roslaunch mobiman eeTrackerdemo.launch
    ```

2. 在文件`eeTrackerdemo.launch`中，执行以下命令：
    ```Bash
    <param name="joint_states_topic" value="/joint_states" /> # the topic /joint_states  represents the channel for acquiring simulated values, specifically the states of the robot's joints, within a simulation environment.
    <param name="arm_joint_command_topic" value="/arm_joint_command_host" /> # the topic /arm_joint_command_host topic represents the channel for issuing commands to the motors.
    ```

3. 在`/a1_ee_target`话题上发布消息以控制末端执行器运动。此操作是非阻塞的，允许连续发布消息，实现末端执行器的无缝运动。但是，目标端点不要离末端执行器的当前位置太远，以避免过度拉伸机械结构或碰撞风险。
    ```Bash
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
        # Create PoseStamped message
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
        # Publish message
        pose_pub.publish(pose_msg)
        rospy.loginfo("Published PoseStamped message to /a1_ee_target")
    
    if __name__ == '__main__':
        try:
            publish_pose()
        except rospy.ROSInterruptException:
            pass
    ```

4. Demo示例：

    <div style="display: flex; justify-content: center; align-items: center;">
    <video width="1920" height="1080" controls>
      <source src="../assets/A1_End-Effector_Motion.mp4" type="video/mp4">
      Your browser does not support the video tag.
    </video>
    </div>

#### 末端执行器轨迹运动

1. 启动末端执行器轨迹运动脚本。启动RViz可视化，默认关节位置设置为零。
    ```Bash
    cd A1_SDK/install
    source setup.bash
    roslaunch mobiman eeTrajTrackerdemo.launch
    ```

2. 在文件`eeTrajTrackerdemo.launch`中，执行以下命令：
    ```Bash
    <param name="joint_states_topic" value="/joint_states" /> # the /joint_states topic represents the channel for acquiring simulated values, specifically the states of the robot's joints, within a simulation environment.
    <param name="joint_command" value="/arm_joint_command_host" /> #the /arm_joint_command_host topic represents the channel for issuing commands to the motors.
    ```

3. 在`/arm_target_trajectory`话题上发布消息，为末端执行器运动指定轨迹。此操作是非阻塞的，允许连续发布。确保轨迹不会显著偏离当前末端执行器的位置。建议在发送下一个轨迹之前等待当前轨迹完成，以避免跟踪所需路径时不准确。
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

4. Demo示例:

    <div style="display: flex; justify-content: center; align-items: center;">
    <video width="1920" height="1080" controls>
      <source src="../assets/A1_End-Effector_Trajectory_Motion.mp4" type="video/mp4">
      Your browser does not support the video tag.
    </video>
    </div>

##### 末端执行器姿态运动接口

<table style="border-collapse: collapse;text-align:left">
    <thead>
        <tr style="background-color: black; color: white;">
            <th>话题名称</th>
            <th>描述</th>
            <th>消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>/a1_ee_target</td>
            <td>臂末端关节的目标姿态</td>
            <td>Geometry_msgs::PoseStamped</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：
<table style="border-collapse: collapse; text-align: left; width: 100%;">
    <tr style="background-color: black; color: white;">
        <th style="padding: 10px; border: 1px solid #ddd;">话题名称</th>
        <th style="padding: 10px; border: 1px solid #ddd;">字段</th>
        <th style="padding: 10px; border: 1px solid #ddd;">描述</th>
    </tr>
    <tr style="background-color: white;">
        <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="8">/a1_ee_target</td>
        <td style="padding: 10px; border: 1px solid #ddd;">header</td>
        <td style="padding: 10px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
        <td style="padding: 10px; border: 1px solid #ddd;">pose.position.x</td>
        <td style="padding: 10px; border: 1px solid #ddd;">x轴偏移</td>
    </tr>
    <tr style="background-color: white;">
        <td style="padding: 10px; border: 1px solid #ddd;">pose.position.y</td>
        <td style="padding: 10px; border: 1px solid #ddd;">y轴偏移</td>
    </tr>
    <tr style="background-color: white;">
        <td style="padding: 10px; border: 1px solid #ddd;">pose.position.z</td>
        <td style="padding: 10px; border: 1px solid #ddd;">z轴偏移</td>
    </tr>
    <tr style="background-color: white;">
        <td style="padding: 10px; border: 1px solid #ddd;">pose.orientation.x</td>
        <td style="padding: 10px; border: 1px solid #ddd;">旋转四元数</td>
    </tr>
    <tr style="background-color: white;">
        <td style="padding: 10px; border: 1px solid #ddd;">pose.orientation.y</td>
        <td style="padding: 10px; border: 1px solid #ddd;">旋转四元数</td>
    </tr>
    <tr style="background-color: white;">
        <td style="padding: 10px; border: 1px solid #ddd;">pose.orientation.z</td>
        <td style="padding: 10px; border: 1px solid #ddd;">旋转四元数</td>
    </tr>
    <tr style="background-color: white;">
        <td style="padding: 10px; border: 1px solid #ddd;">pose.orientation.w</td>
        <td style="padding: 10px; border: 1px solid #ddd;">旋转四元数</td>
    </tr>
</table>

#### 关节角度运动

1. 启动关节角度运动脚本。启动RViz可视化，默认关节位置设置为零。
    ```Bash
    cd A1_SDK/install
    source setup.bash
    roslaunch mobiman jointTrackerdemo.launch
    ```

2. 在文件`jointTrackerdemo.launch`中，执行以下命令：
    ```Bash
    <param name="joint_states_sub_topic" value="/joint_states" /> # the /joint_states topic represents the channel for acquiring simulated values, specifically the states of the robot's joints, within a simulation environment.
    <param name="joint_command" value="/arm_joint_command_host" /> #the /arm_joint_command_host topic represents the channel for issuing commands to the motors.
    ```

3. 在`/arm_joint_target_position`话题上发布关节运动的消息。此操作是非阻塞的，允许连续发布，使关节运动不间断。
    ``` Python
    import rospy
    from sensor_msgs.msg import JointState

    def publish_joint_state():
        rospy.init_node('joint_state_publisher', anonymous=True)
        pub = rospy.Publisher('/arm_joint_target_position', JointState, queue_size=10)

        rate = rospy.Rate(10)  # 1 Hz
        joint_state = JointState()
        joint_state.header.seq = 0
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = 'world'
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state.velocity = []
        joint_state.effort = []
        joint_state.position = [0, 0, 0, 0, 0, 0]  # 初始化位置
        steps = 100 # 设定步数
        target_position = [0.5, 0, 0, 0, 0, 0]  # 目标位置

        # 计算每个关节的增量
        step_increment = [(target - current) / steps for target, current in zip(target_position, joint_state.position)]
        rospy.loginfo(f"Step increment: {step_increment}")

        # **等待发布器建立连接**
        rospy.loginfo("Waiting for subscribers to connect...")
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)  # 100ms 等待

        rospy.loginfo("Subscribers connected, starting publishing...")

        # **开始发布 JointState**
        for step in range(steps):
            joint_state.header.stamp = rospy.Time.now()  # 更新时间戳
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

##### 关节位置运动接口

`/joint_move`是一个用于Galaxea A1单关节控制的ROS包。可指定每个关节从当前位置移动到目标位置，可配置最大速度和加速度。如果参数没有指定，默认值将被使用。默认最大速度是20 rad/s，默认最大加速度是20 rad/s²。运动接口的话题名称和字段如下表详细说明。

<table style="border-collapse: collapse;text-align:left">
    <thead>
        <tr style="background-color: black; color: white;">
            <th>话题名称</th>
            <th>描述</th>
            <th>消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white;">
            <td>/arm_joint_target_position</td>
            <td>每个关节的目标位置</td>
            <td>Sensor_msgs/JointState</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：
<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">话题名称</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">字段</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 10px; border: 1px solid #ddd;">描述</th>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;" rowspan="4">/arm_joint_target_position</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">标准消息头</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">name</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">每个关节的名称</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">position</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">每个关节的目标位置</td>
  </tr>
  <tr>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">velocity</td>
    <td style="vertical-align: middle; padding: 10px; border: 1px solid #ddd;">每个关节的最大速度</td>
  </tr>
</table>