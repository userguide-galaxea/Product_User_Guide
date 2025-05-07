# R1 Lite 软件介绍

## 环境依赖

1. 硬件依赖：R1 Lite计算单元
2. 操作系统依赖：Ubuntu 20.04 LTS（PC安装双系统，非虚拟机）
3. 中间件依赖：ROS Noetic

## 软件版本日志

当前为R1 Lite的首发软件版本。后续的版本更新您可以在R1 Lite软件版本更新日志中查看，获取最新的SDK包及更新信息。

## 首次操作指引

访问[Galaxea R1 Lite开箱启动指南](./R1Lite_Unbox_Startup_Guide.md)，并按照说明操作R1 Lite。

## 软件接口
### 驱动接口
#### 手臂驱动接口
该接口是用于机械臂控制和状态反馈的ROS软件包，定义了多个话题用于发布和订阅臂的状态、控制命令和相关错误代码。以下是每个话题及其对应消息类型的详细描述：

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;table-layout: fixed;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">I/O</th>          
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">描述</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_left</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">左臂关节反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_right</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                                   
            <td style="padding: 8px; border: 1px solid #ddd;">右臂关节反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_gripper_left</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">左夹爪行程</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_gripper_right</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                                   
            <td style="padding: 8px; border: 1px solid #ddd;">右夹爪行程</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_status_arm_left</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                                   
            <td style="padding: 8px; border: 1px solid #ddd;">左臂状态反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::feedback_status</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_status_arm_right</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">右臂状态反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::feedback_status</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_status_gripper_left</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                
            <td style="padding: 8px; border: 1px solid #ddd;">左夹爪状态反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::feedback_status</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_status_gripper_right</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">右夹爪状态反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::feedback_status</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_left</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                   
            <td style="padding: 8px; border: 1px solid #ddd;">左臂电机控制</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_right</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                     
            <td style="padding: 8px; border: 1px solid #ddd;">右臂电机控制</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_gripper_left</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                 
            <td style="padding: 8px; border: 1px solid #ddd;">左夹爪电机控制</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_gripper_right</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">右夹爪电机控制</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/position_control_gripper_left</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>                    
            <td style="padding: 8px; border: 1px solid #ddd;">左夹爪位置控制</td>
            <td style="padding: 8px; border: 1px solid #ddd;">std_msgs::Float32</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/position_control_gripper_right</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>              
            <td style="padding: 8px; border: 1px solid #ddd;">右夹爪位置控制</td>
            <td style="padding: 8px; border: 1px solid #ddd;">std_msgs::Float32</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：
<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">字段</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd;width: 600px;">描述</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="4">/hdas/feedback_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1位置, 关节2位置, 关节3位置, 关节4位置, 关节5位置, 关节6位置, 夹爪关节位置]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1速度, 关节2速度, 关节3速度, 关节4速度, 关节5速度, 关节6速度, 夹爪关节速度]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1力矩, 关节2力矩, 关节3力矩, 关节4力矩, 关节5力矩, 关节6力矩, 夹爪关节力矩]</td>
<tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="4">/hdas/feedback_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1位置, 关节2位置, 关节3位置, 关节4位置, 关节5位置, 关节6位置, 夹爪关节位置]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1速度, 关节2速度, 关节3速度, 关节4速度, 关节5速度, 关节6速度, 夹爪关节速度]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1力矩, 关节2力矩, 关节3力矩, 关节4力矩, 关节5力矩, 关节6力矩, 夹爪关节力矩]</td>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="4">/hdas/feedback_gripper_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪行程 (0-100mm)</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">暂不使用</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">暂不使用</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="4">/hdas/feedback_gripper_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪行程 (0-100mm)</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">暂不使用</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">暂不使用</td>
    </tr>        
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="3">/hdas/feedback_status_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name_id</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">关节名称</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">errors</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">包含错误码和对应的错误描述</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="3">/hdas/feedback_status_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name_id</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">关节名称</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">errors</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">包含错误码和对应的错误描述</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="3">/hdas/feedback_status_gripper_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name_id</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">关节名称</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">errors</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">包含错误码和对应的错误描述</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="3">/hdas/feedback_status_gripper_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name_id</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">关节名称</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">errors</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">包含错误码和对应的错误描述</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="6">/motion_control/control_arm_left</br>（伺服模式）</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1位置, 关节2位置, 关节3位置, 关节4位置, 关节5位置, 关节6位置, 夹爪关节位置]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1速度最大限制, 关节2速度最大限制, 关节3速度最大限制, 关节4速度最大限制, 关节5速度最大限制, 关节6速度最大限制, 夹爪关节速度最大限制]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1力矩最大限制, 关节2力矩最大限制, 关节3力矩最大限制, 关节4力矩最大限制, 关节5力矩最大限制, 关节6力矩最大限制, 夹爪关节力矩最大限制]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="8">/motion_control/control_arm_left</br>（力位混合控制模式）</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1位置, 关节2位置, 关节3位置, 关节4位置, 关节5位置, 关节6位置, 夹爪关节位置]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1速度, 关节2速度, 关节3速度, 关节4速度, 关节5速度, 关节6速度, 夹爪关节速度]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kp</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1kp, 关节2kp, 关节3kp, 关节4kp, 关节5kp, 关节6kp, 夹爪关节kp]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kd</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1kd, 关节2kd, 关节3kd, 关节4kd, 关节5kd, 关节6kd, 夹爪关节kd]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1力矩, 关节2力矩, 关节3力矩, 关节4力矩, 关节5力矩, 关节6力矩, 夹爪关节力矩]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="6">/motion_control/control_arm_right</br>（伺服模式）</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1位置, 关节2位置, 关节3位置, 关节4位置, 关节5位置, 关节6位置, 夹爪关节位置]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1速度最大限制, 关节2速度最大限制, 关节3速度最大限制, 关节4速度最大限制, 关节5速度最大限制, 关节6速度最大限制, 夹爪关节速度最大限制]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1力矩最大限制, 关节2力矩最大限制, 关节3力矩最大限制, 关节4力矩最大限制, 关节5力矩最大限制, 关节6力矩最大限制, 夹爪关节力矩最大限制]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
     <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="8">/motion_control/control_arm_right</br>（力位混合控制模式）</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1位置, 关节2位置, 关节3位置, 关节4位置, 关节5位置, 关节6位置, 夹爪关节位置]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1速度, 关节2速度, 关节3速度, 关节4速度, 关节5速度, 关节6速度, 夹爪关节速度]</td>
    </tr>
        <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kp</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1kp, 关节2kp, 关节3kp, 关节4kp, 关节5kp, 关节6kp, 夹爪关节kp]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kd</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1kd, 关节2kd, 关节3kd, 关节4kd, 关节5kd, 关节6kd, 夹爪关节kd]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1力矩, 关节2力矩, 关节3力矩, 关节4力矩, 关节5力矩, 关节6力矩, 夹爪关节力矩]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="8">/motion_control/control_gripper_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪电机位置</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪电机速度</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kp</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪电机kp</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kd</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪电机kd</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪力矩</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="8">/motion_control/control_gripper_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪电机位置</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪电机速度</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kp</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪电机kp</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kd</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪电机kd</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪力矩</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="2">/motion_control/position_control_gripper_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">data</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪行程期待值 (0-100mm)</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="2">/motion_control/position_control_gripper_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">data</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">夹爪行程期待值 (0-100mm)</td>
    </tr>
  </tbody>
</table>

#### 躯干驱动接口
该接口是用于躯干控制和提供状态反馈的ROS软件包，它定义了多个话题，用于发布和订阅躯干电机的状态和控制命令。以下是每个话题及其对应消息类型的详细描述：

<table style="width: 100%; border-collapse: collapse;table-layout: fixed;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 33%; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 33%; padding: 8px; border: 1px solid #ddd;">I/O</th>                
            <th style="width: 33%; padding: 8px; border: 1px solid #ddd;">描述</th>
            <th style="width: 33%; padding: 8px; border: 1px solid #ddd;">消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_torso</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>                 
            <td style="padding: 8px; border: 1px solid #ddd;">躯干关节反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs/JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_status_torso</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>    
            <td style="padding: 8px; border: 1px solid #ddd;">躯干状态反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::feedback_status</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_torso</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>    
            <td style="padding: 8px; border: 1px solid #ddd;">躯干电机控制</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：
<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 100px;">字段</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">描述</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="4">/hdas/feedback_torso</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1位置, 关节2位置, 关节3位置, 0]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1速度, 关节2速度, 关节3速度, 0]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1力矩, 关节2力矩, 关节3力矩, 0]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="3">/hdas/feedback_status_torso</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name_id</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">关节名称</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">errors</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">包含错误码和对应的错误描述</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="8">/motion_control/control_torso</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1位置, 关节2位置, 关节3位置, 0]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1速度, 关节2速度, 关节3速度, 0]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kp</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1kp, 关节2kp, 关节3kp, 0]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kd</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1kd, 关节2kd, 关节3kd, 0]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[关节1力矩, 关节2力矩, 关节3力矩, 0]</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>
  </tbody>
</table>

#### 底盘驱动接口
该接口是用于底盘状态反馈的ROS软件包，定义了多个话题以报告底盘电机的状态。以下是每个话题及其相关消息类型的详细描述：

<table style="width: 100%; border-collapse: collapse;table-layout: fixed;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">I/O</th>                
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">描述</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_chassis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">底盘轮毂和转向电机反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/feedback_status_chassis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">底盘状态反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::feedback_status</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/motion_control/control_chassis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Input</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">底盘轮毂和转向电机控制</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：

<table style="width: 100%; border-collapse: collapse;">
  </thead>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">字段</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">描述</th>
    </tr>
  </thead>
  <tbody>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="4">/hdas/feedback_chassis</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[左前轮角度, 右前轮角度, 后轮角度]</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[左前轮线速度, 右前轮线速度, 后轮线速度]<br>[0.0, 0.0, 0.0]</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">effort</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="3">/hdas/feedback_status_chassis</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name_id</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">关节名称</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">errors</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">包含错误码和对应的错误描述</td>
  </tr>
<tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="8">/motion_control/control_chassis</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">name</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">p_des</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[左前轮角度, 右前轮角度, 后轮角度]</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">v_des</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">[左前轮线速度, 右前轮线速度, 后轮线速度]</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kp</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">kd</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
  </tr>    
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">t_ff</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
  </tr> 
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">mode</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
    </tr>  
   </tbody>
</table>

#### 相机接口
<table style="width: 100%; border-collapse: collapse;table-layout: fixed;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">I/O</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">描述</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/camera_wrist_left/color/image_raw/compressed</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">右腕相机RGB压缩图（如存在）</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::CompressedImage</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/camera_wrist_right/color/image_raw/compressed</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">左腕相机RGB压缩图（如存在）</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::CompressedImage</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/camera_wrist_left/aligned_depth_to_color/image_raw</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">左腕相机深度图（如存在）</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::Image</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/camera_wrist_right/aligned_depth_to_color/image_raw</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">右腕相机深度图（如存在）</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::Image</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：
<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">字段</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">描述</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="3">/hdas/camera_wrist_left/color/image_raw/compressed</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
      <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">format</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">JPEG</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">data</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">图像数据</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="3">/hdas/camera_wrist_right/color/image_raw/compressed</td>
        <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
      <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">format</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">JPEG</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">data</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">图像数据</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="7">/hdas/camera_wrist_left/aligned_depth_to_color/image_raw</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">height</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">取决于设置</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">width</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">取决于设置</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">encoding</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">16UC1</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">is_bigendian</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">0</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">step</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">取决于设置</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">data</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">图像数据</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="7">/hdas/camera_wrist_right/aligned_depth_to_color/image_raw</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">height</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">取决于设置</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">width</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">取决于设置</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">encoding</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">16UC1</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">is_bigendian</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">0</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">step</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">根据配置决定</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">data</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">图像数据</td>
    </tr>
  </tbody>
</table>

#### IMU 接口
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">I/O</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">描述</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/imu_chassis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>
            <td style="padding: 8px; border: 1px solid #ddd;">底盘IMU反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::Imu</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/imu_torso</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>   
            <td style="padding: 8px; border: 1px solid #ddd;">躯干IMU反馈</td>
            <td style="padding: 8px; border: 1px solid #ddd;">sensor_msgs::Imu</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">字段</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">描述</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;" rowspan="11">/hdas/imu_chassis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="padding: 8px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">orientation.x</td>
            <td style="padding: 8px; border: 1px solid #ddd;">四元数x</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">orientation.y</td>
            <td style="padding: 8px; border: 1px solid #ddd;">四元数y</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">orientation.z</td>
            <td style="padding: 8px; border: 1px solid #ddd;">四元数z</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">orientation.w</td>
            <td style="padding: 8px; border: 1px solid #ddd;">四元数w</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">angular_velocity.x</td>
            <td style="padding: 8px; border: 1px solid #ddd;">陀螺仪角速度x</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">angular_velocity.y</td>
            <td style="padding: 8px; border: 1px solid #ddd;">陀螺仪角速度y</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">angular_velocity.z</td>
            <td style="padding: 8px; border: 1px solid #ddd;">陀螺仪角速度z</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">linear_acceleration.x</td>
            <td style="padding: 8px; border: 1px solid #ddd;">线加速度x</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">linear_acceleration.y</td>
            <td style="padding: 8px; border: 1px solid #ddd;">线加速度y</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">linear_acceleration.z</td>
            <td style="padding: 8px; border: 1px solid #ddd;">线加速度z</td>
        </tr>
        <!-- Repeat the structure for /hdas/imu_torso -->
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;" rowspan="11">/hdas/imu_torso</td>
            <td style="padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="padding: 8px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">orientation.x</td>
            <td style="padding: 8px; border: 1px solid #ddd;">四元数x</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">orientation.y</td>
            <td style="padding: 8px; border: 1px solid #ddd;">四元数y</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">orientation.z</td>
            <td style="padding: 8px; border: 1px solid #ddd;">四元数z</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">orientation.w</td>
            <td style="padding: 8px; border: 1px solid #ddd;">四元数w</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">angular_velocity.x</td>
            <td style="padding: 8px; border: 1px solid #ddd;">陀螺仪角速度x</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">angular_velocity.y</td>
            <td style="padding: 8px; border: 1px solid #ddd;">陀螺仪角速度y</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">angular_velocity.z</td>
            <td style="padding: 8px; border: 1px solid #ddd;">陀螺仪角速度z</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">linear_acceleration.x</td>
            <td style="padding: 8px; border: 1px solid #ddd;">线加速度x</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">linear_acceleration.y</td>
            <td style="padding: 8px; border: 1px solid #ddd;">线加速度y</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">linear_acceleration.z</td>
            <td style="padding: 8px; border: 1px solid #ddd;">线加速度z</td>
        </tr>
    </tbody>
</table>

#### BMS 接口
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">I/O</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">描述</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/hdas/bms</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>
            <td style="padding: 8px; border: 1px solid #ddd;">电池BMS消息</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::bms</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">字段</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">描述</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;" rowspan="4">/hdas/bms</td>
            <td style="padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="padding: 8px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">voltage</td>
            <td style="padding: 8px; border: 1px solid #ddd;">电压 (V)</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">current</td>
            <td style="padding: 8px; border: 1px solid #ddd;">电流 (I)</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">capital</td>
            <td style="padding: 8px; border: 1px solid #ddd;">电量剩余 (%)</td>
        </tr>
    </tbody>
</table>

#### 遥控器接口
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">I/O</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">描述</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">消息类型</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">/controller</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Output</td>
            <td style="padding: 8px; border: 1px solid #ddd;">遥控器信号</td>
            <td style="padding: 8px; border: 1px solid #ddd;">hdas_msg::controller_signal_stamped</td>
        </tr>
    </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">话题名称</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">字段</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">描述</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;" rowspan="6">/controller</td>
            <td style="padding: 8px; border: 1px solid #ddd;">header</td>
            <td style="padding: 8px; border: 1px solid #ddd;">标准消息头</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">data.left_x_axis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">左摇杆x方向</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">data.left_y_axis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">左摇杆y方向</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">data.right_x_axis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">右摇杆x方向</td>
        </tr>
         <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">data.right_y_axis</td>
            <td style="padding: 8px; border: 1px solid #ddd;">右摇杆y方向</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">mode</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2: 遥控器控制 (底盘)</br>5: 上位机控制 (底盘)</td>
        </tr>
    </tbody>
</table>

### 运动控制接口

- 手臂关节控制：该节点负责控制手臂的每个关节。
- 手臂姿态控制：该节点负责控制手臂移动到目标末端执行器（ee）坐标帧，分为左臂的姿态控制和右臂的姿态控制。
- 躯干速度控制：该节点负责控制躯干移动到目标浮动基座坐标帧。
- 底盘控制：该节点使用矢量控制来控制R1 Lite底盘，可同时发送三个方向的速度命令：x、y 和w。

#### 手臂关节控制

手臂关节控制节点负责控制手臂的每个关节，总共有6个关节。可通过以下命令启动：
```Bash
# 以下两个launch选择一个启动，本质是关节速度限制参数不一样
source ~/work/galaxea/install/setup.bash
roslaunch mobiman r1_lite_jointTrackerdemo.launch                #（正常模式）         
roslaunch mobiman r1_lite_jointTrackerdemo_fast.launch           #（高跟随模式）
```

接口信息如下：
<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">Topic Name</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 100px;">I/O</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">Description</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">Message Type</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/target_joint_state_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">左臂各关节的目标位置</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/target_joint_state_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">右臂各关节的目标位置</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">左臂关节反馈</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">右臂关节反馈</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Output</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">左臂电机控制</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Output</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">右臂电机控制</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
    </tr>
  </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">字段</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 500px;">描述</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="3">/motion_target/target_joint_state_arm_left <br>/motion_target/target_joint_state_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">position</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">这是一个包含六个元素的向量，代表每个关节的六个目标位置。</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">velocity</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">这是一个包含六个元素的向量，代表每个关节在运动过程中的最大速度。</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考手臂驱动接口</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考手臂驱动接口</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考手臂驱动接口</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考手臂驱动接口</td>
    </tr>
  </tbody>
</table>

#### 手臂姿态控制

手臂姿态控制是一个用于控制手臂移动到目标末端执行器（ee）坐标帧的ROS软件包。它主要包括两个launch文件,分别对应 左臂的姿态控制和右臂的姿态控制。可通过以下命令启动:

```Bash
source ~/work/galaxea/install/setup.bash
roslaunch mobiman R1_Lite_left_arm_relaxed_ik.launch  #启动左手
roslaunch mobiman R1_Lite_right_arm_relaxed_ik.launch #启动右手
```

接口信息如下：

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 100px;">I/O</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 500px;">描述</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">消息类型</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/target_pose_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">目标左臂末端执行器姿态</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Geometry_msgs::PoseStamped</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/target_pose_arm_right </td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>        
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">目标右臂末端执行器姿态</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Geometry_msgs::PoseStamped</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>   
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">左臂关节反馈</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Sensor_msgs::JointState</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>   
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">右臂关节反馈</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Sensor_msgs::JointState</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Output</td>   
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">左臂关节控制</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Output</td>   
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">右臂关节控制</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
    </tr>
  </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 500px;">字段</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 500px;">描述</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
        <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="8">/motion_target/pose_ee_arm_left</br>/motion_target/pose_ee_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.x</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">X轴偏移</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.y</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Y轴偏移</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.position.z</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Z轴偏移</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.x</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">旋转四元数</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.y</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">旋转四元数</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.z</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">旋转四元数</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">pose.orientation.w</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">旋转四元数</td>
    </tr>      
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考手臂驱动接口</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_arm_right</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考手臂驱动接口</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_left</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考手臂驱动接口</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_arm_right </td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考手臂驱动接口</td>
    </tr>
  </tbody>
</table>

#### 躯干速度控制

R1 Lite躯干速度控制是一个用于控制躯干移动到目标浮动基座坐标帧的ROS软件包。可通过以下命令启动：

```Bash
source ~/work/galaxea/install/setup.bash
roslaunch mobiman torso_speed_control_example.launch
```

接口信息如下：

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 100px;">I/O</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 500px;">描述</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">消息类型</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/target_speed_torso</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input </td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">浮动基座框架的目标速度</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Geometry_msgs::Twist::ConstPtr</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_torso</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input </td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">躯干关节反馈</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Sensor_msgs::JointState</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_torso</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Output </td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">躯干关节控制</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
    </tr>
  </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">字段</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">描述</th>
    </tr>
  </thead>
  <tbody>   
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="4">/motion_target/target_speed_torso</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">target_speed.v_x = msg->linear.x</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">躯干笛卡尔空间x方向线速度，[-0.2,0.2]m/s；<br>-0.1≤x≤0.2</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">target_speed.v_z = msg->linear.z</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">躯干笛卡尔空间z方向线速度，[-0.2,0.2]m/s；<br>0.1≤z≤0.7</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">target_speed.v_pitch = msg->twist.angular.y</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">躯干笛卡尔空间pitch角速度</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">target_speed.v_yaw = msg->twist.angular.z</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">躯干笛卡尔空间yaw角速度</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_torso</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考躯干驱动接口</td>
    </tr>
  </tbody>
</table>

#### 底盘控制

底盘控制是一个使用矢量控制来控制R1 Lite底盘的节点，可同时发送三个方向的速度命令：x、y 和w。可通过以下命令启动：

```Bash
source ~/work/galaxea/install/setup.bash
roslaunch mobiman r1_lite_chassis_control_w_o_eepose.launch
```

该文件将启动一个节点：`r1_lite_chassis_control_node`，负责R1 Lite底盘的速度控制。其接口如下所示：

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">话题名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 100px;">I/O</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">描述</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">消息类型</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/target_speed_chassis</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">底盘的目标速度，包括vx, vy and omega.</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">geometry_msgs::Twist</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/chassis_acc_limit</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">底盘的加速度限制，最大值分别为2.5, 1.0, 1.0</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">geometry_msgs::Twist</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/brake_mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">发出底盘是否进入制动模式的指令。如果处于制动模式，当速度为0时，底盘将通过将车轮转动一定角度来锁定自身。</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">std_msgs::Bool</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_chassis</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Input</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">R1 Lite底盘控制订阅此话题并控制底盘的目标速度。</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">sensor_msgs::JointState</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_chassis</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Output</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">R1 Lite底盘控制发布此话题以控制电机。</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">hdas_msg::motor_control</td>
    </tr>
  </tbody>
</table>

针对以上话题的具体字段及其详细描述如下表所示：

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">消息名称</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">字段</th>
      <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 500px;">描述</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="6">/motion_target/target_speed_chassis</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">linear</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">线速度</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.x</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">线速度x, 范围 (-1.5, 1.5) m/s</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.y</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">线速度y, 范围 (-1.5, 1.5) m/s</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">angular</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">角速度</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.z</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">角速度, 范围  (-3 - 3) rad/s</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;" rowspan="6">/motion_target/chassis_acc_limit</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">header</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">标准消息头</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">linear</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">线速度</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.x</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">加速度限制x, 范围 (-2.5, 2.5) m/s²</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.y</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">加速度限制y, 范围 (-1.0, 1.0) m/s²</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">angular</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">角速度</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.z</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">角速度限制, 范围 (-3, 3) rad/s²</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_target/brake_mode</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">data</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">布尔值<br>进入刹车模式：True<br>退出刹车模式：False</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/hdas/feedback_chassis</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考底盘驱动接口</td>
    </tr>
    <tr style="background-color: white;">
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">/motion_control/control_chassis</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">-</td>
      <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">请参考底盘驱动接口</td>
    </tr>
  </tbody>
</table>