# R1 Demo演示指南
## 准备工作
在开始测试之前，请确保完成以下事项：

1. R1的双臂已准确安装完毕，此时手臂姿态应为肘关节(J3&J4)向外伸展，夹爪垂直朝下指向地面。
2. R1的躯干已保持站立姿态。
3. 以R1为中心，方圆 1.5 米范围内不得有人员以及障碍物存在。（可参考如下图所示内容。）

![R1_Pro](assets/R1_Pro.png)

**请访问GitHub仓库，下载R1 Demo脚本`r1_demo_easy.py`。**

```Bash
git clone https://github.com/userguide-galaxea/Demo.git
#r1_demo_easy.py is located in path ${your_work_path}/Demo/demo_r1.
```

## 动作描述
<span style="color:red">**重要提示：为了您的安全，在测试过程中遇到危险时，请务必在运行 “r1_demo_easy.py" 的 Python 3终端中输入 `Ctrl + C`，以停止正在执行的动作。**</span >

- **arm_test_1：**双臂向上垂直抬起，接着放下双臂，夹爪垂直向下指向地面。
<p style="text-align: center;">
<video src="../assets/R1_arm_test_1.mp4" style="width: 95%;"autoplay muted loop>
   </p>

- **arm_test_2:** 双臂从身体两侧向上抬起，并在胸腔两侧收拢折叠。（此时 R1 的姿态与 URDF中 的零点姿态（默认状态）一致。）
<p style="text-align: center;">
      <video src="../assets/R1_arm_test_2.mp4" style="width: 95%;"autoplay muted loop>
   </p>

- **arm_test_3:**  先放下双臂，让夹爪垂直向下指向地面；随后，从躯干两侧将双臂垂直向上抬起 90°；接着，双臂从两侧向中间水平向前平移 90°；之后，双臂分别从两侧向上抬起，举过头顶并做出爱心形状；最后，双臂分别从两侧放下，夹爪再次垂直向下指向地面。

<p style="text-align: center;">
      <video src="../assets/R1_arm_test_3.mp4" style="width: 95%;"autoplay muted loop>
   </p>
- **torso_test_1: **躯干弯曲下蹲，同时双臂在胸腔两侧收拢折叠。（该动作用于测试 T1/T2/T3 躯干电机。）

<p style="text-align: center;">
      <video src="../assets/torso_test_1.mp4" style="width: 95%;"autoplay muted loop>
   </p>

- **torso_test_2:** 躯干升起站立，同时手臂在躯干两侧放下，接着腰部向左转动 45°，再向右转动 90°，然后向左转动 45°，最后使躯干回正。（此动作用于测试 T4 躯干电机。）

<p style="text-align: center;">
      <video src="../assets/torso_test_2.mp4" style="width: 95%;"autoplay muted loop>
   </p>


## 脚本执行
<span style="color:red">**重要提示：请严格按照顺序进行测试。**</span>

**只有在上述准备工作完成后，才能使用机器人开始测试。**

<span style="color:red">**重要提示：如果出现任何错误，请及时联系我们提供技术支持。**</span>

1.停止后台所有运行中的TMUX，并关闭所有ROS程序。
   ```Bash
   sudo tmux kill-server
   tmux kill-server
   pkill -9 ros
   ```

2.启动FDCAN通信。
   ```Bash
   sudo ip link set dev can0 type can bitrate 1000000 dbitrate 5000000 fd on
   # 若出现“RTNETLINK answers: Device or resource busy”这个错误信息，通常表示你尝试配置的设备（如网络接口或CAN收发器）已经被配置并且正在运行中。
   sudo ip link set up can0
   ```

3.进入TMUX。
   ```Bash
   tmux
   ```

4.启动roscore。
   ```Bash
   roscore
   ```

5.按下 `Ctrl + B` 加 `C` 创建一个新终端，并执行启动文件。
   ```Bash
   source ~/{your_download_path}/install/setup.bash
   roslaunch HDAS r1.launch
   ```

6.按下 `Ctrl + B` 加 `C` 创建一个新终端，进行自检。

   <span style="color:red">**重要提示：如果有任何错误，请及时联系我们提供技术支持。如果没有异常，请按下`Ctrl + C`关闭。**</span>

   ```Bash
   source {your_download_path}/install/setup.bash
   rosrun HDAS check_node 
   # 输入1 (0 表示安装机械臂后的自检；1表示未装机械臂时的自检)
   ```

7.按下 `Ctrl + B` 加 `C` 创建一个新终端，开启底盘控制。
   ```Bash
   source {your_download_path}/install/setup.bash
   roslaunch mobiman r1_chassis_control.launch
   ```

8.按下 `Ctrl + B` 加 `C` 创建一个新终端，开启手臂和躯干控制。
   ```Bash
   source {your_download_path}/install/setup.bash
   roslaunch mobiman r1_jointTrackerdemo.launch
   ```

9.按下 `Ctrl + B` 加 `C` 创建一个新终端，开始测试脚本。
   ```Bash
   source {your_download_path}/install/setup.bash
   python3 r1_demo_easy.py
   ```

10.输入对应每个测试动作的数字并按Enter键。随后，R1将开始执行测试动作。

<span style="color:red">**重要提示：在执行torso_test_1之前，请确保先验证arm_test_3。**这是为了避免在双臂处于不可控状态时下蹲，与R1本身或地面发生干扰。</span>

![R1_demo_code_cn](./assets/R1_demo_code_cn.png)

**注意：当您完成所有测试后，按`Q`退出测试。R1 将回到原始姿势。**
<p style="text-align: center;">
      <video src="../assets/R1_quit_test.mp4" style="width: 100%;"autoplay muted loop>
   </p>