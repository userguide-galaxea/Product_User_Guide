# R1 Pro Demo演示指南
<p style="text-align: center;">
      <video src="../assets/R1_quit_test.mp4" style="width: 100%;"autoplay muted loop>
   </p>
   
## 使用前准备
在执行Demo演示前，请确保：

1. R1 Pro的双臂已准确安装完毕，此时手臂姿态应为肘关节连接处（黑色橡胶朝正前方）；夹爪垂直朝下指向地面，且夹爪电缆朝外。
2. R1 Pro的躯干保持站立姿态。
3. 以R1 Pro为中心，方圆 1.5 米范围内不得有人员以及障碍物存在。

![R1Pro_6_2ndcheck_cn](./assets/unbox/R1Pro_6_2ndcheck_cn.png)

## 脚本执行

<span style="color:red">**注意：请严格按照顺序进行演示，如出现任何错误，请及时按下`Ctrl + C`关闭程序，并联系我们提供技术支持。**</span>

1. 停止后台所有运行中的TMUX，并关闭所有ROS程序。
   ```Bash
   sudo tmux kill-server
   tmux kill-server
   pkill -9 ros
   ```

2. 进入tmux。
   ```Bash
   tmux
   ```

3. 启动FDCAN通信。
   ```Bash
   sudo ip link set dev can0 type can bitrate 1000000 dbitrate 5000000 fd on
   sudo ip link set up can0  
   # 可能需要再次输入密码：nvidia
   # 若出现“RTNETLINK answers: Device or resource busy”这个错误信息，通常表示你尝试配置的设备（如网络接口或CAN收发器）已经被配置并且正在运行中。
   ```

4. 启动roscore。
   ```Bash
   roscore
   ```

5. 按下 `Ctrl + B` 加 `C` 创建一个新终端，并执行启动文件。
   ```Bash
   source {your_download_path}/install/setup.bash
   roslaunch HDAS r1pro.launch
   ```

6. 按下 `Ctrl + B` 加 `C` 创建一个新终端，进行自检。
   ```Bash
   source {your_download_path}/install/setup.bash
   rosrun HDAS check_node 
   # 按1 (0 表示安装机械臂后的自检；1表示未装机械臂时的自检)
   ```

7. 按下 `Ctrl + B` 加 `C` 创建一个新终端，开启底盘控制。
   ```Bash
   source {your_download_path}/install/setup.bash
   roslaunch mobiman r1_pro_chassis_control.launch
   ```

8. 按下 `Ctrl + B` 加 `C` 创建一个新终端，开启手臂和躯干控制。
   ```Bash
   source {your_download_path}/install/setup.bash
   roslaunch mobiman r1_pro_jointTrackerdemo_pid.launch
   ```

9. 按下 `Ctrl + B` 加 `C` 创建一个新终端，开始执行脚本。
   ```Bash
   source {your_download_path}/install/setup.bash
   cd {your_download_path}/install/share/mobiman/script
   python3 r1pro_test_open_box.py
   ```

10. 输入对应每个动作的数字并按Enter键。随后，R1将开始执行demo动作。
   ![R1Pro_7_demo_cn](./assets/unbox/R1Pro_7_demo_cn.png)

11. 当完成所有动作后，按`q`退出。R1 将回到零点姿势。