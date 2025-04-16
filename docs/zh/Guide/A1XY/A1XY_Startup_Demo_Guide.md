# A1XY 启动与Demo演示指南
## 1. 启动前准备

### 1.1 硬件准备

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">物品</th>
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">数量</th>
            <th style="width: 500px; padding: 8px; border: 1px solid #ddd;">备注</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">A1XY 机械臂</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">机械臂本体</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">上位机</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">操作系统：Ubuntu 20.04 LTS<br>中间件依赖：ROS Noetic</td>
        </tr>
    </tbody>
</table>

<span style="color:red;">**注意：请确保已按照A1XY开箱安装指南完成机械臂的本体安装及线束连接后，方可按照以下内容启动机械臂控制**</span>

### 1.2 软件准备
#### 1.2.1 下载SDK包

- 百度云：[https://pan.baidu.com/s/1jVEqjL-r_Ll7bKFB1XxKIw?pwd=a1xy](https://pan.baidu.com/s/1jVEqjL-r_Ll7bKFB1XxKIw?pwd=a1xy)
- Google Drive：[https://drive.google.com/drive/folders/180qSZTc7bZgwklVuwcuhq5O2DPteI2nR?usp=sharing](https://drive.google.com/drive/folders/180qSZTc7bZgwklVuwcuhq5O2DPteI2nR?usp=sharing)

当前为A1XY机械臂的首发软件版本。后续的版本更新您可以在A1XY软件版本更新日志中查看，获取最新的SDK包及更新信息。

#### 1.2.2 安装SDK

<span style="color:red;">**注意：SDK文件仅支持X86架构。**</span>

1. 执行以下命令检查通信连接状态。

      ```Bash
      ifconfig can0
      ```

      ![A1XY_sdk_ifconfigcan0_cn](./assets/A1XY_sdk_ifconfigcan0_cn.PNG)

2. 执行以下命令安装SDK：

      ```Bash
      tar -xf ${your_download_path}/A1_XY.tar.gz -C ~/
      ```

## 2. 启动SDK

### 2.1 启动CAN驱动程序

在控制A1XY的整个过程中，您需要打开多个终端。我们建议您使用TMUX。常用指令如下：

- 创建新终端：按`Ctrl + B`然后按`C`。
- 切换终端：按`Ctrl + B`然后按数字，数字表示终端窗口的序号。

现在，您可以按照以下步骤启动CAN驱动程序。

1. 启动TMUX。

      ```Bash
      tmux
      ```

2. 启动FDCAN通信。

      ```Bash
      ./can.bash    
      
      # 或执行以下操作
      sudo ip link set dev can0 type can bitrate 1000000 dbitrate 5000000 fd on
      sudo ip link set up can0
      ```

3. 启动roscore。

      ```Bash
      roscore    
      ```

4. 按`Ctrl + B`然后按`C`创建新终端。然后启动HDAS。

      ```Bash
      source {your_download_path}/install/setup.bash
      roslaunch HDAS A1XY.launch
      ```

### 2.2 启动控制

1. 运行以下命令以转储 CAN 总线消息。

      ```Bash
      candump can0
      ```
   ![A1XY_sdk_candumpcan0_cn](./assets/A1XY_sdk_candumpcan0_cn.PNG)

2. 按`Ctrl + B`然后按`C`创建新终端。运行以下命令以启用硬件接口和ROS接口。

      ```Bash
      source {your_download_path}/install/setup.bash
      
      # 根据产品型号，选择以下启动方式之一：
      roslaunch mobiman a1x_jointTrackerdemo.launch
      roslaunch mobiman a1y_jointTrackerdemo.launch
      ```

### 2.3 Demo演示

<span style="color:red;">**注意：如果出现任何错误，请及时联系我们提供技术支持support@galaxea.ai。**</span>

1. 停止后台所有运行中的TMUX，并关闭所有ROS程序。

      ```Bash
      sudo tmux kill-server
      tmux kill-server
      pkill -9 ros
      ```

2. 启动FDCAN通信。

      ```Bash
      sudo ip link set can0 down
      sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
      sudo ip link set can0 up
      ```

4. 进入tmux。

      ```Bash
      tmux
      ```

5. 启动roscore。

      ```Bash
      roscore
      ```

6. 按`Ctrl + B`然后按`C`创建一个新窗口，并执行启动文件。

      ```Bash
      cd {your_download_path}/install && source setup.bash
      roslaunch HDAS A1XY.launch
      ```

7. 按`Ctrl + B`然后按`C`创建一个新窗口。然后，开启关节控制。

      ```Bash
      cd {your_download_path}/install && source setup.bash

      # 根据产品型号，选择以下启动方式之一：
      roslaunch mobiman a1x_jointTrackerdemo.launch
      roslaunch mobiman a1y_jointTrackerdemo.launch
      ```

8. 按`Ctrl + B`然后按`C`创建一个新终端。然后，启动demo脚本。

      ```Bash
      cd {your_download_path}/install && source setup.bash
      rosrun mobiman test_mobiman_a1xy
      ```