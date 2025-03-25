# R1 常见问题

## 1. 如何在机器人端进行模型推理？
我们推荐两种方式来实现：

1. 将您需要运行的模型转换成TensorRT平台下的端侧模型。
2. 您外接一台工作站运行您的模型，通过ROS多机通信的方式实现工作站和机器人端的互通。

## 2. 机械臂无法控制/断联/显示为红灯？
您可参考以下步骤依次排查问题：

**第一步：排查机械臂末端的线路问题：**

1. 请您确认机械臂末端的连接线是否正确连接
2. 请您更换左右机械臂末端的连接线，进行aba测试。断电重启后观察问题是否在另一侧机械臂上复现。

**第二步：执行以下代码，查看CAN信号是否正常输出。**

1. 若有输出，可排除线路问题，建议您关机冷却后重启测试；
2. 若无输出，可能为机械臂的线路问题，请及时联系我们提供技术支持。
    ``` Bash
    candump can0 | grep 052  #查看左臂
    candump can0 | grep 062  #查看右臂
    ```

**第三步：运行HDAS，观察机械臂各关节的反馈，建议您截图记录此次反馈，用于后面对比定位问题出处。**
``` Bash
cd ~
./can.bash
source ~/work/galaxea/install/setup.bash
rostopic echo /hdas/feedback_status_arm_right #以右臂为例
```

**第四步：OTA更新**

1. 执行以下代码：
    ``` Bash
    cd ~ 
    ./can.bash
    source ~/work/galaxea/install/setup.bash
    rosrun HDAS mcu_ota   ~/work/galaxea//install/share/Embedded_Software_Firmware/R1/ACUR.bin 111
    ```
    注意：若是第4步完成后显示错误`file cannot read`，则需要修改最后一步的ACUR的路径。您可以在`Embeded_software_firmware`里查找一下ACUR的路径。

2. OTA更新成功后重启机器人，并观察机械臂问题是否仍在。

**第五步：再次观察机械臂各关节的反馈**

运行HDAS执行以下代码，对比第3步的截图，若是某个关节的`error_description`始终为`DISCONNECT`，请及时联系我们提供技术支持。
``` Bash
cd ~
./can.bash
source ~/work/galaxea/install/setup.bash
rostopic echo /hdas/feedback_status_arm_right #以右臂为例
```

## 3. R1运行时可否不安装机械臂？
可以，当不安装机械臂时，需要在机器人的双臂底座端口处安装电阻帽。以下为电阻的相关信息：

<span style= "color: red;">**注意：机器人的出厂配置中已在该位置加装了电阻。**</span>

1. 电阻值：120 欧姆（Ω）
2. 型号：WAFER-GH1.25-2PWB
3. 电阻的安装位置:机器人躯干的双臂各有一个黄色XT60的电源口，其旁边有个黄白双色的CAN口线，旁边可加装电阻。

## 4. R1能爬的最大坡度是多少？
经过我们内部测试，理想状态下R1可爬的最大坡度为14°。

## 5. R1头部ZED相机的焦距是多少？
R1头部ZED相机的焦距为2.1mm。

## 6. R1配备的腕部相机型号和连接方式是什么?
R1标配的腕部相机型号为：Intel Real Sense D435i，连接方式如下：

1. 准备一根≥1.5m的连接线（Type-C转USB接口）；
2. 将Type-C端连接腕部相机；
3. 将USB端连接机器人后颈部的任一USB接口。

## 7. 第二次自检过程中，显示双臂数据或者频率超过预期范围。
第二次自检前请依次确认机器人姿态：

1. R1已站立；
2. R1已安装手臂；
3. 机械臂肘部朝外，夹爪电缆朝外。