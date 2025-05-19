# R1 Pro Teleop 同构遥操作教程
> 欢迎使用 **R1 Pro Teleop（简称 R1 Pro-T）** —— 专为 R1 Pro 打造的同构遥操作平台。

## 1. 产品介绍
R1 ProTeleop平台采用按比例缩小设计，完美复刻R1 Pro 的各项功能，实现全身力反馈遥操作、全关节映射，并支持本体端力反馈至遥操端，确保操作精准同步，具备毫米级精度与毫秒级响应速度。

接下来的教程将详细指导您如何安装和启动 R1 Pro Teleop，助您迅速体验这款高性能遥操作平台。

## 2. 开箱

收到产品时，请根据以下货品交付清单检查包装盒内的物品是否齐全。

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">物品</th>
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">数量</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">R1 Pro-T 本体</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
                <td style="padding: 8px; border: 1px solid #ddd;">R1 Pro-T Box（电源/通信盒）</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">电源适配器（24V）</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">4芯航插——XT30母 (2+2)</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">2芯航插——XT60公</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">USB-CAN转换线</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">USB-Type-C充电线</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">魔术贴绑带</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">G夹固定器</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">G夹固定板</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">内六角扳手</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1（套）</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">M6x16 六角螺丝</td>
            <td style="padding: 8px; border: 1px solid #ddd;">8</td>
        </tr>
    </tbody>
</table>


## 3. 启动前准备
### 3.1 硬件准备
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">物品</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">数量</th>
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">备注</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">R1-T 上位机（双系统，非虚拟机）</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">系统：Ubuntu20.04 ROS Noetic <br /><span style="color:red;">注意：请勿在虚拟机中使用R1 Pro-T上位机，否则可能无法连接蓝牙遥控器。</span></td>
        </tr>
        <tr style="background-color: white; text-align: left;">
                <td style="padding: 8px; border: 1px solid #ddd;">局域网</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">用于R1 Pro和R1 Pro-T之间的无线通信。</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">网线</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">用于查看R1 Pro的IP地址。</td>
        </tr>
    </tbody>
</table>


### 3.2 软件准备
请使用对应版本的SDK和使用说明文档：

#### ROS1
- [R1 Pro-T ROS1 SDK_V1.1.7](https://pan.baidu.com/s/1dQ-ii0b84lJU0PYj_rMIxA?pwd=v117)
- [R1 Pro-T ROS1 使用说明](./R1Pro_Teleop_Usage_Tutorial_ros1.md)

#### ROS2
- [R1 Pro-T ROS2 SDK_V2.0.4](https://pan.baidu.com/s/1zoj-OHocixBZ4AcR_5v2fQ?pwd=v204)
- [R1 Pro-T ROS2 使用说明](./R1Pro_Teleop_Usage_Tutorial_ros2.md)
注意：文件`atc_host_standard_V2.0.4.tar.gz`配合文件`atc_standard-V2.0.4.tar.gz`一起使用。
