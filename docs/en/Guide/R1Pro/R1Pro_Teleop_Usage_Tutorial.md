# R1 Pro Teleop Usage Tutorial
> Welcome to R1 Pro Teleop (R1 Pro-T) — the isomorphic teleoperation platform designed specifically for Galaxea R1 Pro. 

## 1. Product Introduction

R1 Pro Teleop is a platform which is designed with a scaled-down version that perfectly replicates the full functionality of the R1 Pro, enabling full-body force feedback teleoperation and full-joint mapping. It also supports force feedback from the body to the remote control side, ensuring precise synchronization of operations with millimeter-level accuracy and millisecond-level response time. 

The following tutorial will guide you through the installation and setup of R1 Pro Teleop, helping you quickly experience this high-performance teleoperation platform.

## 2. Unboxing
Please check whether all items in the shipping container are present.
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">Item</th>
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">Quantity</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">R1 Pro-T Base</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
                <td style="padding: 8px; border: 1px solid #ddd;">R1 Pro-T Box</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">Power Unit (24V)</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">4-pin plug—XT30-F (2+2)</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">2-pin plug—XT60-M</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">USB-CAN Adaptor</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">USB-Type-C Charging Cable</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">Velcro band</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">G-clips</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">G-clip board</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">L-hex wrench</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">M6x16 Screws</td>
            <td style="padding: 8px; border: 1px solid #ddd;">8</td>
        </tr>
    </tbody>
</table>

## 3. Preparation Before Start
### 3.1 Hardware Preparation
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">Item</th>
            <th style="width: 100px; padding: 8px; border: 1px solid #ddd;">Quantity</th>
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">Notes</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">R1 Pro-T Host Computer</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">System: Ubuntu 20.04 ROS1 Noetic or Ubuntu 22.04 ROS2 Humble
            </br><span style="color:red;">Note: Do not use the R1 Pro-T host computer in a virtual machine, as this may prevent Bluetooth controller connection.</span></td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">Local Area Network (LAN)</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Used for wireless communication between R1 Pro and R1 Pro Teleop.</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">Ethernet Cable</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Used to check the IP address of R1 Pro.</td>
        </tr>
    </tbody>
</table>

### 3.2 Software Preparation
Pleases use the appropriate version of SDK and usage tutorial.

#### ROS1
- [R1 Pro-T ROS1 SDK_V1.1.7](https://drive.google.com/drive/folders/1awWaOjIlWNk8olzHGkcmlt8sC9VpzHX9?usp=sharing)
- [R1 Pro-T ROS1 Usage Tutorial](./R1Pro_Teleop_Usage_Tutorial_ros1.md)

#### ROS2
- [R1 Pro-T ROS2 SDK_V2.0.4](https://drive.google.com/drive/folders/1ZQX9nKik3OO9LzeD6MqQF0x8Mf7VN5YD?usp=sharing)
- [R1 Pro-T ROS2 Usage Tutorial](./R1Pro_Teleop_Usage_Tutorial_ros2.md)