## Galaxea B1 Software Guide

### Software Dependency

1. [Ubuntu](https://en.wikipedia.org/wiki/Ubuntu) 20.04 LTS
2. ROS Noetic

### Installation

The [SDK](https://github.com/userguide-galaxea/B1_SDK) does not require recompilation. Please refer to the contents below.

### CAN Connection Tutorial

<u>Important: To ensure the USB-to-CAN Adapter works properly on Linux, please install the driver file that matches your system’s kernel version; otherwise, it may cause errors.</u>

**1. Launch the Driver**

Use the following command to check the current kernel version:

```Plain
name -r
```

The driver file name usually includes the version information. If you need drivers for other kernel versions, please contact us for assistance.

<table style="border-collapse: collapse; width: 100%;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Driver File</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">5.15.0-67-generic: 5.15.0_67.ko</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">5.15.0-101-generic: 5.15.0_101.ko</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">5.15.0-102-generic: 5.15.0_102.ko</td>
        </tr>
    </tbody>
</table>



Use the following command to load the kernel module:

```Bash
sudo insmod /usr/lib/modules/$(uname -r)/kernel/drivers/net/can/dev/can-dev.ko
sudo insmod 5.15.0_101.ko
```

**2. Set 1M Baud Rate**

Use the following command to set the CAN interface baud rate to 1 Mbps:

```Plain
sudo ip link set can0 type can bitrate 1000000
```

**3. Open the CAN Interface**

Use the following command to bring up the CAN interface:

```Plain
sudo ip link set up can0
```

If no errors occurred in the previous steps, use the following commands to check the CAN interface's configuration and status:

```Plain
sudo apt-get install net-tools
ifconfig can0
```

**4. Install Driver Dependencies**

Use the following command to install the necessary dependencies for the driver:

```Plain
 sudo apt-get install ros-noetic-socketcan-interface ros-noetic-can-msgs
```

**5. Install and Use can_utils**

Use the following commands to install the tool and monitor data from Galaxea B1 in real-time, provided that:

- B1 is already running, and
- The CAN-to-USB Adapter is properly connected.

```Plain
sudo apt-get install can_utils
candump can0
```

## Operation and Control

### Launch the Chassis Node

```Plain
source devel/setup.bash
roslaunch signal_chassis chassis_node.launch
```

### Run Keyboard Control

```Bash
rosrun signal_chassis keyboard_control.py
```

Use the following instructions to control the speed, angle, and motion mode of Galaxea B1.

<table style="border-collapse: collapse; width: 100%;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="padding: 8px; width: 100px;">Item</th>
            <th style="padding: 8px; width: 650px;">Description</th>
            <th style="padding: 8px;">Notes</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">⬆️</td>
            <td style="padding: 8px;">Used to increase the linear speed, which returns to 0 when the key is released.</td>
            <td style="padding: 8px;">Range (m/s): (-1, 1)</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">⬇️</td>
            <td style="padding: 8px;">Used to decrease the linear speed, which returns to 0 when the key is released.</td>
            <td style="padding: 8px;"></td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">⬅️</td>
            <td style="padding: 8px;">Used to increase the steering angle, which returns to 0 when the key is released.</td>
            <td style="padding: 8px;">Range (°): <br />Motion Mode 0: (-45,45) <br />Motion Mode 1: (-90,90)</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">➡️</td>
            <td style="padding: 8px;">Used to decrease the steering angle, which returns to 0 when the key is released.</td>
            <td style="padding: 8px;"></td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">0</td>
            <td style="padding: 8px;">Switch to Dual Ackerman Mode</td>
            <td style="padding: 8px;"></td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">1</td>
            <td style="padding: 8px;">Switch to Parallel Mode</td>
            <td style="padding: 8px;"></td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">2</td>
            <td style="padding: 8px;">Switch to Spinning Mode</td>
            <td style="padding: 8px;"></td>
        </tr>
    </tbody>
</table>


To connect remotely, use the following command to establish an SSH connection.

- Replace the `name` and `IP` with the actual name and address of the industrial computer.
- The `-X` option enables X11 (X Window System) forwarding to allow the handling of remote graphical interfaces and the transmission of local keyboard and mouse inputs.

```Bash
ssh name@IP -X
```

## Software Interface

### Driver Interface

This interface is used for the chassis status feedback ROS package. The package defines multiple topics for posting the status of multiple motors in the chassis. The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr style="background-color: black; color: white; text-align: left;">
      <th style="padding: 8px; border: 1px solid #ddd; width: 200px;">Topic Name</th>
      <th style="padding: 8px; border: 1px solid #ddd; width: 300px;">Description</th>
      <th style="padding: 8px; border: 1px solid #ddd;">Message Type</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color: white; text-align: left;">
      <td style="padding: 8px; border: 1px solid #ddd; width: 200px;">/hdas/motor_state</td>
      <td style="padding: 8px; border: 1px solid #ddd; width: 300px;">Chassis motion state feedback</td>
      <td style="padding: 8px; border: 1px solid #ddd;">chassis_msg/DrivetrainStamped</td>
    </tr>
  </tbody>
</table>




<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">Topic Name</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 300px;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Description</th>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;" rowspan="13">/hdas/motor_state</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.steering_angle_fl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel left-wheel steering angle degrees</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.steering_angle_fr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel right-wheel steering angle degrees</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.steering_angle_rl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel left-wheel steering angle degrees</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.steering_angle_rr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel right-wheel steering angle degrees</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_speed_fl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel left-wheel linear speed m/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_speed_fr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel right-wheel linear speed m/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_speed_rl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel left-wheel linear speed m/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_speed_rr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel right-wheel linear speed m/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_angular_speed_fl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel left-wheel angular speed rad/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_angular_speed_fr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Front-wheel right-wheel angular speed rad/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_angular_speed_rl</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel left-wheel angular speed rad/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.drive_angular_speed_rr</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Rear-wheel right-wheel angular speed rad/s</td>
  </tr>
  <tr style="background-color: white;">
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">.motion_mode</td>
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">Motion Mode 0: Dual Ackerman Mode <br /> Motion Mode 1: Parallel Mode <br /> Motion Mode 2: Spinning Mode</td>
  </tr>
</table>




### Control Interface

The chassis control interface can be used to command the chassis to move at the target speed, which includes the combination of X, Y, and Yaw Rate:

<table style="border-collapse: collapse; width: 100%;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="padding: 8px; width: 200px;">Topic Name</th>
            <th style="padding: 8px; width: 300px;">Description</th>
            <th style="padding: 8px;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">/cmd_vel</td>
            <td style="padding: 8px;">Issuing the chassis control signal</td>
            <td style="padding: 8px;">geometry_msgs/Twist</td>
        </tr>
    </tbody>
</table>



<table style="border-collapse: collapse; width: 100%;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="padding: 8px; width: 200px;">Topic Name</th>
            <th style="padding: 8px; width: 300px;">Field</th>
            <th style="padding: 8px;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;"rowspan="13">/cmd_vel</td>
            <td style="padding: 8px;">header</td>
            <td style="padding: 8px;">Standard ROS header</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">linear</td>
            <td style="padding: 8px;">Control of linear speed</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.x</td>
            <td style="padding: 8px;">Linear speed of x;<br />Range (m/s): (-1, 1)</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.y</td>
            <td style="padding: 8px;">Not used</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.z</td>
            <td style="padding: 8px;">Not used</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">angular</td>
            <td style="padding: 8px;">Control of motion mode and steering angle</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.x</td>
            <td style="padding: 8px;">Motion Mode 0: Dual Ackerman Mode <br />Motion Mode 1: Translation Mode<br />Motion Mode 2: Spinning Mode</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.y</td>
            <td style="padding: 8px;">Not used</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.z</td>
            <td style="padding: 8px;">Control of steering angle;<br />Range (°): <br />Motion Mode 0: (-45, 45) <br />Motion Mode 1: (-90, 90) <br />Motion Mode 2: 0 by default</td>
        </tr>
    </tbody>
</table>

