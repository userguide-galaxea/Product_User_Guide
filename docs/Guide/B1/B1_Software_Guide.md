## Galaxea B1 Software Guide

### Software Dependency

1. [Ubuntu](https://en.wikipedia.org/wiki/Ubuntu) 20.04 LTS
2. ROS Noetic

### Installation

The SDK does not require recompilation. Please refer to the contents below.

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
- Make sure to switch the joystick controller SWB to the bottom and SWC to the middle. Otherwise, `candump can0` cwill be unable to access the CAN data, and the upper computer will not be able to control the chassis.

```Plain
sudo apt-get install can_utils
candump can0
```

### First Move

#### Launch the Chassis Node

```shell
source install/setup.bash
roslaunch signal_chassis r1.launch   
```

#### Control the Chassis

```shell
rostopic pub /control_command signal_chassis/ControlCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
motion_mode: 0
x: 0.5
y: 0.0
w: 0.0"
```



### Software Interface

#### Driver Interface

This interface is used for the chassis status feedback ROS package. The package defines multiple topics for posting the status of multiple motors in the chassis. The following are detailed descriptions of each topic and its related message types:

<table style="width: 100%; border-collapse: collapse;">
  <thead>
    <tr style="background-color: black; color: white; text-align: left;">
      <th style="padding: 8px; border: 1px solid #ddd; width: 200px;">Topic Name</th>
      <th style="padding: 8px; border: 1px solid #ddd; width: 200px;">Description</th>
      <th style="padding: 8px; border: 1px solid #ddd; width: 400px;">Message Type</th>
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
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd; width: 200px;">Field</th>
    <th style="background-color: black; color: white; vertical-align: middle; padding: 8px; border: 1px solid #ddd;width: 350px;">Description</th>
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
    <td style="vertical-align: middle; padding: 8px; border: 1px solid #ddd;">0: Dual Ackerman Mode <br /> 1: Parallel Mode <br />: Spinning Mode</td>
  </tr>
</table>

#### Control Interface

The  chassis control interface can be used to command the chassis to move at  the target speed, which includes the combination of X, Y, and Yaw Rate:

<table style="border-collapse: collapse; width: 100%;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="padding: 8px; width: 200px;">Topic Name</th>
            <th style="padding: 8px; width: 200px;">Description</th>
            <th style="padding: 8px; width: 400px;">Message Type</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">/control_command</td>
            <td style="padding: 8px;">Issuing the chassis control signal</td>
            <td style="padding: 8px;">signal_chassis/ControlCommand.msg</td>
        </tr>
    </tbody>
</table>




<table style="border-collapse: collapse; width: 100%;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="padding: 8px; width: 200px;">Topic Name</th>
            <th style="padding: 8px; width: 200px;">Field</th>
            <th style="padding: 8px; width: 400px;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="vertical-align: middle; padding: 8px;"rowspan="13">/control_command</td>
            <td style="padding: 8px;">header</td>
            <td style="padding: 8px;">Standard ROS header</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.motion_mode</td>
            <td style="padding: 8px;">0: Dual Ackerman Mode <br>1: Transition Mode<br>2: Spinning Mode<br>3: Vector Control Mode<br>4: Breaking Mode</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.x</td>
            <td style="padding: 8px;">Not used for Mode 2 & Mode 4;<br> Mode 0: Linear speed of x in the range of (-1, 1) m/s<br>Mode 1: Linear speed of x-direction in the range of (-1, 1) m/s<br>Mode 3: Linear speed of x-direction in the range of (-1, 1) m/s</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.y</td>
            <td style="padding: 8px;">Not used for Mode 0 & Mode 2 & Mode 4<br>Mode 1: Linear speed of y-direction in the range of (-1, 1) m/s<br>Mode 3: Linear speed of y-direction in the range of  (-1, 1) m/s </td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px;">.w</td>
            <td style="padding: 8px;">Not used for Mode 1 & Mode 4;<br>Motion Mode 0: (-45°, 45°) <br />Motion Mode 2: angular speed in the range of (-3.7, 3.7) rad/s <br />Motion Mode 3: angular speed in the range of (-3.7, 3.7) rad/s</td>
        </tr>
    </tbody>
</table>

