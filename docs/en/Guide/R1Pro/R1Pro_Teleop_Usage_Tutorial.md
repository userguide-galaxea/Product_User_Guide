# R1 Teleop Usage Tutorial
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
            <td style="padding: 8px; border: 1px solid #ddd;">System: Ubuntu 20.04 ROS Noetic 
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
#### 3.2.1 Download and Unzip the Folder.

<span style="color:red;">Please use the dedicated SDK version of R1 Pro V1.1.7 and R1 Pro-T provided in the following link. Do not use the V1.1.7 version in the version log.</span>

- Google Drive：[R1 Pro-T ROS 1 SDK](https://drive.google.com/drive/folders/1awWaOjIlWNk8olzHGkcmlt8sC9VpzHX9?usp=sharing)
- Baidu Cloud：[R1 Pro-T ROS 1 SDK](https://pan.baidu.com/s/1BgKtbmyzy-Bn8NpHZtQc_g?pwd=r1pt)



#### 3.2.2 Install Software Dependencies
Please install the required software dependencies on the R1 Pro-T host computer.

```Bash
sudo apt install ros-noetic-joy 
sudo apt install ros-noetic-trac-ik
sudo apt install tmuxp
sudo apt install tmux
```

#### 3.2.3 Multi-device Communication Configuration
<span style="color:red;">Note: When R1Pro and R1Pro - T are on the same gateway, separately modify the ~/.bashrc file of the upper - computer environment of R1 Pro and R1 Pro - T.</span>

1. Add the following command to the end of the **`~/.bashrc`** file on the R1 Pro ECU:
    ```Bash
    export ROS_MASTER_URI=http://R1 Pro_IP address:11311
    export ROS_IP=R1 Pro_IP address
    ```

2. Add the following command to the end of the **`/.bashrc`** file on the R1 Pro-T host computer:
    ```Bash
    export ROS_MASTER_URI=http://R1 Pro-T_IP address:11311
    export ROS_IP=R1 Pro-T_IP address
    ```

3. Example:

    When the host computers of R1 Pro and R1 Pro-T are both on the 192.168.10.0 gateway, separately modify the `~/.bashrc` of the host computers of R1 Pro and R1 Pro-T.

    ![R1Pro-T_3.2.3](./assets/r1prot/R1Pro-T_3.2.3.png)


## 4. R1 Pro Teleop Connection
### 4.1 Securing the Device
Use the M6x16 screws and G-clips to secure the R1 Pro-T Base to the desktop, as shown in the diagram below:

![R1Pro-T_4.1_base_mount](./assets/r1prot/R1Pro-T_4.1_base_mount.png)

### 4.2 Connecting the Device
Following the hardware connection architecture diagram:

![R1Pro-T_4.2_conection](./assets/r1prot/R1Pro-T_4.2_conection.png)

1. Connect one end of the 2-pin plug to the port on R1 Pro-T Box and the other end to the XT60-F port on the power adapter.
2. Connect one end of the 4-pin plug to the port on R1 Pro-T Box and the other end to the XT30(2+2)-F port to the motor on the torso of R1 Pro-T.
3. Connect one end of the blue USB-CAN adaptor cable to R1 Pro-T Box and the other end to the USB port on the host computer.

<span style="color:red;">**Note: After completing the connections, do not power on immediately. Please follow the steps below to proceed.**</span>

### 4.3 Powering On
After securing the R1 Pro-T Base to the desktop, **place the arms and torso in the initial position** as shown in the diagram below. <span style="color:red;">**Ensure that the J5, J6, and J7 joints of both arms, as well as the R1 Pro-T Base, are at zero-point (initial posture)**</span>, then you may insert the power unit into the power outlet and press the power button on the R1 Pro-T Box to turn it on.

![R1Pro-T_4.3_initial_position](./assets/r1prot/R1Pro-T_4.3_initial_position.png)

<span style="color:red;">**Note: Before starting the R1 Pro-T every time, make sure to adjust to its initial posture. Failing to do so may pose a risk during operation.**</span>

### 4.4 Connect Bluetooth Remote Controller
Please be sure to connect two arms to the R1 Pro-T host conputer in sequence, ensuring that <span style="color:red;">**the left arm is connected first, followed by the right arm. Due to program settings, to streamline the operation process, if the controller connection sequence is incorrect, please turn off the power and power it on again, then follow the correct sequence for connection.**</span>.

#### 4.4.1 Bluetooth Controller
![]()

#### 4.4.2 Connection Steps

1. Press the power switch to turn on the handle Bluetooth.
2. Turn on the host computer's Bluetooth and connect to the two devices named `HID`.
3. After connection, "Device Connected" will be displayed. Enter the following command to check whether the connection is successful:
    ```Bash
    ls /dev/input/ | grep js
    ```

    When both`js0` (left arm) and `js1` (right arm) are returned simultaneously, the connection is successful.
    ![R1Pro-T_4.4.2_grep_js](./assets/r1prot/R1Pro-T_4.4.2_grep_js.png)

## 5. Launch SDK
### 5.1 Start R1
Use the following script to launch R1 Pro with one click:

```bash
cd ${SDK_path}/install/share/startup_config/script/
./robot_startup.sh boot ../session.d/ATCStandard/R1PROIsomorphicTeleop.d/
```

This script will launch nodes such as roscore, HDAS, mobiman, the camera, and the radar.

![R1Pro-T_5.1_r1prosdk](./assets/r1prot/R1Pro-T_5.1_r1prosdk.png)

### 5.2 Start R1 Pro-T SDK
<span style="color:red;">Note: This step must be performed on the R1 Pro-T host computer.</span>

1. Start TMUX.
    ```Python
    tmux
    ```
2. Start FDCAN Communication
    ```Python
    sudo ip link set dev can0 type can bitrate 1000000 dbitrate 5000000 fd on
    sudo ip link set up can0
    ```
3. Press `Ctrl + D` to exit TMUX.
4. Start R1 Pro-T HDAS node.
    ```Bash
    cd ${R1Pro-T_SDK_path}/install
    source setup.bash
    roslaunch HDAS r1prot.launch
    ```
5. Start R1 Pro-T teleoperation node.
    ```Bash
    cd ${R1Pro-T_SDK_path}/install
    source setup.bash
    roslaunch mobiman r1_pro_teleoperation.launch
    ```

After completing the above steps, wait 3-5 seconds and you will be able to control R1 Pro-T.

## 6. Data Collection Process
### 6.1 Introduction to Data Format
The file format for data acquisition is rosbag，and the file suffix is `*.bag`.

### 6.2 Data Acquisition
Default storage path：`/home/nvidia/GalaxeaDataset/{date}/`.

>

### 6.3 Data Write-to-Disk Files
The data is saved in `rosbag + json` format, and each file corresponds to one another. For example:

```json
# For example, the two files below represent a data bag.

S2R12000P18245_20240213173320125_RAW.bag
S2R12000P18245_20240213173320125_RAW.json

# The format is "robot_serial_number + timestamp + RAW"
# robot_serial_number：the serial number of robot which is located in the path "/opt/galaxea/body/RSN".
# timestamp：The timestamp of data collection in ms.
# RAW: the raw data.
```

###  6.4 Start Recording
Perform data recording operation through the VR left remote controller:
<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 200px; padding: 8px; border: 1px solid #ddd;">Function</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">Operation</th>
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">Description</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">Start recording</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Click button C on the left controller</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Start data recording.</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
                <td style="padding: 8px; border: 1px solid #ddd;">Stop recording</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Click button D on the left controller</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Stop data recording.</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
                <td style="padding: 8px; border: 1px solid #ddd;">Delete the current recording</td>
            <td style="padding: 8px; border: 1px solid #ddd;">Click button C on the left controller</td>
            <td style="padding: 8px; border: 1px solid #ddd;">If a recording is already in progress, click button C again will stop and delete the current recording, and it will not be written to disk.</td>
        </tr>
    </tbody>
</table>

If you encounter any issues during installation or startup, please contact us at [support@galaxea.ai](mailto:support@galaxea.ai) or call 4008 780 980 for technical support !