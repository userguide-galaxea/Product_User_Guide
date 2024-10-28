# R1 Step-By-Step Startup Guide
In this step-by-step guide, we will provide detailed instructions on how to unpack Galaxea R1 correctly, connecting cables, install the robot arm, and how to remotely control R1 to achieve better communication and explore more functions.

**At the end of the page, there will be an unboxing instruction video.**



## 1. Preparation
Please ensure that the following items/environments are prepared before powering on：

<table style="width: 100%; border-collapse: collapse;">
    <thead>
        <tr style="background-color: black; color: white; text-align: left;">
            <th style="width: 300px; padding: 8px; border: 1px solid #ddd;">Item</th>
            <th style="width: 400px; padding: 8px; border: 1px solid #ddd;">Quantity</th>
        </tr>
    </thead>
    <tbody>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">R1 Base</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">A1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">G1</td>
            <td style="padding: 8px; border: 1px solid #ddd;">2</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">Hardware Set</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">HDMI Cable - 1.5 m</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">Display & Keyboard & Mouse</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">Computer</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>
        <tr style="background-color: white; text-align: left;">
            <td style="padding: 8px; border: 1px solid #ddd;">WiFi Network</td>
            <td style="padding: 8px; border: 1px solid #ddd;">1</td>
        </tr>        
    </tbody>
</table>

<u>Note: To ensure the normal operation of the product, place R1 in a dry and well-ventilated environment, and ensure that there are no obstacles or dangerous items around</u>


## 2. Unboxing

![2.2_getting_out](assets/2.2_getting_out.png)

### 2.1 Unlock the Box

Locate two locks on the left side of the box. Open the two locks on the left side of the box door, take out the lock tabs and rotate counterclockwise to open the front door. <u>You will see a folded R1 with head, chest cavity, torso, chassis and no arms, a sealed box containing a full set of screwdrivers and a joystick controller, and replacement wiring harnesses.</u>

### 2.2 Remove Box Fixings
Lower the board. Use the L-hex key to remove six fixing screws shown in the figure. It might require at least two people to pull the robot out of the box to avoid collision. 

### 2.3 Remove Chassis Fixings
There are fixings on both sides of the front and at the rear of the chassis. Rotate the wheels of the fixings to both sides to expose the fixed screws. Use the matching L-hex key to remove the fixing screws on the wheels, and then turn the wheels back to their original positions.

![2.3_chassis_fixings_remove](assets/2.3_chassis_fixings_remove.png)

### 2.4 Detach Rear Shell
Open the peripheral interface cover on the rear shell. Use the L-hex key to remove two M4 screw. Use the L-hex key to remove two M5 screws on each side of the chest. Then you can detach the rear shell. 

![2.4_detach_rear_shell](assets/2.4_detach_rear_shell.png)

### 2.5 Detach Front Shell
Use the L-hex key to remove six M5 screws on two sides of the chest, as boxed in the figure, then you can detach the front shell.

![2.5_front_shell_remove](assets/2.5_front_shell_remove.png)

### 2.6 Remove Arm Fixings
Use the largest L-hex key to remove two screws inside the chest, as boxed in the figure, then you can detach the fixing stick.

![2.6_arm_fixings_remove](assets/2.6_arm_fixings_remove.png)



## 3. Turn On R1

### 3.1 Connect the Display, Keyboard and Mouse
<span style="color:red;">**Important: For your safety, please power off R1 before connecting any cables.**</span>

Use the L-hex key to remove two M3 screws on the peripheral interface cover on the chassis. Connect HDMI cable to the chassis and the display. 

<u>Note: Please ensure a firm connection to avoid looseness.</u>

![3.1_HDMI_Connecting](assets/3.1_HDMI_Connecting.png)

Connect the mouse and keyboard to the USB interface on the chest. 
<u>Note: Please ensure a firm connection to avoid looseness.</u>

![3.1_USB_Connecting](assets/3.1_USB_Connecting.png)

### 3.2 Power On

The battery is located at the bottom of the right side of the chassis. To change the battery, remove two screws and slide the cover right to detach it. Then, put the battery into the chassis, connect the battery cable to the chassis, and close the cover. To remove and change the battery, reverse the above steps.

![3.2_battery_in](assets/3.2_battery_change.png)

The power supply port is located at the bottom of the rear of the chassis. To charge the robot, unscrew the plug case, and insert the power cord into the port. When the red light on the charger is on and the battery indicator on the chassis is flashing in green, it indicates that R1 is charging.

<u>Note: Please keep the emergency stop button raised and turn on the boat-shaped power button located at the bottom of the left rear of the chassis. If the power was previously on, you need to turn it off and then turn it on again; otherwise, the display may not be able to work.</u>

![3.2_power_open](assets/3.2_power_open.png)


### 3.3 Obtain IP Address
After R1 is powered on, wait for the display to show the desktop. Click "Settings" and connect to WiFi.

![3.3_wifi_connect](assets/3.3_wifi_connect.png)

Then open the command terminal and enter the following command:

```bash
ifconfig mlan0
```

The IP address of R1 Orin is: `192.168.xxx.xxx`.

![3.3_IP_address](assets/3.3_IP_address.png)



### 3.4 Remotely Connect to R1

Use the other computer, and enter the following command in the terminal, connecting to Orin. 

```bash
ssh nvidia@IP address
# Enter the password  (default: nvidia)
```

After completing the above steps, if the connection is successful, disconnect the HDMI and USB cable, and close the interface cover plates on the chassis and chest, to avoid affecting the activity range. **<u>If the connection fails, please contact us in time for technical support.</u>**

### 3.5 The First Remote Self-Check
<span style="color:red;">**Important: Before you do any actions on R1, you must complete R1 remote self-checks to ensure the safety.**</span>

Perform the first remote self-check according to the following steps. Please make sure:

- R1 has pulled out of the box and all fixings are removed.
- R1 remains folded and does not have the robot arms installed.

After the remote connection is successfully established, you need to open multiple terminals. We recommend you to use TMUX: 

- Create a new terminal: Press `Ctrl + B` then `C`. 

- Switch terminals: Press `Ctrl + B` then press number which indicates the number of terminal windows. 

**Step 1:** Start TMUX.

```bash
tmux
```

**Step 2:** Start FDCAN Communication

```bash
./can.bash    
```

**Step 3:** Start roscore

```bash
roscore
```

**Step 4:** Start HDAS

``` bash
source ~/work/ci_pipeline/workspace/body/install/setup.bash
roslaunch HDAS hdas.launch
```

**Step 5:** Press `Ctrl + B` then `C` to create a new terminal. Then, start self-check.

```bash
source work/ci_pipeline/workspace/body/install/setup.bash
rosrun HDAS check_node #after execute the command, please press 0. 0 means the self-check when the arms are uninstalled.
```

<u>**If there is any error, please contact us in time for technical support. If there is no abnormality, press `ctrl+c` to close .**</u>



### 3.6 Stand Up

<u>Note: Please ensure that R1 is out of the box and there is no interference or fixings. Otherwise, the torso may enter the locked-rotor protection state.</u>

Once the self-check is completed, you can make R1 standing up with the following steps by using the joystick controller. 

![3.6_controller](assets/3.6_controller.png)

<u>Note: Please ensure all switches (SWA/SWB/SWC/SWD) are in the top position before you do any actions.</u> This will place the machine in a stop state, preventing the robot from operating. 

![3.6_controller_warnning](assets/3.6_controller_warnning.png)

**Step 1:**  Press `Ctrl + B` then `C` to create a new terminal. Then, start arm and torso control.

```bash
source work/ci_pipeline/workspace/body/install/setup.bash
roslaunch mobiman r1_jointTrackerdemo.launch
```

**Step 2:** Press `Ctrl + B` then `C` to create a new terminal. Then, start chassis control.

```bash
source work/ci_pipeline/workspace/body/install/setup.bash
roslaunch mobiman r1_chassis_control.launch
```

**Step 3:** Switch SWA, SWD to the bottom, and switch SWB,SWC to the middle. 

**Step 4:** Move the left joystick to the upper left, and move the right joystick to the upper right simultaneously. Waiting for 3 seconds,  **R1 will stand up.**

![3.7.2_controller_standup](assets/3.6_controller_standup.png)



### 3.7 Install Arms 

<span style="color:red;">**Important: For your safety, please power off R1 before installing arms.**</span>

**Step 1:** Attach gripper to arm. To remove them, simply reverse these steps.

![3.7_G1GEN2_attaching](assets/3.7_G1GEN2_attaching.png)


- **Alignment Check:** Ensure that the three mounting holes around the gripper are aligned with the three mounting holes at the end of A1.
- **Screw Fixation:** Once aligned, secure and tighten the gripper to the arm using the three screws provided.
- **Final Check:** After tightening the screws, double-check the alignment and stability of the gripper. It should be firmly attached and not wobble or move independently of the robot arm.

**Step 2:** Use the hex L-key (5 mm) and four M6 screws to secure the arm.

![3.7_arm_install](assets/3.7_arm_install.png)

**<u>Note: When installing the robot arm, you must ensure that the ports on the arm base are facing backward.</u>**

**Step 3:** Connect the power and CAN cables provided with arms to the ports on the arm base.

![3.7_arm_cables](assets/3.7_arm_cables.png)

**Step 4:** After confirming that the communication connection with the robot arms is successful, reattach the covers by reversing the steps in **2.4** and **2.5** above.



### 3.8 The Second Remote Self-Check
Power on R1 and perform the second remote self-check according to the following steps. Please make sure:

- R1 has stood up.
- R1 has arms installed correctly.

**Step 1:** Start TMUX

```bash
tmux
```

**Step 2:** Start FDCAN Communication

```bash
./can.bash    
```

**Step 3:** Start roscore

```bash
roscore
```

**Step 4:** Start HDAS

```bash
source ~/work/ci_pipeline/workspace/body/install/setup.bash
roslaunch HDAS hdas.launch
```

**Step 5:** Press `Ctrl + B` then `C` to create a new terminal. Then, start self-check.

```bash
source work/ci_pipeline/workspace/body/install/setup.bash
rosrun HDAS check_node #press 1  #1 means the self-check when the arms are installed.
```

<u>**If there is any error, please contact us in time for technical support. If there is no abnormality, press `ctrl+c` to close .**</u>

**Step 6:** Press `Ctrl + B` then `C` to create a new terminal. Then, start arm and torso control.

```bash
source work/ci_pipeline/workspace/body/install/setup.bash
roslaunch mobiman r1_jointTrackerdemo.launch
```

**Step 7:** Press `Ctrl + B` then `C` to create a new terminal. Then, start chassis control.

```bash
source work/ci_pipeline/workspace/body/install/setup.bash
roslaunch mobiman r1_chassis_control.launch
```

**Step 8 (Optional):** Press `Ctrl + B` then `C` to create a new terminal. Then, play the demo. 

```bash
source work/ci_pipeline/workspace/body/install/setup.bash
python3 demo.py
```



## Instruction Video

**Coming Soon**
