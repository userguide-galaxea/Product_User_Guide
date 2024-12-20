# Connecting and Installing 

## Step 1: Disassembling the Covers

- **Detach the front shoulder cover:** Use the 3 mm hex L-key provided in the package to remove one M4 screw located on each side of the shoulder.
- **Detach the rear shoulder cover:** Use the 3 mm hex L-key to remove two hidden M4 screws located on each side of the shoulder.

![R1_shoulder_cover](assets/R1_shoulder_cover.png)

- **Detach the rear chest cover:** Use the 4 mm hex L-key provided in the package to remove the two M5 screws on each side of the waist.
- **Detach the front chest cover:** Use the 4 mm hex L-key to remove two hidden M5 screws on each side of the waist.

![R1_chest_cover](assets/R1_chest_cover.png)

## Step 2: Logging and Connecting Main Control Board

After removing the shoulder and chest covers:

-  The HDMI cable is located on the right side of the chest for connecting your display device.
-  The USB port is located below the chest for connecting a keyboard and mouse.

![R1_USB_HDMI](assets/R1_USB_HDMI.png)

- The current '_main control board*_' of R1 is industrial computer running Linux Ubuntu, with the username `r1` and the default password `1`.

- After connecting to Wi-Fi, you can remotely connect to the R1 via SSH using the command, with `192.168.xxx.xxx` as its IP address.

  - ```Bash
    ssh r1@192.168.xxx.xxx
    ```

- When you have finished the above procedures, **disconnect the HDMI and USB cables.**

'_*It is configurable: Industrial Computer/ECU/etc._'

## Step 3: Installing Arms

- **<span style="color:red;">When installing the robot arm, you must ensure that the arm and the base ports are facing backward.</span>**
- Use the 5 mm hex L-key and four M6 screws to secure the arm.

![R1_install_arms](assets/R1_install_arms.png)

- Plug the power and USB cables provide with Galaxea A1 arms to the ports on the arm base.

![R1_connect_cables](assets/R1_connect_cables.png)

## Step 4: Reattaching the Covers

After confirming that the communication connection with the robot arms is successful, reattach the covers by reversing the steps in Disassembling the Covers above.

## Next Step

Our quickstart journey has come to an end. To deepen your mastery of Galaxea R1, we strongly recommend exploring the following chapters in [Galaxea R1 Hardware User Guide](Hardware_Guide.md) and [Software User Guide](Software_Guide_firstmove.md). These resources offer a wealth of additional information and practical examples, guiding you through the intricacies of programming for Galaxea R1 with confidence and ease.
