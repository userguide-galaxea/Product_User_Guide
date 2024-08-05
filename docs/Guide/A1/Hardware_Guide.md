# A1 Hardware Guide
This manual provides engineering data and user guidance for working with the Galaxea Robot A1 hardware.

## Safety Guide
<div>
<img src="../assets/warning_sign.jpg" alt="warning_sign" width="320">
</div>
Galaxea robots are potentially dangerous machines with safety hazards. If improperly used, they can cause injury.

- All users must carefully read the following safety information before using the robot.
- Anyone near the robot who has not read this safety information must be closely supervised at all times and made aware that the robot could be dangerous.
- Only use the robot after inspecting the surrounding environment for potential hazards.

### Intended use
**The Galaxea robot is explicitly designed for use by researchers in highly controlled indoor scientific environments. **
Since it has not undergone the certification process required for other uses (e.g., consumer use in a home environment), this product is not recommended or suitable for such unspecified uses.

### Safety Hazards
As described later in this document, the design of A1 is safer than previous commercial robot arms, allowing researchers to explore a broader range of use scenarios. However, A1 is still a potentially dangerous research robot. Researchers must use it with caution to avoid damage, injury, or death. Here, we outline several safety concerns that researchers must consider before and during the use of A1.

#### Operation Guide for Electrical Safety:
A1 is a highly electrified device that integrates motors, precision electronics, intricate wires, and other critical electrical components. It is equipped with exposed connectors for power connections. While A1 is designed with fuses to mitigate potential electrical risks, users must remain extremely vigilant. Ensure the arm is dry and away from any form of liquid to prevent electric shock. Additionally, regularly inspect the power cord and wires, handle the robot's connectors carefully, and maintain continuous attention to electrical safety throughout use.

#### Safety Tips for Loose Clothing and Hair
A1 has several twisting joints that provide great flexibility and pose a risk of accidentally pulling in and pinching loose clothing or hair. To avoid this, it is recommended that users wear well-fitting clothes that are less likely to get caught and tie up long hair or use hair nets and other protective measures. Always remain vigilant to ensure your safety and that of others while moving in the working area of the robot arm.

### Additional Risks
The key to safe use of the A1 arm is indeed good judgment and common sense, while following a strict set of operating guidelines.

- **Stop Using if Damaged**: If any signs of damage are detected, such as abnormal noise, erratic movement, or loose parts, stop using the robot arm immediately. A damaged arm can pose a safety hazard, and continued use may lead to accidents.
- **Maintain Immediate Stop Capability**: While operating the robot arm, ensure you are always within reach to quickly and safely stop its movement. Make sure the emergency stop button on the PC or charging cable is easily accessible and responds promptly in an emergency.
- **Limit Operation to Specialists**: Due to the complexity and potential hazards of the A1 robot arm, only users with specialized knowledge and experience should operate it. Non-professionals should avoid approaching or operating the arm unless under the supervision of an experienced user.
- **Ensure Wire Safety**: During operation, the robot arm’s wires and cables may be pulled or worn. Regularly check the integrity of the wire connections and the condition of the wire surfaces. Ensure that wires are not tripped over or stepped on during the arm’s movement.
- **Avoid Outdoor Operation**: The A1 is designed for indoor environments. Its construction and materials may not be suitable for outdoor use. Outdoor factors such as wind, rain, dust, and temperature changes can damage the robot arm, impacting its performance and safety.

## Disclaimer
<u>Galaxea Robots are explicitly designed for use by researchers in highly controlled indoor scientific environments. Since they have not undergone the certification process required for other uses (e.g., consumer use at home), these products are not recommended or suitable for such unspecified applications.<u>
## Technical Specification
### Electric Parameters
The electrical parameters of A1 robot arm include its voltage, current, and communication interface. Its design ensures stable and reliable performance, even in high payload and dynamic applications.
| Parameters              | Value                 |
|-------------------------|-----------------------|
| Nominal Voltage         | 48 V                   |
| Rated Current           | 30 A                   |
| Maximum Current         | 50 A                   |
| Communication Interface | USB serial port       |

### Performance Parameters
The performance parameters highlight A1's key specifications, such as weight, payload capacity, arm reach, and speed, reflecting its superior performance in highly dynamic operations.
| Performance                          | Parameter  |
|--------------------------------------|------------|
| Weight                               | $6 kg$      |
| Rated Payload                        | $2 kg$ |
| Maximum Payload                      | $5 kg$ |
| Reach                                | $700 mm$    |
| Maximum End-Effector Linear Velocity | $10 m/s$    |
| Maximum End-Effector Acceleration    | $40 m/s²$  |
| Maximum Gripping Force               | $200 N$     |
| Repeatability                        | $1 mm$      |


## Hardware Structure
![A1_hardware_architecture_topo](assets/A1_hardware_architecture_topo.jpg)

## Robot Structure
### Joint
The joint performance parameters detail the operating range, rated torque, and peak torque of the six joints, showcasing the robot's flexibility and power across a variety of operations.

![A1_joints](assets/A1_joints.png)

| Joint   | Range           | Rated Torque |
|---------|-----------------|--------------|
| Joint 1 | $[-165°, 165°]$ | $20 Nm$      |
| Joint 2 | $[0°, 180°]$    | $20 Nm$      |
| Joint 3 | $[0°, 190°]$    | $9 Nm$       |
| Joint 4 | $[-165°, 165°]$ | $3 Nm$       |
| Joint 5 | $[-95°, 95°]$   | $3 Nm$       |
| Joint 6 | $[-105°, 105°]$ | $3 Nm$       |

![A1_working_space_with_gripper](assets/A1_working_space_with_gripper.png)

View 1: Shows the working radius and rotation angle of Joint 1, with a rotation radius of 780 mm and a maximum rotation angle of 330 degrees.
View 2: Displays the rotation ranges for Joint 2 and Joint 3, with a maximum rotation angle of 180 degrees for Joint 2 and 190 degrees for Joint 3.
View 3: Illustrates the rotation angles of Joints 4, 5, and 6, as well as the end position of the robot arm. The maximum rotation angle for Joint 4 and 6 is 330 degrees, while Joint 5 has a maximum rotation angle of 190 degrees.

### Base
A1 features two ports on the rear of the base for development and charging.

<div style="text-align: center;">
    <img src="../assets/A1_base.png" alt="A1_base" width="450">
</div>



| Item           | Notes                                  |
|----------------|----------------------------------------|
| Charging Port  | Rated voltage 48 V                      |
| USB Port       | USB 2.0                                 |
| Mounting Holes | Four M6 threads with a diameter of 6.3 mm |
| Size           | 100 mm x 100 mm                          |


### Link
A1 robot arm consists of two Acrylonitrile Butadiene Styrene (ABS) main links mounted on a base, interconnected by six joints. Each joint is equipped with planetary gear motors that provide independent variable-speed operation, delivering high precision and torque. This design allows the arm to maneuver in any direction commanded by the controller.
![A1_size](assets/A1_size.jpg)


The arm is designed to have:

| Item                | Notes                                     |
|---------------------|-------------------------------------------|
| Length              | Deployed $775 mm$ <br/> Folded $449 mm$   |
| Height              | Deployed $237 mm$ <br/> Folded $277 mm$ |
| Width               | $128 mm$                                   |
| Reach               | $700 mm$                                    |
| Degree of Freedom   | $6$                                       |
| Maximum Payload     | $5 kg$                                     |
| Weight              | $6 kg$                                     |



### End-Effector
#### Gripper 1 Generation II

The gripper is composed of one motor, two clips, and one specially designed joint module.
*Note: No gripper is included with the product. You can purchase end-effectors or customized tools from Galaxea AI if needed.*
![A1_G1_size](assets/A1_G1_size.png)


| Feature                 | Value       |
|-------------------------|-------------|
| Length                  | 149 mm    |
| Length of Fingers       | 77 mm     |
| Diameter of Motor       | 60 mm      |
| Gripper Operating Range | 0~60 mm |
| Gripper Rated Force     | 100 N       |

![A1_size_g1](assets/A1_size_G1.jpg)

A1 with end-effector G1 should have:


| Item              | Notes                            |
|-------------------|----------------------------------|
| Length            | Deployed 918 mm, Folded 545 mm |
| Height            | Deployed 237 mm, Folded 277 mm |
| Width             | 128 mm                            |
| Degree of Freedom | 7                                |
| Maximum Payload   | 5 kg                              |
| Weight            | 6 kg                              |


#### Attaching
Here we describe how to attach grippers to A1. To remove them, simply reverse these steps.

<div style="text-align: center;">
    <img src="../assets/A1_attaching_g1.png" alt="A1_attaching_g1" width="680">
</div>

<div style="text-align: center;">
    <img src="../assets/A1_attaching_a1.png" alt="A1_attaching_a1" width="680">
</div>


1. **Alignment Check**: Ensure that the four protruding points on the gripper align perfectly with the corresponding recessed points on the robot arm's end effector mount. Proper alignment ensures the gripper is correctly positioned and centered.
2. **Initial Placement**: Carefully place the gripper onto the robot arm's end effector mount, ensuring that the protrusions fit snugly into the recesses. This step is crucial for achieving the correct orientation and balance.
3. **Screw Fixation**: Once aligned, secure the gripper to the robot arm using the three screws provided. These screws should be placed around the outer circle of the mounting area, as shown in the figure. Tighten the screws evenly to prevent any skewing or misalignment.
4. **Final Check**: After tightening the screws, double-check the alignment and stability of the gripper. It should be firmly attached and not wobble or move independently of the robot arm.
5. **Testing**: Before using the robot arm, perform a test run to ensure that the gripper moves smoothly and holds objects securely without any slippage or unexpected movement.


### Dexterous Hand
The dexterous hand boasts significant gripping strength and moderate speed, making it suitable for grasping and manipulating tasks in robotics or prosthetic applications. Its combination of power and control allows for effective handling of various objects, similar to the versatility of a human hand, thereby enhancing the functionality of robots or prostheses in performing complex tasks.

| Feature                   | Value   |
|---------------------------|---------|
| Degrees of Freedom        | 6       |
| Number of Joints          | 12      |
| Weight                    | 540 g    |
| Repeatability             | ±0.20 mm |
| Max. Finger Grip Force    | 10 N    |

![A1_size_dexterous_hand](assets/A1_size_dexterous_hand.jpg)


## Robot Care
- **Keep It Clean**: To maintain the robot arm's optimal operating condition and extend its service life, regular cleaning is essential. Use alcohol wipes or clean, damp rags to gently wipe the surface of the robot arm, including the shell, base, and other components. This will effectively remove debris, oil, and other impurities.