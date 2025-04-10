# A1 FAQ
> This page collects common questions and answers about the Galaxea A1 robot arm. Whether you're a beginner or an experienced developer, you can find practical information here to quickly solve technical problems and improve the use and development efficiency of the robot arm.

## 1. Why doesn't the A1 robot arm reach the set target position?
This situation is normal and may be caused by the following reasons:

1. The target point you set is difficult to reach in reality, and the machine will converge to a nearby achievable position.
2. The MPC convergence direction may be wrong, causing it to stop at the current local optimum instead of reaching the target point. It is recommended to use planned tracking instead of giving a distant target point directly.

## 2. How to reproduce the maximum acceleration of the robot arm's end?
For the demand of reproducing the maximum acceleration of the robot arm's end, you can refer to the following steps.

At the same time, in order to ensure the safety of the operation, please pay attention to the following matters:

- Make sure there are no personnel, obstacles, or other equipment in the working area of the robot arm to avoid collision risks.
- Check if the installation of the robot arm is firm, and if the base and connecting parts are secure.
- Strictly follow the parameter ranges specified in the document when setting acceleration and speed to avoid over-limit operation.
- Monitor the robot arm throughout the testing process. If any abnormality is detected, press the emergency stop button immediately.
- Note that reproducing the maximum acceleration of the robot arm's end may involve high-risk operations. We strongly recommend that you fully understand the relevant risks and take appropriate protective measures before conducting the test. We will not be liable for any losses or injuries caused by failure to follow safety regulations.

Then, follow those steps:

1. Enter the following directory:
    ```Bash
    cd install/share/mobiman/launch/simpleExample
    ```

2. Open the  file:`vim jointTrackerdemo.launch`
    ```Bash
    vim jointTrackerdemo.launch
    ```

3. Add the following code to the corresponding position:
    ```Bash
    <param name="high_tracking_mode" value="high" />
    ```

    ![FAQ_A1_max_acc](./assets/FAQ_assets/FAQ_A1_max_acc.JPEG)
    Save and exit to run.

4. After completing the demo operation, delete the code block added in step 3 and restore to the original settings for your safety.

## 3. Where can I download the STP file for the robot arm's end effector fingers?
You can download the STP file for the gripper fingers from our GitHub community.

Download link: [Finger_GEN2.stp](https://github.com/userguide-galaxea/STP/blob/main/G1/Finger_GEN2.stp)