# A1 Simulation Isaac Sim Usage Tutorial

## Install Isaac Sim 4.0.0

Isaac Sim has specific requirements for GPU versions and drivers. Arbitrary changes may cause issues. Therefore, it is recommended to directly download and install the necessary files.

The download size is approximately 8 GB, so please plan your time accordingly.

### Download Omniverse Launcher

1. **Visit the page:** Go to NVIDIA's official website at https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html and download the Omniverse Launcher.
2. **Register an account:** During the download process, you may be prompted to log in. If you don't have an NVIDIA account, follow the instructions to register. Once registered, continue with the download.
3. **Install** **Omniverse Launcher:** After the download is complete, launch the installer and follow the prompts to complete the installation.
4. **Launch Omniverse Launcher:** Once the installation is complete, open the Omniverse Launcher. For the first time use, you will need to log in with your registered NVIDIA account.
5. **Set up paths:** After logging in, the system will prompt you to configure some related file paths. Set them up according to your needs.
6. **Install** **Cache:** Follow the system prompts to install Cache. It is recommended to choose the default installation option to enhance your experience.
7. **Access the main interface:** Once these settings are completed, you will enter the Omniverse Launcher main interface.

### Install Isaac Sim

1. **Open "Exchange" :** In the Omniverse Launcher main interface, click on the "Exchange" tab.
2. **Find Isaac Sim:** Search the "Isaac Sim" application within the "Exchange".
3. **Select version:** Ensure you install **<u>version 4.0.0</u>** of Isaac Sim to avoid potential issues with the ROS bridge. Please note that the version number might be hidden in a dropdown menu, so carefully check and select the correct version.
4. **Install** **Isaac Sim:** After selecting the version, click the "Install" button to begin the installation of Isaac Sim.

![exchange1_en](assets/exchange1_en.png)

![exchange2_en](assets/exchange2_en.png)

<u>**Important:** The installed applications can be found in the "Library" tab. Simply locate the desired application and launch it from there.</u>

### Isaac Sim Basic Operations

The following is the official tutorial to help you understand the basic Isaac Sim interface.

- **UI** **Interface**
    - [Isaac Sim Interface](https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_interface.html)
    - [Isaac Sim Workflows](https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_workflows.html)
- **Add Objects and** **Set** **Physical Properties**
    - [Add Simple Objects](https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_intro_simple_objects.html#isaac-sim-app-tutorial-intro-simple-objects)
- **Assemble Robots and Import**
    - [Assemble a Simple Robot](https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_simple_robot.html)
    - [URDF Importer](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_urdf.html)



## Importing USD File

First，you need to clone our repositry [A1_Simulation_Isaac_Sim_Usage_Tutorial](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial) in our GitHub.

Visit [A1_Simulation_SDK](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/tree/galaxea/main/A1_simulation_SDK) to get the resources of  [A1_fixed_base_scene.usd](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/blob/galaxea/main/A1_simulation_SDK/A1_fixed_base_scene.usd) and [A1_raw.usd ](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/blob/galaxea/main/A1_simulation_SDK/A1_raw.usd) for A1.

Visit [A1_Simulation_A1 G1_SDK](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/tree/galaxea/main/A1_simulation_A1_G1_SDK) to get the resources of [A1_G1_scene.usd](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/blob/galaxea/main/A1_simulation_A1_G1_SDK/A1_G1_scene.usd) and [A1_G1_raw.usd](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/blob/galaxea/main/A1_simulation_A1_G1_SDK/A1_G1_raw.usd) for A1 with gripper G1.

1. **Open Isaac Sim:** Start Isaac Sim 4.0.0 from Omniverse Launcher. Ensure you select `omni.isaac.ros_bridge(deprecated)` at startup to enable communication between Isaac Sim and ROS nodes.   
   ![library_en](assets/library_en.png)

2. **Open the USD File:** After starting Isaac Sim, select **"File -> Open"**. In the file dialog that appears, choose the `A1_fixed_base_scene.usd` file from the folder if you are using A1, or choose `A1_G1_scene.usd` file if you are using A1 with gripper G1.  
   ![launcher1_en](assets/launcher1_en.png)

3. **Run the Synchronization Script:** After opening the file, you will see the corresponding scene. Click the "**Play"** button on the left sidebar.   
   ![launcher2_en](assets/launcher2_en.png)
   Run the `a1_jointsync.py` script from the folder to synchronize the RViz simulation with the Isaac Sim simulation, which is further elaborated in the following tutorial.    
```shell
  python a1_jointsync.py
```
    <u>**Important:** The Isaac Sim ROS Bridge can only publish/subscribe to `rostopic` when `roscore` is running.</u>
   
## Demonstration Example

After clicking the Play button, start the python script [A1_simulation_SDK/a1_joint_move_sin.py](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/blob/galaxea/main/A1_simulation_SDK/a1_joint_move_sin.py). 

A1 robot arm will begin executing a sinusoidal trajectory in joint space, as shown in the image below. You can also play the controller trajectory by running the python file [A1_simulation_SDK/a1_control_from_traj.py](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/blob/galaxea/main/A1_simulation_SDK/a1_control_from_traj.py). 

This will play the trajectory based on the given data file [A1_simulation_SDK/joint_trajectory.npz](https://github.com/userguide-galaxea/A1_Simulation_Isaac_Sim_Usage_Tutorial/blob/galaxea/main/A1_simulation_SDK/joint_trajectory.npz).

![launcher3_en](assets/launcher3_en.jpg)

With this, the Isaac Sim A1 robot arm simulation process is complete. You may play the demo in the system with the code provided, like shown below.

<img src="../assets/a1_asaacsim_demo.gif" alt="video_git" width="1080" />

## Gripping

After clicking the “Play”, please refer to the [End-Effector Movement Example](https://github.com/userguide-galaxea/A1_SDK/blob/galaxea/main/README_CONTROL.md#end-effector-movement-example) in A1_SDK and clone the A1_SDK repository to complete the simuation process in Isaac Sim.

Take the following code as an example:

1. Trace the end pose and position
```python
##Initiate the end motion script to start one RViz of the arm. Joint position is on zero-point by default.
cd A1_SDK/install
source setup.bash
roslaunch mobiman eeTrackerdemo.launch

##Initiate one terminal,e.g. "terminal_1", open the A1 simulation sync.
python a1_jointsync.py

##Initiate one terminal,e.g. "terminal_2", publish the example trajactory ponits.
rostopic pub /a1_ee_target geometry_msgs/PoseStamped "{
header: {
    seq: 0,
    stamp: {secs: 0, nsecs: 0},
    frame_id: 'world'
},
pose: {
    position: {x: 0.08, y: 0.0, z: 0.5},
    orientation: {x: 0.5, y: 0.5, z: 0.5, w: 0.5}
}
}"
```

2. Gripping the object. demo
```Bash
##Initiate the end motion script to start one RViz of the arm. Joint position is on zero-point by default.
cd A1_SDK/install
source setup.bash
roslaunch mobiman eeTrackerdemo.launch

##Initiate another terminal,e.g. "terminal_3", run the script.
python mpc_picker.py
```

3. Demo Video
<div style="display: flex; justify-content: center; align-items: center;">
<video width="1920" height="1080" controls>
  <source src="../assets/mp4_1.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</div>