# CoHEXist_HRI_Simulation

A Gazebo-based simulation environment replicating the [CoHEXist][3] setup was created to evaluate industrial human-robot interactions (HRI). [actor_collisions][12] plugin allowed actor to have collision properties. [GMapping][4] + [AMCL][5] was selected as 2D LiDAR-based mapping and localization strategies due to its low computational load and high stability. The ROS [move_base][6] framework served as the core navigation system, employing the Dijkstra algorithm as global planner and Time Elastic Band ([TEB][7]) as local planner, and the costmap-based mechanism for obstacle avoidance. The [waterplus_map_tools][9] was used to enable robot waypoint navigation. [leg_tracker][8] package was used to detect People messages and the advanced costmap layers such as [virtual_wall_layer][11] and [social_navigation_layers][10] for social-aware navigation were explored.<br />

### System requirements
[Ubuntu 20.04][1] and [ROS Noetic][2]

[1]: https://releases.ubuntu.com/focal/                                          "Ubuntu 20.04"
[2]: http://wiki.ros.org/noetic/Installation/Ubuntu                              "ROS Noetic"
[3]: https://www.researchgate.net/profile/Sergey-Yurish/publication/378144846_Proceedings_of_the_4th_IFSA_Winter_Conference_on_Automation_Robotics_and_Communications_for_Industry_4050_ARCI_2024/links/65c9fe381bed776ae34ac345/Proceedings-of-the-4th-IFSA-Winter-Conference-on-Automation-Robotics-and-Communications-for-Industry-40-50-ARCI-2024.pdf#page=280         "CoHEXist"
[4]: https://openslam-org.github.io/gmapping.html                                "GMapping"
[5]: https://wiki.ros.org/amcl                                                   "AMCL"
[6]: https://wiki.ros.org/move_base                                              "move_base"
[7]: https://wiki.ros.org/teb_local_planner                                      "TEB"
[8]: https://github.com/angusleigh/leg_tracker                                   "leg_tracker"
[9]: https://github.com/6-robot/waterplus_map_tools                              "waterplus_map_tools"
[10]: https://wiki.ros.org/social_navigation_layers                              "social_navigation_layers"
[11]: https://github.com/DylanLN/virtual_wall                                    "virtual_wall_layer"
[12]: https://github.com/JiangweiNEU/actor_collisions                            "actor_collisions"



### Posture generation
We define a custom pose (which contains end-effector’s position and orientation) for the
robot by dragging the marker (that locates at robot’s end-effector) in RViz to a specific pose and we implement a ROS node to extract the orientation and position values of this pose at real-time.
```bash
source devel/setup.bash
roslaunch franka_gazebo panda.launch controller:=cartesian_impedance_example_controller     rviz:=true
```
The command above opens Gazebo and RViz at the same time, we can then drag the marker to move the robot:

<p align="center">
  <img src="pictures/rviz pose.png" width="350" title="hover text">
</p>

Now open another terminal and input:
```bash
source devel/setup.bash
rosrun panda_clik_control end_effector_orientation
```
The orientation matrix and translation vector appears as follows:
<p align="center">
  <img src="pictures/rviz pose read.png" width="350" title="hover text">
</p>



### Results

Given the same goal pose (position and orientation extracted from RViz), the following images show the 
qualitative comparison between the results: <br />
(first: Jacobian transpose; second: Pseudoinverse; third: Damped least squares)

<div align="center">
  <p style="text-align:center;">IKs with the first goal pose</p>

  <img src="gif folder/11.gif" width="300" alt="First GIF" />
  <img src="gif folder/21.gif" width="300" alt="Second GIF" />
  <img src="gif folder/31.gif" width="300" alt="Third GIF" />

  <p style="text-align:center;">IKs with the second goal pose</p>

  <img src="gif folder/12.gif" width="300" alt="First GIF" />
  <img src="gif folder/22.gif" width="300" alt="Second GIF" />
  <img src="gif folder/32.gif" width="300" alt="Third GIF" />

</div>

## How to Install

### 1. Install Dependencies
Install Franka description and control packages:
```bash
sudo apt-get install ros-noetic-franka-description ros-noetic-franka-ros
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
```
To start a example of franka panda in Gazebo:
```bash
roslaunch franka_gazebo panda.launch controller:=cartesian_impedance_example_controller     rviz:=true
```
Install Pinocchio library:
```bash
sudo apt install ros-noetic-pinocchio
```
After installing the ros-noetic-franka-description, the robot arm panda.urdf.xacro file is typically located in the ‘share’ directory of the package:
```bash
opt/ros/noetic/share/franka_description/robots/panda/panda.urdf.xacro
```
The ‘.xacro’ file is a format used to simplify the creation of complex URDF file. Before using this file, it must be converted into a standart ‘.urdf’ file: 
```bash
rosrun xacro xacro panda.urdf.xacro > panda.urdf
```
### 2. Create the clik control ROS package
Create a ROS package: panda_clik_control
```bash
cd ~/catkin_ws/src
catkin_create_pkg panda_clik_control roscpp sensor_msgs
cd panda_clik_control
cd src
```
Copy the source files (.cpp files) from here [src][3] to your panda_clik_control/src directory.<br />
Replace the 'CMakeLists.txt' which is auto-generated with command catkin_create_pkg before with this file [CMakeLists.txt][4] to include necessary dependencies and build instructions. <br />
Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
### 3. Build Panda-gazebo from source inside catkin_ws folder
To move the panda arm inside Gazebo, we need to build [panda_gazebo][5] from source to utilize its features:
```bash
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/rickstaa/panda-gazebo.git
cd ..
catkin_make
source devel/setup.bash
```
## How to run the code

### Part 1. Read the pose using end_effector_orientation node
Open a terminal and input:
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch franka_gazebo panda.launch controller:=cartesian_impedance_example_controller     rviz:=true
```
Open another terminal and input:
```bash
source devel/setup.bash
rosrun panda_clik_control end_effector_orientation
```
You have to manually copy the orientation matrix and translation vector printed here and paste it inside source files (.cpp files) to replace the previous one, and compile panda_clik_control again to have it working.

### Part 2. CLIK Control
Open a terminal and input:
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch panda_gazebo start_simulation.launch
```
Open another terminal and input:
```bash
source devel/setup.bash
rosrun panda_clik_control 1_JacobianTranspose
```
You can change the 1_JacobianTranspose to 2_Pseudoinverse or 3_DampedLeastSquares to experiment different IK methods.




## References

[3] Niessen, N., & Bengler, K. (2024). *CoHEXist--Introducing a New Test Setup for Coexistent Interactions with Mobile Robots in Open Space Encounters*.


        























