## Executing a Circumference with the Franka Robot in Gazebo
---
## Code Structure
     src/ — C++ nodes and files:

        circle_motion.cpp

        EE_Recorder_Node.cpp

        rosbag_to_csv_executed.cpp

     msg/ — Custom message:

        JointStateWithPose.msg

     results/ — Data analysis:

        executed.csv   #simulation example

        desired.csv    #simulation example

        compare_trajectories.m 
---
## Dipendencies
Project requirements:

    1) ROS Noetic with Packets: 
        - roscpp 
        - std_msgs 
        - geometry_msgs 
        - moveit_ros_planning_interface 
        - moveit_ros_planning
        - rosbag 
        - tf 
        - sensor_msgs 
        - message_generation
        - roslib
    2) MoveIt
    3) Gazebo 11
    4) Matlab 

---
## Installation and Build
Clone the repository in a ROS workspace which includes MoveIt ("ROS-MOVEIT Workspace"):
```bash
cd ~/"ROS-MOVEIT Workspace"/src
git clone https://github.com/Samm14s/industrial_robotics_project.git
```
Compile and source:
```bash
cd ~/"ROS-MOVEIT Workspace"
catkin_make
source ~/"ROS-MOVEIT Workspace"/devel/setup.bash
```
---
## Execution
After the installation and build run:
```bash
roslaunch panda_moveit_config demo_gazebo.launch
```
To plan and simulate the trajectory:
```bash
rosrun industrial_robotics_project circle_motion
```
To record the data first run:
```bash
rosrun industrial_robotics_project EE_Recorder_Node
```
Then to store them in a .bag file:
```bash
rosbag record -O executed.bag /joint_states_with_pose
```
To convert .bag to .csv:
```bash
rosrun industrial_robotics_project rosbag_to_csv_executed executed.bag
```
---
## Data Analysis
To plot and compare the trajectories execute the Matlab script:
```bash
compare_trajectories.m 
```
