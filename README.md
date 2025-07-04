## Executing a Trajectory with the Franka Robot in Gazebo


## Code Structure

    📁 src/ — C++ nodes:

        circle_motion.cpp

        EE_Recorder_Node.cpp

        rosbag_to_csv_executed.cpp

    📁 msg/ — Custom message:

        JointStateWithPose.msg

    📁 results/ — Data analysis:

        executed.csv   #simulation example

        planned.csv    #simulation example

        compare_trajectories.m 
---
## Dipendencies

Project requirements:

    1) ROS Noetic with Packets: 
        - roscpp 
        - std_msgs 
        - geometry_msgs 
        - sensor_msgs 
        - rosbag
        - tf 
        - moveit_ros_planning_interface 
        - moveit_ros_planning 
        - message_generation
    2) MoveIt
    3) Gazebo 11
    4) Matlab 

---
## Installation and Build

Clone repository in a ROS workspace which includes MoveIt:
```bash
cd ~/"ROS-MOVEIT Workspace"/src
git clone https://github.com/Samm14s/Industrial_Robotics_Project.git
```
Compile and source:
```bash
cd ~/"ROS-MOVEIT Workspace"
catkin_make
source ~/"ROS-MOVEIT Workspace"/devel/setup.bash
```
---
## Execution

After the installation and build execute:
```bash
roslaunch panda_moveit_config demo_gazebo.launch
```
To plan and simulate the trajectory:
```bash
rosrun Industrial_Robotics_Project circle_motion
```
To start the node necessary for recording data:
```bash
rosrun Industrial_Robotics_Project EE_Recorder_Node
```
To record in a .bag file
```bash
rosbag record -O executed.bag /joint_states_with_pose
```
To convert data from .bag to .csv:
```bash
rosrun Industrial_Robotics_Project rosbag_to_csv_executed executed.bag
```
---
## Data Analysis
To plot and compare the 2 trajectories run the file compare_trajectories.m in Matlab
