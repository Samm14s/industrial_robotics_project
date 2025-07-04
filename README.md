## Executing a Trajectory with the Franka Robot in Gazebo

The objective of this project is to make the Franka robot execute a pre-defined trajectory in the Gazebo simulation environment. The project involves creating the trajectory, executing it using ROS (Robot Operating System) with a suitable controller, collecting data from the robot's execution and analyzing the results by comparing the desired trajectory with the executed one.


## Code Structure

    📁 src/ — C++ nodes:

        circle_motion.cpp

        EE_Recorder_Node.cpp

        rosbag_to_csv_executed.cpp

    📁 msg/ — Custom messages:

        JointStateWithPose.msg

        TrajectoryPointStamped.msg

    📁 results/ — Data analysis and comparison:

        executed.csv

        planned.csv

        compare_trajectories.m 
---
## Dipendencies

To be correctly executed the project requires:

    - ROS Noetic with Packets: 
        roscpp, 
        std_msgs, 
        geometry_msgs, 
        sensor_msgs, 
        rosbag, 
        tf, 
        moveit_ros_planning_interface, 
        moveit_ros_planning, 
        message_generation

    - MoveIt
    - Gazebo 11


## Installation and Build

Clone repository in your ROS workspace which includes MoveIt:
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
## Execution

After the installation and build you can execute in this order:

To plan and simulate the trajectory:
```bash
roslaunch panda_moveit_config demo_gazebo.launch
rosrun Industrial_Robotics_Project circle_motion
```
To record data in a .bag file:
```bash
rosrun Industrial_Robotics_Project EE_Recorder_Node
rosbag record -O executed.bag /joint_states_with_pose
```
To convert data from .bag to .csv:
```bash
rosrun Industrial_Robotics_Project rosbag_to_csv_executed executed.bag
```
