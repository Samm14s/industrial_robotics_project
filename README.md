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


## Installation and Build

Clone repository in your ROS workspace which includes MoveIt:
```bash
cd ~/ws_moveit/src
git clone https://github.com/Samm14s/Industrial_Robotics_Project.git
```
Compile and source:
```bash
cd ~/ws_moveit
catkin_make
source ~/ws_moveit/devel/setup.bash
```
## Execution

After the installation and build you can execute in this order:

To plan and execute the trajectory:
```bash
roslaunch panda_moveit_config demo_gazebo.launch
rosrun panda_motion_planning circle_motion
```
Per registrare la posizione dell’end-effector su rosbag:
```bash
rosrun panda_motion_planning EE_Recorder_Node
rosbag record -O executed.bag /joint_states_with_pose
```
Per convertire i dati rosbag in CSV:
```bash
rosrun panda_motion_planning rosbag_to_csv_executed executed.bag
```
