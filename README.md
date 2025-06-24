## Executing a Trajectory with the Franka Robot in Gazebo

The objective of this project is to make the Franka robot execute a pre-defined trajectory in the Gazebo simulation environment. The project involves creating the trajectory, executing it using ROS (Robot Operating System) with a suitable controller, collecting data from the robot's execution and analyzing the results by comparing the desired trajectory with the executed one.


## 📦 Code Structure

    📁 src/ — C++ nodes:

        circle_motion.cpp → Plan and execute the trajectory

        EE_Recorder_Node.cpp → registrazione della posizione dell’end-effector su rosbag

        rosbag_to_csv_executed.cpp → Convert rosbag data in .CSV

    📁 msg/ — Custom messages:

        JointStateWithPose.msg

        TrajectoryPointStamped.msg

    📁 results/ — Data analysis and comparison:

        executed.csv

        planned.csv

        compare_trajectories.m per il confronto grafico delle traiettorie
---
## 📋 Dipendenze

Per eseguire il progetto sono richiesti:

    ROS Noetic

    MoveIt

    Pacchetti ROS:

        roscpp

        std_msgs

        geometry_msgs

        sensor_msgs

        rosbag

        tf

        moveit_ros_planning_interface

        moveit_ros_planning

        message_generation

## 📥 Installazione MoveIt (se necessario)
```bash
sudo apt install ros-noetic-moveit
```
## 🛠️ Installazione e build

Clona il repository nel tuo workspace ROS:
```bash
cd ~/ws_moveit/src
git clone https://github.com/Samm14s/panda_motion_planning.git
```
Compila il workspace:
```bash
cd ~/ws_moveit
catkin_make
```
Attiva l’ambiente ROS per il workspace:
```bash
source ~/ws_moveit/devel/setup.bash
```
## 🚀 Esecuzione

Dopo aver compilato e attivato il workspace, puoi eseguire i nodi del pacchetto:
Per pianificare ed eseguire la traiettoria circolare:
```bash
rosrun panda_motion_planning circle_motion
```
Per registrare la posizione dell’end-effector su rosbag:
```bash
rosrun panda_motion_planning EE_Recorder_Node
```
Per convertire i dati rosbag in CSV:
```bash
rosrun panda_motion_planning rosbag_to_csv_executed
```
