# panda_motion_planning

Progetto di **robotica industriale** sviluppato con **ROS Noetic** e **MoveIt**, per la pianificazione ed esecuzione di traiettorie cartesiane e la registrazione di dati del robot **Franka Emika Panda** in ambiente simulato.

---

## 📦 Contenuto del pacchetto

- 📁 `src/` — Nodi C++ per:
  - `circle_motion.cpp` → pianificazione ed esecuzione di traiettorie circolari
  - `EE_Recorder_Node.cpp` → registrazione della posizione dell’end-effector su rosbag
  - `rosbag_to_csv_executed.cpp` → conversione dei dati rosbag in CSV
- 📁 `msg/` — Messaggi custom:
  - `JointStateWithPose.msg`
  - `TrajectoryPointStamped.msg`
- 📁 `result/` — Dati raccolti e script MATLAB:
  - `executed.csv`
  - `planned.csv`
  - `compare_trajectories.m` per il confronto grafico delle traiettorie

---

## 📋 Dipendenze

Per eseguire il progetto sono richiesti:

- **ROS Noetic**
- **MoveIt**
- Pacchetti ROS:
  - `roscpp`
  - `std_msgs`
  - `geometry_msgs`
  - `sensor_msgs`
  - `rosbag`
  - `tf`
  - `moveit_ros_planning_interface`
  - `moveit_ros_planning`
  - `message_generation`

### 🛠️ Installazione e build

    Clona il repository nel tuo workspace ROS:

cd ~/ws_moveit/src
git clone https://github.com/Samm14s/panda_motion_planning.git

    Compila il workspace:

cd ~/ws_moveit
catkin_make

    Attiva l’ambiente ROS per il workspace:

source devel/setup.bash

---
## 🚀 Esecuzione

Dopo aver compilato e attivato il workspace, puoi eseguire i nodi del pacchetto:

Per pianificare ed eseguire la traiettoria circolare:
```bash
  rosrun panda_motion_planning circle_motion

Per registrare la posizione dell’end-effector su rosbag:
```bash
  rosrun panda_motion_planning EE_Recorder_Node

Per convertire i dati rosbag in CSV:
```bash
  rosrun panda_motion_planning rosbag_to_csv_executed

---
