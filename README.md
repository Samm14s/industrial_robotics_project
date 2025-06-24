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

### 📥 Installazione MoveIt (se necessario)

```bash
sudo apt install ros-noetic-moveit
