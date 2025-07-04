#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <ros/package.h>

using namespace std;

void scaleTrajectorySpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale){
  auto &traj = plan.trajectory_.joint_trajectory;
  size_t n_joints = traj.joint_names.size(); // #joints
  
  for(auto &pt : traj.points){
    pt.time_from_start = ros::Duration(pt.time_from_start.toSec() / scale); // scale the time
    for(size_t j = 0; j < n_joints; ++j){
      pt.velocities[j] /= scale; // scale the velocity
      if(pt.accelerations.size() > j) pt.accelerations[j] /= (scale * scale); // scale the acceleration if present
    }
  }
}

void appendTrajectory(moveit_msgs::RobotTrajectory& base_traj, const moveit_msgs::RobotTrajectory& new_traj){
  if(base_traj.joint_trajectory.points.empty()){
    base_traj = new_traj;
    return;
  }

  ros::Duration last_time = base_traj.joint_trajectory.points.back().time_from_start;
  for(auto pt : new_traj.joint_trajectory.points){
    pt.time_from_start += last_time;  //adjust the time values so that they continue from where the base_traj finishes
    base_traj.joint_trajectory.points.push_back(pt);
  }
}

void saveTrajectoryToCSV(const moveit_msgs::RobotTrajectory& trajectory, const moveit::planning_interface::MoveGroupInterface& move_group, ofstream& file, bool write_header = false){
  const robot_state::JointModelGroup* joint_group = move_group.getCurrentState()->getJointModelGroup("panda_arm");
  auto joint_names = move_group.getVariableNames();
  
  if(write_header){
    file << "time_from_start";
    for(const auto& name : joint_names){
 	file << "," << name << "_pos";
    }
    for(const auto& name : joint_names){
        file << "," << name << "_vel";
    }
    file << ",ee_pos_x,ee_pos_y,ee_pos_z,ee_ori_x,ee_ori_y,ee_ori_z,ee_ori_w\n";
  }

  for(const auto& point : trajectory.joint_trajectory.points){
    file << point.time_from_start.toSec();
    int n = point.positions.size();  // numero joint
    for(int i = 0; i < n; ++i){
        file << "," << point.positions[i] << "," << point.velocities[i];
    }

    robot_state::RobotState current_state(*move_group.getCurrentState());
    current_state.setJointGroupPositions(joint_group, point.positions);
    const Eigen::Isometry3d& ee_tf = current_state.getGlobalLinkTransform(move_group.getEndEffectorLink());
    
    Eigen::Quaterniond q(ee_tf.rotation());
    file << "," << ee_tf.translation().x()
         << "," << ee_tf.translation().y()
         << "," << ee_tf.translation().z()
         << "," << q.x()
         << "," << q.y()
         << "," << q.z()
         << "," << q.w() << "\n";
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "circle_motion_planning");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
  //ROS_INFO_STREAM("End effector link: " << move_group.getEndEffectorLink());
  double r = 0.1;
  double speed_factor = 1;
  int n_points = 100;
  const double eef_step = 0.01;

  ros::Duration(1.0).sleep();

  geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
  ROS_INFO("Home: [%.3f, %.3f, %.3f]", start_pose.position.x, start_pose.position.y, start_pose.position.z);

  // Waypoints circolari
  vector<geometry_msgs::Pose> waypoints;
  for(int i = 0; i <= n_points; ++i){
    double theta = 2 * M_PI * i / n_points;
    geometry_msgs::Pose p = start_pose;
    p.position.x += r * cos(theta);
    p.position.z += r * sin(theta);
    waypoints.push_back(p);
  }

  // Pianifica traiettoria circolare
  moveit_msgs::RobotTrajectory circ_traj;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, circ_traj);
  ROS_INFO("Circular path planned: %.2f%%", fraction * 100.0);

  string path = ros::package::getPath("panda_motion_planning") + "/results/desired.csv";
  ROS_INFO_STREAM("Scrivo il CSV in: " << path);  
  ofstream csv_file(path);
  if(!csv_file.is_open()){
    ROS_ERROR("Unable to open CSV file at %s", path.c_str());
    return 1;
  }


  if(fraction > 0.9){
    // Crea plan traiettoria circolare
    moveit::planning_interface::MoveGroupInterface::Plan plan_circ;
    plan_circ.trajectory_ = circ_traj;
    scaleTrajectorySpeed(plan_circ, speed_factor);

    //Inizializza total_trajectory 
    moveit_msgs::RobotTrajectory total_traj = plan_circ.trajectory_;
    
    // Esegue traiettoria circolare
    move_group.execute(plan_circ);
    move_group.setStartStateToCurrentState();

    // Pianifica ritorno a home
    vector<geometry_msgs::Pose> home_waypoints;
    home_waypoints.push_back(start_pose);
    moveit_msgs::RobotTrajectory home_traj;
    double home_fraction = move_group.computeCartesianPath(home_waypoints, eef_step, home_traj);

    if (home_fraction > 0.9){
      //Crea plan e esegue ritorno a home
      moveit::planning_interface::MoveGroupInterface::Plan plan_home;
      plan_home.trajectory_ = home_traj;
      scaleTrajectorySpeed(plan_home, speed_factor);
      appendTrajectory(total_traj, plan_home.trajectory_);
      move_group.execute(plan_home);
    } else {
      ROS_WARN("Return to home path planning failed.");
    }
    //Scrive il CSV completo
    saveTrajectoryToCSV(total_traj, move_group, csv_file, true);
  } else {
    ROS_WARN("Circular path planning failed.");
  }

  csv_file.close();
  ros::shutdown();
  return 0;
}
