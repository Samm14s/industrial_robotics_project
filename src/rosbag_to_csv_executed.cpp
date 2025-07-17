#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <industrial_robotics_project/JointStateWithPose.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cstdlib>

using namespace std;

int main(int argc, char** argv){
    if(argc < 2){
        cerr << "Usage: rosrun your_package rosbag_to_csv <input.bag> [output.csv]" << endl;
        return 1;
    }

    string bag_file = argv[1];

    string output_file = ros::package::getPath("industrial_robotics_project") + "/results/executed.csv";

    rosbag::Bag bag;
    try{
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (...){
        cerr << "Failed to open bag file: " << bag_file << endl;
        return 1;
    }

    vector<string> topics = {"/joint_states_with_pose"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ofstream ofs(output_file);
    if(!ofs.is_open()){
        cerr << "Cannot open output file: " << output_file << endl;
        return 1;
    }

    ofs << "time_from_start";
    for(int i = 1; i <= 7; ++i){
        ofs << ",panda_joint" << i << "_pos,panda_joint" << i << "_vel";
    }
    ofs << ",eef_pos_x,eef_pos_y,eef_pos_z,eef_orient_x,eef_orient_y,eef_orient_z,eef_orient_w\n";
    
    double t_start = -1;
    for (const rosbag::MessageInstance& m : view){
        industrial_robotics_project::JointStateWithPose::ConstPtr msg = m.instantiate<industrial_robotics_project::JointStateWithPose>();
        if (msg){
            double t = msg->header.stamp.toSec();
            if (t_start < 0) t_start = t;
        	
            double time_from_start = t - t_start;
	    ofs << time_from_start;
	    if (msg->joint_state.position.size() >= 7 && msg->joint_state.velocity.size() >= 7) {
                for (int i = 0; i < 7; ++i) {
               	    ofs << "," << msg->joint_state.position[i] << "," << msg->joint_state.velocity[i];
                }
            } else {
              	for (int i = 0; i < 7; ++i) ofs << ",0,0";
            }		
            ofs << "," << msg->ee_pose.pose.position.x << "," << msg->ee_pose.pose.position.y << "," << msg->ee_pose.pose.position.z;
            ofs << "," << msg->ee_pose.pose.orientation.x << "," << msg->ee_pose.pose.orientation.y << "," << msg->ee_pose.pose.orientation.z << "," << msg->ee_pose.pose.orientation.w << "\n";
        }
    }
	
    ofs.close();
    bag.close();
    cout << "CSV file saved: " << output_file << endl;
    return 0;
}
