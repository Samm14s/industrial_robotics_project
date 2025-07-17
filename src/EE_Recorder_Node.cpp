#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <industrial_robotics_project/JointStateWithPose.h> // create custom message


class EERecorderNode{
 public:
     EERecorderNode() : tfListener(tfBuffer){
         jointSub = nh.subscribe("/joint_states", 10, &EERecorderNode::jointCallback, this);
         pub = nh.advertise<industrial_robotics_project::JointStateWithPose>("/joint_states_with_pose", 10);
     }

 private:
     ros::NodeHandle nh;
     ros::Subscriber jointSub;
     ros::Publisher pub;
     tf2_ros::Buffer tfBuffer;
     tf2_ros::TransformListener tfListener;
 	
     void jointCallback(const sensor_msgs::JointState::ConstPtr& jointMsg){
         geometry_msgs::TransformStamped transformStamped;
	 try{
             transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_link8", ros::Time(0), ros::Duration(0.1));
         }
         catch (tf2::TransformException &ex){
             ROS_WARN("Could not get transform: %s", ex.what());
             return;
         }

         // Custom message
         industrial_robotics_project::JointStateWithPose msg;
         msg.header.stamp = jointMsg->header.stamp;
       	 msg.joint_state = *jointMsg;

         // Copy end-effector pose
	 msg.ee_pose.header = transformStamped.header;
	 msg.ee_pose.pose.position.x = transformStamped.transform.translation.x;
       	 msg.ee_pose.pose.position.y = transformStamped.transform.translation.y;
    	 msg.ee_pose.pose.position.z = transformStamped.transform.translation.z;
     	 msg.ee_pose.pose.orientation = transformStamped.transform.rotation;

       	 pub.publish(msg);
      }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "ee_recorder_node");
    EERecorderNode node;
    ros::spin();
    return 0;
}
