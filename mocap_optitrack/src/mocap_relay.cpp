#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher mocap_pose;

void relay_func(const geometry_msgs::PoseStamped msg){
mocap_pose.publish(msg);
}

int main(int argc, char **argv)
{

	//initialize ros
	ros::init(argc, argv, "mocap_relay");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("mocap_node/Robot_1/pose", 2, relay_func);
	mocap_pose=n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose",2);
	
	ros::spin();
}


