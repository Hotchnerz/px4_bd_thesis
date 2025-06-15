#include <ros/ros.h>
#include <flight_test/trajectory_generator.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "flight_test_planner");

  ros::NodeHandle nh;
//   MavPlanner planner("mav_trajectory", nh);
  //FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}