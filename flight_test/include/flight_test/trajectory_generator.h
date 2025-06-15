#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <flight_test/MavTrajectoryAction.h>
#include <mav_trajectory_generation/trajectory.h>

class MavPlanner {
 protected:
  ros::NodeHandle& nh_;
  actionlib::SimpleActionServer<flight_test::MavTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;

  // create messages that are used to published feedback/result
  flight_test::MavTrajectoryActionFeedback feedback_;
  flight_test::MavTrajectoryActionResult result_;


 public:
  // MavPlanner(ros::NodeHandle& nh);
  MavPlanner(const std::string& name, ros::NodeHandle& nh_);

  void setMaxSpeed(double max_v);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

  void executeCB(const flight_test::MavTrajectoryActionGoalConstPtr& goal);

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  // ros::Subscriber sub_odom_;

  ros::Subscriber local_pose_sub_;
  ros::Subscriber local_vel_sub_;

  void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
  void localVelCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity);

  geometry_msgs::PoseStamped fc_pose_;
  geometry_msgs::TwistStamped fc_vel_;
  bool pose_received_;
  bool odom_received_;

  
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;

};

#endif