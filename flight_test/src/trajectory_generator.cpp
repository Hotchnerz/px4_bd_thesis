#include "ros/ros.h"
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <flight_test/trajectory_generator.h>
#include <actionlib/server/simple_action_server.h>
#include <flight_test/MavTrajectoryAction.h>


//Delete this...
#include <actionlib_tutorials/FibonacciAction.h>

// class MavPlanner
// {
//   ros::NodeHandle nh_;
//   actionlib::SimpleActionServer<flight_test::MavTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
//   std::string action_name_;

//   // create messages that are used to published feedback/result
//   flight_test::MavTrajectoryActionFeedback feedback_;
//   flight_test::MavTrajectoryActionResult result_;

//   //actionlib_tutorials::FibonacciFeedback feedback_;
//   //actionlib_tutorials::FibonacciResult result_;



//   // odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, 
//   //                           &MavPlanner::odomCallback, this);
//     // Initialize subscribers
//   ros::Subscriber local_pose_sub = nh_.subscribe("/mavros/local_position/pose", 10, &localPoseCallback, this);
//   ros::Subscriber local_vel_sub = nh_.subscribe("/mavros/local_position/velocity_local", 10, &localVelCallback, this);


//   void MavPlanner::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {

// }

// void MavPlanner::localVelCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity) {
//     // Your implementation here
// }
//   //Setup Action constructor
//   MavPlanner() :
//     as_(nh_, "mav_trajectory_action", boost::bind(&MavPlanner::executeCB, this, _1), false)
//   {
//     as_.start();
//   }



//   void executeCB(const flight_test::MavTrajectoryActionGoalConstPtr& goal)
//   {
//     // helper variables
//     ros::Rate r(1);
//     bool success = true;

//     // push_back the seeds for the fibonacci sequence
//     feedback_.sequence.clear();
//     feedback_.sequence.push_back(0);
//     feedback_.sequence.push_back(1);

//     // publish info to the console for the user
//     ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

//     // start executing the action
//     for(int i=1; i<=goal->order; i++)
//     {
//       // check that preempt has not been requested by the client
//       if (as_.isPreemptRequested() || !ros::ok())
//       {
//         ROS_INFO("%s: Preempted", action_name_.c_str());
//         // set the action state to preempted
//         as_.setPreempted();
//         success = false;
//         break;
//       }
//       feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
//       // publish the feedback
//       as_.publishFeedback(feedback_);
//       // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
//       r.sleep();
//     }

//     if(success)
//     {
//       result_.sequence = feedback_.sequence;
//       ROS_INFO("%s: Succeeded", action_name_.c_str());
//       // set the action state to succeeded
//       as_.setSucceeded(result_);
//     }
//   }


// };

// MavPlanner::MavPlanner(const std::string& name, ros::NodeHandle &nh_handle) :                                  // private NodeHandle
//     nh_(nh_handle),
//     max_v_(2.0),
//     max_a_(2.0),
//     current_velocity_(Eigen::Vector3d::Zero()),
//     current_pose_(Eigen::Affine3d::Identity()),
//     as_(nh_, name,
//         boost::bind(&MavPlanner::executeCB, this, _1),
//         false),
//         action_name_(name)
//   {
//     as_.start();  // start accepting goals
//     ROS_INFO("Action server '%s' started", action_name_.c_str());

//     local_pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &MavPlanner::localPoseCallback, this);
//     local_vel_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &MavPlanner::localVelCallback, this);

//     pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
//     pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

//   }

// bool MavPlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
//                                     const Eigen::VectorXd& goal_vel,
//                                     mav_trajectory_generation::Trajectory* trajectory) {


//   // 3 Dimensional trajectory => through carteisan space, no orientation
//   const int dimension = 3;

//   // Array for all waypoints and their constrains
//   mav_trajectory_generation::Vertex::Vector vertices;

//   // Optimze up to 4th order derivative (SNAP)
//   const int derivative_to_optimize =
//       mav_trajectory_generation::derivative_order::SNAP;

//   // we have 2 vertices:
//   // Start = current position
//   // end = desired position and velocity
//   mav_trajectory_generation::Vertex start(dimension), end(dimension);


//   /******* Configure start point *******/
//   // set start point constraints to current position and set all derivatives to zero
//   start.makeStartOrEnd(current_pose_.translation(),
//                        derivative_to_optimize);

//   // set start point's velocity to be constrained to current velocity
//   start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
//                       current_velocity_);

//   // add waypoint to list
//   vertices.push_back(start);


//   /******* Configure end point *******/
//   // set end point constraints to desired position and set all derivatives to zero
//   end.makeStartOrEnd(goal_pos,
//                      derivative_to_optimize);

//   // set start point's velocity to be constrained to current velocity
//   end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
//                     goal_vel);

//   // add waypoint to list
//   vertices.push_back(end);

//   // setimate initial segment times
//   std::vector<double> segment_times;
//   segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

//   // Set up polynomial solver with default params
//   mav_trajectory_generation::NonlinearOptimizationParameters parameters;

//   // set up optimization problem
//   const int N = 10;
//   mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
//   opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

//   // constrain velocity and acceleration
//   opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
//   opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

//   // solve trajectory
//   opt.optimize();

//   // get trajectory as polynomial parameters
//   opt.getTrajectory(&(*trajectory));

//   return true;
// }

// bool MavPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
//   // send trajectory as markers to display them in RVIZ
//   visualization_msgs::MarkerArray markers;
//   double distance =
//       0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
//   std::string frame_id = "world";

//   mav_trajectory_generation::drawMavTrajectory(trajectory,
//                                                distance,
//                                                frame_id,
//                                                &markers);
//   pub_markers_.publish(markers);

//   // send trajectory to be executed on UAV
//   mav_planning_msgs::PolynomialTrajectory msg;
//   mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
//                                                                  &msg);
//   msg.header.frame_id = "world";
//   pub_trajectory_.publish(msg);

//   return true;
// }

  
void MavPlanner::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
  tf::poseMsgToEigen(pose->pose, current_pose_);

}

void MavPlanner::localVelCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity){
  tf::vectorMsgToEigen(velocity->twist.linear, current_velocity_);

}

// void MavPlanner::executeCB(const flight_test::MavTrajectoryActionGoalConstPtr& goal){
//     // helper variables
//     ros::Rate r(1);
//     bool success = true;

//   //   // push_back the seeds for the fibonacci sequence
//   //   feedback_.sequence.clear();
//   //   feedback_.sequence.push_back(0);
//   //   feedback_.sequence.push_back(1);

//   //   // publish info to the console for the user
//   //   ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

//   //   // start executing the action
//   //   for(int i=1; i<=goal->order; i++)
//   //   {
//   //     // check that preempt has not been requested by the client
//   //     if (as_.isPreemptRequested() || !ros::ok())
//   //     {
//   //       ROS_INFO("%s: Preempted", action_name_.c_str());
//   //       // set the action state to preempted
//   //       as_.setPreempted();
//   //       success = false;
//   //       break;
//   //     }
//   //     feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
//   //     // publish the feedback
//   //     as_.publishFeedback(feedback_);
//   //     // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
//   //     r.sleep();
//   //   }

//   //   if(success)
//   //   {
//   //     result_.sequence = feedback_.sequence;
//   //     ROS_INFO("%s: Succeeded", action_name_.c_str());
//   //     // set the action state to succeeded
//   //     as_.setSucceeded(result_);
//   //   }
//   // }
// }