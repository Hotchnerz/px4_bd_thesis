#!/usr/bin/env python3
"""
Single-node implementation showing 3D cubic trajectory interpolation
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Vector3
from threading import Lock
import math

class TrajectoryPoint:
    """Represents a 3D trajectory point with timestamp"""
    def __init__(self, x=0, y=0, z=0, yaw=0, timestamp=None):
        self.position = np.array([x, y, z], dtype=float)
        self.yaw = yaw
        self.timestamp = timestamp if timestamp else rospy.Time.now()
    
    def distance_to(self, other):
        """Calculate Euclidean distance to another point"""
        return np.linalg.norm(self.position - other.position)
    
    def to_pose_stamped(self):
        """Convert to ROS PoseStamped message"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1] 
        pose.pose.position.z = self.position[2]
        # Could add yaw to orientation here
        return pose

class CubicTrajectory3D:
    """
    Handles smooth 3D cubic interpolation between trajectory points
    """
    def __init__(self, trajectory_speed=0.5):
        self.trajectory_speed = trajectory_speed  # m/s
        self.trajectory_lock = Lock()
        
        # Trajectory state
        self.trajectory_active = False
        self.start_point = TrajectoryPoint()
        self.target_point = TrajectoryPoint()
        self.current_point = TrajectoryPoint()
        
        # Timing
        self.trajectory_start_time = None
        self.trajectory_duration = 0.0
        self.min_trajectory_time = 1.0  # Minimum time for any trajectory
        
    def start_trajectory(self, target_point):
        """Start a new trajectory to the target point"""
        with self.trajectory_lock:
            self.start_point = TrajectoryPoint(
                self.current_point.position[0],
                self.current_point.position[1], 
                self.current_point.position[2],
                self.current_point.yaw
            )
            self.target_point = target_point
            
            # Calculate trajectory duration based on 3D distance
            distance = self.start_point.distance_to(self.target_point)
            self.trajectory_duration = max(
                distance / self.trajectory_speed, 
                self.min_trajectory_time
            )
            
            self.trajectory_start_time = rospy.Time.now()
            self.trajectory_active = True
            
            rospy.loginfo("Starting 3D trajectory: %.2fm over %.2fs", 
                         distance, self.trajectory_duration)
            rospy.loginfo("  From: (%.2f, %.2f, %.2f)", 
                         self.start_point.position[0],
                         self.start_point.position[1], 
                         self.start_point.position[2])
            rospy.loginfo("  To: (%.2f, %.2f, %.2f)",
                         self.target_point.position[0],
                         self.target_point.position[1],
                         self.target_point.position[2])
    
    def update_trajectory(self):
        """Update current position along trajectory. Returns True when complete."""
        if not self.trajectory_active:
            return False
            
        with self.trajectory_lock:
            elapsed = (rospy.Time.now() - self.trajectory_start_time).to_sec()
            progress = elapsed / self.trajectory_duration
            
            if progress >= 1.0:
                # Trajectory complete - snap to final position
                self.current_point.position = np.copy(self.target_point.position)
                self.current_point.yaw = self.target_point.yaw
                self.trajectory_active = False
                rospy.loginfo("3D trajectory complete at (%.2f, %.2f, %.2f)",
                             self.current_point.position[0],
                             self.current_point.position[1], 
                             self.current_point.position[2])
                return True
            
            # Apply cubic ease-in-out to progress
            smooth_progress = self.cubic_ease_in_out(progress)
            
            # 3D interpolation using vectorized operations
            self.current_point.position = (
                self.start_point.position + 
                smooth_progress * (self.target_point.position - self.start_point.position)
            )
            
            # Interpolate yaw angle (handling wrap-around)
            yaw_diff = self.target_point.yaw - self.start_point.yaw
            # Handle yaw wrap-around (choose shortest rotation)
            if yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            elif yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            self.current_point.yaw = self.start_point.yaw + smooth_progress * yaw_diff
            
            return False
    
    @staticmethod
    def cubic_ease_in_out(t):
        """Cubic ease-in-out interpolation function"""
        return t * t * (3.0 - 2.0 * t)
    
    def get_current_pose(self):
        """Get current trajectory position as PoseStamped"""
        with self.trajectory_lock:
            return self.current_point.to_pose_stamped()
    
    def get_velocity_vector(self):
        """Calculate current velocity vector for debugging/monitoring"""
        if not self.trajectory_active:
            return Vector3(0, 0, 0)
            
        with self.trajectory_lock:
            elapsed = (rospy.Time.now() - self.trajectory_start_time).to_sec()
            progress = elapsed / self.trajectory_duration
            
            if progress >= 1.0:
                return Vector3(0, 0, 0)
            
            # Derivative of cubic ease-in-out: 6t(1-t)
            velocity_scale = 6 * progress * (1 - progress) / self.trajectory_duration
            
            # Direction vector
            direction = self.target_point.position - self.start_point.position
            
            # Current velocity vector
            velocity = velocity_scale * direction
            
            vel_msg = Vector3()
            vel_msg.x = velocity[0]
            vel_msg.y = velocity[1]
            vel_msg.z = velocity[2]
            return vel_msg
    
    def is_active(self):
        """Check if trajectory is currently active"""
        return self.trajectory_active
    
    def stop_trajectory(self):
        """Emergency stop - halt at current position"""
        with self.trajectory_lock:
            self.trajectory_active = False
            rospy.logwarn("Trajectory stopped at current position")

# Usage example in FSM controller
class FSMControllerWithCubic:
    def __init__(self):
        rospy.init_node('fsm_cubic_controller')
        
        # Initialize 3D cubic trajectory handler
        self.trajectory = CubicTrajectory3D(trajectory_speed=0.5)
        
        # ROS publishers for monitoring
        self.pose_pub = rospy.Publisher('/trajectory/current_pose', PoseStamped, queue_size=1)
        self.velocity_pub = rospy.Publisher('/trajectory/current_velocity', Vector3, queue_size=1)
        
        # Initialize current position (e.g., from takeoff)
        self.trajectory.current_point = TrajectoryPoint(0, 0, 1.5)
        
    def move_to_waypoint(self, x, y, z, yaw=0):
        """Command to move to a new waypoint"""
        target = TrajectoryPoint(x, y, z, yaw)
        self.trajectory.start_trajectory(target)
    
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(50)  # 50Hz for smooth trajectories
        
        while not rospy.is_shutdown():
            # Update trajectory
            trajectory_complete = self.trajectory.update_trajectory()
            
            # Handle trajectory completion (trigger FSM events)
            if trajectory_complete:
                self.handle_waypoint_reached()
            
            # Publish current pose and velocity for monitoring
            current_pose = self.trajectory.get_current_pose()
            current_velocity = self.trajectory.get_velocity_vector()
            
            self.pose_pub.publish(current_pose)
            self.velocity_pub.publish(current_velocity)
            
            rate.sleep()
    
    def handle_waypoint_reached(self):
        """Handle when waypoint is reached - trigger FSM transition"""
        rospy.loginfo("Waypoint reached - triggering FSM event")
        # This would trigger your FSM transition
        # e.g., self.waypoint_reached() if using transitions library

# Example usage:
if __name__ == '__main__':
    controller = FSMControllerWithCubic()
    
    # Example trajectory sequence
    rospy.sleep(2)  # Wait for initialization
    controller.move_to_waypoint(1.2, 0, 1.5)    # First waypoint
    rospy.sleep(5)  # Wait for completion
    controller.move_to_waypoint(1.2, 1.0, 1.5)  # Second waypoint  
    rospy.sleep(5)
    controller.move_to_waypoint(0, 0, 1.5)      # Return home
    
    controller.run()