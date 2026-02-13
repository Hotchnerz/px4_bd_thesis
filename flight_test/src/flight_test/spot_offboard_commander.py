#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, Pose, TwistStamped, Twist, Point, TransformStamped, PoseArray
from mavros_msgs.msg import State, ExtendedState, AttitudeTarget
# from tf.transformations import euler_from_quaternion #Cannot use due to melodic pkgs being built with python2
from flight_test.transform_utils import euler_from_quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandLong, CommandLongRequest
from fg40_msgs.msg import FG40Feedback, FG40MagnetCmd
from aruco_msgs.msg import MarkerArray, Marker
import numpy as np
from transitions import Machine
from transitions.extensions.states import add_state_features, Timeout
from flight_test.srv import mission, missionResponse
import copy

# Configuration Management
class MissionConfig:
    """Centralized configuration for mission parameters"""
    # Flight parameters
    FLIGHT_HEIGHT = 1.25
    TAKEOFF_SPEED = 0.3
    APPROACH_SPEED = 0.3
    LANDING_SPEED = 0.2
    DEFAULT_SPEED = 0.5
    
    # Tolerances
    POSITION_TOLERANCE = 0.05
    VELOCITY_TOLERANCE = 0.04
    ATTITUDE_TOLERANCE = 0.05
    
    # Timing
    PREP_LAND_TIME_SEC = 2.0
    MOVING_AVG_WINDOW = 20
    MAX_SCAN_ATTEMPTS = 25
    MAX_MARKER_DETECT_ATTEMPTS = 25
    
    # Publishing rates
    SETPOINT_RATE_HZ = 20
    CONTROL_RATE_HZ = 1.5
    
    # Trajectory
    MIN_TRAJECTORY_TIME = 1.0

class SmoothTrajectoryManager:
    """Handles smooth cubic interpolation between waypoints"""
    
    def __init__(self, default_speed=MissionConfig.DEFAULT_SPEED):
        self.trajectory_lock = threading.RLock()
        self.trajectory_active = False
        self.default_speed = default_speed
        
        # Trajectory state
        self.start_pose = Pose()
        self.target_pose_smooth = Pose()
        self.current_smooth_pose = Pose()
        
        # Timing
        self.trajectory_start_time = None
        self.trajectory_duration = 0.0
        self.min_trajectory_time = MissionConfig.MIN_TRAJECTORY_TIME
        
    def start_smooth_trajectory(self, target_pose, speed=None, trajectory_type="waypoint"):
        """Start smooth trajectory to target pose"""
        with self.trajectory_lock:
            # Use current OffboardControl.target_pose as start
            self.start_pose = copy.deepcopy(OffboardControl.target_pose)
            self.target_pose_smooth = copy.deepcopy(target_pose)
            
            # Calculate trajectory duration based on 3D distance
            distance = self.calculate_3d_distance(self.start_pose, self.target_pose_smooth)
            
            # Adjust speed based on trajectory type
            if trajectory_type == "takeoff":
                effective_speed = MissionConfig.TAKEOFF_SPEED
            elif trajectory_type == "landing":
                effective_speed = MissionConfig.LANDING_SPEED
            elif trajectory_type == "approach":
                effective_speed = MissionConfig.APPROACH_SPEED
            else:
                effective_speed = speed if speed else self.default_speed
            
            self.trajectory_duration = max(
                distance / effective_speed,
                self.min_trajectory_time
            )
            
            self.trajectory_start_time = rospy.Time.now()
            self.trajectory_active = True
            
            rospy.loginfo(f"Starting smooth {trajectory_type}: {distance:.2f}m over {self.trajectory_duration:.2f}s")
            rospy.loginfo(f"  From: ({self.start_pose.position.x:.2f}, {self.start_pose.position.y:.2f}, {self.start_pose.position.z:.2f})")
            rospy.loginfo(f"  To: ({self.target_pose_smooth.position.x:.2f}, {self.target_pose_smooth.position.y:.2f}, {self.target_pose_smooth.position.z:.2f})")
    
    def update_smooth_trajectory(self):
        """Update smooth trajectory - returns True when complete"""
        if not self.trajectory_active:
            return False
            
        with self.trajectory_lock:
            elapsed = (rospy.Time.now() - self.trajectory_start_time).to_sec()
            progress = elapsed / self.trajectory_duration
            
            if progress >= 1.0:
                # Trajectory complete - snap to final position
                OffboardControl.target_pose = copy.deepcopy(self.target_pose_smooth)
                self.current_smooth_pose = copy.deepcopy(self.target_pose_smooth)
                self.trajectory_active = False
                
                rospy.loginfo(f"Smooth trajectory complete at ({OffboardControl.target_pose.position.x:.2f}, {OffboardControl.target_pose.position.y:.2f}, {OffboardControl.target_pose.position.z:.2f})")
                return True
            
            # Apply cubic ease-in-out to progress
            smooth_progress = self.cubic_ease_in_out(progress)
            
            # 3D position interpolation
            OffboardControl.target_pose.position.x = (
                self.start_pose.position.x + 
                smooth_progress * (self.target_pose_smooth.position.x - self.start_pose.position.x)
            )
            OffboardControl.target_pose.position.y = (
                self.start_pose.position.y + 
                smooth_progress * (self.target_pose_smooth.position.y - self.start_pose.position.y)
            )
            OffboardControl.target_pose.position.z = (
                self.start_pose.position.z + 
                smooth_progress * (self.target_pose_smooth.position.z - self.start_pose.position.z)
            )
            
            # Orientation interpolation (SLERP would be better, but this works)
            OffboardControl.target_pose.orientation.x = (
                self.start_pose.orientation.x + 
                smooth_progress * (self.target_pose_smooth.orientation.x - self.start_pose.orientation.x)
            )
            OffboardControl.target_pose.orientation.y = (
                self.start_pose.orientation.y + 
                smooth_progress * (self.target_pose_smooth.orientation.y - self.start_pose.orientation.y)
            )
            OffboardControl.target_pose.orientation.z = (
                self.start_pose.orientation.z + 
                smooth_progress * (self.target_pose_smooth.orientation.z - self.start_pose.orientation.z)
            )
            OffboardControl.target_pose.orientation.w = (
                self.start_pose.orientation.w + 
                smooth_progress * (self.target_pose_smooth.orientation.w - self.start_pose.orientation.w)
            )
            
            self.current_smooth_pose = copy.deepcopy(OffboardControl.target_pose)
            return False
    
    def calculate_3d_distance(self, pose1, pose2):
        """Calculate 3D Euclidean distance between two poses"""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z
        return np.sqrt(dx*dx + dy*dy + dz*dz)
    
    @staticmethod
    def cubic_ease_in_out(t):
        """Cubic ease-in-out interpolation for smooth acceleration/deceleration"""
        return t * t * (3.0 - 2.0 * t)
    
    def is_trajectory_active(self):
        """Check if smooth trajectory is currently active"""
        with self.trajectory_lock:
            return self.trajectory_active
    
    def cancel_trajectory(self):
        """Emergency stop - cancel current trajectory"""
        with self.trajectory_lock:
            self.trajectory_active = False
            rospy.logwarn("Smooth trajectory cancelled")
    
    def get_trajectory_progress(self):
        """Get current trajectory progress [0.0, 1.0]"""
        if not self.trajectory_active:
            return 1.0
            
        with self.trajectory_lock:
            elapsed = (rospy.Time.now() - self.trajectory_start_time).to_sec()
            return min(elapsed / self.trajectory_duration, 1.0)

@add_state_features(Timeout)
class TimeoutMachine(Machine):
    pass

class DroneState:
    def __init__(self):
        self.final_setpoint = Pose()
        self.flight_height = MissionConfig.FLIGHT_HEIGHT
        self.reset_moving_avg = False
        self.setpoints = []
        self.x_app_setpoint_app = []
        self.y_app_setpoint_app = []
        self.scan_attempt = 0
        self.marker_detect_attempt = 0
        self.start_time = None
        self.first_call = True
        self.thread_start = False
        self.controller = None
        self.x_app = 0.0
        self.y_app = 0.0
        self.final_x_sp = 0.0
        self.final_y_sp = 0.0
        self.offapp_flag = False
        self.state_timer_start = None
        self.state_timer_active = False
        self.scan_time_done = False
        self.land_time_done = False

        # Add smooth trajectory manager
        self.smooth_trajectory = SmoothTrajectoryManager()
        self.trajectory_complete = False

    def test(self):
        return False

    def update_setpoint(self, target):

        OffboardControl.target_pose.position.x = target.position.x
        OffboardControl.target_pose.position.y = target.position.y
        OffboardControl.target_pose.position.z = target.position.z
        
        OffboardControl.target_pose.orientation.x = target.orientation.x
        OffboardControl.target_pose.orientation.y = target.orientation.y
        OffboardControl.target_pose.orientation.z = target.orientation.z
        OffboardControl.target_pose.orientation.w = target.orientation.w

        rospy.loginfo(
            f"Updating Setpoint - X: {OffboardControl.target_pose.position.x}, Y: {OffboardControl.target_pose.position.y}, Z: {OffboardControl.target_pose.position.z}, YAW: {OffboardControl.target_pose.orientation}"
        )

    def update_setpoint_smooth(self, target, speed=None, trajectory_type="waypoint"):
        """New smooth setpoint update using cubic interpolation"""
        # Convert relative positioning to absolute
        absolute_target = Pose()

        # absolute_target.position.x = target.position.x + OffboardControl.home_pose.position.x
        # absolute_target.position.y = target.position.y + OffboardControl.home_pose.position.y
        # absolute_target.position.z = target.position.z + OffboardControl.home_pose.position.z

        absolute_target.position.x = target.position.x
        absolute_target.position.y = target.position.y
        absolute_target.position.z = target.position.z
        absolute_target.orientation = target.orientation
        
        # Start smooth trajectory
        self.smooth_trajectory.start_smooth_trajectory(absolute_target, speed, trajectory_type)
        self.trajectory_complete = False
        
        rospy.loginfo(f"Starting smooth {trajectory_type} to ({target.position.x:.2f}, {target.position.y:.2f}, {target.position.z:.2f})")


    def on_enter_FAILSAFE(self, *args):
        # print("FAILSAFE ENTERED, RESTART PROGRAM...")
        rospy.loginfo("FAILSAFE ENTERED, RESTART PROGRAM...")
        # Cancel any active trajectories
        self.smooth_trajectory.cancel_trajectory()

    def on_enter_ARM(self, *args):
        # Start pub thread if not already running
        if not self.thread_start:
            # Reset stop event flag
            OffboardControl.stop_pub_thread.clear()

            # Create and start the publisher thread
            OffboardControl.pub_thread = threading.Thread(target=self.controller.trajectory_setpoint_publisher)
            OffboardControl.pub_thread.daemon = True
            OffboardControl.pub_thread.start()
            self.thread_start = True

    def on_exit_IDLE(self, *args):
        rospy.loginfo(f"Home Position Recorded: {OffboardControl.home_pose}")

    def on_exit_LOITER(self, *args):
        if not OffboardControl.on_mission and not OffboardControl.start_mission:
            msg = Pose()
            msg.position.x = OffboardControl.spot_pose.position.x
            msg.position.y = OffboardControl.spot_pose.position.y
            msg.position.z = self.flight_height
            msg.orientation = OffboardControl.home_pose.orientation
            # self.update_setpoint(msg)
            # rospy.loginfo("Sending Search Setpoint")
            self.update_setpoint_smooth(msg, trajectory_type="search")
            rospy.loginfo("Sending Search Setpoint")

    def on_exit_SCAN(self, *args):
        self.reset_moving_avg = False
        self.x_app_setpoint_app = []
        self.y_app_setpoint_app = []
        
        with OffboardControl.offboard_lock:
            OffboardControl.marker_window.clear()

        self.state_timer_active = False
        self.scan_time_done = False

    def on_exit_LAND(self, *args):
        # OffboardControl.magnet_publisher("mag")
        #pass
        self.state_timer_active = False

    def on_enter_SCAN(self, *args):
        # self.state_timer_start = rospy.get_rostime()
        # self.state_timer_active = True
        # rospy.loginfo("Entered SCAN state - starting 2 second wait...")
        pass

    def on_enter_LAND(self, *args):
        # self.state_timer_start = rospy.get_rostime()
        # self.state_timer_active = True
        # rospy.loginfo("Entered LAND state - starting 2 second wait...")
        pass

    def on_exit_PREP_LAND(self, *args):
        self.first_call = True
        self.land_time_done = False

    def on_enter_ABORT(self, *args):
        msg = Pose()
        msg.position.x = OffboardControl.home_pose.position.x
        msg.position.y = OffboardControl.home_pose.position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.home_pose.orientation
        self.final_x_sp = OffboardControl.home_pose.position.x
        self.final_y_sp = OffboardControl.home_pose.position.y
        # self.update_setpoint(msg)
        # rospy.loginfo("DRONE IS ABORTING LANDING. CAUTION...")
        self.update_setpoint_smooth(msg, trajectory_type="abort")
        rospy.loginfo("DRONE IS ABORTING LANDING. CAUTION...")
    
    def on_enter_TASK(self, *args):
        msg = Pose()
        msg.position.x = OffboardControl.mission_sps.poses[0].position.x
        msg.position.y = OffboardControl.mission_sps.poses[0].position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.mission_sps.poses[0].orientation

        self.update_setpoint_smooth(msg)
        rospy.loginfo("TASK RECIEVED. Moving to first waypoint.")
    
    def on_exit_TASK(self, *args):
        OffboardControl.done_mission = False
        OffboardControl.start_mission = False
        OffboardControl.on_mission = False
        OffboardControl.mission_sps = PoseArray()


    def on_enter_MAN_OVERRIDE(self, *args):
        self.stop_thread()
        self.smooth_trajectory.cancel_trajectory()
    
    def on_enter_IDLE(self, *args):
        self.stop_thread()
        self.smooth_trajectory.cancel_trajectory()

    def on_enter_DISARM(self, *args):
        self.stop_thread()
        self.smooth_trajectory.cancel_trajectory()

    def stop_thread(self):
        # Stop the publisher thread and join it if it's running
        if self.thread_start and OffboardControl.pub_thread and OffboardControl.pub_thread.is_alive():
            OffboardControl.stop_pub_thread.set()
            OffboardControl.pub_thread.join(timeout=2.0)
            self.thread_start = False

    def marker_found(self):
        
        with OffboardControl.offboard_lock:
            found_marker = OffboardControl.aruco_found
            is_first_msg = OffboardControl.first_aruco_msg

        rospy.loginfo(f"Marker Found: {found_marker}")

        if (
            not found_marker
            and not is_first_msg
        ):
            self.marker_detect_attempt = self.marker_detect_attempt + 1

        return found_marker and is_first_msg

    def distance_check(self):
        distance = np.sqrt(
            np.square(
                OffboardControl.current_pose.position.x - OffboardControl.aruco_pose.position.x
            )
            + np.square(
                OffboardControl.current_pose.position.y - OffboardControl.aruco_pose.position.y
            )
        )

        rospy.loginfo(f"DISTANCE CHECK: {distance}")

        return distance > 0.285

    def attempt_check(self):
        self.scan_check()
        self.marker_found()

        # if self.scan_attempt >= 25 and self.marker_detect_attempt >= 25:
        #     return True
        # return False
        if self.scan_attempt >= MissionConfig.MAX_SCAN_ATTEMPTS and self.marker_detect_attempt >= MissionConfig.MAX_MARKER_DETECT_ATTEMPTS:
            return True
        return False

    def setpoint_check(self):
        # Check odom if x500 has reached the setpoint
        setpointReached = False
        sp2Validiate = OffboardControl.target_pose.position

        if (
            (
                sp2Validiate.x - MissionConfig.POSITION_TOLERANCE
                < OffboardControl.current_pose.position.x
                < sp2Validiate.x + MissionConfig.POSITION_TOLERANCE
            )
            and (
                sp2Validiate.y - MissionConfig.POSITION_TOLERANCE
                < OffboardControl.current_pose.position.y
                < sp2Validiate.y + MissionConfig.POSITION_TOLERANCE
            )
            and (
                sp2Validiate.z - MissionConfig.POSITION_TOLERANCE
                < OffboardControl.current_pose.position.z
                < sp2Validiate.z + MissionConfig.POSITION_TOLERANCE
            )
        ):
            if (
                (-MissionConfig.VELOCITY_TOLERANCE < OffboardControl.current_vel.linear.x < MissionConfig.VELOCITY_TOLERANCE)
                and (-MissionConfig.VELOCITY_TOLERANCE < OffboardControl.current_vel.linear.y < MissionConfig.VELOCITY_TOLERANCE)
                and (-MissionConfig.VELOCITY_TOLERANCE < OffboardControl.current_vel.linear.z < MissionConfig.VELOCITY_TOLERANCE)
            ):
                setpointReached = True

        return setpointReached

    def setpoint_check_smooth(self):
        """Enhanced setpoint check that works with smooth trajectories"""
        # First check if smooth trajectory is complete
        if self.smooth_trajectory.is_trajectory_active():
            return False  # Still moving along trajectory
            
        # If trajectory complete, check if drone has settled at position
        return self.setpoint_check()  # Use original position/velocity check

    def prog_mission_check(self):
        #I think here it should check the progress of the mission right
        #on_ENTER_TASK --> First setpoint in the array should be sent
        #Do this check below, It is at the setpoint and it is level this is good
        if self.setpoint_check_smooth() and self.attitude_check() and OffboardControl.mission_sps:
            #It is safe to assume you made it to the waypoint. Preapre to send a new one from the OffboardControl.mission_sps variable
            OffboardControl.waypoint_index += 1
        else:
            sp = self.setpoint_check_smooth()
            ap = self.attitude_check()
            return False
        #If it is, check if there you are at the end of the array. If you are not at the end, go to the next item and update the setpoint.
        if OffboardControl.waypoint_index < len(OffboardControl.mission_sps.poses):
            rospy.loginfo("UPDATING SETPOINT")
            msg = Pose()
            msg.position.x = OffboardControl.mission_sps.poses[OffboardControl.waypoint_index].position.x
            msg.position.y = OffboardControl.mission_sps.poses[OffboardControl.waypoint_index].position.y
            msg.position.z = self.flight_height
            msg.orientation = OffboardControl.mission_sps.poses[OffboardControl.waypoint_index].orientation

            self.update_setpoint_smooth(msg)
            
            return False
        #If you are at the end of the list. Return the condition that puts the quad back into LOITER!
        else:
            rospy.loginfo("TRUE AND SUCCESS")
            OffboardControl.done_mission = True
            return True

        #I think I can remove this 
        #return OffboardControl.on_mission
    
    def done_mission_check(self):
        return OffboardControl.done_mission

    def spot_reqland_check(self):
        return OffboardControl.land_requested

    def start_mission_check(self):
        return OffboardControl.start_mission and OffboardControl.on_mission

    def attitude_check(self):
        # Check if x500 is level
        drone_level = False

        mav_rpy = euler_from_quaternion(OffboardControl.current_pose.orientation)

        if (-MissionConfig.ATTITUDE_TOLERANCE < mav_rpy[0] < MissionConfig.ATTITUDE_TOLERANCE) and (
            -MissionConfig.ATTITUDE_TOLERANCE < mav_rpy[1] < MissionConfig.ATTITUDE_TOLERANCE
        ):
            drone_level = True

        return drone_level

    def time_check(self):

        if self.first_call:
            self.start_time = rospy.get_rostime()
            self.first_call = False

        current_time = rospy.get_rostime()
        elapsed_duration = current_time - self.start_time
        time_delta_sec = elapsed_duration.to_sec()
        # print(time_delta_sec)
        rospy.loginfo(f"Timer: {time_delta_sec}")
        if time_delta_sec >= 2.0:
            return True

        return False

    def scan_time(self):
        self.scan_time_done = True
        rospy.loginfo(f"SCAN Timer Done: {self.scan_time_done}")
        
    def land_time(self):
        self.land_time_done = True
        rospy.loginfo(f"LAND Timer Done: {self.land_time_done}")
    
    def scan_time_check(self):
        return self.scan_time_done
    
    def land_time_check(self):
        return self.land_time_done

    def moving_avg(self):
        # Calculate Moving Average using cumulative sum and window of 10
        window = 10

        with OffboardControl.offboard_lock:
            marker_copy = list(OffboardControl.marker_window)

        # Always extract fresh data from marker_window
        sp_x = np.array([p.x for p in marker_copy])
        sp_y = np.array([p.y for p in marker_copy])

        # Check if we have enough data points for the moving average
        if len(sp_x) < window:
            rospy.logwarn(f"Not enough data points for moving average: {len(sp_x)} < {window}")
            return np.array([]), np.array([])

        sum_x = np.empty(len(sp_x) + 1, dtype=np.float64)
        sum_y = np.empty(len(sp_y) + 1, dtype=np.float64)
        sum_x[0] = 0.0
        sum_y[0] = 0.0
        np.cumsum(sp_x, out=sum_x[1:])
        np.cumsum(sp_y, out=sum_y[1:])

        moving_avg_x = (sum_x[window:] - sum_x[:-window]) / window
        moving_avg_y = (sum_y[window:] - sum_y[:-window]) / window

        return moving_avg_x, moving_avg_y

    def scan_check(self):
        if (len(OffboardControl.marker_window) == MissionConfig.MOVING_AVG_WINDOW):
            self.x_app_setpoint_app, self.y_app_setpoint_app = self.moving_avg()
            return True
        self.scan_attempt += 1
        return False

    def set_takeoff_setpoint(self):
        take_off_pose = Pose()
        take_off_pose.position.x = 0.0
        take_off_pose.position.y = 0.0
        take_off_pose.position.z = self.flight_height

        take_off_pose.orientation = OffboardControl.home_pose.orientation
        # self.update_setpoint(take_off_pose)
        self.update_setpoint_smooth(take_off_pose, trajectory_type="takeoff")

    def set_takeoff_setpoint_smooth(self):
        """New smooth takeoff setpoint"""
        take_off_pose = Pose()
        take_off_pose.position.x = 0.0
        take_off_pose.position.y = 0.0
        take_off_pose.position.z = self.flight_height
        take_off_pose.orientation = OffboardControl.home_pose.orientation
        
        self.update_setpoint_smooth(take_off_pose, trajectory_type="takeoff")
        
    def set_approach_setpoint(self):
        if not self.offapp_flag:
            self.x_app = np.mean(self.x_app_setpoint_app)
            self.y_app = np.mean(self.y_app_setpoint_app)
            self.offapp_flag = True

        msg = Pose()
        msg.position.x = np.mean(self.x_app_setpoint_app)
        msg.position.y = np.mean(self.y_app_setpoint_app)
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.dock_pose.orientation

        rospy.loginfo(f"Approaching Marker: {msg.position.x}, Y: {msg.position.y}")

        self.update_setpoint(msg)
        rospy.loginfo("Approaching Marker")

    def set_approach_setpoint_smooth(self):
        """New smooth approach setpoint"""
        if not self.offapp_flag:
            self.x_app = np.mean(self.x_app_setpoint_app)
            self.y_app = np.mean(self.y_app_setpoint_app)
            self.offapp_flag = True

        msg = Pose()
        msg.position.x = np.mean(self.x_app_setpoint_app)
        msg.position.y = np.mean(self.y_app_setpoint_app)
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.dock_pose.orientation

        rospy.loginfo(f"Approaching Marker: {msg.position.x}, Y: {msg.position.y}")

        self.update_setpoint_smooth(msg, trajectory_type="approach")
        rospy.loginfo("Starting smooth approach to marker")

    def set_finapp_setpoint(self):
        if not self.offapp_flag:
            self.x_app = np.mean(self.x_app_setpoint_app)
            self.y_app = np.mean(self.y_app_setpoint_app)
            self.offapp_flag = True
            # rospy.loginfo(f"X_ and Y_ app_setpoint_app IN IFNOT: {self.x_app_setpoint_app}, Y: {self.y_app_setpoint_app}")
            # rospy.loginfo(f"X_ and Y_ App MEAN IFNOT: {self.x_app}, Y: {self.y_app}")

        rospy.loginfo(f"X_ and Y_ app_setpoint_app: {self.x_app_setpoint_app}, Y: {self.y_app_setpoint_app}")
        rospy.loginfo(f"X_ and Y_ App MEAN: {self.x_app}, Y: {self.y_app}")

        msg = Pose()
        msg.position.x = self.x_app + OffboardControl.dock_pose.position.x
        msg.position.y = self.y_app + OffboardControl.dock_pose.position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.dock_pose.orientation

        self.final_x_sp = self.x_app + OffboardControl.dock_pose.position.x
        self.final_y_sp = self.y_app + OffboardControl.dock_pose.position.y

        rospy.loginfo(f"Final Marker App: {msg.position.x}, Y: {msg.position.y}")

        self.update_setpoint(msg)
        rospy.loginfo("Final Approach...")

    def set_finapp_setpoint_smooth(self):
        """New smooth final approach setpoint"""
        if not self.offapp_flag:
            self.x_app = np.mean(self.x_app_setpoint_app)
            self.y_app = np.mean(self.y_app_setpoint_app)
            self.offapp_flag = True
            
        msg = Pose()
        msg.position.x = self.x_app + OffboardControl.dock_pose.position.x
        msg.position.y = self.y_app + OffboardControl.dock_pose.position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.dock_pose.orientation

        self.final_x_sp = self.x_app + OffboardControl.dock_pose.position.x
        self.final_y_sp = self.y_app + OffboardControl.dock_pose.position.y

        rospy.loginfo(f"Final Marker App: {msg.position.x}, Y: {msg.position.y}")

        self.update_setpoint_smooth(msg, trajectory_type="final_approach")
        rospy.loginfo("Starting smooth final approach...")
    
    def offapp_check(self):
        return self.offapp_flag

    def set_final_setpoint(self):
        self.final_setpoint.position.x = self.final_x_sp
        self.final_setpoint.position.y = self.final_y_sp
        self.final_setpoint.position.z = OffboardControl.current_pose.position.z 
        self.final_setpoint.orientation = OffboardControl.home_pose.orientation
        
        rospy.loginfo(f"Final Setpoint: {self.final_setpoint.position.x}, Y: {self.final_setpoint.position.y}")


    def landing_check(self):
        drone_land = False

        self.final_setpoint.position.z -= 0.085
        # self.update_setpoint(self.final_setpoint)
        self.update_setpoint_smooth(self.final_setpoint, trajectory_type="landing")

        rospy.loginfo(f"Landing Setpoint Z: {self.final_setpoint.position.z}")

        if OffboardControl.curr_thrust <= 0.45:  #0.120 is the simulated lowest thrust once landed
            drone_land = True

        return drone_land

    def lost_fiducial_check(self):
        # Return True if we've lost the fiducial while at setpoint
        if self.setpoint_check() and self.attitude_check():
            # If we're at setpoint and level, but don't see the marker
            if not OffboardControl.aruco_found and OffboardControl.first_aruco_msg:
                self.marker_detect_attempt += 1
                if self.marker_detect_attempt >= 10:  # More aggressive threshold for loss at setpoint
                    return True
            
            # Return True if attempt check fails
            if self.attempt_check():
                return True
            
        return False
    
    def trajectory_progress(self):
        """Get current trajectory progress for debugging"""
        return self.smooth_trajectory.get_trajectory_progress()

class OffboardControl:
    current_state = State()
    current_pose = Pose()
    home_pose = Pose()
    dock_pose = Pose()
    spot_pose = Pose()
    target_pose = Pose()
    aruco_pose = Pose()
    current_vel = Twist()
    curr_thrust = 0.0
    marker_window = []
    first_aruco_msg = False
    aruco_found = False
    offboard_counter = 0
    landed_state = 0
    stop_pub_thread = None
    pub_thread = None
    
    
    mission_sps = PoseArray()
    waypoint_index = 0
    on_mission = False
    start_mission = False
    land_requested = False
    done_mission = False

    offboard_lock = threading.RLock()

    def __init__(self):
        rospy.init_node('offb_node_py')

        self.homeSetPos = False
        self.mag_status = FG40Feedback()

        self.sp_pub_rate = MissionConfig.SETPOINT_RATE_HZ
        self.control_rate = MissionConfig.CONTROL_RATE_HZ
        #self.rate = rospy.Rate(self.rate_hz)

        OffboardControl.stop_pub_thread = threading.Event()
        OffboardControl.pub_thread = threading.Thread(target=self.trajectory_setpoint_publisher)
        OffboardControl.pub_thread.daemon = True

        #Services
        self.mission_srv = rospy.Service('mission_service', mission, self.service_callback)

        # Publishers
        self.magnet_cmd_pub = rospy.Publisher('/fg40_cmd', FG40MagnetCmd, queue_size=10)
        #Replaces TrajectorySetpoint
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscribers
        self.fg40_status_sub = rospy.Subscriber("/fg40_status", FG40Feedback, self.fg40_status_callback)
        # Replaces VehicleLocalPosition and VehicleAttitude for Pose
        self.localpos_subscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.localpos_callback)
        self.localvel_subscriber = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.localvel_callback)
        #Replacess VehicleStaus
        self.drone_status_sub = rospy.Subscriber("/mavros/state", State, self.vehicle_status_callback)

        #Replacess VehicleLandDetected but is missing the land detector checks built into uORB message.
        self.drone_status_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, self.vehicle_land_det_callback)
        
        #Replaces ArucoMarkers
        self.aruco_subscriber = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.aruco_callback)

        self.thrust_subscriber = rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.thrust_callback)
        self.spot_pos_subscriber = rospy.Subscriber("/spot_pose", PoseStamped, self.spot_pos_callback)
        self.dock_pos_subscriber = rospy.Subscriber("/dock_pose", PoseStamped, self.dock_pos_callback)

        # Clients
        rospy.wait_for_service('/mavros/cmd/command')
        rospy.wait_for_service('/mavros/set_mode')
        self.command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)


        OffboardControl.spot_pose.position.x = 2.1
        OffboardControl.spot_pose.position.y = 0.0
        OffboardControl.spot_pose.position.z = 0.0
        OffboardControl.spot_pose.orientation = OffboardControl.home_pose.orientation


        self.states = [
            "IDLE",
            "FAILSAFE",
            "ARM",
            "DISARM",
            "TAKEOFF",
            "LOITER",
            "TASK",
            "SEARCH",
            {
                'name': 'SCAN',
                'timeout': 5.0,
                'on_timeout': 'scan_time'
            },
            "APPROACH",
            "FINAPP",
            {
                'name': 'PREP_LAND',
                'timeout': 5.0,
                'on_timeout': 'land_time'
            },
            "LAND",
            "ABORT",
            "MAN_OVERRIDE",
        ]

        self.droneState = DroneState()
        self.droneState.controller = self
        self.machine = TimeoutMachine(
            model=self.droneState, states=self.states, initial="IDLE"
        )

        self.machine.add_transition(
            "trs_next",
            "IDLE",
            "ARM",
            conditions=lambda: OffboardControl.current_state.armed,
            #== VehicleStatus.ARMING_STATE_ARMED,
        )
        self.machine.add_transition(
            "trs_next",
            "ARM",
            "IDLE",
            unless=lambda: OffboardControl.current_state.armed,
            #!= VehicleStatus.ARMING_STATE_ARMED,
        )
        self.machine.add_transition(
            "trs_next",
            "ARM",
            "TAKEOFF",
            prepare=["set_takeoff_setpoint_smooth"],
            conditions=lambda: OffboardControl.current_state.mode
            == State.MODE_PX4_OFFBOARD
            #== VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and OffboardControl.offboard_counter >= 100,
        )

        # Need to ensure that if this state transition occurs, some sort of clean up like go back into manual mode or pos mode and restart back to Arm or Idle depending on conds.
        self.machine.add_transition(
            "trs_next",
            "TAKEOFF",
            "FAILSAFE",
            conditions=lambda: OffboardControl.current_state.system_status == 3,
        )
        # self.machine.add_transition('trs_next', 'FAILSAFE', 'FAILSAFE')
        self.machine.add_transition(
            "trs_next", "TAKEOFF", "LOITER", conditions=["setpoint_check_smooth"]
        )
        #Conditions for this need to be changed. Mission must be done before going into search
        self.machine.add_transition(
            "trs_next", "LOITER", "SEARCH", conditions=["spot_reqland_check", "setpoint_check_smooth"]
        )
        #Get SPs from Spot and do your task...
        self.machine.add_transition(
            "trs_next", "LOITER", "TASK", conditions=["start_mission_check", "setpoint_check_smooth"]
        )
        #When mission is done. LOITER...
        self.machine.add_transition(
            "trs_next", "TASK", "LOITER", conditions=["prog_mission_check", "done_mission_check", "setpoint_check_smooth"]
        )

        #TASK MOVE TASK MOVE????

        self.machine.add_transition(
            "trs_next",
            "SEARCH",
            "SCAN",
            conditions=["setpoint_check_smooth", "attitude_check"],
        )

        # Perform moving avg?
        self.machine.add_transition(
            "trs_next",
            "SCAN",
            "APPROACH",
            before=["set_approach_setpoint_smooth"],
            conditions=["scan_time_check", "scan_check", "distance_check", "marker_found"],
        )
        # Perform another scan?
        self.machine.add_transition(
            "trs_next",
            "APPROACH",
            "SCAN",
            conditions=["setpoint_check_smooth", "attitude_check"],
        )

        # Abort landing manuver if fiducial is not found > 3 times
        self.machine.add_transition(
            "trs_next",
            ["SCAN", "FINAPP"],
            "ABORT",
            conditions=["lost_fiducial_check"],
        )

        # Everytime LAND state is entered, set the final setpoint, otherwise it will send -0.04 m
        self.machine.add_transition(
            "trs_next",
            "ABORT",
            "LAND",
            prepare=["set_final_setpoint"],
            conditions=["land_time_check", "setpoint_check_smooth", "attitude_check"],
        )

        self.machine.add_transition(
            "trs_next",
            "SCAN",
            "FINAPP",
            before=["set_finapp_setpoint_smooth"],
            conditions=["scan_time_check", "scan_check", "marker_found"],
            unless=["distance_check"],
        )

        self.machine.add_transition(
            "trs_next",
            "FINAPP",
            "PREP_LAND",
            conditions=["setpoint_check_smooth", "attitude_check"],
        )

        self.machine.add_transition(
            "trs_next",
            "PREP_LAND",
            "LAND",
            prepare=["set_final_setpoint"],
            conditions=["land_time_check", "setpoint_check_smooth", "attitude_check"],
        )
        self.machine.add_transition(
            "trs_next", "LAND", "DISARM", conditions=["landing_check"]
        )
        self.machine.add_transition(
            "trs_next",
            "DISARM",
            "IDLE",
            conditions=lambda: (OffboardControl.current_state.mode == State.MODE_PX4_READY) or 
            (OffboardControl.current_state.mode in [State.MODE_PX4_OFFBOARD, 
                                                    State.MODE_PX4_POSITION, 
                                                    State.MODE_PX4_MANUAL] and 
            not OffboardControl.current_state.armed and OffboardControl.current_state.system_status == 3),
        )


        self.machine.add_transition(
            "trs_next",
            [
                "FAILSAFE",
                "TAKEOFF",
                "LOITER",
                "TASK",
                "SEARCH",
                "SCAN",
                "APPROACH",
                "FINAPP",
                "PREP_LAND",
                "LAND",
                "ABORT",
            ],
            "MAN_OVERRIDE",
            conditions=lambda: OffboardControl.current_state.mode
            == State.MODE_PX4_POSITION
            or OffboardControl.current_state.mode == State.MODE_PX4_MANUAL,
        )
        self.machine.add_transition("trs_next", "MAN_OVERRIDE", "MAN_OVERRIDE")


    def vehicle_status_callback(self, msg):
        OffboardControl.current_state = msg


    def localpos_callback(self, msg):
        OffboardControl.current_pose = msg.pose

        if not self.homeSetPos:
            OffboardControl.home_pose = msg.pose
            self.homeSetPos = True

    def localvel_callback(self, msg):
        OffboardControl.current_vel = msg.twist

    def thrust_callback(self, msg):
        OffboardControl.curr_thrust = msg.thrust

    def vehicle_land_det_callback(self, msg):
        OffboardControl.landed_state = msg.landed_state


    def aruco_callback(self, msg):
        
        for marker in msg.markers:
            if marker.id == 121:
                if marker.confidence > 0.8 and self.droneState.state == "SCAN":
                    with OffboardControl.offboard_lock:
                        OffboardControl.aruco_found = True
                        if not OffboardControl.first_aruco_msg and OffboardControl.aruco_found:
                            OffboardControl.first_aruco_msg = True
                    
                    #Assuming Hamilton convention
                    # Want to remove this later...
                    OffboardControl.aruco_pose = marker.pose.pose
                    aruco_rpy = euler_from_quaternion(marker.pose.pose.orientation)

                    marker_position = Point()
                    marker_position = marker.pose.pose.position

                    with OffboardControl.offboard_lock:
                        if (len(OffboardControl.marker_window) < MissionConfig.MOVING_AVG_WINDOW):
                            OffboardControl.marker_window.append(marker_position)
                        elif (len(OffboardControl.marker_window) == MissionConfig.MOVING_AVG_WINDOW):
                            OffboardControl.marker_window.pop(0)
                            OffboardControl.marker_window.append(marker_position)

            else:
                with OffboardControl.offboard_lock:
                    OffboardControl.aruco_found = False


    def fg40_status_callback(self, msg):
        self.mag_status = msg

    def spot_pos_callback(self, msg):
        OffboardControl.spot_pose = msg.pose

    def dock_pos_callback(self, msg):
        OffboardControl.dock_pose = msg.pose

    def service_callback(self, req):
        if req.stateRequest == "BREAKAWAY":
            #Arm the drone
            #Request Offboard mode
            self.magnet_publisher("demag")
            self.magnet_publisher("demag")
            self.arm()

            while OffboardControl.current_state.mode != State.MODE_PX4_OFFBOARD:
                self.offboard_request()

            #Check px4 state and inform user
            # if OffboardControl.current_state.mode == State.MODE_PX4_OFFBOARD:
            #     rospy.loginfo("OFFBOARD ACCEPTED...X500 IS AUTONOMOUS...CAUTION")
            rospy.loginfo("OFFBOARD ACCEPTED...X500 IS AUTONOMOUS...CAUTION")

            while self.droneState.state != "LOITER":
                rospy.loginfo("WAITING FOR X500 TO FINISH TAKEOFF PROCEDURE...")
                #do someting. How do I make the above send once?
            rospy.loginfo("X500 HAS FINISHED TAKEOFF. WAITING FOR MISSION...")
            #SEND CLIENT CONFIRMATION
            return missionResponse(success=True)

        elif req.stateRequest == "INSPECT":
            if not req.setpoints.poses:
                rospy.logwarn("TASK WAS REQUESTED BUT NO SETPOINTS PROVIDED!")
                return missionResponse(success=False)

            if self.droneState.state == "LOITER" and OffboardControl.current_state.mode == State.MODE_PX4_OFFBOARD and not OffboardControl.on_mission:
                OffboardControl.mission_sps = req.setpoints
                OffboardControl.on_mission = True
                OffboardControl.start_mission = True
                return missionResponse(success=True)
            else:
                rospy.logwarn("X500 IS ALREADY ON A MISSION!")
                return missionResponse(success=False)
        
        elif req.stateRequest == "TOUCHDOWN":
            if self.droneState.state == "LOITER" and OffboardControl.current_state.mode == State.MODE_PX4_OFFBOARD:
                OffboardControl.land_requested = True

            while self.droneState.state != "DISARM":
                rospy.loginfo("WAITING FOR X500 TO LAND...")

            rospy.loginfo("Magnetizing FG40...")
            self.magnet_publisher("mag")
            self.magnet_publisher("mag")
            self.disarm()
            return missionResponse(success=True)
        
        else:
            #request not recognized, send a fail state
            rospy.loginfo("REQUEST ERROR!")
            return missionResponse(success=False)

    def arm(self):
        self.call_vehicle_command(400, 1.0, 0.0)
        rospy.loginfo("ARM COMMAND INVOKED")
    
    def offboard_request(self):
        self.call_vehicle_command(176, 1.0, 6.0)
        rospy.loginfo("SPOT HAS REQUESTED TAKEOFF")

    def disarm(self):
        self.call_vehicle_command(400, 0.0, 21196)
        rospy.loginfo("DISARM COMMAND INVOKED")

    def call_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        cmd = CommandLongRequest()
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.param3 = float(param3)
        cmd.param4 = float(param4)
        cmd.param5 = float(param5)
        cmd.param6 = float(param6)
        cmd.param7 = float(param7)
        cmd.command = command
        resp = self.command_client.call(cmd)
        
        rospy.loginfo(f"Command {command} sent with response: {resp}")
        return resp.success

    def trajectory_setpoint_publisher(self):
        """Enhanced publisher that handles smooth trajectory updates"""
        rate = rospy.Rate(self.sp_pub_rate)
        
        while not rospy.is_shutdown() and not OffboardControl.stop_pub_thread.is_set():
            # Update smooth trajectory (this modifies OffboardControl.target_pose)
            trajectory_complete = self.droneState.smooth_trajectory.update_smooth_trajectory()
            
            # Handle trajectory completion
            if trajectory_complete:
                self.droneState.trajectory_complete = True
            
            # Publish current setpoint (now smoothly interpolated)
            msg = PoseStamped()
            msg.pose = OffboardControl.target_pose
            self.setpoint_publisher.publish(msg)

            if OffboardControl.offboard_counter <= 100:
                OffboardControl.offboard_counter += 1

            rate.sleep()

    def magnet_publisher(self, cmd):
        msg = FG40MagnetCmd()

        if cmd == "magnetize" or cmd == "mag" or cmd == "m":
            msg.cmd_magnet = 1
            self.magnet_cmd_pub.publish(msg)
        elif cmd == "force_magnetize" or cmd == "forcemag" or cmd == "fm":
            msg.cmd_magnet = 2
            self.magnet_cmd_pub.publish(msg)
        elif cmd == "demagnetize" or cmd == "demag" or cmd == "dm":
            msg.cmd_magnet = 0
            self.magnet_cmd_pub.publish(msg)

    def state_parser(self):
        """Enhanced state parser with trajectory progress logging"""
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            progress = self.droneState.trajectory_progress()
            
            # Enhanced logging with trajectory info
            if self.droneState.smooth_trajectory.is_trajectory_active():
                rospy.loginfo(f"STATE: {self.droneState.state} | Trajectory: {progress:.1%} active")
            else:
                rospy.loginfo(f"STATE: {self.droneState.state} | Trajectory: complete")
            
            self.droneState.trs_next()
            
            # if self.droneState.state == "DISARM":
            #     rospy.loginfo("Magnetizing FG40...")
            #     self.magnet_publisher("mag")
            #     self.magnet_publisher("mag")
            #     self.disarm()

            # if self.droneState.state == "TASK":
            #     #Mission setpoints are in OffboardControl.mission_sps
            #     droneState.update_setpoint_smooth()
            #     pass

            rate.sleep()


if __name__ == '__main__':
    controller = OffboardControl()
    controller.state_parser()
