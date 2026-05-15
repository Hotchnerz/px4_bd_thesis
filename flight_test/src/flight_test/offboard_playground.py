#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, Pose, TwistStamped, Twist, Point, TransformStamped
from mavros_msgs.msg import State, ExtendedState, AttitudeTarget
# from tf.transformations import euler_from_quaternion #Cannot use due to melodic pkgs being built with python2
import tf2_ros
from flight_test.transform_utils import euler_from_quaternion, pose_to_matrix, matrix_to_pose, get_transform, transform_to_pose
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandLong, CommandLongRequest
from fg40_msgs.msg import FG40Feedback, FG40MagnetCmd
from aruco_msgs.msg import MarkerArray, Marker
import numpy as np
from transitions import Machine
from transitions.extensions.states import add_state_features, Timeout
import copy

# Configuration Management
class MissionConfig:
    """Centralized configuration for mission parameters"""
    # Flight parameters
    FLIGHT_HEIGHT = 1.5
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
    MOVING_AVG_WINDOW = 50
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
        self.fiducial_pose = Pose()
        self.scan_complete = False

        # Add smooth trajectory manager
        self.smooth_trajectory = SmoothTrajectoryManager()
        self.trajectory_complete = False
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    # def get_transform(self, target_frame, source_frame):
    #     try:
    #         trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
    #         return trans
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         rospy.logwarn(f"TF lookup failed ({target_frame} -> {source_frame}): {e}")
    #         return None

    # def transform_to_pose(self, trans):
    #     pose = Pose()
    #     pose.position.x = trans.transform.translation.x
    #     pose.position.y = trans.transform.translation.y
    #     pose.position.z = trans.transform.translation.z
    #     pose.orientation.x = trans.transform.rotation.x
    #     pose.orientation.y = trans.transform.rotation.y
    #     pose.orientation.z = trans.transform.rotation.z
    #     pose.orientation.w = trans.transform.rotation.w
    #     return pose

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

    def update_setpoint_smooth(self, target, speed=None, trajectory_type="waypoint", home_offset=False):
        """New smooth setpoint update using cubic interpolation"""
        # Convert relative positioning to absolute
        absolute_target = Pose()

        if home_offset == True:
            absolute_target.position.x = target.position.x + OffboardControl.home_pose.position.x
            absolute_target.position.y = target.position.y + OffboardControl.home_pose.position.y
            absolute_target.position.z = target.position.z + OffboardControl.home_pose.position.z
            absolute_target.orientation = target.orientation
        
        else:
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
        #Record home position
        OffboardControl.home_pose = copy.deepcopy(OffboardControl.current_pose)
        rospy.loginfo(f"Home Recorded: ({OffboardControl.home_pose.position.x:.3f}, "
                        f"{OffboardControl.home_pose.position.y:.3f}, "
                        f"{OffboardControl.home_pose.position.z:.3f})")
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
        pass

    def on_exit_LOITER(self, *args):
        msg = Pose()
        msg.position.x = OffboardControl.spot_pose.position.x
        msg.position.y = OffboardControl.spot_pose.position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.spot_pose.orientation
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
            OffboardControl.aruco_found = False
            OffboardControl.first_aruco_msg = False

        self.state_timer_active = False
        self.scan_time_done = False


    def on_exit_LAND(self, *args):
        # OffboardControl.magnet_publisher("mag")
        #pass
        self.state_timer_active = False

    def on_enter_SCAN(self, *args):
        self.scan_attempt = 0
        self.marker_detect_attempt = 0
        self.scan_complete = False

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
        # transform = self.get_transform('map', 'c920_link')
        # camera_pose = self.transform_to_pose(transform)

        # map_tform_camera = pose_to_matrix(camera_pose)
        # camera_tform_fiducial = pose_to_matrix(self.fiducial_pose)

        # map_T_fiducial = map_tform_camera @ camera_tform_fiducial
        # fiducial_in_map = matrix_to_pose(map_T_fiducial)
        fiducial_in_map = self.fiducial_pose

        distance = np.sqrt(
            np.square(
                OffboardControl.current_pose.position.x - fiducial_in_map.position.x
            )
            + np.square(
                OffboardControl.current_pose.position.y - fiducial_in_map.position.y
            )
        )

        rospy.loginfo(f"DISTANCE CHECK: {distance}")

        return distance > 0.125

    def attempt_check(self):
        if (self.scan_attempt >= MissionConfig.MAX_SCAN_ATTEMPTS 
            and self.marker_detect_attempt >= MissionConfig.MAX_MARKER_DETECT_ATTEMPTS):
            return True
        return False
        # self.scan_check()
        # self.marker_found()

        # if self.scan_attempt >= MissionConfig.MAX_SCAN_ATTEMPTS and self.marker_detect_attempt >= MissionConfig.MAX_MARKER_DETECT_ATTEMPTS:
        #     return True
        # return False

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
        window = 30

        with OffboardControl.offboard_lock:
            marker_copy = list(OffboardControl.marker_window)

        # Use only the last 'window' poses
        recent = marker_copy[-window:]
        rospy.loginfo(f"MARKER ARR: {recent}")

        # Average position
        avg_x = np.mean([p.position.x for p in recent])
        avg_y = np.mean([p.position.y for p in recent])
        avg_z = np.mean([p.position.z for p in recent])

        #Take the quaternions and use Markley method 
        quats = [np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]) for p in recent]

        #Flip quartnernions due to double cover problem
        for i in range(1, len(quats)):
            if np.dot(quats[i], quats[0]) < 0:
                quats[i] = -quats[i]

        M = np.zeros((4, 4))
        for q in quats:
            M += np.outer(q, q)
        M /= len(quats)

        eigenvalues, eigenvectors = np.linalg.eigh(M)
        avg_q = eigenvectors[:, -1]

        averaged_pose = Pose()
        averaged_pose.position.x = avg_x
        averaged_pose.position.y = avg_y
        averaged_pose.position.z = avg_z
        averaged_pose.orientation.x = avg_q[0]
        averaged_pose.orientation.y = avg_q[1]
        averaged_pose.orientation.z = avg_q[2]
        averaged_pose.orientation.w = avg_q[3]

        rospy.loginfo("Position: x=%.3f, y=%.3f, z=%.3f", averaged_pose.position.x, averaged_pose.position.y, averaged_pose.position.z)

        rospy.loginfo("Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", averaged_pose.orientation.x, averaged_pose.orientation.y, averaged_pose.orientation.z, averaged_pose.orientation.w)

        return averaged_pose

    def scan_check(self):
        if self.scan_complete:
            return True

        if (len(OffboardControl.marker_window) == MissionConfig.MOVING_AVG_WINDOW):
            self.fiducial_pose = self.moving_avg()
            self.scan_complete = True
            return True
        self.scan_attempt += 1
        return False

    def set_takeoff_setpoint_smooth(self):
        """smooth takeoff setpoint"""
        take_off_pose = Pose()
        take_off_pose.position.x = 0.0
        take_off_pose.position.y = 0.0
        take_off_pose.position.z = self.flight_height
        take_off_pose.orientation = OffboardControl.home_pose.orientation
        
        self.update_setpoint_smooth(take_off_pose, trajectory_type="takeoff", home_offset=True)

    def set_approach_setpoint_smooth(self):
        """approach setpoint"""
        # transform = self.get_transform('map', 'c920_link')
        # camera_pose = self.transform_to_pose(transform)

        # map_tform_camera = pose_to_matrix(camera_pose)
        # camera_tform_fiducial = pose_to_matrix(self.fiducial_pose)

        # map_T_fiducial = map_tform_camera @ camera_tform_fiducial
        # fiducial_in_map = matrix_to_pose(map_T_fiducial)
        fiducial_in_map = self.fiducial_pose

        if not self.offapp_flag:
            self.x_app = fiducial_in_map.position.x
            self.y_app = fiducial_in_map.position.y
            self.offapp_flag = True

        msg = Pose()
        msg.position.x = fiducial_in_map.position.x
        msg.position.y = fiducial_in_map.position.y
        msg.position.z = self.flight_height
        msg.orientation = fiducial_in_map.orientation

        rospy.loginfo(f"Approaching Marker: {msg.position.x}, Y: {msg.position.y}")

        self.update_setpoint_smooth(msg, trajectory_type="approach")
        rospy.loginfo("Starting smooth approach to marker")

    def set_finapp_setpoint_smooth(self):
        """final approach setpoint"""

        #Use the average fiduical pose and move to the dock_pose
        # transform_dock_pose = self.get_transform('id_121', 'dock')
        transform_dock_pose = get_transform(self.tf_buffer, 'id_121', 'dock')
        if transform_dock_pose is None:
            return  # or some other failure handling
        dock_pose = transform_to_pose(transform_dock_pose)

        # transform_camera_pose = self.get_transform('map', 'c920_link')
        # camera_pose = self.transform_to_pose(transform_camera_pose)

        fiducial_tform_dock = pose_to_matrix(dock_pose)
        # camera_tform_fiducial = pose_to_matrix(self.fiducial_pose)
        # map_tform_camera = pose_to_matrix(camera_pose)
        map_tform_fiducial = pose_to_matrix(self.fiducial_pose)

        # camera_tform_dock = camera_tform_fiducial @ fiducial_tform_dock
        # map_tform_dock = map_tform_camera @ camera_tform_dock

        map_tform_dock = map_tform_fiducial @ fiducial_tform_dock
        dock_in_map = matrix_to_pose(map_tform_dock)

        # if not self.offapp_flag:
        #     self.x_app = fiducial_in_map.position.x
        #     self.y_app = fiducial_in_map.position.y
        #     self.offapp_flag = True
            
        msg = Pose()

        msg.position.x = dock_in_map.position.x
        msg.position.y = dock_in_map.position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.spot_pose.orientation

        self.final_x_sp = dock_in_map.position.x
        self.final_y_sp = dock_in_map.position.y

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

        self.droneState = DroneState()
        self.droneState.controller = self

        self.states = [
            "IDLE",
            "FAILSAFE",
            "ARM",
            "DISARM",
            "TAKEOFF",
            "LOITER",
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

        self.machine = TimeoutMachine(
            model=self.droneState, states=self.states, initial="IDLE"
        )

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
        self.extended_state_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, self.vehicle_land_det_callback)
        
        #Replaces ArucoMarkers
        self.aruco_subscriber = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.aruco_callback)

        self.thrust_subscriber = rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.thrust_callback)
        self.spot_pos_subscriber = rospy.Subscriber("/fid_off_pose", PoseStamped, self.spot_pos_callback)
        self.dock_pos_subscriber = rospy.Subscriber("/dock_pose", PoseStamped, self.dock_pos_callback)

        # Clients
        rospy.wait_for_service('/mavros/cmd/command')
        rospy.wait_for_service('/mavros/set_mode')
        self.command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)


        OffboardControl.spot_pose.position.x = 1.5
        OffboardControl.spot_pose.position.y = 0.0
        OffboardControl.spot_pose.position.z = 0.0
        OffboardControl.spot_pose.orientation.w = 1.0


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

        self.machine.add_transition(
            "trs_next", "LOITER", "SEARCH", conditions=["setpoint_check_smooth"]
        )
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

    def localvel_callback(self, msg):
        OffboardControl.current_vel = msg.twist

    def thrust_callback(self, msg):
        OffboardControl.curr_thrust = msg.thrust

    def vehicle_land_det_callback(self, msg):
        OffboardControl.landed_state = msg.landed_state


    def aruco_callback(self, msg):
        found_121 = False
        for marker in msg.markers:
            if marker.id == 121 and marker.confidence > 0.8 and self.droneState.state == "SCAN":
                #marker_poses = marker.pose.pose
                # Compose to map frame outside of the lock, cannot be blocking.
                tf = get_transform(
                    self.droneState.tf_buffer,
                    'map',
                    marker.header.frame_id,
                    stamp=marker.header.stamp,
                    timeout=0.05,
                )
                if tf is None:
                    # Drop the sample
                    rospy.logwarn_throttle(1.0, "ArUco TF compose failed; sample dropped")
                    break

                map_T_cam = pose_to_matrix(transform_to_pose(tf))
                cam_T_fid = pose_to_matrix(marker.pose.pose)
                map_T_fid_pose = matrix_to_pose(map_T_cam @ cam_T_fid)

                with OffboardControl.offboard_lock:
                    if self.droneState.state != "SCAN":
                        break
                    if not OffboardControl.first_aruco_msg:
                        OffboardControl.first_aruco_msg = True
                    if len(OffboardControl.marker_window) < MissionConfig.MOVING_AVG_WINDOW:
                        OffboardControl.marker_window.append(map_T_fid_pose)
                    else:
                        OffboardControl.marker_window.pop(0)
                        OffboardControl.marker_window.append(map_T_fid_pose)
                    found_121 = True
                break

        with OffboardControl.offboard_lock:
            OffboardControl.aruco_found = found_121


    def fg40_status_callback(self, msg):
        self.mag_status = msg

    def spot_pos_callback(self, msg):
        OffboardControl.spot_pose = msg.pose


    def dock_pos_callback(self, msg):
        OffboardControl.dock_pose = msg.pose


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
            
            if self.droneState.state == "DISARM":
                rospy.loginfo("Magnetizing FG40...")
                self.magnet_publisher("mag")
                self.magnet_publisher("mag")
                self.disarm()

            rate.sleep()


if __name__ == '__main__':
    controller = OffboardControl()
    controller.state_parser()