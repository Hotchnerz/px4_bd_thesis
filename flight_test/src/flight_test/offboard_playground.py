#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, Pose, TwistStamped, Twist, Point, TransformStamped
from mavros_msgs.msg import State, ExtendedState, AttitudeTarget
# from tf.transformations import euler_from_quaternion #Cannot use due to melodic pkgs being built with python2
from flight_test.transform_utils import euler_from_quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandLong, CommandLongRequest
from fg40_msgs.msg import FG40Feedback, FG40MagnetCmd
from aruco_msgs.msg import MarkerArray, Marker
import numpy as np
from transitions import Machine

class DroneState:
    def __init__(self):
        self.final_setpoint = Pose()
        self.flight_height = 1.25
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
        self.offapp_flag = False
        #self.clock = Clock()
        #self.logger = logger

    def test(self):
        # print(OffboardControl.curr_pos)
        # print(OffboardControl.setpoints)
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

    def on_enter_FAILSAFE(self, *args):
        # print("FAILSAFE ENTERED, RESTART PROGRAM...")
        rospy.loginfo("FAILSAFE ENTERED, RESTART PROGRAM...")

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

    # def on_enter_TAKEOFF(self, *args):
    #     self.update_setpoint([0,0,-1.25,0])
    #     #rospy.loginfo("Sending Takeoff Setpoint")
    #     print("Sending Takeoff Setpoint")

    def on_exit_IDLE(self, *args):
        rospy.loginfo(f"Home Position Recorded: {OffboardControl.home_pose}")

    def on_exit_LOITER(self, *args):
        msg = Pose()
        msg.position.x = OffboardControl.spot_pose.position.x
        msg.position.y = OffboardControl.spot_pose.position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.home_pose.orientation
        self.update_setpoint(msg)
        rospy.loginfo("Sending Search Setpoint")

    # def on_enter_APPROACH(self, *args):
    #     self.update_setpoint([OffboardControl.marker_pos[0],OffboardControl.marker_pos[1],OffboardControl.curr_pos[2],OffboardControl.marker_pos[3]])
    #     #self.update_setpoint([1.25944,0.0202361,-1.25,OffboardControl.marker_pos[3]])
    #     print("Approaching Marker")

    def on_exit_SCAN(self, *args):
        self.reset_moving_avg = False
        self.x_app_setpoint_app = []
        self.y_app_setpoint_app = []
        OffboardControl.marker_window.clear()

    def on_exit_LAND(self, *args):
        # OffboardControl.magnet_publisher("mag")
        pass

    def on_exit_PREP_LAND(self, *args):
        self.first_call = True

    def on_enter_ABORT(self, *args):
        msg = Pose()
        msg.position.x = OffboardControl.home_pose.position.x
        msg.position.y = OffboardControl.home_pose.position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.home_pose.orientation
        self.update_setpoint(msg)
        rospy.loginfo("DRONE IS ABORTING LANDING. CAUTION...")

    def on_enter_MAN_OVERRIDE(self, *args):
        self.stop_thread()
    
    def on_enter_IDLE(self, *args):
        self.stop_thread()

    def on_enter_DISARM(self, *args):
        self.stop_thread()

    def stop_thread(self):
        # Stop the publisher thread and join it if it's running
        if self.thread_start and OffboardControl.pub_thread and OffboardControl.pub_thread.is_alive():
            OffboardControl.stop_pub_thread.set()
            OffboardControl.pub_thread.join(timeout=2.0)
            self.thread_start = False

    def marker_found(self):
        # print("Marker Found:" + str(OffboardControl.aruco_found))
        rospy.loginfo(f"Marker Found: {OffboardControl.aruco_found}")
        if (
            not OffboardControl.aruco_found
            and not OffboardControl.first_aruco_msg
        ):
            self.marker_detect_attempt = self.marker_detect_attempt + 1

        return OffboardControl.aruco_found and OffboardControl.first_aruco_msg

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

        return distance > 0.3
        # return (np.sqrt(np.square(OffboardControl.curr_pos[0] - 1.25944) + np.square(OffboardControl.curr_pos[1] - 0.0202361))) > 0.05

    def attempt_check(self):
        # print(f"Scan Attempt: {self.scan_attempt} | ")
        # print(f"Marker Detect Attempt: {self.marker_detect_attempt}")
        self.scan_check()
        self.marker_found()

        if self.scan_attempt >= 25 and self.marker_detect_attempt >= 25:
            return True
        return False

    def setpoint_check(self):
        # Check odom if x500 has reached the setpoint
        setpointReached = False
        sp2Validiate = OffboardControl.target_pose.position

        if (
            (
                sp2Validiate.x - 0.05
                < OffboardControl.current_pose.position.x
                < sp2Validiate.x + 0.05
            )
            and (
                sp2Validiate.y - 0.05
                < OffboardControl.current_pose.position.y
                < sp2Validiate.y + 0.05
            )
            and (
                sp2Validiate.z - 0.05
                < OffboardControl.current_pose.position.z
                < sp2Validiate.z + 0.05
            )
        ):
            if (
                (-0.04 < OffboardControl.current_vel.linear.x < 0.04)
                and (-0.04 < OffboardControl.current_vel.linear.y < 0.04)
                and (-0.04 < OffboardControl.current_vel.linear.z < 0.04)
            ):
                setpointReached = True

        return setpointReached

    def attitude_check(self):
        # Check if x500 is level
        drone_level = False

        mav_rpy = euler_from_quaternion(OffboardControl.current_pose.orientation)

        if (-0.05 < mav_rpy[0] < 0.05) and (
            -0.05 < mav_rpy[1] < 0.05
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

    def moving_avg(self):
        # Calculate Moving Average using cumulative sum and window of 10
        window = 10
        sp_x = []
        sp_y = []

        if not self.reset_moving_avg:
            sp_x = np.fromiter((p.x for p in OffboardControl.marker_window),
                            dtype=np.float64, count=len(OffboardControl.marker_window))
            sp_y = np.fromiter((p.y for p in OffboardControl.marker_window),
                            dtype=np.float64, count=len(OffboardControl.marker_window))
            self.reset_moving_avg = True

        sum_x = np.empty(len(OffboardControl.marker_window)+1, dtype=np.float64)
        sum_y = np.empty(len(OffboardControl.marker_window)+1, dtype=np.float64)
        sum_x[0] = 0.0
        sum_y[0] = 0.0
        np.cumsum(sp_x, out=sum_x[1:])
        np.cumsum(sp_y, out=sum_y[1:])

        moving_avg_x = (sum_x[window:] - sum_x[:-window]) / window
        moving_avg_y = (sum_y[window:] - sum_y[:-window]) / window

        return moving_avg_x, moving_avg_y

    def scan_check(self):
        if (len(OffboardControl.marker_window) == 20):
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
        self.update_setpoint(take_off_pose)
        
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

    def set_finapp_setpoint(self):
        if not self.offapp_flag:
            self.x_app = np.mean(self.x_app_setpoint_app)
            self.y_app = np.mean(self.y_app_setpoint_app)
            self.offapp_flag = True
            
        msg = Pose()
        msg.position.x = self.x_app + OffboardControl.dock_pose.position.x
        msg.position.y = self.y_app + OffboardControl.dock_pose.position.y
        msg.position.z = self.flight_height
        msg.orientation = OffboardControl.dock_pose.orientation

        rospy.loginfo(f"Final Marker App: {msg.position.x}, Y: {msg.position.y}")

        self.update_setpoint(msg)
        rospy.loginfo("Final Approach...")
    
    def offapp_check(self):
        return self.offapp_flag

    def set_final_setpoint(self):
        self.final_setpoint.position.x = OffboardControl.current_pose.position.x - OffboardControl.home_pose.position.x
        self.final_setpoint.position.y = OffboardControl.current_pose.position.y - OffboardControl.home_pose.position.y
        self.final_setpoint.position.z = OffboardControl.current_pose.position.z - OffboardControl.home_pose.position.z
        self.final_setpoint.orientation = OffboardControl.home_pose.orientation

    def landing_check(self):
        drone_land = False

        self.final_setpoint.position.z -= 0.04
        self.update_setpoint(self.final_setpoint)

        rospy.loginfo(f"Landing Setpoint Z: {self.final_setpoint.position.z}")

        # if OffboardControl.droneState.state != "ABORT":
        # self.update_setpoint([1.25944,0.0202361, new_z, OffboardControl.marker_pos[3]])
        # return drone_land
        #if OffboardControl.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
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

    def __init__(self):
        rospy.init_node('offb_node_py')

        self.homeSetPos = False
        self.mag_status = FG40Feedback()

        self.sp_pub_rate = 20
        self.control_rate = 1.5
        #self.rate = rospy.Rate(self.rate_hz)

        OffboardControl.stop_pub_thread = threading.Event()
        OffboardControl.pub_thread = threading.Thread(target=self.trajectory_setpoint_publisher)
        OffboardControl.pub_thread.daemon = True

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
        
        #IS THIS setpoint_raw/target_attitude?
        # self.vehicle_att_set_subscriber = self.create_subscription(
        #     VehicleAttitudeSetpoint,
        #     "/fmu/out/vehicle_attitude_setpoint",
        #     self.vehicle_att_set_callback,
        #     qos_profile,
        # )

        #Replacess VehicleLandDetected but is missing the land detector checks built into uORB message.
        self.drone_status_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, self.vehicle_land_det_callback)
        
        #Replaces ArucoMarkers
        self.aruco_subscriber = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.aruco_callback)

        # Do I need this?
        # self.aruco_baselink_subscriber = self.create_subscription(
        #     Pose, "/aruco_baselink", self.aruco_baselink_callback, 10
        # )
        self.thrust_subscriber = rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.thrust_callback)
        self.spot_pos_subscriber = rospy.Subscriber("/spot_pose", PoseStamped, self.spot_pos_callback)
        self.dock_pos_subscriber = rospy.Subscriber("/dock_pose", PoseStamped, self.dock_pos_callback)

        # Clients
        #Not using arming service. Opting to arm via CMD Long service
        #rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/command')
        rospy.wait_for_service('/mavros/set_mode')
        #self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        
        # self.current_state.mode = State.MODE_PX4_READY #NAVIGATION_STATE_MAX
        # self.current_state.armed = False #ARMING_STATE_MAX
        # self.current_state.system_status = 3 #MAV_STATE_STANDBY


        OffboardControl.spot_pose.position.x = 2.1
        OffboardControl.spot_pose.position.y = 0.0
        OffboardControl.spot_pose.position.z = 0.0
        OffboardControl.spot_pose.orientation = OffboardControl.home_pose.orientation

        # OffboardControl.dock_pose.position.x = -0.8
        # OffboardControl.dock_pose.position.y = 0.0
        # OffboardControl.dock_pose.position.z = 0.0
        # OffboardControl.dock_pose.orientation = OffboardControl.home_pose.orientation


        self.states = [
            "IDLE",
            "FAILSAFE",
            "ARM",
            "DISARM",
            "TAKEOFF",
            "LOITER",
            "SEARCH",
            "SCAN",
            "APPROACH",
            "FINAPP",
            "LAND",
            "PREP_LAND",
            "ABORT",
            "MAN_OVERRIDE",
        ]

        self.droneState = DroneState()
        self.droneState.controller = self
        self.machine = Machine(
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
            prepare=["set_takeoff_setpoint"],
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
            "trs_next", "TAKEOFF", "LOITER", conditions=["setpoint_check"]
        )

        self.machine.add_transition(
            "trs_next", "LOITER", "SEARCH", conditions=["setpoint_check"]
        )
        self.machine.add_transition(
            "trs_next",
            "SEARCH",
            "SCAN",
            conditions=["setpoint_check", "attitude_check"],
        )

        # Perform moving avg?
        self.machine.add_transition(
            "trs_next",
            "SCAN",
            "APPROACH",
            before=["set_approach_setpoint"],
            conditions=["scan_check", "distance_check", "marker_found"],
        )
        # Perform another scan?
        self.machine.add_transition(
            "trs_next",
            "APPROACH",
            "SCAN",
            conditions=["setpoint_check", "attitude_check"],
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
            conditions=["setpoint_check", "attitude_check"],
        )

        self.machine.add_transition(
            "trs_next",
            "SCAN",
            "FINAPP",
            before=["set_finapp_setpoint"],
            conditions=["scan_check", "marker_found"],
            unless=["distance_check"],
        )

        self.machine.add_transition(
            "trs_next",
            "FINAPP",
            "PREP_LAND",
            conditions=["setpoint_check", "attitude_check", "time_check"],
        )

        self.machine.add_transition(
            "trs_next",
            "PREP_LAND",
            "LAND",
            prepare=["set_final_setpoint"],
            conditions=["setpoint_check", "attitude_check"],
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
            #== VehicleStatus.ARMING_STATE_STANDBY,
        )

        #This will use mavros_msgs/State
        # Don't want to transition to MAN_OVERRIDE in IDLE or during ARM. So maybe don't use a wild card here?
        # self.machine.add_transition('trs_next', '*', 'MAN_OVERRIDE', conditions = lambda: self.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL)
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
            # == VehicleStatus.NAVIGATION_STATE_POSCTL
            # or self.nav_state == VehicleStatus.NAVIGATION_STATE_MANUAL,
        )
        self.machine.add_transition("trs_next", "MAN_OVERRIDE", "MAN_OVERRIDE")

        # self.machine.add_transition('trs_next', 'LAND', 'IDLE', conditions=['test'])

    def vehicle_status_callback(self, msg):
        # msg.system_status = 8 -> FAILSAFE
        # msg.mode = OFFBOARD -> OFFBOARD
        # msg.arm = T/F -> Armed?

        #msg.system_status = 5 -> Failsafe state but UAV can navigate

        OffboardControl.current_state = msg


        # self.arm_state = msg.arming_state
        # self.nav_state = msg.nav_state
        # self.failsafe_state = msg.failsafe

    def localpos_callback(self, msg):
        OffboardControl.current_pose = msg.pose

        #rospy.loginfo(f"LOCAL EULER: {euler_from_quaternion(OffboardControl.current_pose.orientation)}")

        if not self.homeSetPos:
            OffboardControl.home_pose = msg.pose
            self.homeSetPos = True

    def localvel_callback(self, msg):
        OffboardControl.current_vel = msg.twist

    def thrust_callback(self, msg):
        OffboardControl.curr_thrust = msg.thrust

    def vehicle_land_det_callback(self, msg):
        OffboardControl.landed_state = msg.landed_state
        # OffboardControl.close_to_ground = msg.close_to_ground_or_skipped_check
        # OffboardControl.has_low_throttle = msg.has_low_throttle
        # # OffboardControl.in_descend = msg.in_descend

    # NEED TO WORK ON THIS
    def aruco_callback(self, msg):
        
        for marker in msg.markers:
            if marker.id == 121:
                if marker.confidence > 0.8 and self.droneState.state == "SCAN":
                    OffboardControl.aruco_found = True
                    if not OffboardControl.first_aruco_msg and OffboardControl.aruco_found:
                        OffboardControl.first_aruco_msg = True
                    
                    #Assuming Hamilton convention
                    # Want to remove this later...
                    OffboardControl.aruco_pose = marker.pose.pose
                    aruco_rpy = euler_from_quaternion(marker.pose.pose.orientation)

                    marker_position = Point()
                    marker_position = marker.pose.pose.position
    
                    if (len(OffboardControl.marker_window) < 20):
                        OffboardControl.marker_window.append(marker_position)
                    elif (len(OffboardControl.marker_window) == 20):
                        OffboardControl.marker_window.pop(0)
                        OffboardControl.marker_window.append(marker_position)

            else:
                OffboardControl.aruco_found = False

        #Should I check to make sure this is being published ref to /map?


    def fg40_status_callback(self, msg):
        self.mag_status = msg

    def spot_pos_callback(self, msg):

        OffboardControl.spot_pose = msg.pose

        #Remove this
        # OffboardControl.spot_pos[0] = msg.position.x
        # OffboardControl.spot_pos[1] = msg.position.y

        # q = [
        #     msg.orientation.x,
        #     msg.orientation.y,
        #     msg.orientation.z,    
        #     msg.orientation.w,
        # ]

        # spot_rpy = euler_from_quaternion(q)
        # OffboardControl.spot_pos[2] = spot_rpy[2]

    def dock_pos_callback(self, msg):

        OffboardControl.dock_pose = msg.pose
        # OffboardControl.dock_pos[0] = msg.position.x
        # OffboardControl.dock_pos[1] = msg.position.y
        # OffboardControl.dock_pos[2] = msg.position.z

        # q = [
        #     msg.orientation.x,
        #     msg.orientation.y,
        #     msg.orientation.z,
        #     msg.orientation.w,
        # ]

        # dock_rpy = euler_from_quaternion(q)
        # OffboardControl.spot_pos[2] = dock_rpy[3]

    # def arm(self):
    #     self.publish_vehicle_command(400, 1.0)
    #     rospy.loginfo('Arm command sent')

    def disarm(self):
        self.call_vehicle_command(400, 0.0, 21196)
        rospy.loginfo("DISARM COMMAND INVOKED")


    # def offboard_activate(self):
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2= 6.0)
    #     rospy.loginfo("Switch to Offboard")

    # def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
    #     #https://mavlink.io/en/messages/common.html#mav_commands
    #     msg = CommandLong()
    #     msg.param1 = float(param1)
    #     msg.param2 = float(param2)
    #     msg.command = command
    #     # msg.target_system = 1
    #     # msg.target_component = 1
    #     # msg.source_system = 1
    #     # msg.source_component = 1
    #     # msg.from_external = True
    #     self.vehicle_cmd_pub.publish(msg)

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

    #Only need to send setpoints, no heartbeat
    # def publish_offboard_heartbeat(self):
    #     msg = OffboardControlMode()
    #     msg.position = True
    #     msg.velocity = False
    #     msg.acceleration = False
    #     msg.attitude = False
    #     msg.body_rate = False
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

    #     self.offboard_control_mode_pub.publish(msg)

    def trajectory_setpoint_publisher(self):
        rate = rospy.Rate(self.sp_pub_rate)
        
        while not rospy.is_shutdown() and not OffboardControl.stop_pub_thread.is_set():
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
        """Launch State thread, to enter offboard control loop. ARM state will start the setpoint publisher thread."""
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            rospy.loginfo(f"CURRENT STATE: {self.droneState.state}")
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

