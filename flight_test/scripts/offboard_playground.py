#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandLong
from fg40_msgs.msg import FG40Feedback, FG40MagnetCmd
import numpy as np
import math
from transitions import Machine

class DroneState:
    def __init__(self):
        self.final_setpoint = [0, 0, 0]
        self.flight_height = 1.5
        self.new_z = self.flight_height
        self.reset_moving_avg = False
        self.x_setpoints = []
        self.y_setpoints = []
        self.x_app_setpoint_app = []
        self.y_app_setpoint_app = []
        self.x_offset = -0.326
        self.y_offset = 0.0
        self.scan_attempt = 0
        self.marker_detect_attempt = 0
        self.start_time = None
        self.first_call = True
        self.thread_start = False
        #self.clock = Clock()
        #self.logger = logger

    def test(self):
        # print(OffboardControl.curr_pos)
        # print(OffboardControl.setpoints)
        return False

    def update_setpoint(self, target):
        
        OffboardControl.target_pose.position.x = target.position.x + OffboardControl.home_pose.position.x
        OffboardControl.target_pose.position.y = target.position.y + OffboardControl.home_pose.position.y
        OffboardControl.target_pose.position.z = target.position.z + OffboardControl.home_pose.position.z

        OffboardControl.target_pose.orientation.x = target.orientation.x + OffboardControl.home_pose.orientation.x
        OffboardControl.target_pose.orientation.y = target.orientation.y + OffboardControl.home_pose.orientation.y
        OffboardControl.target_pose.orientation.z = target.orientation.z + OffboardControl.home_pose.orientation.z
        OffboardControl.target_pose.orientation.w = target.orientation.w + OffboardControl.home_pose.orientation.w

        rospy.loginfo(
            f"Updating Setpoint - X: {OffboardControl.target_pose.position.x}, Y: {OffboardControl.target_pose.position.y}, Z: {OffboardControl.target_pose.position.z}, YAW: {OffboardControl.target_pose.orientation}"
        )

    def on_enter_FAILSAFE(self, *args):
        # print("FAILSAFE ENTERED, RESTART PROGRAM...")
        rospy.loginfo("FAILSAFE ENTERED, RESTART PROGRAM...")

    def on_enter_ARM(self, *args):
        if not self.thread_start:
            OffboardControl._pub_thread.start()
            self.thread_start = True

            

    # def on_enter_TAKEOFF(self, *args):
    #     self.update_setpoint([0,0,-1.25,0])
    #     #rospy.loginfo("Sending Takeoff Setpoint")
    #     print("Sending Takeoff Setpoint")

    def on_exit_IDLE(self, *args):
        # print(OffboardControl.home_pos)
        rospy.loginfo(f"Home Position Recorded: {OffboardControl.home_pose}")

    def on_exit_LOITER(self, *args):
        # self.update_setpoint([1.5,0,self.flight_height,0])
        self.update_setpoint(OffboardControl.spot_pose)
        # print("Sending Search Setpoint")
        rospy.loginfo("Sending Search Setpoint")

    # def on_enter_APPROACH(self, *args):
    #     self.update_setpoint([OffboardControl.marker_pos[0],OffboardControl.marker_pos[1],OffboardControl.curr_pos[2],OffboardControl.marker_pos[3]])
    #     #self.update_setpoint([1.25944,0.0202361,-1.25,OffboardControl.marker_pos[3]])
    #     print("Approaching Marker")

    def on_exit_SCAN(self, *args):
        self.reset_moving_avg = False
        self.x_app_setpoint_app = []
        self.y_app_setpoint_app = []
        self.x_setpoints = []
        self.y_setpoints = []
        OffboardControl.marker_pos_x.clear()
        OffboardControl.marker_pos_y.clear()

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
        # print("Sending Search Setpoint")
        rospy.loginfo("DRONE IS ABORTING LANDING. CAUTION")

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
        # print(
        #     np.sqrt(
        #         np.square(
        #             OffboardControl.curr_pos[0] - OffboardControl.marker_pos[0]
        #         )
        #         + np.square(
        #             OffboardControl.curr_pos[1] - OffboardControl.marker_pos[1]
        #         )
        #     )
        # )
        distance = np.sqrt(
            np.square(
                OffboardControl.curr_pos[0] - OffboardControl.marker_pos[0]
            )
            + np.square(
                OffboardControl.curr_pos[1] - OffboardControl.marker_pos[1]
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

        if (-0.05 < OffboardControl.rpy[0] < 0.05) and (
            -0.05 < OffboardControl.rpy[1] < 0.05
        ):
            drone_level = True

        return drone_level

    def time_check(self):

        if self.first_call:
            self.start_time = rospy.get_rostime()
            self.first_call = False

        current_time = rospy.get_rostime()
        elapsed_duration = current_time - self.start_time
        time_delta_sec = elapsed_duration.nanoseconds / 1e9
        # print(time_delta_sec)
        rospy.loginfo(f"Timer: {time_delta_sec}")
        if time_delta_sec >= 2.0:
            return True

        return False

    def moving_avg(self):

        # Simple Moving Average
        window = 10

        if not self.reset_moving_avg:
            self.x_setpoints = np.array(OffboardControl.marker_pos_x)
            self.y_setpoints = np.array(OffboardControl.marker_pos_y)
            self.reset_moving_avg = True

        weight = np.ones(window) / window

        moving_avg_x = np.convolve(self.x_setpoints, weight, mode="valid")
        moving_avg_y = np.convolve(self.y_setpoints, weight, mode="valid")

        return moving_avg_x, moving_avg_y

    def scan_check(self):

        if (len(OffboardControl.marker_pos_x) == 20) and (
            len(OffboardControl.marker_pos_y) == 20
        ):
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
        # take_off_pose.orientation.w = 0.0
        # take_off_pose.orientation.x = 0.0
        # take_off_pose.orientation.y = 0.0
        # take_off_pose.orientation.z = 0.0
        self.update_setpoint(take_off_pose)
        rospy.loginfo("Sending Takeoff Setpoint")
        # print("Sending Takeoff Setpoint")

    def set_approach_setpoint(self):
        self.update_setpoint(
            [
                (
                    np.mean(self.x_app_setpoint_app)
                    + OffboardControl.dock_pos[0]
                ),
                (
                    ((np.mean(self.y_app_setpoint_app)) * -1)
                    + OffboardControl.dock_pos[1]
                ),
                self.flight_height,
                OffboardControl.dock_pos[3],
            ]
        )
        # self.update_setpoint([1.25944,0.0202361,-1.25,OffboardControl.marker_pos[3]])
        # print("Approaching Marker")
        rospy.loginfo("Approaching Marker")

    def set_final_setpoint(self):
        self.final_setpoint[0] = (
            OffboardControl.curr_pos[0] - OffboardControl.home_pos[0]
        )
        self.final_setpoint[1] = (
            OffboardControl.curr_pos[1] - OffboardControl.home_pos[1]
        )
        self.final_setpoint[2] = (
            OffboardControl.curr_pos[3] - OffboardControl.home_pos[3]
        )

    def landing_check(self):

        drone_land = False
        self.new_z += 0.04
        # if OffboardControl.droneState.state != "ABORT":
        self.update_setpoint(
            [
                self.final_setpoint[0],
                self.final_setpoint[1],
                self.new_z,
                self.final_setpoint[2],
            ]
        )
        # self.update_setpoint([1.25944,0.0202361, new_z, OffboardControl.marker_pos[3]])
        # return drone_land
        if (
            OffboardControl.curr_thrust >= -0.48
            and OffboardControl.close_to_ground
            and OffboardControl.has_low_throttle
            # and OffboardControl.in_descend
        ):
            drone_land = True

        return drone_land

class OffboardControl:
    current_state = State()
    current_pose = Pose()
    home_pose = Pose()
    dock_pose = Pose()
    spot_pose = Pose()
    target_pose = Pose()
    current_vel = Twist()
    offboard_counter = 0

    def __init__(self):
        rospy.init_node('offb_node_py')

        # Publishers
        self.magnet_cmd_pub = rospy.Publisher('/fg40_cmd', FG40MagnetCmd, queue_size=10)
        #Replaces TrajectorySetpoint
        self.setpoint_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

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
        self.aruco_subscriber = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.aruco_callback)

        # Do I need this?
        # self.aruco_baselink_subscriber = self.create_subscription(
        #     Pose, "/aruco_baselink", self.aruco_baselink_callback, 10
        # )

        self.spot_pos_subscriber = rospy.Subscriber("/spot_pos", Pose, self.spot_pos_callback)

        #???
        # self.dock_pos_subscriber = self.create_subscription(
            # Po


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
        self.homeSetPos = False
        self.mag_status = FG40Feedback()

        OffboardControl.spot_pose.position.x = 1.5
        OffboardControl.spot_pose.position.y = 0.0
        OffboardControl.spot_pose.position.z = 0.0


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
        # self.droneState = DroneState()
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
            # If the scan failed but you are at the setpoint sent to
            conditions=["attempt_check", "setpoint_check", "attitude_check"],
            # unless=["scan_check", "marker_found"],
        )

        self.machine.add_transition(
            "trs_next",
            "ABORT",
            "LAND",
            conditions=["setpoint_check", "attitude_check"],
        )

        self.machine.add_transition(
            "trs_next",
            "SCAN",
            "FINAPP",
            before=["set_approach_setpoint"],
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
            conditions=lambda: OffboardControl.current_state.mode
            == State.MODE_PX4_READY,
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

        self.sp_pub_rate = 20
        self.control_rate = 0.5
        #self.rate = rospy.Rate(self.rate_hz)

        self._stop_pub_thread = threading.Event()
        OffboardControl._pub_thread = threading.Thread(target=self.trajectory_setpoint_publisher)
        OffboardControl._pub_thread.daemon = True

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

        if not self.homeSetPos:
            OffboardControl.home_pose = msg.pose
            self.homeSetPos = True

    def localvel_callback(sFelf, msg):
        OffboardControl.current_vel = msg.twist

    # def vehicle_att_callback(self, msg):
    #     q = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]

    #     OffboardControl.rpy = euler_from_quaternion(q)

    #     self.curr_yaw = OffboardControl.rpy[2]
    #     if not self.homeSetYaw:
    #         self.home_pos[3] = self.curr_yaw
    #         self.homeSetYaw = True

    # def vehicle_att_set_callback(self, msg):
    #     OffboardControl.curr_thrust = msg.thrust_body[2]

    def vehicle_land_det_callback(self, msg):
        pass
        # OffboardControl.close_to_ground = msg.close_to_ground_or_skipped_check
        # OffboardControl.has_low_throttle = msg.has_low_throttle
        # # OffboardControl.in_descend = msg.in_descend

    # NEED TO WORK ON THIS
    def aruco_callback(self, msg):
        
        
        # self.arucoID = int(msg.marker_ids[0])
        # if self.arucoID == 122 and self.droneState.state == "SCAN":
        #     OffboardControl.aruco_found = True
        # else:
        #     OffboardControl.aruco_found = False
        # # print(self.arucoID)

    def aruco_baselink_callback(self, msg):
        pass
        # # Need a check to ensure that this a new Scan state entrance
        # if self.droneState.state == "SCAN":
        #     if not OffboardControl.first_aruco_msg and self.aruco_found:
        #         OffboardControl.first_aruco_msg = True

        #     q = [
        #         msg.orientation.x,
        #         msg.orientation.y,
        #         msg.orientation.z,
        #         msg.orientation.w,
        #     ]

        #     aruco_rpy = euler_from_quaternion(q)
        #     self.marker_pos[3] = aruco_rpy[2]

        #     # Want to remove this later...
        #     self.marker_pos[0] = msg.position.x
        #     self.marker_pos[1] = msg.position.y
        #     self.marker_pos[2] = msg.position.z

        #     if (len(OffboardControl.marker_pos_x) < 20) and (
        #         len(OffboardControl.marker_pos_y) < 20
        #     ):
        #         OffboardControl.marker_pos_x.append(msg.position.x)
        #         OffboardControl.marker_pos_y.append(msg.position.y)
        #     elif (len(OffboardControl.marker_pos_x) == 20) and (
        #         len(OffboardControl.marker_pos_y) == 20
        #     ):
        #         OffboardControl.marker_pos_x.pop(0)
        #         OffboardControl.marker_pos_y.pop(0)

        #         OffboardControl.marker_pos_x.append(msg.position.x)
        #         OffboardControl.marker_pos_y.append(msg.position.y)

    def fg40_status_callback(self, msg):
        self.mag_status = msg

    def spot_pos_callback(self, msg):

        OffboardControl.spot_pose = msg

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

        OffboardControl.dock_pose = msg
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
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0)
        self.publish_vehicle_command(
            400, 0.0, 21196
        )
        # print("DISARM COMMAND CALLED")
        rospy.loginfo("DISARM COMMAND INVOKED")
        # rospy.loginfo('Shutting down ROS node...')
        # self.destroy_node()
        # rclpy.shutdown()

    # def offboard_activate(self):
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2= 6.0)
    #     rospy.loginfo("Switch to Offboard")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        #https://mavlink.io/en/messages/common.html#mav_commands
        msg = CommandLong()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        # msg.target_system = 1
        # msg.target_component = 1
        # msg.source_system = 1
        # msg.source_component = 1
        # msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

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
        
        while not rospy.is_shutdown():
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
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():

            rospy.loginfo(f"CURRENT STATE: {self.droneState.state}")
            print(OffboardControl.offboard_counter)

            self.droneState.trs_next()
            if self.droneState.state == "DISARM":
                rospy.loginfo("Magnetizing FG40...")
                self.magnet_publisher("mag")
                self.magnet_publisher("mag")
                self.disarm()
                self._stop_pub_thread.set()
                self._pub_thread.join()
                # self.land()
            rate.sleep()
#-----------------------------------------------------------------------------------------------------------------------------------
            
    # def _publish_loop(self):
    #     """Continuously publish setpoints at self.rate_hz until shutdown."""
    #     while not rospy.is_shutdown() and not self.current_state.connected:
    #         self.rate.sleep()

    #     for _ in range(100):
    #         if rospy.is_shutdown() or self._stop_pub_thread.is_set():
    #             return
    #         self.setpoint_publisher.publish(self.pose)
    #         self.offboard_counter += 1
    #         self.rate.sleep()

    #     while not rospy.is_shutdown() and not self._stop_pub_thread.is_set():
    #         self.setpoint_publisher.publish(self.pose)
    #         self.rate.sleep()

    # def arm_and_offboard_loop(self):
    #     """In the main thread: try to switch mode and arm every 5 s."""
    #     # offb_req = SetModeRequest()
    #     # offb_req.custom_mode = 'OFFBOARD'
    #     # arm_req = CommandBoolRequest()
    #     # arm_req.value = True

    #     last_req = rospy.Time.now()

    #     while not rospy.is_shutdown():
    #         now = rospy.Time.now()
    #         # if self.current_state.mode != 'OFFBOARD' and (now - last_req) > rospy.Duration(5.0):
    #         #     resp = self.set_mode_client.call(offb_req)
    #         #     if resp.mode_sent:
    #         #         rospy.loginfo('[OffboardController] OFFBOARD enabled')
    #         #     last_req = now
        
    #         if (now - last_req) > rospy.Duration(1.0):
    #             self.rate = rospy.Rate(self.rate_hz)
    #             self.state_callback()
    #             # resp = self.arming_client.call(arm_req)
    #             # if resp.success:
    #             #     rospy.loginfo('[OffboardController] Vehicle armed')
    #             last_req = now

    #         self.rate.sleep()

    def start(self):
        """Launch publisher thread, then enter arm/mode control loop."""

        try:
            #self.arm_and_offboard_loop()
            self.state_parser()
        except rospy.ROSInterruptException:
            pass
        finally:
            pass
            # self._stop_pub_thread.set()
            # self._pub_thread.join()

if __name__ == '__main__':
    controller = OffboardControl()
    controller.start()

