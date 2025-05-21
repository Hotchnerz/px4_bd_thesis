#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
import math
from transitions import Machine

class DroneState:
    def __init__(self, logger):
        self.final_setpoint = [0, 0, 0]
        self.flight_height = -1.5
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
        self.clock = Clock()
        self.logger = logger

    def test(self):
        # print(OffboardControl.curr_pos)
        # print(OffboardControl.setpoints)
        return False

    def update_setpoint(self, setpoint):
        set_x = setpoint[0] + OffboardControl.home_pos[0]
        set_y = setpoint[1] + OffboardControl.home_pos[1]
        set_z = setpoint[2] + OffboardControl.home_pos[2]
        set_yaw = setpoint[3] + OffboardControl.home_pos[3]

        OffboardControl.setpoints[0] = set_x
        OffboardControl.setpoints[1] = set_y
        OffboardControl.setpoints[2] = set_z
        OffboardControl.setpoints[3] = set_yaw
        self.logger.info(
            f"Updating Setpoint - X: {OffboardControl.setpoints[0]}, Y: {OffboardControl.setpoints[1]}, Z: {OffboardControl.setpoints[2]}, YAW: {OffboardControl.setpoints[3]}"
        )

    def on_enter_FAILSAFE(self, *args):
        # print("FAILSAFE ENTERED, RESTART PROGRAM...")
        self.logger.info("FAILSAFE ENTERED, RESTART PROGRAM...")

    # def on_enter_TAKEOFF(self, *args):
    #     self.update_setpoint([0,0,-1.25,0])
    #     #self.get_logger().info("Sending Takeoff Setpoint")
    #     print("Sending Takeoff Setpoint")

    def on_exit_IDLE(self, *args):
        # print(OffboardControl.home_pos)
        self.logger.info(f"Home Position Recorded: {OffboardControl.home_pos}")

    def on_exit_LOITER(self, *args):
        # self.update_setpoint([1.5,0,self.flight_height,0])
        self.update_setpoint(
            [
                OffboardControl.spot_pos[0],
                OffboardControl.spot_pos[1],
                self.flight_height,
                OffboardControl.spot_pos[2],
            ]
        )
        # print("Sending Search Setpoint")
        self.logger.info("Sending Search Setpoint")

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
        self.update_setpoint([0.0, 0.0, self.flight_height, 0])
        # print("Sending Search Setpoint")
        self.logger.info("DRONE IS ABORTING LANDING. CAUTION")

    def marker_found(self):
        # print("Marker Found:" + str(OffboardControl.aruco_found))
        self.logger.info(f"Marker Found: {OffboardControl.aruco_found}")
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

        self.logger.info(f"DISTANCE CHECK: {distance}")

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
        sp2Validiate = OffboardControl.setpoints

        if (
            (
                sp2Validiate[0] - 0.05
                < OffboardControl.curr_pos[0]
                < sp2Validiate[0] + 0.05
            )
            and (
                sp2Validiate[1] - 0.05
                < OffboardControl.curr_pos[1]
                < sp2Validiate[1] + 0.05
            )
            and (
                sp2Validiate[2] + 0.05
                > OffboardControl.curr_pos[2]
                > sp2Validiate[2] - 0.05
            )
        ):
            if (
                (-0.04 < OffboardControl.curr_vel[0] < 0.04)
                and (-0.04 < OffboardControl.curr_vel[1] < 0.04)
                and (0.04 > OffboardControl.curr_vel[2] > -0.04)
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
            self.start_time = self.clock.now()
            self.first_call = False

        current_time = self.clock.now()
        elapsed_duration = current_time - self.start_time
        time_delta_sec = elapsed_duration.nanoseconds / 1e9
        # print(time_delta_sec)
        self.logger.info(f"Timer: {time_delta_sec}")
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
        self.update_setpoint([0, 0, self.flight_height, 0])
        self.logger.info("Sending Takeoff Setpoint")
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
        self.logger.info("Approaching Marker")

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

class OffboardController:
    def __init__(self):
        rospy.init_node('offb_node_py')
        self.current_state = State()

        # Publishers
        # self.magnet_cmd_pub = rospy.Publisher(
        #     "/fg40_cmd", FG40MagnetCmd, queue_size=10
        # )
        self.setpoint_publisher = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscribers
        # self.fg40_status_sub = self.create_subscription(
        #     FG40Feedback, "/fg40_status", self.fg40_status_callback, 10
        # )
        self.drone_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )
        self.localpos_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.localpos_callback,
            qos_profile,
        )
        self.vehicle_att_subscriber = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.vehicle_att_callback,
            qos_profile,
        )
        self.vehicle_att_set_subscriber = self.create_subscription(
            VehicleAttitudeSetpoint,
            "/fmu/out/vehicle_attitude_setpoint",
            self.vehicle_att_set_callback,
            qos_profile,
        )
        self.vehicle_land_det_subscriber = self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self.vehicle_land_det_callback,
            qos_profile,
        )
        self.aruco_subscriber = self.create_subscription(
            ArucoMarkers, "/aruco_markers", self.aruco_callback, 10
        )
        self.aruco_baselink_subscriber = self.create_subscription(
            Pose, "/aruco_baselink", self.aruco_baselink_callback, 10
        )
        self.spot_pos_subscriber = self.create_subscription(
            Pose, "/spot_pos", self.spot_pos_callback, 10
        )
        self.dock_pos_subscriber = self.create_subscription(
            Po


        self.state_subscriber = rospy.Subscriber('mavros/state', State, self._state_cb)
        self. = rospy.Subscriber('mavros/state', State, self._state_cb)

        # Clients
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

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

        self.droneState = DroneState(self.get_logger())
        # self.droneState = DroneState()
        self.machine = Machine(
            model=self.droneState, states=self.states, initial="IDLE"
        )

        self.machine.add_transition(
            "trs_next",
            "IDLE",
            "ARM",
            conditions=lambda: self.arm_state
            == VehicleStatus.ARMING_STATE_ARMED,
        )
        self.machine.add_transition(
            "trs_next",
            "ARM",
            "IDLE",
            conditions=lambda: self.arm_state
            != VehicleStatus.ARMING_STATE_ARMED,
        )
        self.machine.add_transition(
            "trs_next",
            "ARM",
            "TAKEOFF",
            prepare=["set_takeoff_setpoint"],
            conditions=lambda: self.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.offboard_counter > 10,
        )

        # Need to ensure that if this state transition occurs, some sort of clean up like go back into manual mode or pos mode and restart back to Arm or Idle depending on conds.
        self.machine.add_transition(
            "trs_next",
            "TAKEOFF",
            "FAILSAFE",
            conditions=lambda: self.failsafe_state,
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
            conditions=lambda: self.arm_state
            == VehicleStatus.ARMING_STATE_STANDBY,
        )

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
            conditions=lambda: self.nav_state
            == VehicleStatus.NAVIGATION_STATE_POSCTL
            or self.nav_state == VehicleStatus.NAVIGATION_STATE_MANUAL,
        )
        self.machine.add_transition("trs_next", "MAN_OVERRIDE", "MAN_OVERRIDE")

        # self.machine.add_transition('trs_next', 'LAND', 'IDLE', conditions=['test'])


        self.rate_hz = 20
        self.rate = rospy.Rate(self.rate_hz)

        self._stop_pub_thread = threading.Event()
        self._pub_thread = threading.Thread(target=self._publish_loop)
        self._pub_thread.daemon = True

    def _state_cb(self, msg):
        self.current_state = msg

    def _publish_loop(self):
        """Continuously publish setpoints at self.rate_hz until shutdown."""
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        for _ in range(100):
            if rospy.is_shutdown() or self._stop_pub_thread.is_set():
                return
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

        while not rospy.is_shutdown() and not self._stop_pub_thread.is_set():
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

    def arm_and_offboard_loop(self):
        """In the main thread: try to switch mode and arm every 5 s."""
        offb_req = SetModeRequest()
        offb_req.custom_mode = 'OFFBOARD'
        arm_req = CommandBoolRequest()
        arm_req.value = True

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.current_state.mode != 'OFFBOARD' and (now - last_req) > rospy.Duration(5.0):
                resp = self.set_mode_client.call(offb_req)
                if resp.mode_sent:
                    rospy.loginfo('[OffboardController] OFFBOARD enabled')
                last_req = now

            elif not self.current_state.armed and (now - last_req) > rospy.Duration(5.0):
                resp = self.arming_client.call(arm_req)
                if resp.success:
                    rospy.loginfo('[OffboardController] Vehicle armed')
                last_req = now

            self.rate.sleep()

    def start(self):
        """Launch publisher thread, then enter arm/mode control loop."""
        self._pub_thread.start()
        try:
            self.arm_and_offboard_loop()
        except rospy.ROSInterruptException:
            pass
        finally:
            self._stop_pub_thread.set()
            self._pub_thread.join()

if __name__ == '__main__':
    controller = OffboardController()
    controller.start()

