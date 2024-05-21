import rclpy
import numpy as np
import time
from transitions import Machine
import math
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.clock import Clock
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, Timesync, VehicleOdometry, VehicleLocalPosition, VehicleLocalPositionSetpoint, LandingTargetPose, VehicleAttitude
from ros2_aruco_interfaces.msg import ArucoMarkers

class DroneState():
    #states=['IDLE', 'ARM', 'TAKEOFF', 'LOITER', 'SEARCH', 'SCAN', 'LAND']

    def __init__(self):
        pass
        # self.machine = Machine(model=self, states=DroneState.states, initial= 'IDLE')
        
        # # transitions = [
        # #     { 'trigger': 'melt', 'source': 'solid', 'dest': 'liquid', 'prepare': ['heat_up', 'count_attempts'], 'conditions': 'is_really_hot', 'after': 'stats'},
        # # ]
        
        # self.machine.add_transition('trs_next', 'IDLE', 'TAKEOFF', conditions=['test'], before='update_setpoint')
        # self.machine.add_transition('trs_next', 'IDLE', 'ARM')
    
    def test(self):
        print(OffboardControl.nav_state)
        return False

    def arm_check(self):
        pass

    def update_setpoint(self, setpoint):
        set_x = setpoint[0] + OffboardControl.home_pos[0]
        set_y = setpoint[1] + OffboardControl.home_pos[1]
        set_z = setpoint[2] + OffboardControl.home_pos[2]
        set_yaw = setpoint[3] + OffboardControl.home_pos[3]


        OffboardControl.setpoints[0] = set_x
        OffboardControl.setpoints[1] = set_y
        OffboardControl.setpoints[2] = set_z
        OffboardControl.setpoints[3] = set_yaw
    
    def takeoff_check(self):
        if OffboardControl.arm_state == VehicleStatus.ARMING_STATE_ARMED and OffboardControl.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return True

    # def setpointChecker(self):
    #     #Check odom if x500 has reached the setpoint
    #     setpointReached = False
    #     sp2Validiate = self.setpoints
    #     sp2ValidiateAruco = self.aruco_setpoint

    #     if (sp2Validiate[0] - 0.05 < self.curr_x < sp2Validiate[0] + 0.05) and (sp2Validiate[1] - 0.05 < self.curr_y < sp2Validiate[1] + 0.05) and (sp2Validiate[2] + 0.05 > self.curr_z > sp2Validiate[2] - 0.05):
    #         if (-0.08 < self.curr_vx < 0.08) and  (-0.08 < self.curr_vy < 0.08) and  (0.08 > self.curr_vz > -0.08):
    #             setpointReached = True
            #return setpointReached


class OffboardControl(Node):

    nav_state = VehicleStatus.NAVIGATION_STATE_MAX
    #arm_state = VehicleStatus.ARMING_STATE_MAX
    setpoints = [0.0, 0.0, 0.0, 0.0]
    home_pos = [0.0, 0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__('offboard_control_landing')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        #Publishers
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', qos_profile)
        #self.localpos_pub = self.create_publisher(VehicleLocalPositionSetpoint, '/fmu/vehicle_local_position_setpoint/in', qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, 'fmu/offboard_control_mode/in', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', qos_profile)


        #Subscribers
        self.drone_status_sub = self.create_subscription(VehicleStatus, '/fmu/vehicle_status/out', self.vehicle_status_callback, qos_profile)
        #self.timesync_subscriber = self.create_subscription(Timesync, '/fmu/timesync/out', self.timesync_callback, qos_profile)
        self.localpos_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/vehicle_local_position/out', self.localpos_callback, qos_profile)
        self.vehicle_att_subscriber = self.create_subscription(VehicleAttitude, '/fmu/vehicle_attitude/out', self.vehicle_att_callback, qos_profile)
        self.aruco_subscriber = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        self.aruco_baselink_subscriber = self.create_subscription(Pose, '/aruco_baselink', self.aruco_baselink_callback, qos_profile)


        self.curr_x, self.curr_y, self.curr_z, self.curr_yaw = 0.0, 0.0, 0.0, 0.0
        self.curr_vx, self.curr_vy, self.curr_vz = 0.0, 0.0, 0.0
        self.curr_ax, self.curr_ay, self.curr_az = 0.0, 0.0, 0.0
        # self.aruco_x, self.aruco_y, self.aruco_z = 1.5, 1.5, -1.5
        # self.home_x, self.home_y, self.home_z, self.home_yaw = 0.0, 0.0, 0.0, 0.0
        #self.set_x, self.set_y, self.set_z, self.set_yaw = 0.0, 0.0, 0.0, 0.0
        # self.final_x, self.final_y = 0.0, 0.0
        # self.arucoSample_x, self.arucoSample_y, self.new_aruco_x, self.new_aruco_y = 0.0, 0.0, 0.0, 0.0
        # self.arucoFound = False
        # self.aruco_yaw = 0.0
        # self.arucoCount = 0
        self.homeSetPos = False
        self.homeSetYaw = False
        # self.average = False
        self.timestamp = 0
        self.offboard_counter = 0
        # self.current_setpoint_index = 0
        # self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_MAX
        # self.armFlag = False
        # self.arucoID = 0
        # self.arucoFlag = False
        # self.posCounter = 0
        # self.takeoff = False
        # self.start_time = time.time()
        # self.exec_time = 0


        # self.aruco_setpoint = [self.curr_x, self.curr_y, self.curr_z, self.curr_yaw]

        #self.states=['IDLE', 'OFFBOARD', 'TAKEOFF', 'LOITER', 'SEARCH', 'SCAN', 'LAND']
        # self.transitions = [
        #     { 'trigger': 'takeoff', 'source': 'IDLE', 'dest': 'TAKEOFF'},
        #     { 'trigger': 'hold', 'source': 'TAKEOFF', 'dest': 'LOITER' },
        #     { 'trigger': 'search', 'source': 'LOITER', 'dest': 'SEARCH' },
        #     { 'trigger': 'arucoScan', 'source': 'SEARCH', 'dest': 'SCAN' },
        #     { 'trigger': 'land', 'source': 'SCAN', 'dest': 'LAND' }
        # ]

        # machine = Machine(self.droneState, states=self.states, initial= 'IDLE')

        # machine.add_transition('takeoff', 'IDLE', 'TAKEOFF')
        # machine.add_transition('offboard', 'IDLE', 'OFFBOARD')

        self.states=['IDLE', 'ARM', 'TAKEOFF', 'LOITER', 'SEARCH', 'SCAN', 'LAND']
        
        self.droneState = DroneState()
        self.machine = Machine(model=self.droneState , states=self.states, initial= 'IDLE')
        

        self.machine.add_transition('trs_next', 'IDLE', 'ARM', conditions = lambda: self.arm_state == VehicleStatus.ARMING_STATE_ARMED)
        self.machine.add_transition('trs_next', 'ARM', 'IDLE', conditions = lambda: self.arm_state != VehicleStatus.ARMING_STATE_ARMED)
        self.machine.add_transition('trs_next', 'ARM', 'TAKEOFF', conditions=['takeoff_check'])
        
  
        timer_state = 0.1  # seconds
        self.timer_offboard = self.create_timer(timer_state, self.state_callback)

        timer_period = 0.02  # seconds
        self.timerA = self.create_timer(timer_period, self.cmdloop_callback)


    def vehicle_status_callback(self, msg):
        OffboardControl.nav_state = msg.nav_state
        #OffboardControl.arm_state = msg.arming_state
        self.arm_state = msg.arming_state

    
    def localpos_callback(self, msg):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_z = msg.z

        self.curr_vx = msg.vx
        self.curr_vy = msg.vy
        self.curr_vz = msg.vz

        self.curr_ax = msg.ax
        self.curr_ay = msg.ay
        self.curr_az = msg.az

        self.timestamp = msg.timestamp

        if self.homeSetPos == False:
            self.home_pos[0] = self.curr_x
            self.home_pos[1] = self.curr_y
            self.home_pos[2] = self.curr_z

            self.homeSetPos = True
    

    def vehicle_att_callback(self, msg):
        q = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]

        rpy = euler_from_quaternion(q)

        self.curr_yaw = rpy[2]
        if self.homeSetYaw == False:
            self.home_pos[3] = self.curr_yaw
            self.homeSetYaw = True

    
    def aruco_callback(self, msg):
        self.arucoID = int(msg.marker_ids[0])
        if self.arucoID == 122:
            self.arucoFound = True
        #print(self.arucoID)
    

    def aruco_baselink_callback(self, msg):
            self.aruco_x = msg.position.x
            self.aruco_y = (msg.position.y) * -1
            self.aruco_z = msg.position.z

            self.aruco_qx = msg.orientation.x
            self.aruco_qy = msg.orientation.y
            self.aruco_qz = msg.orientation.z
            self.aruco_qw = msg.orientation.w
            print("CALLED")
            
        #print(self.aruco_x)

        # self.aruco_q = [self.aruco_qx, self.aruco_qy, self.aruco_qz, self.aruco_qw]
        # self.aruco_roll, self.aruco_pitch, self.aruco_yaw = euler_from_quaternion(self.aruco_q)
        

    # def setpointChecker(self):
    #     #Check odom if x500 has reached the setpoint
    #     setpointReached = False
    #     sp2Validiate = self.setpoints
    #     sp2ValidiateAruco = self.aruco_setpoint

    #     if (sp2Validiate[0] - 0.05 < self.curr_x < sp2Validiate[0] + 0.05) and (sp2Validiate[1] - 0.05 < self.curr_y < sp2Validiate[1] + 0.05) and (sp2Validiate[2] + 0.05 > self.curr_z > sp2Validiate[2] - 0.05):
    #         if (-0.08 < self.curr_vx < 0.08) and  (-0.08 < self.curr_vy < 0.08) and  (0.08 > self.curr_vz > -0.08):
    #             setpointReached = True


        # if self.arucoFlag == True:
        #     if (sp2ValidiateAruco[0] - 0.05 < self.curr_x < sp2ValidiateAruco[0] + 0.05) and (sp2ValidiateAruco[1] - 0.05 < self.curr_y < sp2ValidiateAruco[1] + 0.05) and (sp2ValidiateAruco[2] + 0.05 > self.curr_z > sp2ValidiateAruco[2] - 0.05):
        #         if (-0.08 < self.curr_vx < 0.08) and  (-0.08 < self.curr_vy < 0.08) and  (0.08 > self.curr_vz > -0.08):
        #             setpointReached = True
        
        # else:
        # #Check Position of x500 against current setpoint
        #     if (sp2Validiate[0] - 0.05 < self.curr_x < sp2Validiate[0] + 0.05) and (sp2Validiate[1] - 0.05 < self.curr_y < sp2Validiate[1] + 0.05) and (sp2Validiate[2] + 0.05 > self.curr_z > sp2Validiate[2] - 0.05):
        #         if (-0.08 < self.curr_vx < 0.08) and  (-0.08 < self.curr_vy < 0.08) and  (0.08 > self.curr_vz > -0.08):
        #             setpointReached = True
                
        #Stil need one for orientation

        #return setpointReached

    
    # def arm(self):
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    #     self.get_logger().info('Arm command sent')


    # def disarm(self):
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    #     self.get_logger().info('Disarm command sent')
    #     self.get_logger().info('Shutting down ROS node...')
    #     self.destroy_node()
    #     rclpy.shutdown()
    

    # #This function needs to be customized. I can't use this because I loose control of drone position if I used PX4 Landing System
    # def land(self):
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    #     #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND)
    #     self.get_logger().info('Landing initiated')
    #     self.disarm()
    

    def offboard_activate(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2= 6.0)
        self.get_logger().info("Switch to Offboard")


    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)


    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp

        self.offboard_control_mode_pub.publish(msg)
    
    # def publish_aruco_pose(self, arucoSetpoint):
    #     msg = TrajectorySetpoint()
    #     msg.timestamp = self.timestamp
    #     msg.x = arucoSetpoint[0]
    #     msg.y = arucoSetpoint[1]
    #     msg.z = arucoSetpoint[2]
    #     #msg.yaw = arucoSetpoint[3]
    #     vel_x = (arucoSetpoint[0] - self.curr_x) / 2
    #     vel_y = (arucoSetpoint[1] - self.curr_y) / 2
    #     vel_z = (arucoSetpoint[2] - self.curr_z) / 2
    #     msg.vx, msg.vy, msg.vz = vel_x, vel_y, vel_z
    #     #msg.vx, msg.vy, msg.vz = 0.9, 0.9, 0.9
    #     self.trajectory_pub.publish(msg)
    

    def trajectory_setpoint_publisher(self, setpoint):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        #msg.x, msg.y, msg.z, msg.yaw = setpoint[index]
        msg.x, msg.y, msg.z= setpoint[0], setpoint[1], setpoint[2]
        vel_x = (setpoint[0] - self.curr_x) / 2
        vel_y = (setpoint[1] - self.curr_y) / 2
        vel_z = (setpoint[2] - self.curr_z) / 2
        msg.vx, msg.vy, msg.vz = vel_x, vel_y, vel_z
        self.trajectory_pub.publish(msg)
    



    def state_callback(self):
        # if self.arm_state == VehicleStatus.ARMING_STATE_ARMED and (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
        #     #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
        #     self.droneState.trigger('takeoff')
        #     #self.offboard_activate()
        # elif self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.offboard_counter >= 10:
        #     self.droneState.trigger('offboard')
        #     self.update_setpoint(0.0, 0.0, -1.25, self.curr_yaw)

        # self.droneState.test()
        # print(self.droneState.state)
        print(self.droneState.state)
        self.droneState.trs_next()
        
        # if self.droneState.state == 'IDLE':
        #     setpoint_to = [0.0, 0.0, -1.25, 0.0]
        #     self.droneState.trigger(setpoint_to)
        
            

    # #Where I want state changes to occur
    def cmdloop_callback(self):

        self.publish_offboard_heartbeat()
        self.trajectory_setpoint_publisher(self.setpoints)

        if self.offboard_counter < 11:
            self.offboard_counter += 1




def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()