import rclpy
import numpy as np
import time
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.clock import Clock
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, Timesync, VehicleOdometry, VehicleLocalPosition, VehicleLocalPositionSetpoint
from ros2_aruco_interfaces.msg import ArucoMarkers

class OffboardControl(Node):

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
        #self.odom_subscriber = self.create_subscription(VehicleOdometry, '/fmu/vehicle_odometry/out', self.odom_callback, qos_profile)
        self.aruco_subscriber = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        self.aruco_baselink_subscriber = self.create_subscription(Pose, '/aruco_baselink', self.aruco_baselink_callback, qos_profile)


        self.curr_x, self.curr_y, self.curr_z, self.curr_yaw = 0.0, 0.0, 0.0, 0.0
        self.curr_vx, self.curr_vy, self.curr_vz = 0.0, 0.0, 0.0
        self.curr_ax, self.curr_ay, self.curr_az = 0.0, 0.0, 0.0
        self.home_x, self.home_y, self.home_z, self.home_yaw = 0.0, 0.0, 0.0, 0.0
        self.timestamp = 0
        self.offboard_counter = 0
        self.current_setpoint_index = 0
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_MAX
        self.arucoID = 0
        self.arucoFlag = False
        self.posCounter = 0
        self.takeoff = False
        self.start_time = time.time()
        self.exec_time = 0

        self.setpoints = [
            (0.0, 0.0, -1.25, 0.0), 
            (1.5, 0.0, -1.25, 0.0),
        ]

        self.idle = self.idle
        self.arm = self.arm
        self.takeoff = self.takeoff
        self.loiter = self.loiter
        self.offboard = self.offboard
        self.search = self.search
        self.land = self.land

        self.current_state = 'IDLE'
        self.last_state = self.current_state

        self.drone_state = {
            'IDLE': self.idle,
            "ARMING": self.arm,
            'TAKEOFF': self.takeoff,
            'LOITER': self.loiter,
            "OFFBOARD": self.offboard,
            "ARUCO_SEARCH":self.search,
            "PRECISION_LAND": self.land
        }

        
        timer_period = 0.02  # seconds
        self.timer_offboard = self.create_timer(timer_period, self.offboard_callback)

        timer_periodA = 0.1  # seconds
        self.timerA = self.create_timer(timer_period, self.cmdloop_callback)



        

    def takeoff(self):
        #self.current_state = 'LOITER'
        print("TAKEOFF")

    def idle(self):
        self.current_state = 'TAKEOFF'
        print("yoy")

    def loiter(self):
        print("hello")

    def offboard(self):
        pass

    def search(self):
        pass


    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state

    def drone_fsm(self):
        if self.current_state in self.drone_state:
            self.drone_state[self.current_state]()
        else:
            print(f"Method '{self.current_state}' not found.")

    
    
    
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


    def setHome(self):
        if self.homeSet == False:
            self.home_x = self.curr_x
            self.home_y = self.curr_y
            self.home_z = self.curr_z
            self.home_yaw = self.curr_yaw
            self.homeSet = True
    
    def aruco_callback(self, msg):
        self.arucoID = int(msg.marker_ids[0])
        print(self.arucoID)
    
    def aruco_baselink_callback(self, msg):
        self.aruco_x = msg.position.x
        self.aruco_y = msg.position.y
        self.aruco_z = msg.position.z

        self.aruco_qx = msg.orientation.x
        self.aruco_qy = msg.orientation.y
        self.aruco_qz = msg.orientation.z
        self.aruco_qw = msg.orientation.w
        #print(self.aruco_x)

        # self.aruco_q = [self.aruco_qx, self.aruco_qy, self.aruco_qz, self.aruco_qw]
        # self.aruco_roll, self.aruco_pitch, self.aruco_yaw = euler_from_quaternion(self.aruco_q)
        

    def setpointChecker(self):
        #Check odom if x500 has reached the setpoint
        setpointReached = False
        sp2Validiate = self.setpoints[self.current_setpoint_index]

        #Check Position of x500 against current setpoint
        if (sp2Validiate[0] - 0.05 < self.curr_x < sp2Validiate[0] + 0.05) and (sp2Validiate[1] - 0.05 < self.curr_y < sp2Validiate[1] + 0.05) and (sp2Validiate[2] + 0.05 > self.curr_z > sp2Validiate[2] - 0.05):
            if (-0.08 < self.curr_vx < 0.08) and  (-0.08 < self.curr_vy < 0.08) and  (0.08 > self.curr_vz > -0.08):
                setpointReached = True

        #Stil need one for orientation

        return setpointReached

    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')


    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')
        self.get_logger().info('Shutting down ROS node...')
        self.destroy_node()
        rclpy.shutdown()
    

    #This function needs to be customized. I can't use this because I loose control of drone position if I used PX4 Landing System
    def land(self):
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND)
        self.get_logger().info('Landing initiated')
        #self.disarm()
    

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
        msg.velocity = False
        #msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp

        self.offboard_control_mode_pub.publish(msg)
    

    def trajectory_setpoint_publisher(self, setpoint, index):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x, msg.y, msg.z, msg.yaw = setpoint[index]
        #msg.vx, msg.vy, msg.vz = 0.9, 0.9, 0.9
        self.trajectory_pub.publish(msg)


    #PX4 quaternions are in Hamilton Convention (w,x,y,z)
    # def odom_callback(self, msg):
    #     self.curr_x = msg.x
    #     self.curr_y = msg.y
    #     self.curr_z = msg.z

    #     #Convert Hamilton Conv. to JPL Conv.
    #     px4_q = [float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0])]
    #     euler = euler_from_quaternion(px4_q)
    #     self.curr_yaw = euler[2]
    #     #print(self.curr_z)

    # def setHome(self):
    #     self.home_x = self.curr_x
    #     self.home_y = self.curr_y
    #     self.home_z = self.curr_z
    #     self.home_q = euler_from_quaternion(self.cur)

    # def timesync_callback(self, msg):
    #     self.timestamp = msg.timestamp

    # def localpos_setpoint_publisher(self, setpoint):
    #     msg = VehicleLocalPositionSetpoint()
    #     msg.timestamp = self.timestamp
    #     msg.x, msg.y, msg.z, msg.yaw = setpoint[self.current_setpoint_index]
    #     #msg.vx, msg.vy, msg.vz = 0.2, 0.2, 0.2
    #     self.localpos_pub.publish(msg)


    #Where the offboard watchdog signal is produced and sent.
    def offboard_callback(self):
        self.publish_offboard_heartbeat()
        self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)

        if self.offboard_counter < 11:
            self.offboard_counter += 1

    #Where I want state changes to occur
    def cmdloop_callback(self):
        #self.drone_state(self.current_state)
        #self.exec_time = time.time() - self.start_time

        #IDLE STATE --> ARM STATE
        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm()

        #ARM STATE --> TAKEOFF STATE
        elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.offboard_counter ==10:
            self.offboard_activate()
        
        #TAKEOFF STATE --> SEARCH STATE (Where it thinks Aruco will be)
        elif self.setpointChecker() == True and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.current_setpoint_index = 1
        
        #SEARCH STATE --> ARUCO SEARCH AND DETECT
        elif self.setpointChecker == True and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.current_setpoint_index == 1:
            self.land()
            # if self.arucoID == 122 and self.arucoFlag == False:
            #     aruco_q = [self.aruco_qx, self.aruco_qy, self.aruco_qz, self.aruco_qw]
            #     aruco_yaw = euler_from_quaternion(aruco_q)
            #     aruco_setpoint = (self.aruco_x, self.aruco_y, self.setpoints[self.current_setpoint_index][2], aruco_yaw(2))
            #     self.setpoints.append(aruco_setpoint)
            #     self.arucoFlag = True
            #     print("Iamhere")
            # elif self.arucoFlag == True:
            #     print("Trying to sit")
            #     self.current_setpoint_index = 2




        
    
            
                


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()