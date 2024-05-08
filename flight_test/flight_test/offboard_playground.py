import rclpy
import numpy as np
import time
import math
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.clock import Clock
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, Timesync, VehicleOdometry, VehicleLocalPosition, VehicleLocalPositionSetpoint, LandingTargetPose, VehicleAttitude
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
        # self.land_pose_pub = self.create_publisher(LandingTargetPose, 'fmu/landing_target_pose/in', 10)
        #self.localpos_pub = self.create_publisher(VehicleLocalPositionSetpoint, '/fmu/vehicle_local_position_setpoint/in', qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, 'fmu/offboard_control_mode/in', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', qos_profile)


        #Subscribers
        self.drone_status_sub = self.create_subscription(VehicleStatus, '/fmu/vehicle_status/out', self.vehicle_status_callback, qos_profile)
        #self.timesync_subscriber = self.create_subscription(Timesync, '/fmu/timesync/out', self.timesync_callback, qos_profile)
        self.localpos_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/vehicle_local_position/out', self.localpos_callback, qos_profile)
        self.vehicle_att_subscriber = self.create_subscription(VehicleAttitude, '/fmu/vehicle_attitude/out', self.vehicle_att_callback, qos_profile)
        #self.odom_subscriber = self.create_subscription(VehicleOdometry, '/fmu/vehicle_odometry/out', self.odom_callback, qos_profile)
        self.aruco_subscriber = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        self.aruco_baselink_subscriber = self.create_subscription(Pose, '/aruco_baselink', self.aruco_baselink_callback, qos_profile)


        self.curr_x, self.curr_y, self.curr_z, self.curr_yaw = 0.0, 0.0, 0.0, 0.0
        self.curr_vx, self.curr_vy, self.curr_vz = 0.0, 0.0, 0.0
        self.curr_ax, self.curr_ay, self.curr_az = 0.0, 0.0, 0.0
        self.aruco_x, self.aruco_y, self.aruco_z = 1.5, 1.5, -1.5
        self.home_x, self.home_y, self.home_z, self.home_yaw = 0.0, 0.0, 0.0, 0.0
        self.final_x, self.final_y = 0.0, 0.0
        self.arucoSample_x, self.arucoSample_y, self.new_aruco_x, self.new_aruco_y = 0.0, 0.0, 0.0, 0.0
        self.arucoFound = False
        self.aruco_yaw = 0.0
        self.arucoCount = 0
        self.homeSetPos = False
        self.homeSetYaw = False
        self.average = False
        self.timestamp = 0
        self.offboard_counter = 0
        self.current_setpoint_index = 0
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_MAX
        self.armFlag = False
        self.arucoID = 0
        self.arucoFlag = False
        self.posCounter = 0
        self.takeoff = False
        self.start_time = time.time()
        self.exec_time = 0

        # self.setpoints = [
        #     [self.home_x, self.home_y, self.home_z + -1.25, self.home_yaw], 
        #     [self.home_x + 1.5, self.home_y, self.home_z + -1.25, self.home_yaw]
        # ]

        self.setpoints = [(0.0, 0.0, 0.0)]
        self.aruco_setpoint = [self.curr_x, self.curr_y, self.curr_z, self.curr_yaw]


  
        timer_period = 0.02  # seconds
        self.timer_offboard = self.create_timer(timer_period, self.offboard_callback)

        timer_periodA = 0.08  # seconds
        self.timerA = self.create_timer(timer_periodA, self.cmdloop_callback)


    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
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
            self.home_x = self.curr_x
            self.home_y = self.curr_y
            self.home_z = self.curr_z

            self.homeSetPos = True
    
    def vehicle_att_callback(self, msg):
        q = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]

        rpy = euler_from_quaternion(q)

        self.curr_yaw = rpy[2]
        if self.homeSetYaw == False:
            self.home_yaw = self.curr_yaw
            self.homeSetYaw = True

    
    def aruco_callback(self, msg):
        self.arucoID = int(msg.marker_ids[0])
        if self.arucoID == 122:
            self.arucoFound = True
        #print(self.arucoID)
    
    def aruco_baselink_callback(self, msg):
            self.aruco_x = msg.position.x
            self.aruco_y = msg.position.y
            self.aruco_z = msg.position.z

            self.aruco_qx = msg.orientation.x
            self.aruco_qy = msg.orientation.y
            self.aruco_qz = msg.orientation.z
            self.aruco_qw = msg.orientation.w
            print("CALLED")
            
        #print(self.aruco_x)

        # self.aruco_q = [self.aruco_qx, self.aruco_qy, self.aruco_qz, self.aruco_qw]
        # self.aruco_roll, self.aruco_pitch, self.aruco_yaw = euler_from_quaternion(self.aruco_q)
        

    def setpointChecker(self):
        #Check odom if x500 has reached the setpoint
        setpointReached = False
        sp2Validiate = self.setpoints[self.current_setpoint_index]
        sp2ValidiateAruco = self.aruco_setpoint



        if self.arucoFlag == True:
            if (sp2ValidiateAruco[0] - 0.05 < self.curr_x < sp2ValidiateAruco[0] + 0.05) and (sp2ValidiateAruco[1] - 0.05 < self.curr_y < sp2ValidiateAruco[1] + 0.05) and (sp2ValidiateAruco[2] + 0.05 > self.curr_z > sp2ValidiateAruco[2] - 0.05):
                if (-0.08 < self.curr_vx < 0.08) and  (-0.08 < self.curr_vy < 0.08) and  (0.08 > self.curr_vz > -0.08):
                    setpointReached = True
        
        else:
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
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND)
        self.get_logger().info('Landing initiated')
        self.disarm()
    

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
        #msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp

        self.offboard_control_mode_pub.publish(msg)
    
    def publish_aruco_pose(self, arucoSetpoint):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x = arucoSetpoint[0]
        msg.y = arucoSetpoint[1]
        msg.z = arucoSetpoint[2]
        #msg.yaw = arucoSetpoint[3]
        vel_x = (arucoSetpoint[0] - self.curr_x) / 2
        vel_y = (arucoSetpoint[1] - self.curr_y) / 2
        vel_z = (arucoSetpoint[2] - self.curr_z) / 2
        msg.vx, msg.vy, msg.vz = vel_x, vel_y, vel_z
        #msg.vx, msg.vy, msg.vz = 0.9, 0.9, 0.9
        self.trajectory_pub.publish(msg)
    

    def trajectory_setpoint_publisher(self, setpoint, index):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        #msg.x, msg.y, msg.z, msg.yaw = setpoint[index]
        msg.x, msg.y, msg.z= setpoint[index]
        vel_x = (setpoint[index][0] - self.curr_x) / 2
        vel_y = (setpoint[index][1] - self.curr_y) / 2
        vel_z = (setpoint[index][2] - self.curr_z) / 2
        msg.vx, msg.vy, msg.vz = vel_x, vel_y, vel_z
        #msg.vx, msg.vy, msg.vz = 0.9, 0.9, 0.9
        self.trajectory_pub.publish(msg)


    #Where the offboard watchdog signal is produced and sent.
    def offboard_callback(self):
        self.publish_offboard_heartbeat()
        if self.arucoFlag == False:
            self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)
        elif self.arucoFlag == True:
            self.publish_aruco_pose(self.aruco_setpoint)

        if self.offboard_counter < 11:
            self.offboard_counter += 1


    #Where I want state changes to occur
    def cmdloop_callback(self):
        #print(self.home_z)
        #self.setHome()
        #self.drone_state(self.current_state)
        #self.exec_time = time.time() - self.start_time

        #IDLE STATE --> ARM STATE
        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED and self.armFlag == False:
            self.arm()
            self.armFlag = True
            self.setpoints.clear()
            self.setpoints.append((self.home_x, self.home_y, self.home_z + (-1.25)))
            self.setpoints.append((self.home_x + (1.25), self.home_y, self.home_z + (-1.25)))
            print(self.nav_state)
            print("ARM")

        #ARM STATE --> TAKEOFF STATE
        elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.offboard_counter ==10:
            self.offboard_activate()
            print(self.nav_state)
            print("OFFBOARD")
        
        #TAKEOFF STATE --> SEARCH STATE (Where it thinks Aruco will be)
        elif self.setpointChecker() == True and self.current_setpoint_index == 0:
            self.current_setpoint_index = 1
            print(self.nav_state)
            print("SEARCH")
        
        #SEARCH STATE --> ARUCO SEARCH AND DETECT
        elif self.setpointChecker() == True and self.current_setpoint_index == 1 and self.average == False:
            #OLD SEARCH ALGORITHIM
            print(self.aruco_y)
            
            arucoSetpoint = [self.aruco_x, self.aruco_y, self.home_z + (-1.25)]
            self.setpoints.append(arucoSetpoint)
                
            
                #self.arucoFlag = True
                # aruco_q = [self.aruco_qx, self.aruco_qy, self.aruco_qz, self.aruco_qw]
                # self.aruco_yaw = euler_from_quaternion(aruco_q)
                
            #UTILIZE RUNNING AVERAGE LATER ON
            while self.arucoCount != 20:
                self.arucoSample_x += self.aruco_x
                self.arucoSample_y += self.aruco_y
                self.arucoCount += 1

                
            self.new_aruco_x = self.arucoSample_x/20.0
            self.new_aruco_y = self.arucoSample_y/20.0
            self.average = True
            
            if self.arucoFound == True and self.arucoID == 122:
                    self.current_setpoint_index = 2

            print(self.arucoFound)
            print(self.nav_state)
            print("EXECUTE")
            

            print(self.aruco_y)
            
            
        elif self.setpointChecker() == True and self.current_setpoint_index == 2:
            self.arucoFlag = True

            # self.arucoSample_x = self.aruco_x
            # self.arucoSample_y = self.aruco_y
            print(self.aruco_y)
            aruco_z = self.curr_z
            aruco_z += 0.05

            self.aruco_setpoint = [self.new_aruco_x, self.new_aruco_y, aruco_z]



            # if self.curr_z < (self.home_z + (-0.3)):
            #     self.aruco_setpoint = [new_aruco_x, new_aruco_y, aruco_z, self.home_yaw]
            #     self.arucoCount = 0
            #     self.arucoSample_x = 0.0
            #     self.arucoSample_y = 0.0
            #     self.final_x = new_aruco_x
            #     self.final_y = new_aruco_y
            # elif self.curr_z >= (self.home_z + (-0.3)):
            #     self.aruco_setpoint = [self.final_x , self.final_y, aruco_z, self.home_yaw]
            # elif self.curr_z >= (self.home_z + (-0.05)):
            #     self.land()
        
                    
            print(self.aruco_setpoint)

            if (self.home_z + 0.05) > self.curr_z > (self.home_z - 0.05) and self.arucoFlag == True:
                self.land()


        


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()