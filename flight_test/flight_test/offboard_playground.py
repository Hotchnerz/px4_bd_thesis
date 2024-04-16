import rclpy
import numpy as np
import time
import tf2_ros
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from tf_transformations import euler_from_quaternion
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, Timesync, VehicleOdometry
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
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, 'fmu/offboard_control_mode/in', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', qos_profile)


        #Subscribers
        self.drone_status_sub = self.create_subscription(VehicleStatus, '/fmu/vehicle_status/out', self.vehicle_status_callback, qos_profile)
        self.timesync_subscriber = self.create_subscription(Timesync, '/fmu/timesync/out', self.timesync_callback, qos_profile)
        self.odom_subscriber = self.create_subscription(VehicleOdometry, '/fmu/vehicle_odometry/out', self.odom_callback, qos_profile)
        self.aruco_subscriber = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        self.aruco_baselink_subscriber = self.create_subscription(Pose, '/aruco_baselink', self.aruco_baselink_callback, qos_profile)


        self.curr_x, self.curr_y, self.curr_z, self.curr_yaw = 0.0, 0.0, 0.0, 0.0
        # self.home_x, self.home_y, self.home_z, self.home_yaw = 0.0, 0.0, 0.0, 0.0
        self.aruco_x, self.aruco_y, self.aruco_z = 0.0, 0.0, 0.0
        # self.aruco_qx, self.aruco_qy, self.aruco_qz, self.aruco_qw = 0.0, 0.0, 0.0, 0.0
        # self.aruco_roll, self.aruco_pitch, self.aruco_yaw = 0.0, 0.0, 0.0
        # self.Kp_x = 1.0
        # self.Ki_x = 0.1
        # self.Kd_x = 0.1
        # self.error_x, self.error_y = 0.0, 0.0
        # self.time_prev_x = 0
        # self.integral_x, self.integral_y = 0.0, 0.0
        # self.Kp_y, self.Ki_y, self. Kd_z = 0.0, 0.0, 0.0
        # self.aruco_q =[]
        # self.curr_q = []
        # self.new_x = 0.0
        self.timestamp = 0
        self.offboard_counter = 0
        self.current_setpoint_index = 0
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        # self.arucoID = 0
        # self.posCounter = 0
        self.takeoff = False
        # self.arucoFound = False
        # self.FirstStage = False
        # self.SecondStage = False
        self.start_time = time.time()
        self.exec_time = 0
        # self.desired_z = -1.25
        # self.first_x, self.first_y, self.new_x, self.new_y = 0.0, 0.0, 0.0, 0.0

        self.setpoints = [
            (0.0, 0.0, -1.25, 0.0), 
            (1.5, 0.0, -1.25, 0.0),
            (1.5, 1.5, -1.25, 0.0)
        ]

        # self.setpoints = [
        #     (0.0, 0.0, -1.25, 0.0), 
        #     (1.5, 0.0, -1.25, 0.0), 
        #     (self.aruco_x, self.aruco_y, -1.25, 0.0)
        # ]
        
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
 
    def vehicle_status_callback(self, msg):
        #TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    #PX4 quaternions are in Hamilton Convention (w,x,y,z)
    def odom_callback(self, msg):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_z = msg.z

        #Convert Hamilton Conv. to JPL Conv.
        px4_q = [float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0])]
        euler = euler_from_quaternion(px4_q)
        self.curr_yaw = euler[2]
        #print(self.curr_z)

    def setHome(self):
        if self.homeSet == False:
            self.home_x = self.curr_x
            self.home_y = self.curr_y
            self.home_z = self.curr_z
            self.home_yaw = self.curr_yaw
            self.homeSet = True
    
    def aruco_callback(self, msg):
        self.arucoID = msg.marker_ids[0]
        # self.aruco_x = msg.poses[0].position.x
        # self.aruco_y = msg.poses[0].position.y
        # self.aruco_z = msg.poses[0].position.z

        # self.aruco_qx = msg.poses[0].orientation.x
        # self.aruco_qy = msg.poses[0].orientation.y
        # self.aruco_qz = msg.poses[0].orientation.z
        # self.aruco_qw = msg.poses[0].orientation.w
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
        # if (sp2Validiate[0] - 0.05 < self.curr_x < sp2Validiate[0] + 0.05) and (sp2Validiate[1] - 0.05 < self.curr_y < sp2Validiate[1] + 0.05) and (sp2Validiate[2] + 0.05 > self.curr_z > sp2Validiate[2] - 0.05):
        #     setpointReached = True
        
        if (sp2Validiate[0] - 0.05 < self.curr_x < sp2Validiate[0] + 0.05) and (sp2Validiate[1] - 0.05 < self.curr_y < sp2Validiate[1] + 0.05) and (sp2Validiate[2] + 0.05 > self.curr_z > sp2Validiate[2] - 0.05):
            setpointReached = True

        #Stil need one for orientation

        return setpointReached
        
    def checkAruco(self):
        arucoFound = False

        if self.arucoID == 122:
            arucoFound = True
            #arucoSetpoint = (self.aruco_x, self.aruco_y, self.aruco_z*-1)

        return arucoFound

    
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def setHome(self):
        self.home_x = self.curr_x
        self.home_y = self.curr_y
        self.home_z = self.curr_z
        self.home_q = euler_from_quaternion(self.cur)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')
        self.get_logger().info('Shutting down ROS node...')
        self.destroy_node()
        rclpy.shutdown()
    

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
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
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp

        self.offboard_control_mode_pub.publish(msg)
    

    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp


    def trajectory_setpoint_publisher(self, setpoint, index):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x, msg.y, msg.z, msg.yaw = setpoint[index]
        self.trajectory_pub.publish(msg)



    def cmdloop_callback(self):
        # Publish offboard control modes
        self.publish_offboard_heartbeat()
        self.exec_time = time.time() - self.start_time

        if self.offboard_counter == 10 and self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.arm()
            self.offboard_activate()
        elif self.offboard_counter < 11:
            self.offboard_counter += 1

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            #Go to Takeoff
            self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)

            
            if self.setpointChecker() == True and self.current_setpoint_index == 0:
                # self.get_logger().info('Sending Next Setpoint')
                self.current_setpoint_index = 1

                # self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)

            #Find Aruco
            if self.setpointChecker() == True and self.current_setpoint_index == 1:
                # self.get_logger().info('Prepare to Land')
                # self.land()
                # self.get_logger().info('Find Aruco...')
                #Call aruco detect function
                arucoFlag = self.checkAruco()
                print(self.checkAruco())
                if arucoFlag == True:
                    #arucoSetpoint = (self.aruco_x, self.aruco_y, self.aruco_z*-1, 0.0)
                    # self.get_logger().info('Found Aruco')
                    self.current_setpoint_index = 2
                    # self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)

                    if self.setpointChecker() == True:
                        self.land()

                else:
                    print("Lost Aruco")
                    # self.get_logger().info('Lost Aruco')

        if self.exec_time >= 30 and self.checkAruco() == False:
            self.current_setpoint_index = 0
            # self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)
            if self.setpointChecker() == True:
                self.land()


                #If Aruco is found, sit on top of it and wait.
                #Then take 10 samples, descend until you land.


            #What happens if you lose the aruco?
                




            # #Go to Takeoff
            # if self.takeoff == False:
            #     self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)
            
            # if  (-1.3 < self.curr_z < -1.2) and (self.arucoFound == False):
            #     self.takeoff == True
            #     self.get_logger().info('Marker Found')
            #     #Go search for Aruco
            #     self.current_setpoint_index = 1
            #     self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)
                
            # #If aruco found, go sit on it
            # if self.arucoID == 122 and self.exec_time > 25:
                
            #     self.arucoFound = True
            #     # arucoSetpoints = [
            #     #     (self.aruco_x - 0.2, -self.aruco_y, self.desired_z, 0.0)
            #     # ]
            #     # self.trajectory_setpoint_publisher(arucoSetpoints, 0)
            #     while self.posCounter != 10:
            #         self.first_x = self.first_x + self.aruco_x
            #         self.first_y = self.first_y+ self.aruco_y
            #         self.posCounter += 1

            #     self.new_x = self.first_x /10.0
            #     self.new_y = self.first_y /10.0

            #     arucoSetpoints = [
            #         (self.new_x, -self.new_y, self.desired_z, 0.0)
            #     ]

            #     #DRONE NEEDS TO BE STOPPED BEFORE SENDING
            #     self.trajectory_setpoint_publisher(arucoSetpoints, 0)
            #     if self.exec_time > 28:
            #         if self.curr_z <= -0.8:
            #             self.desired_z += 0.002
            #     self.FirstStage = True
            #     self.posCounter = 0
            #     self.first_x, self.first_y = 0.0, 0.0

            #     if self.FirstStage == True and self.curr_z >= -0.8:
            #         while self.posCounter != 10:
            #             self.first_x = self.first_x + self.aruco_x
            #             self.first_y = self.first_y+ self.aruco_y
            #             self.posCounter += 1

            #         self.new_x = self.first_x /10.0
            #         self.new_y = self.first_y /10.0

            #         arucoSetpoints = [
            #             (self.new_x, -self.new_y, self.desired_z, 0.0)
            #         ]
            #         self.SecondStage = True
            #         self.trajectory_setpoint_publisher(arucoSetpoints, 0)

            #     #if self.SecondStage == True and self.exec_time > 30:
            #         self.desired_z += 0.002
            #         if self.curr_z >= -0.10:
            #             self.land()

            # if self.arucoID != 122 and self.exec_time > 35:
            #     self.current_setpoint_index = 0
            #     self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)

            #     if -1.3 < self.curr_z < -1.2 and -0.1 < self.curr_x < 0.2 :
            #         self.get_logger().info('Could not find Marker.')
            #         self.land()

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()