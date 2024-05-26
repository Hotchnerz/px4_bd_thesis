import rclpy
import numpy as np
from transitions import Machine
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.clock import Clock
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, Timesync, VehicleOdometry, VehicleLocalPosition, VehicleLocalPositionSetpoint, LandingTargetPose, VehicleAttitude
from ros2_aruco_interfaces.msg import ArucoMarkers

class DroneState():
    def __init__(self):
        self.final_setpoint = [0,0,0]
        self.flight_height = -1.20
        self.reset_moving_avg = False
        self.x_setpoints = []
        self.y_setpoints = []
        self.x_app_setpoint_app = []
        self.y_app_setpoint_app = []

    
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

    def on_enter_FAILSAFE(self, *args):
        print('FAILSAFE ENTERED, RESTART PROGRAM...')
    
    # def on_enter_TAKEOFF(self, *args):
    #     self.update_setpoint([0,0,-1.25,0])
    #     #self.get_logger().info("Sending Takeoff Setpoint")
    #     print("Sending Takeoff Setpoint")

    def on_exit_IDLE(self, *args):
        print(OffboardControl.home_pos)

    def on_exit_LOITER(self, *args):
        #self.update_setpoint([1.5,0,self.flight_height,0])
        self.update_setpoint([1.35,0,self.flight_height,0])
        print("Sending Search Setpoint")
    
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

    def marker_found(self):
        print("Marker Found:" + str(OffboardControl.aruco_found))
        return OffboardControl.aruco_found and OffboardControl.first_aruco_msg

    def distance_check(self):
        print(np.sqrt(np.square(OffboardControl.curr_pos[0] - OffboardControl.marker_pos[0]) + np.square(OffboardControl.curr_pos[1] - OffboardControl.marker_pos[1])))
        return (np.sqrt(np.square(OffboardControl.curr_pos[0] - OffboardControl.marker_pos[0]) + np.square(OffboardControl.curr_pos[1] - OffboardControl.marker_pos[1]))) > 0.08
        #return (np.sqrt(np.square(OffboardControl.curr_pos[0] - 1.25944) + np.square(OffboardControl.curr_pos[1] - 0.0202361))) > 0.05

    def setpoint_check(self):
        #Check odom if x500 has reached the setpoint
        setpointReached = False
        sp2Validiate = OffboardControl.setpoints

        if (sp2Validiate[0] - 0.05 < OffboardControl.curr_pos[0] < sp2Validiate[0] + 0.05) and (sp2Validiate[1] - 0.05 < OffboardControl.curr_pos[1]< sp2Validiate[1] + 0.05) and (sp2Validiate[2] + 0.05 > OffboardControl.curr_pos[2] > sp2Validiate[2] - 0.05):
            if (-0.04 < OffboardControl.curr_vel[0] < 0.04) and  (-0.04 < OffboardControl.curr_vel[1] < 0.04) and  (0.04 > OffboardControl.curr_vel[2] > -0.04):
                setpointReached = True

        return setpointReached

    def attitude_check(self):
        #Check if x500 is level
        drone_level = False

        if (-0.05 < OffboardControl.rpy[0] < 0.05) and (-0.05 < OffboardControl.rpy[1] < 0.05):
            drone_level = True

        return drone_level
    
    def moving_avg(self):

        #Simple Moving Average
        window = 10
        
        if self.reset_moving_avg == False:
            self.x_setpoints = np.array(OffboardControl.marker_pos_x)
            self.y_setpoints = np.array(OffboardControl.marker_pos_y)
            self.reset_moving_avg = True

        weight = np.ones(window) / window

        moving_avg_x = np.convolve(self.x_setpoints, weight, mode='valid')
        moving_avg_y = np.convolve(self.y_setpoints, weight, mode='valid')

        return moving_avg_x, moving_avg_y


            
    
    def scan_check(self):

        if (len(OffboardControl.marker_pos_x) == 20) and (len(OffboardControl.marker_pos_y) == 20):
            self.x_app_setpoint_app, self.y_app_setpoint_app = self.moving_avg()
            return True
        
        return False



    
    def set_takeoff_setpoint(self):
        self.update_setpoint([0,0,self.flight_height,0])
        #self.get_logger().info("Sending Takeoff Setpoint")
        print("Sending Takeoff Setpoint")
    
    def set_approach_setpoint(self):
        self.update_setpoint([np.median(self.x_app_setpoint_app),np.median(self.y_app_setpoint_app),OffboardControl.curr_pos[2],OffboardControl.marker_pos[3]])
        #self.update_setpoint([1.25944,0.0202361,-1.25,OffboardControl.marker_pos[3]])
        print("Approaching Marker")
    
    def set_final_setpoint(self):
        self.final_setpoint[0] = OffboardControl.curr_pos[0]
        self.final_setpoint[1] = OffboardControl.curr_pos[1]
        self.final_setpoint[2] = OffboardControl.curr_pos[3]
    
    def landing_check(self):
        
        drone_land = False
        new_z = OffboardControl.curr_pos[2] + 0.08
        self.update_setpoint([self.final_setpoint[0], self.final_setpoint[1], new_z, self.final_setpoint[2]])
        #self.update_setpoint([1.25944,0.0202361, new_z, OffboardControl.marker_pos[3]])
            #return drone_land
        if (0.02 > OffboardControl.curr_pos[2] > -0.02):
            drone_land = True
            
        return drone_land
            


class OffboardControl(Node):

    # Set as x, y ,z , yaw (NED)
    setpoints = [0.0, 0.0, 0.0, 0.0]
    home_pos = [0.0, 0.0, 0.0, 0.0]
    curr_pos = [0.0, 0.0, 0.0, 0.0]
    curr_vel = [0.0, 0.0, 0.0, 0.0]
    curr_accel = [0.0, 0.0, 0.0, 0.0]
    marker_pos = [0.0, 0.0, 0.0, 0.0]
    marker_pos_x = []
    marker_pos_y = []

    rpy = (0,0,0)
    aruco_found = False
    first_aruco_msg = False

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
        self.localpos_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/vehicle_local_position/out', self.localpos_callback, qos_profile)
        self.vehicle_att_subscriber = self.create_subscription(VehicleAttitude, '/fmu/vehicle_attitude/out', self.vehicle_att_callback, qos_profile)
        self.aruco_subscriber = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        self.aruco_baselink_subscriber = self.create_subscription(Pose, '/aruco_baselink', self.aruco_baselink_callback, qos_profile)


        self.homeSetPos = False
        self.homeSetYaw = False
        self.timestamp = 0
        self.offboard_counter = 0
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_MAX
        self.failsafe_state = False
        


        self.states=['IDLE', 'FAILSAFE', 'ARM', 'DISARM', 'TAKEOFF', 'LOITER', 'SEARCH', 'SCAN','APPROACH', 'LAND']
        
        self.droneState = DroneState()
        self.machine = Machine(model=self.droneState , states=self.states, initial= 'IDLE')
        

        self.machine.add_transition('trs_next', 'IDLE', 'ARM', conditions = lambda: self.arm_state == VehicleStatus.ARMING_STATE_ARMED)
        self.machine.add_transition('trs_next', 'ARM', 'IDLE', conditions = lambda: self.arm_state != VehicleStatus.ARMING_STATE_ARMED)
        self.machine.add_transition('trs_next', 'ARM', 'TAKEOFF', prepare=['set_takeoff_setpoint'], conditions = lambda: self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.offboard_counter > 10)

        #Need to ensure that if this state transition occurs, some sort of clean up like go back into manual mode or pos mode and restart back to Arm or Idle depending on conds.
        self.machine.add_transition('trs_next', 'TAKEOFF', 'FAILSAFE', conditions = lambda: self.failsafe_state == True)
        self.machine.add_transition('trs_next', 'FAILSAFE', 'FAILSAFE')
        self.machine.add_transition('trs_next', 'TAKEOFF', 'LOITER', conditions=['setpoint_check'])
        self.machine.add_transition('trs_next', 'LOITER', 'SEARCH', conditions=['setpoint_check'])
        self.machine.add_transition('trs_next', 'SEARCH', 'SCAN', conditions=['setpoint_check', 'attitude_check'])

        #Perform moving avg?
        self.machine.add_transition('trs_next', 'SCAN', 'APPROACH', before=['set_approach_setpoint'],conditions=['scan_check', 'distance_check', 'marker_found'])
        #Perform another scan?
        self.machine.add_transition('trs_next', 'APPROACH', 'SCAN', conditions=['setpoint_check', 'attitude_check'])

        self.machine.add_transition('trs_next', 'SCAN', 'LAND', prepare=['set_final_setpoint'], conditions=['scan_check','setpoint_check', 'attitude_check'], unless=['distance_check'])
        self.machine.add_transition('trs_next', 'LAND', 'DISARM', conditions=['landing_check'])
        self.machine.add_transition('trs_next', 'DISARM', 'IDLE', conditions = lambda: self.arm_state == VehicleStatus.ARMING_STATE_STANDBY)



        #self.machine.add_transition('trs_next', 'LAND', 'IDLE', conditions=['test'])

        timer_state = 0.5  # seconds
        self.timer_offboard = self.create_timer(timer_state, self.state_callback)

        timer_period = 0.08  # seconds
        self.timerA = self.create_timer(timer_period, self.cmdloop_callback)


    def vehicle_status_callback(self, msg):
        self.arm_state = msg.arming_state
        self.nav_state = msg.nav_state
        self.failsafe_state = msg.failsafe

    
    def localpos_callback(self, msg):
        self.curr_pos[0] = msg.x
        self.curr_pos[1] = msg.y
        self.curr_pos[2] = msg.z

        self.curr_vel[0] = msg.vx
        self.curr_vel[1] = msg.vy
        self.curr_vel[2] = msg.vz

        self.curr_accel[0] = msg.ax
        self.curr_accel[1] = msg.ay
        self.curr_accel[2] = msg.az
        
        self.timestamp = msg.timestamp

        if self.homeSetPos == False:
            self.home_pos[0] = self.curr_pos[0]
            self.home_pos[1] = self.curr_pos[1]
            self.home_pos[2] = self.curr_pos[2]

            self.homeSetPos = True
    

    def vehicle_att_callback(self, msg):
        q = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]

        OffboardControl.rpy = euler_from_quaternion(q)

        self.curr_yaw = OffboardControl.rpy[2]
        if self.homeSetYaw == False:
            self.home_pos[3] = self.curr_yaw
            self.homeSetYaw = True

    #NEED TO WORK ON THIS
    def aruco_callback(self, msg):
        self.arucoID = int(msg.marker_ids[0])
        if self.arucoID == 122:
            OffboardControl.aruco_found = True
        else:
            OffboardControl.aruco_found = False
        #print(self.arucoID)
    

    def aruco_baselink_callback(self, msg):
        if not OffboardControl.first_aruco_msg:
            OffboardControl.first_aruco_msg = True

        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        aruco_rpy = euler_from_quaternion(q)
        self.marker_pos[3] = aruco_rpy[2]


        #Need a check to ensure that this a new Scan state entrance
        if self.droneState.state == 'SCAN':

            #Want to remove this later...
            self.marker_pos[0] = msg.position.x
            self.marker_pos[1] = msg.position.y
            self.marker_pos[2] = msg.position.z


            if (len(OffboardControl.marker_pos_x) < 20) and (len(OffboardControl.marker_pos_y) < 20):
                OffboardControl.marker_pos_x.append(msg.position.x)
                OffboardControl.marker_pos_y.append(msg.position.y)
            elif (len(OffboardControl.marker_pos_x) == 20) and (len(OffboardControl.marker_pos_y) == 20):
                OffboardControl.marker_pos_x.pop(0)
                OffboardControl.marker_pos_y.pop(0)

                OffboardControl.marker_pos_x.append(msg.position.x)
                OffboardControl.marker_pos_y.append(msg.position.y)


    # def arm(self):
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    #     self.get_logger().info('Arm command sent')


    def disarm(self):
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        print("DISARM COMMAND CALLED")
        # self.get_logger().info('Disarm command sent')
        # self.get_logger().info('Shutting down ROS node...')
        # self.destroy_node()
        # rclpy.shutdown()
    

    # #This function needs to be customized. I can't use this because I loose control of drone position if I used PX4 Landing System
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND)
        self.get_logger().info('Landing initiated')
        self.disarm()
    

    # def offboard_activate(self):
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2= 6.0)
    #     self.get_logger().info("Switch to Offboard")


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
    
    

    def trajectory_setpoint_publisher(self, setpoint):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        #msg.x, msg.y, msg.z, msg.yaw = setpoint[index]
        msg.x, msg.y, msg.z= setpoint[0], setpoint[1], setpoint[2]
        # vel_x = (setpoint[0] - self.curr_pos[0]) / 2
        # vel_y = (setpoint[1] - self.curr_pos[1]) / 2
        # vel_z = (setpoint[2] - self.curr_pos[2]) / 2
        # vel_x = 0.6
        # vel_y = 0.6
        # vel_z = 0.6
        # msg.vx, msg.vy, msg.vz = vel_x, vel_y, vel_z
        self.trajectory_pub.publish(msg)
    


    def state_callback(self):
        print(self.droneState.state)
        self.droneState.trs_next()
        print(OffboardControl.marker_pos_x)
        print(OffboardControl.marker_pos_y)
        if self.droneState.state == 'DISARM':
            self.land()
        

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