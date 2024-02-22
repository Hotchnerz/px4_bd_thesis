#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import CommanderState
from px4_msgs.msg import VehicleCommand, VehicleOdometry


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/vehicle_status/out',
            self.vehicle_status_callback,
            qos_profile)
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/vehicle_odometry/out',
            self.odom_callback,
            qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/offboard_control_mode/in', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.curr_x, self.curr_y, self.curr_z = 0, 0, 0
        # self.theta = 0.0
        # self.radius = 10.0
        # self.omega = 0.5
        self.setpoints = [
        (0, 0, -1.5, -3.14), 
        (1.0, 0.0, -1.5, -3.14), #where you assume the aruco to be, eventually from spot odom data
        (1.0, 0.0, -0.5, -3.14)
    ]
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        


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
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x, msg.y, msg.z, msg.yaw = self.setpoints[self.current_setpoint_index]
        self.trajectory_setpoint_publisher.publish(msg)



    def cmdloop_callback(self):
        # # Publish offboard control modes
        # offboard_msg = OffboardControlMode()
        # offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # offboard_msg.position=True
        # offboard_msg.velocity=False
        # offboard_msg.acceleration=False
        # self.publisher_offboard_mode.publish(offboard_msg)
        # if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            


        #     trajectory_msg = TrajectorySetpoint()
        #     trajectory_msg.x= 0.0
        #     trajectory_msg.y = 0.0
        #     trajectory_msg.z = -5.0
        #     self.publisher_trajectory.publish(trajectory_msg)




        #if (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.offboard_setpoint_counter < 16):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        self.offboard_setpoint_counter += 1
            
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.offboard_setpoint_counter >= 15):
            
            self.arm()



        else:
            print("WAITING FOR OFFBOARD SIGNAL")


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()