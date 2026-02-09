#!/usr/bin/env python3

import time

# from pathlib import Path
# import dotenv

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client import RpcError
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client import frame_helpers, math_helpers, robot_command
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_command
from bosdyn.api import trajectory_pb2
from bosdyn.util import seconds_to_duration
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import basic_command_pb2
# import tf2_ros
# from tf.transformations import quaternion_from_euler, quaternion_multiply
# from tf2_ros import TransformBroadcaster

from bosdyn.client.payload import PayloadClient
from bosdyn.client.payload_registration import PayloadRegistrationClient

import math
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped

import numpy as np




# map_tf_odom = SE3Pose(0, 1, 0, Quat(w=0.5735764, x=0, y=0, z=-0.819152))
map_tf_body = SE3Pose(0, 1, 0, Quat(w=1.0, x=0, y=0, z=0))


class SpotBodyPublisher:

    def __init__(self):
        rospy.init_node('SpotBodyPublisher', anonymous=False)

        self.hostname = "192.168.1.76"
        self.bd_user = "admin"
        self.bd_pass = "4aud2u39hgfd"

        self.sdk = bosdyn.client.create_standard_sdk("findSpot_melodic")
        self.robot = self.sdk.create_robot(self.hostname)
        self._lease_client = None
        self._command_client = None
        self._state_client = None
        self._lease = None
        self._lease_keepalive = None
        self._payload_registration_client = self.robot.ensure_client(PayloadRegistrationClient.default_service_name)

        # Initialize the transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)

        self.get_creds()
        self.connect()
        self.establish_clients()
        self.get_lease()
        self.nominal_pose = None

        message = self._state_client.get_robot_state()
        snapshot = message.kinematic_state.transforms_snapshot

        odom_tf_body_start = get_a_tform_b(snapshot, "odom", "body")

        self.map_tf_odom = map_tf_body * odom_tf_body_start.inverse()

        #Get and store payload creds
        self.deployed_guid, self.deployed_secret = bosdyn.client.util.read_payload_credentials("../../payload_creds/x500_undocked")
        self.docked_guid, self.docked_secret = bosdyn.client.util.read_payload_credentials("../../payload_creds/x500_docked")

        self.publisher = rospy.Publisher('/spot_pose', PoseStamped, queue_size=10)

        #self.timer = self.create_timer(0.1, self.timer_callback)
        self.rate = rospy.Rate(10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def get_creds(self):
        # env_path = Path(__file__).resolve().parent / ".env"
        # config = dotenv.dotenv_values(str(env_path))
        # self.hostname = config.get("ROBOT_IP")
        # self.bd_user = config.get("BOSDYN_CLIENT_USERNAME")
        # self.bd_pass = config.get("BOSDYN_CLIENT_PASSWORD")
        return self.bd_user, self.bd_pass

    def connect(self):

        bosdyn.client.util.authenticate(self.robot, askpass=self.get_creds)

        rospy.loginfo(f"Spot @ {self.hostname} auth successful")

        # LOGGER.info(f"Spot @ {hostname} auth successful")
        self.robot.time_sync.wait_for_sync()
        self._state_client = self.robot.ensure_client(
            RobotStateClient.default_service_name
        )
        rospy.loginfo(f"State client created... Entering ROS Timer callback")

    def establish_clients(self):
        self._lease_client = self.robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        self._command_client = self.robot.ensure_client(bosdyn.client.robot_command.RobotCommandClient.default_service_name)
    
    def get_lease(self):
        rospy.loginfo(f"THIS LEASE BELONGS TO {self.hostname} NOW!")
        self._lease = self._lease_client.take()
        self._lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client, return_at_exit=True)


    def timer_callback(self, event):
        # Make a robot state request
        #while not rospy.is_shutdown():
        message = self._state_client.get_robot_state()
        snapshot = message.kinematic_state.transforms_snapshot

        odom_tf_body = get_a_tform_b(snapshot, "odom", "body")
        map_tf_body = self.map_tf_odom * odom_tf_body

        # t = TransformStamped()
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "spot_body"

        msg.pose.position.x = map_tf_body.position.x
        msg.pose.position.y = map_tf_body.position.y
        msg.pose.position.z = map_tf_body.position.z

        msg.pose.orientation.x = map_tf_body.rotation.x
        msg.pose.orientation.y = map_tf_body.rotation.y
        msg.pose.orientation.z = map_tf_body.rotation.z
        msg.pose.orientation.w = map_tf_body.rotation.w

    
    def takeoff_qc_prepare(self):

        def check_stance_status(cmd):
            # cmd = basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT
            # cmd = robot_command_pb2.RobotCommandFe
            return cmd.feedback.synchronized_feedback.mobility_command_feedback.stance_feedback.status == basic_command_pb2.StanceCommand.Feedback.STATUS_STANCED
            

        # This example ues the current body position, but you can specify any position.
        # A common use is to specify it relative to something you know, like a fiducial.
        self.nominal_pose = self._state_client.get_robot_state()
        vo_T_body = frame_helpers.get_se2_a_tform_b(self.nominal_pose.kinematic_state.transforms_snapshot,
                                                    frame_helpers.VISION_FRAME_NAME,
                                                    frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)

        #Nominal Spot Height = 0.413614684137 / -4.5770745277404785
        #Max Height = 0.502342052748 / -4.641268730163574
        #Min Height = 0.282637324592 / -4.437628746032715
        
        #### Example stance offsets from body position. ####
        #Seems like 0.2 offsets is the normal offsets
        x_offset = 0.2
        y_offset = 0.2

        pos_fl_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, y_offset, 0)
        pos_fr_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, -y_offset, 0)
        pos_hl_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, y_offset, 0)
        pos_hr_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, -y_offset, 0)

        stance_cmd = RobotCommandBuilder.stance_command(
            frame_helpers.VISION_FRAME_NAME, pos_fl_rt_vision.position, pos_fr_rt_vision.position,
            pos_hl_rt_vision.position, pos_hr_rt_vision.position)



        stance_cmd.synchronized_command.mobility_command.stance_request.end_time.CopyFrom(
            self.robot.time_sync.robot_timestamp_from_local_secs(time.time() + 5))

        # Send the command
        # send = self._command_client.robot_command(stance_cmd)
        # time.sleep(4)
        # test = self._command_client.robot_command_feedback(send)
        # print(test)

        blocking_command(self._command_client, stance_cmd, check_stance_status)

        self.x500_undocking()

        # height_cmd= RobotCommandBuilder.synchro_stand_command(body_height= -0.5)
        ####NOT SURE IF I WANT TO GO LOWER AFTER PREPARING YET###
        # robot_state = self._state_client.get_robot_state()

        # odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
        #                                  frame_helpers.ODOM_FRAME_NAME, frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)

        # # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # # Define times (in seconds) for each point in the trajectory.
        # t1 = 0.5


        # # Specify the poses as transformations to the cached flat_body pose.
        # flat_body_T_pose = math_helpers.SE3Pose(x=0, y=0, z=-0.5, rot=math_helpers.Quat())

        # # Build the points in the trajectory.
        # traj_point = trajectory_pb2.SE3TrajectoryPoint(
        #     pose=(odom_T_flat_body * flat_body_T_pose).to_proto(),
        #     time_since_reference=seconds_to_duration(t1))


        # # Build the trajectory proto by combining the points.
        # traj = trajectory_pb2.SE3Trajectory(points=[traj_point])

        # # Build a custom mobility params to specify absolute body control.
        # body_control = spot_command_pb2.BodyControlParams(
        #     body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=frame_helpers.ODOM_FRAME_NAME,
        #                                                           base_offset_rt_root=traj))


        # blocking_stand(self._command_client, timeout_sec=10,
        #                params=spot_command_pb2.MobilityParams(body_control=body_control))
    
    def get_state(self):
        state = self._state_client.get_robot_state()
        print(state)
    
    def stand(self):
        #Might want to look at the blocking robot_commands in documentation
        # height_cmd_1 = RobotCommandBuilder.synchro_stand_command(body_height= -0.5)

        # height_cmd_2 = RobotCommandBuilder.synchro_stand_command(body_height= 0.5)

        # height_cmd_3 = RobotCommandBuilder.synchro_stand_command(body_height=-0.6)

        # height_cmd_4 = RobotCommandBuilder.synchro_stand_command(body_height=0.0)

        # height_cmd.synchronized_command.mobility_command.stand_request.end_time.CopyFrom(
        #     self.robot.time_sync.robot_timestamp_from_local_secs(time.time() + 5))
        
        # popping_up_down = RobotCommandBuilder.build_synchro_command(height_cmd_1, height_cmd_2, height_cmd_3, height_cmd_4)
        # print(popping_up_down)
        # self._command_client.robot_command(popping_up_down)

        # height_cmd = RobotCommandBuilder.synchro_se2_trajectory_command()

        # nom_height = RobotCommandBuilder.synchro_stand_command(body_height= 0.0)
        # self._command_client.robot_command(nom_height)

        blocking_stand(self._command_client, timeout_sec=10)

        robot_state = self._state_client.get_robot_state()

        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         frame_helpers.ODOM_FRAME_NAME, frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)

        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        t1 = 0.375
        t2 = 0.75
        t3 = 1.125
        t4 = 1.5

        # Specify the poses as transformations to the cached flat_body pose.
        flat_body_T_pose1 = math_helpers.SE3Pose(x=0, y=0, z=-0.5, rot=math_helpers.Quat())
        flat_body_T_pose2 = math_helpers.SE3Pose(x=0.0, y=0, z=0.5, rot=math_helpers.Quat())
        flat_body_T_pose3 = math_helpers.SE3Pose(x=0.0, y=0, z=-0.5, rot=math_helpers.Quat())
        flat_body_T_pose4 = math_helpers.SE3Pose(x=0.0, y=0, z=0.0, rot=math_helpers.Quat())

        # Build the points in the trajectory.
        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose1).to_proto(),
            time_since_reference=seconds_to_duration(t1))
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose2).to_proto(),
            time_since_reference=seconds_to_duration(t2))
        traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose3).to_proto(),
            time_since_reference=seconds_to_duration(t3))
        traj_point4 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose4).to_proto(),
            time_since_reference=seconds_to_duration(t4))

        # Build the trajectory proto by combining the points.
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2, traj_point3, traj_point4])

        # Build a custom mobility params to specify absolute body control.
        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=frame_helpers.ODOM_FRAME_NAME,
                                                                  base_offset_rt_root=traj))


        blocking_stand(self._command_client, timeout_sec=10,
                       params=spot_command_pb2.MobilityParams(body_control=body_control))

    def release_lease(self):
        self._lease_client.return_lease(self._lease)

    def reset_pose(self):
        stand_cmd = RobotCommandBuilder.synchro_stand_command(params=params)  
        blocking_stand(command_client, cmd=stand_cmd, timeout_sec=10)

        # def check_stance_status(cmd):
        #     return cmd.feedback.synchronized_feedback.mobility_command_feedback.stance_feedback.status == basic_command_pb2.StanceCommand.Feedback.STATUS_STANCED

        # self.nominal_pose = self._state_client.get_robot_state()
        # vo_T_body = frame_helpers.get_se2_a_tform_b(self.nominal_pose.kinematic_state.transforms_snapshot,
        #                                             frame_helpers.VISION_FRAME_NAME,
        #                                             frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)
        # #Nominal Spot Height = 0.413614684137 / -4.5770745277404785
        # #Max Height = 0.502342052748 / -4.641268730163574
        # #Min Height = 0.282637324592 / -4.437628746032715
        
        # #### Example stance offsets from body position. ####
        # x_offset = 0.3
        # y_offset = 0.3

        # pos_fl_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, -y_offset, 0)
        # pos_fr_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, y_offset, 0)
        # pos_hl_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, -y_offset, 0)
        # pos_hr_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, y_offset, 0)

        # stance_cmd = RobotCommandBuilder.stance_command(
        #     frame_helpers.VISION_FRAME_NAME, pos_fl_rt_vision.position, pos_fr_rt_vision.position,
        #     pos_hl_rt_vision.position, pos_hr_rt_vision.position)



        # stance_cmd.synchronized_command.mobility_command.stance_request.end_time.CopyFrom(
        #     self.robot.time_sync.robot_timestamp_from_local_secs(time.time() + 5))

        # # Send the command
        # # send = self._command_client.robot_command(stance_cmd)
        # # time.sleep(4)
        # # test = self._command_client.robot_command_feedback(send)
        # # print(test)

        # blocking_command(self._command_client, stance_cmd, check_stance_status)

    def landing_qc_prepare(self):
        x500_docking()

    def x500_undocking(self):
        #Unregister x500_docked
        self._payload_registration_client.detach_payload(self.docked_guid, self.docked_secret)

        #Register x500_undocked
        self._payload_registration_client.attach_payload(self.deployed_guid, self.deployed_secret)

        rospy.loginfo(f"QUADCOPTER IS UNDOCKING... DETTACHING {self.docked_guid} AND ATTACHING {self.deployed_guid}")

    def x500_docking(self):
        #Register x500_docked
        self._payload_registration_client.attach_payload(self.docked_guid, self.docked_secret)

        #Unregister x500_undocked
        self._payload_registration_client.detach_payload(self.deployed_guid, self.deployed_secret)

        rospy.loginfo(f"QUADCOPTER IS DOCKED... DETTACHING {self.deployed_guid} AND ATTACHING {self.docked_guid}")
        rospy.loginfo(f"THIS DOES FN DOES NOT SEND A MAGNETIZATION COMMAND")
    
    def stand_test(self):
        blocking_stand(self._command_client, timeout_sec=10)


if __name__ == '__main__':
    node = SpotBodyPublisher()
    # node.stand()
    #node.takeoff_qc_prepare
    #node.reset_pose()
    #node.stand()
    # node.release_lease()
