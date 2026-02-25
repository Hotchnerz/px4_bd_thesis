#!/usr/bin/env python3
import time

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
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         get_se2_a_tform_b)

from bosdyn.client.payload import PayloadClient
from bosdyn.client.payload_registration import PayloadRegistrationClient

from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2
from bosdyn.client.graph_nav import GraphNavClient
import graph_nav_utils

import math
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PoseArray

from flight_test.srv import mission, missionRequest

import numpy as np


class SpotBodyPublisher:

    def __init__(self):
        rospy.init_node('SpotCommander', anonymous=False)

        self.hostname = "192.168.1.76"
        self.bd_user = "admin"
        self.bd_pass = "4aud2u39hgfd"

        self.sdk = bosdyn.client.create_standard_sdk("spotCmdMission_melodic")
        self.robot = self.sdk.create_robot(self.hostname)
        self._lease_client = None
        self._command_client = None
        self._state_client = None
        self._lease = None
        self._lease_keepalive = None
        self._payload_registration_client = None
        self._graph_nav_client = None

        #self._upload_filepath = "../../autowalks/x500_mock_inspectiom.walk"
        self._upload_filepath = "/home/radam/distrobox/bd_home/thesis_ws/src/doggo_walker/autowalks/simple_x500_inspection.walk"

        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()
        self._ordered_ids = dict()


        self.get_creds()
        self.connect()
        self.establish_clients()
        self.get_lease()
        self.nominal_pose = None

        #Get and store payload creds
        self.deployed_guid, self.deployed_secret = bosdyn.client.util.read_payload_credentials("/home/radam/distrobox/bd_home/thesis_ws/src/doggo_walker/payload_creds/x500_undocked")
        self.docked_guid, self.docked_secret = bosdyn.client.util.read_payload_credentials("/home/radam/distrobox/bd_home/thesis_ws/src/doggo_walker/payload_creds/x500_docked")

        #Ensure ROS Clients
        #rospy.wait_for_service('/mission_service')
        self.qc_service = rospy.ServiceProxy('/mission_service', mission)


    def get_creds(self):
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
        self._payload_registration_client = self.robot.ensure_client(PayloadRegistrationClient.default_service_name)
        self._graph_nav_client = self.robot.ensure_client(GraphNavClient.default_service_name)

    def get_lease(self):
        rospy.loginfo(f"THIS LEASE BELONGS TO {self.hostname} NOW!")
        self._lease = self._lease_client.take()
        self._lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client, return_at_exit=True)

    
    def prepare_spot(self):

        def check_stance_status(cmd):
            return cmd.feedback.synchronized_feedback.mobility_command_feedback.stance_feedback.status == basic_command_pb2.StanceCommand.Feedback.STATUS_STANCED
            

        # This example ues the current body position, but you can specify any position.
        self.nominal_pose = self._state_client.get_robot_state()
        vo_T_body = frame_helpers.get_se2_a_tform_b(self.nominal_pose.kinematic_state.transforms_snapshot,
                                                    frame_helpers.VISION_FRAME_NAME,
                                                    frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)

        #Nominal Spot Height = 0.413614684137 / -4.5770745277404785
        #Max Height = 0.502342052748 / -4.641268730163574
        #Min Height = 0.282637324592 / -4.437628746032715
        
        #Seems like 0.2 offsets is the normal offsets
        x_offset = 0.3
        y_offset = 0.3

        pos_fl_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, y_offset, 0)
        pos_fr_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, -y_offset, 0)
        pos_hl_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, y_offset, 0)
        pos_hr_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, -y_offset, 0)

        stance_cmd = RobotCommandBuilder.stance_command(
            frame_helpers.VISION_FRAME_NAME, pos_fl_rt_vision.position, pos_fr_rt_vision.position,
            pos_hl_rt_vision.position, pos_hr_rt_vision.position)

        stance_cmd.synchronized_command.mobility_command.stance_request.end_time.CopyFrom(
            self.robot.time_sync.robot_timestamp_from_local_secs(time.time() + 5))

        blocking_command(self._command_client, stance_cmd, check_stance_status)

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
        blocking_stand(self._command_client, timeout_sec=10)

    def zupvt_init(self):
        robot_state = self._state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         frame_helpers.ODOM_FRAME_NAME, frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)

        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        # t1 = 0.375
        # t2 = 0.75
        # t3 = 1.125
        # t4 = 1.5
    
        t1 = 0.5
        t2 = 1.0
        t3 = 1.5
        t4 = 2.0

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

    def revert_pose(self):
        #Revert Stance
        og_feet = self.nominal_pose
        vo_T_body = frame_helpers.get_se2_a_tform_b(self.nominal_pose.kinematic_state.transforms_snapshot,
                                                    frame_helpers.VISION_FRAME_NAME,
                                                    frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)

        def check_stance_status(cmd):
            return cmd.feedback.synchronized_feedback.mobility_command_feedback.stance_feedback.status == basic_command_pb2.StanceCommand.Feedback.STATUS_STANCED

        # Need to transform from body frame to vision frame
        pos_fl_rt_vision = vo_T_body * math_helpers.SE2Pose(og_feet.foot_state[0].foot_position_rt_body.x, og_feet.foot_state[0].foot_position_rt_body.y, 0)
        pos_fr_rt_vision = vo_T_body * math_helpers.SE2Pose(og_feet.foot_state[1].foot_position_rt_body.x, og_feet.foot_state[1].foot_position_rt_body.y, 0)
        pos_hl_rt_vision = vo_T_body * math_helpers.SE2Pose(og_feet.foot_state[2].foot_position_rt_body.x, og_feet.foot_state[2].foot_position_rt_body.y, 0)
        pos_hr_rt_vision = vo_T_body * math_helpers.SE2Pose(og_feet.foot_state[3].foot_position_rt_body.x, og_feet.foot_state[3].foot_position_rt_body.y, 0)

        test_cmd = RobotCommandBuilder.stance_command(
            frame_helpers.VISION_FRAME_NAME, pos_fl_rt_vision.position, pos_fr_rt_vision.position,
            pos_hl_rt_vision.position, pos_hr_rt_vision.position)

        test_cmd.synchronized_command.mobility_command.stance_request.end_time.CopyFrom(
            self.robot.time_sync.robot_timestamp_from_local_secs(time.time() + 10))

        blocking_command(self._command_client, test_cmd, check_stance_status)

    def takeoff_qc_prepare(self):
        self.zupvt_init()
        self.prepare_spot()
        takeoff_request = missionRequest()

        takeoff_request.stateRequest = 'BREAKAWAY'

        result = self.qc_service(takeoff_request)

        if result:
            self.x500_undocking()
            self.revert_pose()

    def landing_qc_prepare(self):
        self.prepare_spot()
        takeoff_request = missionRequest()
        takeoff_request.stateRequest = 'TOUCHDOWN'

        result = self.qc_service(takeoff_request)

        if result:
            self.x500_docking()
            self.revert_pose()

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
        #blocking_stand(self._command_client, timeout_sec=10)
        robot_command.blocking_stand(self._command_client)

        self.nominal_pose = self._state_client.get_robot_state()
        rospy.loginfo(f"foot x: {self.nominal_pose.foot_state}")
        #rospy.loginfo(f"foot y: {self.nominal_pose.foot_state.foot_position_rt_body.y}")

    def move_spot(self, dx, dy, dyaw, frame_name=ODOM_FRAME_NAME, stairs=False):
        transforms = self._state_client.get_robot_state().kinematic_state.transforms_snapshot
        # Build the transform for where we want the robot to be relative to where the body currently is.
        body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
        # We do not want to command this goal in body frame because the body will move, thus shifting
        # our goal. Instead, we transform this offset to get the goal position in the output frame
        # (which will be either odom or vision).
        out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
        out_tform_goal = out_tform_body * body_tform_goal

        # Command the robot to go to the goal point in the specified frame. The command will stop at the
        # new position.
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
            frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
        end_time = 10.0
        cmd_id = self._command_client.robot_command(lease=None, command=robot_cmd,
                                                    end_time_secs=time.time() + end_time)


        status = robot_command.block_for_trajectory_cmd(self._command_client, cmd_id)
        print(status)

        # # Wait until the robot has reached the goal.
        # while True:
        #     feedback = robot_command_client.robot_command_feedback(cmd_id)
        #     mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        #     if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
        #         print('Failed to reach the goal')
        #         return False
        #     traj_feedback = mobility_feedback.se2_trajectory_feedback
        #     if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
        #             traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
        #         print('Arrived at the goal.')
        #         return True
        #     time.sleep(1)


    def localize_to_map(self):
        """Trigger localization when near a fiducial."""
        robot_state = self._state_client.get_robot_state()

        snapshot = robot_state.kinematic_state.transforms_snapshot

        current_odom_tform_body = get_a_tform_b(snapshot, "odom", "body")

        current_odom_tform_body = current_odom_tform_body.to_proto()

        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body)
        
        localization_state = self._graph_nav_client.get_localization_state()
        #print(localization_state.localization.waypoint_id)

    def test_localization(self):
        localization_state = self._graph_nav_client.get_localization_state()
        print(localization_state.localization.waypoint_id)

    def upload_map(self):
        """Upload the graph and snapshots to the robot."""
        rospy.loginfo('Loading the graph from disk into local storage...')
        with open(self._upload_filepath + '/graph', 'rb') as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            rospy.loginfo(
                f'Loaded graph has {len(self._current_graph.waypoints)} waypoints and {len(self._current_graph.edges)} edges'
            )
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(f'{self._upload_filepath}/waypoint_snapshots/{waypoint.snapshot_id}',
                      'rb') as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
        for edge in self._current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            # Load the edge snapshots from disk.
            with open(f'{self._upload_filepath}/edge_snapshots/{edge.snapshot_id}',
                      'rb') as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        rospy.loginfo('Uploading the graph and snapshots to the robot...')
        time_before = time.time()
        true_if_empty = not len(self._current_graph.anchoring.anchors)
        response = self._graph_nav_client.upload_graph(graph=self._current_graph,
                                                       generate_new_anchoring=true_if_empty)
        # Upload any missing snapshots to the robot.
        upload_individually = False
        try:
            self._graph_nav_client.upload_snapshots(
                graph_nav_pb2.UploadSnapshotsRequest.Snapshots(waypoint_snapshots=[],
                                                               edge_snapshots=[]))
        except:
            # An empty UploadSnapshots request failed, fall back to slow RPC.
            upload_individually = True

        if upload_individually:
            for snapshot_id in response.unknown_waypoint_snapshot_ids:
                waypoint_snapshot = self._current_waypoint_snapshots[snapshot_id]
                self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
                rospy.loginfo(f'Uploaded {waypoint_snapshot.id}')
            for snapshot_id in response.unknown_edge_snapshot_ids:
                edge_snapshot = self._current_edge_snapshots[snapshot_id]
                self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
                rospy.loginfo(f'Uploaded {edge_snapshot.id}')
        else:
            # Upload in groups of 16MB.
            kMaxBytes = 16 * 1024 * 1024
            snapshots = []
            num_bytes = 0

            # Upload waypoint snapshots.
            for snapshot_id in response.unknown_waypoint_snapshot_ids:
                this_bytes = self._current_waypoint_snapshots[snapshot_id].ByteSize()
                if len(snapshots) > 0 and this_bytes + num_bytes > kMaxBytes:
                    rospy.loginfo(f'Uploading {len(snapshots)} waypoint snapshots')
                    self._graph_nav_client.upload_snapshots(
                        graph_nav_pb2.UploadSnapshotsRequest.Snapshots(
                            waypoint_snapshots=snapshots, edge_snapshots=[]))
                    snapshots = []
                    num_bytes = 0
                snapshots.append(self._current_waypoint_snapshots[snapshot_id])
                num_bytes += this_bytes
            if len(snapshots) > 0:
                rospy.loginfo(f'Uploading final {len(snapshots)} waypoint snapshots')
                self._graph_nav_client.upload_snapshots(
                    graph_nav_pb2.UploadSnapshotsRequest.Snapshots(waypoint_snapshots=snapshots,
                                                                   edge_snapshots=[]))

            # Upload edge snapshots.
            snapshots = []
            num_bytes = 0
            for snapshot_id in response.unknown_edge_snapshot_ids:
                this_bytes = self._current_edge_snapshots[snapshot_id].ByteSize()
                if len(snapshots) > 0 and this_bytes + num_bytes > kMaxBytes:
                    rospy.loginfo(f'Uploading {len(snapshots)} edge snapshots')
                    self._graph_nav_client.upload_snapshots(
                        graph_nav_pb2.UploadSnapshotsRequest.Snapshots(
                            waypoint_snapshots=[], edge_snapshots=snapshots))
                    snapshots = []
                    num_bytes = 0
                snapshots.append(self._current_edge_snapshots[snapshot_id])
                num_bytes += this_bytes
            if len(snapshots) > 0:
                rospy.loginfo(f'Uploading final {len(snapshots)} edge snapshots')
                self._graph_nav_client.upload_snapshots(
                    graph_nav_pb2.UploadSnapshotsRequest.Snapshots(waypoint_snapshots=[],
                                                                   edge_snapshots=snapshots))
        upload_time = time.time() - time_before
        rospy.loginfo(
            f'Uploaded graph and {len(response.unknown_waypoint_snapshot_ids)} (of {len(self._current_graph.waypoints)}) waypoints and {len(response.unknown_edge_snapshot_ids)} (of {len(self._current_graph.edges)}) edges, elapsed time {round(upload_time * 1000)}ms'
        )

        #CHECK TO MAKE SURE MAP IS FOUND!!
        localization_state = self._graph_nav_client.get_localization_state()

    def clear_graphs(self):
        """Clear the state of the map on the robot, removing all waypoints and
        edges."""
        return self._graph_nav_client.clear_graph()

    def nav_to_waypoint(self, waypoint):
        """Navigate to a specific waypoint."""

        destination_waypoint = graph_nav_utils.find_unique_waypoint_id(
            waypoint, self._current_graph, self._current_annotation_name_to_wp_id)
        if not destination_waypoint:
            # Failed to find the appropriate unique waypoint id for the navigation command.
            return
        # if not self.toggle_power(should_power_on=True):
        #     print('Failed to power on the robot, and cannot complete navigate to request.')
        #     return

        nav_to_cmd_id = None
        # Navigate to the destination waypoint.
        is_finished = False
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            try:
                nav_to_cmd_id = self._graph_nav_client.navigate_to(destination_waypoint, 1.0,
                                                                   command_id=nav_to_cmd_id)
            except ResponseError as e:
                print(f'Error while navigating {e}')
                break
            time.sleep(.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)

        # # Power off the robot if appropriate.
        # if self._powered_on and not self._started_powered_on:
        #     # Sit the robot down + power off after the navigation command is complete.
        #     self.toggle_power(should_power_on=False)
    
    def nav_route(self, route):
        """Navigate through a specific route of waypoints."""
        waypoint_ids = route
        for i in range(len(waypoint_ids)):
            waypoint_ids[i] = graph_nav_utils.find_unique_waypoint_id(
                waypoint_ids[i], self._current_graph, self._current_annotation_name_to_wp_id)
            if not waypoint_ids[i]:
                # Failed to find the unique waypoint id.
                return

        edge_ids_list = []
        all_edges_found = True
        # Attempt to find edges in the current graph that match the ordered waypoint pairs.
        # These are necessary to create a valid route.
        for i in range(len(waypoint_ids) - 1):
            start_wp = waypoint_ids[i]
            end_wp = waypoint_ids[i + 1]
            edge_id = self._match_edge(self._current_edges, start_wp, end_wp)
            if edge_id is not None:
                edge_ids_list.append(edge_id)
            else:
                all_edges_found = False
                print(f'Failed to find an edge between waypoints: {start_wp} and {end_wp}')
                print(
                    'List the graph\'s waypoints and edges to ensure pairs of waypoints has an edge.'
                )
                break

        if all_edges_found:
            
            # if not self.toggle_power(should_power_on=True):
            #     print('Failed to power on the robot, and cannot complete navigate route request.')
                # print("WTF")
                # return

            # Navigate a specific route.
            route = self._graph_nav_client.build_route(waypoint_ids, edge_ids_list)
            is_finished = False
            while not is_finished:
                # Issue the route command about twice a second such that it is easy to terminate the
                # navigation command (with estop or killing the program).
                nav_route_command_id = self._graph_nav_client.navigate_route(
                    route, cmd_duration=1.0)
                time.sleep(.5)  # Sleep for half a second to allow for command execution.
                # Poll the robot for feedback to determine if the route is complete. Then sit
                # the robot down once it is finished.
                is_finished = self._check_success(nav_route_command_id)

    def _match_edge(self, current_edges, waypoint1, waypoint2):
        """Find an edge in the graph that is between two waypoint ids."""
        # Return the correct edge id as soon as it's found.
        for edge_to_id in current_edges:
            for edge_from_id in current_edges[edge_to_id]:
                if (waypoint1 == edge_to_id) and (waypoint2 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(from_waypoint=waypoint2, to_waypoint=waypoint1)
                elif (waypoint2 == edge_to_id) and (waypoint1 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(from_waypoint=waypoint1, to_waypoint=waypoint2)
        return None

    def list_graphs(self):
        """List the waypoint ids and edge ids of the graph currently on the
        robot."""

        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print('Empty graph.')
            return
        self._current_graph = graph

        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id

        # Update and print waypoints and edges
        self._current_annotation_name_to_wp_id, self._current_edges, self._ordered_ids = graph_nav_utils.update_waypoints_and_edges(
            graph, localization_id)
        target_waypoints = list(self._ordered_ids.values())
        #print(target_waypoints)

    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit
        when command succeeds."""
        if command_id == -1:
            # No command, so we have no status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print('Robot got lost when navigating the route, the robot will now sit down.')
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print('Robot got stuck when navigating the route, the robot will now sit down.')
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            print('Robot is impaired.')
            return True
        else:
            # Navigation command is not complete yet.
            return False

    def pose_spot(self):
        pass

    def mock_autowalk(self):
        # Clear any graphs on the Spot robot
        rospy.loginfo("Clearing Spot's current graph.")
        self.clear_graphs()

        # Upload the graph and intialize
        rospy.loginfo("Uploading mission map to Spot and localizing...")
        self.upload_map()
        self.localize_to_map()
        self.list_graphs()

        #Check that it was localized. If localized:
        #Go to Takeoff Point
        target_waypoints = list(self._ordered_ids.values())
        #print(target_waypoints)
        ###x500_Autowalk_Inspection###

        # self.nav_route(target_waypoints[1:6])
        # self.takeoff_qc_prepare()
        # time.sleep(1.5)
        # #Go to Inspection Point 1
        # self.nav_route(target_waypoints[6:8])
        # time.sleep(1.5)
        # # Go to Inspection Point 2
        # self.nav_route(target_waypoints[8:13])
        # time.sleep(1.5)
        # # Go to Inspection Point 3
        # self.nav_route(target_waypoints[13:17])
        # time.sleep(1.5)
        # # Go to Rendezvous Point
        # self.nav_route(target_waypoints[17:22])
        # self.landing_qc_prepare()
        # time.sleep(1.5)

        ###simple_x500_inspection###
        rospy.loginfo("Starting Mission...")
        self.nav_route(target_waypoints[1:3])
        
        #Give sometime to start the hardware launch
        ospy.loginfo("START THE HARDWARE.LAUNCH ROS LAUNCH FILE AND BAG FILE!")
        time.sleep(18.0)
        self.takeoff_qc_prepare()
        time.sleep(1.5)
        
        #Go to Inspection Point 1
        rospy.loginfo("Going to Inspection Point 1")
        inspect_request = missionRequest()
        mission_sps = PoseArray()
        sp_1 = Pose()

        sp_1.position.x = 1.5
        sp_1.position.y = -1.35
        sp_1.position.z = 1.5

        sp_1.orientation.x = 0
        sp_1.orientation.y = 0
        sp_1.orientation.z = 0
        sp_1.orientation.w = 1

        mission_sps.poses.append(sp_1)

        inspect_request.stateRequest = 'INSPECT'
        inspect_request.setpoints = mission_sps
        

        result = self.qc_service(inspect_request)

        if result:
            self.nav_route(target_waypoints[3:6])
        
        time.sleep(1.5)
        
        # Go to Inspection Point 2
        self.nav_route(target_waypoints[6:13])
        self.landing_qc_prepare()
        time.sleep(1.5)

        rospy.loginfo("Mission Completed. Releasing Leases...")
        #Dont add this, it makes spot sit down after mission.
        #self.release_lease()


if __name__ == '__main__':
    node = SpotBodyPublisher()
    node.stand()
    # node.landing_qc_prepare()
    #node.reset_pose()
    #node.stand()
    # node.release_lease()
    #node.clear_graphs()
    # node.test_localization()
    #node.nav_to_waypoint("sneezy-gadfly-qSLBadUY.hL7LByNUKQaNQ==")
    node.mock_autowalk()