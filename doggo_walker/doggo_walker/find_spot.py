import sys

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client import RpcError
import dotenv

hostname = None
bd_user = None
bd_pass = None
sdk = bosdyn.client.create_standard_sdk("findSpot_ROS2")
robot = None


def get_creds():
    config = dotenv.dotenv_values("../config/.env")
    global hostname
    hostname = config.get("ROBOT_IP")
    bd_user = config.get("BOSDYN_CLIENT_USERNAME")
    bd_pass = config.get("BOSDYN_CLIENT_PASSWORD")

    return bd_user, bd_pass


def connect():
    # Create robot instance and authenticate usertry
    try:
        robot = sdk.create_robot(hostname)
        bosdyn.client.util.authenticate(robot, askpass=get_creds)
        # LOGGER.info(f"Spot @ {hostname} auth successful")
        robot.time_sync.wait_for_sync()
        return robot

    except RpcError:
        # LOGGER.error(f"Error connecting with robot {hostname}")
        print(f"ERROR: Failed to connect to robot {hostname}")


def main():
    get_creds()
    robot = connect()
    # Create robot object with an image client.
    #     robot = sdk.create_robot(options.hostname)
    # bosdyn.client.util.authenticate(robot)
    robot_state_client = robot.ensure_client(
        RobotStateClient.default_service_name
    )

    # # Make a robot state request
    # if options.command == "state":
    message = robot_state_client.get_robot_state()
    # field = str(message.battery_states[0].charge_percentage).split(":")
    # print(field)
    snapshot = message.kinematic_state.transforms_snapshot

    odom = snapshot.child_to_parent_edge_map["odom"]

    print(odom.parent_tform_child)
    print(odom.parent_tform_child.position.x)
    # elif options.command == "hardware":
    #     print(robot_state_client.get_hardware_config_with_link_info())
    # elif options.command == "metrics":
    #     print(robot_state_client.get_robot_metrics())


if __name__ == "__main__":
    if not main():
        sys.exit(1)
