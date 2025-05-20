#!/usr/bin/env python
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class OffboardController:
    def __init__(self):
        rospy.init_node('offb_node_py')
        self.current_state = State()
        self.state_sub = rospy.Subscriber('mavros/state', State, self._state_cb)
        self.local_pos_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

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

