import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class vioNode(object):
    def __init__(self):
        # Initialize the node (name must be unique within the ROS network)
        rospy.init_node('px4_vio', anonymous=False)

        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        self.sub = rospy.Subscriber('/ov_msckf/poseimu', PoseWithCovarianceStamped, self.callback)

    def callback(self, msg):

        px4_msg = PoseStamped()
        #px4_msg.header = msg.header
        px4_msg.pose = msg.pose.pose

        # Publish it
        self.pub.publish(px4_msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = vioNode()
        node.run()
    except rospy.ROSInterruptException:
        # This is raised when the node is killed (e.g., Ctrl-C)
        rospy.loginfo("vioNode shutting down.")