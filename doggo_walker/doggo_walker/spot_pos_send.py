#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses


class CursesPublisherNode(Node):
    def __init__(self):
        super().__init__("curses_publisher")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        self.get_logger().info("Curses Publisher Node started.")

    def publish_message(self, message_str: str):
        msg = String()
        msg.data = message_str
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {message_str}")


def main():
    rclpy.init()
    node = CursesPublisherNode()

    def curses_main(stdscr):
        # Initialize curses settings
        stdscr.clear()
        stdscr.nodelay(True)  # Make getch() non-blocking
        stdscr.addstr(
            0, 0, "Press Enter to publish a message. Press 'q' to quit."
        )
        message_counter = 0

        while True:
            key = stdscr.getch()
            if key == ord("q"):
                break
            elif key in (10, curses.KEY_ENTER):
                # When Enter is pressed, create and publish a message
                message = f"Hello ROS2 {message_counter}"
                node.publish_message(message)
                message_counter += 1
                stdscr.addstr(2, 0, f"Published: {message}          ")
                stdscr.refresh()

            # Allow ROS2 to process callbacks with a short timeout
            rclpy.spin_once(node, timeout_sec=0.1)

    # Wrap the curses interface so it handles initialization and cleanup
    curses.wrapper(curses_main)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
