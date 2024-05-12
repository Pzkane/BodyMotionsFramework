import sys
from rclpy.logging import rclpy
from rclpy.executors import MultiThreadedExecutor

from bm_framework_ros2_pkg.ros_driver.nodes.socket_node import SocketClientNode


RELAY_HOST = "192.168.0.101"  # The server's hostname or IP address
RELAY_PORT = 50001  # The port used by the server


def main():
    rclpy.init(args=sys.argv)
    relay_node = SocketClientNode(RELAY_HOST, RELAY_PORT, 1024, "mmr_relay", "relay_node")
    executor = MultiThreadedExecutor()
    executor.add_node(relay_node)
    executor.spin()


if __name__ == "__main__":
    main()
