
import sys
from rclpy.logging import rclpy
from rclpy.executors import MultiThreadedExecutor

from bm_framework_ros2_pkg.ros_driver.nodes.socket_node import SocketClientNode


RELAY_HOST = "192.168.0.2"  # The NodeMCU's address'
RELAY_PORT = 8266  # The port used by the NodeMCU


def main():
    rclpy.init(args=sys.argv)
    relay_node = SocketClientNode(RELAY_HOST, RELAY_PORT, 1024, "limb_readings", "limb_sensors", True)
    executor = MultiThreadedExecutor()
    executor.add_node(relay_node)
    executor.spin()


if __name__ == "__main__":
    main()
