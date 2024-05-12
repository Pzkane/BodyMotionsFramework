import sys
from rclpy.logging import rclpy
from rclpy.executors import MultiThreadedExecutor

from bm_framework_ros2_pkg.ros_driver.nodes.bm_node import BodyMotionsServerNode


def main():
    rclpy.init(args=sys.argv)
    server_node = BodyMotionsServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(server_node)
    executor.spin()


if __name__ == "__main__":
    main()
