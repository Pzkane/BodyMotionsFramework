import sys
from rclpy.logging import rclpy
from rclpy.executors import MultiThreadedExecutor

from bm_framework_ros2_pkg.ros_driver.nodes.mw_sensor_node import MWSensorNode


def main():
    rclpy.init(args=sys.argv)
    server_node = MWSensorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(server_node)
    executor.spin()


if __name__ == "__main__":
    main()

