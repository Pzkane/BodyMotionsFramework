from typing import Optional
from rclpy.logging import LoggingSeverity
from rclpy.node import List, Node
from rclpy.task import Future

from bm_framework_interfaces_ros2_pkg.srv import GetSensors, InitiateStaticHold
from bm_framework_interfaces_ros2_pkg.msg import StaticPoseResult, Sensors
from bm_framework_ros2_pkg.qt_app.types import Pose


class BMApplicationNode(Node):
    def __init__(self, *args, **kwargs):
        super(BMApplicationNode, self).__init__(node_name="bm_app", namespace="body_motions_framework", *args, **kwargs)
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.__init_services()
        self.__poses: List[Pose]
        self.__active_pose: Optional[Pose] = None

    def __init_services(self):
        self.cli_get_sensors = self.create_client(GetSensors, "get_sensors")
        self.cli_initiate_static_hold = self.create_client(InitiateStaticHold, "initiate_static_hold")

    def get_sensor_readings(self, qt_callback):
        req = GetSensors.Request()
        self.future: Future = self.cli_get_sensors.call_async(req)
        self.future.add_done_callback(qt_callback)
    
    def initiate_static_hold(self):
        if self.__active_pose is None:
            raise ValueError("Cannot initiate static hold: Active pose is not set")
        req = InitiateStaticHold.Request()
        req.target_hold_pose = Sensors(sensors=self.__active_pose.sensors)
        self.future: Future = self.cli_initiate_static_hold.call_async(req)
        # Subscribe onto result from server
        self.future.add_done_callback(self.subscribe_to_static_hold_result)

    def set_poses(self, poses: List[Pose]):
        self.__poses = poses

    def set_current_pose(self, pose_name: str):
        for pose in self.__poses:
            if pose.name == pose_name:
                self.__active_pose = pose
                return
        raise ValueError(f"No such pose '{pose_name}'")

    def subscribe_to_static_hold_result(self, _: Future):
        self.sub_static_hold = self.create_subscription(StaticPoseResult, "static_pose_result", self.__cb_static_pose_result, 1)

    def __cb_static_pose_result(self, result: StaticPoseResult):
        pass
        # self.get_logger().info(str(result.is_aligned))
