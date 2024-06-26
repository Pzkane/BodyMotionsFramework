from typing import Dict, Optional

from rclpy.logging import LoggingSeverity
from rclpy.node import List, Node
from rclpy.task import Future
from std_srvs.srv import Trigger

from bm_framework_interfaces_ros2_pkg.msg import (
    ImpulseData,
    PosesStatus,
    Sensors,
    StaticPoseResult,
)
from bm_framework_interfaces_ros2_pkg.srv import (
    ExecutePosePlan,
    GetSensors,
    InitiateImpulse,
    InitiateStaticHold,
)
from bm_framework_ros2_pkg.qt_app.signals import GuiSignals
from bm_framework_ros2_pkg.qt_app.types import Pose, PoseType


class BMApplicationNode(Node):
    def __init__(self, gui_signals: GuiSignals, *args, **kwargs):
        super(BMApplicationNode, self).__init__(node_name="bm_app", namespace="body_motions_framework", *args, **kwargs)
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.gui_signals = gui_signals
        self.__init_services()
        self.__poses: List[Pose]
        self.active_pose: Optional[Pose] = None

        self.__tmp_impulse_pose_name: str = ""

    @property
    def poses(self) -> Dict[str, Pose]:
        poses: Dict[str, Pose] = {}
        for pose in self.__poses:
            poses[pose.name] = pose
        return poses

    def __init_services(self):
        self.cli_get_sensors = self.create_client(GetSensors, "get_sensors")
        self.cli_initiate_static_hold = self.create_client(InitiateStaticHold, "initiate_static_hold")
        self.cli_stop_static_hold = self.create_client(Trigger, "stop_static_hold")
        self.cli_initiate_impulse = self.create_client(InitiateImpulse, "initiate_impulse")
        self.cli_execute_pose_plan = self.create_client(ExecutePosePlan, "execute_pose_plan")

    def get_sensor_readings(self, qt_callback):
        req = GetSensors.Request()
        self.future: Future = self.cli_get_sensors.call_async(req)
        self.future.add_done_callback(qt_callback)
   
    def initiate_impulse(self, pose_name: str, impulse_target_sensor: str, active_impulse_sensors: Dict[str, bool]):
        req = InitiateImpulse.Request()
        self.__tmp_impulse_pose_name = pose_name
        msg = ImpulseData()
        msg.target_impulse_sensor_name = impulse_target_sensor
        msg.limb_left_active = active_impulse_sensors["limb_left"]
        msg.left_active = active_impulse_sensors["left"]
        msg.center_active = active_impulse_sensors["center"]
        msg.right_active = active_impulse_sensors["right"]
        msg.limb_right_active = active_impulse_sensors["limb_right"]
        req.impulse_data = msg
        self.future: Future = self.cli_initiate_impulse.call_async(req)
        # Subscribe onto result from server
        self.future.add_done_callback(self.subscribe_to_impulse_result)

    def initiate_static_hold(self):
        if self.active_pose is None:
            raise ValueError("Cannot initiate static hold: Active pose is not set")
        req = InitiateStaticHold.Request()
        req.target_hold_pose = Sensors(sensors=self.active_pose.sensors)
        self.future: Future = self.cli_initiate_static_hold.call_async(req)
        # Subscribe onto result from server
        self.future.add_done_callback(self.subscribe_to_static_hold_result)

    def execute_pose_plan(self, poses: List[Pose]):
        if len(poses) == 0:
            raise RuntimeError("Pose planner cannot accept empty plan!")
        # Send list of poses, pose plan executor will execute poses as they are inserted into
        # pose array
        req = ExecutePosePlan.Request()
        planned_poses: List[Sensors] = []
        planned_poses_types: List[str] = []
        for pose in poses:
            planned_poses.append(Sensors(sensors=pose.sensors))
            planned_poses_types.append(pose.type.name)
        req.planned_poses = planned_poses
        req.planned_poses_types = planned_poses_types
        self.future: Future = self.cli_execute_pose_plan.call_async(req)
        # Subscribe to plan execution status
        self.future.add_done_callback(self.subscribe_to_plan_execution)

    def finish_impulse(self):
        self.gui_signals.finish_impulse.emit()

    def finish_static_hold(self):
        if self.active_pose is None:
            raise ValueError("Cannot finish static hold: Active pose is not set")
        req = Trigger.Request()
        self.future: Future = self.cli_stop_static_hold.call_async(req)
        self.future.add_done_callback(self.cleanup_after_hold)

    def set_poses(self, poses: List[Pose]):
        self.__poses = poses

    def set_current_pose(self, pose_name: str):
        for pose in self.__poses:
            if pose.name == pose_name:
                self.active_pose = pose
                return
        raise ValueError(f"No such pose '{pose_name}'")

    def subscribe_to_impulse_result(self, _: Future):
        self.sub_impulse = self.create_subscription(Sensors, "impulse_result", self.__cb_impulse_result, 1)

    def subscribe_to_plan_execution(self, _: Future):
        self.sub_plan_execution_status = self.create_subscription(PosesStatus, "execution_plan_status", self.__cb_execution_plan, 1)

    def subscribe_to_static_hold_result(self, _: Future):
        self.sub_static_hold = self.create_subscription(StaticPoseResult, "static_pose_result", self.__cb_static_pose_result, 1)

    def cleanup_after_hold(self, _: Future):
        self.gui_signals.stop_static_pose.emit()

    def __cb_execution_plan(self, status: PosesStatus):
        # Stop if pose planner response ended its execution
        if status.planner_response == "stop":
            self.gui_signals.stop_plan_execution.emit()
            return
        # Receive status, because pose needs to be "reached" update table response
        self.gui_signals.update_pose_planner_response.emit(status.current_pose_id, status.current_pose_status, status.planner_response)
        # Based on status received, visually move to the next pose, this already done on backend
        if status.planner_response == "next":
            self.gui_signals.iterate_next_pose.emit()

    def __cb_impulse_result(self, result: Sensors):
        self.gui_signals.create_impulse_pose.emit(self.__tmp_impulse_pose_name, PoseType.IMPULSE, result)

    def __cb_static_pose_result(self, result: StaticPoseResult):
        self.gui_signals.update_static_pose_status.emit(result.is_aligned, result.pose_diff)
