from copy import deepcopy
import math
import time
from typing import Any, Dict, Optional

import numpy as np
from rclpy import List
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.timer import Timer
from scipy.spatial import distance
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bm_framework_interfaces_ros2_pkg.msg import (
    ImpulseData,
    Sensor,
    Sensors,
    StaticPoseResult,
)
from bm_framework_interfaces_ros2_pkg.srv import (
    GetSensors,
    InitiateImpulse,
    InitiateStaticHold,
)
from bm_framework_ros2_pkg.ros_driver.parser import (
    CenterDataParser,
    LimbData,
    LimbDataParser,
    PeripheryDataParser,
    TorsoData,
)


class BodyMotionsServerNode(Node):
    def __init__(self, *args, **kwargs):
        super(BodyMotionsServerNode, self).__init__(node_name="bm_server", namespace="body_motions_framework", *args, **kwargs)
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.__torso_data = TorsoData(None, None, None)
        self.__limbs_data: Optional[LimbData] = None

        self.__static_pose: Sensors
        self.__static_pose_timer: Optional[Timer] = None
        self.__static_pose_reference_sensor: Sensor

        self.__target_impulse_sensor: Sensor
        self.__impulse_data: ImpulseData
        self.__impulse_timer: Optional[Timer] = None
        self.__last_accel_target_sensor: Optional[Sensor]
        self.__last_accel_sensors: Optional[Sensors]
        self.__time_frame: float
        self.__impulse_published: bool
        self.__reset_impulse_state()

        self.__init_services()
        self.__init_publishers()
        self.__init_relay_connection()
        self.__init_boards_connrections()
        
        #### Legacy reference ####
        self._handsDownRegistered: bool = False
        self._lastHandsDownTime: float
        self._lastAccelLeft: Optional[Any] = None
        self._lastAccelRight: Optional[Any] = None
        self._timeframeL: float = time.time()
        self._timeframeR: float = time.time()

    def get_rt_sensors(self) -> Dict[str, Sensor]:
        sensors: Dict[str, Sensor] = {}
        if self.__limbs_data:
            sensors["limb_left"] = Sensor(
                name="limb_left",
                x_acc=self.__limbs_data.x_acc_l,
                y_acc=self.__limbs_data.y_acc_l,
                z_acc=self.__limbs_data.z_acc_l,
                pitch=self.__limbs_data.pitch_l,
                roll=self.__limbs_data.roll_l,
                yaw=0.0,
                orientation_active=self.__limbs_data.orientation_active_l,
                orientation_reference=self.__limbs_data.orientation_reference_l,
                target_impulse=self.__limbs_data.target_impulse_l,
            )
        else:
            sensors["limb_left"] = Sensor(
                name="limb_left",
                x_acc=0.0,
                y_acc=0.0,
                z_acc=0.0,
                pitch=0.0,
                roll=0.0,
                yaw=0.0,
                orientation_active=True,
                orientation_reference=False,
                target_impulse=False,
            )

        def _get_torso_sensor_msg(sensor: Any, name: str):
            if sensor is not None:
                rpy = Rotation.from_quat([sensor["qx"], sensor["qy"], sensor["qz"], sensor["qw"]]).as_euler("xyz")
                r = math.degrees(rpy[0])
                p = math.degrees(rpy[1])
                y = math.degrees(rpy[2])
                return Sensor(
                    name=name,
                    x_acc=sensor["x"],
                    y_acc=sensor["y"],
                    z_acc=sensor["z"],
                    roll=r,# if r > 0 else r + 360,
                    pitch=p,# if p > 0 else p + 360,
                    yaw=y,# if y > 0 else y + 360,
                    orientation_active=sensor["orientation_active"],
                    orientation_reference=sensor["orientation_reference"],
                    target_impulse=sensor["target_impulse"],
                )
            else:
                return Sensor(
                    name=name,
                    x_acc=0.0,
                    y_acc=0.0,
                    z_acc=0.0,
                    pitch=0.0,
                    roll=0.0,
                    yaw=0.0,
                    orientation_active=True,
                    orientation_reference=False,
                    target_impulse=False,
                )

        sensors["left"] = _get_torso_sensor_msg(self.__torso_data.left, "left")
        sensors["center"] = _get_torso_sensor_msg(self.__torso_data.center, "center")
        sensors["right"] = _get_torso_sensor_msg(self.__torso_data.right, "right")

        if self.__limbs_data:
            sensors["limb_right"] = Sensor(
                name="limb_right",
                x_acc=self.__limbs_data.x_acc_r,
                y_acc=self.__limbs_data.y_acc_r,
                z_acc=self.__limbs_data.z_acc_r,
                pitch=self.__limbs_data.pitch_r,
                roll=self.__limbs_data.roll_r,
                yaw=0.0,
                orientation_active=self.__limbs_data.orientation_active_r,
                orientation_reference=self.__limbs_data.orientation_reference_r,
                target_impulse=self.__limbs_data.target_impulse_r,
            )
        else:
            sensors["limb_right"] = Sensor(
                name="limb_right",
                x_acc=0.0,
                y_acc=0.0,
                z_acc=0.0,
                pitch=0.0,
                roll=0.0,
                yaw=0.0,
                orientation_active=True,
                orientation_reference=False,
                target_impulse=False,
            )

        return sensors

    def publish_first_impulse_result(self, impulse_data: ImpulseData):
        self.__impulse_published = False
        self.__impulse_data = impulse_data
        # Determine target sensor
        self.__target_impulse_sensor = self.get_rt_sensors()[self.__impulse_data.target_impulse_sensor_name]
        # Spin timer until maximum impulse is received, then pubish the result
        if self.__impulse_timer:
            self.__impulse_timer.reset()
        else:
            self.__impulse_timer = self.create_timer(0.0001, self.__cb_pub_impulse_result)

    def hold_pose_and_publish_result(self, pose: Sensors):
        self.__static_pose = pose
        # Pose is NOT a reference by itself; marked sensor that is a part of a 
        # pose IS the reference
        self.__set_reference_sensor(pose)
        self.__set_sensors_properies(pose)
        if self.__static_pose_timer:
            self.__static_pose_timer.reset()
        else:
            self.__static_pose_timer = self.create_timer(0.0001, self.__cb_pub_static_pose_result)

    def __init_boards_connrections(self):
        self.create_subscription(String, "/body_motions_framework/limb_readings", self.__cb_limb_readings, 1)
        self.get_logger().info("Limb sensors connected!")

    def __init_relay_connection(self):
        self.create_subscription(String, "/body_motions_framework/mmr_ble", self.__cb_ble_data, 1)
        self.get_logger().info("BLE node connected!")
        self.create_subscription(String, "/body_motions_framework/mmr_relay", self.__cb_relay_data, 1)
        self.get_logger().info("Relay connected!")

    def __init_services(self):
        self.cli_get_sensors = self.create_service(GetSensors, "get_sensors", self.__cb_get_rt_sensors)
        self.cli_initiate_static_hold = self.create_service(InitiateStaticHold, "initiate_static_hold", self.__cb_initiate_static_hold)
        self.cli_stop_static_hold = self.create_service(Trigger, "stop_static_hold", self.__cb_stop_static_pose_processing)
        self.cli_initiate_impulse = self.create_service(InitiateImpulse, "initiate_impulse", self.__cb_initiate_impulse)

    def __init_publishers(self):
        self.pub_static_pose_hold_result = self.create_publisher(StaticPoseResult, "static_pose_result", 1)
        self.pub_impulse_result = self.create_publisher(Sensors, "impulse_result", 1)

    def __reset_impulse_state(self):
        self.__last_accel_target_sensor = None
        self.__last_accel_sensors = None
        self.__impulse_published = False
        self.__time_frame = time.time()

    def __set_sensors_properies(self, pose: Sensors):
        if self.__limbs_data is None:
            raise RuntimeError("Limb data has not been set yet!")
        if self.__torso_data.left is None or self.__torso_data.center is None or self.__torso_data.right is None:
            raise RuntimeError("Torso data has not been fully set yet!")
        for sensor in pose.sensors:
            if sensor.name == "limb_left":
                self.__limbs_data.orientation_active_l = sensor.orientation_active
                self.__limbs_data.orientation_reference_l = sensor.orientation_reference
                self.__limbs_data.target_impulse_l = sensor.target_impulse_l
            if sensor.name == "left":
                self.__torso_data.left["orientation_active"] = sensor.orientation_active
                self.__torso_data.left["orientation_reference"] = sensor.orientation_reference
                self.__torso_data.left["target_impulse"] = sensor.target_impulse
            if sensor.name == "center":
                self.__torso_data.center["orientation_active"] = sensor.orientation_active
                self.__torso_data.center["orientation_reference"] = sensor.orientation_reference
                self.__torso_data.center["target_impulse"] = sensor.target_impulse
            if sensor.name == "right":
                self.__torso_data.right["orientation_active"] = sensor.orientation_active
                self.__torso_data.right["orientation_reference"] = sensor.orientation_reference
                self.__torso_data.right["target_impulse"] = sensor.target_impulse
            if sensor.name == "limb_right":
                self.__limbs_data.orientation_active_r = sensor.orientation_active
                self.__limbs_data.orientation_reference_r = sensor.orientation_reference
                self.__limbs_data.target_impulse_r = sensor.target_impulse_r

    def __set_reference_sensor(self, pose: Sensors):
        for sensor in pose.sensors:
            if sensor.orientation_reference:
                self.__static_pose_reference_sensor = sensor
                self.__sprs_rot = Rotation.from_euler("xyz", [self.__static_pose_reference_sensor.roll, self.__static_pose_reference_sensor.pitch, self.__static_pose_reference_sensor.yaw])
                return
        raise RuntimeError("Reference sensor for static pose is not set!")

    def __cb_ble_data(self, ble_meta_motion_r_data: String):
        data = CenterDataParser.deserialize(ble_meta_motion_r_data.data)
        old_center = deepcopy(self.__torso_data.center)
        self.__torso_data.center = data.center
        if old_center is not None:
            self.__torso_data.center["orientation_active"] = old_center["orientation_active"]
            self.__torso_data.center["orientation_reference"] = old_center["orientation_reference"]
            self.__torso_data.center["target_impulse"] = old_center["target_impulse"]

    def __cb_relay_data(self, relay_meta_motion_r_data: String):
        data = PeripheryDataParser.deserialize(relay_meta_motion_r_data.data)
        old_left = deepcopy(self.__torso_data.left)
        old_right = deepcopy(self.__torso_data.right)
        self.__torso_data.left = data.left
        if old_left is not None:
            self.__torso_data.left["orientation_active"] = old_left["orientation_active"]
            self.__torso_data.left["orientation_reference"] = old_left["orientation_reference"]
            self.__torso_data.left["target_impulse"] = old_left["target_impulse"]
        self.__torso_data.right = data.right
        if old_right is not None:
            self.__torso_data.right["orientation_active"] = old_right["orientation_active"]
            self.__torso_data.right["orientation_reference"] = old_right["orientation_reference"]
            self.__torso_data.right["target_impulse"] = old_right["target_impulse"]

    def __cb_get_rt_sensors(self, _: GetSensors.Request, response: GetSensors.Response) -> GetSensors.Response:
        response.success = True
        response.message = "Retrieved all sensors"
        response.sensors = [sensor for sensor in self.get_rt_sensors().values()]
        return response

    def __cb_initiate_impulse(self, request: InitiateImpulse.Request, response: InitiateImpulse.Response) -> InitiateImpulse.Response:
        self.publish_first_impulse_result(request.impulse_data)
        response.success = True
        response.message = "Initiated impulse"
        return response

    def __cb_initiate_static_hold(self, request: InitiateStaticHold.Request, response: InitiateStaticHold.Response) -> InitiateStaticHold.Response:
        self.hold_pose_and_publish_result(request.target_hold_pose)
        response.success = True
        response.message = "Initiated pose hold"
        return response

    def __cb_pub_impulse_result(self):
        if self.__impulse_published:
            return
        rt_sensors = self.get_rt_sensors()
        rt_target_impulse_sensor: Sensor = rt_sensors[self.__target_impulse_sensor.name]
        # Register impulse
        delay = 300  # delay in ms between impulses (not needed)
        impulse_baseline_g = 1 # Minimal combined effort to register impulse
        accel = math.sqrt(rt_target_impulse_sensor.x_acc**2 + rt_target_impulse_sensor.y_acc**2 + rt_target_impulse_sensor.z_acc**2)
        accel_prev = math.sqrt(self.__last_accel_target_sensor.x_acc**2 + self.__last_accel_target_sensor.y_acc**2 + self.__last_accel_target_sensor.z_acc**2) if self.__last_accel_target_sensor else 0
        # Important: Compensate for different sensor type
        if rt_target_impulse_sensor.name == "limb_left":
            accel -= 0.990
            accel_prev -= 0.990
        if rt_target_impulse_sensor.name == "limb_right":
            accel -= 0.980
            accel_prev -= 0.980
        if rt_target_impulse_sensor.name == "center":
            accel -= 1
            accel_prev -= 1

        # Get time again for impulse after so many ticks
        timestamp = time.time() * 1000  # in milliseconds

        if accel >= impulse_baseline_g and accel > accel_prev:
            self.__last_accel_target_sensor = rt_target_impulse_sensor
            self.__last_accel_sensors = rt_sensors
        elif accel < accel_prev and accel_prev >= impulse_baseline_g:
            if self.__time_frame is None or timestamp - self.__time_frame >= delay:
                # Impulse winning condition
                if not self.__last_accel_sensors:
                    raise RuntimeError("Last sensor readings are not set!")

                # Prepare sensor data
                sensors: Dict[str, Sensor] = {}
                for last_sensor in self.__last_accel_sensors.values():
                    sensors[last_sensor.name] = last_sensor
                # Determine which ones are active for current impulse
                sensors["limb_left"].orientation_active = self.__impulse_data.limb_left_active
                sensors["left"].orientation_active = self.__impulse_data.left_active
                sensors["center"].orientation_active = self.__impulse_data.center_active
                sensors["right"].orientation_active = self.__impulse_data.right_active
                sensors["limb_right"].orientation_active = self.__impulse_data.limb_right_active
                # Set target sensor metadata
                sensors[self.__target_impulse_sensor.name].target_impulse = True
                # Publish the readings
                self.pub_impulse_result.publish(Sensors(sensors=[sensor for sensor in sensors.values()]))
                if not self.__impulse_timer:
                    raise RuntimeError("Something went wrong with impule timer initialization!")
                self.__impulse_published = True
                self.__impulse_timer.cancel()
                self.__reset_impulse_state()
                return
                # Do not update time frame as this loop will end after receiving the impulse
                # self.__time_frame = timestamp
            self.__last_accel_target_sensor = rt_target_impulse_sensor
            self.__last_accel_sensors = rt_sensors

    def __cb_pub_static_pose_result(self):
        def angular_difference(euler1, euler2) -> List[float]:
            diff = np.abs(np.array(euler1) - np.array(euler2))
            for idx, component in enumerate(diff):
                if component > 180:
                    diff[idx] = abs(diff[idx]-360)
            return diff

        sensors: Dict[str, Sensor] = self.get_rt_sensors()
        is_aligned: bool = True
        pose_diff: List[Sensor] = []
        rt_reference_sensor: Sensor = sensors[self.__static_pose_reference_sensor.name]
        base_sensor_rot_diff: List[float] = angular_difference(
            [self.__static_pose_reference_sensor.roll, self.__static_pose_reference_sensor.pitch, self.__static_pose_reference_sensor.yaw],
            [rt_reference_sensor.roll, rt_reference_sensor.pitch, rt_reference_sensor.yaw]
        )
        # self.get_logger().warn(str(base_sensor_rot_diff))
        # self.__static_pose_timer.cancel()
        # return
        reference_diff = Sensor(
            name=rt_reference_sensor.name,
            x_acc=distance.euclidean([rt_reference_sensor.x_acc,0,0], [self.__static_pose_reference_sensor.x_acc,0,0]),
            y_acc=distance.euclidean([rt_reference_sensor.y_acc,0,0], [self.__static_pose_reference_sensor.y_acc,0,0]),
            z_acc=distance.euclidean([rt_reference_sensor.z_acc,0,0], [self.__static_pose_reference_sensor.z_acc,0,0]),
            pitch=distance.euclidean([rt_reference_sensor.pitch,0,0], [self.__static_pose_reference_sensor.pitch,0,0]),
            roll=distance.euclidean([rt_reference_sensor.roll,0,0], [self.__static_pose_reference_sensor.roll,0,0]),
            yaw=distance.euclidean([rt_reference_sensor.yaw,0,0], [self.__static_pose_reference_sensor.yaw,0,0]),
            orientation_active=self.__static_pose_reference_sensor.orientation_active,
            orientation_reference=self.__static_pose_reference_sensor.orientation_reference,
        )
        # raise Exception(f"\n{str(self.__static_pose.sensors)}\n{str(sensors)}")
        for idx, sensor in enumerate(sensors.values()):
            # Will include lin acc just in case
            roll_diff = 0.0
            pitch_diff = 0.0
            yaw_diff = 0.0
            x_acc_diff = 0.0
            y_acc_diff = 0.0
            z_acc_diff = 0.0
            if sensor.orientation_active and sensor.name != self.__static_pose_reference_sensor.name:
                rt_reference_sensor_rad = [math.radians(rt_reference_sensor.roll), math.radians(rt_reference_sensor.pitch), math.radians(rt_reference_sensor.yaw)]
                rt_reference_sensor_rot = Rotation.from_euler("xyz", rt_reference_sensor_rad)
                
                reference_sensor = [
                    math.radians(self.__static_pose_reference_sensor.roll),
                    math.radians(self.__static_pose_reference_sensor.pitch),
                    math.radians(self.__static_pose_reference_sensor.yaw)]
                reference_sensor_rot = Rotation.from_euler("xyz", reference_sensor)
                reference_sensor_diff = reference_sensor_rot * rt_reference_sensor_rot.inv()

                static_pose_sensor = [
                    math.radians(self.__static_pose.sensors[idx].roll),
                    math.radians(self.__static_pose.sensors[idx].pitch),
                    math.radians(self.__static_pose.sensors[idx].yaw)]

                # Add RT rotation
                reference_sensor_rot = reference_sensor_rot * reference_sensor_diff
                static_pose_sensor_rot = Rotation.from_euler("xyz", static_pose_sensor) * reference_sensor_diff

                diff_stat_and_ref = reference_sensor_rot * static_pose_sensor_rot.inv()
                stat_transform = diff_stat_and_ref * static_pose_sensor_rot

                # Apply transform to real-time sensor reading
                rt_pose_sensor = [math.radians(sensor.roll), math.radians(sensor.pitch), math.radians(sensor.yaw)]
                rt_transform = diff_stat_and_ref * Rotation.from_euler("xyz", rt_pose_sensor)

                # Calculate difference between rotations
                diff = (rt_transform * stat_transform.inv()).as_euler("xyz", degrees=True)

                roll_diff = diff[0]# if r > 0 else r + 360
                pitch_diff = diff[1]# if p > 0 else p + 360
                yaw_diff = diff[2]# if y > 0 else y + 360
                x_acc_diff = distance.euclidean([sensor.x_acc,0,0], [self.__static_pose.sensors[idx].x_acc,0,0]) + reference_diff.x_acc
                y_acc_diff = distance.euclidean([sensor.y_acc,0,0], [self.__static_pose.sensors[idx].y_acc,0,0]) + reference_diff.y_acc
                z_acc_diff = distance.euclidean([sensor.z_acc,0,0], [self.__static_pose.sensors[idx].z_acc,0,0]) + reference_diff.z_acc
            if abs(roll_diff) > 20:
                is_aligned = False
            if abs(pitch_diff) > 20:
                is_aligned = False
            if abs(yaw_diff) > 20:
                is_aligned = False
            # self.get_logger().warn(f"{str(is_aligned)}, {str(roll_diff)}, {str(pitch_diff)}, {str(yaw_diff)};")
            pose_diff.append(Sensor(
                name=sensor.name,
                x_acc=x_acc_diff,
                y_acc=y_acc_diff,
                z_acc=z_acc_diff,
                pitch=pitch_diff,
                roll=roll_diff,
                yaw=yaw_diff
            ))

        msg = StaticPoseResult(
            pose_diff=Sensors(sensors=pose_diff),
            # pose_diff=Sensors(sensors=[sensor for sensor in sensors.values()]),
            is_aligned=is_aligned
        )
        self.pub_static_pose_hold_result.publish(msg)

    def __cb_stop_static_pose_processing(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = False
        if not self.__static_pose_timer:
            err: str = "Cannot stop timer that is not running"
            self.get_logger().error(err)
            response.message = err
            return response
        self.__static_pose_timer.cancel()
        response.success = True
        response.message = "Static pose hold is cancelled"
        return response








    #### Legacy reference ####

    def __cb_limb_readings(self, nodemcu_data: String):
        try:
            old_limbs_data = deepcopy(self.__limbs_data)
            self.__limbs_data = LimbDataParser.deserialize(nodemcu_data.data)
            if old_limbs_data is not None:
                self.__limbs_data.orientation_active_l = old_limbs_data.orientation_active_l
                self.__limbs_data.orientation_active_r = old_limbs_data.orientation_active_r
                self.__limbs_data.orientation_reference_l = old_limbs_data.orientation_reference_l
                self.__limbs_data.orientation_reference_r = old_limbs_data.orientation_reference_r
                self.__limbs_data.target_impulse_l = old_limbs_data.target_impulse_l
                self.__limbs_data.target_impulse_r = old_limbs_data.target_impulse_r
        except AttributeError:
            # Skip packet
            return

        if self.__torso_data.left is None:
            # self.get_logger().warn("empty")
            return
        # self.get_logger().info(f"Torso: {str(self.__torso_data)}, limbs: {str(self.__limbs_data)}")
        
        rPitch: float = round(((self.__limbs_data.pitch_r-8) * 100.0) / 100.0)
        lPitch: float = round(((self.__limbs_data.pitch_l-8) * 100.0) / 100.0)
        gloveTextToDash: str = f"{str(-1 * round((self.__limbs_data.roll_l+10) * 100.0) / 100.0)}\t\t{str(self.__limbs_data.roll_r)}"
        gloveTextToDash += "\n" + str(lPitch) + "\t\t" + str(rPitch)

        threshold = 5.0
        functionR: float = round( ((25.7143 - 0.714286 * rPitch) * 100.0) / 100.0)
        functionL: float = round( ((25.7143 - 0.714286 * lPitch) * 100.0) / 100.0)

        gloveTextToDash += "\n" + str(-1 * round((self.__torso_data.left["qw"] * 100) * 100.0) / 100.0) + "\t\t"
        gloveTextToDash += str(-1 * round((self.__torso_data.left["qx"] * 100) * 100.0) / 100.0) + "\t\t"
        gloveTextToDash += str(-1 * round((self.__torso_data.left["qy"] * 100) * 100.0) / 100.0) + "\t\t"
        gloveTextToDash += str(-1 * round((self.__torso_data.left["qz"] * 100) * 100.0) / 100.0)
        gloveTextToDash += "\n" + str(functionR)
        # By experimenting value ranges between -20 and {threshold}
        # AND is beyond time threshold
        downDelay: int = 1500
        if functionR >= threshold or functionL >= threshold:
            if not self._handsDownRegistered:
                self._handsDownRegistered = True;
                self._lastHandsDownTime = time.time();
            if time.time() - self._lastHandsDownTime >= downDelay/1000:
                # gloveTextToDash = "LOW HANDS\n" + str(functionL) + ":" + str(functionR)
                # self.get_logger().info("LOW HANDS\n" + str(functionL) + ":" + str(functionR))
                pass
        else:
            self._handsDownRegistered = False
        

# Register punches
        delay = 300  # delay in ms between punches
        punch_baseline = 4000  # Tweak this for database access
        accel_left = abs(self.__limbs_data.x_acc_l + self.__limbs_data.y_acc_l + self.__limbs_data.z_acc_l)
        accel_right = abs(self.__limbs_data.x_acc_r + self.__limbs_data.y_acc_r + self.__limbs_data.z_acc_r)
        accel_left_prev = abs(self._lastAccelLeft.x_acc_l + self._lastAccelLeft.y_acc_l + self._lastAccelLeft.z_acc_l) if self._lastAccelLeft else 0
        accel_right_prev = abs(self._lastAccelRight.x_acc_r + self._lastAccelRight.y_acc_r + self._lastAccelRight.z_acc_r) if self._lastAccelRight else 0

# Get time again for gloves after so many ticks
        timestamp = time.time() * 1000  # in milliseconds

        if accel_left >= punch_baseline and accel_left > accel_left_prev:
            self._lastAccelLeft = self.__limbs_data
        elif accel_left < accel_left_prev and accel_left_prev >= punch_baseline:
            if self._timeframeL is None or timestamp - self._timeframeL >= delay:
                # gl = Glove()
                # gl.time = timestamp
                # gl.glove = 'L'
                # gl.x = DashboardFragment.lastAccelLeftself.__limbs_data.x_acc_l()
                # gl.y = DashboardFragment.lastAccelLeftself.__limbs_data.y_acc_l
                # gl.z = DashboardFragment.lastAccelLeftself.__limbs_data.z_acc_l
                # gl.roll = DashboardFragment.lastAccelLeftself.__limbs_data.getRollL()
                # gl.pitch = DashboardFragment.lastAccelLeftself.__limbs_data.getPitchL()
                # registerAndInsertT3(gl.time)
                # insertAndTransfer(gl, EntityType.Glove)
                self._timeframeL = timestamp
                self.get_logger().info("Hit Left!")
                # DashboardFragment.flashLGlove(MainActivity.this, getResources().getColor(R.color.flashL, getTheme()))
            self._lastAccelLeft = self.__limbs_data

        if accel_right >= punch_baseline and accel_right > accel_right_prev:
            self._lastAccelRight = self.__limbs_data
        elif accel_right < accel_right_prev and accel_right_prev >= punch_baseline:
            if self._timeframeR is None or timestamp - self._timeframeR >= delay:
                # gl = Glove()
                # gl.time = timestamp
                # gl.glove = 'R'
                # gl.x = DashboardFragment.lastAccelRightself.__limbs_data.x_acc_r()
                # gl.y = DashboardFragment.lastAccelRightself.__limbs_data.y_acc_r
                # gl.z = DashboardFragment.lastAccelRightself.__limbs_data.z_acc_r
                # gl.roll = DashboardFragment.lastAccelRightself.__limbs_data.getRollR()
                # gl.pitch = DashboardFragment.lastAccelRightself.__limbs_data.getPitchR()
                # registerAndInsertT3(gl.time)
                # insertAndTransfer(gl, EntityType.Glove)
                self._timeframeR = timestamp
                self.get_logger().info("Hit Right!")
                # DashboardFragment.flashRGlove(MainActivity.this, getResources().getColor(R.color.flashR, getTheme()))
            self._lastAccelRight = self.__limbs_data

        # if DashboardFragment.seriesL is not None:
        #     DashboardFragment.seriesL.appendData(DataPoint(datetime.now(), accel_left), True, 60)
        # if DashboardFragment.seriesR is not None:
        #     DashboardFragment.seriesR.appendData(DataPoint(datetime.now(), accel_right), True, 60)

        # if homeText is not None:
        #     homeText.setText(f"{gloveTextToDash}")
        # if dashGraphL is not None:
        #     dashGraphL.onDataChange()
