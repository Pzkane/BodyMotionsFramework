import math
import time
from rclpy.timer import Timer
from scipy.spatial import distance
from scipy.spatial.transform import Rotation
from typing import Any, Optional
from rclpy import List
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bm_framework_ros2_pkg.ros_driver.parser import CenterDataParser, LimbData, LimbDataParser, PeripheryDataParser, TorsoData
from bm_framework_interfaces_ros2_pkg.msg import Sensor, Sensors, StaticPoseResult
from bm_framework_interfaces_ros2_pkg.srv import GetSensors, InitiateStaticHold


class BodyMotionsServerNode(Node):
    def __init__(self, *args, **kwargs):
        super(BodyMotionsServerNode, self).__init__(node_name="bm_server", namespace="body_motions_framework", *args, **kwargs)
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.__torso_data = TorsoData(None, None, None)
        self.__limbs_data: Optional[LimbData] = None
        self.__static_pose: Sensors
        self.__static_pose_timer: Optional[Timer] = None

        self._handsDownRegistered: bool = False
        self._lastHandsDownTime: float
        self._lastAccelLeft: Optional[Any] = None
        self._lastAccelRight: Optional[Any] = None
        self._timeframeL: float = time.time()
        self._timeframeR: float = time.time()
        self.__init_services()
        self.__init_publishers()
        self.__init_relay_connection()
        self.__init_boards_connrections()

    def get_sensors(self) -> List[Sensor]:
        sensors: List[Sensor] = []
        if self.__limbs_data:
            sensors.append(Sensor(
                name="limb_left",
                x_acc=self.__limbs_data.x_acc_l,
                y_acc=self.__limbs_data.y_acc_l,
                z_acc=self.__limbs_data.z_acc_l,
                pitch=self.__limbs_data.pitch_l,
                roll=self.__limbs_data.roll_l,
                yaw=0.0
            ))
        else:
            sensors.append(Sensor(
                name="limb_left",
                x_acc=0.0,
                y_acc=0.0,
                z_acc=0.0,
                pitch=0.0,
                roll=0.0,
                yaw=0.0
            ))

        def _get_torso_sensor_msg(sensor: Any, name: str):
            if sensor is not None:
                rpy = Rotation.from_quat([sensor["qx"], sensor["qy"], sensor["qz"], sensor["qw"]]).as_euler("xyz")
                return Sensor(
                    name=name,
                    x_acc=sensor["x"],
                    y_acc=sensor["y"],
                    z_acc=sensor["z"],
                    roll=rpy[0],
                    pitch=rpy[1],
                    yaw=rpy[2]
                )
            else:
                return Sensor(
                    name=name,
                    x_acc=0.0,
                    y_acc=0.0,
                    z_acc=0.0,
                    pitch=0.0,
                    roll=0.0,
                    yaw=0.0
                )

        sensors.append(_get_torso_sensor_msg(self.__torso_data.left, "left"))
        sensors.append(_get_torso_sensor_msg(self.__torso_data.center, "center"))
        sensors.append(_get_torso_sensor_msg(self.__torso_data.right, "right"))

        if self.__limbs_data:
            sensors.append(Sensor(
                name="limb_right",
                x_acc=self.__limbs_data.x_acc_r,
                y_acc=self.__limbs_data.y_acc_r,
                z_acc=self.__limbs_data.z_acc_r,
                pitch=self.__limbs_data.pitch_r,
                roll=self.__limbs_data.roll_r,
                yaw=0.0
            ))
        else:
            sensors.append(Sensor(
                name="limb_right",
                x_acc=0.0,
                y_acc=0.0,
                z_acc=0.0,
                pitch=0.0,
                roll=0.0,
                yaw=0.0
            ))

        return sensors

    def hold_pose_and_publish_result(self, pose: Sensors):
        self.__static_pose = pose
        if self.__static_pose_timer:
            self.__static_pose_timer.reset()
        else:
            self.__static_pose_timer = self.create_timer(0.008, self.__cb_pub_static_pose_result)

    def __init_boards_connrections(self):
        self.create_subscription(String, "/body_motions_framework/limb_readings", self.__cb_limb_readings, 1)
        self.get_logger().info("Limb sensors connected!")

    def __init_relay_connection(self):
        self.create_subscription(String, "/body_motions_framework/mmr_ble", self.__cb_ble_data, 1)
        self.get_logger().info("BLE node connected!")
        self.create_subscription(String, "/body_motions_framework/mmr_relay", self.__cb_relay_data, 1)
        self.get_logger().info("Relay connected!")

    def __init_services(self):
        self.cli_get_sensors = self.create_service(GetSensors, "get_sensors", self.__cb_get_sensors)
        self.cli_initiate_static_hold = self.create_service(InitiateStaticHold, "initiate_static_hold", self.__cb_initiate_static_hold)
        self.cli_stop_static_hold = self.create_service(Trigger, "initiate_static_hold", self.__cb_stop_static_pose_processing)

    def __init_publishers(self):
        self.pub_static_pose_hold_result = self.create_publisher(StaticPoseResult, "static_pose_result", 1)

    def __cb_ble_data(self, ble_meta_motion_r_data: String):
        data = CenterDataParser.deserialize(ble_meta_motion_r_data.data)
        self.__torso_data.center = data.center

    def __cb_relay_data(self, relay_meta_motion_r_data: String):
        data = PeripheryDataParser.deserialize(relay_meta_motion_r_data.data)
        self.__torso_data.left = data.left
        self.__torso_data.right = data.right

    def __cb_limb_readings(self, nodemcu_data: String):
        try:
            self.__limbs_data = LimbDataParser.deserialize(nodemcu_data.data)
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
                _timeframeL = timestamp
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

    def __cb_get_sensors(self, _: GetSensors.Request, response: GetSensors.Response) -> GetSensors.Response:
        response.success = True
        response.message = "Retrieved all sensors"
        response.sensors = self.get_sensors()
        return response

    def __cb_initiate_static_hold(self, request: InitiateStaticHold.Request, response: InitiateStaticHold.Response) -> InitiateStaticHold.Response:
        self.hold_pose_and_publish_result(request.target_hold_pose)
        response.success = True
        response.message = "Initiated pose hold"
        return response

    def __cb_pub_static_pose_result(self):
        sensors: List[Sensor] = self.get_sensors()
        is_aligned: bool = True
        pose_diff: List[Sensor] = []
        # raise Exception(f"\n{str(self.__static_pose.sensors)}\n{str(sensors)}")
        for idx, sensor in enumerate(sensors):
            roll_diff = distance.euclidean([sensor.roll,0,0], [self.__static_pose.sensors[idx].roll,0,0])
            pitch_diff = distance.euclidean([sensor.pitch,0,0], [self.__static_pose.sensors[idx].pitch,0,0])
            yaw_diff = distance.euclidean([sensor.yaw,0,0], [self.__static_pose.sensors[idx].yaw,0,0])
            if abs(roll_diff) > 0.2:
                is_aligned = False
            if abs(pitch_diff) > 0.2:
                is_aligned = False
            if abs(yaw_diff) > 0.2:
                is_aligned = False
            # self.get_logger().warn(f"{str(is_aligned)}, {str(roll_diff)}, {str(pitch_diff)}, {str(yaw_diff)};")

            # Will calculate lin acc just in case
            pose_diff.append(Sensor(
                name=sensor.name,
                x_acc=distance.euclidean([sensor.x_acc,0,0], [self.__static_pose.sensors[idx].x_acc,0,0]),
                y_acc=distance.euclidean([sensor.y_acc,0,0], [self.__static_pose.sensors[idx].y_acc,0,0]),
                z_acc=distance.euclidean([sensor.z_acc,0,0], [self.__static_pose.sensors[idx].z_acc,0,0]),
                pitch=pitch_diff,
                roll=roll_diff,
                yaw=yaw_diff
            ))

        msg = StaticPoseResult(
            pose_diff=Sensors(sensors=pose_diff),
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
