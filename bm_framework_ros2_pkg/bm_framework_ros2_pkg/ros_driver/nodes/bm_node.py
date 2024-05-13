import datetime, time
from typing import Optional
from rclpy.logging import LoggingSeverity
from rclpy.node import Any, Node
from std_msgs.msg import String

from bm_framework_ros2_pkg.ros_driver.parser import LimbData, LimbDataParser, TorsoData, TorsoDataParser


class BodyMotionsServerNode(Node):
    def __init__(self, *args, **kwargs):
        super(BodyMotionsServerNode, self).__init__(node_name="bm_server", namespace="body_motions_framework", *args, **kwargs)
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.__torso_data = TorsoData(None, None, None)
        self._handsDownRegistered: bool = False
        self._lastHandsDownTime: float
        self._lastAccelLeft: Optional[Any] = None
        self._lastAccelRight: Optional[Any] = None
        self._timeframeL: float = time.time()
        self._timeframeR: float = time.time()
        self.__init_relay_connection()
        self.__init_boards_connrections()

    def __init_boards_connrections(self):
        self.create_subscription(String, "/body_motions_framework/limb_readings", self.__cb_limb_readings, 1)
        self.get_logger().info("Limb sensors connected!")

    def __init_relay_connection(self):
        self.create_subscription(String, "/body_motions_framework/mmr_relay", self.__cb_relay_data, 1)
        self.get_logger().info("Relay connected!")


    def __cb_relay_data(self, meta_motion_r_data: String):
        self.__torso_data = TorsoDataParser.deserialize(meta_motion_r_data.data)

    def __cb_limb_readings(self, nodemcu_data: String):
        try:
            self.__limbs_data: LimbData = LimbDataParser.deserialize(nodemcu_data.data)
        except AttributeError:
            # Skip packet
            return

        if self.__torso_data.center is None:
            self.get_logger().warn("empty")
            return
        # self.get_logger().info(f"Torso: {str(self.__torso_data)}, limbs: {str(self.__limbs_data)}")
        
        rPitch: float = round(((self.__limbs_data.pitch_r-8) * 100.0) / 100.0)
        lPitch: float = round(((self.__limbs_data.pitch_l-8) * 100.0) / 100.0)
        gloveTextToDash: str = f"{str(-1 * round((self.__limbs_data.roll_l+10) * 100.0) / 100.0)}\t\t{str(self.__limbs_data.roll_r)}"
        gloveTextToDash += "\n" + str(lPitch) + "\t\t" + str(rPitch)

        threshold = 5.0
        functionR: float = round( ((25.7143 - 0.714286 * rPitch) * 100.0) / 100.0)
        functionL: float = round( ((25.7143 - 0.714286 * lPitch) * 100.0) / 100.0)

        gloveTextToDash += "\n" + str(-1 * round((self.__torso_data.center["qw"] * 100) * 100.0) / 100.0) + "\t\t"
        gloveTextToDash += str(-1 * round((self.__torso_data.center["qx"] * 100) * 100.0) / 100.0) + "\t\t"
        gloveTextToDash += str(-1 * round((self.__torso_data.center["qy"] * 100) * 100.0) / 100.0) + "\t\t"
        gloveTextToDash += str(-1 * round((self.__torso_data.center["qz"] * 100) * 100.0) / 100.0)
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
                self.get_logger().info("LOW HANDS\n" + str(functionL) + ":" + str(functionR))
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
