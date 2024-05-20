from PySide2.QtCore import QObject, Signal
from bm_framework_interfaces_ros2_pkg.msg import Sensors
from bm_framework_ros2_pkg.qt_app.types import PoseType


class GuiSignals(QObject):
    create_impulse_pose = Signal(str, PoseType, Sensors)
    finish_impulse = Signal()
    stop_static_pose = Signal()
    update_static_pose_status = Signal(bool, Sensors)
