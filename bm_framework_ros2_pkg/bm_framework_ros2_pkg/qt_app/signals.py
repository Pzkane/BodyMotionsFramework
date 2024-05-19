from PySide2.QtCore import QObject, Signal
from bm_framework_interfaces_ros2_pkg.msg import Sensors


class GuiSignals(QObject):
    stop_static_pose = Signal()
    update_static_pose_status = Signal(bool, Sensors)
