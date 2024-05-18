from dataclasses import dataclass
from bm_framework_interfaces_ros2_pkg.msg import Sensors


@dataclass
class Pose:
    name: str
    sensors: Sensors
