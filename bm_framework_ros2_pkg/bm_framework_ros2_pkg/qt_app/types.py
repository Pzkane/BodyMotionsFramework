from dataclasses import dataclass
from enum import Enum, auto
from bm_framework_interfaces_ros2_pkg.msg import Sensors


class PoseType(Enum):
    STATIC = auto()
    IMPULSE = auto()


@dataclass
class Pose:
    name: str
    type: PoseType
    sensors: Sensors
