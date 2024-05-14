import json
from dataclasses import dataclass
from typing import Any

@dataclass
class LimbData:
    x_acc_l: int
    y_acc_l: int
    z_acc_l: int

    x_acc_r: int
    y_acc_r: int
    z_acc_r: int

    roll_l: float
    pitch_l: float

    roll_r: float
    pitch_r: float


class LimbDataParser:
    def __init__(self):
        pass

    @staticmethod
    def deserialize(data: str) -> LimbData:
        tokens = data.split(':')
        if len(tokens) < 11:
            raise AttributeError("Data is not full")
        return LimbData(
            int(tokens[1]),
            int(tokens[2]),
            int(tokens[3]),
            int(tokens[4]),
            int(tokens[5]),
            int(tokens[6]),
            float(tokens[7]),
            float(tokens[8]),
            float(tokens[9]),
            float(tokens[10])
        )


@dataclass
class TorsoData:
    center: Any
    left: Any
    right: Any


class PeripheryDataParser:
    def __init__(self):
        pass

    @staticmethod
    def deserialize(data: str) -> TorsoData:
        data_json = json.loads(data)
        return TorsoData(
            None,
            data_json["left"],
            data_json["right"]
        )


class CenterDataParser:
    def __init__(self):
        pass

    @staticmethod
    def deserialize(data: str) -> TorsoData:
        acc, rot = data.split('|')
        acc = acc.split(':')
        rot = rot.split(':')
        data_obj = {}
        data_obj["qw"] = rot[0]
        data_obj["qx"] = rot[1]
        data_obj["qy"] = rot[2]
        data_obj["qz"] = rot[3]
        data_obj["x"] = acc[0]
        data_obj["y"] = acc[1]
        data_obj["z"] = acc[2]
        return TorsoData(
            data_obj,
            None,
            None
        )
