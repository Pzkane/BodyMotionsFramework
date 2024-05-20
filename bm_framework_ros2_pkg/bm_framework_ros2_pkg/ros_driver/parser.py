import json
from dataclasses import dataclass
from typing import Any

@dataclass
class LimbData:
    x_acc_l: float
    y_acc_l: float
    z_acc_l: float

    x_acc_r: float
    y_acc_r: float
    z_acc_r: float

    roll_l: float
    pitch_l: float

    roll_r: float
    pitch_r: float

    orientation_active_l: bool
    orientation_reference_l: bool

    orientation_active_r: bool
    orientation_reference_r: bool

    target_impulse_l: bool
    target_impulse_r: bool

class LimbDataParser:
    def __init__(self):
        pass

    @staticmethod
    def deserialize(data: str) -> LimbData:
        tokens = data.split(':')
        if len(tokens) < 11:
            raise AttributeError("Data is not full")
        return LimbData(
            float(int(tokens[1])/1000),
            float(int(tokens[2])/1000),
            float(int(tokens[3])/1000),
            float(int(tokens[4])/1000),
            float(int(tokens[5])/1000),
            float(int(tokens[6])/1000),
            float(tokens[7]),
            float(tokens[8]),
            float(tokens[9]),
            float(tokens[10]),
            True,
            False,
            True,
            False,
            False,
            False,
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
        data_json["left"]["orientation_active"] = True
        data_json["left"]["orientation_reference"] = False
        data_json["left"]["target_impulse"] = False
        data_json["right"]["orientation_active"] = True
        data_json["right"]["orientation_reference"] = False
        data_json["right"]["target_impulse"] = False
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
        data_obj["qw"] = float(rot[0])
        data_obj["qx"] = float(rot[1])
        data_obj["qy"] = float(rot[2])
        data_obj["qz"] = float(rot[3])
        data_obj["x"] = float(acc[0])
        data_obj["y"] = float(acc[1])
        data_obj["z"] = float(acc[2])
        data_obj["orientation_active"] = True
        data_obj["orientation_reference"] = False
        data_obj["target_impulse"] = False

        return TorsoData(
            data_obj,
            None,
            None
        )
