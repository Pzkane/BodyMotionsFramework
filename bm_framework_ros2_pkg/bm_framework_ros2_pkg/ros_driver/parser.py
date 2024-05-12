import json
from dataclasses import dataclass

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
    center: object
    left: object
    right: object


class TorsoDataParser:
    def __init__(self):
        pass

    @staticmethod
    def deserialize(data: str) -> TorsoData:
        data_json = json.loads(data)
        if not "center" in data_json:
            raise AttributeError("Torso raw data does not have 'center' readings!")
        return TorsoData(
            data_json["center"],
            data_json["left"] if "left" in data_json else None,
            data_json["right"] if "right" in data_json else None
        )
