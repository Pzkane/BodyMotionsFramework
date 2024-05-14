from rclpy.node import Node
from time import sleep
from mbientlab.metawear import MetaWear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.metawear.metawear import libmetawear
from mbientlab.warble import *
import faulthandler

from std_msgs.msg import String

faulthandler.enable()

FnVoid_Str_VoidP_DataP = CFUNCTYPE(None, c_char_p, c_void_p, POINTER(Data))


class MWSensorNode(Node):
    def __init__(self, *args, **kwargs):
        super(MWSensorNode, self).__init__(node_name="bm_metawear", namespace="body_motions_framework", *args, **kwargs)
        self.address = "C4:8B:49:73:F6:09"
        self.c_rot_data_handler = FnVoid_VoidP_DataP(self.__cb_rot_data_handler)
        self.c_acc_data_handler = FnVoid_VoidP_DataP(self.__cb_acc_data_handler)
        self.__acc = CartesianFloat()
        self.__init_publishers()
        self.__run()

    def __init_publishers(self):
        self.__pub_mmr_ble = self.create_publisher(String, "mmr_ble", 1)
        self.get_logger().info("Publishers inited!")

    def __run(self):
        addresses = [self.address]

        devices = [MetaWear(address) for address in addresses]
        for device in devices:
            device.connect()
            sleep(3)
            board = device.board

            # Setup the accelerometer sample frequency and range
            libmetawear.mbl_mw_sensor_fusion_set_mode(board, SensorFusionMode.NDOF)
            libmetawear.mbl_mw_sensor_fusion_set_acc_range(board, SensorFusionAccRange._16G)
            libmetawear.mbl_mw_sensor_fusion_set_gyro_range(board, SensorFusionGyroRange._1000DPS)
            libmetawear.mbl_mw_sensor_fusion_write_config(board)
            # Get the accelerometer data signal
            acc_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(board, SensorFusionData.LINEAR_ACC)
            rot_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(board, SensorFusionData.QUATERNION, SensorFusionData.LINEAR_ACC)
            # Subscribe to it
            libmetawear.mbl_mw_datasignal_subscribe(acc_signal, None, self.c_acc_data_handler)
            libmetawear.mbl_mw_datasignal_subscribe(rot_signal, None, self.c_rot_data_handler)

            # Enable the accelerometer
            libmetawear.mbl_mw_sensor_fusion_enable_data(board, SensorFusionData.LINEAR_ACC)
            libmetawear.mbl_mw_sensor_fusion_enable_data(board, SensorFusionData.QUATERNION)
            libmetawear.mbl_mw_sensor_fusion_start(board)

            self.get_logger().info("Sensor fusion in progress!")

        while True:
            pass

    def __cb_acc_data_handler(self, ctx, data):
        self.__acc = parse_value(data)
   
    def __cb_rot_data_handler(self, ctx, data):
        quat: Quaternion = parse_value(data)
        msg = String()
        msg.data = "%s:%s:%s|%s:%s:%s:%s" % (self.__acc.x, self.__acc.y, self.__acc.z, quat.w, quat.x, quat.y, quat.z)
        self.__pub_mmr_ble.publish(msg)
