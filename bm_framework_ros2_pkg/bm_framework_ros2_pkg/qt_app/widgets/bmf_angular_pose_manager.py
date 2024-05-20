from math import sqrt
from threading import RLock
from typing import Dict, List
from PySide2.QtGui import QColor
from PySide2.QtWidgets import QTabWidget, QTableWidgetItem, QWidget
from rclpy.task import Future
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode

from bm_framework_interfaces_ros2_pkg.srv import GetSensors
from bm_framework_ros2_pkg.qt_app.signals import GuiSignals
from bm_framework_ros2_pkg.qt_app.types import Pose, PoseType
from bm_framework_ros2_pkg.qt_app.ui_sources import angular_pose_manager_ui
from bm_framework_interfaces_ros2_pkg.msg import Sensors, Sensor
from bm_framework_ros2_pkg.qt_app.widgets.static_pose_activation import StaticPoseActivation


class BMFAngularPoseManager(QWidget):
    def __init__(self, gui_signals: GuiSignals, node: BMApplicationNode, parent=None):
        super(BMFAngularPoseManager, self).__init__(parent)
        self.gui_signals = gui_signals
        self.node = node
        self.ui = angular_pose_manager_ui.Ui_Form()
        self.ui.setupUi(self)
        self.__sensor_tables = [
            self.ui.tableWidget_limb_left,
            self.ui.tableWidget_left,
            self.ui.tableWidget_center,
            self.ui.tableWidget_right,
            self.ui.tableWidget_limb_right
        ]
        self.__impulse_lock = RLock()
        self.__impulse_started: bool = False
        self.__poses: List[Pose] = []
        self.__init_signals()
        self.__init_tables()
        self.__init_layout()
        self.__init_controls()

    def __init_signals(self):
        self.gui_signals.create_impulse_pose.connect(self.__insert_impulse_pose)
        self.gui_signals.finish_impulse.connect(self.__finish_impulse_activity)

    def __init_controls(self):
        self.ui.pushButton_create_pose.clicked.connect(self.__create_pose)
        self.ui.radioButton_impulse.toggled.connect(self.__manage_pose_type_layout)

    def __init_layout(self):
        self.tab_widget = QTabWidget()
        self.static_pose_activation = StaticPoseActivation(self.gui_signals, self.node)
        self.tab_widget.addTab(self.static_pose_activation, "Static Pose")
        self.ui.verticalLayout.addWidget(self.tab_widget)

    def __init_tables(self):
        self.__update_pose_table()

    def __create_pose(self):
        for pose in self.__poses:
            if pose.name == self.ui.lineEdit_pose_name.text():
                self.node.get_logger().error("Pose with such name already exists!")
                return
        if self.ui.radioButton_static.isChecked():
            self.node.get_sensor_readings(self.__cb_sensor_readings_received)
            return
        if self.ui.radioButton_impulse.isChecked():
            if self.__impulse_started:
                self.node.get_logger().error("Impulse set already in progress!")
                return

            if not self.__impulse_started:
                self.ui.label_impulse_status.setText("Waiting...")
                # Send target pose to server
                impulse_target_sensor: str
                if self.ui.radioButton_limb_left_imp.isChecked():
                    impulse_target_sensor = "limb_left"
                elif self.ui.radioButton_left_imp.isChecked():
                    impulse_target_sensor = "left"
                elif self.ui.radioButton_center_imp.isChecked():
                    impulse_target_sensor = "center"
                elif self.ui.radioButton_right_imp.isChecked():
                    impulse_target_sensor = "right"
                elif self.ui.radioButton_limb_right_imp.isChecked():
                    impulse_target_sensor = "limb_right"
                else:
                    raise RuntimeError("None of target impulse radio buttons is checked!")
                active_impulse_sensors: Dict[str, bool] = {
                    "limb_left": self.ui.checkBox_limb_left.isChecked(),
                    "left": self.ui.checkBox_left.isChecked(),
                    "center": self.ui.checkBox_center.isChecked(),
                    "right": self.ui.checkBox_right.isChecked(),
                    "limb_right": self.ui.checkBox_limb_right.isChecked()
                }
                self.node.initiate_impulse(self.ui.lineEdit_pose_name.text(), impulse_target_sensor, active_impulse_sensors)
            else:
                self.ui.label_impulse_status.setText("")
                # Unsub from results
                self.node.finish_impulse()
                pass

    def __insert_impulse_pose(self, pose_name: str, pose_type: PoseType, data: Sensors):
        # Workaround for bug which posts several messages of same result
        with self.__impulse_lock:
            pose_names = set(pose.name for pose in self.__poses)
            if pose_name in pose_names:
                return
        self.__insert_new_pose(pose_name, pose_type, data)
        self.ui.label_impulse_status.setText("Received")

    def __insert_new_pose(self, pose_name: str, pose_type: PoseType, data: Sensors):
        sensors: List[Sensor] = data.sensors
        self.__set_pose(Pose(
            name=pose_name,
            type=pose_type,
            sensors=sensors
        ))

    def __finish_impulse_activity(self):
        self.__impulse_started = False

    def __update_pose_table(self):
        NUM_OF_COL_SENSORS: int = 4
        NUM_OF_COL_METADATA: int = 2
        self.ui.tableWidget_metadata.clear()
        self.ui.tableWidget_metadata.setColumnCount(NUM_OF_COL_SENSORS)
        self.ui.tableWidget_metadata.setHorizontalHeaderLabels(
            ["Name", "Type"]
        )
        self.ui.tableWidget_metadata.setRowCount(len(self.__poses))
        for row, pose in enumerate(self.__poses):
            for col in range(NUM_OF_COL_METADATA):
                item = QTableWidgetItem()
                if col == 0:
                    item.setText(pose.name)
                elif col == 1:
                    item.setText(pose.type.name)
                else:
                    raise Exception("Wrong number of columns fro metadata!")
                self.ui.tableWidget_metadata.setItem(row, col, item)

        for idx, table in enumerate(self.__sensor_tables):
            table.clear()
            table.setColumnCount(NUM_OF_COL_SENSORS)
            table.setHorizontalHeaderLabels(
                ["Roll", "Pitch", "Yaw", "G-Force"]
            )
            table.setRowCount(len(self.__poses))

            for row, pose in enumerate(self.__poses):
                for col in range(NUM_OF_COL_SENSORS):
                    item = QTableWidgetItem()
                    if not pose.sensors[idx].orientation_active:
                        item.setBackgroundColor(QColor(100,100,100,64))
                    if pose.sensors[idx].orientation_reference:
                        item.setBackgroundColor(QColor(0,100,0,64))
                    if pose.sensors[idx].target_impulse:
                        item.setBackgroundColor(QColor(255,255,0,64))
                  
                    # Same correction for frontend, ugly
                    g: float = sqrt(pose.sensors[idx].x_acc**2 + pose.sensors[idx].y_acc**2 + pose.sensors[idx].z_acc**2)
                    if pose.sensors[idx].name == "limb_left":
                        g -= 0.990
                    if pose.sensors[idx].name == "limb_right":
                        g -= 0.980
                    if pose.sensors[idx].name == "center":
                        g -= 1

                    if col == 0:
                        item.setText(str(round(pose.sensors[idx].roll, 3)))
                    elif col == 1:
                        item.setText(str(round(pose.sensors[idx].pitch, 3)))
                    elif col == 2:
                        item.setText(str(round(pose.sensors[idx].yaw, 3)))
                    elif col == 3:
                        item.setText(str(round(abs(g), 3)))
                    else:
                        raise Exception("Wrong number of columns for sensors!")
                    table.setItem(row, col, item)

    def __manage_pose_type_layout(self):
        if self.ui.radioButton_impulse.isChecked():
            self.ui.widget_imp_target_sensor.setEnabled(True)
        else:
            self.ui.widget_imp_target_sensor.setDisabled(True)

    def __set_pose(self, pose: Pose):
        self.__poses.append(pose)
        self.node.set_poses(self.__poses)
        self.__update_pose_table()
        self.__update_child_widgets()

    def __update_child_widgets(self):
        self.static_pose_activation.pose_selector.set_poses(self.__poses)

    def __cb_sensor_readings_received(self, future: Future):
        data = future.result()
        if not isinstance(data, GetSensors.Response):
            self.node.get_logger().error(f"Error occured during pose creation: {str(data)}")
            return
        if self.ui.lineEdit_pose_name.text() == "":
            self.node.get_logger().error("Pose name was not provided!")
            return
        sensors: Sensors = data.sensors
        for sensor in sensors:
            if sensor.name == "limb_left"  and not self.ui.checkBox_limb_left.isChecked() or\
               sensor.name == "left"       and not self.ui.checkBox_left.isChecked() or\
               sensor.name == "center"     and not self.ui.checkBox_center.isChecked() or\
               sensor.name == "right"      and not self.ui.checkBox_right.isChecked() or\
               sensor.name == "limb_right" and not self.ui.checkBox_limb_right.isChecked():
                sensor.orientation_active = False

            if sensor.name == "limb_left"  and self.ui.radioButton_limb_left.isChecked() or\
               sensor.name == "left"       and self.ui.radioButton_left.isChecked() or\
               sensor.name == "center"     and self.ui.radioButton_center.isChecked() or\
               sensor.name == "right"      and self.ui.radioButton_right.isChecked() or\
               sensor.name == "limb_right" and self.ui.radioButton_limb_right.isChecked():
                sensor.orientation_reference = True

            if not sensor.orientation_active and sensor.orientation_reference:
                self.node.get_logger().error("Disabled pose cannot act as a reference point!")
                return

        pose_type = PoseType.STATIC
        if self.ui.radioButton_impulse.isChecked():
            pose_type = PoseType.IMPULSE

        self.__set_pose(Pose(
            name=self.ui.lineEdit_pose_name.text(),
            type=pose_type,
            sensors=sensors
        ))
