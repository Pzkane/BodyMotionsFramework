from typing import List
from PySide2.QtWidgets import QTabWidget, QTableWidgetItem, QWidget
from rclpy.task import Future
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode

from bm_framework_interfaces_ros2_pkg.srv import GetSensors
from bm_framework_ros2_pkg.qt_app.types import Pose
from bm_framework_ros2_pkg.qt_app.ui_sources import angular_pose_manager_ui
from bm_framework_interfaces_ros2_pkg.msg import Sensors
from bm_framework_ros2_pkg.qt_app.widgets.static_pose_activation import StaticPoseActivation


class BMFAngularPoseManager(QWidget):
    def __init__(self, node: BMApplicationNode, parent=None):
        super(BMFAngularPoseManager, self).__init__(parent)
        self.node = node
        self.ui = angular_pose_manager_ui.Ui_Form()
        self.ui.setupUi(self)
        self.__tables = [
            self.ui.tableWidget_limb_left,
            self.ui.tableWidget_left,
            self.ui.tableWidget_center,
            self.ui.tableWidget_right,
            self.ui.tableWidget_limb_right
        ]
        self.__poses: List[Pose] = []
        self.__init_tables()
        self.__init_layout()
        self.__init_controls()

    def __init_controls(self):
        self.ui.pushButton_create_pose.clicked.connect(self.__create_pose)

    def __init_layout(self):
        self.tab_widget = QTabWidget()
        self.static_pose_activation = StaticPoseActivation(self.node)
        self.tab_widget.addTab(self.static_pose_activation, "Static Pose")
        self.ui.verticalLayout.addWidget(self.tab_widget)

    def __init_tables(self):
        self.__update_pose_table()

    def __create_pose(self):
        self.node.get_sensor_readings(self.__cb_sensor_readings_received)

    def __update_pose_table(self):
        START_NUM_OF_COL: int = 4
        NUM_OF_COL: int = 3
        for idx, table in enumerate(self.__tables):
            table.clear()
            if idx == 0:
                table.setColumnCount(START_NUM_OF_COL)
                table.setHorizontalHeaderLabels(
                    ["Name", "Roll", "Pitch", "Yaw"]
                )
            else:
                table.setColumnCount(NUM_OF_COL)
                table.setHorizontalHeaderLabels(
                    ["Roll", "Pitch", "Yaw"]
                )
            table.setRowCount(len(self.__poses))

            for row, pose in enumerate(self.__poses):
                if idx == 0:
                    for col in range(START_NUM_OF_COL):
                        item = QTableWidgetItem()
                        if col == 0:
                            item.setText(pose.name)
                        elif col == 1:
                            item.setText(str(pose.sensors[idx].roll))
                        elif col == 2:
                            item.setText(str(pose.sensors[idx].pitch))
                        elif col == 3:
                            item.setText(str(pose.sensors[idx].yaw))
                        else:
                            raise Exception("Wrong number of columns for the first table!")
                        table.setItem(row, col, item)
                else:
                    for col in range(NUM_OF_COL):
                        item = QTableWidgetItem()
                        if col == 0:
                            item.setText(str(pose.sensors[idx].roll))
                        elif col == 1:
                            item.setText(str(pose.sensors[idx].pitch))
                        elif col == 2:
                            item.setText(str(pose.sensors[idx].yaw))
                        else:
                            raise Exception("Wrong number of columns!")
                        table.setItem(row, col, item)

    def __update_child_widgets(self):
        self.static_pose_activation.pose_selector.set_poses(self.__poses)

    def __cb_sensor_readings_received(self, future: Future):
        data = future.result()
        if not isinstance(data, GetSensors.Response):
            self.node.get_logger().error(f"Error occured during pose creation: {str(data)}")
            return
        sensors: Sensors = data.sensors
        # Order should always be:
        # limb left, left, center, right, limb right
        if self.ui.lineEdit_pose_name.text() == "":
            self.node.get_logger().error("Pose name was not provided!")
            return
        self.__poses.append(Pose(
            name=self.ui.lineEdit_pose_name.text(),
            sensors=sensors
        ))
        self.node.set_poses(self.__poses)
        self.__update_pose_table()
        self.__update_child_widgets()
