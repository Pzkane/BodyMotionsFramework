from typing import List
from PySide2.QtWidgets import QTableWidgetItem, QWidget
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode

from bm_framework_ros2_pkg.qt_app.signals import GuiSignals
from bm_framework_ros2_pkg.qt_app.types import Pose
from bm_framework_ros2_pkg.qt_app.ui_sources import pose_chaining_widget_ui


class PoseChainingWidget(QWidget):
    def __init__(self, gui_signals: GuiSignals, node: BMApplicationNode, parent=None):
        super(PoseChainingWidget, self).__init__(parent)
        self.ui = pose_chaining_widget_ui.Ui_Form()
        self.ui.setupUi(self)
        self.node = node
        self.gui_signals = gui_signals
        self.__traversable_poses: List[Pose] = []
        self.__current_pose_id: int = 0
        self.STATUS_COL: int = 2
        self.RESPONSE_COL: int = 3
        self.__init_layout()
        self.__init_signals()
        self.__init_constrols()

    def __init_constrols(self):
        self.ui.comboBox.currentTextChanged.connect(self.__cb_update_selected_pose)
        self.ui.pushButton_start.clicked.connect(self.__cb_start_pose_traversal)
        self.ui.pushButton_clear.clicked.connect(self.__cb_clear_pose_plan)
        self.ui.pushButton_add_pose.clicked.connect(self.__cb_add_traversal_pose)

    def __init_layout(self):
        self.__update_table()

    def __init_signals(self):
        self.gui_signals.update_pose_planner_response.connect(self.__cb_update_pose_planner_response)
        self.gui_signals.iterate_next_pose.connect(self.__cb_iterate_next_pose)
        self.gui_signals.stop_plan_execution.connect(self.__cb_stop_plan_execution)

    def __update_table(self):
        NUM_OF_COLS: int = 4
        self.ui.tableWidget.clear()
        self.ui.tableWidget.setColumnCount(NUM_OF_COLS)
        self.ui.tableWidget.setRowCount(len(self.__traversable_poses))
        self.ui.tableWidget.setHorizontalHeaderLabels(
            ["Name", "Type", "Status", "Response"]
        )
        self.ui.tableWidget.setRowCount(len(self.__traversable_poses))

        for row, pose in enumerate(self.__traversable_poses):
            for col in range(NUM_OF_COLS):
                item = QTableWidgetItem()
                if col == 0:
                    item.setText(pose.name)
                elif col == 1:
                    item.setText(pose.type.name)
                elif col == 2:
                    if row == 0:
                        item.setText("Current")
                    else:
                        item.setText("")
                elif col == 3:
                    item.setText("")
                else:
                    raise Exception("Wrong number of columns for traversable pose!")
                self.ui.tableWidget.setItem(row, col, item)

    def set_poses(self, poses: List[Pose]):
        self.ui.comboBox.clear()
        for pose in poses:
            self.ui.comboBox.addItem(pose.name)

    def __cb_update_pose_planner_response(self, row_id: int, status: str, planner_response: str):
        row_count = self.ui.tableWidget.rowCount()
        if row_id > row_count:
            row_id = row_count - 1
        self.ui.tableWidget.item(row_id, self.STATUS_COL).setText(status)
        self.ui.tableWidget.item(row_id, self.RESPONSE_COL).setText(planner_response)

    def __cb_update_selected_pose(self):
        pose_name = self.ui.comboBox.currentText()
        if pose_name:
            self.node.set_current_pose(self.ui.comboBox.currentText())

    def __cb_add_traversal_pose(self):
        pose_name = self.ui.comboBox.currentText()
        if len(self.__traversable_poses) > 0 and pose_name == self.__traversable_poses[-1].name:
            self.node.get_logger().error("Cannot add two identical poses one after another!")
            return
        self.__traversable_poses.append(self.node.poses[pose_name])
        self.__update_table()

    def __cb_clear_pose_plan(self):
        self.__traversable_poses.clear()
        self.ui.tableWidget.clear()
        self.ui.tableWidget.setColumnCount(0)
        self.ui.tableWidget.setRowCount(0)

    def __cb_iterate_next_pose(self):
        self.ui.tableWidget.item(self.__current_pose_id, self.STATUS_COL).setText("Done")
        self.__current_pose_id += 1
        if self.__current_pose_id < self.ui.tableWidget.rowCount():
            self.ui.tableWidget.item(self.__current_pose_id, self.STATUS_COL).setText("Current")

    def __cb_start_pose_traversal(self):
        self.ui.pushButton_clear.setDisabled(True)
        self.ui.pushButton_start.setDisabled(True)
        self.ui.pushButton_add_pose.setDisabled(True)
        # Send pose list to server
        self.node.execute_pose_plan(self.__traversable_poses)

    def __cb_stop_plan_execution(self):
        self.ui.tableWidget.item(self.__current_pose_id-1, self.STATUS_COL).setText("Finished")
        self.ui.pushButton_clear.setEnabled(True)
        self.ui.pushButton_start.setEnabled(True)
        self.ui.pushButton_add_pose.setEnabled(True)
        self.__current_pose_id = 0
