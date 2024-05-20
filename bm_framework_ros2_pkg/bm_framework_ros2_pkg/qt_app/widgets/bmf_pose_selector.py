from typing import List
from PySide2.QtWidgets import QWidget
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode
from bm_framework_ros2_pkg.qt_app.signals import GuiSignals
from bm_framework_ros2_pkg.qt_app.types import Pose
from bm_framework_ros2_pkg.qt_app.ui_sources import pose_selector_ui


class PoseSelector(QWidget):
    def __init__(self, gui_signals: GuiSignals, node: BMApplicationNode, parent=None):
        super(PoseSelector, self).__init__(parent)
        self.gui_signals = gui_signals
        self.node = node
        self.ui = pose_selector_ui.Ui_Form()
        self.ui.setupUi(self)
        self.ui.comboBox.currentTextChanged.connect(self.__cb_update_selected_pose)

    def set_poses(self, poses: List[Pose]):
        self.ui.comboBox.clear()
        for pose in poses:
            self.ui.comboBox.addItem(pose.name)

    def __cb_update_selected_pose(self):
        pose_name = self.ui.comboBox.currentText()
        if pose_name:
            self.node.set_current_pose(self.ui.comboBox.currentText())
