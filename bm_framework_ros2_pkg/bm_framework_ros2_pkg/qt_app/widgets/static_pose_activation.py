
from PySide2.QtWidgets import QWidget
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode
from bm_framework_ros2_pkg.qt_app.signals import GuiSignals
from bm_framework_ros2_pkg.qt_app.ui_sources import static_pose_activation_widget_ui
from bm_framework_ros2_pkg.qt_app.widgets.bmf_pose_selector import PoseSelector
from bm_framework_interfaces_ros2_pkg.msg import Sensors


class StaticPoseActivation(QWidget):
    def __init__(self, gui_signals: GuiSignals, node: BMApplicationNode, parent=None):
        super(StaticPoseActivation, self).__init__(parent)
        self.ui = static_pose_activation_widget_ui.Ui_Form()
        self.ui.setupUi(self)
        self.gui_signals= gui_signals
        self.node = node
        self.__activity_started: bool = False
        self.__init_signals()
        self.__init_layout()
        self.__init_controls()

    def __init_signals(self):
        self.gui_signals.update_static_pose_status.connect(self.__cb_display_static_pose_status)
        self.gui_signals.stop_static_pose.connect(self.__cb_stop_static_pose)

    def __init_controls(self):
        self.ui.pushButton.clicked.connect(self.__cb_start_pose_activity)

    def __init_layout(self):
        self.pose_selector = PoseSelector(self.gui_signals, self.node, self)
        self.ui.verticalLayout.insertWidget(0, self.pose_selector)

    def __cb_start_pose_activity(self):
        if not self.__activity_started:
            self.ui.pushButton.setText("Finish Pose Activity")
            # Send target pose to server
            self.node.initiate_static_hold()
        else:
            self.ui.pushButton.setText("Start Pose Activity")
            # Unsub from results
            self.node.finish_static_hold()
            pass
        self.__activity_started = not self.__activity_started

    def __cb_display_static_pose_status(self, is_aligned: bool, pose_diff: Sensors):
        text: str = ""
        text += f"Aligned: {str(is_aligned)}"
        for pose in pose_diff.sensors:
            text += f"""
            {str(pose.name)}: {str(pose.roll)}, {str(pose.pitch)}, {str(pose.yaw)}
            """
        self.ui.plainTextEdit.setPlainText(text)

    def __cb_stop_static_pose(self):
        self.ui.plainTextEdit.setPlainText("")
