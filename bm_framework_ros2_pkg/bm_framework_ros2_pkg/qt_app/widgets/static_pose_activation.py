
from PySide2.QtWidgets import QWidget
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode
from bm_framework_ros2_pkg.qt_app.ui_sources import static_pose_activation_widget_ui
from bm_framework_ros2_pkg.qt_app.widgets.bmf_pose_selector import PoseSelector


class StaticPoseActivation(QWidget):
    def __init__(self, node: BMApplicationNode, parent=None):
        super(StaticPoseActivation, self).__init__(parent)
        self.ui = static_pose_activation_widget_ui.Ui_Form()
        self.ui.setupUi(self)
        self.node = node
        self.__activity_started: bool = False
        self.__init_layout()
        self.__init_controls()

    def __init_controls(self):
        self.ui.pushButton.clicked.connect(self.__cb_start_pose_activity)

    def __init_layout(self):
        self.pose_selector = PoseSelector(self.node, self)
        self.ui.verticalLayout.insertWidget(0, self.pose_selector)

    def __cb_start_pose_activity(self):
        if not self.__activity_started:
            self.ui.pushButton.setText("Finish Pose Activity")
            # Send target pose to server
            self.node.initiate_static_hold()
        else:
            self.ui.pushButton.setText("Start Pose Activity")
            # Unsub from results
            pass
        self.__activity_started = not self.__activity_started
