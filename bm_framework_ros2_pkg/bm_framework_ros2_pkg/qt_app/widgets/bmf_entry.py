from PySide2.QtWidgets import QTabWidget, QWidget
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode
from bm_framework_ros2_pkg.qt_app.signals import GuiSignals

from bm_framework_ros2_pkg.qt_app.ui_sources import entry_widget_ui
from bm_framework_ros2_pkg.qt_app.widgets.bmf_angular_pose_manager import BMFAngularPoseManager
from bm_framework_ros2_pkg.qt_app.widgets.bmf_sensor_readings import BMFSensorReadings


class BMFEntryWidget(QWidget):
    def __init__(self, gui_signals: GuiSignals, node: BMApplicationNode, parent=None):
        super(BMFEntryWidget, self).__init__(parent)
        self.gui_signals = gui_signals
        self.node = node
        self.__setup_ui()
        self.__init_layout()

    def __setup_ui(self):
        self.ui = entry_widget_ui.Ui_Form()
        self.ui.setupUi(self)
        self.tabWidget = QTabWidget()
        self.ui.verticalLayout.addWidget(self.tabWidget)

    def __init_layout(self):
        self.angular_pose_manager = BMFAngularPoseManager(self.gui_signals, self.node)
        self.tabWidget.addTab(self.angular_pose_manager, "Pose Manager")
        self.sensor_readings = BMFSensorReadings(self.gui_signals, self.node)
        self.tabWidget.addTab(self.sensor_readings, "Sensor Readings")
