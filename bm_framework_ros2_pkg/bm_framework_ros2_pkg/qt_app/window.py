import faulthandler
import sys
import threading

from PySide2.QtWidgets import QApplication, QMainWindow
import rclpy
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode

from bm_framework_ros2_pkg.qt_app.widgets.bmf_entry import BMFEntryWidget
from rclpy.executors import MultiThreadedExecutor

DEBUG = True


class BMFWindow(QMainWindow):
    def __init__(self, parent=None):
        super(BMFWindow, self).__init__(parent)
        self.__init_ros_env()
        self.entry = BMFEntryWidget(self.node)
        self.setCentralWidget(self.entry)

    def __init_ros_env(self):
        rclpy.init()

        self.node = BMApplicationNode()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        # Start ROS thread
        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # Initialise the logger
        self.logger = self.node.get_logger()

def main():
    global DEBUG
    if DEBUG:
        faulthandler.enable()

    app = QApplication(sys.argv)
    window = BMFWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
