from time import sleep
from PySide2 import QtCore
from PySide2.QtWidgets import QVBoxLayout, QWidget
import pyqtgraph as pg
import numpy as np
from bm_framework_ros2_pkg.qt_app.nodes.application_node import BMApplicationNode
from bm_framework_interfaces_ros2_pkg.msg import Sensors
from shared_memory_dict import SharedMemoryDict


class BMFSensorReadings(QWidget):
    def __init__(self, node: BMApplicationNode, parent=None):
        super(BMFSensorReadings, self).__init__(parent)
        self.node = node
        self.Sensors = []
        self.__init_graphics()
        self.__init_subscribers()

    def __init_graphics(self):
        # Start pyqtgraph
        pg.setConfigOptions(antialias=True)
        self.main_layout = QVBoxLayout()
        self.graph_canvas = pg.GraphicsLayoutWidget(show=True, title="Basic plotting examples")
        self.main_layout.addWidget(self.graph_canvas)
        self.limb_left = self.graph_canvas.addPlot(title="Left Limb")
        self.limb_left = self.graph_canvas.addPlot(title="Right Limb")
        self.graph_canvas.nextRow()
        self.left = self.graph_canvas.addPlot(title="Left")
        self.center = self.graph_canvas.addPlot(title="Center")
        self.right = self.graph_canvas.addPlot(title="Right")
        self.setLayout(self.main_layout)

        # Setup SMD for external OpenGL drawing
        self.smd = SharedMemoryDict(name='tokens', size=1024)
        self.smd['some-key'] = 'some-value-with-any-type'

    def __init_subscribers(self):
        self.node.create_subscription(Sensors, "sensors_data", self.__cb_sub_sensors_data, 1)

    def __cb_sub_sensors_data(self, sensors: Sensors):
        self.sensors = sensors

        # Graphs
        self.curve = self.limb_left.plot(pen='y')
        self.data = np.random.normal(size=(10,1000))
        self.ptr = 0
        def update():
            self.curve.setData(self.data[self.ptr%10])
            if self.ptr == 0:
                self.limb_left.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
            self.ptr += 1
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(update)
        self.timer.start(1000)
