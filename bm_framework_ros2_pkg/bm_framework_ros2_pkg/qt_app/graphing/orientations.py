from PySide2 import QtCore
import numpy as np
from shared_memory_dict import SharedMemoryDict
import pyqtgraph as pg
import pyqtgraph.opengl as gl

md = gl.MeshData.sphere(rows=10, cols=20)
x = np.linspace(-8, 8, 6)
limb_left = gl.GLMeshItem(meshdata=md, smooth=True, shader='shaded', glOptions='opaque')
limb_right = gl.GLMeshItem(meshdata=md, smooth=True, shader='shaded', glOptions='opaque')
left = gl.GLMeshItem(meshdata=md, smooth=True, shader='shaded', glOptions='opaque')
center = gl.GLMeshItem(meshdata=md, smooth=True, shader='shaded', glOptions='opaque')
right = gl.GLMeshItem(meshdata=md, smooth=True, shader='shaded', glOptions='opaque')
plt = gl.GLLinePlotItem(width=1, antialias=True)
i = 0

def update_rotations():
    global i

    pts = [[1,1,1], [0,0,0]]
    plt.setData(pos=pts, color=[1,1,1,1], width=1, antialias=True)
    i += 1

def main():
    existing_smd = SharedMemoryDict(name='sensors', size=1024)

    app = pg.mkQApp("Sensor Orientations")
    
    w = gl.GLViewWidget()
    w.show()
    w.setWindowTitle('pyqtgraph example: GL Shaders')
    w.setCameraPosition(distance=15, azimuth=-90)

    limb_left.translate(-5, 5, -2)
    # limb_left.scale(1, 1, 1)

    limb_right.translate(5, 5, -2)
    # limb_right.scale(1, 1, 2)

    left.translate(-3, 1, -0.5)
    # left.scale(1, 1, 2)

    center.translate(0, 0, 0)
    # center.scale(1, 1, 2)

    right.translate(3, 1, -0.5)
    # right.scale(1, 1, 2)

    w.addItem(limb_left)
    w.addItem(limb_right)
    w.addItem(left)
    w.addItem(center)
    w.addItem(right)

    w.addItem(plt)

    timer = QtCore.QTimer()
    timer.timeout.connect(update_rotations)
    timer.start(1000)
    pg.exec()

if __name__ == "__main__":
    main()
