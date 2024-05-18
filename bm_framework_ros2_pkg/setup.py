import os
from glob import glob
from setuptools import setup

package_name = 'bm_framework_ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavelszuravlovs',
    maintainer_email='pavels.zuravlovs@aerones.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "bm_limb_readings = bm_framework_ros2_pkg.ros_driver.entries.limb_readings:main",
            "bm_server = bm_framework_ros2_pkg.ros_driver.entries.server:main",
            "bm_relay = bm_framework_ros2_pkg.ros_driver.entries.relay:main",
            "bm_metawear = bm_framework_ros2_pkg.ros_driver.entries.mw_readings:main",
            "bm_metawear_slave = bm_framework_ros2_pkg.ros_driver.entries.mw_spin_node:main",
            "bm_app = bm_framework_ros2_pkg.qt_app.window:main",
            "bm_app_orientations = bm_framework_ros2_pkg.qt_app.graphing.orientations:main",
        ],
    },
)
