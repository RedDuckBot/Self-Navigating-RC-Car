import glob
import os

from setuptools import find_packages, setup

package_name = 'manual_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dantep',
    maintainer_email='dshillin11799@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "manual_turtle_node = manual_control.turtle_node:main",
            "arduino_node = manual_control.manual_arduino_node:main",
            "odom_node = manual_control.odom_node:main", 
            "sensor_test_node = manual_control.sensor_test_node:main",
            "odom_test_node = manual_control.odom_test:main"
        ],
    },
)
