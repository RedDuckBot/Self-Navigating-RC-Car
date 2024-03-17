from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mac_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='macnav',
    maintainer_email='macnav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drive=mac_drive.drive_node:main",
            "odom=mac_drive.odom_node:main",
            "odom2=mac_drive.odom_node2:main"
        ],
    },
)
