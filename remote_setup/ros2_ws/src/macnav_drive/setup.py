import glob, os

from setuptools import find_packages, setup

package_name = 'macnav_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),        
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*.launch.xml'))),
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*launch.py'))),
        ('share/' + package_name + '/config', 
            glob.glob(os.path.join('config', '*'))),
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            "odom_to_tf_node = macnav_drive.odom_to_tf_node:main",
        ],
    },
)
