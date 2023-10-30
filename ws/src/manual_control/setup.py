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
        ],
    },
)
