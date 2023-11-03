from setuptools import find_packages
from setuptools import setup

setup(
    name='mac_messages',
    version='0.0.0',
    packages=find_packages(
        include=('mac_messages', 'mac_messages.*')),
)
