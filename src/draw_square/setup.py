import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'draw_square'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OUBRAHAM SADI SAMIR SEKOUANE',
    maintainer_email='93816869+sMouaad@users.noreply.github.com',
    description='Draw square action server — Assignment 1 (Create3 compatible)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'draw_square_server = draw_square.draw_square_server:main',
        ],
    },
)
