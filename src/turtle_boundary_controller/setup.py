import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_boundary_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OUBRAHAM SADI SAMIR SEKOUANE',
    maintainer_email='93816869+sMouaad@users.noreply.github.com',
    description='Turtle boundary drawing controller with FSM and manual interrupt',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'draw_boundaries_node = turtle_boundary_controller.draw_boundaries_node:main',
            'keyboard_listener = turtle_boundary_controller.keyboard_listener:main',
        ],
    },
)
