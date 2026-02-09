from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'game_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.json')),
        ('share/' + package_name + '/games', glob('games/*.json')),
        ('share/' + package_name + '/games/answers', glob('games/answers/*.json')),
        ('share/' + package_name + '/games/phases', glob('games/phases/*.json')),
        ('share/' + package_name + '/games/profiles', glob('games/profiles/*.json')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EMOROBOCARE',
    maintainer_email='emorobcare@example.com',
    description='ROS 2 game controller for Colors game with decision_making FSM and generic_ui',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_controller_node = game_controller.node:main',
        ],
    },
)
