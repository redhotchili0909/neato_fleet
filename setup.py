from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'neato_fleet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models/neato'), glob('models/neato/*.sdf')),
        (os.path.join('share', package_name, 'models/worlds'), glob('models/worlds/*.world')),
        (os.path.join('share', package_name, 'models/obstacles'), glob('models/obstacles/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email=['dhvanshah15@gmail.com','cpark1@olin.edu','zbilimoria@olin.edu','ajain@olin.edu'],
    description='ROS 2 package for coordinating a fleet of Neato robots: goal management, command playback/recording, and RVO-based collision avoidance.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rvo_fleet_controller = neato_fleet.rvo_fleet_controller:main',
            'fleet_goals = neato_fleet.fleet_goals:main',
            'cmd_recorder = neato_fleet.cmd_recorder:main',
            'cmd_player = neato_fleet.cmd_player:main',
        ],
    },
)


