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
        (os.path.join('share', package_name, 'models/neato'), glob('models/neato/*.sdf')),
        (os.path.join('share', package_name, 'models/worlds'), glob('models/worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='dhvanshah15@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = neato_fleet.move_neatos:main'
            'simple_move = neato_fleet.move_neatos:main',
            'rvo_fleet_controller = neato_fleet.rvo_fleet_controller:main',
            'fleet_goals = neato_fleet.fleet_goals:main',
            'trajectory_recorder = neato_fleet.trajectory_recorder:main',
        ],
    },
)


