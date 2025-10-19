from setuptools import setup
import os
from glob import glob

package_name = 'tugas-robotika'

setup(
    name=package_name,
    version='0.0.1',
    packages=['PIDController', 'PIDNavigator', 'RobotController'],
    package_dir={
        'PIDController': 'src/PIDController/PIDController',
        'PIDNavigator': 'src/PIDNavigator/PIDNavigator',
        'RobotController': 'src/RobotController/RobotController',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itsrobocon2',
    maintainer_email='itsrobocon2@todo.todo',
    description='ROS2 differential drive robot control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = RobotController.robot_controller:main',
            'pid_controller = PIDController.pid_controller:main',
            'file_handling = PIDNavigator.file_handling:main',
        ],
    },
)

