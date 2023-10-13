"""
Node to handle params from GCS.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

February 02, 2022
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import sys
params_holder_path = '/home/neo/workspace/src/uwb_driver'

config = os.path.join(params_holder_path, 'config/uwb_driver.yaml')

def generate_launch_description():
    """Builds a LaunchDescription for the ParamsManager"""
    port = LaunchConfiguration('port', default='/dev/uwb1')
    id = LaunchConfiguration('id', default='0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port to connect to'),
        DeclareLaunchArgument(
            'id',
            default_value='0',
            description='Tag ID'),

        # Create node launch description
        Node(
            package='uwb_driver',
            executable='uwb_driver',
            exec_name='uwb_driver',
            shell=True,
            emulate_tty=True,
            output='both',
            log_cmd=True,
            parameters=[{'port': port, 'id' : id}]
        )

    ])
