# https://github.com/jdgalviss/realsense_ros2/blob/979350f8b5c1c70bea1d54182f893e8be6bc5e17/realsense_ros2/launch/cartographer_launch.py

import launch
import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    share_dir = get_package_share_directory('sllidar_ros2')
    simulator_dir = get_package_share_directory('arcanain_simulator')
    rviz_config_file = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'rviz',
            'cartographer.rviz')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(share_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rplidar_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share_dir, 'launch', 'sllidar_a1_launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
            ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser_frame']
            ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}],
            ),

            
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            ),
    ])
