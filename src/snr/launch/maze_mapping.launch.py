from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node 
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path


def generate_launch_description():

    ## object detection stuff##
    object_detection_pkg = 'object_detection'
    obj_detection_package_path = get_package_share_directory(object_detection_pkg)
 
    obj_detection_node = Node(
        package=object_detection_pkg,
        executable='color_obj_detection',
        name='color_obj_detection_node',
        output="screen"
    )
    #################################################

    
    ## Gmapping Stuff##
    # RPLIDAR_TYPE = 'a1'
    # rplidar_type_arg = DeclareLaunchArgument(name='rplidar_type', default_value=RPLIDAR_TYPE, 
    #                                           choices=['a1','s2','4ROS'],
    #                                           description='The type of robot')

    gmapping_a1_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_a1_launch.py']),
        
    )
    package_path = get_package_share_directory('snr')
    default_rviz_config_path = os.path.join(package_path, 'map.rviz')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    ##############################################################################
    
    return LaunchDescription([
        obj_detection_node,
        gmapping_a1_launch,
        rviz_arg,
        rviz_node
    ])