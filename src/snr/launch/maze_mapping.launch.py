from launch import LaunchDescription
from launch_ros.actions import Node 
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
import os
from ament_index_python.packages import get_package_share_directory


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
    RPLIDAR_TYPE = 'a1'
    rplidar_type_arg = DeclareLaunchArgument(name='rplidar_type', default_value=RPLIDAR_TYPE, 
                                              choices=['a1','s2','4ROS'],
                                              description='The type of robot')

    gmapping_a1_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_a1_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', 'a1')
    )
    ##############################################################################
    
    return LaunchDescription([
        obj_detection_node,
        rplidar_type_arg,
        gmapping_a1_launch
    ])