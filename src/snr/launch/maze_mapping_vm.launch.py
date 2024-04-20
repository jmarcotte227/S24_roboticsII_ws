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
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    ## Keyboard Control


    ## Camera Stuff Uncomment which camera robot uses
    cam_package_path = get_package_share_directory('astra_camera')

    # astra_camera_launch = IncludeLaunchDescription(XMLLaunchDescriptionSource(
    #     [os.path.join(cam_package_path, 'launch'),
    #      '/astro_pro_plus.launch.xml'])
    # )

    astra_camera_launch = IncludeLaunchDescription(XMLLaunchDescriptionSource([os.path.join(cam_package_path, 'launch'),
                                                                               '/astra_pro.launch.xml']))

    yahboomcar_package_path = get_package_share_directory('yahboomcar_bringup')

    yahboomcar_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(yahboomcar_package_path, 'launch'),
         '/yahboomcar_bringup_X3_launch.py'])
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
     
    ## object detection stuff## A 4/18
    object_detection_pkg = 'object_detection'
    obj_detection_package_path = get_package_share_directory(object_detection_pkg)

    obj_detection_node = Node(
        package=object_detection_pkg,
        executable='color_obj_detection',
        name='color_obj_detection_node',
        output="screen"
    )
    ##############################################################################
    
    return LaunchDescription([
        #yahboomcar_bringup_launch,
        rviz_arg,
        rviz_node,
        # obj_detection_node, #A 4/18
        # astra_camera_launch, #A 4/18
        gmapping_a1_launch,
        
        
    ])
