import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    snr_package_path = get_package_share_directory('snr')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(snr_package_path, 'my_map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        package_path, 'params', 'dwa_nav_params.yaml'))

    ## object detection stuff##
    object_detection_pkg = 'object_detection'
    obj_detection_package_path = get_package_share_directory(object_detection_pkg)
 
    obj_detection_node = Node(
        package=object_detection_pkg,
        executable='color_obj_detection',
        name='color_obj_detection_node',
        output="screen"
    )

    ## Camera Stuff Uncomment which camera robot uses
    cam_package_path = get_package_share_directory('astra_camera')

    # astra_camera_launch = IncludeLaunchDescription(XMLLaunchDescriptionSource(
    #     [os.path.join(cam_package_path, 'launch'),
    #      '/astro_pro_plus.launch.xml'])
    # )

    astra_camera_launch = IncludeLaunchDescription(XMLLaunchDescriptionSource([os.path.join(cam_package_path, 'launch'),
                                                                               '/astra_pro.launch.xml']))


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path,
                              description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        # obj_detection_node,
        # astra_camera_launch
    ])