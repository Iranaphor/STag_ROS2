from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate parameter file
    PKG = get_package_share_directory('stag_ros2')

    # Construct Launch Description
    LD = LaunchDescription()

    # Remappings
    remapping=[
        (f'/image_raw', '/camera/camera/color/image_raw'),
        (f'/image_depth', '/camera/camera/depth/image_rect_raw')
    ]

    # Processor
    LD.add_action(Node(package='stag_ros2',
                       executable='processor.py',
                       name='processor',
                       remappings=remapping))

    # Realsense Camera
    LD.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py'])))

    # RViz
    LD.add_action(Node(package='rviz2',
                       executable='rviz2',
                       arguments=['-d', PKG+'/config/images_realsense.rviz']))

    return LD
