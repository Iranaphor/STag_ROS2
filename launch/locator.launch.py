from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate parameter file
    PKG = get_package_share_directory('stag_ros2')
    params_file = f'{PKG}/config/params_robot.yaml'

    # Construct Launch Description
    LD = LaunchDescription()

    # Remappings
    remapping=[
        (f'/image_raw', '/camera1/image_raw'),
    ]

    # MQTT Receiver
    LD.add_action(Node(package='stag_ros2',
                       executable='mqtt_receiver.py',
                       name='mqtt_receiver',
                       remappings=remapping,
                       parameters=[params_file]))

    return LD
