import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory



def launch_setup(context, *args, **kwargs):
    # Locate parameter file
    PKG = get_package_share_directory('stag_ros2')
    params_file = f'{PKG}/config/params_robot.yaml'
    print(params_file)

    # Components to add to launch description
    components = []

    # Remappings
    remapping = []
    ns = context.launch_configurations['namespace']

    # MQTT Receiver
    components += [Node(package='stag_ros2',
                       executable='mqtt_receiver.py',
                       name='mqtt_receiver',
                       namespace=context.launch_configurations['namespace'],
                       parameters=[params_file])]

    return components


def declare3(arg_name, description, envvar='', default=None):
    return DeclareLaunchArgument(
        arg_name,
        default_value=os.getenv(envvar, default),
        description=description
    )


def generate_launch_description():
    # Locate parameter file
    PKG = get_package_share_directory('stag_ros2')

    # Construct Launch Description
    LD = LaunchDescription()

    # Define MQTT Arguments
    desc='Namespace for the stag_ros2 nodes'
    LD.add_action(declare3('namespace', desc, default='/stag_ros2'))

    # Define MQTT Arguments
    desc='IP address of the MQTT broker'
    LD.add_action(declare3('mqtt_broker_ip', desc, envvar='MQTT_BROKER_IP', default='0.0.0.0'))
    desc='Port of the MQTT broker'
    LD.add_action(declare3('mqtt_broker_port', desc, envvar='MQTT_BROKER_PORT', default='8883'))

    # Add the Components to Launch
    LD.add_action(OpaqueFunction(function=launch_setup))

    return LD
