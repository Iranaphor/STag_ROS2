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
    params_file = f'{PKG}/config/params.yaml'
    print(params_file)

    # Components to add to launch description
    components = []

    # Remappings
    remapping = []
    ns = context.launch_configurations['namespace']
    if context.launch_configurations['camera'] == 'usb_cam':
        remapping+=[(f'{ns}/image_raw', '/camera1/image_raw')]
    if context.launch_configurations['camera'] == 'realsense':
        remapping+=[(f'{ns}/image_raw',   '/camera/camera/color/image_raw'),
                   (f'{ns}/image_depth', '/camera/camera/depth/image_rect_raw')]

    # Processor
    components += [Node(package='stag_ros2',
                       executable='processor.py',
                       name='processor',
                       namespace=context.launch_configurations['namespace'],
                       remappings=remapping,
                       parameters=[params_file])]

    # Calibrator
    components += [Node(package='stag_ros2',
                       executable='calibrator.py',
                       name='calibrator',
                       namespace=context.launch_configurations['namespace'],
                       parameters=[params_file])]

    # Renderer
    components += [Node(package='stag_ros2',
                       executable='renderer.py',
                       name='renderer',
                       namespace=context.launch_configurations['namespace'])]

    # MQTT Forwarder
    components += [Node(package='stag_ros2',
                       executable='mqtt_forwarder.py',
                       name='mqtt_forwarder',
                       namespace=context.launch_configurations['namespace'],
                       parameters=[params_file])]

    # MQTT Broker
    if context.launch_configurations['use_local_broker'] == True:
        script_path = PathJoinSubstitution([PKG, 'bash/mosquitto_broker.sh'])
        components += [
            ExecuteProcess(
                cmd=['bash', script_path, context.launch_configurations['mqtt_broker_port']],
                output='screen')
        ]


    # Camera (usb_cam or realsense)
    if context.launch_configurations['camera'] == 'usb_cam':
        components += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('usb_cam'),
                    '/launch/camera.launch.py']))]

    elif context.launch_configurations['camera'] == 'realsense':
        components += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('realsense2_camera'),
                    '/launch/rs_launch.py']))]


    # RViz
    if str(context.launch_configurations['use_rviz']).lower() == 'true':
        components += [
            Node(package='rviz2',
                 executable='rviz2',
                 arguments=['-d', PKG+'/config/render.rviz'])]

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
    desc='Whether to launch a local mosquitto broker'
    LD.add_action(declare3('use_local_broker', desc, envvar='USE_LOCAL_BROKER', default='False'))
    desc='IP address of the MQTT broker'
    LD.add_action(declare3('mqtt_broker_ip', desc, envvar='MQTT_BROKER_IP', default='0.0.0.0'))
    desc='Port of the MQTT broker'
    LD.add_action(declare3('mqtt_broker_port', desc, envvar='MQTT_BROKER_PORT', default='8883'))

    # Define Camera Arguments
    desc='Camera type to be used. Either usb_cam or realsense'
    LD.add_action(declare3('camera', desc, envvar='STAG_CAMERA', default='usb_cam'))

    # Define Camera Arguments
    desc='Whether to launch rviz'
    LD.add_action(declare3('use_rviz', desc, envvar='USE_RVIZ', default=True))


    # Add the Components to Launch
    LD.add_action(OpaqueFunction(function=launch_setup))

    return LD
