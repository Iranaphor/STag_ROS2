import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory



def launch_setup(context, *args, **kwargs):

    ###########################################
    # Define STag param file
    ###########################################

    # Locate parameter file
    PKG = get_package_share_directory('stag_ros2')
    #stag_params_file = f'{PKG}/config/params_stag.yaml'
    stag_params_file = context.launch_configurations['stag_params_file']

    fiducial_markers_file = context.launch_configurations['fiducial_markers_file']
    stag_params_file = RewrittenYaml(source_file=stag_params_file,
                                     root_key='',
                                     param_rewrites={'calibration_config_file':fiducial_markers_file},
                                     convert_types=True)


    ###########################################
    # Define Camera param file
    ###########################################

    if context.launch_configurations['camera'] == 'usb_cam':
        #usb_cam_params_file = f'{CAM}/config/params_usb_cam.yaml'
        usb_cam_params_file = context.launch_configurations['usb_cam_params_file']

    ###########################################
    # Define map server details
    ###########################################

    #map_server_params_file = f'{ENV}/config/params_map_server.yaml'
    map_server_params_file = context.launch_configurations['map_server_params_file']

    #map_input = f'{ENV}/config/metric/map/map.yaml
    map_input = context.launch_configurations['map_file']

    #rewrites={'use_sim_time':'false', 'yaml_filename':map_input}
    map_server_params_file_2 = RewrittenYaml(source_file=map_server_params_file,
                                             root_key='',
                                             param_rewrites={'yaml_filename':map_input},
                                             convert_types=True)

    ###########################################
    #Construct components to launch
    ###########################################

    # Components to add to launch description
    components = []

    # Remappings
    remapping = []
    ns = context.launch_configurations['namespace']
    if context.launch_configurations['camera'] == 'usb_cam':
        pass

    if context.launch_configurations['camera'] == 'realsense':
        remapping+=[(f'{ns}/image_raw',   '/camera/camera/color/image_raw'),
                   (f'{ns}/image_depth', '/camera/camera/depth/image_rect_raw')]

    # Processor
    components += [Node(package='stag_ros2',
                        executable='processor.py',
                        name='processor',
                        namespace=context.launch_configurations['namespace'],
                        remappings=remapping,
                        parameters=[stag_params_file])]

    # Calibrator
    components += [Node(package='stag_ros2',
                        executable='calibrator.py',
                        name='calibrator',
                        namespace=context.launch_configurations['namespace'],
                        parameters=[stag_params_file])]

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
                        parameters=[stag_params_file])]

    # Map Server
    components += [Node(package='nav2_map_server',
                        executable='map_server',
                        name='map_server',
                        output='screen',
                        parameters=[map_server_params_file_2])]
    components += [Node(package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_localization',
                        output='screen',
                        parameters=[{'use_sim_time': False,
                                     'autostart': True,
                                     'node_names': ['map_server']}])]

    # MQTT Broker
    if str(context.launch_configurations['use_local_broker']).lower() == "true":
        script_path = PathJoinSubstitution([PKG, 'bash/mosquitto_broker.sh'])
        port = context.launch_configurations['mqtt_broker_port']
        conf = context.launch_configurations['mqtt_broker_conf']
        components += [
            ExecuteProcess(cmd=['bash', script_path, port, conf], output='screen')
        ]


    # Camera (usb_cam or realsense)
    if context.launch_configurations['camera'] == 'usb_cam':
        components += [Node(package='usb_cam',
                            executable='usb_cam_node_exe',
                            name='usb_cam',
                            namespace=ns,
                            parameters=[usb_cam_params_file])]

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

    # Locate packages for defaults
    PKG = get_package_share_directory('stag_ros2')
    CAM = get_package_share_directory('cam_calibration')
    COM = get_package_share_directory('environment_common')
    ENV = get_package_share_directory('environment_template')

    # Construct Launch Description
    LD = LaunchDescription()

    ###########################################
    # STag Args
    ###########################################

    desc='Paramater file for stag systems'
    default = f'{PKG}/config/params_stag.yaml'
    LD.add_action(declare3('stag_params_file', desc, envvar='STAG_PARAMS', default=default))

    desc='Marker locations file for stag systems'
    default = f'{ENV}/config/world/fiducial_markers.yaml'
    LD.add_action(declare3('fiducial_markers_file', desc, envvar='FIDUCIAL_MARKERS_FILE', default=default))

    desc='Namespace for the stag_ros2 nodes'
    LD.add_action(declare3('namespace', desc, default='/stag_ros2'))

    ###########################################
    # MQTT Args
    ###########################################

    desc='Whether to launch a local mosquitto broker'
    LD.add_action(declare3('use_local_broker', desc, envvar='USE_LOCAL_BROKER', default='false'))

    desc='IP address of the MQTT broker'
    LD.add_action(declare3('mqtt_broker_ip', desc, envvar='MQTT_BROKER_IP', default='0.0.0.0'))

    desc='Port of the MQTT broker'
    LD.add_action(declare3('mqtt_broker_port', desc, envvar='MQTT_BROKER_PORT', default='8883'))

    desc='Config path for the MQTT broker'
    default = os.path.join(PKG, 'config', 'mosquitto.conf')
    LD.add_action(declare3('mqtt_broker_conf', desc, envvar='MQTT_BROKER_CONF', default=default))

    ###########################################
    # RViz Args
    ###########################################

    desc='Whether to launch rviz'
    LD.add_action(declare3('use_rviz', desc, envvar='USE_RVIZ', default='true'))

    ###########################################
    # Camera Args
    ###########################################

    desc='Camera type to be used. Either usb_cam or realsense'
    LD.add_action(declare3('camera', desc, envvar='STAG_CAMERA', default='usb_cam'))

    desc='Paramater file for usb_cam'
    default = f'{CAM}/config/params_usb_cam.yaml'
    LD.add_action(declare3('usb_cam_params_file', desc, envvar='USB_PARAMS', default=default))

    ###########################################
    # Map Server Args
    ###########################################

    desc='Paramater file for map_server'
    default = f'{COM}/config/params_map_server.yaml'
    LD.add_action(declare3('map_server_params_file', desc, envvar='COSTMAP_PARAM_FILE', default=default))

    desc='Full path to the metric map yaml file.'
    default = f'{ENV}/metric/map/map.yaml'
    LD.add_action(declare3('map_file', desc, envvar='COSTMAP_YAML_FILE', default=default))

    ###########################################
    # Add the Components to Launch
    ###########################################

    LD.add_action(OpaqueFunction(function=launch_setup))

    return LD
