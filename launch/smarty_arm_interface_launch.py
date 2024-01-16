import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_node(context):
    smarty_arm_type = context.launch_configurations['smarty_arm_type']
    node_name = 'smarty_arm_interface_' + smarty_arm_type
    arguments = [smarty_arm_type]

    return [Node(
        package='smarty_arm_interface',
        executable='smarty_arm_interface_node',
        name=node_name,
        arguments=arguments,
    )]

def generate_launch_description():
    config_file_path = PathJoinSubstitution([
        FindPackageShare('smarty_arm_interface'),
        'cfg/origin_shift.yaml'
    ])
    
    return launch.LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_file_path),
        DeclareLaunchArgument(
            'output',
            default_value='screen',
            description='Display output to screen or log file.'
        ),
        DeclareLaunchArgument(
            'smarty_arm_type',
            default_value='r',
            description='RDDA type could be r or l'
        ),
        OpaqueFunction(function=generate_node)
    ])
