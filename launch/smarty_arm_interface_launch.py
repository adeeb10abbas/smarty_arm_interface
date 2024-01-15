from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

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
    return LaunchDescription([
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
