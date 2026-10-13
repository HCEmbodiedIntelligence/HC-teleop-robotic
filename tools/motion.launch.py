"""Start only the optional motion server; feedback is provided externally."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('humanoid_motion_server'))
    defaults = {
        'params_file': share / 'config/motion_control.yaml',
        'channel_config_file': share / 'config/channels.yaml',
        'sdk_config_file': share / 'config/robo_manip.test_humanoid.yaml',
        'tool_config_file': share / 'config/tools.yaml',
        'urdf_file': share / 'urdf/test_humanoid.urdf',
        'joint_state_endpoint': '/observation/joint_states',
        'joint_command_endpoint': '/control/motion/joint_targets',
    }
    arguments = [DeclareLaunchArgument(key, default_value=str(value)) for key, value in defaults.items()]
    parameters = {key: LaunchConfiguration(key) for key in defaults if key != 'params_file'}
    return LaunchDescription(arguments + [Node(
        package='humanoid_motion_server', executable='humanoid_motion_control_node',
        name='humanoid_motion_control', output='screen',
        parameters=[LaunchConfiguration('params_file'), parameters],
    )])
