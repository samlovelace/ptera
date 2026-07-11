import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    sim_pkg_share = get_package_share_directory('ptera_sim')
    models_dir    = os.path.join(sim_pkg_share, 'models')
    world_path    = os.path.join(sim_pkg_share, 'worlds', 'elm4.world')

    ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = (
        models_dir + ':' + ign_resource_path
    )

    gazebo_ign_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = (
        sim_pkg_share + "/../lib/:" + gazebo_ign_plugin_path
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', world_path],
        output='screen'
    )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ign_ros_bridge',
        output='screen',
        parameters=[{
            'lazy': False,
            'config_file': os.path.join(sim_pkg_share, 'config', 'bridge.yaml'),
        }]
    )

    delayed_bridge = TimerAction(period=1.0, actions=[bridge])

    return [gazebo, delayed_bridge]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
