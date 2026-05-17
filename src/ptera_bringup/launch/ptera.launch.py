import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot = os.environ.get('PTERA_ROBOT')
    if robot is None:
        raise RuntimeError("PTERA_ROBOT environment variable is not set")

    sim = LaunchConfiguration('sim').perform(context)

    # Load system config
    pkg_share = get_package_share_directory('ptera_bringup')
    config_path = os.path.join(pkg_share, 'config', 'robots', robot, 'system.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    actions = []
    for module_name, module_cfg in config['modules'].items():
        if not module_cfg.get('enabled', False):
            continue
        
        if 'simulation' == module_name and 'false' == sim: 
            continue # skip simulation module if not running sim

        bringup_pkg = module_cfg['bringup_pkg']
        launch_file = module_cfg['launch_file']
        ws_path     = module_cfg.get('workspace_path', '')

        if ws_path:
            launch_path = os.path.join(
                ws_path, 'install', bringup_pkg,
                'share', bringup_pkg, 'launch', launch_file
            )
        else:
            launch_path = os.path.join(
                get_package_share_directory(bringup_pkg), 'launch', launch_file
            )

        if not os.path.exists(launch_path):
            raise RuntimeError(f"[{module_name}] Launch file not found: {launch_path}")

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path),
                launch_arguments={
                    'robot': robot,
                    'sim':   sim,
                }.items()
            )
        )
        
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Enable simulation mode'
        ),
        OpaqueFunction(function=launch_setup)
    ])