from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(
        get_package_share_directory('micromouse'),
        'worlds',
        'micromouse.world'
    )

    gui_config = os.path.join(
        get_package_share_directory('micromouse'),
        'worlds',
        'gui.config'
    )

    bridge_config = os.path.join(
        get_package_share_directory('micromouse'),
        'params',
        'micromouse_bridge.yaml'
    )

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ]
    )

    gz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [world, ' --gui-config ', gui_config, ' -v4']}.items()
    )

    micromouse_node = Node(
        package='micromouse',
        executable='micromouse',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(bridge_cmd)
    ld.add_action(gz_cmd)
    ld.add_action(micromouse_node)
    return ld
