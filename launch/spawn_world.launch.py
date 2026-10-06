"""
Launch one Gazebo Sim world and its ROS-Gazebo bridge.

This launch file starts a Gazebo server from one SDF world source and starts the bridge configured
by a YAML file.
It can also start the Gazebo GUI as a separate client process.
Static project obstacles should normally live directly in the SDF world file.
Dynamic entities can be spawned by a separate tool.

This launch file is inspired by the file
`/opt/ros/jazzy/share/ros_gz_sim/launch/ros_gz_sim.launch.py`.
"""

from pathlib import Path

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameters_type import ParametersDict
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

from ros_gz_tools.helpers import get_world_name
from ros_gz_tools.helpers import get_world_name_from_string


def generate_launch_description() -> LaunchDescription:
    """Declare the inputs and the action that creates the generic world bringup."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='namespace',
                default_value='',
                description='Top-level namespace used by the world bridge.',
            ),
            DeclareLaunchArgument(
                'gzserver_use_composition',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description=(
                    'Load compatible Gazebo and bridge processes into a ROS component container '
                    'when true.'
                ),
            ),
            DeclareLaunchArgument(
                'gzserver_create_own_container',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description=(
                    'Start a dedicated ROS component container when composition is enabled.'
                ),
            ),
            DeclareLaunchArgument(
                'gzserver_container_name',
                default_value='ros_gz_container',
                description='ROS component container used when composition is enabled.',
            ),
            DeclareLaunchArgument(
                'gzserver_initial_sim_time',
                default_value='0.0',
                description='Initial simulation time in seconds.',
            ),
            DeclareLaunchArgument(
                'gzserver_verbosity_level',
                default_value='4',
                choices=['0', '1', '2', '3', '4'],
                description='Gazebo server verbosity level from 0 (FATAL) to 4 (DEBUG).',
            ),
            DeclareLaunchArgument(
                'gzgui_enabled',
                default_value='True',
                choices=['True', 'true', 'False', 'false'],
                description='Start the Gazebo GUI client when true.',
            ),
            DeclareLaunchArgument(
                'gzgui_config_file',
                default_value='',
                description='Gazebo Sim GUI client configuration file.',
            ),
            DeclareLaunchArgument(
                'world_sdf_file', default_value='', description='Path to the SDF world file.'
            ),
            DeclareLaunchArgument(
                'world_sdf_string', default_value='', description='Inline SDF world XML string.'
            ),
            DeclareLaunchArgument(
                'world_bridge_name',
                default_value='',
                description='Name assigned to the ROS-Gazebo bridge action.',
            ),
            DeclareLaunchArgument(
                'world_bridge_config_file',
                default_value='',
                description='YAML file that configures the world ROS-Gazebo bridge.',
            ),
            DeclareLaunchArgument(
                'world_bridge_subscription_heartbeat',
                default_value='1000',
                description='Milliseconds between bridge subscription heartbeat checks.',
            ),
            DeclareLaunchArgument(
                'world_bridge_expand_gz_topic_names',
                default_value='True',
                choices=['True', 'true', 'False', 'false'],
                description='Expand Gazebo topic names in bridge rules.',
            ),
            DeclareLaunchArgument(
                'world_bridge_override_timestamps_with_wall_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Replace bridged message timestamps with wall-clock time.',
            ),
            DeclareLaunchArgument(
                'world_bridge_override_frame_id',
                default_value='',
                description='Frame ID override applied by the bridge when supported.',
            ),
            DeclareLaunchArgument(
                'world_bridge_use_respawn',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description=('Respawn the bridge after a crash when composition is disabled.'),
            ),
            DeclareLaunchArgument(
                'world_bridge_log_level',
                default_value='info',
                choices=['debug', 'info', 'warn', 'error', 'fatal'],
                description='ROS-Gazebo bridge log level.',
            ),
            DeclareLaunchArgument(
                'world_bridge_use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use the simulation clock for the world bridge.',
            ),
            OpaqueFunction(function=_spawn_world),
        ]
    )


def _spawn_world(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Validate the world inputs and create the server, bridge, and optional GUI actions."""
    world_sdf_file = LaunchConfiguration('world_sdf_file').perform(ctx)
    world_sdf_string = LaunchConfiguration('world_sdf_string').perform(ctx)

    # An empty or whitespace-only value does not provide a world source.
    file_was_provided = bool(world_sdf_file.strip())
    string_was_provided = bool(world_sdf_string.strip())
    if file_was_provided == string_was_provided:
        raise ValueError(
            "Exactly one of launch arguments 'world_sdf_file' or 'world_sdf_string' must "
            'be provided.'
        )

    if world_sdf_file and not Path(world_sdf_file).is_file():
        raise FileNotFoundError(f"World SDF file '{world_sdf_file}' not found")

    world_name = (
        get_world_name(world_sdf_file)
        if file_was_provided
        else get_world_name_from_string(world_sdf_string)
    )

    world_bridge_config_file = LaunchConfiguration('world_bridge_config_file').perform(ctx)

    if not world_bridge_config_file:
        raise ValueError("Launch argument 'world_bridge_config_file' must be provided.")

    if not Path(world_bridge_config_file).is_file():
        raise FileNotFoundError(f"World ROS-GZ bridge file '{world_bridge_config_file}' not found")

    world_bridge_name = LaunchConfiguration('world_bridge_name').perform(ctx)
    effective_world_bridge_name = world_bridge_name or f'{world_name}_bridge'

    extra_bridge_params: ParametersDict = {
        (TextSubstitution(text='use_sim_time'),): ParameterValue(
            LaunchConfiguration('world_bridge_use_sim_time'), value_type=bool
        ),
        (TextSubstitution(text='subscription_heartbeat'),): ParameterValue(
            LaunchConfiguration('world_bridge_subscription_heartbeat'), value_type=int
        ),
        (TextSubstitution(text='expand_gz_topic_names'),): ParameterValue(
            LaunchConfiguration('world_bridge_expand_gz_topic_names'), value_type=bool
        ),
        (TextSubstitution(text='override_timestamps_with_wall_time'),): ParameterValue(
            LaunchConfiguration('world_bridge_override_timestamps_with_wall_time'), value_type=bool
        ),
        (TextSubstitution(text='override_frame_id'),): ParameterValue(
            LaunchConfiguration('world_bridge_override_frame_id'), value_type=str
        ),
    }

    launch_gui = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('gzgui_enabled'), bool), bool
    )

    launch_entities: list[LaunchDescriptionEntity] = [
        LogInfo(msg=f'World file: {world_sdf_file}'),
        LogInfo(msg=f'Bridge file: {world_bridge_config_file}'),
        LogInfo(msg=f'Bridge name: {effective_world_bridge_name}'),
        LogInfo(msg=f'World name: {world_name}'),
        LogInfo(msg=f'Launch GUI: {launch_gui}'),
        GzServer(
            world_sdf_file=world_sdf_file,
            world_sdf_string=world_sdf_string,
            container_name=LaunchConfiguration('gzserver_container_name'),
            create_own_container=LaunchConfiguration('gzserver_create_own_container'),
            use_composition=LaunchConfiguration('gzserver_use_composition'),
            initial_sim_time=LaunchConfiguration('gzserver_initial_sim_time'),
            verbosity_level=LaunchConfiguration('gzserver_verbosity_level'),
        ),
        RosGzBridge(
            bridge_name=effective_world_bridge_name,
            config_file=world_bridge_config_file,
            container_name=LaunchConfiguration('gzserver_container_name'),
            create_own_container=False,
            namespace=LaunchConfiguration('namespace'),
            use_composition=LaunchConfiguration('gzserver_use_composition'),
            use_respawn=LaunchConfiguration('world_bridge_use_respawn'),
            log_level=LaunchConfiguration('world_bridge_log_level'),
            bridge_params='',
            extra_bridge_params=extra_bridge_params,
        ),
    ]

    if launch_gui:
        launch_entities.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('ros_gz_tools'), 'launch', 'spawn_gui.launch.py']
                    )
                ),
                launch_arguments={
                    'gzgui_config_file': LaunchConfiguration('gzgui_config_file')
                }.items(),
            )
        )

    return launch_entities
