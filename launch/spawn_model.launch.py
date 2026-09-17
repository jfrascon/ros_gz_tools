"""
Spawn one model into a Gazebo world.

This launch file is inspired in the file
`/opt/ros/jazzy/share/ros_gz_sim/launch/gz_spawn_model.launch.py`.
"""

import math
from pathlib import Path

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Declare the launch arguments consumed by the generic model spawner."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world_name', description='Gazebo world where the model will be spawned.'
            ),
            DeclareLaunchArgument('model_sdf_file', default_value='', description='SDF filename'),
            DeclareLaunchArgument(
                'model_sdf_string', default_value='', description='Inline SDF XML string.'
            ),
            DeclareLaunchArgument(
                'model_sdf_topic', default_value='', description='ROS topic providing the SDF XML.'
            ),
            DeclareLaunchArgument(
                'model_entity_name', description='Name assigned to the spawned Gazebo entity.'
            ),
            DeclareLaunchArgument(
                'model_allow_renaming',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Allow Gazebo to rename the entity when its requested name is in use.',
            ),
            DeclareLaunchArgument(
                'model_pose_x',
                default_value='0.0',
                description='Initial model X position in meters',
            ),
            DeclareLaunchArgument(
                'model_pose_y',
                default_value='0.0',
                description='Initial model Y position in meters',
            ),
            DeclareLaunchArgument(
                'model_pose_z',
                default_value='0.0',
                description='Initial model Z position in meters',
            ),
            DeclareLaunchArgument(
                'model_pose_roll', default_value='0.0', description='Initial model roll in radians'
            ),
            DeclareLaunchArgument(
                'model_pose_pitch',
                default_value='0.0',
                description='Initial model pitch in radians',
            ),
            DeclareLaunchArgument(
                'model_pose_yaw', default_value='0.0', description='Initial model yaw in radians'
            ),
            DeclareLaunchArgument(
                'node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            OpaqueFunction(function=_spawn_model),
        ]
    )


def _spawn_model(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Validate the model request and create the Gazebo model-spawn actions."""
    world_name = LaunchConfiguration('world_name').perform(ctx)
    model_entity_name = LaunchConfiguration('model_entity_name').perform(ctx)
    model_sources = {
        'model_sdf_file': LaunchConfiguration('model_sdf_file').perform(ctx),
        'model_sdf_string': LaunchConfiguration('model_sdf_string').perform(ctx),
        'model_sdf_topic': LaunchConfiguration('model_sdf_topic').perform(ctx),
    }

    if not world_name.strip():
        raise ValueError("Launch argument 'world_name' must identify a Gazebo world.")

    if not model_entity_name.strip():
        raise ValueError("Launch argument 'model_entity_name' must identify the Gazebo entity.")

    provided_sources = [name for name, value in model_sources.items() if value.strip()]

    if len(provided_sources) != 1:
        raise ValueError(
            "Exactly one of launch arguments 'model_sdf_file', 'model_sdf_string', or "
            "'model_sdf_topic' must be provided."
        )

    model_sdf_file = model_sources['model_sdf_file']

    if model_sdf_file and not Path(model_sdf_file).is_file():
        raise FileNotFoundError(f"Model SDF file '{model_sdf_file}' not found")

    return [
        LogInfo(msg=f"Spawning model '{model_entity_name}' into world '{world_name}'"),
        Node(
            package='ros_gz_sim',
            executable='create',
            parameters=[
                {
                    'world': world_name,
                    'file': model_sdf_file,
                    'string': model_sources['model_sdf_string'],
                    'topic': model_sources['model_sdf_topic'],
                    'name': model_entity_name,
                    'allow_renaming': ParameterValue(
                        LaunchConfiguration('model_allow_renaming'), value_type=bool
                    ),
                    'x': _finite_float_argument(ctx, 'model_pose_x'),
                    'y': _finite_float_argument(ctx, 'model_pose_y'),
                    'z': _finite_float_argument(ctx, 'model_pose_z'),
                    'R': _finite_float_argument(ctx, 'model_pose_roll'),
                    'P': _finite_float_argument(ctx, 'model_pose_pitch'),
                    'Y': _finite_float_argument(ctx, 'model_pose_yaw'),
                }
            ],
            **rlh.resolve_node_arguments(LaunchConfiguration('node_args').perform(ctx)),
        ),
    ]


def _finite_float_argument(ctx: LaunchContext, argument_name: str) -> float:
    """Resolve one launch argument as a finite floating-point value."""
    value = LaunchConfiguration(argument_name).perform(ctx)
    try:
        parsed_value = float(value)
    except ValueError as exc:
        raise ValueError(f"Launch argument '{argument_name}' must be a finite number.") from exc

    if not math.isfinite(parsed_value):
        raise ValueError(f"Launch argument '{argument_name}' must be a finite number.")

    return parsed_value
