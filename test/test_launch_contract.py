"""Test the public launch contracts."""

import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_NODE_ARGS = '{"output":"both","ros_arguments":["--log-level","info"]}'


def _load_launch_file(filename: str) -> ModuleType:
    """Load one source launch file as a Python module."""
    path = PACKAGE_ROOT / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename.replace('.', '_'), path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('filename', ['spawn_model.launch.py', 'remove_model.launch.py'])
def test_model_launchers_use_the_standard_node_arguments(filename: str) -> None:
    """Expose one JSON node_args input with the workspace logging defaults."""
    module = _load_launch_file(filename)
    declarations = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert 'node_args' in declarations
    assert not any(name.endswith('_node_output') for name in declarations)
    assert not any(name.endswith('_node_log_level') for name in declarations)

    context = LaunchContext()
    declarations['node_args'].visit(context)
    assert context.launch_configurations['node_args'] == DEFAULT_NODE_ARGS


def _model_context() -> LaunchContext:
    """Create a complete context for testing model-spawn validation."""
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'world_name': 'factory',
            'model_sdf_file': '',
            'model_sdf_string': '',
            'model_sdf_topic': '',
            'model_entity_name': 'robot',
            'model_allow_renaming': 'False',
            'model_pose_x': '0.0',
            'model_pose_y': '0.0',
            'model_pose_z': '0.0',
            'model_pose_roll': '0.0',
            'model_pose_pitch': '0.0',
            'model_pose_yaw': '0.0',
            'node_args': DEFAULT_NODE_ARGS,
        }
    )
    return context


@pytest.mark.parametrize(
    'sources',
    [
        {},
        {'model_sdf_file': 'robot.sdf', 'model_sdf_string': '<sdf/>'},
        {'model_sdf_string': '<sdf/>', 'model_sdf_topic': 'robot_description'},
    ],
)
def test_spawn_model_requires_exactly_one_model_source(sources: dict) -> None:
    """Reject ambiguous requests before the ros_gz_sim create node starts."""
    module = _load_launch_file('spawn_model.launch.py')
    context = _model_context()
    context.launch_configurations.update(sources)

    with pytest.raises(ValueError, match='Exactly one'):
        module._spawn_model(context)


def test_spawn_model_rejects_a_missing_sdf_file(tmp_path: Path) -> None:
    """Report a missing SDF file before starting the model-spawn node."""
    module = _load_launch_file('spawn_model.launch.py')
    context = _model_context()
    context.launch_configurations['model_sdf_file'] = str(tmp_path / 'missing.sdf')

    with pytest.raises(FileNotFoundError, match='missing.sdf'):
        module._spawn_model(context)


def test_spawn_model_passes_validated_inputs_to_the_node(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Pass one validated model source and the resolved node arguments to ros_gz_sim."""
    module = _load_launch_file('spawn_model.launch.py')
    context = _model_context()
    model_file = tmp_path / 'robot.sdf'
    model_file.write_text('<sdf version="1.9"><model name="robot"/></sdf>', encoding='utf-8')
    context.launch_configurations['model_sdf_file'] = str(model_file)
    captured: dict[str, object] = {}

    class FakeNode:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    monkeypatch.setattr(module, 'Node', FakeNode)

    actions = module._spawn_model(context)

    assert len(actions) == 2
    assert captured['package'] == 'ros_gz_sim'
    assert captured['executable'] == 'create'
    assert captured['output'] == 'both'
    assert captured['ros_arguments'] == ['--log-level', 'info']
    parameters = captured['parameters'][0]
    assert parameters['world'] == 'factory'
    assert parameters['file'] == str(model_file)
    assert parameters['name'] == 'robot'
    assert parameters['x'] == 0.0
    assert parameters['Y'] == 0.0


@pytest.mark.parametrize('value', ['nan', 'inf', '-inf', 'invalid'])
def test_spawn_model_rejects_invalid_pose_values(tmp_path: Path, value: str) -> None:
    """Reject pose values that cannot identify a finite location or orientation."""
    module = _load_launch_file('spawn_model.launch.py')
    context = _model_context()
    model_file = tmp_path / 'robot.sdf'
    model_file.write_text('<sdf version="1.9"><model name="robot"/></sdf>', encoding='utf-8')
    context.launch_configurations['model_sdf_file'] = str(model_file)
    context.launch_configurations['model_pose_x'] = value

    with pytest.raises(ValueError, match='model_pose_x'):
        module._spawn_model(context)


@pytest.mark.parametrize(('world_name', 'model_entity_name'), [('', 'robot'), ('factory', '')])
def test_remove_model_requires_world_and_entity_names(
    world_name: str, model_entity_name: str
) -> None:
    """Reject an incomplete removal request before starting ros_gz_sim."""
    module = _load_launch_file('remove_model.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'world_name': world_name,
            'model_entity_name': model_entity_name,
            'node_args': DEFAULT_NODE_ARGS,
        }
    )

    with pytest.raises(ValueError):
        module._remove_model(context)


def test_remove_model_passes_validated_inputs_to_the_node(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pass validated removal names and node arguments to ros_gz_sim."""
    module = _load_launch_file('remove_model.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        {'world_name': 'factory', 'model_entity_name': 'robot', 'node_args': DEFAULT_NODE_ARGS}
    )
    captured: dict[str, object] = {}

    class FakeNode:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    monkeypatch.setattr(module, 'Node', FakeNode)

    actions = module._remove_model(context)

    assert len(actions) == 2
    assert captured['package'] == 'ros_gz_sim'
    assert captured['executable'] == 'remove'
    assert captured['output'] == 'both'
    assert captured['ros_arguments'] == ['--log-level', 'info']
    assert captured['parameters'] == [{'world': 'factory', 'entity_name': 'robot'}]


def _world_context(tmp_path: Path) -> LaunchContext:
    """Create a complete context for testing world bringup without starting Gazebo."""
    bridge_file = tmp_path / 'bridge.yaml'
    bridge_file.write_text('[]\n', encoding='utf-8')
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'namespace': '',
            'gzserver_use_composition': 'False',
            'gzserver_create_own_container': 'False',
            'gzserver_container_name': 'ros_gz_container',
            'gzserver_initial_sim_time': '0.0',
            'gzserver_verbosity_level': '4',
            'gzgui_enabled': 'False',
            'gzgui_config_file': '',
            'world_sdf_file': '',
            'world_sdf_string': '',
            'world_bridge_name': '',
            'world_bridge_config_file': str(bridge_file),
            'world_bridge_subscription_heartbeat': '1000',
            'world_bridge_expand_gz_topic_names': 'True',
            'world_bridge_override_timestamps_with_wall_time': 'False',
            'world_bridge_override_frame_id': '',
            'world_bridge_use_respawn': 'False',
            'world_bridge_use_sim_time': 'False',
            'world_bridge_log_level': 'info',
        }
    )
    return context


@pytest.mark.parametrize(
    'sources',
    [
        {},
        {
            'world_sdf_file': 'factory.sdf',
            'world_sdf_string': '<sdf version="1.9"><world name="factory"/></sdf>',
        },
    ],
)
def test_spawn_world_requires_exactly_one_world_source(
    tmp_path: Path, sources: dict[str, str]
) -> None:
    """Reject missing or ambiguous world sources before Gazebo starts."""
    module = _load_launch_file('spawn_world.launch.py')
    context = _world_context(tmp_path)
    context.launch_configurations.update(sources)

    with pytest.raises(ValueError, match='Exactly one'):
        module._spawn_world(context)


def test_spawn_world_rejects_a_missing_world_file(tmp_path: Path) -> None:
    """Reject a missing SDF world file before creating Gazebo actions."""
    module = _load_launch_file('spawn_world.launch.py')
    context = _world_context(tmp_path)
    context.launch_configurations['world_sdf_file'] = str(tmp_path / 'missing.sdf')

    with pytest.raises(FileNotFoundError, match='missing.sdf'):
        module._spawn_world(context)


def test_spawn_world_rejects_a_missing_bridge_file(tmp_path: Path) -> None:
    """Reject a missing bridge configuration before creating Gazebo actions."""
    module = _load_launch_file('spawn_world.launch.py')
    context = _world_context(tmp_path)
    context.launch_configurations['world_sdf_string'] = (
        '<sdf version="1.9"><world name="factory"/></sdf>'
    )
    context.launch_configurations['world_bridge_config_file'] = str(tmp_path / 'missing.yaml')

    with pytest.raises(FileNotFoundError, match='missing.yaml'):
        module._spawn_world(context)


@pytest.mark.parametrize('use_sim_time', ['False', 'True'])
def test_spawn_world_builds_server_and_bridge_actions(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, use_sim_time: str
) -> None:
    """Use the validated inline SDF name for the Gazebo server and bridge."""
    module = _load_launch_file('spawn_world.launch.py')
    context = _world_context(tmp_path)
    context.launch_configurations['world_sdf_string'] = (
        '<sdf version="1.9"><world name="factory"/></sdf>'
    )
    captured_server: dict[str, object] = {}
    captured_bridge: dict[str, object] = {}

    class FakeServer:
        def __init__(self, **kwargs: object) -> None:
            captured_server.update(kwargs)

    class FakeBridge:
        def __init__(self, **kwargs: object) -> None:
            captured_bridge.update(kwargs)

    monkeypatch.setattr(module, 'GzServer', FakeServer)
    monkeypatch.setattr(module, 'RosGzBridge', FakeBridge)

    actions = module._spawn_world(context)

    assert len(actions) == 7
    assert captured_server['world_sdf_file'] == ''
    assert captured_server['world_sdf_string'] == context.launch_configurations['world_sdf_string']
    assert captured_bridge['bridge_name'] == 'factory_bridge'
    context.launch_configurations['world_bridge_use_sim_time'] = use_sim_time
    bridge_params = {
        key[0].text: value.evaluate(context)
        for key, value in captured_bridge['extra_bridge_params'].items()
    }
    assert bridge_params['use_sim_time'] is {'False': False, 'True': True}[use_sim_time]
    assert (
        captured_bridge['config_file'] == context.launch_configurations['world_bridge_config_file']
    )


def test_spawn_gui_rejects_a_missing_configuration_file(tmp_path: Path) -> None:
    """Reject an explicit GUI layout path before starting the Gazebo client."""
    module = _load_launch_file('spawn_gui.launch.py')
    context = LaunchContext()
    context.launch_configurations['gzgui_config_file'] = str(tmp_path / 'missing.config')

    with pytest.raises(FileNotFoundError, match='missing.config'):
        module._spawn_gui(context)
