# [`ros_gz_tools`](https://github.com/jfrascon/ros_gz_tools)

`ros_gz_tools` provides reusable ROS 2 launch files and package-owned resources for Gazebo Sim.
It starts worlds, bridges and the Gazebo GUI, and it can create or remove model entities after a world is running.
Robot-specific orchestration belongs in the package that owns each robot, not in this generic package.

## Installed resources

The package installs:

- Gazebo worlds under `worlds/`.
- Gazebo models under `models/`.
- Shared meshes under `meshes/`.
- The Gazebo GUI layout under `config/gui.config`.
- Reusable launch files under `launch/`.
- `wait_for_gz_service.py` under `lib/ros_gz_tools/`.

## Launch files

### `spawn_world.launch.py`

This launch file starts one Gazebo server and one ROS-Gazebo bridge.
It optionally starts the Gazebo GUI as a separate client.

Exactly one world source must be provided:

- `world_sdf_file`: path to an SDF world file.
- `world_sdf_string`: complete SDF world XML supplied as a string.

`world_bridge_config_file` is required and must identify an existing bridge YAML file.
`world_bridge_name` is optional; when empty, the launch file uses `<world_name>_bridge`.

`world_bridge_use_sim_time` selects the simulation clock for the bridge and defaults to `False`.
Simulation orchestrators can explicitly pass `True`.

The launch description creates the server action before the bridge action, but it does not wait for Gazebo to finish loading the world before starting the bridge.
The bridge can start before its Gazebo topics exist and connect when they become available.

The remaining arguments configure Gazebo composition, server verbosity, bridge behavior and the optional GUI.
Run the following command to inspect their names, defaults and accepted values:

```bash
ros2 launch ros_gz_tools spawn_world.launch.py --show-args
```

### `spawn_gui.launch.py`

This launch file starts only the Gazebo GUI process with `gz sim -g`.
It does not start a Gazebo server or create a bridge.

`gzgui_config_file` is optional.
When provided, it must be a filesystem path to an existing Gazebo GUI configuration file.
When omitted, Gazebo uses its default client layout.

### `spawn_model.launch.py`

This launch file starts the `ros_gz_sim create` node to insert one entity into a running world.
`world_name` and `model_entity_name` are required.

Exactly one model source must be provided:

- `model_sdf_file`: path to an SDF model file.
- `model_sdf_string`: complete SDF model XML supplied as a string.
- `model_sdf_topic`: ROS topic that publishes the model XML.

The `model_pose_*` arguments set the initial position in meters and orientation in radians.
`model_allow_renaming` allows Gazebo to select another entity name when the requested name is already in use.

### `remove_model.launch.py`

This launch file starts the `ros_gz_sim remove` node to remove one entity from a running world.
`world_name` and `model_entity_name` are required.

### Node arguments

`spawn_model.launch.py` and `remove_model.launch.py` expose one `node_args` JSON object for supported `launch_ros.actions.Node` arguments.
The default prints to both the screen and the ROS log and selects the `info` ROS log level:

```json
{"output":"both","ros_arguments":["--log-level","info"]}
```

The obsolete `model_spawn_node_output`, `model_spawn_node_log_level`, `model_remove_node_output` and `model_remove_node_log_level` arguments are no longer supported.
Set `output` and `ros_arguments` inside `node_args` instead.

## Waiting for a Gazebo service

`wait_for_gz_service.py` polls the service list returned by `gz service -l`.
It exits with status `0` when the requested absolute service name appears and status `1` when the timeout expires.
Both the timeout and polling period must be positive finite numbers.

Example:

```bash
ros2 run ros_gz_tools wait_for_gz_service.py \
  /world/factory/create \
  --timeout 60 \
  --poll-period 0.25
```

The current launch files do not invoke this executable automatically.
Downstream orchestration can use its exit status as a readiness barrier before starting actions that require a Gazebo service.

## Gazebo resource resolution

The SDF files use both `package://ros_gz_tools/...` and `model://<model_name>/...` URIs.
The package exports these Gazebo search roots:

- The installation `share/` directory resolves `package://ros_gz_tools/...` resources.
- `share/ros_gz_tools/models` resolves package-owned `model://...` resources.

The exports are declared in `package.xml` and in the package environment hook.
Source the workspace installation before starting Gazebo so `GZ_SIM_RESOURCE_PATH` contains those paths.

## Maintenance scripts

The scripts used to rebuild package-owned assets are documented in [scripts/README.md](scripts/README.md).
They are development tools and are not installed as runtime executables.
Their OpenCV, Shapely and Trimesh dependencies are therefore not runtime dependencies of this ROS package.

## Dependencies

Runtime dependencies are declared in `package.xml`:

- `launch`
- `launch_ros`
- `ros2_launch_helpers`
- `ros_gz_bridge`
- `ros_gz_sim`

`deps.repos` pins the source repository used to obtain `ros2_launch_helpers` when this repository is imported independently.
It complements `package.xml`; it does not replace the ROS dependency declaration.

## Build and test

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --symlink-install --packages-select ros_gz_tools
source install/setup.bash
colcon test --merge-install --packages-select ros_gz_tools
colcon test-result --test-result-base build/ros_gz_tools --verbose
```

## Examples

Start a world, its bridge and the Gazebo GUI:

```bash
ros2 launch ros_gz_tools spawn_world.launch.py \
  world_sdf_file:=/absolute/path/to/world.sdf \
  world_bridge_config_file:=/absolute/path/to/world_bridge.yaml \
  gzgui_enabled:=True
```

Start only the GUI with the layout installed by this package:

```bash
ros2 launch ros_gz_tools spawn_gui.launch.py \
  gzgui_config_file:="$(ros2 pkg prefix ros_gz_tools)/share/ros_gz_tools/config/gui.config"
```

Spawn one model from an SDF file with debug logging:

```bash
ros2 launch ros_gz_tools spawn_model.launch.py \
  world_name:=factory \
  model_entity_name:=robot_01 \
  model_sdf_file:=/absolute/path/to/robot.sdf \
  node_args:='{"output":"both","ros_arguments":["--log-level","debug"]}'
```

Remove that model:

```bash
ros2 launch ros_gz_tools remove_model.launch.py \
  world_name:=factory \
  model_entity_name:=robot_01
```

## License

This package is distributed under the Apache License 2.0.
See [LICENSE](LICENSE).
