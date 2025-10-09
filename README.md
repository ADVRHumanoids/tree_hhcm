# bt_hhcm

This package provides a BehaviorTree.CPP-based executable for hierarchical hybrid control and mission execution.

## Provided BTCPP TreeNodes
- **LoadParamFromConfig**: Loads parameters from YAML config files into the blackboard.
- **NodeProvider**: Provides a ROS2 node to the tree for communication and services.
- **RobotLoader**: Loads robot models and configuration, integrates with RobotInterface.
- **CartesioLoader**: Loads and configures Cartesio solvers and tasks for whole-body control.
- **CartesioTaskControl**: Enables or disables Cartesio tasks during execution, set references, monitors execution.

Additional custom nodes can be loaded via plugins using the `--plugins` option.

## Features
- Loads and runs behavior trees from XML files
- Supports plugin loading for custom tree nodes
- Command-line argument parsing for rate, name, plugins, parameters, and xacro arguments
- ROS2 integration and time synchronization

## Usage
Run the main executable with:
```
tree_hhcm_main [options] <tree_path>
```
Options:
- `--rate/-r <rate>`: Set tick rate (Hz)
- `--name/-n <name>`: Set node name
- `--plugins/-p <plugin>`: Load plugin (can be repeated)
- `--param/-m <param>`: Set blackboard param (key:=value, can be repeated)
- `--xacro-args/-a <arg>`: Pass xacro arguments
- `--dont-sync/-d`: Disable time synchronization

## Example
```
tree_hhcm_main --rate 100 --name my_tree --plugins libtree_hhcm.so --param foo:=42 --param bar:=true my_tree.xml
```

## License
See LICENSE file for details.
