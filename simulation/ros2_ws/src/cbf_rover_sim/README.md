# cbf_rover_sim

Gazebo Classic adapter for the geometry in `cbf_rover_description`. Simulator
plugins, the base world, RGB-D sensing, and obstacle projection are isolated
here so the description package remains simulator-independent. The
depth-camera plugin publishes RGB image, depth image, camera information, and a
point cloud under the `/camera` namespace; point-cloud range is bounded in the
simulator Xacro. The base-link contact sensor remaps Gazebo bumper output to the
unified `/collision` topic.

The default launch uses `worlds/base.world`, which contains the inherited
ground and sun but no scenario obstacles. Neither the world nor
`simulation.launch.py` defines a research scenario. Gazebo Classic plugin
availability and the exact `libgazebo_ros_bumper.so` message/topic behavior
must be confirmed on the target ROS 2 server because they vary across
Gazebo/ROS package versions.

`obstacle_spawner` accepts obstacle strings in this format:

```text
name,x,y,size_x,size_y,size_z
```

All dimensions and positions are expressed in metres.
The `obstacles` parameter defaults to an empty string array. The repository's
application-level scenario runner is responsible for resolving the root
`ScenarioSpec` and injecting its Gazebo projection. Do not add demonstration
obstacles or a second scenario definition to this package.

## Inherited visual assets

The seven simulator assets expected under
`models/cbf_rover_ground/{meshes,textures}` are copied from the inherited
working copy. They are geometry/texture resources only: this package contains
no learned checkpoints or other model weights. `setup.py` installs the mesh and
the five PNG textures as package data; their inherited hashes are recorded in
the provenance history.
