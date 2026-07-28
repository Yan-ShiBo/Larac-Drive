# cbf_rover_gazebo

Gazebo Classic integration for the geometry in `cbf_rover_description`.
Simulator plugins, worlds, RGB-D sensing, and obstacles are isolated here so
the description package remains simulator-independent. The depth-camera plugin
publishes RGB image, depth image, camera information, and a point cloud under
the `/camera` namespace; point-cloud range is bounded in the simulator Xacro.
The base-link contact sensor remaps Gazebo bumper output to the unified
`/collision` topic.

The default launch uses `worlds/static_obstacles.world`, matching the
`static_obstacles` scenario identifier. Gazebo Classic plugin availability and
the exact `libgazebo_ros_bumper.so` message/topic behavior must be confirmed on
the target ROS 2 server because they vary across Gazebo/ROS package versions.

`obstacle_spawner` accepts obstacle strings in this format:

```text
name,x,y,size_x,size_y,size_z
```

All dimensions and positions are expressed in metres.

## Inherited visual assets

The seven simulator assets expected under
`models/cbf_rover_ground/{meshes,textures}` are copied from the inherited
working copy. They are geometry/texture resources only: this package contains
no learned checkpoints or other model weights. `setup.py` installs the mesh and
the five PNG textures after those binary assets have been restored by the main
project preparation workflow.
