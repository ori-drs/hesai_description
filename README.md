# hesai_description



![Hesai XT-32](./media/hesai_xt32.png)

This package contains a **URDF Xacro description of the Hesai XT32 and QT64 lidars**. In simulation the Gazebo Velodyne plug-in is used. The meshes were obtained from the original CAD provided by Hesai.

This repository is not associated to or maintained by Hesai.



For visualizing the URDF with RViz launch:

```bash
$ ros2 launch hesai_description visualize.launch.py
```

The simulation with Gazebo can be tested by launching:

```bash
$ ros2 launch hesai_description simulate.launch.py
```

For the latter you will have to install the following dependencies:

```bash
$ sudo apt-get install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-velodyne-gazebo-plugins
```

