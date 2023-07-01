# event_array_viewer

This repository holds tools for viewing
[event_array_msgs](https://github.com/ros-event-camera/event_array_msgs). It
builds under both ROS1 and ROS2.

![event_image](images/event_viewer.png)

## Supported platforms

Currently tested on Ubuntu 20.04 (ROS Noetic and ROS2 Galactic) and
Ubuntu 22.04 (ROS2 Humble).


## How to build
Create a ROS workspace, clone this repo, and use ``vcs``
to pull in the remaining dependencies:

```
pkg=event_array_viewer
mkdir -p ~/$pkg/src
cd ~/$pkg
git clone https://github.com/ros-event-camera/${pkg}.git src/${pkg}
cd src
vcs import < ${pkg}/${pkg}.repos
cd ..
```

### configure and build on ROS1:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

### configure and build on ROS2:

```
cd ~/$pkg/src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## How to use

Examine the launch file and adjust the topic remapping, frequency
etc, then start as follows (assuming the camera driver is running
under node name ``event_camera``):

ROS1:
```
# create rendered ROS image stream from events
roslaunch event_array_viewer viewer.launch camera:=event_camera
rqt_image_view
```

ROS2:
```
# create rendered ROS image stream from events
ros2 launch event_array_viewer viewer.launch camera:=event_camera
ros2 run rqt_image_view rqt_image_view
```

Parameters:

- ``fps`` Frequency (in hz) at which images are emitted. Default: 25.
- ``display_type`` Supported types are ``time_slice`` (all events
  between frames are aggregated) or ``sharp`` (number of events is
  auto-controlled to produce sharp features). Default is
  ``time_slice``. This image shows the difference (left is sharp,
  right is time_slice): 
  <img src="images/time_slice_vs_sharp.png" width="800"/>

## License

This software is issued under the Apache License Version 2.0.
