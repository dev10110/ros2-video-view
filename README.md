# Video Viewer

This provides a simple alternative to visualize videos on the ROS2 network, since the QoS settings and the compression settings are important and seemingly cant be set in the native tools like `rqt_image_view`.


## Installation

You need access to the following libraries:
```
  image-transport
  compressed-image-transport
  theora-image-transport      [OPTIONAL]
  OpenCV
```
Note, the `image_transport` libraries must also be installed on the computer publishing the images. `OpenCV` is not requried on the publisher side.

Eg.
```
sudo apt-get update && sudo apt-get install \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-theora-image-transport
```

Clone this repo to your `colcon_ws` and just build it:
```
colcon build
```

## Usage

```
ros2 run video_view video_viewer_node  --ros-args -p image_topic:=/camera/color/image_raw
```
Don't forget to set the `image_topic` and source your workspace!

A GUI will pop up and it will show you the video stream. To exit, press `ESC` or `q`.

You can also run it as a composable node, using the component `video_view::VideoViewer`

## Parameters:

- `image_topic`: base name of the image topic to subscribe to
  - Default: `image`
  - Example `-p image_topic:=/camera/color/image_raw`
- `qos`: Quality of service
  - Choose from `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA`
  - Default: `SENSOR_DATA`
  - Example: `-p qos:=SENSOR_DATA`
- `rotate`: rotate the image
  - Choose from `rotate_0`, `rotate_90`, `rotate_180`, `rotate_270`
  - Default: `rotate_0`
  - Example: `-p rotate:=rotate_180`
- `image_transport`: method for the image transport
  - Choose from `raw`, `compressed`, `theora`
  - Default: `compressed`
  - Example: `-p image_transport:=theora`
  - Theora reduces the images the most, and is best for video streams.


## Troubleshooting
If you see the error message
```
[WARN] [1689224185.637869370] [TheoraSubscriber]: [theora] Packet was not a Theora header
```
it is because the image transport was started before this node was launched. You need to restart the source image stream.
If using a realsense camera, it is as simple as running
```
ros2 param set camera/camera enable_color False
ros2 param set camera/camera enable_color True
```
I've noticed that this needs to be run on the local device that has the realsense node running.
