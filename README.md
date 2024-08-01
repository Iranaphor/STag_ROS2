# STag_ROS2
This work utilises the STag markers from [ref].
[Google Drive Marker Folder](https://drive.google.com/drive/folders/0ByNTNYCAhWbIemJDbEpXTmRncnM?resourcekey=0-OpEpUjxCopVUlSSSZ8cOoA)


# Launching
The launch file includes multiple options for arguments.
Environment variables can be set to override these rather than passing into the launch file.
Current settings for these environment variable launch arguments can be viewed with:
`ros2 launch stag2_ros system.launch.py --show-arguments`


# For running with 2 cameras:
```
ros2 launch stag_ros2 system.launch.py
    usb_cam_params_file:=$HOME/ros2_ws/src/stag_ros2/config/params_usb_cam.yaml
    namespace:=my_first_camera
```

```
ros2 launch stag_ros2 system.launch.py
    usb_cam_params_file:=$HOME/ros2_ws/src/stag_ros2/config/params_usb_cam_2.yaml
    namespace:=my_second_camera
    use_rviz:=false
```

RViz requires you to configure manually for the second camera onwards.

