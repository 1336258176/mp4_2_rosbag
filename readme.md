# MP4_2_ROSBAG

## Brief

A cpp program for converting video files to ros2 bag. (Not just MP4 files)

## Dependency
- C++17
- ROS2-Humble
- OpenCV 4.5.2

## Start

```cpp
colcon build
source install/setup.sh
ros2 run mp4_2_rosbag mp4_2_rosbag /path/to/your/file
```

## Usage

```cpp
Usage: mp4_2_rosbag <mp4_files>

brief: Convert video files to ros2 bag.(Not just MP4)

example: mp4_2_rosbag /path/to/video_file
         mp4_2_rosbag /path/to/directory
```