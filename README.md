# ROS2 Bag to Image Extractor

`ros2bag_to_image` is a ROS2 package that extracts images from a specified topic in a `.db3` ROS2 bag file and saves them as individual image files. This package is particularly useful for processing recorded image data from robots or sensors in ROS2 environments, saving each frame as a sequential image file.

## Features
- Extracts images from a specified topic in ROS2 bag files.
- Saves images in PNG format with sequential naming.
- Uses OpenCV and `cv_bridge` to handle and convert image data.
- Ros2 Humble support. 

## Dependencies

This package requires the following ROS2 and Python packages:
- `rclpy`: ROS2 Python client library
- `rosbag2_py`: ROS2 bag reading and processing library
- `cv_bridge`: Bridges ROS image messages with OpenCV
- `opencv-python`: OpenCV library for image handling in Python

To install these dependencies, use:

```bash
sudo apt update
sudo apt install ros-humble-rclpy ros-humble-rosbag2-py ros-humble-cv-bridge python3-opencv
```

## Installation

1. Clone the Repository:
```bash
cd ~/ros2_ws/src
git clone https://github.com/AbdullahGM1/ros2bag_to_images.git
```

2. Build the Package:
```bash
cd ~/ros2_ws
colcon build --packages-select ros2bag_to_images
```

3. Source the Workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Uage

1. To extract images from a ROS2 bag file, you can specify the bag file path, topic name, and output directory as command-line arguments:
```bash
ros2 run ros2bag_to_images bag_image_extractor /path/to/your_bag_file.db3 --topic /your_topic_name --output_dir /path/to/save/images
```
* The script extracts images from the `/your_topic_name` topic in the bag file located at `/path/to/your_bag_file.db3`.
* Images are saved in `/path/to/save/images` path.
